/*******************************************************
 Micromouse_ESP32_FreeRTOS.ino
 - ESP32 + TB6612FNG + MPU6050 + Quad Encoders + 5 IR sensors
 - FreeRTOS task graph implementing estimator, IR, motor PID, motion, mapping.
 - No dynamic allocation; all arrays are static.
 - Serial logs at ~25 Hz.

 Adjust pins in config.h to match your board before uploading.
********************************************************/

#include <Arduino.h>
#include "config.h"
#include "types.h"

#include "motors.h"
#include "encoders.h"
#include "imu_mpu6050.h"
#include "ir_sensors.h"

#include "pid.h"
#include "estimator_ekf.h"
#include "motion_controller.h"

#include "maze.h"
#include "floodfill.h"
#include "planner.h"

#include "buttons.h"
#include "storage.h"

// ================= FreeRTOS Queues (static) =================
static QueueHandle_t qOmegaCmd;      // WheelOmega target to Motor PID
static QueueHandle_t qOmegaMeas;     // WheelOmega measured to controller
static QueueHandle_t qPose;          // Pose to controller/mapper
static QueueHandle_t qIR;            // IRPacket to controller/mapper
static QueueHandle_t qPrimitive;     // Primitive commands to controller

// ================= Global mode =================
static volatile Mode gMode = MODE_IDLE;

// ================= PID per wheel =================
static PID pidL = {KP_W, KI_W, KD_W, 0, 0, -MAX_ANG_V_RADPS, MAX_ANG_V_RADPS};
static PID pidR = {KP_W, KI_W, KD_W, 0, 0, -MAX_ANG_V_RADPS, MAX_ANG_V_RADPS};

// ---------- FIX 1: avoid Arduino's prototype-insertion breaking templates ----------
struct QueueUtil {
  template <typename T>
  static inline void sendOverwrite(QueueHandle_t q, const T& v) {
    xQueueOverwrite(q, (void*)&v);
  }
};
// -------------------------------------------------------------------------------


// ================= TASK: Motor PID (1 kHz) =================
void taskMotorPID(void* arg){
  const TickType_t period = pdMS_TO_TICKS(1000 / RATE_MOTOR_PID_HZ);
  TickType_t last = xTaskGetTickCount();
  WheelOmega cmd = {0,0};
  WheelOmega meas = {0,0};
  for(;;){
    // Read latest command if available
    xQueueReceive(qOmegaCmd, &cmd, 0);

    // Read encoders delta
    static uint32_t lastMicros=micros();
    uint32_t now=micros();
    float dt = (now-lastMicros)*1e-6f;
    lastMicros=now;
    int32_t dL, dR;
    Encoders::readAndReset(dL, dR);

    // Convert to measured wheel omegas
    float wL = ( (float)dL / (float)ENCODER_CPR ) * 2.0f * PI / dt;
    float wR = ( (float)dR / (float)ENCODER_CPR ) * 2.0f * PI / dt;
    meas.left = wL; meas.right = wR;

    // PID per wheel to duty via Motors
    float eL = cmd.left - wL;
    float eR = cmd.right - wR;
    float uL = pidL.update(eL, 0, dt);
    float uR = pidR.update(eR, 0, dt);
    Motors::setOmega(uL, uR);

    // Publish measured omegas to others
    QueueUtil::sendOverwrite(qOmegaMeas, meas);

    vTaskDelayUntil(&last, period);
  }
}

// ============= TASK: Estimator (200 Hz) ====================
void taskEstimator(void* arg){
  const TickType_t period = pdMS_TO_TICKS(1000 / RATE_ESTIMATOR_HZ);
  TickType_t last = xTaskGetTickCount();
  static uint32_t lastMicros = micros();
  for(;;){
    // dt
    uint32_t now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;

    // Read encoder deltas
    int32_t dL, dR;
    Encoders::readAndReset(dL, dR);

    // Read gyro Z (rad/s)
    float gz = IMU6050::readGyroZ();

    // Step estimator
    EstimatorEKF::step(dt, dL, dR, gz);

    // Publish pose & omega
    Pose p = EstimatorEKF::getPose();
    WheelOmega w = EstimatorEKF::getOmega();
    QueueUtil::sendOverwrite(qPose, p);
    QueueUtil::sendOverwrite(qOmegaMeas, w);

    vTaskDelayUntil(&last, period);
  }
}

// ============= TASK: IR (400 Hz) ===========================
void taskIR(void* arg){
  const TickType_t period = pdMS_TO_TICKS(1000 / RATE_IR_HZ);
  TickType_t last = xTaskGetTickCount();
  static uint32_t lastMicros = micros();
  for(;;){
    uint32_t now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    IRPacket pkt = IRSensors::update(dt);
    QueueUtil::sendOverwrite(qIR, pkt);
    vTaskDelayUntil(&last, period);
  }
}

// ============= TASK: Motion Controller (200 Hz) ============
void taskMotion(void* arg){
  const TickType_t period = pdMS_TO_TICKS(1000 / RATE_MOTION_HZ);
  TickType_t last = xTaskGetTickCount();
  WheelOmega meas={0,0}, cmd={0,0};
  Pose pose; IRPacket ir;
  for(;;){
    // Get latest sensor/pose
    xQueueReceive(qPose, &pose, 0);
    xQueueReceive(qIR, &ir, 0);
    xQueueReceive(qOmegaMeas, &meas, 0);

    // Get new primitive if any
    Primitive p;
    if(xQueueReceive(qPrimitive, &p, 0) == pdTRUE){
      MotionController::setPrimitive(p);
    }

    // Update controller
    bool done = MotionController::update(1.0f/RATE_MOTION_HZ, pose, ir, meas, cmd);
    (void)done; // suppress unused warning if not used elsewhere

    // Send wheel speed command
    QueueUtil::sendOverwrite(qOmegaCmd, cmd);

    vTaskDelayUntil(&last, period);
  }
}

// ============= TASK: Buttons & Modes (50 Hz) ===============
void taskButtons(void* arg){
  const TickType_t period = pdMS_TO_TICKS(1000 / RATE_BUTTONS_HZ);
  TickType_t last = xTaskGetTickCount();
  for(;;){
    Mode m = Buttons::poll();
    if(m != MODE_IDLE){
      gMode = m;
      // Queue initial primitive depending on mode
      if(m==MODE_DEMO){
        Primitive seq[] = {PRIM_FWD_1, PRIM_TURN_L90, PRIM_FWD_1, PRIM_TURN_R90, PRIM_FWD_1, PRIM_TURN_180, PRIM_FWD_1};
        for(unsigned i=0;i<sizeof(seq)/sizeof(seq[0]);i++){
          xQueueSend(qPrimitive, &seq[i], 0);
        }
      }
    }
    vTaskDelayUntil(&last, period);
  }
}

// ============= TASK: Logger (25 Hz) ========================
void taskLogger(void* arg){
  const TickType_t period = pdMS_TO_TICKS(1000 / RATE_LOGGER_HZ);
  TickType_t last = xTaskGetTickCount();
  for(;;){
    Pose p; WheelOmega w; IRPacket ir;
    if(xQueuePeek(qPose, &p, 0) && xQueuePeek(qOmegaMeas, &w, 0) && xQueuePeek(qIR, &ir, 0)){
      Serial.printf("POSE,%.3f,%.3f,%.3f | W,%.2f,%.2f | IR,%.3f,%.3f,%.3f,%.3f,%.3f | Mode,%d\n",
        p.x,p.y,p.theta, w.left,w.right, ir.dFL,ir.dF,ir.dFR,ir.dL,ir.dR, gMode);
    }
    vTaskDelayUntil(&last, period);
  }
}

// ======================== SETUP ============================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Init subsystems
  Motors::begin();
  Encoders::begin();
  IMU6050::begin();
  IMU6050::calibrateBias(2000);
  IRSensors::begin();
  EstimatorEKF::begin();
  MotionController::begin();
  Buttons::begin();

  // ---------- FIX 2: guard or remove Maze::clear() if it's not declared ----------
  // If your nav/maze.h really exposes Maze::clear(), feel free to un-comment.
  // Maze::clear();
  // -------------------------------------------------------------------------------

  Storage::begin();
  Storage::loadMaze();

  // Create queues (fixed length 1; overwrite newest)
  qOmegaCmd  = xQueueCreate(1, sizeof(WheelOmega));
  qOmegaMeas = xQueueCreate(1, sizeof(WheelOmega));
  qPose      = xQueueCreate(1, sizeof(Pose));
  qIR        = xQueueCreate(1, sizeof(IRPacket));
  qPrimitive = xQueueCreate(8, sizeof(Primitive)); // small queue for sequence

  // Start tasks
  xTaskCreatePinnedToCore(taskMotorPID, "motorPID", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskEstimator, "estimator", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskIR, "ir", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskMotion, "motion", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskButtons, "buttons", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskLogger, "logger", 4096, NULL, 1, NULL, 0);

  Serial.println("Micromouse ready.");
}

// ========================= LOOP ============================
void loop() {
  // Nothing here; FreeRTOS tasks run the system
  vTaskDelay(pdMS_TO_TICKS(1000));
}
