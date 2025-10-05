// config.h - Pin mappings and global constants for the Micromouse
// NOTE: Adjust pins to match your exact wiring. All arrays sized statically; no malloc.

#ifndef CONFIG_H
#define CONFIG_H
#ifndef PIN_NONE
#define PIN_NONE 255
#endif

#include <Arduino.h>

/* ===================== GPIO MAP (CHANGE TO YOUR BOARD) =====================
   The labels match your schematic net names where possible.
   Use GPIO numbers (not Dx silks). Avoid strapping pins (0,2,12,15) when possible.
*/

// --- Motor driver TB6612FNG ---
static const uint8_t PIN_PWMA = 12;       // PWM for right motor channel A
static const uint8_t PIN_AIN1 = 2;       // Direction A IN1
static const uint8_t PIN_AIN2 = 15;       // Direction A IN2
static const uint8_t PIN_PWMB = 13;       // PWM for left motor channel B
static const uint8_t PIN_BIN1 = 4;       // Direction B IN1
static const uint8_t PIN_BIN2 = 16;       // Direction B IN2
static const uint8_t PIN_STBY = -1;        // STBY pin (HIGH to enable)

// --- Quadrature Encoders (interrupt-capable pins) ---
static const uint8_t PIN_ENC_L_A = 18;    // Left encoder channel A (input only pin OK)
static const uint8_t PIN_ENC_L_B = 5;    // Left encoder channel B
static const uint8_t PIN_ENC_R_A = 19;    // Right encoder channel A
static const uint8_t PIN_ENC_R_B = 23;    // Right encoder channel B

// --- IR Emitters (you have 2 total) ---
static const uint8_t PIN_EMIT_FL = 33;   // EM_FL  -> GPIO25
static const uint8_t PIN_EMIT_FR = 32;   // EM_FR  -> GPIO33

// Map side emitters to the two you have (reuse)
static const uint8_t PIN_EMIT_L  = PIN_EMIT_FL;   // reuse left emitter
static const uint8_t PIN_EMIT_R  = PIN_EMIT_FR;   // reuse right emitter
static const uint8_t PIN_EMIT_F  = PIN_NONE;      // no dedicated front emitter

// --- IR Receivers (you have 4 total; all on ADC2) ---
// NOTE: ADC2 can't be used when WiFi is active on ESP32. Keep WiFi off.
static const uint8_t PIN_RECV_R  = 25;   // RE_R  -> GPIO26 (ADC2)
static const uint8_t PIN_RECV_FR = 26;   // RE_FR -> GPIO27 (ADC2)
static const uint8_t PIN_RECV_FL = 27;   // RE_FL -> GPIO14 (ADC2)
static const uint8_t PIN_RECV_L  = 14;   // RE_L  -> GPIO12 (ADC2)
static const uint8_t PIN_RECV_F  = PIN_NONE; // no center-front receiver

// --- Push buttons (pull-up; pressed = LOW) ---
static const uint8_t PIN_BTN_SEARCH  = 35;  // Mode: Search run
static const uint8_t PIN_BTN_SPEED   = 34;  // Mode: Speed run
static const uint8_t PIN_BTN_CALIB   = 39;  // Mode: Calibration/health
static const uint8_t PIN_BTN_DEMO    = 17;   // Mode: Diagnostics motion demo

// --- I2C (MPU6050) ---
static const uint8_t PIN_I2C_SDA = 21;
static const uint8_t PIN_I2C_SCL = 22;

// ===================== ROBOT GEOMETRY =====================
static const float WHEEL_DIAMETER_M = 0.032f;         // 32mm
static const float WHEEL_RADIUS_M   = WHEEL_DIAMETER_M * 0.5f;
static const float AXLE_LENGTH_M    = 0.112f;         // 112mm
static const float CELL_SIZE_M      = 0.18f;          // 180mm

// ===================== ENCODERS ============================
// Set counts-per-rev AFTER quadrature (e.g., 12 CPR * 4x = 48).
static const int32_t ENCODER_CPR = 48;

// ===================== CONTROL RATES =======================
static const uint32_t RATE_MOTOR_PID_HZ   = 1000;   // inner loop
static const uint32_t RATE_ESTIMATOR_HZ   = 200;    // EKF & odometry
static const uint32_t RATE_IR_HZ          = 400;    // synchronous IR
static const uint32_t RATE_MOTION_HZ      = 200;    // outer loop
static const uint32_t RATE_PLANNER_HZ     = 50;     // mapping/planning
static const uint32_t RATE_BUTTONS_HZ     = 50;     // debounced inputs
static const uint32_t RATE_LOGGER_HZ      = 25;     // serial telemetry

// ===================== LIMITS & GAINS ======================
static const float MAX_LIN_V_MPS      = 1.2f;       // conservative
static const float MAX_ANG_V_RADPS    = 8.0f;
static const float MAX_ACCEL_MPS2     = 1.5f;

// Velocity PID (per wheel) start values
static const float KP_W = 0.4f;
static const float KI_W = 60.0f;
static const float KD_W = 0.0f;

// Heading PD (gyro-anchored)
static const float KP_TH = 3.5f;
static const float KD_TH = 0.05f;

// Lateral centering blend
static const float KY_LAT = 0.010f;

// IR IIR time-constant (seconds)
static const float IR_TAU_S = 0.015f;

// Front-stop thresholds (meters)
static const float FRONT_STOP_M = 0.11f;

// ===================== MAZE SIZE ===========================
#define MAZE_W 16
#define MAZE_H 16

// ===================== PWM/LEDC ============================
static const int LEDC_FREQ_HZ = 20000;  // 20kHz
static const int LEDC_TIMER_BITS = 12;  // 0..4095

// ===================== EEPROM ==============================
static const size_t EEPROM_BYTES = 4096;   // adjust if needed

#endif // CONFIG_H