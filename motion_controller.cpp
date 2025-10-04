#include "motion_controller.h"

Primitive MotionController::current = PRIM_IDLE;
float MotionController::progress = 0.0f;
float MotionController::target = 0.0f;
PID MotionController::pidHeading = {KP_TH, 0.0f, KD_TH, 0.0f, 0.0f, -MAX_ANG_V_RADPS, MAX_ANG_V_RADPS};
float MotionController::thetaRef = 0.0f;

void MotionController::begin(){
  pidHeading.reset();
  current = PRIM_IDLE;
  progress = 0.0f;
  target = 0.0f;
  thetaRef = 0.0f;
}

void MotionController::setPrimitive(Primitive p){
  current = p;
  progress = 0.0f;
  pidHeading.reset();
  if(p==PRIM_FWD_1){ target = CELL_SIZE_M; }
  else if(p==PRIM_TURN_L90 || p==PRIM_TURN_R90){ target = PI/2.0f; }
  else if(p==PRIM_TURN_180){ target = PI; }
}

Primitive MotionController::getPrimitive(){ return current; }

bool MotionController::isDone(){
  return current == PRIM_IDLE;
}

void MotionController::reset(){
  current = PRIM_IDLE;
  progress = 0.0f;
  target = 0.0f;
}

static inline WheelOmega vOmegaToWheels(float vCmd, float uCmd){
  // uCmd is desired angular yaw rate
  float wR = ( vCmd + (AXLE_LENGTH_M*0.5f) * uCmd ) / WHEEL_RADIUS_M;
  float wL = ( vCmd - (AXLE_LENGTH_M*0.5f) * uCmd ) / WHEEL_RADIUS_M;
  WheelOmega o; o.left = wL; o.right = wR; return o;
}

bool MotionController::update(float dt, const Pose& pose, const IRPacket& ir, WheelOmega measured, WheelOmega& cmdOut){
  if(current == PRIM_IDLE){
    cmdOut.left = 0; cmdOut.right = 0; return true;
  }

  // Estimate forward progress from wheel speeds
  float v = 0.5f * WHEEL_RADIUS_M * (measured.right + measured.left);
  float w = (WHEEL_RADIUS_M/AXLE_LENGTH_M) * (measured.right - measured.left);

  if(current == PRIM_FWD_1){
    // Heading hold around thetaRef (freeze at entry)
    if(progress == 0.0f) thetaRef = pose.theta;

    float eTheta = pose.theta - thetaRef;
    // Blend lateral centering if both side walls seen
    float uYaw = pidHeading.update(eTheta, w, dt);
    if(ir.presentL && ir.presentR){
      float corridorHalf = 0.09f; // 180mm / 2
      float eLat = 0.5f*((ir.dR - corridorHalf) - (ir.dL - corridorHalf));
      uYaw += KY_LAT * eLat;
    }

    // Trapezoid velocity profile across cell
    float vCmd = MAX_LIN_V_MPS;
    // Slow down when near front wall
    if(ir.presentF && ir.dF < FRONT_STOP_M){
      vCmd = min(vCmd, 0.2f + 3.0f*(ir.dF - 0.05f)); // simple braking
      if(vCmd < 0.1f) vCmd = 0.1f;
    }

    // Integrate progress
    progress += fabsf(v) * dt;
    if(progress >= target){
      current = PRIM_IDLE;
      cmdOut.left = 0; cmdOut.right = 0;
      return true;
    }

    // Output wheel omegas
    cmdOut = vOmegaToWheels(vCmd, uYaw);
    return false;
  }

  // Rotations in place (gyro-based)
  if(current == PRIM_TURN_L90 || current == PRIM_TURN_R90 || current == PRIM_TURN_180){
    float dir = (current == PRIM_TURN_L90) ? +1.0f : (current == PRIM_TURN_R90 ? -1.0f : +1.0f);
    float angTarget = target;

    // Angular speed profile
    float uCmd = dir * MAX_ANG_V_RADPS * 0.7f;
    float vCmd = 0.0f;

    // Integrate abs angular progress using measured w
    progress += fabsf(w) * dt;
    if(progress >= angTarget){
      current = PRIM_IDLE;
      cmdOut.left = 0; cmdOut.right = 0;
      return true;
    }
    cmdOut = vOmegaToWheels(vCmd, uCmd);
    return false;
  }

  // Fallback
  cmdOut.left = 0; cmdOut.right = 0;
  return true;
}
