#include "estimator_ekf.h"

float EstimatorEKF::x[6] = {0};
float EstimatorEKF::P[6][6] = {0};

void EstimatorEKF::begin(){
  // Initialize covariance modest
  for(int i=0;i<6;i++){ for(int j=0;j<6;j++){ P[i][j]=(i==j)?0.01f:0.0f; } }
  x[0]=0; x[1]=0; x[2]=0; // pose
  x[3]=0; x[4]=0;         // wheel omegas
  x[5]=0;                 // gyro bias
}

static inline float ticksToOmega(int32_t dticks, float dt){
  // ticks -> rev -> rad; dt -> rad/s
  float rev = (float)dticks / (float)ENCODER_CPR;
  float rad = rev * 2.0f * PI;
  return rad / dt;
}

void EstimatorEKF::step(float dt, int32_t dTicksL, int32_t dTicksR, float gyroZ){
  // --- Process model ---
  // Unpack
  float wL = x[3];
  float wR = x[4];
  float bg = x[5];

  // Update wheel speeds from encoders as pseudo inputs (simple blending)
  float measWL = ticksToOmega(dTicksL, dt);
  float measWR = ticksToOmega(dTicksR, dt);
  // First-order: trust encoders but filter a little
  wL = 0.85f * wL + 0.15f * measWL;
  wR = 0.85f * wR + 0.15f * measWR;

  // Kinematics
  float v = 0.5f * WHEEL_RADIUS_M * (wR + wL);
  float w = (WHEEL_RADIUS_M / AXLE_LENGTH_M) * (wR - wL);
  // Fuse gyro to heading by simple complementary (EKF-lite)
  float yawRate = 0.7f * (gyroZ) + 0.3f * w; // gyro dominates
  x[2] += yawRate * dt;
  // Normalize angle
  if(x[2] > PI) x[2]-=2*PI;
  if(x[2] < -PI) x[2]+=2*PI;
  // Position
  x[0] += v * cosf(x[2]) * dt;
  x[1] += v * sinf(x[2]) * dt;

  // Store wheel speeds
  x[3] = wL;
  x[4] = wR;
  x[5] = bg; // bias unchanged here (kept for API symmetry)

  // Note: For brevity we run an EKF-lite (complementary fusion). On ESP32 this is sufficient
  // and numerically stable at 200 Hz. You can expand to full EKF with Jacobians if desired.
}

Pose EstimatorEKF::getPose(){
  Pose p; p.x = x[0]; p.y = x[1]; p.theta = x[2]; return p;
}

WheelOmega EstimatorEKF::getOmega(){
  WheelOmega o; o.left = x[3]; o.right = x[4]; return o;
}

float EstimatorEKF::getGyroBias(){ return x[5]; }
