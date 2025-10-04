// control/estimator_ekf.h - Simple EKF fusing encoder wheel speeds and gyro Z
#ifndef ESTIMATOR_EKF_H
#define ESTIMATOR_EKF_H

#include <Arduino.h>
#include "types.h"
#include "config.h"

class EstimatorEKF {
public:
  static void begin();
  static void step(float dt, int32_t dTicksL, int32_t dTicksR, float gyroZ);
  static Pose  getPose();
  static WheelOmega getOmega();
  static float getGyroBias();

private:
  // State x = [x, y, theta, wL, wR, bg]^T
  static float x[6];
  static float P[6][6];
};

#endif // ESTIMATOR_EKF_H
