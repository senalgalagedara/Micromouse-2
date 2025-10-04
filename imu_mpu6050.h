// drivers/imu_mpu6050.h - Minimal MPU6050 gyro-Z reader with bias calibration.
#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

class IMU6050 {
public:
  static void begin();
  static void calibrateBias(uint16_t samples = 2000);
  static float readGyroZ();        // rad/s, bias-subtracted
  static float getBiasZ();         // rad/s
private:
  static float biasZ;
  static int16_t readReg16(uint8_t addr);
  static void writeReg(uint8_t reg, uint8_t val);
};

#endif // IMU_MPU6050_H
