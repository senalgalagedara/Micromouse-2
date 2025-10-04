#include "imu_mpu6050.h"

// MPU6050 registers
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_GYRO_ZOUT_H = 0x47;

float IMU6050::biasZ = 0.0f;

void IMU6050::begin(){
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  delay(50);
  // Wake up device
  writeReg(REG_PWR_MGMT_1, 0x01); // clock source X gyro
  // DLPF: set to 42 Hz (CONFIG=3) to reduce noise, you can change
  writeReg(REG_CONFIG, 0x03);
  // Gyro full-scale ±1000 dps (GYRO_CONFIG=0x10) for better LSB/°/s
  writeReg(REG_GYRO_CONFIG, 0x10);
}

void IMU6050::calibrateBias(uint16_t samples){
  float sum = 0.0f;
  for(uint16_t i=0;i<samples;i++){
    int16_t gz = readReg16(REG_GYRO_ZOUT_H);
    // Sensitivity for ±1000 dps is 32.8 LSB/°/s → convert to rad/s
    float radps = ( (float)gz / 32.8f ) * (PI/180.0f);
    sum += radps;
    delay(1);
  }
  biasZ = sum / samples;
}

float IMU6050::readGyroZ(){
  int16_t gz = readReg16(REG_GYRO_ZOUT_H);
  float radps = ( (float)gz / 32.8f ) * (PI/180.0f);
  return radps - biasZ;
}

float IMU6050::getBiasZ(){ return biasZ; }

int16_t IMU6050::readReg16(uint8_t addr){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(addr);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi<<8) | lo;
}

void IMU6050::writeReg(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
