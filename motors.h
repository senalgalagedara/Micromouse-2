// drivers/motors.h - TB6612FNG motor driver control via LEDC PWM
#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"

class Motors {
public:
  // Initialize GPIOs and LEDC
  static void begin();

  // Set wheel angular velocities command (rad/s): + forward
  static void setOmega(float omegaLeft, float omegaRight);

  // Emergency stop (brake)
  static void brake();

  // Utility: convert desired wheel angular velocity to duty (0..4095)
  static uint16_t omegaToDuty(float omega);

  // Read back last set duty (for debugging)
  static uint16_t lastDutyL, lastDutyR;

private:
  static void setChannel(int pwmPin, int in1, int in2, float omega, uint8_t ledcChannel);
};

#endif // MOTORS_H
