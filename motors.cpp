#include "motors.h"

uint16_t Motors::lastDutyL = 0;
uint16_t Motors::lastDutyR = 0;

void Motors::begin() {
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  // LEDC setup for ESP32 Arduino Core 3.x (new API)
  // Channel 0 for right motor (PWMA)
  ledcAttach(PIN_PWMA, LEDC_FREQ_HZ, LEDC_TIMER_BITS);
  
  // Channel 1 for left motor (PWMB)
  ledcAttach(PIN_PWMB, LEDC_FREQ_HZ, LEDC_TIMER_BITS);
  
  // Set initial duty to 0
  ledcWrite(PIN_PWMA, 0);
  ledcWrite(PIN_PWMB, 0);
}

uint16_t Motors::omegaToDuty(float omega) {
  // Map absolute omega to duty using linear map to max command
  float mag = fabsf(omega);
  float dutyf = constrain(mag / MAX_ANG_V_RADPS, 0.0f, 1.0f);
  uint16_t duty = (uint16_t)(dutyf * ((1<<LEDC_TIMER_BITS)-1));
  return duty;
}

void Motors::setChannel(int pwmPin, int in1, int in2, float omega, bool isRight) {
  bool forward = omega >= 0.0f;
  uint16_t duty = omegaToDuty(omega);
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW  : HIGH);
  ledcWrite(pwmPin, duty);
  if(isRight) lastDutyR = duty; 
  else lastDutyL = duty;
}

void Motors::setOmega(float omegaLeft, float omegaRight) {
  setChannel(PIN_PWMA, PIN_AIN1, PIN_AIN2, omegaRight, true);  // Right motor
  setChannel(PIN_PWMB, PIN_BIN1, PIN_BIN2, omegaLeft, false);  // Left motor
}

void Motors::brake() {
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, HIGH);
  ledcWrite(PIN_PWMA, 0);
  ledcWrite(PIN_PWMB, 0);
}