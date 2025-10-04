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

  // LEDC setup: two channels for PWMA/PWMB
  ledcSetup(0, LEDC_FREQ_HZ, LEDC_TIMER_BITS);
  ledcSetup(1, LEDC_FREQ_HZ, LEDC_TIMER_BITS);
  ledcAttachPin(PIN_PWMA, 0);
  ledcAttachPin(PIN_PWMB, 1);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

uint16_t Motors::omegaToDuty(float omega) {
  // Map absolute omega to duty using linear map to max command
  float mag = fabsf(omega);
  float dutyf = constrain(mag / MAX_ANG_V_RADPS, 0.0f, 1.0f);
  uint16_t duty = (uint16_t)(dutyf * ((1<<LEDC_TIMER_BITS)-1));
  return duty;
}

void Motors::setChannel(int pwmPin, int in1, int in2, float omega, uint8_t ledcChannel) {
  bool forward = omega >= 0.0f;
  uint16_t duty = omegaToDuty(omega);
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW  : HIGH);
  ledcWrite(ledcChannel, duty);
  if(ledcChannel==0) lastDutyR = duty; else lastDutyL = duty; // A=right, B=left by convention
}

void Motors::setOmega(float omegaLeft, float omegaRight) {
  setChannel(PIN_PWMA, PIN_AIN1, PIN_AIN2, omegaRight, 0); // Channel A -> Right
  setChannel(PIN_PWMB, PIN_BIN1, PIN_BIN2, omegaLeft, 1);  // Channel B -> Left
}

void Motors::brake() {
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, HIGH);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}
