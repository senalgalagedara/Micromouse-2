#include "encoders.h"

volatile int32_t Encoders::ticksL = 0;
volatile int32_t Encoders::ticksR = 0;

static volatile uint8_t stateL = 0;
static volatile uint8_t stateR = 0;

void Encoders::begin() {
  pinMode(PIN_ENC_L_A, INPUT);
  pinMode(PIN_ENC_L_B, INPUT);
  pinMode(PIN_ENC_R_A, INPUT);
  pinMode(PIN_ENC_R_B, INPUT);
  // Initialize states
  stateL = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);
  stateR = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrRightB, CHANGE);
}

inline void stepQuad(volatile uint8_t &state, int pinA, int pinB, volatile int32_t &ticks, bool invert) {
  uint8_t s = (digitalRead(pinA) << 1) | digitalRead(pinB);
  // Gray-code transitions: 00->01->11->10->00 is forward
  int8_t delta = 0;
  if ((state == 0b00 && s == 0b01) || (state == 0b01 && s == 0b11) ||
      (state == 0b11 && s == 0b10) || (state == 0b10 && s == 0b00)) delta = 1;
  else if ((state == 0b00 && s == 0b10) || (state == 0b10 && s == 0b11) ||
           (state == 0b11 && s == 0b01) || (state == 0b01 && s == 0b00)) delta = -1;
  state = s;
  if (invert) delta = -delta;
  ticks += delta;
}

void Encoders::isrLeftA(){ stepQuad(stateL, PIN_ENC_L_A, PIN_ENC_L_B, ticksL, false); }
void Encoders::isrLeftB(){ stepQuad(stateL, PIN_ENC_L_A, PIN_ENC_L_B, ticksL, false); }
void Encoders::isrRightA(){ stepQuad(stateR, PIN_ENC_R_A, PIN_ENC_R_B, ticksR, false); }
void Encoders::isrRightB(){ stepQuad(stateR, PIN_ENC_R_A, PIN_ENC_R_B, ticksR, false); }

void Encoders::readAndReset(int32_t &dL, int32_t &dR) {
  noInterrupts();
  dL = ticksL; ticksL = 0;
  dR = ticksR; ticksR = 0;
  interrupts();
}
