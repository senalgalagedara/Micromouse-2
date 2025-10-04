#include "buttons.h"

uint8_t Buttons::lastStates[4] = {1,1,1,1};

void Buttons::begin(){
  pinMode(PIN_BTN_SEARCH, INPUT_PULLUP);
  pinMode(PIN_BTN_SPEED,  INPUT_PULLUP);
  pinMode(PIN_BTN_CALIB,  INPUT_PULLUP);
  pinMode(PIN_BTN_DEMO,   INPUT_PULLUP);
}

Mode Buttons::poll(){
  uint8_t sA = digitalRead(PIN_BTN_SEARCH);
  uint8_t sB = digitalRead(PIN_BTN_SPEED);
  uint8_t sC = digitalRead(PIN_BTN_CALIB);
  uint8_t sD = digitalRead(PIN_BTN_DEMO);
  // simple edge detect
  if(sA==LOW && lastStates[0]==HIGH){ lastStates[0]=sA; return MODE_SEARCH; }
  if(sB==LOW && lastStates[1]==HIGH){ lastStates[1]=sB; return MODE_SPEED; }
  if(sC==LOW && lastStates[2]==HIGH){ lastStates[2]=sC; return MODE_CALIB; }
  if(sD==LOW && lastStates[3]==HIGH){ lastStates[3]=sD; return MODE_DEMO; }
  lastStates[0]=sA; lastStates[1]=sB; lastStates[2]=sC; lastStates[3]=sD;
  return MODE_IDLE;
}
