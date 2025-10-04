// control/pid.h - Lightweight PID controller with anti-windup, no malloc.
#ifndef PID_H
#define PID_H

#include <Arduino.h>

struct PID {
  float kp;
  float ki;
  float kd;
  float iTerm;
  float prev;
  float outMin;
  float outMax;
  inline void reset(){ iTerm=0; prev=0; }
  inline float update(float err, float dInput, float dt){
    iTerm += ki * err * dt;
    if(iTerm > outMax) iTerm = outMax;
    if(iTerm < outMin) iTerm = outMin;
    float out = kp * err + iTerm - kd * dInput;
    if(out > outMax) out = outMax;
    if(out < outMin) out = outMin;
    prev = err;
    return out;
  }
};

#endif // PID_H
