// drivers/encoders.h - Quadrature encoder handling using interrupts, no malloc.
#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include "config.h"

class Encoders {
public:
  static volatile int32_t ticksL;  // accumulated ticks for left wheel
  static volatile int32_t ticksR;  // accumulated ticks for right wheel

  static void begin();
  static void readAndReset(int32_t &dL, int32_t &dR);

private:
  static void isrLeftA();
  static void isrLeftB();
  static void isrRightA();
  static void isrRightB();
};

#endif // ENCODERS_H
