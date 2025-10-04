// util/buttons.h - Debounced push buttons to choose modes
#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>
#include "types.h"
#include "config.h"

class Buttons {
public:
  static void begin();
  static Mode poll(); // returns current mode based on presses
private:
  static uint8_t lastStates[4];
};

#endif // BUTTONS_H
