// nav/planner.h - Simple planner that turns flood-fill direction into a primitive
#ifndef PLANNER_H
#define PLANNER_H

#include <Arduino.h>
#include "types.h"

class Planner {
public:
  static Primitive decide(uint8_t heading, uint8_t desiredDir);
};

#endif // PLANNER_H
