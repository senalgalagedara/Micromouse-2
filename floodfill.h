// nav/floodfill.h - Flood fill with dynamic weights
#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <Arduino.h>
#include "maze.h"

class FloodFill {
public:
  static void compute(uint8_t goalMask); // goalMask for 4 center cells usage ignored here
  static uint8_t chooseDir(uint8_t x,uint8_t y,uint8_t heading);
  static uint16_t weightTurn(uint8_t fromDir, uint8_t toDir, bool speedRun);
};

#endif // FLOODFILL_H
