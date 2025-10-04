// nav/maze.h - Compact maze representation with bit-packed walls, no malloc.
#ifndef MAZE_H
#define MAZE_H

#include <Arduino.h>
#include "config.h"

// Bits per cell: N=1, E=2, S=4, W=8, visited=16
struct MazeCell {
  uint8_t bits;
  uint16_t dist;
};

class Maze {
public:
  static void clear();
  static void setWall(uint8_t x,uint8_t y,uint8_t dir, bool present);
  static bool hasWall(uint8_t x,uint8_t y,uint8_t dir);
  static void setVisited(uint8_t x,uint8_t y);
  static bool isVisited(uint8_t x,uint8_t y);
  static uint16_t getDist(uint8_t x,uint8_t y);
  static void setDist(uint8_t x,uint8_t y,uint16_t d);
  static MazeCell cells[MAZE_W][MAZE_H];
};

// Direction indices
enum {NORTH=0, EAST=1, SOUTH=2, WEST=3};

#endif // MAZE_H
