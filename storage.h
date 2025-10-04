// util/storage.h - EEPROM-backed save/load for maze and parameters
#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>
#include <EEPROM.h>
#include "maze.h"

class Storage {
public:
  static void begin();
  static void saveMaze();
  static void loadMaze();
};

#endif // STORAGE_H
