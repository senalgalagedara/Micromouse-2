// types.h - Common structs and enums

#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// Motion primitive enumeration
enum Primitive : uint8_t {
  PRIM_IDLE = 0,
  PRIM_FWD_1,
  PRIM_TURN_L90,
  PRIM_TURN_R90,
  PRIM_TURN_180
};

// Modes selected by buttons
enum Mode : uint8_t {
  MODE_IDLE = 0,
  MODE_SEARCH,
  MODE_SPEED,
  MODE_CALIB,
  MODE_DEMO
};

// Pose structure
struct Pose {
  float x;     // meters
  float y;     // meters
  float theta; // radians
};

// Wheel speeds (rad/s)
struct WheelOmega {
  float left;
  float right;
};

// IR distances (meters) and flags
struct IRPacket {
  float dFL, dF, dFR, dL, dR; // distances (meters), 0 if invalid
  uint8_t presentFL, presentF, presentFR, presentL, presentR; // booleans
  uint8_t satFL, satF, satFR, satL, satR; // saturation flags
};

// Cell coordinate
struct Cell {
  uint8_t x;
  uint8_t y;
};

#endif // TYPES_H
