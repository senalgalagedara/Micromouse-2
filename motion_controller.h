// control/motion_controller.h - Motion primitives and outer-loop control
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include "types.h"
#include "config.h"
#include "pid.h"

class MotionController {
public:
  static void begin();
  static void setPrimitive(Primitive p);
  static Primitive getPrimitive();
  static bool update(float dt, const Pose& pose, const IRPacket& ir, WheelOmega measured, WheelOmega& cmdOut);

  // status flags
  static bool isDone();
  static void reset();

private:
  static Primitive current;
  static float progress;      // meters or radians depending on primitive
  static float target;        // total to achieve
  static PID pidHeading;      // PD actually (KD uses gyro in dInput)
  static float thetaRef;      // desired heading for straight
};

#endif // MOTION_CONTROLLER_H
