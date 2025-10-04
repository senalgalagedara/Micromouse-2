// drivers/ir_sensors.h - Synchronous IR sampling with ambient rejection + IIR LPF
#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <Arduino.h>
#include "types.h"
#include "config.h"

class IRSensors {
public:
  // Initialize pins and coefficients
  static void begin();

  // Perform one synchronous update (turn LEDs on/off, read ADC, filter)
  static IRPacket update(float dt);

  // Calibration coefficients (per channel): d = a/(y - b) + c
  static void setCoeffsFL(float a,float b,float c);
  static void setCoeffsF (float a,float b,float c);
  static void setCoeffsFR(float a,float b,float c);
  static void setCoeffsL (float a,float b,float c);
  static void setCoeffsR (float a,float b,float c);

private:
  static float aFL,bFL,cFL, yFL;
  static float aF ,bF ,cF , yF ;
  static float aFR,bFR,cFR, yFR;
  static float aL ,bL ,cL , yL ;
  static float aR ,bR ,cR , yR ;

  static float adcToDist(float y, float a,float b,float c);
  static uint16_t readADC(uint8_t pin);
};

#endif // IR_SENSORS_H
