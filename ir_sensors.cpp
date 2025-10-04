#include "ir_sensors.h"

// Initialize coefficients with safe defaults (will produce ~0.2m when mid)
float IRSensors::aFL=1200, IRSensors::bFL=0, IRSensors::cFL=0.02f, IRSensors::yFL=0;
float IRSensors::aF =1200, IRSensors::bF =0, IRSensors::cF =0.02f, IRSensors::yF =0;
float IRSensors::aFR=1200, IRSensors::bFR=0, IRSensors::cFR=0.02f, IRSensors::yFR=0;
float IRSensors::aL =1200, IRSensors::bL =0, IRSensors::cL =0.02f, IRSensors::yL =0;
float IRSensors::aR =1200, IRSensors::bR =0, IRSensors::cR =0.02f, IRSensors::yR =0;

void IRSensors::begin(){
  pinMode(PIN_EMIT_FL, OUTPUT);
  pinMode(PIN_EMIT_F, OUTPUT);
  pinMode(PIN_EMIT_FR, OUTPUT);
  pinMode(PIN_EMIT_L, OUTPUT);
  pinMode(PIN_EMIT_R, OUTPUT);
  digitalWrite(PIN_EMIT_FL, LOW);
  digitalWrite(PIN_EMIT_F, LOW);
  digitalWrite(PIN_EMIT_FR, LOW);
  digitalWrite(PIN_EMIT_L, LOW);
  digitalWrite(PIN_EMIT_R, LOW);

  analogReadResolution(12); // 0..4095
}

void IRSensors::setCoeffsFL(float a,float b,float c){aFL=a;bFL=b;cFL=c;}
void IRSensors::setCoeffsF (float a,float b,float c){aF=a;bF=b;cF=c;}
void IRSensors::setCoeffsFR(float a,float b,float c){aFR=a;bFR=b;cFR=c;}
void IRSensors::setCoeffsL (float a,float b,float c){aL=a;bL=b;cL=c;}
void IRSensors::setCoeffsR (float a,float b,float c){aR=a;bR=b;cR=c;}

uint16_t IRSensors::readADC(uint8_t pin){
  // small settling delay for ESP32 ADC multiplexer
  uint16_t v = analogRead(pin);
  return v;
}

static inline float iir(float yPrev, float x, float alpha){
  return yPrev + alpha * (x - yPrev);
}

float IRSensors::adcToDist(float y, float a,float b,float c){
  // Avoid division by zero and clamp
  float denom = max(1.0f, (y - b));
  float d = a / denom + c;
  // Bounds in meters
  if(d < 0.03f) d = 0.03f;
  if(d > 0.40f) d = 0.40f;
  return d;
}

IRPacket IRSensors::update(float dt){
  // Compute IIR coefficient
  float alpha = dt / IR_TAU_S;
  if(alpha > 1.0f) alpha = 1.0f;

  // Helper lambda to sample one channel synchronously
  auto sample = [&](uint8_t emitPin, uint8_t recvPin, float &yFilt)->std::pair<float, uint8_t> {
    // OFF sample (ambient)
    digitalWrite(emitPin, LOW);
    delayMicroseconds(80);
    uint16_t offv = readADC(recvPin);
    // ON sample
    digitalWrite(emitPin, HIGH);
    delayMicroseconds(80);
    uint16_t onv  = readADC(recvPin);
    digitalWrite(emitPin, LOW);
    // Subtract ambient
    int32_t s = (int32_t)onv - (int32_t)offv;
    if (s < 0) s = 0;
    // IIR filter
    yFilt = iir(yFilt, (float)s, alpha);
    // Present flag via threshold on filtered ADC
    uint8_t present = (yFilt > 40) ? 1 : 0; // tune threshold after calibration
    return {yFilt, present};
  };

  auto fl = sample(PIN_EMIT_FL, PIN_RECV_FL, yFL);
  auto f  = sample(PIN_EMIT_F,  PIN_RECV_F,  yF );
  auto fr = sample(PIN_EMIT_FR, PIN_RECV_FR, yFR);
  auto l  = sample(PIN_EMIT_L,  PIN_RECV_L,  yL );
  auto r  = sample(PIN_EMIT_R,  PIN_RECV_R,  yR );

  IRPacket pkt;
  pkt.dFL = adcToDist(fl.first, aFL,bFL,cFL);
  pkt.dF  = adcToDist(f.first,  aF,bF,cF);
  pkt.dFR = adcToDist(fr.first, aFR,bFR,cFR);
  pkt.dL  = adcToDist(l.first,  aL,bL,cL);
  pkt.dR  = adcToDist(r.first,  aR,bR,cR);
  pkt.presentFL = fl.second;
  pkt.presentF  = f.second;
  pkt.presentFR = fr.second;
  pkt.presentL  = l.second;
  pkt.presentR  = r.second;
  // rudimentary saturation flag
  pkt.satFL = (fl.first > 3500);
  pkt.satF  = (f.first  > 3500);
  pkt.satFR = (fr.first > 3500);
  pkt.satL  = (l.first  > 3500);
  pkt.satR  = (r.first  > 3500);
  return pkt;
}
