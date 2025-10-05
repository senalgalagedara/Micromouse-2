#include "ir_sensors.h"
#ifndef PIN_NONE
#define PIN_NONE 255
#endif

// Initialize coefficients with safe defaults (will produce ~0.2 m when mid)
float IRSensors::aFL = 1200, IRSensors::bFL = 0, IRSensors::cFL = 0.02f, IRSensors::yFL = 0;
float IRSensors::aF  = 1200, IRSensors::bF  = 0, IRSensors::cF  = 0.02f, IRSensors::yF  = 0;
float IRSensors::aFR = 1200, IRSensors::bFR = 0, IRSensors::cFR = 0.02f, IRSensors::yFR = 0;
float IRSensors::aL  = 1200, IRSensors::bL  = 0, IRSensors::cL  = 0.02f, IRSensors::yL  = 0;
float IRSensors::aR  = 1200, IRSensors::bR  = 0, IRSensors::cR  = 0.02f, IRSensors::yR  = 0;

void IRSensors::begin() {
  // configure only emitters that exist
  if (PIN_EMIT_FL != PIN_NONE) pinMode(PIN_EMIT_FL, OUTPUT);
  if (PIN_EMIT_F  != PIN_NONE) pinMode(PIN_EMIT_F,  OUTPUT);
  if (PIN_EMIT_FR != PIN_NONE) pinMode(PIN_EMIT_FR, OUTPUT);
  if (PIN_EMIT_L  != PIN_NONE) pinMode(PIN_EMIT_L,  OUTPUT);
  if (PIN_EMIT_R  != PIN_NONE) pinMode(PIN_EMIT_R,  OUTPUT);

  digitalWrite(PIN_EMIT_FL, LOW);
  digitalWrite(PIN_EMIT_F,  LOW);
  digitalWrite(PIN_EMIT_FR, LOW);
  digitalWrite(PIN_EMIT_L,  LOW);
  digitalWrite(PIN_EMIT_R,  LOW);

  analogReadResolution(12);  // 0..4095
}

// -------- coefficient setters --------
void IRSensors::setCoeffsFL(float a, float b, float c) { aFL = a; bFL = b; cFL = c; }
void IRSensors::setCoeffsF (float a, float b, float c) { aF  = a; bF  = b; cF  = c; }
void IRSensors::setCoeffsFR(float a, float b, float c) { aFR = a; bFR = b; cFR = c; }
void IRSensors::setCoeffsL (float a, float b, float c) { aL  = a; bL  = b; cL  = c; }
void IRSensors::setCoeffsR (float a, float b, float c) { aR  = a; bR  = b; cR  = c; }

uint16_t IRSensors::readADC(uint8_t pin) {
  if (pin == PIN_NONE) return 0;
  // small settling delay for ESP32 ADC multiplexer
  return analogRead(pin);
}

static inline float iir(float yPrev, float x, float alpha) {
  return yPrev + alpha * (x - yPrev);
}

float IRSensors::adcToDist(float y, float a, float b, float c) {
  float denom = max(1.0f, (y - b));
  float d = a / denom + c;
  if (d < 0.03f) d = 0.03f;
  if (d > 0.40f) d = 0.40f;
  return d;
}

IRPacket IRSensors::update(float dt) {
  float alpha = dt / IR_TAU_S;
  if (alpha > 1.0f) alpha = 1.0f;

  // --- helper for one channel ---
  auto sample = [&](uint8_t emitPin, uint8_t recvPin, float &yFilt)
      -> std::pair<float, uint8_t> {
    // skip if this channel doesn't exist
    if (emitPin == PIN_NONE || recvPin == PIN_NONE) {
      yFilt *= 0.95f;          // gentle decay
      return {yFilt, (uint8_t)0};
    }

    // ambient off sample
    digitalWrite(emitPin, LOW);
    delayMicroseconds(80);
    uint16_t offv = readADC(recvPin);

    // active on sample
    digitalWrite(emitPin, HIGH);
    delayMicroseconds(80);
    uint16_t onv = readADC(recvPin);
    digitalWrite(emitPin, LOW);

    // ambient subtraction
    int32_t s = (int32_t)onv - (int32_t)offv;
    if (s < 0) s = 0;

    yFilt = iir(yFilt, (float)s, alpha);
    uint8_t present = (yFilt > 40) ? 1 : 0;
    return {yFilt, present};
  };

  // ---- your 4-sensor geometry ----
  auto fl = sample(PIN_EMIT_FL, PIN_RECV_FL, yFL);  // front-left
  auto f  = sample(PIN_NONE,    PIN_NONE,    yF );  // none
  auto fr = sample(PIN_EMIT_FR, PIN_RECV_FR, yFR);  // front-right
  auto l  = sample(PIN_EMIT_FL, PIN_RECV_L,  yL );  // left  (reuse FL emitter)
  auto r  = sample(PIN_EMIT_FR, PIN_RECV_R,  yR );  // right (reuse FR emitter)

  IRPacket pkt;
  pkt.dFL = adcToDist(fl.first, aFL, bFL, cFL);
  pkt.dF  = adcToDist(f.first,  aF,  bF,  cF);
  pkt.dFR = adcToDist(fr.first, aFR, bFR, cFR);
  pkt.dL  = adcToDist(l.first,  aL,  bL,  cL);
  pkt.dR  = adcToDist(r.first,  aR,  bR,  cR);

  pkt.presentFL = fl.second;
  pkt.presentF  = f.second;
  pkt.presentFR = fr.second;
  pkt.presentL  = l.second;
  pkt.presentR  = r.second;

  pkt.satFL = (fl.first > 3500);
  pkt.satF  = (f.first  > 3500);
  pkt.satFR = (fr.first > 3500);
  pkt.satL  = (l.first  > 3500);
  pkt.satR  = (r.first  > 3500);

  return pkt;
}
