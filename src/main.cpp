#include <Arduino.h>
#include "motor.h"
#include "sensor.h"
#include "striker.h"
#include "log.h"
#include "display.h"

// Combined firmware: one ESP32 reads the heartbeat sensor locally and drives the
// drum-strike motor — no ESP-NOW, no sender/receiver split. (The wireless,
// two-board variant is kept as a fallback under src/Wireless/.)
//
// The motor drives a string-wound gravity/elastic striker (see striker.cpp); the
// old continuous-rotation cam approach (motorPhaseLock.*) is superseded.

void setup() {
  Serial.begin(115200);

  // Optional debug OLED: auto-detected, mirrors the serial log on units that
  // have one. Bring it up before anything logs so boot messages show up there.
  if (displayInit()) logSetSink(displayLine);
  logPrintln("Hardbeatz ready");

  sensorInit();
  motorInit();
  strikerInit();
}

void loop() {
  sensorUpdate();      // detect beats, update shared heartbeat state
  strikerUpdate();     // cock/arm/release the striker, timed to the predicted beat
}
