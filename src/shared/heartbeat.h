#pragma once
#include <stdint.h>

// Shared heartbeat-state contract between a *producer* and the phase-lock layer.
// The phase-lock (motorPhaseLock) depends only on this interface, not on how the
// beat is produced:
//   - combined firmware: sensor.cpp detects the beat locally (single core)
//   - wireless fallback: espnow.cpp fills it from an ESP-NOW packet, written on
//     the WiFi task (core 0) and read in loop() (core 1)
// Declared volatile for that cross-core wireless case; harmless in the
// single-core combined build.
extern volatile uint8_t  g_bpm;             // current BPM, DEFAULT_BPM until measured
extern volatile bool     g_idle;            // true = no recent heartbeat
extern volatile bool     g_triggerPending;  // set per beat, cleared by the phase-lock
extern volatile uint32_t g_triggerMs;       // millis() captured at beat onset
