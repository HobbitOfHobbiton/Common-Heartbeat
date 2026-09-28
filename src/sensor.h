#pragma once
#include "heartbeat.h"

// Local heartbeat sensor: detects beats on a digital pin and fills the shared
// heartbeat state (see heartbeat.h) that the phase-lock layer consumes. This is
// the combined-firmware producer; the wireless fallback uses espnow.cpp instead.

// Configure the heartbeat sensor pin. Call once from setup().
void sensorInit();

// Poll the sensor, detect beat onsets, and update the shared state above.
// Must be called as often as possible from loop().
void sensorUpdate();

#ifdef HEARTBEAT_DEMO
// Demo/showcase build only: live-adjust the synthetic heartbeat tempo (clamped to
// the plausible BPM band). Wired to the 'demo <bpm>' serial command.
void sensorSetDemoBpm(uint8_t bpm);
#endif
