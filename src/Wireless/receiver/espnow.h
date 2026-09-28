#pragma once
#include "heartbeat.h"

// Wireless heartbeat producer (fallback): fills the shared heartbeat state (see
// heartbeat.h) from incoming ESP-NOW packets, written on the WiFi task (core 0)
// and read in loop() (core 1) by the phase-lock layer.

// Bring up WiFi (STA) and ESP-NOW, and register the receive callback.
void espnowInit();
