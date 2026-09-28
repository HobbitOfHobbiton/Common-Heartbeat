#pragma once

// Frequency- and phase-locks the cam wheel to the incoming heartbeat (a software
// PLL). Reads the shared ESP-NOW state from espnow.h and drives the stepper.

// Set up the optional index/home sensor pin. Call once from setup().
void motorPhaseLockInit();

// Run one iteration: re-pin phase on the index pulse, fold in any new beat, and
// emit steps. Must be called as often as possible from loop().
void motorPhaseLockUpdate();
