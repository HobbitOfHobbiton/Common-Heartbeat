#pragma once
#include <AccelStepper.h>
#include <TMCStepper.h>

// --- Mechanism config (shared with the phase-lock layer) ---
// A two-tooth cam wheel: one full wheel revolution = one heartbeat.
static const int     STEPS_PER_REV = 200;     // NEMA17 full steps/rev (1.8°)
static const int     MICROSTEPS    = 16;       // configured via UART; no MS1/MS2 wiring
static const float   REVS_PER_BEAT = 1.0f;     // motor revs per beat; raise if a gearbox is added
static const int     SPIN_DIR      = 1;        // flip to -1 if the wheel turns the wrong way
static const uint8_t MAX_BPM       = 220;      // matches sender clamp
static const uint8_t MIN_BPM       = 30;       // bounds the measured beat period

static const long STEPS_PER_BEAT = (long)(STEPS_PER_REV * MICROSTEPS * REVS_PER_BEAT);

// Defined in motor.cpp. The TMC2209 is configured over UART once at boot; motion
// is driven over STEP/DIR by AccelStepper for minimal per-step latency.
extern AccelStepper   stepper;
extern TMC2209Stepper driver;

// Configure the TMC2209 (current, microstepping, SpreadCycle) and the stepper.
// Leaves the driver disabled (no holding current) — the system boots idle.
void motorInit();

// Enable (EN LOW) / disable (EN HIGH) the driver output stage. Disabling cuts
// holding current, silencing the stopped-motor chopper whine.
void motorEnable();
void motorDisable();

// Read DRV_STATUS once and log the live drive health: cs_actual (is current
// actually being delivered?), open-load A/B (miswired/disconnected coil),
// short-to-ground, and over-temperature. One UART read; call throttled — it
// briefly interrupts step emission, so don't call it every loop.
void motorLogLive();

// Switch microstep resolution at runtime over UART (one register write; briefly
// pauses stepping). Cocking uses the fine MICROSTEPS resolution for smooth, strong
// torque; the release drops to coarse/full steps so the loop's fixed pulse-rate
// ceiling yields far higher rev/s — the string must pay out faster than the mallet
// falls to stay slack. AccelStepper still counts pulses, so the caller must rescale
// positions/speeds by the resolution ratio when it switches (see striker.cpp).
void motorSetMicrosteps(uint16_t ms);
