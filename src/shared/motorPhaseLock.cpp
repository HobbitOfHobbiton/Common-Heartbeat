#include <Arduino.h>
#include "motor.h"
#include "heartbeat.h"
#include "motorPhaseLock.h"

// --- Phase alignment (PLL) ---
// Phase within a revolution (0..STEPS_PER_BEAT) that the tooth should occupy at
// the instant of the beat. Tune so the strike lands on the beat. NOTE: without a
// home sensor (USE_INDEX below) this is only boot-stable if the wheel always
// starts from the same angle — see the homing note in the README/comments.
static const long  PHASE_OFFSET_STEPS = 0;
static const float PHASE_GAIN = 0.5f;        // fraction of phase error corrected per beat (0..1)

static const uint32_t MIN_PERIOD_MS = 60000UL / MAX_BPM;   // fastest plausible beat
static const uint32_t MAX_PERIOD_MS = 60000UL / MIN_BPM;   // slowest plausible beat

// --- Warm-up ---
// After the sensor goes from idle to active, hold the motor off for this long
// while the sensor gathers a few beats. This starts the wheel on a settled BPM
// and rejects brief activity from users swapping/adjusting their sensor.
static const uint32_t WARMUP_MS = 3000;

// If beats stop for longer than this mid-warm-up, abort immediately rather than
// waiting out the sensor's full idle timeout — a gap beyond the slowest plausible
// beat means the heartbeat is gone (sensor pulled/swapped), so the motor must not
// start on a dead sensor. Re-arms on the next real beat.
static const uint32_t WARMUP_STALL_MS = MAX_PERIOD_MS;

// --- Optional index/home sensor ---
// Strongly recommended for true absolute alignment: a hall/optical sensor that
// pulses once per revolution at a known wheel angle. Each pulse re-pins the phase,
// cancelling accumulated step error and making PHASE_OFFSET boot-stable.
static const bool USE_INDEX = false;
static const int  HOME_PIN  = 27;            // 25 is now the driver EN pin
static const long HOME_PHASE_STEPS = 0;      // wheel phase when the index sensor triggers

// --- Velocity slew limit (software acceleration) ---
// runSpeed() emits steps at the commanded rate with NO ramp, and ignores
// setMaxSpeed/setAcceleration. A cam-loaded stepper can't jump from standstill to
// full BPM (cold start) or absorb a phase-correction speed jump without losing
// torque and stalling. So we rate-limit the speed actually sent to the driver,
// letting it chase the PLL target instead of snapping to it. This is the primary
// stall knob: lower it if the beater still jams, raise it for snappier phase lock.
static const float MAX_ACCEL = 8000.0f;      // steps/s^2 at MICROSTEPS=16

// Phase-lock state (owned here, touched only on core 1 / loop()).
static uint32_t lastTriggerMs = 0;
static bool     locked = false;              // have we seen a previous trigger to measure against?
static float    baseSpeed = 0.0f;            // PLL target step rate (signed)
static float    cmdSpeed  = 0.0f;            // rate-limited rate actually sent to the driver
static uint32_t lastSlewUs = 0;
static bool     lastHome = false;
static bool     motorEnabled = false;        // mirrors the driver EN state
static bool     warmingUp = false;           // true between idle->active and motor start
static uint32_t warmupStartMs = 0;

// BPM -> step rate (steps/sec): one rev per beat means BPM revs per minute.
static float bpmToStepsPerSec(uint8_t bpm) {
  return (bpm / 60.0f) * STEPS_PER_BEAT * SPIN_DIR;
}

void motorPhaseLockInit() {
  if (USE_INDEX) pinMode(HOME_PIN, INPUT);
}

void motorPhaseLockUpdate() {
  // Idle (sensor off): disable the driver (silent, no holding current) and drop
  // the lock so we re-acquire cleanly on resume.
  if (g_idle) {
    if (motorEnabled) { motorDisable(); motorEnabled = false; }
    locked = false;
    warmingUp = false;
    cmdSpeed = 0.0f;             // ramp from standstill again on the next start
    return;
  }

  // Motor off: run the warm-up gate before letting the wheel spin.
  if (!motorEnabled) {
    // Begin warm-up on the first real beat. Stay disabled while the sensor
    // settles, so we start on a stable BPM and ride out brief sensor swaps.
    if (!warmingUp) {
      if (!g_triggerPending) return;   // active but no beat yet -> keep waiting
      warmingUp = true;
      warmupStartMs = millis();
    }

    // Bail the instant beats stop mid-warm-up (sensor pulled or swapped), without
    // waiting out the sensor's idle timeout. Re-arms on the next beat above.
    if (millis() - g_triggerMs > WARMUP_STALL_MS) {
      warmingUp = false;
      g_triggerPending = false;
      return;
    }

    // Still warming up: keep the motor off; the sensor keeps measuring so g_bpm
    // is settled by the time we start.
    if (millis() - warmupStartMs < WARMUP_MS) return;

    // Warm-up complete: start the motor at the now-settled BPM.
    warmingUp = false;
    motorEnable();
    motorEnabled = true;
    locked = false;            // anchor on the next beat / nominal BPM
    cmdSpeed = 0.0f;           // ramp up from standstill, don't slam to full BPM
    lastSlewUs = micros();     // seed dt so the first slew step is small
  }

  // Index sensor: re-pin the wheel phase to a known angle once per revolution.
  if (USE_INDEX) {
    bool h = digitalRead(HOME_PIN);
    if (h && !lastHome) stepper.setCurrentPosition(SPIN_DIR * HOME_PHASE_STEPS);
    lastHome = h;
  }

  // On each beat, lock frequency (from trigger spacing) and correct phase.
  if (g_triggerPending) {
    g_triggerPending = false;
    uint32_t t = g_triggerMs;

    if (!locked) {
      // First beat after (re)start: anchor timing, spin at the nominal BPM rate.
      lastTriggerMs = t;
      locked = true;
      baseSpeed = bpmToStepsPerSec(g_bpm);
    } else {
      uint32_t periodMs = constrain(t - lastTriggerMs, MIN_PERIOD_MS, MAX_PERIOD_MS);
      lastTriggerMs = t;
      float periodS = periodMs / 1000.0f;

      // Wheel phase in the travel direction, 0..STEPS_PER_BEAT-1.
      long travel = SPIN_DIR * stepper.currentPosition();
      long phase  = ((travel % STEPS_PER_BEAT) + STEPS_PER_BEAT) % STEPS_PER_BEAT;

      // Shortest signed error from where the tooth should be at the beat.
      long err = ((phase - PHASE_OFFSET_STEPS) % STEPS_PER_BEAT + STEPS_PER_BEAT) % STEPS_PER_BEAT;
      if (err > STEPS_PER_BEAT / 2) err -= STEPS_PER_BEAT;

      // Travel one full rev per beat, minus a fraction of the phase error:
      // err > 0 (ahead) -> shorter distance, slow down; err < 0 (behind) -> speed up.
      float distance = (float)STEPS_PER_BEAT - PHASE_GAIN * err;
      baseSpeed = SPIN_DIR * (distance / periodS);
    }
  } else if (!locked) {
    // Active but no beat measured yet: hold nominal speed so the wheel is moving.
    baseSpeed = bpmToStepsPerSec(g_bpm);
  }

  // Slew the commanded speed toward the PLL target so the motor can actually
  // accelerate into it instead of stalling against the cam load.
  uint32_t nowUs = micros();
  float dt = (nowUs - lastSlewUs) * 1e-6f;
  lastSlewUs = nowUs;
  if (dt > 0.05f) dt = 0.05f;            // clamp huge dt after a stall/long gap
  float maxDelta = MAX_ACCEL * dt;
  float d = baseSpeed - cmdSpeed;
  if      (d >  maxDelta) d =  maxDelta;
  else if (d < -maxDelta) d = -maxDelta;
  cmdSpeed += d;

  stepper.setSpeed(cmdSpeed);
  stepper.runSpeed();   // must be called as often as possible to emit steps

  // Diagnostics: once a second while spinning, dump live drive health so we can
  // see whether current actually reaches the coils (cs_act) or a coil is open
  // (ola/olb). Remove once the torque issue is resolved — the UART read briefly
  // pauses stepping.
  static uint32_t lastDiagMs = 0;
  if (millis() - lastDiagMs >= 1000) {
    lastDiagMs = millis();
    motorLogLive();
  }
}
