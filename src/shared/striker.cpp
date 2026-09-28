#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include "motor.h"
#include "heartbeat.h"
#include "striker.h"
#include "log.h"
#ifdef HEARTBEAT_DEMO
#include "sensor.h"   // demo builds only: 'demo <bpm>' live-adjusts the synthetic tempo
#endif

// =============================================================================
// Drum-strike state machine: gravity + elastic hammer, capstan-cocked.
//
//   IDLE ──beat──▶ COCKING ──reached top──▶ ARMED ──release due──▶ RELEASING
//     ▲                                                                 │
//     └──────────────── re-cock for the next beat ◀────────────────────┘
//
// COCK winds the string in (lifts the mallet, stretches the elastic) slowly and
// strongly. RELEASE unwinds back to rest as fast as the loop sustains, throwing
// the string slack so gravity + the elastic — not the motor — drive the hit; the
// rotor's inertia is decoupled while the string is slack. The release is started
// `strikeLeadMs` before the *predicted* beat so the mallet lands on the beat.
//
// Cock/release are AccelStepper position moves (this layer wants the accel ramps
// AccelStepper provides, unlike the old constant-velocity cam loop). Geometry is
// unknown until the rig exists, so everything is a live serial tunable (type '?').
// =============================================================================

static const long STEPS_PER_MOTOR_REV = (long)STEPS_PER_REV * MICROSTEPS;  // 3200

// --- Live tunables (serial-settable) ---
static int      windDir      = 1;        // +1 winds string in (lifts); flip if reversed
static float    cockRevs     = 1.0f;     // motor turns to lift/cock the mallet
static float    cockSpeed    = 8000.0f;  // steps/s while cocking (slow + strong) ref = 4000
static float    cockAccel    = 16000.0f;  // steps/s^2 while cocking ref = 8000
// Speeds/accels below are in "canonical" units = one microstep at the cocking
// resolution (MICROSTEPS). The release runs the driver at `releaseMicro` (coarser,
// default full step), so these get divided by the resolution ratio before they go
// to AccelStepper — that's what lets the loop's fixed pulse-rate ceiling translate
// into far higher rev/s when the string dumps slack. Tune them in canonical units;
// the unit basis stays the same whatever `releaseMicro` you pick.
static float    releaseSpeed = 20000.0f; // unwind rate (canonical units/s) ref - 80000
static float    releaseAccel = 400000.0f;// unwind accel (canonical units/s^2) ref = 400000
static int      releaseMicro = 1;        // microstep res during release (1 = full step)
static float    parkSpeed    = 8000.0f;  // gentle return-to-home rate (canonical units/s)
static float    parkAccel    = 16000.0f; // return-to-home accel (canonical units/s^2)
static uint32_t strikeLeadMs = 100;      // release this many ms before the beat
static bool     autoMode     = true;     // follow the heartbeat (false = manual only)

// --- Beat tracking / prediction ---
static const uint32_t MIN_PERIOD_MS = 60000UL / MAX_BPM;
static const uint32_t MAX_PERIOD_MS = 60000UL / MIN_BPM;
static uint32_t lastBeatMs = 0;
static uint32_t periodMs   = 600;        // smoothed beat period; seeded on 1st beat
static uint8_t  beatCount  = 0;          // need >=2 beats before auto-releasing

// --- State machine ---
enum StrikeState { ST_IDLE, ST_COCKING, ST_ARMED, ST_RELEASING };
static const char* STATE_NAME[] = { "IDLE", "COCKING", "ARMED", "RELEASING" };
static StrikeState state = ST_IDLE;
static bool manualFire = false;          // 'f': release as soon as cocking finishes
static bool parking    = false;          // release is a return-to-home: idle on arrival

static long cockedSteps() { return windDir * lround(cockRevs * STEPS_PER_MOTOR_REV); }

// Canonical units per release pulse, e.g. 16/1 = 16 when releasing at full step.
// AccelStepper counts pulses, so on the resolution switch we divide the cocked
// position and the release speed/accel by this to keep the physical move correct.
static long releaseRatio() {
  int rm = (releaseMicro < 1) ? 1 : releaseMicro;
  return MICROSTEPS / rm;
}

static void startCock() {
  motorSetMicrosteps(MICROSTEPS);     // fine steps: smooth, strong cocking (pos is at rest)
  stepper.setMaxSpeed(cockSpeed);
  stepper.setAcceleration(cockAccel);
  stepper.moveTo(cockedSteps());
  motorEnable();
  state = ST_COCKING;
}

// Unwind to home (position 0). `park` true = this is a return-to-rest, so disable
// on arrival instead of re-cocking. Don't call while already RELEASING — it would
// rescale the already-coarse position a second time.
static void startRelease(bool park) {
  parking = park;
  if (park) {
    // Parking is a deliberate, gentle return to home — not a slack-dump. Run it at
    // fine resolution (lands accurately on 0, no rescale) and its own slower speed.
    // Always entered from a fine-res state (COCKING/ARMED), so no rescale is needed.
    motorSetMicrosteps(MICROSTEPS);
    stepper.setMaxSpeed(parkSpeed);
    stepper.setAcceleration(parkAccel);
  } else {
    // Strike: coarse/full steps to outrun the fall and keep the string slack. Rescale
    // the canonical pulse count -> coarse pulses so moveTo(0) unwinds the same revs.
    long ratio = releaseRatio();
    motorSetMicrosteps(releaseMicro);
    stepper.setCurrentPosition(lround(stepper.currentPosition() / (float)ratio));
    stepper.setMaxSpeed(releaseSpeed / ratio);
    stepper.setAcceleration(releaseAccel / ratio);
  }
  stepper.moveTo(0);                  // unwind to home (rest)
  state = ST_RELEASING;
}

// Cut current and park. ONLY ever called at home (position 0) — reached by a
// controlled unwind — so the rotor has no wound load to back-drive and the step
// counter stays the truth. We deliberately do NOT re-zero here: home is set once
// (boot or the 'z' command) and maintained by always returning to it. Disabling
// while wound + blind re-zeroing is exactly what made the rest point drift.
static void enterIdle() {
  motorDisable();                     // mallet free, no holding current/whine
  motorSetMicrosteps(MICROSTEPS);     // back to fine res for the next cock
  state = ST_IDLE;
}

// True once it's time to release so the strike lands on the next predicted beat.
static bool releaseDue() {
  if (beatCount < 2) return false;                 // need a measured period first
  uint32_t now  = millis();
  uint32_t next = lastBeatMs + periodMs;           // next predicted beat
  // Roll forward to a beat still in the future (covers missed/stale predictions).
  for (int i = 0; i < 8 && (int32_t)(now - next) > 0; i++) next += periodMs;
  return (int32_t)(now - (next - strikeLeadMs)) >= 0;
}

// ---------------------------------------------------------------------------
// Serial console
// ---------------------------------------------------------------------------
static void printParams() {
  // Split across two lines: the full set overflows log's 128-byte line buffer.
  logPrintf("striker %s pos=%ld auto=%d dir=%d cockRevs=%.2f lead=%lums period=%lums(~%lubpm)\n",
            STATE_NAME[state], (long)stepper.currentPosition(), autoMode, windDir, cockRevs,
            (unsigned long)strikeLeadMs, (unsigned long)periodMs,
            (unsigned long)(periodMs ? 60000UL / periodMs : 0));
  logPrintf("  cockSpd=%.0f cockAcc=%.0f relSpd=%.0f relAcc=%.0f relMicro=1/%d\n",
            cockSpeed, cockAccel, releaseSpeed, releaseAccel, releaseMicro);
  logPrintf("  parkSpd=%.0f parkAcc=%.0f\n", parkSpeed, parkAccel);
}

static void help() {
  logPrintln("Striker console (motor turns = revolutions):");
  logPrintln("  f          fire once (cock + release), ignores heartbeat");
  logPrintln("  k          cock & hold     r  release now     x  stop -> home");
  logPrintln("  z          set current position as home (zero)");
  logPrintln("  e / d      auto (heartbeat) mode on / off");
  logPrintln("  c <rev>    cock revolutions      lead <ms>  release lead time");
  logPrintln("  cs <sps>   cock speed   ca <sps2>  cock accel");
  logPrintln("  rs <sps>   release speed ra <sps2> release accel");
  logPrintln("  ps <sps>   park speed   pa <sps2>  park accel (return-to-home)");
  logPrintln("  ms <n>     release microstep res (1=full step, fastest)");
  logPrintln("  dir <1|-1> wind direction   p  print params");
#ifdef HEARTBEAT_DEMO
  logPrintln("  demo <bpm> set synthetic heartbeat tempo (demo build)");
#endif
  logPrintln("  g          driver DRV_STATUS readback   ?  help");
}

static void handleLine(char* s) {
  while (*s == ' ') s++;
  if (*s == 0) return;

  char  cmd[8] = {0};
  float val    = 0.0f;
  int   n      = sscanf(s, "%7s %f", cmd, &val);
  if (n < 1) return;
  bool hasVal = (n == 2);

  if      (!strcmp(cmd, "f"))   { autoMode = false; manualFire = true; startCock(); logPrintln("fire"); }
  else if (!strcmp(cmd, "k"))   { autoMode = false; startCock();   logPrintln("cock"); }
  else if (!strcmp(cmd, "r"))   { startRelease(/*park=*/false); logPrintln("release"); }
  else if (!strcmp(cmd, "x"))   {            // stop = unwind to home, then idle (no drift)
    autoMode = false;
    if (state == ST_RELEASING)  parking = true;       // already heading home; idle on arrival
    else if (state != ST_IDLE)  startRelease(/*park=*/true);
    else                        enterIdle();           // already home: just ensure disabled
    logPrintln("stop -> home");
  }
  else if (!strcmp(cmd, "z"))   {            // declare the current spot the home/zero
    stepper.setCurrentPosition(0);
    logPrintln("home set (pos=0)");
  }
  else if (!strcmp(cmd, "e"))   { autoMode = true;  logPrintln("auto ON"); }
  else if (!strcmp(cmd, "d"))   { autoMode = false; logPrintln("auto OFF"); }
  else if (!strcmp(cmd, "c")    && hasVal) { cockRevs = val;          printParams(); }
  else if (!strcmp(cmd, "lead") && hasVal) { strikeLeadMs = (uint32_t)val; printParams(); }
  else if (!strcmp(cmd, "cs")   && hasVal) { cockSpeed = val;         printParams(); }
  else if (!strcmp(cmd, "ca")   && hasVal) { cockAccel = val;         printParams(); }
  else if (!strcmp(cmd, "rs")   && hasVal) { releaseSpeed = val;      printParams(); }
  else if (!strcmp(cmd, "ra")   && hasVal) { releaseAccel = val;      printParams(); }
  else if (!strcmp(cmd, "ps")   && hasVal) { parkSpeed = val;         printParams(); }
  else if (!strcmp(cmd, "pa")   && hasVal) { parkAccel = val;         printParams(); }
  else if (!strcmp(cmd, "ms")   && hasVal) {
    // Clamp to a power of two that divides MICROSTEPS (1,2,4,8,16): keeps the ratio exact.
    int m = (int)val, v = 1;
    while (v * 2 <= MICROSTEPS && v * 2 <= m) v *= 2;
    releaseMicro = v;
    printParams();
  }
  else if (!strcmp(cmd, "dir")  && hasVal) { windDir = (val < 0) ? -1 : 1; printParams(); }
#ifdef HEARTBEAT_DEMO
  else if (!strcmp(cmd, "demo") && hasVal) { sensorSetDemoBpm((uint8_t)val); }
#endif
  else if (!strcmp(cmd, "p"))   { printParams(); }
  else if (!strcmp(cmd, "g"))   { motorLogLive(); }   // DRV_STATUS health readback
  else if (!strcmp(cmd, "?") || !strcmp(cmd, "h")) { help(); }
  else logPrintln("? (type ? for help)");
}

static void pollSerial() {
  static char    buf[48];
  static uint8_t len = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { buf[len] = 0; handleLine(buf); len = 0; }
    else if (len < sizeof(buf) - 1) buf[len++] = ch;
  }
}

// ---------------------------------------------------------------------------
void strikerInit() {
  // Boot assumes we power up at mechanical rest: the driver was disabled through the
  // power-off, so the load relaxed the spool to its hanging rest. If that's not true
  // on a given unit, hand-wind to rest and press 'z' (or use maintenance mode) to
  // re-declare home. From there the step counter tracks position with no drift.
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(cockSpeed);
  stepper.setAcceleration(cockAccel);
  motorDisable();
  state = ST_IDLE;
  logPrintln("Striker ready - type ? for serial commands");
}

void strikerUpdate() {
  pollSerial();

  // Fold each beat into the period estimate used to predict the next one.
  if (g_triggerPending) {
    g_triggerPending = false;
    uint32_t t = g_triggerMs;            // copy volatile once (constrain re-evaluates)
    if (beatCount == 0) {
      uint8_t bpm = g_bpm ? g_bpm : 1;
      periodMs = constrain(60000UL / bpm, MIN_PERIOD_MS, MAX_PERIOD_MS);
    } else {
      uint32_t p = constrain(t - lastBeatMs, MIN_PERIOD_MS, MAX_PERIOD_MS);
      periodMs = (periodMs * 3 + p) / 4;   // light smoothing
    }
    lastBeatMs = t;
    if (beatCount < 255) beatCount++;
  }

  // Sensor idle: return to home *under power*, then idle. We don't snap the driver
  // off where it stands — a wound, disabled rotor gets back-driven an untracked,
  // detent-dependent amount, which is what drifted the rest point over on/off
  // cycles. A cocked/cocking rig starts a parking release; an in-flight release just
  // runs to 0 below (its completion handler idles because g_idle is set).
  if (autoMode && g_idle) {
    beatCount = 0;
    if (state == ST_COCKING || state == ST_ARMED) startRelease(/*park=*/true);
  }

  switch (state) {
    case ST_IDLE:
      if (autoMode && !g_idle && beatCount >= 1) startCock();  // lift on the first beat
      break;

    case ST_COCKING:
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        state = ST_ARMED;
        if (manualFire) { manualFire = false; startRelease(/*park=*/false); }
      }
      break;

    case ST_ARMED:
      stepper.run();                                 // holds the cocked position
      if (autoMode && !g_idle && releaseDue()) startRelease(/*park=*/false);
      break;

    case ST_RELEASING:
      stepper.run();
      if (stepper.distanceToGo() == 0) {             // reached home (strike delivered)
        if (parking)                  { parking = false; enterIdle(); }
        else if (autoMode && !g_idle) { startCock(); }   // re-cock for the next beat
        else                          { enterIdle(); }   // manual fire/release: park at home
      }
      break;
  }
}
