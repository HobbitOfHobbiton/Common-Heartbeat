#include <Arduino.h>
#include "sensor.h"
#include "log.h"

#ifdef HEARTBEAT_DEMO
// Demo/showcase build: ignore the physical pin and synthesize a steady heartbeat
// so the rig runs with no sensor wired (see HEARTBEAT_DEMO in platformio.ini).
#ifndef HEARTBEAT_DEMO_BPM
#define HEARTBEAT_DEMO_BPM 50
#endif
#endif

// --- Heartbeat sensor ---
// GPIO13: free pin on the DOIT ESP32 DevKit V1 with pull-up support. (The wireless
// sender used GPIO1, which is UART0 TX here and would clash with Serial logging.)
// Motor/TMC use 32, 33, 14, 16, 17 (and 25 for the optional home sensor).
static const int      SENSOR_PIN      = 13;      // digital pulse, HIGH per beat
static const uint32_t REFRACTORY_MS   = 300;     // ignore edges within this window (caps at ~200 BPM)
static const int      WINDOW_N        = 5;       // median window over recent *accepted* intervals
static const uint8_t  MAX_BPM         = 220;     // plausibility ceiling + upper clamp
static const uint8_t  MIN_BPM         = 30;      // plausibility floor + lower clamp
static const uint8_t  DEFAULT_BPM     = 75;      // fallback rate before first measurement / when idle
static const uint32_t IDLE_TIMEOUT_MS = 3000;    // no beat for this long -> assume sensor removed

// --- Shared state (defined here, declared in heartbeat.h) ---
volatile uint8_t  g_bpm = DEFAULT_BPM;
volatile bool     g_idle = true;             // start paused until a real beat arrives
volatile bool     g_triggerPending = false;
volatile uint32_t g_triggerMs = 0;

#ifdef HEARTBEAT_DEMO
// ===========================================================================
// Synthetic producer: emit a steady beat at `demoBpm`, filling the same shared
// state a real beat would. The striker is producer-agnostic, so it can't tell
// the difference — it locks to this tempo exactly as to a real pulse.
// ===========================================================================
static uint8_t  demoBpm    = HEARTBEAT_DEMO_BPM;
static uint32_t lastBeatMs = 0;

void sensorSetDemoBpm(uint8_t bpm) {
  if (bpm < MIN_BPM) bpm = MIN_BPM;
  if (bpm > MAX_BPM) bpm = MAX_BPM;
  demoBpm = bpm;
  logPrintf("Demo BPM=%u\n", (unsigned)demoBpm);
}

void sensorInit() {
  // No pin to configure — the beat is generated, not measured.
  g_idle = false;                 // demo never goes idle; always playing
  logPrintf("Demo mode - synthetic heartbeat @ %u BPM\n", (unsigned)demoBpm);
}

void sensorUpdate() {
  uint32_t now      = millis();
  uint32_t interval = 60000UL / demoBpm;   // demoBpm is clamped >= MIN_BPM, never 0
  // Wrap-safe across the ~49.7-day millis() rollover, same idiom as the real path.
  if (lastBeatMs == 0 || (uint32_t)(now - lastBeatMs) >= interval) {
    lastBeatMs = now;
    g_idle = false;
    g_bpm  = demoBpm;
    g_triggerMs = now;            // timestamp the beat for the striker's prediction
    g_triggerPending = true;
    logPrintf("Demo beat! BPM=%u\n", (unsigned)demoBpm);
  }
}

#else

// --- Beat-detection state (owned here) ---
static bool     lastLevel = false;
static uint32_t lastBeatMs = 0;
static uint32_t intervals[WINDOW_N];   // recent *accepted* inter-beat intervals (ms)
static int      intervalCount = 0;     // how many slots filled (0..WINDOW_N)
static int      intervalIdx = 0;       // next slot to write (ring buffer)
static uint32_t estIntervalMs = 0;     // EMA-smoothed inter-beat interval (0 = unseeded)
static uint8_t  rejectStreak = 0;      // consecutive intervals rejected as outliers

// Drop all rate history (on idle, or when re-seeding after a real tempo change).
static void resetBeatStats() {
  intervalCount = 0;
  intervalIdx   = 0;
  estIntervalMs = 0;
  rejectStreak  = 0;
}

// Median of the accepted-interval window. The median (not the mean) is what makes
// the estimate robust: a single doubled interval (a missed beat) or a short noise
// spike that slips past the gate moves the median by at most one rank, never drags
// it the way an arithmetic mean does.
static uint32_t medianInterval() {
  uint32_t tmp[WINDOW_N];
  for (int i = 0; i < intervalCount; i++) tmp[i] = intervals[i];
  for (int i = 1; i < intervalCount; i++) {        // insertion sort (tiny N)
    uint32_t v = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = v;
  }
  return tmp[intervalCount / 2];
}

// Fold a trusted interval into the window + EMA.
static void acceptInterval(uint32_t iv) {
  intervals[intervalIdx] = iv;
  intervalIdx = (intervalIdx + 1) % WINDOW_N;
  if (intervalCount < WINDOW_N) intervalCount++;
  uint32_t med = medianInterval();
  estIntervalMs = (estIntervalMs == 0) ? med : (estIntervalMs * 3 + med) / 4;  // light EMA
  rejectStreak = 0;
}

// Smoothed interval -> BPM, clamped to the plausible band on BOTH sides (the old
// code clamped only the high end, so a stray short interval could underflow/wrap).
static uint8_t computeBpm() {
  if (estIntervalMs == 0) return DEFAULT_BPM;
  uint32_t bpm = 60000UL / estIntervalMs;
  if (bpm < MIN_BPM) bpm = MIN_BPM;
  if (bpm > MAX_BPM) bpm = MAX_BPM;
  return (uint8_t)bpm;
}

void sensorInit() {
  pinMode(SENSOR_PIN, INPUT);   // use INPUT_PULLUP if the sensor is open-collector
}

void sensorUpdate() {
  bool level = digitalRead(SENSOR_PIN);
  uint32_t now = millis();

  // Rising edge = beat onset.
  if (level && !lastLevel) {
    // Refractory window rejects noise and multiple edges within one pulse. The
    // (uint32_t) subtraction is wrap-safe across the ~49.7-day millis() rollover.
    if (lastBeatMs == 0 || (uint32_t)(now - lastBeatMs) >= REFRACTORY_MS) {
      uint32_t prevBeat = lastBeatMs;
      bool     wasIdle  = g_idle;

      // Fire the strike trigger FIRST — it's latency-critical. The rate filtering
      // below never gates the trigger, only what we believe the tempo is.
      lastBeatMs = now;
      g_idle = false;
      g_triggerMs = now;          // timestamp the beat as early as possible
      g_triggerPending = true;

      // Only measure a rate across two real consecutive beats — not across an idle
      // gap (that "interval" would be the time the sensor was off).
      if (prevBeat != 0 && !wasIdle) {
        uint32_t iv    = now - prevBeat;
        uint32_t minIv = 60000UL / MAX_BPM;        // shortest plausible interval
        uint32_t maxIv = 60000UL / MIN_BPM;        // longest plausible interval
        bool ok = (iv >= minIv && iv <= maxIv);

        // Relative gate vs the current estimate: reject single outliers, i.e. a
        // missed beat (~2x) or a noise double-trigger (~0.5x) that would otherwise
        // poison the rate. Band ~ [0.6x, 1.8x] of the smoothed interval.
        if (ok && estIntervalMs != 0 &&
            (iv < (estIntervalMs * 3) / 5 || iv > (estIntervalMs * 9) / 5)) {
          ok = false;
        }

        if (ok) {
          acceptInterval(iv);
        } else if (++rejectStreak >= 3) {
          // Several intervals in a row disagree with the estimate -> the tempo
          // really changed (or we lost lock). Drop history and re-seed from this
          // beat so we converge instead of clinging to a stale rate forever.
          resetBeatStats();
          if (iv >= minIv && iv <= maxIv) acceptInterval(iv);
        }
        g_bpm = computeBpm();
      }
      logPrintf("Beat! BPM=%u\n", g_bpm);
    }
  }

  lastLevel = level;

  // No beat for IDLE_TIMEOUT_MS -> assume the sensor was taken off.
  if (!g_idle && lastBeatMs != 0 && (uint32_t)(now - lastBeatMs) >= IDLE_TIMEOUT_MS) {
    g_idle = true;
    g_bpm = DEFAULT_BPM;
    resetBeatStats();             // drop history so BPM falls back to default
    logPrintln("Sensor idle - BPM reset to default");
  }
}

#endif  // HEARTBEAT_DEMO
