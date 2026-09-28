#include <Arduino.h>
#include "motor.h"
#include "log.h"

// --- Motor pins ---
static const int STEP_PIN = 32;
static const int DIR_PIN  = 33;
static const int EN_PIN   = 25;   // TMC2209 EN (active LOW)

// --- TMC2209 UART ---
// Single-wire half-duplex: place a 1kΩ resistor between TX and RX, then tie
// the junction to PDN_UART on the driver board. MS1=GND, MS2=GND (address 0).
#define TMC_SERIAL Serial2
static const int     TMC_RX_PIN = 16;
static const int     TMC_TX_PIN = 17;
static const uint8_t TMC_ADDR   = 0b00;
static const float   R_SENSE    = 0.11f;  // Ω — verify on your driver PCB

AccelStepper   stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
TMC2209Stepper driver(&TMC_SERIAL, R_SENSE, TMC_ADDR);

// Read the config back off the chip over UART and log it, so we can prove the
// current/microstep writes actually landed rather than assuming they did.
static void motorLogStatus(uint16_t requested_mA) {
  uint8_t conn = driver.test_connection();   // 0 = OK, 1 = no reply, 2 = bad CRC/version
  uint8_t ver  = driver.version();           // expect 0x21 for a genuine TMC2209
  uint8_t ifc  = driver.IFCNT();             // increments on every accepted write

  logPrintf("TMC2209 conn=%u ver=0x%02X ifcnt=%u\n", conn, ver, ifc);

  if (conn != 0 || ver != 0x21) {
    logPrintln("  -> UART link bad: check half-duplex 1k resistor, RX/TX, ADDR, R_SENSE");
    return;
  }

  // Read-backs: if these don't match what we wrote, the chip never took the config.
  logPrintf("  rms_current: set=%umA read=%umA cs=%u vsense=%u\n",
            requested_mA, driver.rms_current(), driver.cs_actual(), driver.vsense());
  logPrintf("  irun=%u ihold=%u microsteps=%u\n",
            driver.irun(), driver.ihold(), driver.microsteps());

  uint32_t g = driver.GSTAT();               // bit0 reset, bit1 drv_err, bit2 uv_cp
  if (g) logPrintf("  GSTAT=0x%02X (reset/drv_err/uv flags set)\n", g);
}

void motorInit() {
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);             // disable during config to avoid random motion

  TMC_SERIAL.begin(115200, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);
  driver.begin();
  if (driver.test_connection() != 0)
    logPrintln("TMC2209 UART not responding - check wiring and R_SENSE");

  const uint16_t kRmsCurrent = 1400;     // mA RMS — rated for JK42HS60-1704A (1.7A/phase ÷ √2)
  driver.rms_current(kRmsCurrent);        // push to ~1400 for short-burst peak torque, but it runs hot
  driver.microsteps(MICROSTEPS);
  driver.toff(4);                        // enable driver output (0 = off)
  driver.en_spreadCycle(true);           // SpreadCycle: consistent torque for fast hits
  //driver.en_spreadCycle(false);           // stealthChop: quieter for slow hits, but torque drops off at higher speeds

  motorLogStatus(kRmsCurrent);           // prove the config landed over UART

  // Leave the driver disabled (EN HIGH) at boot: the system starts idle, and a
  // disabled driver draws no holding current — no torque, and no chopper whine
  // while stopped. The phase-lock layer enables it once the beat is locked in.
  digitalWrite(EN_PIN, HIGH);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Phase corrections can briefly exceed the nominal rate, so give the cap headroom.
  stepper.setMaxSpeed((MAX_BPM / 60.0f) * STEPS_PER_BEAT * 1.3f + 100.0f);
}

// Decode DRV_STATUS in one UART read. cs_act should sit near irun (24) when the
// motor is moving under SpreadCycle — if it's low, current isn't reaching the
// coils. ola/olb flag an open or miswired coil; s2g* a short; ot/otpw heat.
void motorLogLive() {
  uint32_t s = driver.DRV_STATUS();
  logPrintf("DRV cs_act=%lu ola=%lu olb=%lu s2ga=%lu s2gb=%lu ot=%lu otpw=%lu stst=%lu\n",
            (s >> 16) & 0x1F,           // CS_ACTUAL: live current scaler (0..31)
            (s >> 6) & 1, (s >> 7) & 1, // open load A / B
            (s >> 2) & 1, (s >> 3) & 1, // short to ground A / B
            (s >> 1) & 1, (s >> 0) & 1, // over-temp / over-temp prewarn
            (s >> 31) & 1);             // standstill
}

// Change microstep resolution on the fly (UART write to CHOPCONF MRES). The
// striker drops to full steps for the fast release move and back to MICROSTEPS to
// cock. Done between moves while the motor is stopped, so the brief UART stall is
// harmless. mstep_reg_select is already set (boot readback confirmed UART MRES).
void motorSetMicrosteps(uint16_t ms) {
  // TMCStepper's setter encodes full step as 0 and has NO case for 1, so
  // microsteps(1) silently no-ops (driver stays at the old resolution). Translate
  // 1 -> 0 so callers can pass the literal microstep count (1 = full step).
  driver.microsteps(ms == 1 ? 0 : ms);
}

// Energize the coils (EN LOW). Call when the wheel should be spinning.
void motorEnable() {
  digitalWrite(EN_PIN, LOW);
}

// Cut the coils (EN HIGH): no holding current, so no high-pitched chopper noise
// while the motor is stopped. The rotor is left free (no holding torque).
void motorDisable() {
  digitalWrite(EN_PIN, HIGH);
}
