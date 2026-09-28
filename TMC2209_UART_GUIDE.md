# TMC2209 UART Setup Guide (Arduino UNO)

Working notes and lessons from getting a BigTreeTech TMC2209 V1.3 driving a
JK42HS40-0504 stepper, configured over UART, on an Arduino UNO.

---

## Hardware

| Item     | Detail                                                        |
|----------|--------------------------------------------------------------|
| MCU      | Arduino UNO (16 MHz, ATmega328)                              |
| Driver   | BigTreeTech TMC2209 V1.3 — `R_SENSE = 0.11 Ω`               |
| Motor    | JK42HS40-0504: 12 V, 0.5 A/coil (peak), 0.43 Nm, bipolar (4 wires) |
| Libs     | TMCStepper, AccelStepper, AltSoftSerial                      |

---

## Current limit calculation

The motor's **0.5 A is the peak current per coil**. `rms_current()` wants RMS:

```
I_RMS = I_peak x 0.707 = 0.5 A x 0.707 = ~354 mA  ->  use 350 mA
```

- `rms_current()` sets the current **per coil/phase**. The driver regulates each
  coil independently — **do NOT double it for the two coils.**
- Setting `rms_current(350)` and reading it back gives `336` — this is normal.
  Current is set via a 5-bit register (32 steps), so it rounds down to the
  nearest achievable step. 336 mA is correct and slightly conservative.

---

## CRITICAL: I_scale_analog (the bit that made torque actually work)

By default the TMC2209 powers up with `I_scale_analog = 1`, meaning it uses the
**board's VREF potentiometer** as the current ceiling and **ignores** the current
you set over UART. Symptoms when this is wrong:

- Motor cold even after hours of running
- Weak torque, no change when you set current over UART
- `rms_current()` reads back a sensible value but it's fiction

**Fix — call this BEFORE `rms_current()`:**

```cpp
driver.I_scale_analog(false);   // use internal reference, ignore VREF pot
driver.internal_Rsense(false);  // use the board's external 0.11 ohm sense resistors
```

After this the current is set purely digitally and the VREF pot is irrelevant.

---

## Wiring (single-wire UART)

The BTT V1.3 exposes pads `EN, MS1, MS2, RX, TX, CLK, STEP, DIR`. Internally the
UART is **single-wire** — the **RX pad is the live PDN_UART line** (verified: it
idles at VIO and wiggles during a read). The TX pad is unused on this board.

```
Arduino pin 9 (AltSoftSerial TX) --[1k..4.7k]--+
                                               +--- TMC RX pad
Arduino pin 8 (AltSoftSerial RX) --------------+

TMC TX pad   -> leave unconnected
TMC CLK      -> leave unconnected
```

Other connections:

| Arduino       | TMC2209 / Motor          |
|---------------|--------------------------|
| GND           | GND (must be common)     |
| 5V            | VIO                      |
| Pin 7         | EN (LOW = enabled)       |
| Pin 5         | DIR                      |
| Pin 6         | STEP                     |
| —             | MS1 -> GND, MS2 -> GND (UART address 0b00) |
| 12V PSU       | VM                       |
| PSU GND       | GND                      |

Notes:
- **Use AltSoftSerial, not SoftwareSerial.** SoftwareSerial was unreliable for
  TMC UART on the UNO; AltSoftSerial uses a hardware timer (fixed pins RX=8/TX=9).
- Do NOT use pins 0/1 — they share the USB serial used for debugging.
- Resistor value is not critical: 1k and 4.7k both worked. Range ~470 ohm to ~4.7k.

---

## Software gotchas

- `version()` reads **0 in `setup()` and on the first loop pass, then 33**. The
  chip's UART needs a moment to settle after `begin()`. Add a delay and a
  `test_connection()` check, and put all config AFTER it so writes land.
- Method is `en_spreadCycle(bool)` (lowercase). **Inverted meaning:**
  `false` = StealthChop (quiet), `true` = SpreadCycle (louder, better high-speed torque).
- There is **no** combined `ihold_irun(a, b, c)` method. Use `irun()`, `ihold()`,
  `iholddelay()` separately, or the 2nd arg of `rms_current(mA, holdMultiplier)`.
- A high-pitched warbling/"FM" whine in SpreadCycle is **normal** chopper noise,
  not a fault.

---

## Verifying it works

- `driver.test_connection()` returns `0` = good link.
- `driver.version()` returns `33` (0x21) once settled.
- `driver.DRV_STATUS()` returns a real value, e.g. `0xC0050000`:
  - bit 31 `stst` = 1 (standstill)
  - bit 30 `stealth` = 1 (StealthChop active)
  - bits 20:16 `CS_ACTUAL` = 5 (actual current scale)
  - A failed read is `0x0` or `0xFFFFFFFF`.
- Physical check: with current correctly applied, the motor is **warm, not hot**
  (~1.5 W at 336 mA for this low-current motor), and noticeably harder to stop by
  hand. Test torque while the motor is **energized/holding**, not at idle (at
  standstill the IHOLD multiplier deliberately reduces current).

---

## Known-good setup() sketch

```cpp
#include <AltSoftSerial.h>
#include <TMCStepper.h>
#include <AccelStepper.h>

#define EN_PIN       7
#define DIR_PIN      5
#define STEP_PIN     6
#define DRIVER_ADDR  0b00
#define R_SENSE      0.11f

AltSoftSerial SerialTMC;                 // RX=8, TX=9 (fixed)
TMC2209Stepper driver(&SerialTMC, R_SENSE, DRIVER_ADDR);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);
  SerialTMC.begin(19200);                // 115200 is unreliable on UNO soft serial

  driver.begin();
  delay(200);                            // let the UART settle
  while (driver.test_connection() != 0) {
    Serial.println("TMC not responding...");
    delay(200);
  }
  Serial.print("version: ");
  Serial.println(driver.version());      // expect 33

  driver.I_scale_analog(false);          // <-- use internal ref (ignore VREF pot)
  driver.internal_Rsense(false);         // external 0.11 ohm sense resistors
  driver.rms_current(350, 0.5);          // 350 mA run, 50% hold
  driver.microsteps(16);
  driver.en_spreadCycle(false);          // StealthChop (quiet)
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);

  Serial.print("rms reads back: ");
  Serial.println(driver.rms_current());  // ~336

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);             // enable driver

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
}

void loop() {
  stepper.runToNewPosition(1600);        // 1 rev at 1/16 microstep
  delay(1000);
  stepper.runToNewPosition(0);
  delay(1000);
}
```

---

## Possible next steps

- `TCOOLTHRS` + `en_spreadCycle` to auto-switch StealthChop->SpreadCycle by speed.
- Sensorless homing via `SGTHRS` and the `DIAG` pin.
- Raise hold current (e.g. `rms_current(350, 0.9)`) if stronger standstill holding
  torque is needed.
