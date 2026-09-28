# Common-Heartbeat

https://survival.art.pl/en/wspolne-bicie-serca/

https://survival.art.pl/en/edition/survival-24/nagroda-publicznosci/

![alt text](https://survival.art.pl/wp-content/uploads/2026/08/33-anna-underowicz-wspolne-bicie-serca-common-heartbeat-fot.-m.-kujda17-1600x1066.jpg)

An art installation, transcribing heartbeat into drum hit
ESP32 drives a NEMA17 stepper as an automated drummer that
strikes a large cajón. A second ESP reads a heartbeat sensor and sends a trigger over
ESP-NOW on every beat, so the drum follows a person's pulse. Past the installation
phase the receiver is intended to become a MIDI-controllable physical drum machine.

## Hardware

| Role | Board | Notes |
|------|-------|-------|
| **Receiver** (drummer) | ESP32 DOIT DevKit V1 | Drives the stepper, locks to the beat |
| **Sender** (heartbeat) | ESP32-C3-Zero (dev on ESP32-S3-Zero) | Reads the pulse sensor, transmits beats |

- **Motor**: NEMA17 — currently `JK42HS40-0504` (12 V, 0.5 A/phase, 0.43 Nm)
- **Driver**: TMC2209 — STEP/DIR for motion, UART for configuration
- **Mechanism**: two-tooth cam wheel; one full revolution = one heartbeat (under review)

## Architecture

The two boards talk over **ESP-NOW** using a shared `HeartbeatPacket` (`include/protocol.h`).
The sender fires a `trigger` the instant it detects a beat (latency-critical); the receiver
runs a software PLL that frequency- and phase-locks the cam wheel so the strike lands on
the beat.

```
heartbeat sensor → [sender ESP32-C3] --ESP-NOW--> [receiver ESP32] → TMC2209 → NEMA17 → cajón
```

## Project layout

```
include/protocol.h            Shared HeartbeatPacket struct (identical on both sides)
platformio.ini                Boards, envs, and dependencies
src/sender/main.cpp           Heartbeat board firmware
src/receiver/
  main.cpp                    setup()/loop() — wires the layers together
  motor.h / motor.cpp         TMC2209 UART config + AccelStepper, mechanism constants
  motorPhaseLock.h / .cpp     Software PLL: frequency/phase lock + optional index sensor
  espnow.h / espnow.cpp       ESP-NOW bring-up, receive callback, shared heartbeat state
  motor_test.cpp              Standalone serial-driven motor bench test (no ESP-NOW)
```

The receiver firmware is split by responsibility. `main.cpp` only calls `espnowInit()`,
`motorInit()`, and `motorPhaseLockInit()`, then `motorPhaseLockUpdate()` in the loop. The
ESP-NOW receive callback writes shared `g_*` state (on core 0) that the phase-lock layer
reads (on core 1).

## TMC2209 wiring

Motion runs over STEP/DIR; the driver is configured once at boot over single-wire UART.

| TMC2209 | ESP32 | Notes |
|---------|-------|-------|
| STEP | GPIO 32 | |
| DIR  | GPIO 33 | |
| EN   | GPIO 14 | active LOW |
| PDN_UART | GPIO 17 (TX) via 1 kΩ, also to GPIO 16 (RX) | half-duplex single wire |
| MS1, MS2 | GND | sets UART address `0b00` |

Verify `R_SENSE` (0.11 Ω in `motor.cpp`) against your driver PCB, and confirm MS1/MS2 are
pulled to GND so the UART address matches. Current is set to **350 mA RMS** (≈ 0.5 A peak)
for the JK42HS40-0504; microstepping is 16; SpreadCycle is enabled for consistent torque
on fast strikes.

## Build and upload

> Building/flashing is done on the user's hardware — the commands below are for reference.

```bash
pio run -e receiver --target upload      # flash the ESP32 DevKit (drummer)
pio run -e sender   --target upload      # flash the heartbeat board
pio device monitor                       # 115200 baud
```

`default_envs` builds `receiver` and `sender-s3` (the S3 dev board). Switch to
`receiver, sender-c3` in `platformio.ini` once the C3-Zero boards arrive.

### Motor bench test

A standalone test that drives the motor directly with no ESP-NOW, for checking motions
and wiring. It compiles only `motor.cpp` + `motor_test.cpp`, so it does not clash with the
normal firmware:

```bash
pio run -e receiver-motortest --target upload && pio device monitor
```

Then send single-char commands over the serial monitor:

| Key | Action |
|-----|--------|
| `s` | spin continuously (constant rate, like the real strike loop) |
| `x` | stop |
| `r` | single full revolution (one strike) |
| `o` | oscillate back and forth ±half a revolution |
| `+` / `-` | faster / slower |
| `d` | reverse direction |
| `?` | reprint help |

## Pairing workflow

Required before any sender/receiver pair will talk:

1. Flash the **sender**, open the serial monitor, copy its `Sender MAC: XX:XX:...` line.
2. Paste that MAC into `SENDER_MAC[]` in `src/receiver/espnow.cpp`.
3. Paste the **receiver's** MAC into `RECEIVER_MAC[]` in `src/sender/main.cpp`.
4. Flash the **receiver**.
5. Repeat per pair, with unique MACs for each.

If a board's peer MAC is left unset, the sender runs in measure-only mode (prints BPM over
serial, no transmit) so you can verify heartbeat detection before pairing.

## Libraries

- **TMCStepper** (`teemuatlut/TMCStepper@^0.7.3`) — TMC2209 UART configuration.
- **AccelStepper** (`waspinator/AccelStepper@^1.64`) — STEP/DIR motion with accel profiles.
