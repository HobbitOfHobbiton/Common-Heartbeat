// Standalone motor bench test — drives the TMC2209 directly, no ESP-NOW.
// Build/flash with:  pio run -e receiver-motortest -t upload
//
// Open the serial monitor at 115200 and send single-char commands:
//
//   s  spin continuously (constant rate, like the real strike loop)
//   x  stop
//   r  single full revolution (one strike)
//   o  oscillate back and forth ±half a revolution
//   +  faster        -  slower
//   d  reverse direction
//   ?  reprint this help
//
// 's' uses constant-rate runSpeed(); 'r'/'o' use AccelStepper's accel profile.
#include <Arduino.h>
#include "motor.h"

enum class Mode { Idle, Spin, Position, Oscillate };
static Mode  mode = Mode::Idle;
static float spinSpeed = STEPS_PER_BEAT;   // steps/s — start at ~1 rev/s
static int   dir = 1;                      // +1 / -1
static long  oscTarget = STEPS_PER_BEAT / 2;

static const float SPEED_STEP = STEPS_PER_BEAT * 0.25f;

static void printHelp() {
  Serial.println(F("\nMotor bench test — commands:"));
  Serial.println(F("  s spin   x stop   r 1 rev   o oscillate"));
  Serial.println(F("  + faster - slower  d reverse  ? help"));
}

static void handleCommand(char c) {
  switch (c) {
    case 's':
      mode = Mode::Spin;
      Serial.println(F("spin"));
      break;
    case 'x':
      mode = Mode::Idle;
      Serial.println(F("stop"));
      break;
    case 'r':
      mode = Mode::Position;
      stepper.move((long)dir * STEPS_PER_BEAT);
      Serial.println(F("single revolution"));
      break;
    case 'o':
      mode = Mode::Oscillate;
      stepper.setCurrentPosition(0);
      oscTarget = (long)dir * STEPS_PER_BEAT / 2;
      stepper.moveTo(oscTarget);
      Serial.println(F("oscillate"));
      break;
    case '+':
      spinSpeed = min(spinSpeed + SPEED_STEP, stepper.maxSpeed());
      Serial.printf("speed = %.0f steps/s\n", spinSpeed);
      break;
    case '-':
      spinSpeed = max(spinSpeed - SPEED_STEP, 0.0f);
      Serial.printf("speed = %.0f steps/s\n", spinSpeed);
      break;
    case 'd':
      dir = -dir;
      Serial.printf("direction = %d\n", dir);
      break;
    case '?':
      printHelp();
      break;
    default:
      break;   // ignore stray newlines / whitespace
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  motorInit();
  stepper.setAcceleration(STEPS_PER_BEAT * 4.0f);  // only used by positioned moves
  printHelp();
}

void loop() {
  if (Serial.available()) handleCommand((char)Serial.read());

  switch (mode) {
    case Mode::Idle:
      stepper.setSpeed(0.0f);
      stepper.runSpeed();
      break;

    case Mode::Spin:
      stepper.setSpeed(dir * spinSpeed);
      stepper.runSpeed();
      break;

    case Mode::Position:
      stepper.run();
      if (stepper.distanceToGo() == 0) mode = Mode::Idle;   // done, hold
      break;

    case Mode::Oscillate:
      if (stepper.distanceToGo() == 0) {
        oscTarget = -oscTarget;          // flip ends and keep swinging
        stepper.moveTo(oscTarget);
      }
      stepper.run();
      break;
  }
}
