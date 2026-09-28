#include <Arduino.h>
#include "motor.h"
#include "espnow.h"
#include "motorPhaseLock.h"

void setup() {
  Serial.begin(115200);
  espnowInit();
  motorInit();
  motorPhaseLockInit();
}

void loop() {
  motorPhaseLockUpdate();
}
