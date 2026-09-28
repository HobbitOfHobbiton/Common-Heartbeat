#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "display.h"

// --- I2C / panel config ---
// SDA/SCL are the ESP32 DevKit V1 defaults and are free (motor uses 32/33/14/16/
// 17/25, sensor 13). 0x3C is the usual address for these 128x64 modules (a few
// are 0x3D — change here if the probe fails).
static const int     I2C_SDA   = 21;
static const int     I2C_SCL   = 22;
static const uint8_t OLED_ADDR = 0x3C;

static const int OLED_W = 128;
static const int OLED_H = 64;

// 6x8 font at text size 1 -> 21 chars wide, 8 rows tall.
static const int COLS  = 22;   // 21 visible chars + null
static const int LINES = 8;

static Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);   // -1 = no reset pin
static bool present = false;

// Scrolling line buffer (oldest at index 0).
static char lines[LINES][COLS];
static int  lineCount = 0;

static void render() {
  oled.clearDisplay();
  for (int i = 0; i < lineCount; i++) {
    oled.setCursor(0, i * 8);
    oled.print(lines[i]);
  }
  oled.display();
}

bool displayInit() {
  Wire.begin(I2C_SDA, I2C_SCL);

  // Probe first: if nothing ACKs at the address, there's no panel on this unit.
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() != 0) return false;

  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) return false;

  present = true;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.display();
  return true;
}

void displayLine(const char* line) {
  if (!present) return;

  if (lineCount < LINES) {
    snprintf(lines[lineCount++], COLS, "%s", line);
  } else {
    // Full: scroll up one row, drop the oldest, append at the bottom.
    for (int i = 1; i < LINES; i++) memcpy(lines[i - 1], lines[i], COLS);
    snprintf(lines[LINES - 1], COLS, "%s", line);
  }
  render();
}
