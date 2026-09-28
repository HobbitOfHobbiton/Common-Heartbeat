#pragma once

// Optional 128x64 SSD1306 OLED over I2C — a debug mirror of the serial log, used
// only on units that have one wired. displayInit() auto-detects the panel; if
// none is present every call is a no-op, so the same firmware runs everywhere.

// Probe the I2C bus and bring up the OLED if found. Returns true if present.
bool displayInit();

// Append one line to the on-screen scrolling log. No-op if no display.
// Signature matches LogSink so it can be registered via logSetSink().
void displayLine(const char* line);
