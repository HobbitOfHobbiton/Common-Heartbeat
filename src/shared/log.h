#pragma once

// Tiny logging facade: writes to Serial and, optionally, mirrors each line to a
// registered sink (the OLED display on units that have one). Hardware-agnostic —
// it knows nothing about the display, so shared/ stays free of display deps.

// Sink receives one already-formatted line, with no trailing newline.
typedef void (*LogSink)(const char* line);

// Register (or clear, with nullptr) the mirror sink. Until set, logs go only to
// Serial. Call once from setup() after the display is up.
void logSetSink(LogSink sink);

// Serial-style logging that also fans out to the sink. logPrintf accepts a
// printf format (a trailing '\n' is fine; it's stripped before the sink sees it).
void logPrintln(const char* msg);
void logPrintf(const char* fmt, ...);
