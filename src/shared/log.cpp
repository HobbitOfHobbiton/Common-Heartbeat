#include <Arduino.h>
#include <stdarg.h>
#include <string.h>
#include "log.h"

static LogSink g_sink = nullptr;

void logSetSink(LogSink sink) {
  g_sink = sink;
}

// Forward to the sink after trimming any trailing newline/carriage return, so the
// display always gets clean single lines.
static void forward(char* line) {
  if (!g_sink) return;
  size_t n = strlen(line);
  while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r')) line[--n] = '\0';
  g_sink(line);
}

void logPrintln(const char* msg) {
  Serial.println(msg);
  if (g_sink) g_sink(msg);   // msg carries no newline
}

void logPrintf(const char* fmt, ...) {
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  Serial.print(buf);         // preserve the caller's formatting (incl. any '\n')
  forward(buf);              // trims the newline for the line-oriented display
}
