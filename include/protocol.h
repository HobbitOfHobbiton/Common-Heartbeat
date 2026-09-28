#pragma once
#include <stdint.h>

// Packet sent from sender (heartbeat board) to receiver (motor board) via ESP-NOW.
// Keep this struct identical on both sides.
struct HeartbeatPacket {
  bool idle;     // true = no heartbeat, false = heartbeat detected
  uint8_t bpm;    // current BPM, 0 if unknown
  bool trigger;   // true = hit the drum now
};
