#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "protocol.h"

// --- Pairing ---
// Flash this board first, read its MAC from serial, then put that MAC into
// receiver/main.cpp SENDER_MAC[] before flashing the receiver.
static const uint8_t RECEIVER_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// --- Heartbeat sensor ---
static const int SENSOR_PIN = 1;             // digital pulse, HIGH per beat
static const uint32_t REFRACTORY_MS = 300;   // ignore edges within this window (caps at ~200 BPM)
static const int AVG_WINDOW = 4;             // rolling average over last N inter-beat intervals
static const uint8_t MAX_BPM = 220;          // clamp implausible readings
static const uint8_t DEFAULT_BPM = 75;       // fallback rate before first measurement / when idle
static const uint32_t IDLE_TIMEOUT_MS = 3000; // no beat for this long -> assume sensor removed

// Beat-detection state
static bool lastLevel = false;
static uint32_t lastBeatMs = 0;
static uint32_t intervals[AVG_WINDOW];
static int intervalCount = 0;                // how many slots filled (0..AVG_WINDOW)
static int intervalIdx = 0;                  // next slot to write (ring buffer)
static bool idle = true;                     // true = sensor presumed off (no recent beat)

static esp_now_peer_info_t peer;

// Average of stored inter-beat intervals -> BPM. Falls back to DEFAULT_BPM until
// at least one full interval has been measured (a single beat can't give a rate).
uint8_t computeBpm() {
  if (intervalCount == 0) return DEFAULT_BPM;
  uint32_t sum = 0;
  for (int i = 0; i < intervalCount; i++) sum += intervals[i];
  uint32_t avg = sum / intervalCount;
  if (avg == 0) return DEFAULT_BPM;
  uint32_t bpm = 60000UL / avg;
  return bpm > MAX_BPM ? MAX_BPM : (uint8_t)bpm;
}

static bool espNowReady = false;   // true only after init + peer added

static bool isMacSet(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) if (mac[i] != 0) return true;
  return false;
}

void sendHeartbeat(bool idle, uint8_t bpm, bool trigger) {
  if (!espNowReady) return;   // measure-only mode: don't touch an uninitialized stack
  HeartbeatPacket pkt = {idle, bpm, trigger};
  esp_now_send(RECEIVER_MAC, (uint8_t *)&pkt, sizeof(pkt));
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  // Bring ESP-NOW up without halting: if anything fails the board still boots
  // and prints BPM over serial (measure-only mode), it just won't transmit.
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed — measure-only mode");
  } else if (!isMacSet(RECEIVER_MAC)) {
    Serial.println("RECEIVER_MAC not set — measure-only mode (no transmit)");
  } else {
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, RECEIVER_MAC, 6);
    peer.channel = 0;       // 0 = use current WiFi channel (default: ch 1)
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("Failed to add peer — measure-only mode");
    } else {
      espNowReady = true;
      Serial.println("ESP-NOW ready");
    }
  }

  pinMode(SENSOR_PIN, INPUT);   // use INPUT_PULLUP if the sensor is open-collector
}

void loop() {
  bool level = digitalRead(SENSOR_PIN);
  uint32_t now = millis();

  // Rising edge = beat onset.
  if (level && !lastLevel) {
    // Refractory window rejects noise and multiple edges within one pulse.
    if (lastBeatMs == 0 || now - lastBeatMs >= REFRACTORY_MS) {
      // Only record an interval between two consecutive beats — not across an
      // idle gap (that interval would be the time the sensor was off).
      if (lastBeatMs != 0 && !idle) {
        intervals[intervalIdx] = now - lastBeatMs;
        intervalIdx = (intervalIdx + 1) % AVG_WINDOW;
        if (intervalCount < AVG_WINDOW) intervalCount++;
      }
      lastBeatMs = now;
      idle = false;

      uint8_t bpm = computeBpm();
      sendHeartbeat(false, bpm, true);   // fire trigger immediately, latency-critical
      Serial.printf("Beat! BPM=%u\n", bpm);
    }
  }

  lastLevel = level;

  // No beat for IDLE_TIMEOUT_MS -> assume the sensor was taken off.
  if (!idle && lastBeatMs != 0 && now - lastBeatMs >= IDLE_TIMEOUT_MS) {
    idle = true;
    intervalCount = 0;   // drop history so BPM falls back to DEFAULT_BPM
    intervalIdx = 0;
    sendHeartbeat(true, DEFAULT_BPM, false);   // notify receiver, no trigger
    Serial.println("Sensor idle — BPM reset to default");
  }
}
