#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "protocol.h"
#include "espnow.h"

// --- Pairing ---
// Flash sender first, read its MAC from serial, then fill this in and flash receiver.
static const uint8_t SENDER_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

volatile uint8_t  g_bpm = 0;
volatile bool     g_idle = true;             // start paused until a real packet arrives
volatile bool     g_triggerPending = false;
volatile uint32_t g_triggerMs = 0;

static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(HeartbeatPacket)) return;
  if (memcmp(info->src_addr, SENDER_MAC, 6) != 0) return;

  HeartbeatPacket pkt;
  memcpy(&pkt, data, len);

  g_idle = pkt.idle;
  g_bpm  = pkt.bpm;
  if (pkt.trigger) {
    g_triggerMs = millis();    // timestamp the beat as early as possible
    g_triggerPending = true;
  }
}

void espnowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed — halting");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);
}
