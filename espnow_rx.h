#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include "fc.h"

// Must match the transmitter sketch (espflight_tx)
struct __attribute__((packed)) RcPacket {
  uint16_t magic;
  uint8_t  seq;
  uint16_t ch[8];     // 1000..2000 : A E T R AUX1 AUX2 AUX3 AUX4
  uint8_t  crc;
};

static uint8_t espnowCrc8(const uint8_t* d, size_t n) {
  uint8_t crc = 0;
  while (n--) {
    crc ^= *d++;
    for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static void handleEspNow(const uint8_t* data, int len) {
  if (len != (int)sizeof(RcPacket)) return;
  RcPacket p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != ESPNOW_MAGIC) return;
  if (espnowCrc8(data, len - 1) != p.crc) return;
  for (int i = 0; i < 8; i++) {
    uint16_t v = p.ch[i];
    if (v < 800) v = 800;
    if (v > 2200) v = 2200;
    espRc[i] = v;
  }
  espLastMs = millis();
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onEspNow(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  handleEspNow(data, len);
}
#else
static void onEspNow(const uint8_t* mac, const uint8_t* data, int len) {
  handleEspNow(data, len);
}
#endif

static bool espnowBegin() {
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onEspNow);
  return true;
}
