/*
  ESPFLIGHT TX - ESP-NOW transmitter for the espflight firmware (Arduino IDE)

  Board: "ESP32 Dev Module" (any ESP32 works). No extra libraries.

  Wiring (ADC1 pins only, WiFi is on so ADC2 is unusable):
    Throttle stick  -> GPIO 32   (left stick, up/down, no spring centre)
    Yaw stick       -> GPIO 33   (left stick, left/right)
    Roll stick      -> GPIO 34   (right stick, left/right)
    Pitch stick     -> GPIO 35   (right stick, up/down)
    ARM switch      -> GPIO 25 to GND   (ON  = armed request)
    ANGLE switch    -> GPIO 26 to GND   (ON  = self-level mode, OFF = acro)
    AUX3 / AUX4     -> GPIO 27 / 14 to GND (optional, sent as channels 7/8)
    Status LED      -> GPIO 2 (on-board)

  No pairing needed: packets are broadcast on WiFi channel 1 and the drone only
  accepts packets with the right magic number + CRC.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ESPNOW_CHANNEL 1
#define ESPNOW_MAGIC   0xFC5A
#define SEND_PERIOD_MS 10          // 100 Hz

#define PIN_THR   32
#define PIN_YAW   33
#define PIN_ROLL  34
#define PIN_PITCH 35
#define PIN_ARM   25
#define PIN_ANGLE 26
#define PIN_AUX3  27
#define PIN_AUX4  14
#define PIN_LED   2

// Raw ADC range of your sticks (check with the serial monitor, set PRINT_ADC to 1)
#define ADC_MIN   200
#define ADC_MAX   3900
#define CENTER_DEADBAND 25         // microseconds around 1500 snapped to 1500

// Flip a stick direction if it is reversed
#define REV_THR   0
#define REV_YAW   0
#define REV_ROLL  0
#define REV_PITCH 0

#define PRINT_ADC 0

struct __attribute__((packed)) RcPacket {
  uint16_t magic;
  uint8_t  seq;
  uint16_t ch[8];
  uint8_t  crc;
};

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t seqNo = 0;
static uint32_t tLast = 0;

static uint8_t crc8(const uint8_t* d, size_t n) {
  uint8_t crc = 0;
  while (n--) {
    crc ^= *d++;
    for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static int readAdc(int pin) {
  int sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(pin);
  return sum / 4;
}

static uint16_t stickToPwm(int pin, bool rev, bool centred) {
  int raw = readAdc(pin);
  long us = map(raw, ADC_MIN, ADC_MAX, 1000, 2000);
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
  if (rev) us = 3000 - us;
  if (centred && abs((int)us - 1500) < CENTER_DEADBAND) us = 1500;
  return (uint16_t)us;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ARM, INPUT_PULLUP);
  pinMode(PIN_ANGLE, INPUT_PULLUP);
  pinMode(PIN_AUX3, INPUT_PULLUP);
  pinMode(PIN_AUX4, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  analogReadResolution(12);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void loop() {
  uint32_t now = millis();
  if (now - tLast < SEND_PERIOD_MS) return;
  tLast = now;

  RcPacket p;
  p.magic = ESPNOW_MAGIC;
  p.seq = seqNo++;
  p.ch[0] = stickToPwm(PIN_ROLL,  REV_ROLL,  true);    // A  roll
  p.ch[1] = stickToPwm(PIN_PITCH, REV_PITCH, true);    // E  pitch (up/forward = high)
  p.ch[2] = stickToPwm(PIN_THR,   REV_THR,   false);   // T  throttle
  p.ch[3] = stickToPwm(PIN_YAW,   REV_YAW,   true);    // R  yaw
  p.ch[4] = digitalRead(PIN_ARM)   == LOW ? 2000 : 1000;   // AUX1 = ARM
  p.ch[5] = digitalRead(PIN_ANGLE) == LOW ? 2000 : 1000;   // AUX2 = ANGLE
  p.ch[6] = digitalRead(PIN_AUX3)  == LOW ? 2000 : 1000;
  p.ch[7] = digitalRead(PIN_AUX4)  == LOW ? 2000 : 1000;
  p.crc = crc8((const uint8_t*)&p, sizeof(p) - 1);

  esp_err_t e = esp_now_send(BCAST, (const uint8_t*)&p, sizeof(p));
  digitalWrite(PIN_LED, e == ESP_OK ? ((now / 250) & 1) : HIGH);

#if PRINT_ADC
  Serial.printf("R %d P %d T %d Y %d\n", readAdc(PIN_ROLL), readAdc(PIN_PITCH), readAdc(PIN_THR), readAdc(PIN_YAW));
#endif
}
