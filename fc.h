#pragma once
#include <Arduino.h>
#include "config.h"

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Arming-disable flag bits (match Betaflight bit positions so Configurator shows names)
#define ARMDIS_NO_GYRO      (1u << 0)
#define ARMDIS_RX_FAILSAFE  (1u << 2)
#define ARMDIS_THROTTLE     (1u << 7)
#define ARMDIS_ANGLE        (1u << 8)
#define ARMDIS_CALIB        (1u << 12)
#define ARMDIS_ARMSWITCH    (1u << 25)
#define ARMDIS_COUNT        26

struct FcState {
  volatile bool     armed = false;
  volatile bool     angleMode = false;
  volatile bool     gyroOk = false;
  volatile bool     calibrating = false;
  volatile uint32_t armingDisable = 0;
  volatile uint8_t  rcSource = 0;          // 0 none, 1 ESP-NOW, 2 ESP-Drone app (CRTP)
  float roll = 0, pitchDown = 0, yaw = 0;  // degrees. roll + = right, pitch + = nose DOWN, yaw + = right
  float gyro[3] = {0, 0, 0};               // deg/s  roll, pitch(down+), yaw(right+)
  float acc[3]  = {0, 0, 1};               // g, body frame (flat = +Z 1g)
  float motor[4] = {0, 0, 0, 0};           // 0..1
  volatile uint16_t rc[MAX_RC_CH] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500}; // A E T R aux...
  float vbat = 0;
  volatile uint16_t cycleUs = 1000;
  volatile uint16_t i2cErrors = 0;
  volatile uint8_t  load = 0;
};

struct CrtpSetpoint {
  volatile float roll = 0, pitch = 0, yaw = 0;
  volatile uint16_t thrust = 0;
  volatile uint32_t lastMs = 0;
  volatile bool unlocked = false;
};

struct Inputs {
  float stick[4];    // roll, pitch(+ = forward/nose down), yaw (-1..1), throttle (0..1)
  bool  armReq;
  bool  angle;
  bool  link;
};

extern FcState fc;
extern Settings cfg;
extern CrtpSetpoint crtpSp;
extern volatile uint16_t espRc[MAX_RC_CH];
extern volatile uint32_t espLastMs;
extern volatile bool accCalRequested;
extern volatile uint32_t motorTestUntil;
extern volatile uint16_t motorTestVal[4];

void settingsDefaults();
void settingsSave();
