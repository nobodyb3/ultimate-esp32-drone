#pragma once
#include <Arduino.h>

// ============================================================
//  ESPFLIGHT - ESP32 (WROOM) brushed-quad flight controller
//  Arduino IDE sketch. Board: "ESP32 Dev Module"
// ============================================================

#define FW_NAME_STR    "ESPFLIGHT"   // keep exactly 9 chars (msp.h board-info length byte)

// ---------------- Hardware pins ----------------
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_MOTOR1     25   // M1 rear  right  (Betaflight numbering)
#define PIN_MOTOR2     26   // M2 front right
#define PIN_MOTOR3     27   // M3 rear  left
#define PIN_MOTOR4     14   // M4 front left
#define PIN_LED        2

// Battery voltage (optional). ADC1 pins only: 32..39. Use a divider!
#define VBAT_ENABLED   0
#define PIN_VBAT       35
#define VBAT_DIVIDER   2.0f   // Vbat = Vpin * VBAT_DIVIDER
#define VBAT_COMP      0      // 1 = scale motor output by VBAT_NOMINAL / vbat
#define VBAT_NOMINAL   3.9f

// ---------------- IMU ----------------
#define IMU_I2C_HZ     400000
// Map raw chip axes -> body frame (X forward, Y left, Z up).
// CURRENT MOUNT: board flat, components facing UP, VCC/header side pointing to the FRONT.
//   (GY-521 silkscreen: chip +Y points toward the header/VCC side, chip +X points to the right.)
//   => body X (forward) = chip Y,  body Y (left) = -chip X,  body Z (up) = chip Z
// Other mounts (edit only these 3 lines):
//   chip X to front, Y to left (default flat) : X=(x)   Y=(y)   Z=(z)
//   header to the BACK                        : X=(-y)  Y=(x)   Z=(z)
//   header to the RIGHT                       : X=(-x)  Y=(-y)  Z=(z)
//   header to the LEFT                        : X=(x)   Y=(y)   Z=(z)  (same as default)
#define IMU_X(x,y,z)   (y)
#define IMU_Y(x,y,z)   (-(x))
#define IMU_Z(x,y,z)   (z)

// ---------------- Control ----------------
#define PWM_FREQ       20000
#define PWM_BITS       10
#define GYRO_LPF_HZ    100.0f
#define DTERM_LPF_HZ   60.0f
#define FAILSAFE_MS    500
#define THROTTLE_CUT   0.03f   // below this (armed) motors stop, I-term resets
#define ITERM_LIMIT    300.0f
#define ITERM_MIN_THR  0.10f   // I-term only builds above this throttle (stops flips at take-off)
#define MOTOR_IDLE     0.04f   // min duty when motors run, so brushed motors do not stall at low throttle
#define MAX_LEVEL_RATE 400.0f  // deg/s limit from angle controller

// Betaflight PID scaling (so Configurator numbers feel familiar)
#define PTERM_SCALE    0.032029f
#define ITERM_SCALE    0.244381f
#define DTERM_SCALE    0.000529f

// Flip to -1.0f if yaw spins the wrong way (depends on your prop directions).
// Default assumes BF "props in": M1 CCW, M2 CW, M3 CW, M4 CCW.
#define YAW_MIX_SIGN   1.0f

// ---------------- Radio ----------------
#define ESPNOW_CHANNEL 1
#define ESPNOW_MAGIC   0xFC5A
#define CRTP_UDP_PORT  2390
// App input scaling/inversion is now set from the web page (http://192.168.43.42) -> "App input scale".
// Negative value = inverted. Default 1.0 = app sends degrees (roll/pitch) and deg/s (yaw).
#define CRTP_YAW_MAX    200.0f // deg/s that maps to full yaw stick
#define AP_PASSWORD    "12345678"

// ---------------- MSP / Betaflight Configurator ----------------
#define MSP_API_MAJOR  1
#define MSP_API_MINOR  42      // 1.42 = Betaflight 4.2 (same as ESPFC uses with Configurator 10.10)

// ---------------- Limits ----------------
#define MAX_RC_CH        8
#define MAX_MODE_RANGES  20
#define SETTINGS_MAGIC   0xE5F10003u   // changed: old saved settings (and old acc offsets) are wiped on first boot

enum { BOX_ARM = 0, BOX_ANGLE = 1 };

struct ModeRange {
  uint8_t id;      // box id (0 = ARM, 1 = ANGLE)
  uint8_t aux;     // aux channel index (0 = AUX1 = channel 5)
  uint8_t start;   // (pwm-900)/25
  uint8_t end;
};

struct Settings {
  uint32_t magic;
  uint8_t  pid[5][3];       // ROLL, PITCH, YAW, LEVEL, MAG  -> P,I,D (BF style ints)
  uint8_t  rcRate[3];       // roll, pitch, yaw
  uint8_t  rcExpo[3];
  uint8_t  rate[3];
  uint8_t  tpaRate;         // %
  uint16_t tpaBreak;        // pwm
  uint8_t  thrMid;
  uint8_t  thrExpo;
  uint8_t  levelAngle;      // max angle in ANGLE mode (deg)
  ModeRange modes[MAX_MODE_RANGES];
  float    accOffset[3];    // g
  float    crtpScale[3];    // ESP-Drone app: roll, pitch, yaw multiplier (negative = invert)
};
