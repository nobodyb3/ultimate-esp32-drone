#pragma once
#include <Arduino.h>
#include "fc.h"

// MSP v1 + v2 server over USB serial (115200) so Betaflight Configurator can connect.
// Supported: Setup (attitude/sensors), Receiver, Motors (test), Modes (ARM/ANGLE), PID + Rates basics.

enum : uint16_t {
  MSP_API_VERSION = 1, MSP_FC_VARIANT = 2, MSP_FC_VERSION = 3, MSP_BOARD_INFO = 4, MSP_BUILD_INFO = 5,
  MSP_NAME = 10,
  MSP_MODE_RANGES = 34, MSP_SET_MODE_RANGE = 35, MSP_FEATURE_CONFIG = 36,
  MSP_BOARD_ALIGNMENT = 38, MSP_MIXER_CONFIG = 42, MSP_RX_CONFIG = 44, MSP_RSSI_CONFIG = 50,
  MSP_ARMING_CONFIG = 61,
  MSP_RX_MAP = 64, MSP_SET_RX_MAP = 65, MSP_REBOOT = 68,
  MSP_STATUS = 101, MSP_RAW_IMU = 102, MSP_MOTOR = 104, MSP_RC = 105, MSP_ATTITUDE = 108,
  MSP_ALTITUDE = 109, MSP_ANALOG = 110, MSP_RC_TUNING = 111, MSP_PID = 112,
  MSP_ADVANCED_CONFIG = 90, MSP_FILTER_CONFIG = 92, MSP_PID_ADVANCED = 94, MSP_SET_PID_ADVANCED = 95,
  MSP_SENSOR_CONFIG = 96,
  MSP_BOXNAMES = 116, MSP_BOXIDS = 119, MSP_MOTOR_3D_CONFIG = 124, MSP_MOTOR_CONFIG = 131,
  MSP_STATUS_EX = 150, MSP_UID = 160, MSP_MODE_RANGES_EXTRA = 238,
  MSP_SET_PID = 202, MSP_SET_RC_TUNING = 204, MSP_ACC_CALIBRATION = 205, MSP_MAG_CALIBRATION = 206,
  MSP_RESET_CONF = 208, MSP_SET_MOTOR = 214, MSP_ACC_TRIM = 240, MSP_EEPROM_WRITE = 250
};

struct MspBuf {
  uint8_t d[160];
  uint16_t n = 0;
  void u8(uint8_t v) { if (n < sizeof(d)) d[n++] = v; }
  void u16(uint16_t v) { u8(v & 0xFF); u8(v >> 8); }
  void u32(uint32_t v) { u16(v & 0xFFFF); u16(v >> 16); }
  void s16(int16_t v) { u16((uint16_t)v); }
  void str(const char* s) { while (*s) u8((uint8_t)*s++); }
  void zeros(uint16_t k) { while (k--) u8(0); }
};

class MspServer {
public:
  void poll() {
    while (Serial.available()) feed((uint8_t)Serial.read());
    if (_rebootAt && millis() > _rebootAt) ESP.restart();
  }

private:
  enum St { IDLE, GOT_DOLLAR, GOT_M, GOT_X, V1_SIZE, V1_CMD, V1_DATA, V1_CRC,
            V2_FLAG, V2_CL, V2_CH, V2_SL, V2_SH, V2_DATA, V2_CRC };
  St _st = IDLE;
  uint8_t _buf[256];
  uint16_t _cmd = 0, _size = 0, _idx = 0;
  uint8_t _crc = 0;
  bool _v2 = false;
  uint32_t _rebootAt = 0;

  static uint8_t crc8(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
  }

  void feed(uint8_t c) {
    switch (_st) {
      case IDLE: if (c == '$') _st = GOT_DOLLAR; break;
      case GOT_DOLLAR:
        if (c == 'M') { _v2 = false; _st = GOT_M; }
        else if (c == 'X') { _v2 = true; _st = GOT_X; }
        else _st = IDLE;
        break;
      case GOT_M: _st = (c == '<') ? V1_SIZE : IDLE; break;
      case GOT_X: _st = (c == '<') ? V2_FLAG : IDLE; break;
      case V1_SIZE: _size = c; _crc = c; _st = V1_CMD; break;
      case V1_CMD:
        _cmd = c; _crc ^= c; _idx = 0;
        _st = _size ? V1_DATA : V1_CRC;
        break;
      case V1_DATA:
        _buf[_idx++] = c; _crc ^= c;
        if (_idx >= _size) _st = V1_CRC;
        break;
      case V1_CRC:
        if (c == _crc) handle();
        _st = IDLE;
        break;
      case V2_FLAG: _crc = crc8(0, c); _st = V2_CL; break;
      case V2_CL: _cmd = c; _crc = crc8(_crc, c); _st = V2_CH; break;
      case V2_CH: _cmd |= (uint16_t)c << 8; _crc = crc8(_crc, c); _st = V2_SL; break;
      case V2_SL: _size = c; _crc = crc8(_crc, c); _st = V2_SH; break;
      case V2_SH:
        _size |= (uint16_t)c << 8; _crc = crc8(_crc, c); _idx = 0;
        if (_size > sizeof(_buf)) _st = IDLE;
        else _st = _size ? V2_DATA : V2_CRC;
        break;
      case V2_DATA:
        _buf[_idx++] = c; _crc = crc8(_crc, c);
        if (_idx >= _size) _st = V2_CRC;
        break;
      case V2_CRC:
        if (c == _crc) handle();
        _st = IDLE;
        break;
    }
  }

  void handle() {
    MspBuf r;
    bool ok = process(_cmd, _buf, _size, r);
    send(_cmd, r, ok);
  }

  void send(uint16_t cmd, MspBuf& r, bool ok) {
    uint8_t dir = ok ? '>' : '!';
    if (!ok) r.n = 0;
    if (!_v2) {
      uint8_t hdr[5] = {'$', 'M', dir, (uint8_t)r.n, (uint8_t)cmd};
      uint8_t crc = (uint8_t)r.n ^ (uint8_t)cmd;
      for (uint16_t i = 0; i < r.n; i++) crc ^= r.d[i];
      Serial.write(hdr, 5);
      if (r.n) Serial.write(r.d, r.n);
      Serial.write(crc);
    } else {
      uint8_t hdr[8] = {'$', 'X', dir, 0, (uint8_t)(cmd & 0xFF), (uint8_t)(cmd >> 8),
                        (uint8_t)(r.n & 0xFF), (uint8_t)(r.n >> 8)};
      uint8_t crc = 0;
      for (int i = 3; i < 8; i++) crc = crc8(crc, hdr[i]);
      for (uint16_t i = 0; i < r.n; i++) crc = crc8(crc, r.d[i]);
      Serial.write(hdr, 8);
      if (r.n) Serial.write(r.d, r.n);
      Serial.write(crc);
    }
  }

  void statusPayload(MspBuf& r, bool ex) {
    r.u16(fc.cycleUs);
    r.u16(fc.i2cErrors);
    r.u16(fc.gyroOk ? ((1 << 0) | (1 << 5)) : 0);            // ACC | GYRO
    uint32_t modes = 0;
    if (fc.armed) modes |= (1u << BOX_ARM);
    if (fc.angleMode) modes |= (1u << BOX_ANGLE);
    r.u32(modes);
    r.u8(0);                                                  // pid profile
    r.u16(fc.load);
    if (ex) { r.u8(1); r.u8(0); } else { r.u16(0); }
    r.u8(0);                                                  // extra flight-mode bytes
    r.u8(ARMDIS_COUNT);
    r.u32(fc.armingDisable);
    r.u8(0);                                                  // config state flags
  }

  static bool isSetCmd(uint16_t c) {
    if (c >= 200 && c < 250) return true;
    switch (c) {
      case 33: case 37: case 39: case 41: case 43: case 45: case 47: case 49: case 51: case 53:
      case 55: case 57: case 60: case 62: case 91: case 93: case 97: case 99: case 239:
        return true;
      default: return false;
    }
  }

  bool process(uint16_t cmd, const uint8_t* d, uint16_t n, MspBuf& r) {
    switch (cmd) {
      case MSP_API_VERSION: r.u8(0); r.u8(MSP_API_MAJOR); r.u8(MSP_API_MINOR); return true;
      case MSP_FC_VARIANT:  r.str("BTFL"); return true;
      case MSP_FC_VERSION:  r.u8(4); r.u8(2); r.u8(11); return true;   // BF 4.2.11 (API 1.42)
      case MSP_BUILD_INFO:  r.str(__DATE__); r.str(__TIME__); r.str("0000000"); return true;
      case MSP_BOARD_INFO:
        r.str("ESPF");            // board id
        r.u16(0);                 // hw revision
        r.u8(0);                  // FC only
        r.u8(0);                  // capabilities
        r.u8(5);  r.str("ESP32"); // target name
        r.u8(9);  r.str(FW_NAME_STR);   // board name
        r.u8(3);  r.str("ESP");   // manufacturer
        r.zeros(32);              // signature
        r.u8(0);                  // mcu type id
        r.u8(1);                  // configuration state: configured
        r.u16(1000);              // gyro sample rate
        r.u32(0);                 // configuration problems
        r.u8(0);                  // spi devices
        r.u8(1);                  // i2c devices
        return true;
      case MSP_UID: {
        uint64_t id = ESP.getEfuseMac();
        r.u32((uint32_t)id); r.u32((uint32_t)(id >> 32)); r.u32(0);
        return true;
      }
      case MSP_NAME: r.str(FW_NAME_STR); return true;
      case MSP_STATUS: statusPayload(r, false); return true;
      case MSP_STATUS_EX: statusPayload(r, true); return true;
      case MSP_RAW_IMU:
        for (int i = 0; i < 3; i++) r.s16((int16_t)lrintf(fc.acc[i] * 512.0f));
        for (int i = 0; i < 3; i++) r.s16((int16_t)lrintf(fc.gyro[i]));
        r.zeros(6);
        return true;
      case MSP_MOTOR:
        for (int i = 0; i < 8; i++) r.u16(i < 4 ? (uint16_t)(1000 + fc.motor[i] * 1000.0f) : 0);
        return true;
      case MSP_RC:
        for (int i = 0; i < MAX_RC_CH; i++) r.u16(fc.rc[i]);
        return true;
      case MSP_ATTITUDE: {
        r.s16((int16_t)lrintf(fc.roll * 10.0f));
        r.s16((int16_t)lrintf(fc.pitchDown * 10.0f));        // pitch shown in Configurator (flipped to match real nose up/down)
        float h = fc.yaw; if (h < 0) h += 360.0f;
        r.s16((int16_t)lrintf(h));
        return true;
      }
      case MSP_ALTITUDE: r.u32(0); r.u16(0); return true;
      case MSP_ANALOG: {
        uint16_t v10 = (uint16_t)clampf(fc.vbat * 10.0f, 0, 255);
        r.u8((uint8_t)v10); r.u16(0); r.u16(0); r.u16(0);
        r.u16((uint16_t)clampf(fc.vbat * 100.0f, 0, 65535));
        return true;
      }
      case MSP_RC_TUNING:
        r.u8(cfg.rcRate[0]); r.u8(cfg.rcExpo[0]);
        r.u8(cfg.rate[0]); r.u8(cfg.rate[1]); r.u8(cfg.rate[2]);
        r.u8(cfg.tpaRate); r.u8(cfg.thrMid); r.u8(cfg.thrExpo);
        r.u16(cfg.tpaBreak);
        r.u8(cfg.rcExpo[2]); r.u8(cfg.rcRate[2]); r.u8(cfg.rcRate[1]); r.u8(cfg.rcExpo[1]);
        r.u8(0); r.u8(100);                     // throttle limit type / percent
        r.u16(1998); r.u16(1998); r.u16(1998);  // rate limits
        r.u8(0);                                // rates type: Betaflight
        return true;
      case MSP_SET_RC_TUNING:
        if (n < 14) return false;
        cfg.rcRate[0] = d[0]; cfg.rcExpo[0] = d[1];
        cfg.rate[0] = d[2]; cfg.rate[1] = d[3]; cfg.rate[2] = d[4];
        cfg.tpaRate = d[5]; cfg.thrMid = d[6]; cfg.thrExpo = d[7];
        cfg.tpaBreak = d[8] | (d[9] << 8);
        cfg.rcExpo[2] = d[10]; cfg.rcRate[2] = d[11]; cfg.rcRate[1] = d[12]; cfg.rcExpo[1] = d[13];
        return true;
      case MSP_PID:
        for (int i = 0; i < 5; i++) { r.u8(cfg.pid[i][0]); r.u8(cfg.pid[i][1]); r.u8(cfg.pid[i][2]); }
        return true;
      case MSP_SET_PID:
        if (n < 15) return false;
        for (int i = 0; i < 5; i++) for (int k = 0; k < 3; k++) cfg.pid[i][k] = d[i * 3 + k];
        return true;
      case MSP_BOXNAMES: r.str("ARM;ANGLE;"); return true;
      case MSP_BOXIDS: r.u8(BOX_ARM); r.u8(BOX_ANGLE); return true;
      case MSP_MODE_RANGES:
        for (int i = 0; i < MAX_MODE_RANGES; i++) {
          r.u8(cfg.modes[i].id); r.u8(cfg.modes[i].aux); r.u8(cfg.modes[i].start); r.u8(cfg.modes[i].end);
        }
        return true;
      case MSP_SET_MODE_RANGE:
        if (n < 5 || d[0] >= MAX_MODE_RANGES) return false;
        cfg.modes[d[0]].id = d[1]; cfg.modes[d[0]].aux = d[2];
        cfg.modes[d[0]].start = d[3]; cfg.modes[d[0]].end = d[4];
        return true;
      case MSP_FEATURE_CONFIG: r.u32(0); return true;
      case MSP_RX_MAP:
        r.u8(0); r.u8(1); r.u8(3); r.u8(2); r.u8(4); r.u8(5); r.u8(6); r.u8(7);   // AETR
        return true;
      case MSP_SET_RX_MAP: return true;
      case MSP_ACC_TRIM: r.s16(0); r.s16(0); return true;
      case MSP_ACC_CALIBRATION: if (!fc.armed) accCalRequested = true; return true;
      case MSP_MAG_CALIBRATION: return true;
      case MSP_SET_MOTOR:
        if (n < 8 || fc.armed) return false;
        for (int i = 0; i < 4; i++) motorTestVal[i] = d[i * 2] | (d[i * 2 + 1] << 8);
        motorTestUntil = millis() + 500;       // keeps spinning only while Configurator keeps sending
        return true;
      case MSP_RESET_CONF: if (!fc.armed) settingsDefaults(); return true;
      case MSP_EEPROM_WRITE: if (!fc.armed) settingsSave(); return true;
      case MSP_REBOOT: _rebootAt = millis() + 300; return true;
      case MSP_MODE_RANGES_EXTRA:
        r.u8(MAX_MODE_RANGES);
        for (int i = 0; i < MAX_MODE_RANGES; i++) { r.u8(0); r.u8(0); }   // logic OR, not linked
        return true;
      case MSP_MIXER_CONFIG: r.u8(3); r.u8(0); r.zeros(4); return true;       // 3 = QUAD X
      case MSP_RX_CONFIG:
        r.u8(0); r.u16(1900); r.u16(1500); r.u16(1050); r.u8(0); r.u16(885); r.u16(2115);
        r.zeros(30);
        return true;
      case MSP_ARMING_CONFIG: r.u8(0); r.u8(0); r.u8(25); r.zeros(5); return true;
      case MSP_RSSI_CONFIG: r.u8(0); return true;
      case MSP_BOARD_ALIGNMENT: r.zeros(6); return true;
      case MSP_ADVANCED_CONFIG:
        r.u8(1); r.u8(1); r.u8(0);       // gyro denom, pid denom, unsynced pwm
        r.u8(4);                         // motor protocol: BRUSHED
        r.u16(PWM_FREQ);                 // motor pwm rate
        r.zeros(18);
        return true;
      case MSP_FILTER_CONFIG: r.zeros(48); return true;
      case MSP_PID_ADVANCED:
        r.zeros(15); r.u8(cfg.levelAngle);   // byte 15 = level angle limit
        r.zeros(40);
        return true;
      case MSP_SET_PID_ADVANCED:
        if (n > 15 && d[15] >= 10 && d[15] <= 80) cfg.levelAngle = d[15];
        return true;
      case MSP_SENSOR_CONFIG: r.zeros(8); return true;
      case MSP_MOTOR_3D_CONFIG: r.u16(1406); r.u16(1514); r.u16(1460); return true;
      case MSP_MOTOR_CONFIG:
        r.u16(1050); r.u16(2000); r.u16(1000);   // min throttle, max throttle, min command
        r.u8(4);                                 // motor count
        r.u8(14); r.u8(0); r.u8(0);              // pole count, dshot telemetry, esc sensor
        return true;
      case 32: case 40: case 46: case 48: case 52: case 54: case 56: case 80: case 125: case 126: case 184:
        r.zeros(32);                             // harmless blank configs for tabs we do not use
        return true;
      default:
        // Unknown SET_* commands: acknowledge so Configurator "Save" does not stall. Unknown reads: error.
        if (isSetCmd(cmd)) return true;
        return false;
    }
  }
};
