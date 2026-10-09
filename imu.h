#pragma once
#include <Wire.h>
#include "config.h"

// Output: gyro in deg/s, accel in g, both already mapped to body frame (X fwd, Y left, Z up)
struct ImuSample {
  float g[3];
  float a[3];
};

class Imu {
public:
  uint8_t addr = 0x68;
  uint8_t whoami = 0;
  bool is6500 = false;
  uint32_t errors = 0;

  bool begin() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(IMU_I2C_HZ);
    Wire.setTimeOut(5);

    bool found = false;
    for (uint8_t i = 0; i < 2 && !found; i++) {
      uint8_t a = 0x68 + i;
      uint8_t id = 0;
      if (rd(a, 0x75, &id, 1) && id != 0x00 && id != 0xFF) {
        addr = a; whoami = id; found = true;
      }
    }
    if (!found) return false;

    // 0x68 = MPU6050, 0x70 = MPU6500, 0x71 = MPU9250, 0x73 = MPU9255 (6500 family)
    is6500 = (whoami == 0x70 || whoami == 0x71 || whoami == 0x73 || whoami == 0x74);

    wr(0x6B, 0x80); delay(100);          // reset
    wr(0x68, 0x07); delay(100);          // reset signal paths
    wr(0x6B, 0x01); delay(10);           // clock = PLL gyro X
    wr(0x6C, 0x00);                      // all axes on
    wr(0x1A, 0x01);                      // gyro DLPF ~184 Hz, 1 kHz internal rate
    wr(0x1B, 0x18);                      // gyro  +-2000 dps
    wr(0x1C, 0x10);                      // accel +-8 g
    if (is6500) wr(0x1D, 0x02);          // accel DLPF ~92 Hz (MPU6500 only)
    wr(0x19, 0x00);                      // sample rate divider 0 -> 1 kHz
    delay(50);
    return true;
  }

  const char* name() const {
    switch (whoami) {
      case 0x68: return "MPU6050";
      case 0x70: return "MPU6500";
      case 0x71: return "MPU9250";
      case 0x73: return "MPU9255";
      default:   return "MPU-compatible";
    }
  }

  bool read(ImuSample& s) {
    uint8_t b[14];
    if (!rd(addr, 0x3B, b, 14)) { errors++; return false; }
    int16_t ax = (int16_t)((b[0] << 8) | b[1]);
    int16_t ay = (int16_t)((b[2] << 8) | b[3]);
    int16_t az = (int16_t)((b[4] << 8) | b[5]);
    int16_t gx = (int16_t)((b[8] << 8) | b[9]);
    int16_t gy = (int16_t)((b[10] << 8) | b[11]);
    int16_t gz = (int16_t)((b[12] << 8) | b[13]);
    float fax = ax / 4096.0f, fay = ay / 4096.0f, faz = az / 4096.0f;
    float fgx = gx / 16.4f,   fgy = gy / 16.4f,   fgz = gz / 16.4f;
    s.a[0] = IMU_X(fax, fay, faz);
    s.a[1] = IMU_Y(fax, fay, faz);
    s.a[2] = IMU_Z(fax, fay, faz);
    s.g[0] = IMU_X(fgx, fgy, fgz);
    s.g[1] = IMU_Y(fgx, fgy, fgz);
    s.g[2] = IMU_Z(fgx, fgy, fgz);
    return true;
  }

private:
  bool wr(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }
  bool rd(uint8_t a, uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(a);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint16_t)a, (size_t)n, true) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
  }
};
