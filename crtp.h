#pragma once
#include <Arduino.h>
#include <AsyncUDP.h>
#include "fc.h"

// ESP-Drone app protocol: Crazyflie CRTP packets over UDP.
//  Drone = WiFi AP 192.168.43.42, UDP port 2390. App sends from port 2399.
//  Packet = [header][payload...][checksum]   checksum = sum of all previous bytes
//  header = (port << 4) | 0x0C | channel
//  Port 3 ch 0 : legacy commander  { float roll, float pitch, float yaw, uint16 thrust }  (little endian)
//  Port 7 ch 0 : generic setpoint, type 0 = stop
//  Port 2 / 5  : param / log TOC  (best-effort empty reply so the app does not hang)

static AsyncUDP crtpUdp;

static void crtpSend(AsyncUDPPacket& p, uint8_t port, uint8_t ch, const uint8_t* pl, uint8_t n) {
  uint8_t buf[40];
  if (n > 36) n = 36;
  buf[0] = (uint8_t)((port << 4) | 0x0C | (ch & 3));
  memcpy(buf + 1, pl, n);
  uint8_t s = 0;
  for (uint8_t i = 0; i < n + 1; i++) s += buf[i];
  buf[n + 1] = s;
  p.write(buf, n + 2);
}

static void crtpHandle(AsyncUDPPacket& p) {
  const uint8_t* d = p.data();
  size_t n = p.length();
  if (n < 2) return;
  uint8_t sum = 0;
  for (size_t i = 0; i < n - 1; i++) sum += d[i];
  if (sum != d[n - 1]) return;

  uint8_t port = d[0] >> 4;
  uint8_t ch = d[0] & 3;
  const uint8_t* pl = d + 1;
  size_t pn = n - 2;

  if (port == 3 && ch == 0 && pn >= 14) {
    float r, pi, y;
    uint16_t t;
    memcpy(&r, pl, 4);
    memcpy(&pi, pl + 4, 4);
    memcpy(&y, pl + 8, 4);
    memcpy(&t, pl + 12, 2);
    crtpSp.roll = r;
    crtpSp.pitch = pi;
    crtpSp.yaw = y;
    crtpSp.thrust = t;
    if (t == 0) crtpSp.unlocked = true;     // Crazyflie style "thrust lock": send thrust 0 first
    crtpSp.lastMs = millis();
  } else if (port == 7 && pn >= 1 && pl[0] == 0) {
    crtpSp.thrust = 0;                      // stop setpoint
    crtpSp.unlocked = false;
    crtpSp.lastMs = millis();
  } else if ((port == 2 || port == 5) && ch == 0 && pn >= 1) {
    uint8_t cmd = pl[0];
    uint8_t rep[8];
    uint8_t k = 0;
    if (cmd == 1) {          // TOC info v1 : count(u8) crc(u32)
      rep[k++] = 1; rep[k++] = 0;
      rep[k++] = 0; rep[k++] = 0; rep[k++] = 0; rep[k++] = 0;
      crtpSend(p, port, 0, rep, k);
    } else if (cmd == 3) {   // TOC info v2 : count(u16) crc(u32)
      rep[k++] = 3; rep[k++] = 0; rep[k++] = 0;
      rep[k++] = 0; rep[k++] = 0; rep[k++] = 0; rep[k++] = 0;
      crtpSend(p, port, 0, rep, k);
    }
  }
}

static void crtpBegin() {
  if (crtpUdp.listen(CRTP_UDP_PORT)) {
    crtpUdp.onPacket([](AsyncUDPPacket& pkt) { crtpHandle(pkt); });
  }
}
