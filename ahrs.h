#pragma once
#include <math.h>

// Mahony IMU filter (gyro + accel). Body frame: X forward, Y left, Z up.
class Mahony {
public:
  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
  float kp = 2.0f;

  // gyro in rad/s, accel in g
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n > 0.7f && n < 1.3f) {            // ignore accel during hard maneuvers
      float inv = 1.0f / n;
      ax *= inv; ay *= inv; az *= inv;
      float vx = 2.0f * (q1 * q3 - q0 * q2);
      float vy = 2.0f * (q0 * q1 + q2 * q3);
      float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
      float ex = ay * vz - az * vy;
      float ey = az * vx - ax * vz;
      float ez = ax * vy - ay * vx;
      gx += kp * ex;
      gy += kp * ey;
      gz += kp * ez;
    }
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += -qb * gx - qc * gy - q3 * gz;
    q1 +=  qa * gx + qc * gz - q3 * gy;
    q2 +=  qa * gy - qb * gz + q3 * gx;
    q3 +=  qa * gz + qb * gy - qc * gx;
    float r = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= r; q1 *= r; q2 *= r; q3 *= r;
  }

  // radians. roll + = right bank, pitchDown + = nose down, yawRight + = clockwise from above
  void euler(float& roll, float& pitchDown, float& yawRight) const {
    roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float s = 2.0f * (q0 * q2 - q3 * q1);
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    pitchDown = asinf(s);
    yawRight = -atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
  }
};
