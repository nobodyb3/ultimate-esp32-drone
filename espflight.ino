/*
  ESPFLIGHT - ESP32 WROOM brushed quadcopter flight controller (Arduino IDE)

  IMU      : MPU6050 / MPU6500 (I2C, auto-detected)
  Control  : 1) ESP-NOW transmitter (see espflight_tx sketch)
             2) ESP-Drone phone app (WiFi AP, UDP 2390, CRTP protocol)
  Config   : Betaflight Configurator over USB serial (MSP, 115200)

  Board    : "ESP32 Dev Module"   (esp32 core 2.x or 3.x by Espressif)
  Libraries: none extra (everything is part of the ESP32 core)

  !! Always test with PROPS OFF first. See README.md !!
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "config.h"
#include "fc.h"
#include "imu.h"
#include "ahrs.h"
#include "espnow_rx.h"
#include "crtp.h"
#include "msp.h"
#include "webui.h"

FcState fc;
Settings cfg;
CrtpSetpoint crtpSp;
volatile uint16_t espRc[MAX_RC_CH] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500};
volatile uint32_t espLastMs = 0;
volatile bool accCalRequested = false;
volatile uint32_t motorTestUntil = 0;
volatile uint16_t motorTestVal[4] = {1000, 1000, 1000, 1000};

static Imu imu;
static Mahony ahrs;
static MspServer msp;
static Preferences prefs;
static float gyroBias[3] = {0, 0, 0};

// Mixer rows = motors M1..M4 (Betaflight order), columns = roll, pitch, yaw
//   M1 rear-right  M2 front-right  M3 rear-left  M4 front-left
static const float MIX[4][3] = {
  {-1.f, +1.f, +1.f},
  {-1.f, -1.f, -1.f},
  {+1.f, +1.f, -1.f},
  {+1.f, -1.f, +1.f}
};

static const uint8_t motorPins[4] = {PIN_MOTOR1, PIN_MOTOR2, PIN_MOTOR3, PIN_MOTOR4};

// ============================================================
//  Settings (stored in NVS flash)
// ============================================================
static uint8_t rangeStep(int pwm) { return (uint8_t)((pwm - 900) / 25); }

void settingsDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = SETTINGS_MAGIC;
  const uint8_t pid[5][3] = {{50, 60, 30}, {52, 62, 32}, {50, 60, 0}, {50, 50, 75}, {40, 0, 0}};
  memcpy(cfg.pid, pid, sizeof(pid));
  for (int i = 0; i < 3; i++) { cfg.rcRate[i] = 100; cfg.rcExpo[i] = 10; cfg.rate[i] = 40; }
  cfg.rate[2] = 50;
  cfg.tpaRate = 0;
  cfg.tpaBreak = 1250;
  cfg.thrMid = 50;
  cfg.thrExpo = 0;
  cfg.levelAngle = 45;
  for (int i = 0; i < 3; i++) cfg.crtpScale[i] = 1.0f;
  cfg.modes[0] = {BOX_ARM, 0, rangeStep(1700), rangeStep(2100)};     // AUX1 high = ARM
  cfg.modes[1] = {BOX_ANGLE, 1, rangeStep(1300), rangeStep(2100)};   // AUX2 high = ANGLE
}

static void settingsLoad() {
  settingsDefaults();
  Settings tmp;
  prefs.begin("espflight", true);
  size_t n = prefs.getBytesLength("cfg");
  if (n == sizeof(tmp)) {
    prefs.getBytes("cfg", &tmp, n);
    if (tmp.magic == SETTINGS_MAGIC) cfg = tmp;
  }
  prefs.end();
}

void settingsSave() {
  prefs.begin("espflight", false);
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
}

// ============================================================
//  Motors (LEDC PWM, works on core 2.x and 3.x)
// ============================================================
static void motorsInit() {
  for (int i = 0; i < 4; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(motorPins[i], PWM_FREQ, PWM_BITS);
    ledcWrite(motorPins[i], 0);
#else
    ledcSetup(i, PWM_FREQ, PWM_BITS);
    ledcAttachPin(motorPins[i], i);
    ledcWrite(i, 0);
#endif
  }
}

static void motorWrite(uint8_t i, float v) {
  uint32_t duty = (uint32_t)(clampf(v, 0.0f, 1.0f) * (float)((1 << PWM_BITS) - 1));
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(motorPins[i], duty);
#else
  ledcWrite(i, duty);
#endif
}

static void motorsOff() {
  for (int i = 0; i < 4; i++) { motorWrite(i, 0); fc.motor[i] = 0; }
}

// ============================================================
//  Calibration
// ============================================================
static void calibrateGyro() {
  for (int attempt = 0; attempt < 5; attempt++) {
    double sum[3] = {0, 0, 0}, sq[3] = {0, 0, 0};
    int n = 0;
    for (int i = 0; i < 800; i++) {
      ImuSample s;
      if (imu.read(s)) {
        for (int k = 0; k < 3; k++) { sum[k] += s.g[k]; sq[k] += (double)s.g[k] * s.g[k]; }
        n++;
      }
      delay(1);
    }
    if (n < 400) continue;
    bool still = true;
    for (int k = 0; k < 3; k++) {
      double mean = sum[k] / n;
      double var = sq[k] / n - mean * mean;
      gyroBias[k] = (float)mean;
      if (var > 4.0) still = false;      // std-dev > 2 deg/s : board was moving, retry
    }
    if (still) break;
  }
}

static void calibrateAcc() {
  double sum[3] = {0, 0, 0};
  int n = 0;
  fc.calibrating = true;
  for (int i = 0; i < 500; i++) {
    ImuSample s;
    if (imu.read(s)) { for (int k = 0; k < 3; k++) sum[k] += s.a[k]; n++; }
    delay(2);
  }
  if (n > 250) {
    cfg.accOffset[0] = (float)(sum[0] / n);
    cfg.accOffset[1] = (float)(sum[1] / n);
    cfg.accOffset[2] = (float)(sum[2] / n) - 1.0f;
    settingsSave();
  }
  fc.calibrating = false;
}

// ============================================================
//  RC input handling
// ============================================================
static float dz(float v, float d) { return fabsf(v) < d ? 0.0f : v; }

static bool modeActive(uint8_t boxId, const uint16_t* ch) {
  for (int i = 0; i < MAX_MODE_RANGES; i++) {
    const ModeRange& m = cfg.modes[i];
    if (m.id != boxId || m.end <= m.start || m.aux >= MAX_RC_CH - 4) continue;
    uint16_t v = ch[4 + m.aux];
    if (v >= 900 + 25 * m.start && v < 900 + 25 * m.end) return true;
  }
  return false;
}

static void updateInputs(Inputs& in) {
  uint32_t now = millis();
  uint32_t eLast = espLastMs;
  uint32_t cLast = crtpSp.lastMs;
  uint16_t ch[MAX_RC_CH] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500};

  in.link = false; in.armReq = false; in.angle = false;
  in.stick[0] = in.stick[1] = in.stick[2] = in.stick[3] = 0;

  if (eLast && (now - eLast) < FAILSAFE_MS) {
    // ---- ESP-NOW transmitter ----
    for (int i = 0; i < MAX_RC_CH; i++) ch[i] = espRc[i];
    fc.rcSource = 1;
    in.link = true;
    in.stick[0] = dz((ch[0] - 1500) / 500.0f, 0.01f);
    in.stick[1] = dz((ch[1] - 1500) / 500.0f, 0.01f);
    in.stick[2] = dz((ch[3] - 1500) / 500.0f, 0.01f);
    in.stick[3] = clampf((ch[2] - 1000) / 1000.0f, 0.0f, 1.0f);
    in.armReq = modeActive(BOX_ARM, ch);
    in.angle = modeActive(BOX_ANGLE, ch);
  } else if (cLast && (now - cLast) < FAILSAFE_MS) {
    // ---- ESP-Drone app (CRTP) ----
    fc.rcSource = 2;
    in.link = true;
    float lim = (float)cfg.levelAngle;
    float r = crtpSp.roll * cfg.crtpScale[0];
    float p = crtpSp.pitch * cfg.crtpScale[1];
    float y = crtpSp.yaw * cfg.crtpScale[2];
    in.stick[0] = clampf(r / lim, -1.0f, 1.0f);
    in.stick[1] = clampf(p / lim, -1.0f, 1.0f);
    in.stick[2] = clampf(y / CRTP_YAW_MAX, -1.0f, 1.0f);
    float thr = (float)crtpSp.thrust / 65535.0f;
    in.stick[3] = crtpSp.unlocked ? thr : 0.0f;
    in.armReq = crtpSp.unlocked;
    in.angle = true;
    ch[0] = (uint16_t)(1500 + in.stick[0] * 500);
    ch[1] = (uint16_t)(1500 + in.stick[1] * 500);
    ch[2] = (uint16_t)(1000 + in.stick[3] * 1000);
    ch[3] = (uint16_t)(1500 + in.stick[2] * 500);
    ch[4] = in.armReq ? 2000 : 1000;
    ch[5] = 2000;
  } else {
    fc.rcSource = 0;
    crtpSp.unlocked = false;
  }
  for (int i = 0; i < MAX_RC_CH; i++) fc.rc[i] = ch[i];
}

// Betaflight-style "actual/BF" rate curve. stick -1..1 -> deg/s
static float applyRate(float stick, int axis) {
  float rcRate = cfg.rcRate[axis] / 100.0f;
  float expo = cfg.rcExpo[axis] / 100.0f;
  float rate = cfg.rate[axis] / 100.0f;
  float a = fabsf(stick);
  float c = stick * a * a * a * expo + stick * (1.0f - expo);
  if (rcRate > 2.0f) rcRate += 14.54f * (rcRate - 2.0f);
  float ang = 200.0f * rcRate * c;
  float sf = 1.0f / clampf(1.0f - a * rate, 0.01f, 1.0f);
  return ang * sf;
}

// ============================================================
//  Flight controller task (1 kHz, core 1)
// ============================================================
static void fcTask(void* arg) {
  bool ok = imu.begin();
  if (ok) {
    fc.calibrating = true;
    calibrateGyro();
    fc.calibrating = false;
    fc.gyroOk = true;
  }

  TickType_t last = xTaskGetTickCount();
  uint32_t tPrev = micros();
  uint8_t imuFail = 0;
  float gF[3] = {0, 0, 0}, prevG[3] = {0, 0, 0}, dF[3] = {0, 0, 0}, iTerm[3] = {0, 0, 0};
  bool sat = false;
  bool armSwLatched = true;

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    uint32_t t0 = micros();
    float dt = (t0 - tPrev) * 1e-6f;
    tPrev = t0;
    dt = clampf(dt, 0.0004f, 0.004f);
    fc.cycleUs = (uint16_t)(dt * 1e6f);

    if (!ok) { fc.armed = false; fc.gyroOk = false; motorsOff(); continue; }

    if (accCalRequested) {
      accCalRequested = false;
      if (!fc.armed) calibrateAcc();
      tPrev = micros();
      continue;
    }

    // ---------- sensors ----------
    ImuSample s;
    if (!imu.read(s)) {
      fc.i2cErrors = (uint16_t)imu.errors;
      if (++imuFail > 10) { fc.armed = false; fc.gyroOk = false; motorsOff(); }
      continue;
    }
    imuFail = 0;
    fc.gyroOk = true;

    float gx = s.g[0] - gyroBias[0];
    float gy = s.g[1] - gyroBias[1];
    float gz = s.g[2] - gyroBias[2];
    float ax = s.a[0] - cfg.accOffset[0];
    float ay = s.a[1] - cfg.accOffset[1];
    float az = s.a[2] - cfg.accOffset[2];

    ahrs.update(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az, dt);
    float r, p, yw;
    ahrs.euler(r, p, yw);
    fc.roll = r * RAD_TO_DEG;
    fc.pitchDown = p * RAD_TO_DEG;
    fc.yaw = yw * RAD_TO_DEG;

    // controller axes: roll + = right, pitch + = nose down, yaw + = right
    float rate[3] = {gx, gy, -gz};
    for (int k = 0; k < 3; k++) fc.gyro[k] = rate[k];
    fc.acc[0] = ax; fc.acc[1] = ay; fc.acc[2] = az;

    float aG = dt / (dt + 1.0f / (TWO_PI * GYRO_LPF_HZ));
    float aD = dt / (dt + 1.0f / (TWO_PI * DTERM_LPF_HZ));
    for (int k = 0; k < 3; k++) gF[k] += aG * (rate[k] - gF[k]);

    // ---------- inputs & arming ----------
    Inputs in;
    updateInputs(in);
    float thr = in.stick[3];
    fc.angleMode = in.angle;

    if (!in.link) armSwLatched = true;
    else if (!in.armReq || fc.rcSource == 2) armSwLatched = false;

    float tilt = fmaxf(fabsf(fc.roll), fabsf(fc.pitchDown));
    if (!fc.armed) {
      uint32_t f = 0;
      if (!fc.gyroOk) f |= ARMDIS_NO_GYRO;
      if (!in.link) f |= ARMDIS_RX_FAILSAFE;
      if (thr > 0.05f) f |= ARMDIS_THROTTLE;
      if (tilt > 25.0f) f |= ARMDIS_ANGLE;
      if (armSwLatched && in.armReq) f |= ARMDIS_ARMSWITCH;
      if (fc.calibrating) f |= ARMDIS_CALIB;
      fc.armingDisable = f;
      if (in.armReq && f == 0) {
        fc.armed = true;
        for (int k = 0; k < 3; k++) iTerm[k] = 0;
      }
    } else {
      fc.armingDisable = 0;
      if (!in.armReq || !in.link) fc.armed = false;
    }

    // ---------- setpoints ----------
    float sp[3];
    if (in.angle) {
      float lim = (float)cfg.levelAngle;
      float gain = cfg.pid[3][0] * 0.1f;
      sp[0] = clampf((in.stick[0] * lim - fc.roll) * gain, -MAX_LEVEL_RATE, MAX_LEVEL_RATE);
      sp[1] = clampf((in.stick[1] * lim - fc.pitchDown) * gain, -MAX_LEVEL_RATE, MAX_LEVEL_RATE);
      sp[2] = applyRate(in.stick[2], 2);
    } else {
      sp[0] = applyRate(in.stick[0], 0);
      sp[1] = applyRate(in.stick[1], 1);
      sp[2] = applyRate(in.stick[2], 2);
    }

    // ---------- PID ----------
    bool active = fc.armed && thr > THROTTLE_CUT;
    float tpa = 1.0f;
    if (cfg.tpaRate > 0) {
      float bp = (cfg.tpaBreak - 1000) / 1000.0f;
      if (thr > bp) tpa = 1.0f - (cfg.tpaRate / 100.0f) * (thr - bp) / (1.0f - bp);
    }
    float out[3];
    for (int k = 0; k < 3; k++) {
      float err = sp[k] - gF[k];
      float d = -(gF[k] - prevG[k]) / dt;
      prevG[k] = gF[k];
      dF[k] += aD * (d - dF[k]);
      if (active) {
        if (!sat && thr > ITERM_MIN_THR) iTerm[k] = clampf(iTerm[k] + ITERM_SCALE * cfg.pid[k][1] * err * dt, -ITERM_LIMIT, ITERM_LIMIT);
      } else {
        iTerm[k] = 0;
      }
      float pt = PTERM_SCALE * cfg.pid[k][0] * err;
      float dt_ = DTERM_SCALE * cfg.pid[k][2] * dF[k];
      out[k] = ((pt + dt_) * tpa + iTerm[k]) * 0.001f;
    }

    // ---------- mixer ----------
    float m[4] = {0, 0, 0, 0};
    if (active) {
      float mix[4];
      float mn = 1e9f, mx = -1e9f;
      for (int i = 0; i < 4; i++) {
        mix[i] = out[0] * MIX[i][0] + out[1] * MIX[i][1] + YAW_MIX_SIGN * out[2] * MIX[i][2];
        if (mix[i] < mn) mn = mix[i];
        if (mix[i] > mx) mx = mix[i];
      }
      float range = mx - mn;
      if (range > 1.0f) {
        float kk = 1.0f / range;
        for (int i = 0; i < 4; i++) mix[i] *= kk;
        mn *= kk; mx *= kk;
        sat = true;
      } else {
        sat = false;
      }
      float thrAdj = clampf(thr, -mn, 1.0f - mx);
      float vc = 1.0f;
#if VBAT_COMP
      if (fc.vbat > 2.5f) vc = clampf(VBAT_NOMINAL / fc.vbat, 0.85f, 1.3f);
#endif
      for (int i = 0; i < 4; i++) {
        float v = clampf((thrAdj + mix[i]) * vc, 0.0f, 1.0f);
        m[i] = MOTOR_IDLE + v * (1.0f - MOTOR_IDLE);      // keep brushed motors above stall duty
      }
    } else {
      sat = false;
    }

    // ---------- motor test (Configurator, disarmed only) ----------
    if (!fc.armed && (int32_t)(motorTestUntil - millis()) > 0) {
      for (int i = 0; i < 4; i++) m[i] = clampf((motorTestVal[i] - 1000) / 1000.0f, 0.0f, 1.0f);
    }

    for (int i = 0; i < 4; i++) { motorWrite(i, m[i]); fc.motor[i] = m[i]; }

    uint32_t used = micros() - t0;
    fc.load = (uint8_t)(used > 1000 ? 100 : used / 10);
  }
}

// ============================================================
//  Arduino entry points
// ============================================================
void setup() {
  Serial.begin(115200);          // MSP lives here - do NOT print debug text to Serial
  pinMode(PIN_LED, OUTPUT);
  motorsInit();                  // outputs low immediately
  settingsLoad();

  // WiFi access point for the ESP-Drone app; ESP-NOW rides on the same channel
  char ssid[32];
  uint64_t id = ESP.getEfuseMac();
  snprintf(ssid, sizeof(ssid), "ESP-DRONE_%02X%02X%02X",
           (uint8_t)(id >> 24), (uint8_t)(id >> 32), (uint8_t)(id >> 40));
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 43, 42), IPAddress(192, 168, 43, 42), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, AP_PASSWORD, ESPNOW_CHANNEL, 0, 2);
  esp_wifi_set_ps(WIFI_PS_NONE);

  espnowBegin();
  crtpBegin();
  webBegin();

  xTaskCreatePinnedToCore(fcTask, "fc", 8192, NULL, 5, NULL, 1);
}

void loop() {
  msp.poll();
  webPoll();

  uint32_t now = millis();

  // LED: solid = armed | fast blink = calibrating / no gyro | slow blink = no radio | double flash = ready
  bool led;
  if (fc.armed) led = true;
  else if (fc.calibrating || !fc.gyroOk) led = (now / 100) & 1;
  else if (fc.rcSource == 0) led = (now / 500) & 1;
  else { uint32_t ph = now % 1000; led = (ph < 80) || (ph > 200 && ph < 280); }
  digitalWrite(PIN_LED, led);

#if VBAT_ENABLED
  static uint32_t tBat = 0;
  if (now - tBat >= 100) {
    tBat = now;
    fc.vbat = analogReadMilliVolts(PIN_VBAT) * 0.001f * VBAT_DIVIDER;
  }
#endif

  delay(1);
}
