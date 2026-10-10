# ESPFLIGHT – complete guide

ESP32 (WROOM) flight controller firmware for a small **brushed-motor quadcopter**, written for the **Arduino IDE**.

| | |
|---|---|
| IMU | MPU6050 or MPU6500 (auto-detected, I2C) |
| Control | ESP-NOW transmitter **or** ESP-Drone phone app (they work without any mode switch) |
| Setup | Betaflight Configurator over USB (MSP) **and** a built-in web page |
| Frame | Quad-X, brushed motors, one MOSFET per motor |
| Loop | 1 kHz flight loop |

> **Safety first.** Always test with **propellers removed** until motor order, motor direction and tilt response are verified. Keep your hand on the ARM switch for the first flights. This firmware is hobby software with no warranty.

---

## 1. Folder contents

The sketch folder must be named **`espflight`** and keep every file together.

| File | What it does |
|---|---|
| `espflight.ino` | Main program: settings, motor PWM, calibration, RC input selection, arming, PID, mixer, 1 kHz flight task, LED, setup/loop |
| `config.h` | **All user options**: pins, IMU axis mapping, filters, limits, WiFi password, MSP version, settings structure |
| `fc.h` | Shared flight state (`fc`), arming-disable flags, shared declarations |
| `imu.h` | MPU6050 / MPU6500 driver (I2C), scaling and axis mapping |
| `ahrs.h` | Mahony filter that turns gyro + accelerometer into roll / pitch / yaw angles |
| `espnow_rx.h` | ESP-NOW receiver (checks magic number and CRC) |
| `crtp.h` | ESP-Drone app protocol (Crazyflie CRTP over UDP port 2390) |
| `msp.h` | MSP server for Betaflight Configurator (v1 and v2 frames) |
| `webui.h` | Built-in web page (live view, PID/rates editing, motor test) |
| `README.md` | This file |

The transmitter is a separate sketch: **`espflight_tx/espflight_tx.ino`** (second ESP32 + 2 joysticks + switches).

---

## 2. Arduino IDE setup

1. Arduino IDE 2.x. Boards Manager → install **"esp32 by Espressif Systems"** (core 2.x and 3.x both supported).
2. Open `espflight/espflight.ino`.
3. **Tools → Board → ESP32 Dev Module**. Other settings default. Upload speed 921600 is fine.
4. No extra libraries are needed (WiFi, WebServer, Preferences, AsyncUDP and ESP-NOW all come with the ESP32 core).
5. Upload. Do the same for `espflight_tx.ino` on the second ESP32.

Common compile problem: "No such file" → a header is missing from the sketch folder, or the folder is nested twice after unzipping. "exit status 1" is only a summary; the real reason is the first line containing `error:` above it.

---

## 3. Hardware and wiring

### Flight controller (ESP32 WROOM)

| Part | ESP32 pin | Notes |
|---|---|---|
| MPU SDA | GPIO 21 | 3.3 V I2C |
| MPU SCL | GPIO 22 | |
| Motor 1 (rear right) | GPIO 25 | via logic-level N-MOSFET |
| Motor 2 (front right) | GPIO 26 | |
| Motor 3 (rear left) | GPIO 27 | |
| Motor 4 (front left) | GPIO 14 | outputs a short pulse at ESP32 boot → add a gate pulldown (100 kΩ) or move to GPIO 33 in `config.h` |
| Status LED | GPIO 2 | on-board LED |
| Battery sense (optional) | GPIO 35 | resistor divider; set `VBAT_ENABLED 1` |

Each motor needs: logic-level N-MOSFET, 100 kΩ gate pulldown, flyback diode across the motor, and a common ground between the ESP32 and the battery.

### Motor layout and prop direction (Betaflight numbering)

```
        FRONT
   M4 (CCW)   M2 (CW)
        \     /
         \   /
         /   \
        /     \
   M3 (CW)    M1 (CCW)
        REAR
```

| Motor | Position | Spins |
|---|---|---|
| M1 | rear right | CCW |
| M2 | front right | CW |
| M3 | rear left | CW |
| M4 | front left | CCW |

If yaw reacts the wrong way, set `YAW_MIX_SIGN` to `-1.0f` in `config.h`.

### IMU mounting

`config.h` currently assumes: board flat, components up, **VCC/header side towards the front of the drone**. Mapping: forward = chip Y, left = −chip X, up = chip Z. Other mounts are listed in a comment next to `IMU_X / IMU_Y / IMU_Z`. After changing the mapping, calibrate the accelerometer again.

---

## 4. How the firmware works

### 4.1 Startup (`setup()`)
1. Serial 115200 (this port is used by MSP, so **nothing else may be printed to Serial**).
2. Motor PWM starts at 0 immediately.
3. Settings are loaded from flash (NVS). If there are none or the magic number differs, defaults are used.
4. WiFi access point is started: SSID `ESP-DRONE_xxxxxx` (last 3 bytes of the chip MAC), password `12345678`, channel 1, IP `192.168.43.42`.
5. ESP-NOW receiver, ESP-Drone (CRTP/UDP) listener and the web server start.
6. The 1 kHz flight task is created.
7. In the flight task: the IMU is detected and initialised, then the **gyro is calibrated** (about 1–2 s, drone must stay still; up to 5 attempts if it moves).

### 4.2 The 1 kHz flight loop (every cycle)
1. Read IMU (14-byte burst). If the IMU fails repeatedly, the drone disarms and motors stop.
2. Subtract gyro bias and accelerometer offsets.
3. Mahony filter → roll, pitch (nose down = +), yaw.
4. Low-pass filter the gyro (100 Hz).
5. Read the RC source, decide arm / angle request.
6. Arming logic (section 6).
7. Build setpoints (ANGLE or ACRO).
8. PID per axis (roll, pitch, yaw).
9. Mixer → four motor outputs → PWM.

### 4.3 IMU driver
- Detects address 0x68 / 0x69 and reads WHO_AM_I: `0x68` = MPU6050, `0x70` = MPU6500, `0x71/0x73` = MPU9250/9255.
- Gyro ±2000 °/s, accelerometer ±8 g, internal 1 kHz sample rate, digital low-pass filters on.
- I2C at 400 kHz.

### 4.4 Attitude (AHRS)
Mahony complementary filter with Kp = 2. Accelerometer correction is ignored when the measured acceleration is not near 1 g (hard manoeuvres). There is no magnetometer, so **yaw angle drifts**; this is fine because yaw is controlled as a rate.

### 4.5 Control modes
- **ANGLE (self-level):** stick gives a target angle (up to *max angle*, default 45°). The angle error × *level strength* becomes a rate setpoint (limited to 400 °/s).
- **ACRO (rate):** stick gives a rate setpoint through the Betaflight-style rate curve (RC rate, expo, super rate).
- **Yaw** is always rate controlled.
- The ESP-Drone app always flies in ANGLE.

### 4.6 PID
Betaflight-style integer P, I, D values (so numbers feel familiar).
- **P** = P × error
- **I** = accumulated error, limited, only builds above 10 % throttle and when the mixer is not saturated, reset when throttle is low or disarmed
- **D** = derivative of the **gyro** (not of the error), low-pass filtered at 60 Hz
- **TPA** reduces P and D at high throttle (optional, default 0 %)

Default PID (P / I / D): Roll 50/60/30, Pitch 52/62/32, Yaw 50/60/0, Level strength 50.

### 4.7 Mixer
Quad-X matrix with **desaturation**: if the mix asks for more than the motors can deliver, the roll/pitch/yaw part is scaled down so attitude control keeps priority. Throttle is shifted to keep all four motors inside 0–100 %. A minimum duty (`MOTOR_IDLE`, 4 %) keeps brushed motors from stalling. Below 3 % throttle motors are stopped.

### 4.8 Motors
LEDC PWM at 20 kHz, 10-bit resolution (works on both ESP32 core 2.x and 3.x).

### 4.9 Calibration
- **Gyro:** automatic at power-up (keep still).
- **Accelerometer:** from Configurator (Setup tab) or the web page, drone flat and still. Offsets are saved to flash.

### 4.10 Settings storage
Settings structure is stored with `Preferences` (namespace `espflight`, key `cfg`). Changing the `SETTINGS_MAGIC` number in `config.h` wipes old saved settings on the next boot.

---

## 5. Controlling the drone

There is **no mode switch**. The firmware picks the source automatically every cycle:

1. **ESP-NOW transmitter** – used if a packet arrived in the last 500 ms.
2. **ESP-Drone app** – used if no ESP-NOW packet but app packets arrived in the last 500 ms.
3. **Nothing** – failsafe: motors stop and the drone disarms.

Do not change source while flying (disarm first).

### 5.1 ESP-NOW transmitter (`espflight_tx`)
- No pairing. The TX broadcasts on WiFi channel 1 at 100 Hz.
- Packet: magic `0xFC5A`, sequence number, 8 channels (1000–2000 µs, order A E T R AUX1–AUX4), CRC-8 (polynomial 0x07). Anything with the wrong magic or CRC is ignored.
- TX wiring (ADC1 pins only): throttle GPIO 32, yaw 33, roll 34, pitch 35; ARM switch GPIO 25, ANGLE switch GPIO 26, AUX3/AUX4 GPIO 27/14 (switches to GND); LED GPIO 2.
- Calibrate stick ranges with `ADC_MIN/ADC_MAX`; reverse any axis with `REV_THR/YAW/ROLL/PITCH`; set `PRINT_ADC 1` to see raw values.
- Default modes: **AUX1 high = ARM**, **AUX2 high = ANGLE**.

### 5.2 ESP-Drone phone app
1. Turn the transmitter **off**.
2. Connect the phone to WiFi `ESP-DRONE_xxxxxx` (password `12345678`), keep it connected even if it says "no internet".
3. Open the app and connect. Keep the throttle at the **bottom** first: the app's first thrust-0 packet unlocks the drone.
4. Protocol: Crazyflie CRTP over UDP port 2390 (drone IP 192.168.43.42). Commander packet (port 3): roll/pitch in degrees, yaw in °/s, thrust 0–65535. Stop setpoint (port 7) cuts thrust. Parameter/log requests (ports 2 and 5) get empty replies so the app does not hang.
5. If the app moves only the throttle, open the web page and check "App raw" values; adjust **App input scale** (see section 8).

---

## 6. Arming and failsafe

The drone arms only when **all** of these are true:
- Gyro/IMU OK and calibration finished
- A control source is present (no failsafe)
- Throttle below about 5 %
- Drone tilt below 25°
- The ARM switch was switched **off → on** after power-up or after a link loss (a switch that is already on at power-up will not arm)

It disarms when the ARM switch goes off, the link is lost for 500 ms, or the IMU fails.

Arming blockers (shown on the web page and in Configurator): NO_GYRO, NO_RADIO (RX_FAILSAFE), THROTTLE_NOT_LOW, TILTED (ANGLE), CALIBRATING, ARM_SWITCH_ON_FIRST.

### LED (GPIO 2)
| Pattern | Meaning |
|---|---|
| Solid on | Armed |
| Fast blink | Calibrating or IMU/gyro problem |
| Slow blink | No radio (no ESP-NOW / app signal) |
| Double flash | Ready: gyro OK, radio connected, disarmed |

---

## 7. Betaflight Configurator

Use **Betaflight Configurator 10.10** (the firmware reports API 1.42 / Betaflight 4.2, like ESPFC). Connect over USB at 115200. Opening the port resets the ESP32 (about 2 s gyro calibration); if connecting times out, press Connect again.

Works: Setup (3D model, sensors, calibrate accelerometer), Receiver (channel monitor), Motors (test), Modes (ARM / ANGLE ranges), PID Tuning (P/I/D, level strength, max angle via PID_ADVANCED), Rates (RC rate, expo, super rate, TPA). Click **Save** to write to flash.

Not implemented: CLI, Blackbox, OSD, Filters tab, GPS, Ports, Configuration tab features, firmware flashing. Unsupported SET commands are acknowledged and ignored. The pitch angle shown in Configurator is sign-flipped in `msp.h` so nose up/down matches the real drone.

MSP commands answered: API/FC/board/build info, name, UID, status (+ex), raw IMU, motor, RC, attitude, altitude (zero), analog, RC tuning, PID, PID advanced, box names/IDs, mode ranges (+extra), feature, RX map/config, arming config, mixer/advanced/filter/sensor/motor config stubs, accelerometer calibration, motor set (test), settings reset, EEPROM write, reboot.

---

## 8. Built-in web page

Connect to the drone WiFi and open **http://192.168.43.42** (works at the same time as the app).

- **Live view:** armed state, why arming is blocked, control source, roll/pitch/yaw, raw app values, RC channels, motor bars, loop time, load, battery, I2C errors.
- **Edit:** PID (roll/pitch/yaw), level strength, max angle, RC rate, expo, super rate, TPA. **Apply** = use now, **Save to flash** = keep after reboot, **Defaults** = reset.
- **App input scale:** multiplier for roll / pitch / yaw coming from the ESP-Drone app. Negative = inverted. If the app values are small (about ±1) instead of degrees, try Roll/Pitch ≈ 30 and Yaw ≈ 200.
- **Calibrate accel** (drone flat and still).
- **Motor test:** hold a button to spin one motor (disarmed only, limited to 50 %, stops 0.6 s after you release). Props off.

The page has **no password**: anyone on the drone WiFi can open it. Change `AP_PASSWORD` in `config.h`.

---

## 9. Bench test before the first flight (PROPS OFF)

1. USB power. LED double flash = ready.
2. Setup tab (or web page): nose down → pitch positive, right side down → roll positive. If wrong, edit `IMU_X/Y/Z`.
3. Calibrate accelerometer with the drone flat.
4. Receiver tab: each stick moves the correct bar, ARM switch moves AUX1.
5. Motor test: M1 rear right, M2 front right, M3 rear left, M4 front left, each spinning in the direction shown above.
6. Arm in ANGLE mode, throttle about 30 %, tilt the board by hand: the **low side** motors must speed up. Roll right → M1 + M2 faster; nose down → M2 + M4 faster; rotate clockwise → M2 + M3 faster (otherwise flip `YAW_MIX_SIGN`).
7. Fit props, first hop low and over soft ground, ANGLE mode, low throttle.

---

## 10. PID tuning

Tune in ANGLE mode, one value at a time, changes of 10–15 %, always on the same charged battery.

1. **P** – raise until the drone reacts quickly; if it shakes fast, lower it.
2. **D** – raise to remove 2–3 bounces after a stick release or knock; too much = hot motors and a buzzing sound (brushed motors need little D).
3. **I** – raise if it drifts or does not hold the angle; too much = slow wobble.

| Symptom | Fix |
|---|---|
| Fast shaking | Lower P or D |
| Slow wobble | Lower I |
| Overshoots / bounces | Raise D |
| Hot motors, buzzing | Lower D, balance props, damp the IMU |
| Sluggish | Raise P |
| Drifts to one side | Raise I, recalibrate accelerometer |
| Flips on take-off | Check motor order, prop direction, IMU axes first |

Yaw: raise P until it holds heading; keep D at 0.
Level strength: higher = snappier self-leveling, lower = softer.

---

## 11. Important settings in `config.h`

| Setting | Meaning |
|---|---|
| `PIN_*` | All pins |
| `IMU_X/Y/Z` | IMU mounting orientation |
| `PWM_FREQ` | Motor PWM frequency (20 kHz) |
| `GYRO_LPF_HZ`, `DTERM_LPF_HZ` | Filters (100 Hz, 60 Hz) |
| `FAILSAFE_MS` | Link-loss timeout (500 ms) |
| `THROTTLE_CUT` | Motors off below this throttle |
| `ITERM_MIN_THR` | I-term only active above this throttle |
| `MOTOR_IDLE` | Minimum duty when running |
| `YAW_MIX_SIGN` | Flip yaw direction |
| `VBAT_ENABLED`, `VBAT_COMP` | Battery voltage reading and compensation |
| `AP_PASSWORD` | Drone WiFi password |
| `ESPNOW_CHANNEL` | Must be the same in both sketches |
| `MSP_API_MINOR` | Version reported to Configurator |
| `SETTINGS_MAGIC` | Change to wipe saved settings |

---

## 12. Troubleshooting

| Problem | Check |
|---|---|
| Won't compile | Board = ESP32 Dev Module, esp32 core installed, all files in the folder named `espflight` |
| Fast LED blink | IMU not found: SDA/SCL wiring, 3.3 V, address 0x68/0x69 |
| Won't arm | Open the web page, read "Arm blockers" |
| App only moves throttle | Web page → App raw values → App input scale |
| TX not detected | Same `ESPNOW_CHANNEL`, TX powered, drone WiFi on channel 1 |
| Motor 4 twitches at boot | GPIO 14 boot pulse: add pulldown or use another pin |
| Motor stalls at low throttle | Raise `MOTOR_IDLE` |
| Configurator won't connect | Use version 10.10, press Connect again, change `MSP_API_MINOR` if needed |
| Nothing prints in Serial Monitor | Intentional: Serial is reserved for MSP |

---

## 13. Limits and honest notes

- No magnetometer, barometer, GPS, blackbox, OSD, CLI or battery failsafe.
- Yaw angle drifts (no compass); yaw is rate controlled so flight is unaffected.
- ESP-NOW is not encrypted and the web page has no password; CRC only protects against corrupted packets, not against hostile ones.
- ESP-Drone app support (TOC replies) is best-effort; some app versions may behave differently.
- Written without access to your exact hardware. Test carefully and report anything odd so it can be fixed.
