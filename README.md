# ESPFLIGHT – ESP32 WROOM flight controller (Arduino IDE)

Brushed-motor quad firmware. IMU: **MPU6050 or MPU6500** (auto-detected). Control from:

1. **ESP-NOW transmitter** (`espflight_tx` sketch)
2. **ESP-Drone phone app** (drone is a WiFi AP, CRTP over UDP)
3. **Betaflight Configurator** over USB (MSP) for setup / PID / rates / modes / motor test

## Arduino IDE setup

1. Install **Arduino IDE 2.x**, then *Boards Manager → "esp32 by Espressif Systems"* (2.0.x or 3.x both work).
2. Open `espflight/espflight.ino` (folder name must stay `espflight`).
3. Board: **ESP32 Dev Module**. Everything else default. Upload speed 921600.
4. No extra libraries needed.

Do the same for the transmitter: open `espflight_tx/espflight_tx.ino` and upload to a second ESP32.

## Wiring (flight controller)

| Part | ESP32 pin |
|---|---|
| MPU SDA / SCL | GPIO 21 / 22 (3.3 V, GND) |
| Motor 1 – rear right | GPIO 25 (via MOSFET) |
| Motor 2 – front right | GPIO 26 |
| Motor 3 – rear left | GPIO 27 |
| Motor 4 – front left | GPIO 14 |
| Battery sense (optional) | GPIO 35 via divider, set `VBAT_ENABLED 1` in `config.h` |

Use logic-level MOSFETs with a gate pulldown (100 kΩ) and a flyback diode per motor. Pins can be changed in `config.h`.

Prop directions (Betaflight "props in"): M1 CCW, M2 CW, M3 CW, M4 CCW. If your layout differs, flip `YAW_MIX_SIGN`.

## Radio links

* **ESP-NOW**: no pairing. TX broadcasts on channel 1; drone checks magic + CRC. Channels are AETR + AUX. Default modes: **AUX1 high = ARM**, **AUX2 high = ANGLE** (change in Configurator → Modes). Arm only works with throttle low, drone level, and the arm switch toggled OFF → ON after power-up/link loss. No packets for 500 ms → motors stop and disarm.
* **ESP-Drone app**: connect the phone to WiFi `ESP-DRONE_xxxxxx` (password `12345678`), then open the app (drone IP `192.168.43.42`, UDP 2390). Always angle mode. Send thrust 0 first (app does this when the stick is at the bottom) to unlock.
* ESP-NOW has priority if both are active.

## Betaflight Configurator

Connect USB, pick the COM port, **115200** baud, connect. Opening the port resets the ESP32 (≈2 s gyro calibration) – if it times out, press Connect again. Do not print anything to `Serial` in this firmware, it would corrupt MSP.

Supported: Setup (3D model, sensors, calibrate accelerometer), Receiver (channel monitor), Motors (test, props off!), Modes (ARM / ANGLE), PID tuning (P/I/D roll-pitch-yaw, level strength) and Rates (BF rates, TPA). Press **Save** to write to flash. Not implemented: CLI, Blackbox, OSD, filters tab, GPS, etc. Unknown read commands return an MSP error.

If Configurator refuses the firmware version, change `MSP_API_MINOR` in `config.h`.

## Before the first flight (PROPS OFF)

1. Power via USB. LED double-flash = gyro OK + radio link. Fast blink = IMU problem (check wiring / address).
2. Setup tab: tilt the board – model must follow. Roll right → right side down. Nose up → nose up. If wrong, edit `IMU_X/Y/Z` in `config.h`.
3. Press *Calibrate Accelerometer* with the board flat.
4. Receiver tab: all sticks move the right bars, arm switch moves AUX1.
5. Motors tab: each slider spins the right motor and the right direction (M1 rear-right … M4 front-left).
6. Arm with props off, ANGLE mode, throttle ~30 %, and tilt the board by hand: the motors on the **low side** must speed up (they push it back level). Roll right (right side down) → M1 + M2 faster. Nose down → front motors M2 + M4 faster. Rotate the board clockwise (yaw right) → M2 + M3 faster to counter it. If the yaw reaction is the opposite (M1 + M4 faster), flip `YAW_MIX_SIGN` in `config.h`.
7. First hop: low ANGLE mode, low throttle, over grass, kill switch ready. Tune P first, then D, then I.

## Built-in web page (new)

Connect your phone/PC to the drone WiFi and open **http://192.168.43.42**. It works at the same time as the ESP-Drone app.

* **Live view**: arm state, why arming is blocked, control source, attitude, raw values the app sends, RC channels, motor outputs.
* **Edit**: PID, level strength, max angle, rates, TPA. *Apply* = use now, *Save to flash* = keep after reboot.
* **App input scale**: if the drone ignores roll/pitch/yaw from the app, look at "App raw" while moving the app sticks. If the numbers are tiny (about -1..1) instead of degrees, set Roll/Pitch scale to about 30 and Yaw to about 200. Use a negative number to invert a direction.
* **Calibrate accel**, **Defaults**, and **Motor test** (hold a button, disarmed only, props off, max 50 %).

## Troubleshooting

* App moves only throttle: open the web page, compare "App raw" with the attitude. See "App input scale" above.
* Motors twitch or one does not start at low throttle: raise `MOTOR_IDLE` in `config.h` (default 0.04). I-term only builds above `ITERM_MIN_THR` (0.10) so the drone does not flip at take-off.
* Motor 4 on GPIO14 gets a short pulse at boot on ESP32. Use a gate pulldown, or move it to GPIO33 in `config.h`.
* After changing the IMU axis mapping, press *Calibrate accel* again. New firmware wipes old saved settings once.
* Betaflight Configurator: firmware now reports API 1.42 / BF 4.2 (what ESPFC reports for Configurator 10.10). Use Configurator **10.10**. PID, Rates, Modes, Receiver, Motors tabs load; options that are not supported are simply ignored on Save.

## Honest status

This is a first version written without access to your hardware: it has not been compiled or flown by me. Expect small fixes (pins, axis directions, PID values, MSP fields for your Configurator version). The ESP-Drone app handshake (TOC replies) is best-effort; if the app does not connect, report what it shows. Default PIDs are deliberately soft.
