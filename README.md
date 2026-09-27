# VR Controller Orientation Tracker

ESP-IDF firmware for an ESP32-C3, MPU6050, and QMC5883P. It calibrates the
sensors and performs Madgwick orientation fusion at 100 Hz. The filter uses
9-DOF updates while valid magnetometer samples are available and falls back to
6-DOF updates when they are not. It sends the resulting orientation and
acceleration to the `vrrecv` receiver over ESP-NOW.

## Hardware defaults

- SDA: GPIO 8
- SCL: GPIO 9
- Magnetometer calibration LED: GPIO 5 (active high)
- Magnetometer calibration boot trigger: GPIO 7 (active high, internal pull-down)
- MPU6050: address 0x68 or 0x69
- QMC5883P: address 0x2c

Change these values, the Madgwick beta, or either sensor's signed axis map in
`main/tracker_config.h`.

## Calibration

Connect a 115200-baud UART0 monitor or the ESP32-C3 native USB Serial/JTAG
console before the first boot. With no calibration in NVS, the firmware starts
a guided sequence for stationary gyro bias, six-face accelerometer calibration,
and figure-eight magnetometer calibration. A valid result is saved to NVS.

On later boots, enter `c` during the three-second startup prompt to run the
complete calibration wizard, or enter `m` to repeat only the figure-eight
magnetometer calibration while preserving the saved gyro and accelerometer
calibration. If optional recalibration fails, the last valid calibration
remains active.

Alternatively, hold GPIO 7 high continuously for two seconds from boot to go
directly into magnetometer-only calibration. This shortcut requires an existing
saved calibration; first boot still runs the complete calibration wizard.

## ESP-NOW protocol

The receiver's station MAC address is `34:b7:da:fb:23:f8` in
`main/vrcontroller.c`. Both devices use channel 1, configured by
`TRACKER_ESPNOW_CHANNEL` in `main/tracker_config.h`.

Each sample is a 24-byte version 1 packet in little-endian order:

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 1 | Packet ID (`1`) |
| 1 | 1 | Protocol version (`1`) |
| 2 | 2 | Sequence number |
| 4 | 4 | Low 32 bits of the microsecond sample timestamp |
| 8 | 8 | Quaternion W, X, Y, Z as four signed 16-bit values |
| 16 | 6 | Acceleration X, Y, Z as three signed 16-bit values |
| 22 | 2 | Status flags (currently zero) |

Quaternion values use 32767 counts per unit. Acceleration uses 2048 counts per
g and is calibrated and mapped into the same body axes used by the orientation
filter. Scaled values are rounded to the nearest integer and saturated to the
signed 16-bit range.

`esp_now_send()` starts an asynchronous transmission. The firmware allows only
one pending send at a time and uses its callback to count acknowledged and
failed transmissions. The packet sequence advances when the Wi-Fi stack
accepts a send request, so gaps can identify packets lost after submission.

## Build and test

Build normally with `idf.py build`. The platform-independent fusion tests can
be run with:

```sh
cc -std=c11 -D_GNU_SOURCE -Imain tests/test_tracker.c \
  main/madgwick.c main/tracker_protocol.c -lm -o /tmp/vrcontroller_test
/tmp/vrcontroller_test
```
