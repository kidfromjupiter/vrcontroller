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
- MPU6050: address 0x68 or 0x69
- QMC5883P: address 0x2c

Change these values, the Madgwick beta, or either sensor's signed axis map in
`main/tracker_config.h`.

## Raw sensor values over serial

The controller prints the native signed 16-bit register values at 10 Hz on the
ESP-IDF serial console. Each `RAW` line contains MPU6050 accelerometer XYZ,
MPU6050 gyroscope XYZ, and QMC5883P magnetometer XYZ:

```text
I (...) vrcontroller: RAW accel=[120,-64,4070] gyro=[3,-7,1] mag=[812,-245,1330] mag_seen=1
```

`mag_seen=0` means that no fresh magnetometer sample has been received since
startup, so the displayed magnetometer values are still zero. Change
`TRACKER_RAW_SERIAL_RATE_HZ` in `main/tracker_config.h` to adjust the output
rate, then use `idf.py monitor` to view it.

## Calibration

Calibration is controlled wirelessly through the `vrrecv` receiver; the
controller does not need a USB or UART connection. Build and flash `../vrrecv`,
connect that receiver to the computer, close any pose tracker using its serial
port, and run:

```sh
python ../vrrecv/calibrate_controller.py /dev/ttyACM0 --slot right
```

Use `--slot left` for the second controller. The tool guides stationary gyro
bias, the six accelerometer faces, and figure-eight magnetometer calibration.
An uncalibrated controller advertises itself over ESP-NOW and waits for this
wizard. Calibration is committed to NVS only after every stage succeeds; a
failed or cancelled recalibration leaves the previous value active.

## Magnetometer calibration quality check

Flash the updated controller and receiver firmware, remove the receiver's
GPIO 0–1 diagnostic jumper, and close any other program using its USB port.
Check the saved calibration without repeating the wizard:

```sh
python ../vrrecv/check_magnetometer.py /dev/ttyACM0 --slot right --duration 30
```

Use `--slot left` for the second controller. Press Enter when ready, then
rotate through all orientations near one location for the entire capture.
The tool compares raw and calibrated magnetic-field magnitude in µT, reporting
minimum, maximum, mean, population standard deviation, and peak-to-peak range.
It also reports standard deviation and peak-to-peak range as percentages of
the mean, plus their relative reduction after calibration. Negative reduction
means variation increased. A zero raw variation or mean makes the relative
comparison unavailable.

Lower calibrated variation indicates more consistent magnitude during this
rotation test; the report does not assign a pass/fail grade or establish heading
accuracy. Changing magnetic surroundings or incomplete rotation coverage can
affect the result. Finite readings outside the fusion range (10–100 µT) remain
in the statistics and are counted. Stale, duplicate, nonfinite, uncalibrated,
overflow, and sensor-error readings are excluded and reported. At least 20
usable pairs are required. The duration defaults to 30 seconds and accepts
5–300 seconds.

Diagnostic data is streamed only during a check, at up to 10 Hz. Fusion
continues at its normal rate; diagnostic sends can replace some orientation
transmissions while the check runs. The normal orientation packet format stays
unchanged. Ctrl-C sends a stop command, and the controller automatically stops
streaming when the requested duration expires or calibration starts.

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

Packet IDs 2 and 3 carry receiver-to-controller calibration commands and
controller-to-receiver status. Commands include a request ID and expected stage
so retransmission cannot advance more than one wizard step.

## Build and test

Build normally with `idf.py build`. The platform-independent fusion tests can
be run with:

```sh
cc -std=c11 -D_GNU_SOURCE -Imain tests/test_tracker.c \
  main/madgwick.c main/tracker_protocol.c -lm -o /tmp/vrcontroller_test
/tmp/vrcontroller_test
```

### Magnetometer diagnostic packets

Packet ID 5 is an 8-byte little-endian command: ID/version at bytes 0–1,
start (1) or stop (2) at byte 2, zero reserved byte 3, nonzero request ID at
bytes 4–5, and duration in seconds at bytes 6–7 (5–300 for start, zero for stop).
A repeated start for the latest request ID does not restart an expired,
cancelled, or active session. A matching stop ends that session.

Packet ID 4 is a 20-byte diagnostic sample: ID/version at bytes 0–1,
request ID at 2–3, independent sequence at 4–5, flags at 6–7, low 32 bits of
the microsecond sensor timestamp at 8–11, and raw/calibrated magnitudes as
IEEE-754 float32 µT values at 12–15 and 16–19. Flags are fresh (0x01), sensor
error (0x02), overflow (0x04), outside the calibrated fusion range (0x08), and
saved calibration active (0x10). No fresh reading is reported with NaN
magnitudes. All multi-byte values are little-endian.

Run the standalone diagnostic protocol/session tests with:

```sh
cc -std=c11 -Wall -Wextra -Werror -Imain \
  tests/test_magnetometer_protocol.c main/magnetometer_protocol.c \
  -lm -o /tmp/vrcontroller_mag_test
/tmp/vrcontroller_mag_test
```
