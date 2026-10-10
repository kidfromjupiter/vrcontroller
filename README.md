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

Change hardware/fusion settings and calibration acceptance limits in
`main/tracker_config.h`. Its axis maps are fallbacks for legacy calibration.

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

### Learned sensor axes

Build and flash **both** controller and receiver firmware and use the updated
`calibrate_controller.py`; calibration commands/status now use protocol v2.
Orientation samples, the USB envelope, and magnitude diagnostics keep their
existing formats/versions. Old calibration tools cannot start the new wizard.

Body directions are **+X right, +Y front, +Z top**, a right-handed frame.
The first six poses now refer to these physical controller directions, not to
MPU package markings. The wizard determines a signed MPU axis permutation and
native accelerometer offsets/scales from the opposite faces. That same MPU
map applies to both acceleration and gyro. Gyro bias is measured while still;
gyro scale remains the nominal sensitivity.

After figure-eight magnetometer calibration, follow eight stationary holds:
right/front/top, left/front/top, right/back/top, left/back/top,
right/front/bottom, left/front/bottom, right/back/bottom, and left/back/bottom.
Point each indicated corner upward so its three directions point roughly equally
upward. Corner holds allow about 32 degrees of tilt error; alignment uses the
actual measured tilt. No precise heading is required; vary the front's compass direction
between holds while staying near the same location. Each hold gathers 100
fresh paired readings, with a five-second limit and a short settling delay.

The alignment solver compares all 24 proper signed magnetometer maps using
the consistency of the normalized gravity–magnetic-field dot product. Poor
or ambiguous alignment fails calibration and leaves the previous value active.
Movement or incorrect accelerometer poses return to the same prompt for a
manual retry, preserving completed steps. An invalid combined six-face fit
repeats all six faces. Ctrl-C still cancels. Magnetic overflow, insufficient
samples, and sensor errors abort without replacing saved calibration. The host prints both saved maps
as `body <- sensor` assignments when the wizard succeeds.

NVS v2 stores both maps and a learned-map flag together with the coefficients.
A valid v1 record remains usable with the configured fallback maps until the
new wizard succeeds. Those maps are explicitly labeled legacy. Native offsets
and scales are applied before mapping, exactly once. Only per-axis scaling
and mounting at 90-degree increments are supported; arbitrary rotation
matrices and full soft-iron compensation are not estimated.

Acceptance limits live in `main/tracker_config.h`: face dominance 0.95,
face pose cosine 0.94, corner pose cosine 0.85, accelerometer standard deviation 0.02 g, residual gyro rate
5 degrees/second, and magnetic standard deviation 1 µT during a hold. The
magnetometer map score must be at most 0.05, and the runner-up at least
`max(1.5 * best, best + 0.03)`. These are initial defaults requiring hardware
validation.

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

### Calibration v2 reports and host tests

Packet IDs 2 (command), 3 (status), and 6 (map report) use calibration version
2; quaternion packet ID 1 and magnetometer diagnostics IDs 4/5 stay version 1.
The command/status sizes stay 8/12 bytes. Corner wait/capture stages are
21/22 through 35/36; the earlier stage numbers are unchanged. New result
codes are 14 (invalid map), 15 (ambiguous map), and 16 (poor alignment).
Status flag 0x04 identifies calibration with learned maps.

The 12-byte map report has ID/version at 0–1, request ID at 2–3, signed
one-based MPU map at 4–6, magnetometer map at 7–9, status flags at 10, and
reserved zero byte 11. Each map lists the native source of body X/Y/Z;
for example `{2,-1,3}` means body X = native Y, body Y = -native X,
body Z = native Z. Reports are sent after success and in response to queries.

Run the platform-independent calibration math, protocol, and wizard/storage
tests (the latter uses fake sensors and NVS, not hardware) with:

```sh
bash tests/run_calibration_tests.sh
```
