# VR Controller Orientation Filter

ESP-IDF firmware for an ESP32-C3, MPU6050, and QMC5883P. It calibrates the
sensors and performs Madgwick orientation fusion at 100 Hz. The filter uses
9-DOF updates while valid magnetometer samples are available and falls back to
6-DOF updates when they are not.

> **Current test mode:** The sensor application is temporarily disabled with
> `#if 0` in `main/vrcontroller.c`. The active `app_main()` sends the
> null-terminated string `Hello world!` to the ESP-NOW peer once per second.

## Hardware defaults

- SDA: GPIO 8
- SCL: GPIO 9
- MPU6050: address 0x68 or 0x69
- QMC5883P: address 0x2c

Change these values, the Madgwick beta, or either sensor's signed axis map in
`main/tracker_config.h`.

## Calibration

Connect a 115200-baud UART0 monitor or the ESP32-C3 native USB Serial/JTAG
console before the first boot. With no calibration in NVS, the firmware starts
a guided sequence for stationary gyro bias, six-face accelerometer calibration,
and figure-eight magnetometer calibration. A valid result is saved to NVS.

On later boots, enter `c` during the three-second startup prompt to recalibrate.
If optional recalibration fails, the last valid calibration remains active.

## ESP-NOW hello-world test

The active program sends `Hello world!` once per second. The packet is 13 bytes:
12 visible characters followed by the null byte (`\0`) that ends a C string.
The receiver's station MAC address is `34:b7:da:fb:23:f8` in
`main/vrcontroller.c`. Both ESP32 devices must use channel 1, configured by
`TRACKER_ESPNOW_CHANNEL` in `main/tracker_config.h`.

`esp_now_send()` only starts a transmission. Its return value says whether
ESP-IDF accepted the request; the later send callback says whether the peer
acknowledged it. The active program waits for that callback, prints the result,
then sends again after one second.

The current `vrrecv` firmware will acknowledge this message, so the sender's
`ok` counter should increase, but its quaternion-only callback discards the
13-byte string. To display the message on the receiver during this test, its
receive callback can temporarily use:

```c
#include "esp_now.h"
#include "esp_log.h"

static void espnow_receive_cb(const esp_now_recv_info_t *info,
                              const uint8_t *data, int data_len)
{
    (void)info;

    /* The sender includes '\0', so this packet is safe to print as a string. */
    if (data_len == 13 && data[12] == '\0') {
        ESP_LOGI("recv", "Received: %s", (const char *)data);
    }
}

/* Register after starting Wi-Fi and calling esp_now_init(). */
ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_receive_cb));
```

Printing inside the Wi-Fi callback is acceptable for this slow one-message-per-
second test. For high-rate sensor data, copy packets to a FreeRTOS queue and
process them from a normal task instead.

## Build and test

Build normally with `idf.py build`. The platform-independent fusion tests can
be run with:

```sh
cc -std=c11 -D_GNU_SOURCE -Imain tests/test_tracker.c \
  main/madgwick.c -lm -o /tmp/vrcontroller_test
/tmp/vrcontroller_test
```
