#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_now.h"
#include "calibration.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "madgwick.h"
#include "nvs_flash.h"
#include "sensors.h"
#include "tracker_config.h"
#include "tracker_protocol.h"


static const char *TAG = "vrcontroller";
static const int8_t IMU_MAP[3] = TRACKER_IMU_AXIS_MAP;
static const int8_t MAG_MAP[3] = TRACKER_MAG_AXIS_MAP;
/* Station MAC address of the ESP32 that receives the orientation data. */
static const uint8_t recv_addr[ESP_NOW_ETH_ALEN] = {
    0x34, 0xb7, 0xda, 0xfb, 0x23, 0xf8
};
// static const uint8_t recv_addr[ESP_NOW_ETH_ALEN] = {
//     0xff, 0xff, 0xff, 0xff, 0xff, 0xff
// };

/* The Wi-Fi callback puts each asynchronous send result into this queue. */
static QueueHandle_t espnow_q = NULL;

static float vector_norm(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

/*
 * ESP-NOW sends asynchronously. This callback runs in the high-priority Wi-Fi
 * task, so it only copies the result to a queue. Logging and other work happen
 * later in app_main().
 */
static void espnow_send_cb(const esp_now_send_info_t *tx_info,
                           esp_now_send_status_t status)
{
    if (tx_info == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
    xQueueOverwrite(espnow_q, &status);
}

static void espnow_init(void)
{
    /* A one-item queue is enough because only one send is allowed at a time. */
    espnow_q = xQueueCreate(1, sizeof(esp_now_send_status_t));
    ESP_ERROR_CHECK(espnow_q == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    /* ESP-NOW uses the Wi-Fi radio, even though it does not need a router. */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(34));
    ESP_ERROR_CHECK(esp_wifi_set_channel(TRACKER_ESPNOW_CHANNEL,
                                         WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    /*
     * A unicast destination must be added as a peer before esp_now_send().
     * The receiver must use this same Wi-Fi channel.
     */
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, recv_addr, ESP_NOW_ETH_ALEN);
    peer_info.channel = TRACKER_ESPNOW_CHANNEL;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "ESP-NOW ready: peer=" MACSTR " channel=%d",
             MAC2STR(recv_addr), TRACKER_ESPNOW_CHANNEL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(calibration_storage_init());
    bool boot_mag_calibration = calibration_magnetometer_boot_requested();
    espnow_init();

    sensors_t sensors;
    ESP_ERROR_CHECK(sensors_init(&sensors));

    tracker_calibration_t calibration;
    bool has_calibration = calibration_load(&calibration);
    calibration_request_t calibration_request = CALIBRATION_REQUEST_FULL;
    if (has_calibration) {
        calibration_request = boot_mag_calibration
            ? CALIBRATION_REQUEST_MAG_ONLY
            : calibration_recalibration_request();
    }
    while (calibration_request != CALIBRATION_REQUEST_NONE) {
        esp_err_t err = calibration_request == CALIBRATION_REQUEST_MAG_ONLY
            ? calibration_run_magnetometer(&sensors, &calibration, &calibration)
            : calibration_run_wizard(&sensors, &calibration, has_calibration,
                                     &calibration);
        if (err == ESP_OK) {
            has_calibration = true;
            break;
        }
        if (has_calibration) {
            ESP_LOGE(TAG, "Recalibration failed: %s; keeping saved calibration",
                     esp_err_to_name(err));
            break;
        }
        ESP_LOGE(TAG, "Calibration failed: %s; retrying", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (!has_calibration) {
        ESP_LOGE(TAG, "No valid calibration available");
        return;
    }

    madgwick_t filter;
    madgwick_init(&filter, TRACKER_MADGWICK_BETA);

    float latest_mag[3] = {0};
    int64_t latest_mag_time = 0;
    bool filter_seeded = false;
    uint32_t imu_errors = 0, mag_errors = 0;
    uint32_t mag_ready_samples = 0, mag_overflows = 0, mag_range_rejections = 0;
    float last_raw_mag_norm = NAN, last_mag_norm = NAN;
    uint32_t espnow_successes = 0, espnow_failures = 0;
    bool espnow_send_pending = false;
    uint16_t packet_sequence = 0;
    int64_t previous_sample_time = esp_timer_get_time();
    int64_t next_log_time = previous_sample_time + 1000000;
    TickType_t next_wake = xTaskGetTickCount();

    for (;;) {
        int64_t sample_time = esp_timer_get_time();
        float dt = (sample_time - previous_sample_time) * 1.0e-6f;
        previous_sample_time = sample_time;
        if (dt < 0.001f || dt > 0.05f) dt = 1.0f / TRACKER_SAMPLE_RATE_HZ;

        mpu6050_sample_t raw_imu;
        if (sensors_read_mpu(&sensors, &raw_imu) != ESP_OK) {
            ++imu_errors;
            vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(1000 / TRACKER_SAMPLE_RATE_HZ));
            continue;
        }

        qmc5883p_sample_t raw_mag;
        bool mag_ready = false;
        esp_err_t mag_err = sensors_read_mag(&sensors, &raw_mag, &mag_ready);
        if (mag_err != ESP_OK) {
            ++mag_errors;
        } else if (raw_mag.overflow) {
            ++mag_overflows;
            latest_mag_time = 0;
        } else if (mag_ready) {
            ++mag_ready_samples;
            last_raw_mag_norm = vector_norm(raw_mag.magnetic_ut);
            float corrected_native[3];
            calibration_apply_mag(&calibration, &raw_mag, corrected_native);
            sensors_apply_axis_map(corrected_native, MAG_MAP, latest_mag);
            float mag_norm = vector_norm(latest_mag);
            last_mag_norm = mag_norm;
            if (isfinite(mag_norm) && mag_norm >= TRACKER_MAG_MIN_UT &&
                mag_norm <= TRACKER_MAG_MAX_UT) latest_mag_time = sample_time;
            else {
                ++mag_range_rejections;
                latest_mag_time = 0;
            }
        }

        float accel_native[3], gyro_native_dps[3], accel[3], gyro_dps[3];
        calibration_apply_mpu(&calibration, &raw_imu, accel_native,
                              gyro_native_dps);
        sensors_apply_axis_map(accel_native, IMU_MAP, accel);
        sensors_apply_axis_map(gyro_native_dps, IMU_MAP, gyro_dps);
        float gyro_rad[3];
        for (int i = 0; i < 3; ++i) gyro_rad[i] = gyro_dps[i] * (float)M_PI / 180.0f;

        float accel_norm = vector_norm(accel);
        bool accel_valid = isfinite(accel_norm) && accel_norm > 0.1f && accel_norm < 8.5f;
        bool mag_valid = latest_mag_time != 0 &&
                         sample_time - latest_mag_time <= TRACKER_MAG_MAX_AGE_US;

        if (!filter_seeded && accel_valid) {
            madgwick_seed(&filter, accel, latest_mag, mag_valid);
            filter_seeded = true;
        }
        if (!filter_seeded) {
            vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(1000 / TRACKER_SAMPLE_RATE_HZ));
            continue;
        }

        float previous_q[4] = {filter.q[0],filter.q[1],filter.q[2],filter.q[3]};
        bool fusion_ok = accel_valid
            ? madgwick_update(&filter, gyro_rad, accel, latest_mag, mag_valid, dt)
            : madgwick_update_gyro(&filter, gyro_rad, dt);
        if (!fusion_ok) {
            for (int i = 0; i < 4; ++i) filter.q[i] = previous_q[i];
        }

        /*
         * A send result means the previous transmission is finished. Waiting
         * for it prevents ESP-NOW callbacks from arriving out of order.
         */
        esp_now_send_status_t send_status;
        if (xQueueReceive(espnow_q, &send_status, 0) == pdTRUE) {
            espnow_send_pending = false;
            if (send_status == ESP_NOW_SEND_SUCCESS) ++espnow_successes;
            else
            {
               ESP_LOGI(TAG,"FAIL") ;
                ++espnow_failures;
            };
        }

        if (!espnow_send_pending) {
            /*
             * esp_now_send() copies the packet bytes before it returns. The
             * sequence advances once the Wi-Fi stack accepts the send, even
             * if the peer does not acknowledge it later.
             */
            uint8_t packet[TRACKER_PACKET_SIZE];
            tracker_packet_encode(packet, packet_sequence,
                                  (uint32_t)sample_time, filter.q, accel, 0);
            espnow_send_pending = true;
            esp_err_t send_err = esp_now_send(recv_addr, packet,
                                              sizeof(packet));
            if (send_err != ESP_OK) {
                /* No callback occurs when ESP-NOW rejects the send request. */
                espnow_send_pending = false;
                ++espnow_failures;
            } else {
                ++packet_sequence;
            }
        }

        if (sample_time >= next_log_time) {
            ESP_LOGI(TAG, "q=[%.3f %.3f %.3f %.3f] mag=%s raw_norm=%.1f"
                     " norm=%.1f"
                     " vec=[%.1f %.1f %.1f] ready=%" PRIu32
                     " reject=%" PRIu32 " ovfl=%" PRIu32
                     " imu_err=%" PRIu32 " mag_err=%" PRIu32 " espnow_ok=%" PRIu32
                     " espnow_fail=%" PRIu32,
                     filter.q[0], filter.q[1], filter.q[2], filter.q[3],
                     mag_valid ? "9dof" : "6dof", last_raw_mag_norm,
                     last_mag_norm,
                     latest_mag[0], latest_mag[1], latest_mag[2],
                     mag_ready_samples, mag_range_rejections, mag_overflows,
                     imu_errors, mag_errors,
                     espnow_successes, espnow_failures);
            next_log_time = sample_time + 1000000;
        }
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(1000 / TRACKER_SAMPLE_RATE_HZ));
    }
}
