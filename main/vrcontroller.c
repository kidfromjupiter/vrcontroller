#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "calibration.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "madgwick.h"
#include "magnetometer_protocol.h"
#include "sensors.h"
#include "tracker_config.h"
#include "tracker_protocol.h"

#define CALIBRATION_COMMAND_QUEUE_LENGTH 8U
#define STATUS_SEND_TIMEOUT_MS 200U

static const char *TAG = "vrcontroller";
static const uint8_t recv_addr[ESP_NOW_ETH_ALEN] = {
    0x34, 0xb7, 0xda, 0xfb, 0x23, 0xf8
};

static QueueHandle_t espnow_send_queue;
static QueueHandle_t calibration_command_queue;
static QueueHandle_t mag_command_queue;
static mag_session_t mag_session;
static bool espnow_send_pending;
static uint32_t espnow_successes;
static uint32_t espnow_failures;
static tracker_calibration_status_t last_terminal_status;
static bool has_terminal_status;

typedef struct {
    uint16_t request_id;
    bool has_previous_calibration;
    bool has_previous_maps;
    tracker_calibration_status_t current;
} remote_calibration_t;

static float vector_norm(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info,
                           esp_now_send_status_t status)
{
    if (tx_info != NULL) {
        (void)xQueueOverwrite(espnow_send_queue, &status);
    }
}

static void espnow_receive_cb(const esp_now_recv_info_t *info,
                              const uint8_t *data, int data_len)
{
    if (info == NULL || info->src_addr == NULL || data == NULL ||
        memcmp(info->src_addr, recv_addr, ESP_NOW_ETH_ALEN) != 0) {
        return;
    }
    mag_command_t mag_command;
    if (mag_command_decode(data, (size_t)data_len, &mag_command)) {
        (void)xQueueSend(mag_command_queue, &mag_command, 0);
        return;
    }
    tracker_calibration_command_t command;
    if (tracker_calibration_command_decode(data, (size_t)data_len, &command)) {
        (void)xQueueSend(calibration_command_queue, &command, 0);
    }
}

static bool finish_pending_send(TickType_t timeout)
{
    if (!espnow_send_pending) return true;
    esp_now_send_status_t status;
    if (xQueueReceive(espnow_send_queue, &status, timeout) != pdTRUE) {
        return false;
    }
    espnow_send_pending = false;
    if (status == ESP_NOW_SEND_SUCCESS) ++espnow_successes;
    else ++espnow_failures;
    return true;
}

static bool send_blocking(const uint8_t *data, size_t size)
{
    if (!finish_pending_send(pdMS_TO_TICKS(STATUS_SEND_TIMEOUT_MS))) {
        ++espnow_failures;
        return false;
    }
    esp_err_t err = esp_now_send(recv_addr, data, size);
    if (err != ESP_OK) {
        ++espnow_failures;
        return false;
    }
    espnow_send_pending = true;
    return finish_pending_send(pdMS_TO_TICKS(STATUS_SEND_TIMEOUT_MS));
}

static void send_calibration_status(const tracker_calibration_status_t *status)
{
    uint8_t packet[TRACKER_CALIBRATION_STATUS_SIZE];
    tracker_calibration_status_encode(packet, status);
    (void)send_blocking(packet, sizeof(packet));
}

static void remember_and_send_terminal(tracker_calibration_status_t status)
{
    last_terminal_status = status;
    has_terminal_status = true;
    send_calibration_status(&status);
}

static uint8_t calibration_flags(const tracker_calibration_t *calibration)
{
    return calibration == NULL ? 0 : TRACKER_CAL_STATUS_HAS_CALIBRATION |
        ((calibration->flags & CALIBRATION_HAS_MAPS) ? TRACKER_CAL_STATUS_HAS_MAPS : 0);
}

static void send_map_report(uint16_t request_id, const tracker_calibration_t *calibration)
{
    if (calibration == NULL) return;
    tracker_calibration_map_report_t report = {
        .request_id = request_id, .flags = calibration_flags(calibration),
    };
    memcpy(report.imu_axis_map, calibration->imu_axis_map, sizeof(report.imu_axis_map));
    memcpy(report.mag_axis_map, calibration->mag_axis_map, sizeof(report.mag_axis_map));
    uint8_t packet[TRACKER_CALIBRATION_MAP_REPORT_SIZE];
    tracker_calibration_map_report_encode(packet, &report);
    (void)send_blocking(packet, sizeof(packet));
}

static void send_idle_status(uint16_t request_id, const tracker_calibration_t *calibration)
{
    if (has_terminal_status && request_id != 0 &&
        request_id == last_terminal_status.request_id) {
        send_calibration_status(&last_terminal_status);
        send_map_report(request_id, calibration);
        return;
    }
    tracker_calibration_status_t status = {
        .stage = TRACKER_CAL_STAGE_IDLE,
        .result = TRACKER_CAL_RESULT_NONE,
        .flags = calibration_flags(calibration),
        .request_id = request_id,
    };
    send_calibration_status(&status);
    send_map_report(request_id, calibration);
}

static void calibration_notify(void *context,
                               tracker_calibration_stage_t stage,
                               tracker_calibration_result_t result,
                               uint8_t elapsed_seconds,
                               uint16_t sample_count, bool can_finish)
{
    remote_calibration_t *remote = context;
    remote->current.stage = stage;
    remote->current.result = result;
    remote->current.flags = remote->has_previous_calibration
        ? TRACKER_CAL_STATUS_HAS_CALIBRATION
        : 0;
    if (remote->has_previous_maps) remote->current.flags |= TRACKER_CAL_STATUS_HAS_MAPS;
    if (can_finish) remote->current.flags |= TRACKER_CAL_STATUS_CAN_FINISH;
    remote->current.elapsed_seconds = elapsed_seconds;
    remote->current.sample_count = sample_count;
    remote->current.request_id = remote->request_id;
    send_calibration_status(&remote->current);
}

static calibration_control_t calibration_wait_control(
    void *context, tracker_calibration_stage_t expected_stage,
    uint32_t timeout_ms)
{
    remote_calibration_t *remote = context;
    (void)xQueueReset(mag_command_queue);
    TickType_t timeout = timeout_ms == UINT32_MAX
        ? portMAX_DELAY
        : pdMS_TO_TICKS(timeout_ms);
    tracker_calibration_command_t command;
    if (xQueueReceive(calibration_command_queue, &command, timeout) != pdTRUE) {
        return CALIBRATION_CONTROL_NONE;
    }
    if (command.action == TRACKER_CAL_ACTION_QUERY) {
        tracker_calibration_status_t reply = remote->current;
        reply.request_id = command.request_id;
        send_calibration_status(&reply);
        return CALIBRATION_CONTROL_NONE;
    }
    if (command.request_id != remote->request_id) {
        tracker_calibration_status_t busy = remote->current;
        busy.request_id = command.request_id;
        busy.result = TRACKER_CAL_RESULT_BUSY;
        send_calibration_status(&busy);
        return CALIBRATION_CONTROL_NONE;
    }
    if (command.action == TRACKER_CAL_ACTION_START_FULL) {
        send_calibration_status(&remote->current);
        return CALIBRATION_CONTROL_NONE;
    }
    if (command.action == TRACKER_CAL_ACTION_CANCEL) {
        return CALIBRATION_CONTROL_CANCEL;
    }
    if (command.expected_stage != expected_stage) {
        send_calibration_status(&remote->current);
        return CALIBRATION_CONTROL_NONE;
    }
    if (command.action == TRACKER_CAL_ACTION_CONFIRM) {
        return CALIBRATION_CONTROL_CONFIRM;
    }
    if (command.action == TRACKER_CAL_ACTION_FINISH_MAG) {
        return CALIBRATION_CONTROL_FINISH;
    }
    return CALIBRATION_CONTROL_NONE;
}

static bool run_remote_calibration(sensors_t *sensors,
                                   tracker_calibration_t *calibration,
                                   bool had_calibration, uint16_t request_id)
{
    mag_session.active = false;
    (void)xQueueReset(mag_command_queue);
    remote_calibration_t remote = {
        .request_id = request_id,
        .has_previous_calibration = had_calibration,
        .has_previous_maps = had_calibration && (calibration->flags & CALIBRATION_HAS_MAPS),
        .current = {
            .stage = TRACKER_CAL_STAGE_WAIT_STILL,
            .result = TRACKER_CAL_RESULT_NONE,
            .flags = calibration_flags(had_calibration ? calibration : NULL),
            .request_id = request_id,
        },
    };
    calibration_remote_io_t io = {
        .context = &remote,
        .notify = calibration_notify,
        .wait_control = calibration_wait_control,
    };
    tracker_calibration_t candidate;
    tracker_calibration_result_t failure = TRACKER_CAL_RESULT_SENSOR;
    esp_err_t err = calibration_run_wizard(sensors, &io, &candidate, &failure);
    (void)xQueueReset(mag_command_queue);

    tracker_calibration_status_t terminal = remote.current;
    terminal.elapsed_seconds = 0;
    terminal.sample_count = 0;
    terminal.flags = calibration_flags(had_calibration ? calibration : NULL);
    if (err == ESP_OK) {
        *calibration = candidate;
        terminal.stage = TRACKER_CAL_STAGE_COMPLETE;
        terminal.result = TRACKER_CAL_RESULT_OK;
        terminal.flags = calibration_flags(calibration);
        remember_and_send_terminal(terminal);
        send_map_report(request_id, calibration);
        ESP_LOGI(TAG, "Remote calibration saved");
        return true;
    }

    terminal.stage = failure == TRACKER_CAL_RESULT_CANCELLED
        ? TRACKER_CAL_STAGE_CANCELLED
        : TRACKER_CAL_STAGE_FAILED;
    terminal.result = failure;
    remember_and_send_terminal(terminal);
    ESP_LOGE(TAG, "Remote calibration failed: result=%d error=%s",
             failure, esp_err_to_name(err));
    return had_calibration;
}

static void espnow_init(void)
{
    espnow_send_queue = xQueueCreate(1, sizeof(esp_now_send_status_t));
    calibration_command_queue = xQueueCreate(
        CALIBRATION_COMMAND_QUEUE_LENGTH,
        sizeof(tracker_calibration_command_t));
    mag_command_queue = xQueueCreate(4, sizeof(mag_command_t));
    ESP_ERROR_CHECK(espnow_send_queue == NULL ||
                    calibration_command_queue == NULL || mag_command_queue == NULL
                        ? ESP_ERR_NO_MEM
                        : ESP_OK);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(34));
    ESP_ERROR_CHECK(esp_wifi_set_channel(TRACKER_ESPNOW_CHANNEL,
                                         WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_receive_cb));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, recv_addr, ESP_NOW_ETH_ALEN);
    peer.channel = TRACKER_ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_LOGI(TAG, "ESP-NOW ready: peer=" MACSTR " channel=%d",
             MAC2STR(recv_addr), TRACKER_ESPNOW_CHANNEL);
}

static bool wait_for_initial_calibration(sensors_t *sensors,
                                         tracker_calibration_t *calibration)
{
    int64_t next_beacon = 0;
    for (;;) {
        (void)xQueueReset(mag_command_queue);
        int64_t now = esp_timer_get_time();
        if (now >= next_beacon) {
            send_idle_status(0, NULL);
            next_beacon = now + 1000000;
        }
        tracker_calibration_command_t command;
        if (xQueueReceive(calibration_command_queue, &command,
                          pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        if (command.action == TRACKER_CAL_ACTION_QUERY) {
            send_idle_status(command.request_id, NULL);
        } else if (command.action == TRACKER_CAL_ACTION_START_FULL) {
            if (run_remote_calibration(sensors, calibration, false,
                                       command.request_id)) {
                return true;
            }
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(calibration_storage_init());
    espnow_init();

    sensors_t sensors;
    ESP_ERROR_CHECK(sensors_init(&sensors));
    tracker_calibration_t calibration;
    bool has_calibration = calibration_load(&calibration);
    if (!has_calibration) {
        ESP_LOGW(TAG, "No calibration; waiting for vrrecv");
        has_calibration = wait_for_initial_calibration(&sensors, &calibration);
    }

    for (;;) {
        madgwick_t filter;
        madgwick_init(&filter, TRACKER_MADGWICK_BETA);
        float latest_mag[3] = {0};
        int64_t latest_mag_time = 0;
        bool filter_seeded = false;
        uint32_t imu_errors = 0, mag_errors = 0;
        uint32_t mag_ready_samples = 0, mag_overflows = 0;
        uint32_t mag_range_rejections = 0;
        float last_raw_mag_norm = NAN, last_mag_norm = NAN;
        uint16_t packet_sequence = 0;
        int64_t previous_sample_time = esp_timer_get_time();
        int64_t next_log_time = previous_sample_time + 1000000;
        TickType_t next_wake = xTaskGetTickCount();
        bool restart_fusion = false;
        int64_t next_mag_report = 0;
        mag_sample_t diagnostic_sample = {0};

        while (!restart_fusion) {
            tracker_calibration_command_t command;
            while (xQueueReceive(calibration_command_queue, &command, 0) ==
                   pdTRUE) {
                if (command.action == TRACKER_CAL_ACTION_QUERY) {
                    send_idle_status(command.request_id, &calibration);
                } else if (command.action == TRACKER_CAL_ACTION_START_FULL) {
                    if (has_terminal_status && command.request_id == last_terminal_status.request_id) {
                        send_idle_status(command.request_id, &calibration);
                        continue;
                    }
                    (void)finish_pending_send(
                        pdMS_TO_TICKS(STATUS_SEND_TIMEOUT_MS));
                    has_calibration = run_remote_calibration(
                        &sensors, &calibration, true, command.request_id);
                    restart_fusion = true;
                    break;
                } else {
                    send_idle_status(command.request_id, &calibration);
                }
            }
            if (restart_fusion) break;

            int64_t sample_time = esp_timer_get_time();
            mag_command_t diagnostic_command;
            while (xQueueReceive(mag_command_queue, &diagnostic_command, 0) == pdTRUE) {
                bool was_active = mag_session_active(&mag_session, sample_time);
                uint16_t previous_request = mag_session.request_id;
                mag_session_command(&mag_session, &diagnostic_command, sample_time);
                if (mag_session.active && (!was_active || previous_request != mag_session.request_id)) {
                    next_mag_report = sample_time;
                    diagnostic_sample.flags = MAG_FLAG_CALIBRATED;
                }
            }
            bool diagnostic_active = mag_session_active(&mag_session, sample_time);
            float dt = (sample_time - previous_sample_time) * 1.0e-6f;
            previous_sample_time = sample_time;
            if (dt < 0.001f || dt > 0.05f) {
                dt = 1.0f / TRACKER_SAMPLE_RATE_HZ;
            }

            mpu6050_sample_t raw_imu;
            if (sensors_read_mpu(&sensors, &raw_imu) != ESP_OK) {
                ++imu_errors;
                vTaskDelayUntil(&next_wake,
                    pdMS_TO_TICKS(1000 / TRACKER_SAMPLE_RATE_HZ));
                continue;
            }

            qmc5883p_sample_t raw_mag;
            bool mag_ready = false;
            esp_err_t mag_err = sensors_read_mag(&sensors, &raw_mag,
                                                  &mag_ready);
            if (mag_err != ESP_OK) {
                ++mag_errors;
                diagnostic_sample.flags = MAG_FLAG_CALIBRATED | MAG_FLAG_SENSOR_ERROR;
                diagnostic_sample.timestamp_us = (uint32_t)sample_time;
                diagnostic_sample.raw_ut = diagnostic_sample.calibrated_ut = NAN;
            } else if (raw_mag.overflow) {
                ++mag_overflows;
                diagnostic_sample.flags = MAG_FLAG_CALIBRATED | MAG_FLAG_OVERFLOW;
                diagnostic_sample.timestamp_us = (uint32_t)sample_time;
                diagnostic_sample.raw_ut = diagnostic_sample.calibrated_ut = NAN;
                latest_mag_time = 0;
            } else if (mag_ready) {
                ++mag_ready_samples;
                last_raw_mag_norm = vector_norm(raw_mag.magnetic_ut);
                float corrected_native[3];
                calibration_apply_mag(&calibration, &raw_mag,
                                      corrected_native);
                sensors_apply_axis_map(corrected_native, calibration.mag_axis_map, latest_mag);
                float mag_norm = vector_norm(latest_mag);
                last_mag_norm = mag_norm;
                diagnostic_sample.flags = MAG_FLAG_CALIBRATED | MAG_FLAG_FRESH;
                diagnostic_sample.timestamp_us = (uint32_t)sample_time;
                diagnostic_sample.raw_ut = last_raw_mag_norm;
                diagnostic_sample.calibrated_ut = mag_norm;
                if (!isfinite(mag_norm) || mag_norm < TRACKER_MAG_MIN_UT ||
                    mag_norm > TRACKER_MAG_MAX_UT)
                    diagnostic_sample.flags |= MAG_FLAG_OUT_OF_RANGE;
                if (isfinite(mag_norm) && mag_norm >= TRACKER_MAG_MIN_UT &&
                    mag_norm <= TRACKER_MAG_MAX_UT) {
                    latest_mag_time = sample_time;
                } else {
                    ++mag_range_rejections;
                    latest_mag_time = 0;
                }
            }

            float accel_native[3], gyro_native_dps[3], accel[3], gyro_dps[3];
            calibration_apply_mpu(&calibration, &raw_imu, accel_native,
                                  gyro_native_dps);
            sensors_apply_axis_map(accel_native, calibration.imu_axis_map, accel);
            sensors_apply_axis_map(gyro_native_dps, calibration.imu_axis_map, gyro_dps);
            float gyro_rad[3];
            for (int i = 0; i < 3; ++i) {
                gyro_rad[i] = gyro_dps[i] * (float)M_PI / 180.0f;
            }

            float accel_norm = vector_norm(accel);
            bool accel_valid = isfinite(accel_norm) && accel_norm > 0.1f &&
                               accel_norm < 8.5f;
            bool mag_valid = latest_mag_time != 0 &&
                sample_time - latest_mag_time <= TRACKER_MAG_MAX_AGE_US;
            if (!filter_seeded && accel_valid) {
                madgwick_seed(&filter, accel, latest_mag, mag_valid);
                filter_seeded = true;
            }
            if (!filter_seeded) {
                vTaskDelayUntil(&next_wake,
                    pdMS_TO_TICKS(1000 / TRACKER_SAMPLE_RATE_HZ));
                continue;
            }

            float previous_q[4] = {
                filter.q[0], filter.q[1], filter.q[2], filter.q[3]
            };
            bool fusion_ok = accel_valid
                ? madgwick_update(&filter, gyro_rad, accel, latest_mag,
                                  mag_valid, dt)
                : madgwick_update_gyro(&filter, gyro_rad, dt);
            if (!fusion_ok) {
                memcpy(filter.q, previous_q, sizeof(previous_q));
            }

            (void)finish_pending_send(0);
            if (diagnostic_active && sample_time >= next_mag_report && !espnow_send_pending) {
                uint8_t diagnostic_packet[MAG_SAMPLE_SIZE];
                diagnostic_sample.request_id = mag_session.request_id;
                diagnostic_sample.sequence = mag_session.sequence;
                if ((uint32_t)((uint32_t)sample_time - diagnostic_sample.timestamp_us) >
                    TRACKER_MAG_MAX_AGE_US) {
                    diagnostic_sample.flags &= ~MAG_FLAG_FRESH;
                }
                if (!(diagnostic_sample.flags & MAG_FLAG_FRESH)) {
                    diagnostic_sample.raw_ut = diagnostic_sample.calibrated_ut = NAN;
                    diagnostic_sample.timestamp_us = (uint32_t)sample_time;
                }
                mag_sample_encode(diagnostic_packet, &diagnostic_sample);
                if (esp_now_send(recv_addr, diagnostic_packet, sizeof(diagnostic_packet)) == ESP_OK) {
                    espnow_send_pending = true;
                    ++mag_session.sequence;
                    next_mag_report = sample_time + 100000;
                    diagnostic_sample.flags = MAG_FLAG_CALIBRATED;
                } else {
                    ++espnow_failures;
                    next_mag_report = sample_time + 100000;
                }
            }
            if (!espnow_send_pending) {
                uint8_t packet[TRACKER_PACKET_SIZE];
                tracker_packet_encode(packet, packet_sequence,
                                      (uint32_t)sample_time, filter.q, accel, 0);
                if (esp_now_send(recv_addr, packet, sizeof(packet)) == ESP_OK) {
                    espnow_send_pending = true;
                    ++packet_sequence;
                } else {
                    ++espnow_failures;
                }
            }

            if (sample_time >= next_log_time) {
                ESP_LOGI(TAG, "q=[%.3f %.3f %.3f %.3f] mag=%s raw=%.1f "
                         "norm=%.1f ready=%" PRIu32 " reject=%" PRIu32
                         " ovfl=%" PRIu32 " imu_err=%" PRIu32
                         " mag_err=%" PRIu32 " espnow_ok=%" PRIu32
                         " espnow_fail=%" PRIu32,
                         filter.q[0], filter.q[1], filter.q[2], filter.q[3],
                         mag_valid ? "9dof" : "6dof", last_raw_mag_norm,
                         last_mag_norm, mag_ready_samples,
                         mag_range_rejections, mag_overflows, imu_errors,
                         mag_errors, espnow_successes, espnow_failures);
                next_log_time = sample_time + 1000000;
            }
            vTaskDelayUntil(&next_wake,
                pdMS_TO_TICKS(1000 / TRACKER_SAMPLE_RATE_HZ));
        }
    }
}
