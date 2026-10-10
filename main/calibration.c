#include "calibration.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tracker_config.h"

#define CAL_MAGIC   0x56435243U
#define CAL_VERSION 1U
#define CAL_NAMESPACE "sensor_cal"
#define CAL_KEY       "v1"

static const char *TAG = "calibration";

static bool valid_calibration(const tracker_calibration_t *c)
{
    if (c->magic != CAL_MAGIC || c->version != CAL_VERSION ||
        c->size != sizeof(*c)) {
        return false;
    }
    const float *values = c->gyro_bias_dps;
    size_t count = (sizeof(*c) - offsetof(tracker_calibration_t, gyro_bias_dps)) /
                   sizeof(float);
    for (size_t i = 0; i < count; ++i) {
        if (!isfinite(values[i])) return false;
    }
    for (int i = 0; i < 3; ++i) {
        if (c->accel_scale[i] < 0.25f || c->accel_scale[i] > 4.0f ||
            c->mag_scale[i] < 0.1f || c->mag_scale[i] > 10.0f) {
            return false;
        }
    }
    return true;
}

esp_err_t calibration_storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        err = nvs_flash_init();
    }
    return err;
}

bool calibration_load(tracker_calibration_t *calibration)
{
    nvs_handle_t handle;
    if (nvs_open(CAL_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return false;
    size_t size = sizeof(*calibration);
    esp_err_t err = nvs_get_blob(handle, CAL_KEY, calibration, &size);
    nvs_close(handle);
    return err == ESP_OK && size == sizeof(*calibration) &&
           valid_calibration(calibration);
}

static esp_err_t save_calibration(const tracker_calibration_t *calibration)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(CAL_NAMESPACE, NVS_READWRITE, &handle), TAG,
                        "Could not open calibration NVS");
    esp_err_t err = nvs_set_blob(handle, CAL_KEY, calibration,
                                 sizeof(*calibration));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

static void notify(const calibration_remote_io_t *io,
                   tracker_calibration_stage_t stage,
                   tracker_calibration_result_t result,
                   uint8_t elapsed_seconds, uint16_t samples,
                   bool can_finish)
{
    io->notify(io->context, stage, result, elapsed_seconds, samples,
               can_finish);
}

static esp_err_t wait_for_confirmation(
    const calibration_remote_io_t *io, tracker_calibration_stage_t stage,
    tracker_calibration_result_t *failure)
{
    notify(io, stage, TRACKER_CAL_RESULT_NONE, 0, 0, false);
    for (;;) {
        calibration_control_t control =
            io->wait_control(io->context, stage, 1000);
        if (control == CALIBRATION_CONTROL_CONFIRM) return ESP_OK;
        if (control == CALIBRATION_CONTROL_CANCEL) {
            *failure = TRACKER_CAL_RESULT_CANCELLED;
            return ESP_ERR_INVALID_STATE;
        }
        notify(io, stage, TRACKER_CAL_RESULT_NONE, 0, 0, false);
    }
}

static esp_err_t average_mpu(sensors_t *sensors, int samples,
                             float accel[3], float gyro[3], float gyro_sq[3],
                             const calibration_remote_io_t *io,
                             tracker_calibration_stage_t stage,
                             tracker_calibration_result_t *failure)
{
    memset(accel, 0, 3 * sizeof(float));
    memset(gyro, 0, 3 * sizeof(float));
    if (gyro_sq) memset(gyro_sq, 0, 3 * sizeof(float));
    for (int n = 0; n < samples; ++n) {
        if (io->wait_control(io->context, stage, 0) ==
            CALIBRATION_CONTROL_CANCEL) {
            *failure = TRACKER_CAL_RESULT_CANCELLED;
            return ESP_ERR_INVALID_STATE;
        }
        mpu6050_sample_t sample;
        if (sensors_read_mpu(sensors, &sample) != ESP_OK) {
            *failure = TRACKER_CAL_RESULT_SENSOR;
            return ESP_FAIL;
        }
        for (int i = 0; i < 3; ++i) {
            accel[i] += sample.accel_g[i];
            gyro[i] += sample.gyro_dps[i];
            if (gyro_sq) {
                gyro_sq[i] += sample.gyro_dps[i] * sample.gyro_dps[i];
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    for (int i = 0; i < 3; ++i) {
        accel[i] /= samples;
        gyro[i] /= samples;
        if (gyro_sq) gyro_sq[i] /= samples;
    }
    return ESP_OK;
}

static esp_err_t capture_magnetometer(
    sensors_t *sensors, tracker_calibration_t *calibration,
    const calibration_remote_io_t *io, tracker_calibration_result_t *failure)
{
    const gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << TRACKER_CALIBRATION_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&led_config) != ESP_OK ||
        gpio_set_level(TRACKER_CALIBRATION_LED_GPIO, 1) != ESP_OK) {
        *failure = TRACKER_CAL_RESULT_SENSOR;
        return ESP_FAIL;
    }

    esp_err_t result = ESP_OK;
    float minimum[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
    float maximum[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
    uint16_t samples = 0;
    int64_t start = esp_timer_get_time();
    uint8_t last_reported_second = UINT8_MAX;
    while (esp_timer_get_time() - start < 30000000) {
        int64_t elapsed_us = esp_timer_get_time() - start;
        uint8_t elapsed_seconds = (uint8_t)(elapsed_us / 1000000);
        bool can_finish = elapsed_us >= 10000000;
        calibration_control_t control = io->wait_control(
            io->context, TRACKER_CAL_STAGE_CAPTURE_MAG, 0);
        if (control == CALIBRATION_CONTROL_CANCEL) {
            *failure = TRACKER_CAL_RESULT_CANCELLED;
            result = ESP_ERR_INVALID_STATE;
            goto cleanup;
        }
        if (control == CALIBRATION_CONTROL_FINISH && can_finish) break;

        qmc5883p_sample_t sample;
        bool ready = false;
        if (sensors_read_mag(sensors, &sample, &ready) == ESP_OK && ready &&
            !sample.overflow) {
            for (int i = 0; i < 3; ++i) {
                minimum[i] = fminf(minimum[i], sample.magnetic_ut[i]);
                maximum[i] = fmaxf(maximum[i], sample.magnetic_ut[i]);
            }
            if (samples != UINT16_MAX) ++samples;
        }
        if (elapsed_seconds != last_reported_second) {
            notify(io, TRACKER_CAL_STAGE_CAPTURE_MAG, TRACKER_CAL_RESULT_NONE,
                   elapsed_seconds, samples, can_finish);
            last_reported_second = elapsed_seconds;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (samples < 100) {
        *failure = TRACKER_CAL_RESULT_MAG_SAMPLES;
        result = ESP_ERR_INVALID_STATE;
        goto cleanup;
    }
    float radius_sum = 0.0f;
    float radius[3];
    for (int i = 0; i < 3; ++i) {
        float span = maximum[i] - minimum[i];
        calibration->mag_bias_ut[i] = (maximum[i] + minimum[i]) * 0.5f;
        radius[i] = span * 0.5f;
        radius_sum += radius[i];
    }
    float average_radius = radius_sum / 3.0f;
    for (int i = 0; i < 3; ++i) {
        calibration->mag_scale[i] = radius[i] > 1.0e-6f
            ? average_radius / radius[i]
            : NAN;
        ESP_LOGI(TAG, "Mag axis %d: min=%.2f max=%.2f bias=%.2f scale=%.4f",
                 i, minimum[i], maximum[i], calibration->mag_bias_ut[i],
                 calibration->mag_scale[i]);
    }
    if (average_radius < TRACKER_MAG_MIN_UT ||
        average_radius > TRACKER_MAG_MAX_UT) {
        *failure = TRACKER_CAL_RESULT_MAG_RANGE;
        result = ESP_ERR_INVALID_STATE;
    }

cleanup:
    if (gpio_set_level(TRACKER_CALIBRATION_LED_GPIO, 0) != ESP_OK &&
        result == ESP_OK) {
        *failure = TRACKER_CAL_RESULT_SENSOR;
        result = ESP_FAIL;
    }
    return result;
}

esp_err_t calibration_run_wizard(sensors_t *sensors,
                                 const calibration_remote_io_t *io,
                                 tracker_calibration_t *result,
                                 tracker_calibration_result_t *failure)
{
    if (sensors == NULL || io == NULL || io->notify == NULL ||
        io->wait_control == NULL || result == NULL || failure == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *failure = TRACKER_CAL_RESULT_SENSOR;
    tracker_calibration_t c = {
        .magic = CAL_MAGIC,
        .version = CAL_VERSION,
        .size = sizeof(c),
        .accel_scale = {1, 1, 1},
        .mag_scale = {1, 1, 1},
    };

    ESP_RETURN_ON_ERROR(wait_for_confirmation(
                            io, TRACKER_CAL_STAGE_WAIT_STILL, failure),
                        TAG, "Stationary confirmation cancelled");
    notify(io, TRACKER_CAL_STAGE_CAPTURE_STILL, TRACKER_CAL_RESULT_NONE,
           0, 0, false);
    float accel[3], gyro[3], gyro_sq[3];
    ESP_RETURN_ON_ERROR(average_mpu(
                            sensors, 200, accel, gyro, gyro_sq, io,
                            TRACKER_CAL_STAGE_CAPTURE_STILL, failure),
                        TAG, "Stationary calibration failed");
    float anorm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] +
                        accel[2] * accel[2]);
    for (int i = 0; i < 3; ++i) {
        float variance = fmaxf(0.0f, gyro_sq[i] - gyro[i] * gyro[i]);
        if (sqrtf(variance) > 1.5f) {
            *failure = TRACKER_CAL_RESULT_MOTION;
            return ESP_ERR_INVALID_STATE;
        }
        c.gyro_bias_dps[i] = gyro[i];
    }
    if (anorm < 0.8f || anorm > 1.2f) {
        *failure = TRACKER_CAL_RESULT_MOTION;
        return ESP_ERR_INVALID_STATE;
    }

    static const tracker_calibration_stage_t wait_stages[6] = {
        TRACKER_CAL_STAGE_WAIT_POS_X, TRACKER_CAL_STAGE_WAIT_NEG_X,
        TRACKER_CAL_STAGE_WAIT_POS_Y, TRACKER_CAL_STAGE_WAIT_NEG_Y,
        TRACKER_CAL_STAGE_WAIT_POS_Z, TRACKER_CAL_STAGE_WAIT_NEG_Z,
    };
    static const tracker_calibration_stage_t capture_stages[6] = {
        TRACKER_CAL_STAGE_CAPTURE_POS_X, TRACKER_CAL_STAGE_CAPTURE_NEG_X,
        TRACKER_CAL_STAGE_CAPTURE_POS_Y, TRACKER_CAL_STAGE_CAPTURE_NEG_Y,
        TRACKER_CAL_STAGE_CAPTURE_POS_Z, TRACKER_CAL_STAGE_CAPTURE_NEG_Z,
    };
    float face_values[6][3];
    for (int face = 0; face < 6; ++face) {
        ESP_RETURN_ON_ERROR(wait_for_confirmation(io, wait_stages[face],
                                                  failure),
                            TAG, "Face confirmation cancelled");
        notify(io, capture_stages[face], TRACKER_CAL_RESULT_NONE,
               0, 0, false);
        ESP_RETURN_ON_ERROR(average_mpu(
                                sensors, 100, face_values[face], gyro, NULL,
                                io, capture_stages[face], failure),
                            TAG, "Accelerometer face capture failed");
        int axis = face / 2;
        float expected = (face % 2 == 0) ? 1.0f : -1.0f;
        if (face_values[face][axis] * expected < 0.75f) {
            *failure = TRACKER_CAL_RESULT_WRONG_FACE;
            return ESP_ERR_INVALID_STATE;
        }
    }
    for (int i = 0; i < 3; ++i) {
        float positive = face_values[i * 2][i];
        float negative = face_values[i * 2 + 1][i];
        float span = positive - negative;
        if (span < 1.5f) {
            *failure = TRACKER_CAL_RESULT_WRONG_FACE;
            return ESP_ERR_INVALID_STATE;
        }
        c.accel_bias_g[i] = (positive + negative) * 0.5f;
        c.accel_scale[i] = 2.0f / span;
    }

    ESP_RETURN_ON_ERROR(wait_for_confirmation(
                            io, TRACKER_CAL_STAGE_WAIT_MAG, failure),
                        TAG, "Magnetometer confirmation cancelled");
    notify(io, TRACKER_CAL_STAGE_CAPTURE_MAG, TRACKER_CAL_RESULT_NONE,
           0, 0, false);
    ESP_RETURN_ON_ERROR(capture_magnetometer(sensors, &c, io, failure), TAG,
                        "Magnetometer calibration failed");
    if (!valid_calibration(&c)) {
        *failure = TRACKER_CAL_RESULT_MAG_RANGE;
        return ESP_ERR_INVALID_STATE;
    }

    notify(io, TRACKER_CAL_STAGE_SAVING, TRACKER_CAL_RESULT_NONE,
           0, 0, false);
    if (save_calibration(&c) != ESP_OK) {
        *failure = TRACKER_CAL_RESULT_SAVE;
        return ESP_FAIL;
    }
    *result = c;
    *failure = TRACKER_CAL_RESULT_OK;
    return ESP_OK;
}

void calibration_apply_mpu(const tracker_calibration_t *c,
                           const mpu6050_sample_t *input, float accel[3],
                           float gyro[3])
{
    for (int i = 0; i < 3; ++i) {
        accel[i] = (input->accel_g[i] - c->accel_bias_g[i]) *
                   c->accel_scale[i];
        gyro[i] = input->gyro_dps[i] - c->gyro_bias_dps[i];
    }
}

void calibration_apply_mag(const tracker_calibration_t *c,
                           const qmc5883p_sample_t *input, float mag[3])
{
    for (int i = 0; i < 3; ++i) {
        mag[i] = (input->magnetic_ut[i] - c->mag_bias_ut[i]) * c->mag_scale[i];
    }
}
