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

#define CAL_NAMESPACE "sensor_cal"
#define CAL_KEY_V1 "v1"
#define CAL_KEY_V2 "v2"

static const char *TAG = "calibration";

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
    tracker_calibration_t current;
    size_t size = sizeof(current);
    esp_err_t err = nvs_get_blob(handle, CAL_KEY_V2, &current, &size);
    bool loaded = err == ESP_OK && size == sizeof(current) &&
        calibration_decode_blob(&current, size, calibration);
    if (!loaded) {
        tracker_calibration_v1_t legacy;
        size = sizeof(legacy);
        err = nvs_get_blob(handle, CAL_KEY_V1, &legacy, &size);
        loaded = err == ESP_OK && size == sizeof(legacy) &&
            calibration_decode_blob(&legacy, size, calibration);
        if (loaded) ESP_LOGW(TAG, "Legacy calibration: using configured axis maps; rerun the wizard to learn maps");
    }
    nvs_close(handle);
    return loaded;
}

static esp_err_t save_calibration(const tracker_calibration_t *calibration)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(CAL_NAMESPACE, NVS_READWRITE, &handle), TAG,
                        "Could not open calibration NVS");
    esp_err_t err = nvs_set_blob(handle, CAL_KEY_V2, calibration,
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

static esp_err_t wait_for_confirmation_result(
    const calibration_remote_io_t *io, tracker_calibration_stage_t stage,
    tracker_calibration_result_t reason,
    tracker_calibration_result_t *failure)
{
    notify(io, stage, reason, 0, 0, false);
    for (;;) {
        calibration_control_t control =
            io->wait_control(io->context, stage, 1000);
        if (control == CALIBRATION_CONTROL_CONFIRM) return ESP_OK;
        if (control == CALIBRATION_CONTROL_CANCEL) {
            *failure = TRACKER_CAL_RESULT_CANCELLED;
            return ESP_ERR_INVALID_STATE;
        }
        notify(io, stage, reason, 0, 0, false);
    }
}

static esp_err_t wait_for_confirmation(
    const calibration_remote_io_t *io, tracker_calibration_stage_t stage,
    tracker_calibration_result_t *failure)
{
    return wait_for_confirmation_result(io, stage, TRACKER_CAL_RESULT_NONE, failure);
}

static esp_err_t average_mpu(sensors_t *sensors, int samples,
                             float accel[3], float gyro[3], float gyro_sq[3],
                             const float gyro_bias[3],
                             const calibration_remote_io_t *io,
                             tracker_calibration_stage_t stage,
                             tracker_calibration_result_t *failure)
{
    float accel_sq[3] = {0};
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
            accel_sq[i] += sample.accel_g[i] * sample.accel_g[i];
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
        float variance = fmaxf(0, accel_sq[i] / samples - accel[i]*accel[i]);
        if (!isfinite(accel[i]) || !isfinite(gyro[i]) ||
            sqrtf(variance) > TRACKER_CAL_ACCEL_STDDEV_G ||
            (gyro_bias && fabsf(gyro[i] - gyro_bias[i]) > TRACKER_CAL_STILL_GYRO_DPS)) {
            *failure = TRACKER_CAL_RESULT_MOTION;
            return ESP_ERR_INVALID_STATE;
        }
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

static esp_err_t capture_corner(
    sensors_t *sensors, const tracker_calibration_t *c, unsigned corner,
    const calibration_remote_io_t *io, float accel_body[3], float mag_native[3],
    tracker_calibration_result_t *failure)
{
    tracker_calibration_stage_t stage = (tracker_calibration_stage_t)
        (TRACKER_CAL_STAGE_CAPTURE_CORNER_0 + 2 * corner);
    float a_sum[3] = {0}, a_sq[3] = {0}, m_sum[3] = {0}, m_sq[3] = {0};
    unsigned samples = 0;
    int64_t start = esp_timer_get_time();
    uint8_t reported_second = UINT8_MAX;
    /* Let placement motion settle before evaluating the stationary hold. */
    vTaskDelay(pdMS_TO_TICKS(200));
    while (samples < 100 && esp_timer_get_time() - start < 5000000) {
        if (io->wait_control(io->context, stage, 0) == CALIBRATION_CONTROL_CANCEL) {
            *failure = TRACKER_CAL_RESULT_CANCELLED;
            return ESP_ERR_INVALID_STATE;
        }
        mpu6050_sample_t imu;
        qmc5883p_sample_t mag;
        bool ready = false;
        if (sensors_read_mpu(sensors, &imu) != ESP_OK ||
            sensors_read_mag(sensors, &mag, &ready) != ESP_OK) {
            *failure = TRACKER_CAL_RESULT_SENSOR;
            return ESP_FAIL;
        }
        for (int i = 0; i < 3; ++i) {
            if (!isfinite(imu.gyro_dps[i]) ||
                fabsf(imu.gyro_dps[i] - c->gyro_bias_dps[i]) > TRACKER_CAL_STILL_GYRO_DPS) {
                *failure = TRACKER_CAL_RESULT_MOTION;
                return ESP_ERR_INVALID_STATE;
            }
        }
        if (ready && !mag.overflow) {
            float corrected_accel[3], corrected_mag[3], body[3];
            calibration_correct_vector(imu.accel_g, c->accel_bias_g, c->accel_scale, corrected_accel);
            calibration_map_vector(corrected_accel, c->imu_axis_map, body);
            calibration_apply_mag(c, &mag, corrected_mag);
            float mnorm = sqrtf(corrected_mag[0]*corrected_mag[0] +
                                corrected_mag[1]*corrected_mag[1] + corrected_mag[2]*corrected_mag[2]);
            if (!isfinite(mnorm) || mnorm < TRACKER_MAG_MIN_UT || mnorm > TRACKER_MAG_MAX_UT) {
                *failure = TRACKER_CAL_RESULT_MAG_RANGE;
                return ESP_ERR_INVALID_STATE;
            }
            for (int i = 0; i < 3; ++i) {
                a_sum[i] += body[i]; a_sq[i] += body[i]*body[i];
                m_sum[i] += corrected_mag[i]; m_sq[i] += corrected_mag[i]*corrected_mag[i];
            }
            ++samples;
        }
        uint8_t second = (uint8_t)((esp_timer_get_time() - start) / 1000000);
        if (second != reported_second) {
            notify(io, stage, TRACKER_CAL_RESULT_NONE, second, (uint16_t)samples, false);
            reported_second = second;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (samples < 100) {
        *failure = TRACKER_CAL_RESULT_MAG_SAMPLES;
        return ESP_ERR_INVALID_STATE;
    }
    for (int i = 0; i < 3; ++i) {
        accel_body[i] = a_sum[i] / samples;
        mag_native[i] = m_sum[i] / samples;
        float a_variance = fmaxf(0, a_sq[i] / samples - accel_body[i]*accel_body[i]);
        float m_variance = fmaxf(0, m_sq[i] / samples - mag_native[i]*mag_native[i]);
        if (!isfinite(accel_body[i]) || sqrtf(a_variance) > TRACKER_CAL_ACCEL_STDDEV_G ||
            sqrtf(m_variance) > TRACKER_CAL_MAG_STDDEV_UT) {
            *failure = TRACKER_CAL_RESULT_MOTION;
            return ESP_ERR_INVALID_STATE;
        }
    }
    float expected[3];
    calibration_corner_direction(corner, expected);
    float length = sqrtf(accel_body[0]*accel_body[0] + accel_body[1]*accel_body[1] +
                         accel_body[2]*accel_body[2]);
    float cosine = (accel_body[0]*expected[0] + accel_body[1]*expected[1] +
                    accel_body[2]*expected[2]) / length;
    if (!isfinite(cosine) || length < 0.9f || length > 1.1f || cosine < TRACKER_CAL_CORNER_COSINE) {
        ESP_LOGW(TAG, "Corner %u pose rejected: body accel={%.3f,%.3f,%.3f} g norm=%.3f cosine=%.3f (min %.2f)",
                 corner, accel_body[0], accel_body[1], accel_body[2], length, cosine,
                 TRACKER_CAL_CORNER_COSINE);
        *failure = TRACKER_CAL_RESULT_WRONG_FACE;
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
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
    tracker_calibration_t c;
    calibration_defaults(&c);

    float accel[3], gyro[3], gyro_sq[3];
    tracker_calibration_result_t retry_reason = TRACKER_CAL_RESULT_NONE;
    for (;;) {
        ESP_RETURN_ON_ERROR(wait_for_confirmation_result(
                                io, TRACKER_CAL_STAGE_WAIT_STILL, retry_reason, failure),
                            TAG, "Stationary confirmation cancelled");
        notify(io, TRACKER_CAL_STAGE_CAPTURE_STILL, TRACKER_CAL_RESULT_NONE, 0, 0, false);
        esp_err_t err = average_mpu(
                            sensors, 200, accel, gyro, gyro_sq, NULL, io,
                            TRACKER_CAL_STAGE_CAPTURE_STILL, failure);
        if (err != ESP_OK && *failure != TRACKER_CAL_RESULT_MOTION) return err;
        if (err == ESP_OK) {
            float anorm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
            if (!isfinite(anorm) || anorm < 0.8f || anorm > 1.2f) err = ESP_ERR_INVALID_STATE;
            for (int i = 0; i < 3; ++i) {
                float variance = fmaxf(0.0f, gyro_sq[i] - gyro[i]*gyro[i]);
                if (sqrtf(variance) > 1.5f) err = ESP_ERR_INVALID_STATE;
            }
            if (err == ESP_OK) {
                memcpy(c.gyro_bias_dps, gyro, sizeof(gyro));
                break;
            }
        }
        retry_reason = TRACKER_CAL_RESULT_MOTION;
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
    retry_reason = TRACKER_CAL_RESULT_NONE;
    for (;;) {
        for (int face = 0; face < 6; ++face) {
            for (;;) {
                ESP_RETURN_ON_ERROR(wait_for_confirmation_result(
                                        io, wait_stages[face], retry_reason, failure),
                                    TAG, "Face confirmation cancelled");
                notify(io, capture_stages[face], TRACKER_CAL_RESULT_NONE, 0, 0, false);
                esp_err_t err = average_mpu(
                    sensors, 100, face_values[face], gyro, NULL,
                    c.gyro_bias_dps, io, capture_stages[face], failure);
                if (err == ESP_OK) {
                    retry_reason = TRACKER_CAL_RESULT_NONE;
                    break;
                }
                if (*failure != TRACKER_CAL_RESULT_MOTION) return err;
                retry_reason = *failure;
            }
        }
        *failure = calibration_fit_accelerometer(face_values, &c);
        if (*failure == TRACKER_CAL_RESULT_OK) break;
        if (*failure != TRACKER_CAL_RESULT_WRONG_FACE &&
            *failure != TRACKER_CAL_RESULT_MAP_INVALID) return ESP_ERR_INVALID_STATE;
        /* The fit checks opposite-face pairs together, so repeat all six faces. */
        retry_reason = *failure;
    }

    ESP_RETURN_ON_ERROR(wait_for_confirmation(
                            io, TRACKER_CAL_STAGE_WAIT_MAG, failure),
                        TAG, "Magnetometer confirmation cancelled");
    notify(io, TRACKER_CAL_STAGE_CAPTURE_MAG, TRACKER_CAL_RESULT_NONE,
           0, 0, false);
    ESP_RETURN_ON_ERROR(capture_magnetometer(sensors, &c, io, failure), TAG,
                        "Magnetometer calibration failed");
    if (!calibration_valid(&c)) {
        *failure = TRACKER_CAL_RESULT_MAG_RANGE;
        return ESP_ERR_INVALID_STATE;
    }

    float alignment_accel[CALIBRATION_CORNER_COUNT][3];
    float alignment_mag[CALIBRATION_CORNER_COUNT][3];
    for (unsigned corner = 0; corner < CALIBRATION_CORNER_COUNT; ++corner) {
        tracker_calibration_stage_t wait = (tracker_calibration_stage_t)
            (TRACKER_CAL_STAGE_WAIT_CORNER_0 + 2 * corner);
        tracker_calibration_stage_t capture = (tracker_calibration_stage_t)
            (TRACKER_CAL_STAGE_CAPTURE_CORNER_0 + 2 * corner);
        retry_reason = TRACKER_CAL_RESULT_NONE;
        for (;;) {
            ESP_RETURN_ON_ERROR(wait_for_confirmation_result(
                                    io, wait, retry_reason, failure),
                                TAG, "Alignment confirmation cancelled");
            notify(io, capture, TRACKER_CAL_RESULT_NONE, 0, 0, false);
            esp_err_t err = capture_corner(sensors, &c, corner, io,
                alignment_accel[corner], alignment_mag[corner], failure);
            if (err == ESP_OK) break;
            if (*failure != TRACKER_CAL_RESULT_WRONG_FACE &&
                *failure != TRACKER_CAL_RESULT_MOTION) return err;
            retry_reason = *failure;
        }
    }
    float best_score = 0, runner_up_score = 0;
    *failure = calibration_fit_mag_map(alignment_accel, alignment_mag,
        CALIBRATION_CORNER_COUNT, c.mag_axis_map, &best_score, &runner_up_score);
    ESP_LOGI(TAG, "Mag map scores: best=%.4f runner_up=%.4f", best_score, runner_up_score);
    if (*failure != TRACKER_CAL_RESULT_OK) return ESP_ERR_INVALID_STATE;
    c.flags = CALIBRATION_HAS_MAPS;
    if (!calibration_valid(&c)) {
        *failure = TRACKER_CAL_RESULT_MAP_INVALID;
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "Maps: MPU={%d,%d,%d} mag={%d,%d,%d}",
        c.imu_axis_map[0], c.imu_axis_map[1], c.imu_axis_map[2],
        c.mag_axis_map[0], c.mag_axis_map[1], c.mag_axis_map[2]);

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
    calibration_correct_vector(input->accel_g, c->accel_bias_g, c->accel_scale, accel);
    for (int i = 0; i < 3; ++i) gyro[i] = input->gyro_dps[i] - c->gyro_bias_dps[i];
}

void calibration_apply_mag(const tracker_calibration_t *c,
                           const qmc5883p_sample_t *input, float mag[3])
{
    calibration_correct_vector(input->magnetic_ut, c->mag_bias_ut, c->mag_scale, mag);
}
