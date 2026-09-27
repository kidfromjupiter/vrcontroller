#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sensors.h"

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float gyro_bias_dps[3];
    float accel_bias_g[3];
    float accel_scale[3];
    float mag_bias_ut[3];
    float mag_scale[3];
} tracker_calibration_t;

typedef enum {
    CALIBRATION_REQUEST_NONE,
    CALIBRATION_REQUEST_FULL,
    CALIBRATION_REQUEST_MAG_ONLY,
} calibration_request_t;

esp_err_t calibration_storage_init(void);
bool calibration_load(tracker_calibration_t *calibration);
bool calibration_magnetometer_boot_requested(void);
calibration_request_t calibration_recalibration_request(void);
esp_err_t calibration_run_wizard(sensors_t *sensors,
                                 const tracker_calibration_t *previous,
                                 bool has_previous,
                                 tracker_calibration_t *result);
esp_err_t calibration_run_magnetometer(sensors_t *sensors,
                                       const tracker_calibration_t *previous,
                                       tracker_calibration_t *result);
void calibration_apply_mpu(const tracker_calibration_t *calibration,
                           const mpu6050_sample_t *input,
                           float accel_g[3], float gyro_dps[3]);
void calibration_apply_mag(const tracker_calibration_t *calibration,
                           const qmc5883p_sample_t *input, float mag_ut[3]);
