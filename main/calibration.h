#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sensors.h"
#include "tracker_protocol.h"

#include "calibration_math.h"

typedef enum {
    CALIBRATION_CONTROL_NONE = 0,
    CALIBRATION_CONTROL_CONFIRM,
    CALIBRATION_CONTROL_FINISH,
    CALIBRATION_CONTROL_CANCEL,
} calibration_control_t;

typedef struct {
    void *context;
    void (*notify)(void *context, tracker_calibration_stage_t stage,
                   tracker_calibration_result_t result,
                   uint8_t elapsed_seconds, uint16_t sample_count,
                   bool can_finish);
    calibration_control_t (*wait_control)(
        void *context, tracker_calibration_stage_t expected_stage,
        uint32_t timeout_ms);
} calibration_remote_io_t;

esp_err_t calibration_storage_init(void);
bool calibration_load(tracker_calibration_t *calibration);
esp_err_t calibration_run_wizard(sensors_t *sensors,
                                 const calibration_remote_io_t *io,
                                 tracker_calibration_t *result,
                                 tracker_calibration_result_t *failure);
void calibration_apply_mpu(const tracker_calibration_t *calibration,
                           const mpu6050_sample_t *input,
                           float accel_g[3], float gyro_dps[3]);
void calibration_apply_mag(const tracker_calibration_t *calibration,
                           const qmc5883p_sample_t *input, float mag_ut[3]);
