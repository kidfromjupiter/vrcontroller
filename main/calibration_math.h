#pragma once

#include <stddef.h>
#include "tracker_protocol.h"

#define CALIBRATION_MAGIC 0x56435243U
#define CALIBRATION_VERSION 2U
#define CALIBRATION_HAS_MAPS 0x01U
#define CALIBRATION_CORNER_COUNT 8U

/* Legacy layout is kept verbatim for the NVS v1 reader. Coefficients are native-axis values. */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float gyro_bias_dps[3];
    float accel_bias_g[3];
    float accel_scale[3];
    float mag_bias_ut[3];
    float mag_scale[3];
} tracker_calibration_v1_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    float gyro_bias_dps[3];
    float accel_bias_g[3];
    float accel_scale[3];
    float mag_bias_ut[3];
    float mag_scale[3];
    int8_t imu_axis_map[3];
    int8_t mag_axis_map[3];
    uint8_t flags;
    uint8_t reserved;
} tracker_calibration_t;

void calibration_defaults(tracker_calibration_t *calibration);
bool calibration_valid(const tracker_calibration_t *calibration);
bool calibration_decode_blob(const void *data, size_t size, tracker_calibration_t *calibration);
void calibration_correct_vector(const float input[3], const float bias[3],
                                const float scale[3], float output[3]);
void calibration_map_vector(const float input[3], const int8_t map[3], float output[3]);
void calibration_corner_direction(unsigned corner, float direction[3]);
tracker_calibration_result_t calibration_fit_accelerometer(
    const float faces[6][3], tracker_calibration_t *calibration);
tracker_calibration_result_t calibration_fit_mag_map(
    const float acceleration[][3], const float magnetic[][3], size_t count,
    int8_t map[3], float *best_score, float *runner_up_score);
