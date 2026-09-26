#pragma once

#include <stdbool.h>

typedef struct {
    float q[4]; /* W, X, Y, Z */
    float beta;
} madgwick_t;

void madgwick_init(madgwick_t *filter, float beta);
void madgwick_seed(madgwick_t *filter, const float accel[3],
                   const float mag[3], bool use_mag);
bool madgwick_update(madgwick_t *filter, const float gyro_rad_s[3],
                     const float accel[3], const float mag[3],
                     bool use_mag, float dt);
bool madgwick_update_gyro(madgwick_t *filter, const float gyro_rad_s[3],
                          float dt);

