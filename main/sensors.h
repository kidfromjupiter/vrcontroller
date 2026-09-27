#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    float accel_g[3];
    float gyro_dps[3];
    float temperature_c;
} mpu6050_sample_t;

typedef struct {
    float magnetic_ut[3];
    bool overflow;
} qmc5883p_sample_t;

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t mpu6050;
    i2c_master_dev_handle_t qmc5883p;
    uint8_t mpu6050_address;
} sensors_t;

esp_err_t sensors_init(sensors_t *sensors);
esp_err_t sensors_read_mpu(sensors_t *sensors, mpu6050_sample_t *sample);
esp_err_t sensors_read_mag(sensors_t *sensors, qmc5883p_sample_t *sample,
                           bool *data_ready);
void sensors_apply_axis_map(const float input[3], const int8_t map[3],
                            float output[3]);
