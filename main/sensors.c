#include "sensors.h"

#include <stddef.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tracker_config.h"

#define I2C_TIMEOUT_MS             100
#define MPU6050_ADDR_LOW           0x68
#define MPU6050_ADDR_HIGH          0x69
#define MPU6050_REG_SMPLRT_DIV     0x19
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_GYRO_CFG       0x1B
#define MPU6050_REG_ACCEL_CFG      0x1C
#define MPU6050_REG_DATA           0x3B
#define MPU6050_REG_PWR_MGMT1      0x6B
#define MPU6050_REG_WHO_AM_I       0x75
#define MPU6050_WHO_AM_I           0x68

#define QMC5883P_ADDR              0x2C
#define QMC5883P_REG_CHIP_ID       0x00
#define QMC5883P_REG_DATA          0x01
#define QMC5883P_REG_STATUS        0x09
#define QMC5883P_REG_CTRL1         0x0A
#define QMC5883P_REG_CTRL2         0x0B
#define QMC5883P_CHIP_ID           0x80
#define QMC5883P_STATUS_DRDY       BIT(0)
#define QMC5883P_STATUS_OVFL       BIT(1)

/* OSR2=8, OSR1=8, ODR=100 Hz, continuous mode. */
#define QMC5883P_CTRL1_VALUE       ((2U << 2) | 3U)
/* +/-8 gauss, set/reset on. */
#define QMC5883P_CTRL2_VALUE       (2U << 2)
#define QMC5883P_8G_LSB_PER_GAUSS  3750.0f

static const char *TAG = "sensors";

static esp_err_t write_reg(i2c_master_dev_handle_t device, uint8_t reg,
                           uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(device, data, sizeof(data), I2C_TIMEOUT_MS);
}

static esp_err_t read_regs(i2c_master_dev_handle_t device, uint8_t first_reg,
                           uint8_t *data, size_t size)
{
    return i2c_master_transmit_receive(device, &first_reg, 1, data, size,
                                       I2C_TIMEOUT_MS);
}

static esp_err_t add_device(i2c_master_bus_handle_t bus, uint8_t address,
                            i2c_master_dev_handle_t *device)
{
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = TRACKER_I2C_CLOCK_HZ,
    };
    return i2c_master_bus_add_device(bus, &config, device);
}

static int16_t be16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t le16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

esp_err_t sensors_init(sensors_t *sensors)
{
    memset(sensors, 0, sizeof(*sensors));
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)TRACKER_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)TRACKER_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &sensors->bus), TAG,
                        "Could not create I2C bus");

    uint8_t address = MPU6050_ADDR_LOW;
    esp_err_t err = i2c_master_probe(sensors->bus, address, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        address = MPU6050_ADDR_HIGH;
        err = i2c_master_probe(sensors->bus, address, I2C_TIMEOUT_MS);
    }
    ESP_RETURN_ON_ERROR(err, TAG, "MPU6050 not found at 0x68 or 0x69");
    ESP_RETURN_ON_ERROR(add_device(sensors->bus, address, &sensors->mpu6050),
                        TAG, "Could not add MPU6050");

    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(read_regs(sensors->mpu6050, MPU6050_REG_WHO_AM_I,
                                  &id, 1), TAG, "MPU6050 identity read failed");
    if ((id & 0x7eU) != MPU6050_WHO_AM_I) {
        ESP_LOGE(TAG, "Unexpected MPU6050 WHO_AM_I 0x%02x", id);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_RETURN_ON_ERROR(write_reg(sensors->mpu6050, MPU6050_REG_PWR_MGMT1,
                                  0x80), TAG, "MPU6050 reset failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(write_reg(sensors->mpu6050, MPU6050_REG_PWR_MGMT1,
                                  0x01), TAG, "MPU6050 wake failed");
    ESP_RETURN_ON_ERROR(write_reg(sensors->mpu6050, MPU6050_REG_SMPLRT_DIV,
                                  9), TAG, "MPU6050 rate setup failed");
    ESP_RETURN_ON_ERROR(write_reg(sensors->mpu6050, MPU6050_REG_CONFIG, 3),
                        TAG, "MPU6050 filter setup failed");
    ESP_RETURN_ON_ERROR(write_reg(sensors->mpu6050, MPU6050_REG_GYRO_CFG,
                                  3U << 3), TAG, "MPU6050 gyro setup failed");
    ESP_RETURN_ON_ERROR(write_reg(sensors->mpu6050, MPU6050_REG_ACCEL_CFG,
                                  2U << 3), TAG, "MPU6050 accel setup failed");
    sensors->mpu6050_address = address;

    ESP_RETURN_ON_ERROR(i2c_master_probe(sensors->bus, QMC5883P_ADDR,
                                         I2C_TIMEOUT_MS), TAG,
                        "QMC5883P not found at 0x2c");
    ESP_RETURN_ON_ERROR(add_device(sensors->bus, QMC5883P_ADDR,
                                   &sensors->qmc5883p), TAG,
                        "Could not add QMC5883P");
    ESP_RETURN_ON_ERROR(read_regs(sensors->qmc5883p, QMC5883P_REG_CHIP_ID,
                                  &id, 1), TAG, "QMC5883P identity read failed");
    if (id != QMC5883P_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected QMC5883P chip ID 0x%02x", id);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_RETURN_ON_ERROR(write_reg(sensors->qmc5883p, QMC5883P_REG_CTRL2,
                                  BIT(7)), TAG, "QMC5883P reset failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(write_reg(sensors->qmc5883p, QMC5883P_REG_CTRL2,
                                  QMC5883P_CTRL2_VALUE), TAG,
                        "QMC5883P range setup failed");
    ESP_RETURN_ON_ERROR(write_reg(sensors->qmc5883p, QMC5883P_REG_CTRL1,
                                  QMC5883P_CTRL1_VALUE), TAG,
                        "QMC5883P mode setup failed");

    ESP_LOGI(TAG, "MPU6050=0x%02x QMC5883P=0x%02x SDA=%d SCL=%d",
             address, QMC5883P_ADDR, TRACKER_I2C_SDA_GPIO,
             TRACKER_I2C_SCL_GPIO);
    return ESP_OK;
}

esp_err_t sensors_read_mpu(sensors_t *sensors, mpu6050_sample_t *sample)
{
    uint8_t data[14];
    ESP_RETURN_ON_ERROR(read_regs(sensors->mpu6050, MPU6050_REG_DATA, data,
                                  sizeof(data)), TAG, "MPU6050 read failed");
    sample->accel_g[0] = be16(&data[0]) / 4096.0f;
    sample->accel_g[1] = be16(&data[2]) / 4096.0f;
    sample->accel_g[2] = be16(&data[4]) / 4096.0f;
    sample->temperature_c = be16(&data[6]) / 340.0f + 36.53f;
    sample->gyro_dps[0] = be16(&data[8]) / 16.4f;
    sample->gyro_dps[1] = be16(&data[10]) / 16.4f;
    sample->gyro_dps[2] = be16(&data[12]) / 16.4f;
    return ESP_OK;
}

esp_err_t sensors_read_mag(sensors_t *sensors, qmc5883p_sample_t *sample,
                           bool *data_ready)
{
    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(read_regs(sensors->qmc5883p, QMC5883P_REG_STATUS,
                                  &status, 1), TAG, "QMC5883P status failed");
    *data_ready = (status & QMC5883P_STATUS_DRDY) != 0;
    sample->overflow = (status & QMC5883P_STATUS_OVFL) != 0;
    if (!*data_ready) {
        return ESP_OK;
    }
    uint8_t data[6];
    ESP_RETURN_ON_ERROR(read_regs(sensors->qmc5883p, QMC5883P_REG_DATA, data,
                                  sizeof(data)), TAG, "QMC5883P read failed");
    const float ut_per_lsb = 100.0f / QMC5883P_8G_LSB_PER_GAUSS;
    sample->magnetic_ut[0] = le16(&data[0]) * ut_per_lsb;
    sample->magnetic_ut[1] = le16(&data[2]) * ut_per_lsb;
    sample->magnetic_ut[2] = le16(&data[4]) * ut_per_lsb;
    return ESP_OK;
}

void sensors_apply_axis_map(const float input[3], const int8_t map[3],
                            float output[3])
{
    for (int i = 0; i < 3; ++i) {
        int source = map[i];
        int index = (source < 0 ? -source : source) - 1;
        output[i] = (source < 0 ? -1.0f : 1.0f) * input[index];
    }
}
