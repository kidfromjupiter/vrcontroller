#include "calibration.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/usb_serial_jtag_ll.h"
#include "nvs.h"
#include "nvs_flash.h"

#define CAL_MAGIC   0x56435243U
#define CAL_VERSION 1U
#define CAL_NAMESPACE "sensor_cal"
#define CAL_KEY       "v1"

static const char *TAG = "calibration";
static const uart_port_t CONSOLE_UART = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;

static bool valid_calibration(const tracker_calibration_t *c)
{
    if (c->magic != CAL_MAGIC || c->version != CAL_VERSION ||
        c->size != sizeof(*c)) return false;
    const float *values = c->gyro_bias_dps;
    size_t count = (sizeof(*c) - offsetof(tracker_calibration_t, gyro_bias_dps)) / sizeof(float);
    for (size_t i = 0; i < count; ++i) if (!isfinite(values[i])) return false;
    for (int i = 0; i < 3; ++i) {
        if (c->accel_scale[i] < 0.25f || c->accel_scale[i] > 4.0f ||
            c->mag_scale[i] < 0.1f || c->mag_scale[i] > 10.0f) return false;
    }
    return true;
}

esp_err_t calibration_storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;

    /*
     * Do not read calibration commands through getchar().  The default
     * picolibc stdin backend is not safe to switch between blocking and
     * non-blocking operation on all ESP-IDF console configurations.
     */
    if (!uart_is_driver_installed(CONSOLE_UART)) {
        uart_config_t uart_config = {
            .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        ESP_RETURN_ON_ERROR(uart_param_config(CONSOLE_UART, &uart_config), TAG,
                            "Console UART setup failed");
        ESP_RETURN_ON_ERROR(uart_driver_install(CONSOLE_UART, 256, 0, 0, NULL,
                                                 0), TAG,
                            "Console UART driver install failed");
    }
    return ESP_OK;
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
    esp_err_t err = nvs_set_blob(handle, CAL_KEY, calibration, sizeof(*calibration));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

static void drain_input(void)
{
    (void)uart_flush_input(CONSOLE_UART);
    uint8_t ignored;
    while (usb_serial_jtag_ll_read_rxfifo(&ignored, 1) == 1) {}
}

static int read_console_char(TickType_t timeout)
{
    uint8_t value = 0;
    TickType_t start = xTaskGetTickCount();
    for (;;) {
        if (uart_read_bytes(CONSOLE_UART, &value, 1, 0) == 1 ||
            usb_serial_jtag_ll_read_rxfifo(&value, 1) == 1) {
            return value;
        }
        if (timeout == 0 ||
            (timeout != portMAX_DELAY && xTaskGetTickCount() - start >= timeout)) {
            return -1;
        }
        vTaskDelay(1);
    }
}

static void wait_for_enter(const char *message)
{
    printf("%s Press Enter when ready.\n", message);
    fflush(stdout);
    for (;;) {
        int ch = read_console_char(portMAX_DELAY);
        if (ch == '\n' || ch == '\r') break;
    }
}

bool calibration_recalibration_requested(void)
{
    printf("Enter 'c' within 3 seconds to recalibrate sensors...\n");
    fflush(stdout);
    drain_input();
    int64_t end = esp_timer_get_time() + 3000000;
    bool requested = false;
    while (esp_timer_get_time() < end) {
        int ch = read_console_char(pdMS_TO_TICKS(20));
        if (ch == 'c' || ch == 'C') { requested = true; break; }
    }
    drain_input();
    return requested;
}

static esp_err_t average_mpu(sensors_t *sensors, int samples,
                             float accel[3], float gyro[3], float gyro_sq[3])
{
    memset(accel, 0, 3 * sizeof(float));
    memset(gyro, 0, 3 * sizeof(float));
    if (gyro_sq) memset(gyro_sq, 0, 3 * sizeof(float));
    for (int n = 0; n < samples; ++n) {
        mpu6050_sample_t sample;
        ESP_RETURN_ON_ERROR(sensors_read_mpu(sensors, &sample), TAG,
                            "MPU read failed during calibration");
        for (int i = 0; i < 3; ++i) {
            accel[i] += sample.accel_g[i];
            gyro[i] += sample.gyro_dps[i];
            if (gyro_sq) gyro_sq[i] += sample.gyro_dps[i] * sample.gyro_dps[i];
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

esp_err_t calibration_run_wizard(sensors_t *sensors,
                                 const tracker_calibration_t *previous,
                                 bool has_previous,
                                 tracker_calibration_t *result)
{
    (void)previous;
    (void)has_previous;
    tracker_calibration_t c = {
        .magic = CAL_MAGIC, .version = CAL_VERSION, .size = sizeof(c),
        .accel_scale = {1,1,1}, .mag_scale = {1,1,1},
    };
    printf("\nSensor calibration wizard\n");
    wait_for_enter("Keep the controller completely still on a stable surface.");
    float accel[3], gyro[3], gyro_sq[3];
    ESP_RETURN_ON_ERROR(average_mpu(sensors, 200, accel, gyro, gyro_sq), TAG,
                        "Stationary calibration failed");
    float anorm = sqrtf(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    for (int i = 0; i < 3; ++i) {
        float variance = fmaxf(0.0f, gyro_sq[i] - gyro[i] * gyro[i]);
        if (sqrtf(variance) > 1.5f) {
            ESP_LOGE(TAG, "Too much motion during gyro calibration");
            return ESP_ERR_INVALID_STATE;
        }
        c.gyro_bias_dps[i] = gyro[i];
    }
    if (anorm < 0.8f || anorm > 1.2f) {
        ESP_LOGE(TAG, "Unexpected stationary acceleration %.3f g", anorm);
        return ESP_ERR_INVALID_STATE;
    }

    const char *faces[6] = {"MPU +X upward", "MPU -X upward", "MPU +Y upward",
                            "MPU -Y upward", "MPU +Z upward", "MPU -Z upward"};
    float face_values[6][3];
    for (int face = 0; face < 6; ++face) {
        wait_for_enter(faces[face]);
        ESP_RETURN_ON_ERROR(average_mpu(sensors, 100, face_values[face], gyro,
                                        NULL), TAG, "Accel face capture failed");
        int axis = face / 2;
        float expected = (face % 2 == 0) ? 1.0f : -1.0f;
        if (face_values[face][axis] * expected < 0.75f) {
            ESP_LOGE(TAG, "Wrong or unstable face for %s", faces[face]);
            return ESP_ERR_INVALID_STATE;
        }
    }
    for (int i = 0; i < 3; ++i) {
        float positive = face_values[i * 2][i];
        float negative = face_values[i * 2 + 1][i];
        float span = positive - negative;
        if (span < 1.5f) return ESP_ERR_INVALID_STATE;
        c.accel_bias_g[i] = (positive + negative) * 0.5f;
        c.accel_scale[i] = 2.0f / span;
    }

    printf("Move the controller through broad figure-eights in every orientation for 30 seconds.\n"
           "After 10 seconds, Enter may be used to finish early.\n");
    fflush(stdout);
    drain_input();
    float minimum[3] = {FLT_MAX,FLT_MAX,FLT_MAX};
    float maximum[3] = {-FLT_MAX,-FLT_MAX,-FLT_MAX};
    int samples = 0;
    int64_t start = esp_timer_get_time();
    while (esp_timer_get_time() - start < 30000000) {
        qmc5883p_sample_t sample;
        bool ready = false;
        if (sensors_read_mag(sensors, &sample, &ready) == ESP_OK && ready && !sample.overflow) {
            for (int i = 0; i < 3; ++i) {
                minimum[i] = fminf(minimum[i], sample.magnetic_ut[i]);
                maximum[i] = fmaxf(maximum[i], sample.magnetic_ut[i]);
            }
            ++samples;
        }
        int ch = read_console_char(0);
        if ((ch == '\n' || ch == '\r') && esp_timer_get_time() - start >= 10000000) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (samples < 100) {
        ESP_LOGE(TAG, "Not enough magnetometer samples");
        return ESP_ERR_INVALID_STATE;
    }
    float radius_sum = 0.0f, radius[3];
    for (int i = 0; i < 3; ++i) {
        float span = maximum[i] - minimum[i];
        if (span < 15.0f) {
            ESP_LOGE(TAG, "Insufficient magnetometer movement on axis %d", i);
            return ESP_ERR_INVALID_STATE;
        }
        c.mag_bias_ut[i] = (maximum[i] + minimum[i]) * 0.5f;
        radius[i] = span * 0.5f;
        radius_sum += radius[i];
    }
    float average_radius = radius_sum / 3.0f;
    for (int i = 0; i < 3; ++i) c.mag_scale[i] = average_radius / radius[i];
    if (!valid_calibration(&c)) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(save_calibration(&c), TAG, "Calibration save failed");
    *result = c;
    ESP_LOGI(TAG, "Calibration saved");
    return ESP_OK;
}

void calibration_apply_mpu(const tracker_calibration_t *c,
                           const mpu6050_sample_t *input, float accel[3],
                           float gyro[3])
{
    for (int i = 0; i < 3; ++i) {
        accel[i] = (input->accel_g[i] - c->accel_bias_g[i]) * c->accel_scale[i];
        gyro[i] = input->gyro_dps[i] - c->gyro_bias_dps[i];
    }
}

void calibration_apply_mag(const tracker_calibration_t *c,
                           const qmc5883p_sample_t *input, float mag[3])
{
    for (int i = 0; i < 3; ++i)
        mag[i] = (input->magnetic_ut[i] - c->mag_bias_ut[i]) * c->mag_scale[i];
}
