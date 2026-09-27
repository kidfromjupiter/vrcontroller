#include "calibration.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/usb_serial_jtag_ll.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tracker_config.h"

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

bool calibration_magnetometer_boot_requested(void)
{
    const gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << TRACKER_MAG_CAL_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&button_config) != ESP_OK ||
        gpio_get_level(TRACKER_MAG_CAL_BUTTON_GPIO) == 0) {
        return false;
    }

    int64_t start = esp_timer_get_time();
    while (esp_timer_get_time() - start < 2000000) {
        if (gpio_get_level(TRACKER_MAG_CAL_BUTTON_GPIO) == 0) return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "GPIO%d held high; requesting magnetometer calibration",
             TRACKER_MAG_CAL_BUTTON_GPIO);
    return true;
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

calibration_request_t calibration_recalibration_request(void)
{
    printf("Enter 'c' within 3 seconds for full calibration, or 'm' for magnetometer-only calibration...\n");
    fflush(stdout);
    drain_input();
    int64_t end = esp_timer_get_time() + 3000000;
    calibration_request_t request = CALIBRATION_REQUEST_NONE;
    while (esp_timer_get_time() < end) {
        int ch = read_console_char(pdMS_TO_TICKS(20));
        if (ch == 'c' || ch == 'C') {
            request = CALIBRATION_REQUEST_FULL;
            break;
        }
        if (ch == 'm' || ch == 'M') {
            request = CALIBRATION_REQUEST_MAG_ONLY;
            break;
        }
    }
    drain_input();
    return request;
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

static esp_err_t capture_magnetometer(sensors_t *sensors,
                                      tracker_calibration_t *calibration)
{
    const gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << TRACKER_CALIBRATION_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&led_config), TAG,
                        "Calibration LED setup failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(TRACKER_CALIBRATION_LED_GPIO, 1), TAG,
                        "Could not turn on calibration LED");

    esp_err_t result = ESP_OK;
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
        result = ESP_ERR_INVALID_STATE;
        goto cleanup;
    }
    float radius_sum = 0.0f, radius[3], span[3];
    for (int i = 0; i < 3; ++i) {
        span[i] = maximum[i] - minimum[i];
        calibration->mag_bias_ut[i] = (maximum[i] + minimum[i]) * 0.5f;
        radius[i] = span[i] * 0.5f;
        radius_sum += radius[i];
    }
    float average_radius = radius_sum / 3.0f;
    for (int i = 0; i < 3; ++i) {
        calibration->mag_scale[i] = radius[i] > 1.0e-6f
            ? average_radius / radius[i]
            : NAN;
        ESP_LOGI(TAG, "Mag axis %d: min=%.2f max=%.2f span=%.2f bias=%.2f scale=%.4f",
                 i, minimum[i], maximum[i], span[i],
                 calibration->mag_bias_ut[i], calibration->mag_scale[i]);
    }
    ESP_LOGI(TAG, "Mag average radius: %.2f uT", average_radius);
    if (average_radius < TRACKER_MAG_MIN_UT ||
        average_radius > TRACKER_MAG_MAX_UT) {
        ESP_LOGE(TAG, "Magnetometer average radius %.2f uT is outside %.2f-%.2f uT",
                 average_radius, (double)TRACKER_MAG_MIN_UT,
                 (double)TRACKER_MAG_MAX_UT);
        result = ESP_ERR_INVALID_STATE;
    }
cleanup:
    {
        esp_err_t led_err = gpio_set_level(TRACKER_CALIBRATION_LED_GPIO, 0);
        if (led_err != ESP_OK) {
            ESP_LOGE(TAG, "Could not turn off calibration LED: %s",
                     esp_err_to_name(led_err));
            if (result == ESP_OK) result = led_err;
        }
    }
    return result;
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

    ESP_RETURN_ON_ERROR(capture_magnetometer(sensors, &c), TAG,
                        "Magnetometer calibration failed");
    if (!valid_calibration(&c)) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(save_calibration(&c), TAG, "Calibration save failed");
    *result = c;
    ESP_LOGI(TAG, "Calibration saved");
    return ESP_OK;
}

esp_err_t calibration_run_magnetometer(sensors_t *sensors,
                                       const tracker_calibration_t *previous,
                                       tracker_calibration_t *result)
{
    if (!valid_calibration(previous)) return ESP_ERR_INVALID_ARG;

    tracker_calibration_t c = *previous;
    printf("\nMagnetometer-only calibration\n");
    ESP_RETURN_ON_ERROR(capture_magnetometer(sensors, &c), TAG,
                        "Magnetometer calibration failed");
    if (!valid_calibration(&c)) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(save_calibration(&c), TAG, "Calibration save failed");
    *result = c;
    ESP_LOGI(TAG, "Magnetometer calibration saved");
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
