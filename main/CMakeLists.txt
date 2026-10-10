idf_component_register(
    SRCS
        "vrcontroller.c"
        "sensors.c"
        "calibration.c"
        "madgwick.c"
        "tracker_protocol.c"
        "magnetometer_protocol.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_i2c
        esp_event
        esp_netif
        esp_wifi
        nvs_flash
        esp_timer
)
