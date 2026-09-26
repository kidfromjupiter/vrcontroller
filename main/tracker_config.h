#pragma once

/* Hardware configuration. */
#define TRACKER_I2C_SDA_GPIO       8
#define TRACKER_I2C_SCL_GPIO       9
#define TRACKER_I2C_CLOCK_HZ       400000

/* Fusion configuration. */
#define TRACKER_SAMPLE_RATE_HZ     100
#define TRACKER_MADGWICK_BETA      0.05f
#define TRACKER_MAG_MIN_UT         10.0f
#define TRACKER_MAG_MAX_UT         100.0f
#define TRACKER_MAG_MAX_AGE_US     20000

/* ESP-NOW transport configuration. All peers must use the same channel. */
#define TRACKER_ESPNOW_CHANNEL     1

/*
 * Signed one-based axis maps from each sensor's native frame to the body
 * frame.  1, 2, 3 mean +X, +Y, +Z; negative values invert that axis.
 */
#define TRACKER_IMU_AXIS_MAP       { 1, 2, 3 }
#define TRACKER_MAG_AXIS_MAP       { 1, 2, 3 }
