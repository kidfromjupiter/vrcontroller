#pragma once

/* Hardware configuration. */
#define TRACKER_I2C_SDA_GPIO       8
#define TRACKER_I2C_SCL_GPIO       9
#define TRACKER_I2C_CLOCK_HZ       400000
#define TRACKER_CALIBRATION_LED_GPIO 5

/* Fusion configuration. */
#define TRACKER_SAMPLE_RATE_HZ     100
#define TRACKER_MADGWICK_BETA      0.05f
#define TRACKER_MAG_MIN_UT         10.0f
#define TRACKER_MAG_MAX_UT         100.0f
#define TRACKER_MAG_MAX_AGE_US 150000

/* Calibration acceptance limits. Maps are signed permutations into +X right, +Y front, +Z top. */
#define TRACKER_CAL_FACE_DOMINANCE       0.95f
#define TRACKER_CAL_POSE_COSINE          0.94f
/* Alignment uses measured tilt; allow about 32 degrees from the corner diagonal.
 * A face-up pose (cosine 0.577) or wrong octant (at most 0.816) still fails. */
#define TRACKER_CAL_CORNER_COSINE        0.85f
#define TRACKER_CAL_ACCEL_STDDEV_G       0.02f
#define TRACKER_CAL_MAG_STDDEV_UT        1.0f
#define TRACKER_CAL_STILL_GYRO_DPS       5.0f
#define TRACKER_CAL_MAG_MAP_MAX_SCORE    0.05f
#define TRACKER_CAL_MAG_MAP_MARGIN       0.03f
#define TRACKER_CAL_MAG_MAP_RATIO        1.5f

/* ESP-NOW transport configuration. All peers must use the same channel. */
#define TRACKER_ESPNOW_CHANNEL     1

/*
 * Legacy fallback maps; successful v2 calibration learns and saves its own maps.
 * Signed one-based axis maps from each sensor's native frame to the body
 * frame.  1, 2, 3 mean +X, +Y, +Z; negative values invert that axis.
 */
#define TRACKER_IMU_AXIS_MAP       { 1, 2, 3 }
#define TRACKER_MAG_AXIS_MAP       { -1, -2, 3 }
