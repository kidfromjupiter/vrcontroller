#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TRACKER_PACKET_SIZE 24U
#define TRACKER_PACKET_ID_QUATERNION 1U
#define TRACKER_PACKET_ID_CALIBRATION_COMMAND 2U
#define TRACKER_PACKET_ID_CALIBRATION_STATUS 3U
#define TRACKER_PACKET_VERSION 1U

#define TRACKER_CALIBRATION_COMMAND_SIZE 8U
#define TRACKER_CALIBRATION_STATUS_SIZE 12U

#define TRACKER_QUATERNION_SCALE 32767.0f
#define TRACKER_ACCELERATION_SCALE 2048.0f

typedef enum {
    TRACKER_CAL_ACTION_QUERY = 1,
    TRACKER_CAL_ACTION_START_FULL,
    TRACKER_CAL_ACTION_CONFIRM,
    TRACKER_CAL_ACTION_FINISH_MAG,
    TRACKER_CAL_ACTION_CANCEL,
} tracker_calibration_action_t;

typedef enum {
    TRACKER_CAL_STAGE_IDLE = 0,
    TRACKER_CAL_STAGE_WAIT_STILL,
    TRACKER_CAL_STAGE_CAPTURE_STILL,
    TRACKER_CAL_STAGE_WAIT_POS_X,
    TRACKER_CAL_STAGE_CAPTURE_POS_X,
    TRACKER_CAL_STAGE_WAIT_NEG_X,
    TRACKER_CAL_STAGE_CAPTURE_NEG_X,
    TRACKER_CAL_STAGE_WAIT_POS_Y,
    TRACKER_CAL_STAGE_CAPTURE_POS_Y,
    TRACKER_CAL_STAGE_WAIT_NEG_Y,
    TRACKER_CAL_STAGE_CAPTURE_NEG_Y,
    TRACKER_CAL_STAGE_WAIT_POS_Z,
    TRACKER_CAL_STAGE_CAPTURE_POS_Z,
    TRACKER_CAL_STAGE_WAIT_NEG_Z,
    TRACKER_CAL_STAGE_CAPTURE_NEG_Z,
    TRACKER_CAL_STAGE_WAIT_MAG,
    TRACKER_CAL_STAGE_CAPTURE_MAG,
    TRACKER_CAL_STAGE_SAVING,
    TRACKER_CAL_STAGE_COMPLETE,
    TRACKER_CAL_STAGE_FAILED,
    TRACKER_CAL_STAGE_CANCELLED,
} tracker_calibration_stage_t;

typedef enum {
    TRACKER_CAL_RESULT_NONE = 0,
    TRACKER_CAL_RESULT_OK,
    TRACKER_CAL_RESULT_BUSY,
    TRACKER_CAL_RESULT_INVALID_STAGE,
    TRACKER_CAL_RESULT_SENSOR,
    TRACKER_CAL_RESULT_MOTION,
    TRACKER_CAL_RESULT_WRONG_FACE,
    TRACKER_CAL_RESULT_MAG_SAMPLES,
    TRACKER_CAL_RESULT_MAG_RANGE,
    TRACKER_CAL_RESULT_SAVE,
    TRACKER_CAL_RESULT_CANCELLED,
    TRACKER_CAL_RESULT_UNAVAILABLE,
    TRACKER_CAL_RESULT_TIMEOUT,
    TRACKER_CAL_RESULT_TRANSPORT,
} tracker_calibration_result_t;

#define TRACKER_CAL_STATUS_HAS_CALIBRATION 0x01U
#define TRACKER_CAL_STATUS_CAN_FINISH       0x02U

typedef struct {
    tracker_calibration_action_t action;
    tracker_calibration_stage_t expected_stage;
    uint16_t request_id;
} tracker_calibration_command_t;

typedef struct {
    tracker_calibration_stage_t stage;
    tracker_calibration_result_t result;
    uint8_t flags;
    uint8_t elapsed_seconds;
    uint16_t request_id;
    uint16_t sample_count;
} tracker_calibration_status_t;

void tracker_packet_encode(uint8_t output[TRACKER_PACKET_SIZE],
                           uint16_t sequence, uint32_t timestamp_us,
                           const float quaternion[4],
                           const float acceleration_g[3], uint16_t status);

void tracker_calibration_command_encode(
    uint8_t output[TRACKER_CALIBRATION_COMMAND_SIZE],
    const tracker_calibration_command_t *command);
bool tracker_calibration_command_decode(
    const uint8_t *data, size_t size, tracker_calibration_command_t *command);
void tracker_calibration_status_encode(
    uint8_t output[TRACKER_CALIBRATION_STATUS_SIZE],
    const tracker_calibration_status_t *status);
bool tracker_calibration_status_decode(
    const uint8_t *data, size_t size, tracker_calibration_status_t *status);
