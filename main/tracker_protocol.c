#include "tracker_protocol.h"

#include <limits.h>
#include <math.h>
#include <string.h>

static void put_u16(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8);
    output[2] = (uint8_t)(value >> 16);
    output[3] = (uint8_t)(value >> 24);
}

static int16_t quantize(float value, float scale)
{
    if (!isfinite(value)) {
        return 0;
    }

    float scaled = value * scale;
    if (scaled >= (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (scaled <= (float)INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)lroundf(scaled);
}

void tracker_packet_encode(uint8_t output[TRACKER_PACKET_SIZE],
                           uint16_t sequence, uint32_t timestamp_us,
                           const float quaternion[4],
                           const float acceleration_g[3], uint16_t status)
{
    memset(output, 0, TRACKER_PACKET_SIZE);
    output[0] = TRACKER_PACKET_ID_QUATERNION;
    output[1] = TRACKER_PACKET_VERSION;
    put_u16(&output[2], sequence);
    put_u32(&output[4], timestamp_us);

    for (size_t i = 0; i < 4; ++i) {
        put_u16(&output[8 + i * 2],
                (uint16_t)quantize(quaternion[i],
                                   TRACKER_QUATERNION_SCALE));
    }
    for (size_t i = 0; i < 3; ++i) {
        put_u16(&output[16 + i * 2],
                (uint16_t)quantize(acceleration_g[i],
                                   TRACKER_ACCELERATION_SCALE));
    }
    put_u16(&output[22], status);
}

static uint16_t get_u16(const uint8_t *input)
{
    return (uint16_t)input[0] | ((uint16_t)input[1] << 8);
}

void tracker_calibration_command_encode(
    uint8_t output[TRACKER_CALIBRATION_COMMAND_SIZE],
    const tracker_calibration_command_t *command)
{
    memset(output, 0, TRACKER_CALIBRATION_COMMAND_SIZE);
    output[0] = TRACKER_PACKET_ID_CALIBRATION_COMMAND;
    output[1] = TRACKER_CALIBRATION_PACKET_VERSION;
    output[2] = (uint8_t)command->action;
    output[3] = (uint8_t)command->expected_stage;
    put_u16(&output[4], command->request_id);
}

bool tracker_calibration_command_decode(
    const uint8_t *data, size_t size, tracker_calibration_command_t *command)
{
    if (data == NULL || command == NULL ||
        size != TRACKER_CALIBRATION_COMMAND_SIZE ||
        data[0] != TRACKER_PACKET_ID_CALIBRATION_COMMAND ||
        data[1] != TRACKER_CALIBRATION_PACKET_VERSION ||
        data[2] < TRACKER_CAL_ACTION_QUERY ||
        data[2] > TRACKER_CAL_ACTION_CANCEL ||
        data[3] > TRACKER_CAL_STAGE_LAST || data[6] != 0 || data[7] != 0) {
        return false;
    }
    command->action = (tracker_calibration_action_t)data[2];
    command->expected_stage = (tracker_calibration_stage_t)data[3];
    command->request_id = get_u16(&data[4]);
    return true;
}

void tracker_calibration_status_encode(
    uint8_t output[TRACKER_CALIBRATION_STATUS_SIZE],
    const tracker_calibration_status_t *status)
{
    memset(output, 0, TRACKER_CALIBRATION_STATUS_SIZE);
    output[0] = TRACKER_PACKET_ID_CALIBRATION_STATUS;
    output[1] = TRACKER_CALIBRATION_PACKET_VERSION;
    output[2] = (uint8_t)status->stage;
    output[3] = (uint8_t)status->result;
    output[4] = status->flags;
    output[5] = status->elapsed_seconds;
    put_u16(&output[6], status->request_id);
    put_u16(&output[8], status->sample_count);
}

bool tracker_calibration_status_decode(
    const uint8_t *data, size_t size, tracker_calibration_status_t *status)
{
    if (data == NULL || status == NULL ||
        size != TRACKER_CALIBRATION_STATUS_SIZE ||
        data[0] != TRACKER_PACKET_ID_CALIBRATION_STATUS ||
        data[1] != TRACKER_CALIBRATION_PACKET_VERSION ||
        data[2] > TRACKER_CAL_STAGE_LAST ||
        data[3] > TRACKER_CAL_RESULT_MAP_ALIGNMENT ||
        (data[4] & ~0x07U) || data[10] != 0 || data[11] != 0) {
        return false;
    }
    status->stage = (tracker_calibration_stage_t)data[2];
    status->result = (tracker_calibration_result_t)data[3];
    status->flags = data[4];
    status->elapsed_seconds = data[5];
    status->request_id = get_u16(&data[6]);
    status->sample_count = get_u16(&data[8]);
    return true;
}

bool tracker_axis_map_valid(const int8_t map[3])
{
    if (map == NULL) return false;
    unsigned used = 0;
    int axes[3], determinant = 1;
    for (int i = 0; i < 3; ++i) {
        axes[i] = map[i] < 0 ? -map[i] : map[i];
        if (axes[i] < 1 || axes[i] > 3 || (used & (1U << axes[i]))) return false;
        used |= 1U << axes[i];
        if (map[i] < 0) determinant = -determinant;
    }
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j)
            if (axes[i] > axes[j]) determinant = -determinant;
    return determinant == 1;
}

void tracker_calibration_map_report_encode(
    uint8_t output[TRACKER_CALIBRATION_MAP_REPORT_SIZE],
    const tracker_calibration_map_report_t *report)
{
    memset(output, 0, TRACKER_CALIBRATION_MAP_REPORT_SIZE);
    output[0] = TRACKER_PACKET_ID_CALIBRATION_MAP_REPORT;
    output[1] = TRACKER_CALIBRATION_PACKET_VERSION;
    put_u16(&output[2], report->request_id);
    for (int i = 0; i < 3; ++i) {
        output[4 + i] = (uint8_t)report->imu_axis_map[i];
        output[7 + i] = (uint8_t)report->mag_axis_map[i];
    }
    output[10] = report->flags;
}

bool tracker_calibration_map_report_decode(
    const uint8_t *data, size_t size, tracker_calibration_map_report_t *report)
{
    if (data == NULL || report == NULL || size != TRACKER_CALIBRATION_MAP_REPORT_SIZE ||
        data[0] != TRACKER_PACKET_ID_CALIBRATION_MAP_REPORT ||
        data[1] != TRACKER_CALIBRATION_PACKET_VERSION || data[11] != 0 ||
        (data[10] & ~(TRACKER_CAL_STATUS_HAS_CALIBRATION | TRACKER_CAL_STATUS_HAS_MAPS)) ||
        !(data[10] & TRACKER_CAL_STATUS_HAS_CALIBRATION)) return false;
    tracker_calibration_map_report_t decoded = {.request_id = get_u16(&data[2]), .flags = data[10]};
    for (int i = 0; i < 3; ++i) {
        decoded.imu_axis_map[i] = (int8_t)data[4 + i];
        decoded.mag_axis_map[i] = (int8_t)data[7 + i];
    }
    if (!tracker_axis_map_valid(decoded.imu_axis_map) ||
        !tracker_axis_map_valid(decoded.mag_axis_map)) return false;
    *report = decoded;
    return true;
}
