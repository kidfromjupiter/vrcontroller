#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAG_PACKET_SAMPLE 4U
#define MAG_PACKET_COMMAND 5U
#define MAG_PROTOCOL_VERSION 1U
#define MAG_COMMAND_SIZE 8U
#define MAG_SAMPLE_SIZE 20U
#define MAG_ACTION_START 1U
#define MAG_ACTION_STOP 2U
#define MAG_FLAG_FRESH 0x01U
#define MAG_FLAG_SENSOR_ERROR 0x02U
#define MAG_FLAG_OVERFLOW 0x04U
#define MAG_FLAG_OUT_OF_RANGE 0x08U
#define MAG_FLAG_CALIBRATED 0x10U

typedef struct {
    uint8_t action;
    uint16_t request_id;
    uint16_t duration_seconds;
} mag_command_t;

typedef struct {
    uint16_t request_id;
    uint16_t sequence;
    uint16_t flags;
    uint32_t timestamp_us;
    float raw_ut;
    float calibrated_ut;
} mag_sample_t;

typedef struct {
    bool active;
    bool seen_request;
    uint16_t request_id;
    uint16_t sequence;
    int64_t expires_us;
} mag_session_t;

void mag_command_encode(uint8_t output[MAG_COMMAND_SIZE], const mag_command_t *command);
bool mag_command_decode(const uint8_t *data, size_t size, mag_command_t *command);
void mag_sample_encode(uint8_t output[MAG_SAMPLE_SIZE], const mag_sample_t *sample);
bool mag_sample_decode(const uint8_t *data, size_t size, mag_sample_t *sample);
void mag_session_command(mag_session_t *session, const mag_command_t *command, int64_t now_us);
bool mag_session_active(mag_session_t *session, int64_t now_us);
