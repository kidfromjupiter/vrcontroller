#include "magnetometer_protocol.h"

#include <string.h>

_Static_assert(sizeof(float) == 4, "magnetometer protocol needs float32");

static void put16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void put32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(value >> (8 * i));
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)p[0] | (uint16_t)p[1] << 8;
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
           (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void put_float(uint8_t *p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put32(p, bits);
}

static float get_float(const uint8_t *p)
{
    uint32_t bits = get32(p);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

void mag_command_encode(uint8_t output[MAG_COMMAND_SIZE], const mag_command_t *command)
{
    memset(output, 0, MAG_COMMAND_SIZE);
    output[0] = MAG_PACKET_COMMAND;
    output[1] = MAG_PROTOCOL_VERSION;
    output[2] = command->action;
    put16(&output[4], command->request_id);
    put16(&output[6], command->duration_seconds);
}

bool mag_command_decode(const uint8_t *data, size_t size, mag_command_t *command)
{
    if (!data || !command || size != MAG_COMMAND_SIZE ||
        data[0] != MAG_PACKET_COMMAND || data[1] != MAG_PROTOCOL_VERSION ||
        data[3] != 0 || get16(&data[4]) == 0 ||
        (data[2] != MAG_ACTION_START && data[2] != MAG_ACTION_STOP) ||
        (data[2] == MAG_ACTION_START &&
         (get16(&data[6]) < 5 || get16(&data[6]) > 300)) ||
        (data[2] == MAG_ACTION_STOP && get16(&data[6]) != 0)) return false;
    command->action = data[2];
    command->request_id = get16(&data[4]);
    command->duration_seconds = get16(&data[6]);
    return true;
}

void mag_sample_encode(uint8_t output[MAG_SAMPLE_SIZE], const mag_sample_t *sample)
{
    output[0] = MAG_PACKET_SAMPLE;
    output[1] = MAG_PROTOCOL_VERSION;
    put16(&output[2], sample->request_id);
    put16(&output[4], sample->sequence);
    put16(&output[6], sample->flags);
    put32(&output[8], sample->timestamp_us);
    put_float(&output[12], sample->raw_ut);
    put_float(&output[16], sample->calibrated_ut);
}

bool mag_sample_decode(const uint8_t *data, size_t size, mag_sample_t *sample)
{
    if (!data || !sample || size != MAG_SAMPLE_SIZE ||
        data[0] != MAG_PACKET_SAMPLE || data[1] != MAG_PROTOCOL_VERSION ||
        get16(&data[2]) == 0 || (get16(&data[6]) & ~0x1fU) != 0) return false;
    sample->request_id = get16(&data[2]);
    sample->sequence = get16(&data[4]);
    sample->flags = get16(&data[6]);
    sample->timestamp_us = get32(&data[8]);
    sample->raw_ut = get_float(&data[12]);
    sample->calibrated_ut = get_float(&data[16]);
    return true;
}

void mag_session_command(mag_session_t *session, const mag_command_t *command, int64_t now_us)
{
    if (command->action == MAG_ACTION_STOP) {
        if (session->seen_request && session->request_id == command->request_id)
            session->active = false;
    } else if (command->action == MAG_ACTION_START &&
               !(session->seen_request && session->request_id == command->request_id)) {
        session->active = true;
        session->seen_request = true;
        session->request_id = command->request_id;
        session->sequence = 0;
        session->expires_us = now_us + (int64_t)command->duration_seconds * 1000000;
    }
}

bool mag_session_active(mag_session_t *session, int64_t now_us)
{
    if (now_us >= session->expires_us) session->active = false;
    return session->active;
}
