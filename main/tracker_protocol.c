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
