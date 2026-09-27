#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "madgwick.h"
#include "tracker_protocol.h"

static uint16_t get_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void test_tracker_packet(void)
{
    const float quaternion[4] = {1.0f, -1.0f, 0.5f, -0.5f};
    const float acceleration[3] = {1.0f, -1.0f, 20.0f};
    uint8_t packet[TRACKER_PACKET_SIZE];

    tracker_packet_encode(packet, 0x1234U, 0x89abcdefU, quaternion,
                          acceleration, 0x0013U);
    assert(packet[0] == TRACKER_PACKET_ID_QUATERNION);
    assert(packet[1] == TRACKER_PACKET_VERSION);
    assert(get_u16(&packet[2]) == 0x1234U);
    assert(get_u32(&packet[4]) == 0x89abcdefU);
    assert((int16_t)get_u16(&packet[8]) == 32767);
    assert((int16_t)get_u16(&packet[10]) == -32767);
    assert((int16_t)get_u16(&packet[12]) == 16384);
    assert((int16_t)get_u16(&packet[14]) == -16384);
    assert((int16_t)get_u16(&packet[16]) == 2048);
    assert((int16_t)get_u16(&packet[18]) == -2048);
    assert((int16_t)get_u16(&packet[20]) == 32767);
    assert(get_u16(&packet[22]) == 0x0013U);
}

static void test_tracker_packet_invalid_and_saturated_values(void)
{
    const float quaternion[4] = {NAN, INFINITY, -INFINITY, 2.0f};
    const float acceleration[3] = {-20.0f, NAN, 0.25f};
    uint8_t packet[TRACKER_PACKET_SIZE];

    tracker_packet_encode(packet, 0xffffU, 0xffffffffU, quaternion,
                          acceleration, 0);
    assert((int16_t)get_u16(&packet[8]) == 0);
    assert((int16_t)get_u16(&packet[10]) == 0);
    assert((int16_t)get_u16(&packet[12]) == 0);
    assert((int16_t)get_u16(&packet[14]) == 32767);
    assert((int16_t)get_u16(&packet[16]) == -32768);
    assert((int16_t)get_u16(&packet[18]) == 0);
    assert((int16_t)get_u16(&packet[20]) == 512);
    assert(get_u16(&packet[22]) == 0);
}

int main(void)
{
    test_tracker_packet();
    test_tracker_packet_invalid_and_saturated_values();

    madgwick_t filter;
    madgwick_init(&filter, 0.05f);
    float gyro[3] = {0, 0, (float)M_PI / 2.0f};
    for (int i = 0; i < 100; ++i)
        assert(madgwick_update_gyro(&filter, gyro, 0.01f));
    float norm = sqrtf(filter.q[0]*filter.q[0]+filter.q[1]*filter.q[1]
                     +filter.q[2]*filter.q[2]+filter.q[3]*filter.q[3]);
    assert(fabsf(norm - 1.0f) < 1e-5f);
    assert(fabsf(fabsf(filter.q[3]) - 0.7071f) < 0.01f);

    float zero[3] = {0,0,0};
    assert(madgwick_update(&filter, zero, zero, zero, true, 0.01f));
    assert(isfinite(filter.q[0]));

    float gravity[3] = {0,0,1}, north[3] = {1,0,0}, still[3] = {0,0,0};
    madgwick_init(&filter, 0.05f);
    for (int i = 0; i < 100; ++i)
        assert(madgwick_update(&filter, still, gravity, north, true, 0.01f));
    assert(isfinite(filter.q[0]));
    puts("tracker tests passed");
    return 0;
}
