#pragma once

#include <stdint.h>

#define TRACKER_PACKET_SIZE 24U
#define TRACKER_PACKET_ID_QUATERNION 1U
#define TRACKER_PACKET_VERSION 1U

#define TRACKER_QUATERNION_SCALE 32767.0f
#define TRACKER_ACCELERATION_SCALE 2048.0f

void tracker_packet_encode(uint8_t output[TRACKER_PACKET_SIZE],
                           uint16_t sequence, uint32_t timestamp_us,
                           const float quaternion[4],
                           const float acceleration_g[3], uint16_t status);
