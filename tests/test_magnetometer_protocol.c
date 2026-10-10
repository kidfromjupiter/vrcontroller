#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "magnetometer_protocol.h"
#ifdef TEST_RECEIVER
#include "receiver_protocol.h"
#endif

int main(void)
{
    mag_command_t command = {MAG_ACTION_START, 0x1234, 30}, decoded;
    uint8_t packet[MAG_COMMAND_SIZE];
    mag_command_encode(packet, &command);
    assert(packet[0] == 5 && packet[1] == 1 && packet[4] == 0x34 && packet[5] == 0x12);
    assert(mag_command_decode(packet, sizeof(packet), &decoded));
    assert(decoded.duration_seconds == 30 && decoded.request_id == 0x1234);
    assert(!mag_command_decode(packet, sizeof(packet) - 1, &decoded));
    assert(!mag_command_decode(NULL, sizeof(packet), &decoded));
    assert(!mag_command_decode(packet, sizeof(packet), NULL));
    packet[1] = 2;
    assert(!mag_command_decode(packet, sizeof(packet), &decoded));
    mag_command_encode(packet, &command);
    packet[3] = 1;
    assert(!mag_command_decode(packet, sizeof(packet), &decoded));
    command.duration_seconds = 301;
    mag_command_encode(packet, &command);
    assert(!mag_command_decode(packet, sizeof(packet), &decoded));
    command.duration_seconds = 4;
    mag_command_encode(packet, &command);
    assert(!mag_command_decode(packet, sizeof(packet), &decoded));
    command.duration_seconds = 30;
    command.request_id = 0;
    mag_command_encode(packet, &command);
    assert(!mag_command_decode(packet, sizeof(packet), &decoded));
    command.request_id = 0x1234;

    mag_sample_t sample = {0x1234, 65535, MAG_FLAG_FRESH | MAG_FLAG_CALIBRATED,
                           0x89abcdef, 50.0f, 42.5f}, roundtrip;
    uint8_t data[MAG_SAMPLE_SIZE];
    mag_sample_encode(data, &sample);
    assert(data[8] == 0xef && data[11] == 0x89);
    assert(data[12] == 0 && data[13] == 0 && data[14] == 0x48 && data[15] == 0x42);
    assert(mag_sample_decode(data, sizeof(data), &roundtrip));
    assert(roundtrip.request_id == sample.request_id && roundtrip.sequence == sample.sequence);
    assert(roundtrip.timestamp_us == sample.timestamp_us && roundtrip.flags == sample.flags);
    assert(roundtrip.raw_ut == 50.0f && roundtrip.calibrated_ut == 42.5f);
    assert(!mag_sample_decode(data, sizeof(data) - 1, &roundtrip));
    assert(!mag_sample_decode(NULL, sizeof(data), &roundtrip));
    data[7] = 0x80;
    assert(!mag_sample_decode(data, sizeof(data), &roundtrip));
    sample.raw_ut = NAN;
    sample.calibrated_ut = INFINITY;
    mag_sample_encode(data, &sample);
    assert(mag_sample_decode(data, sizeof(data), &roundtrip));
    assert(isnan(roundtrip.raw_ut) && isinf(roundtrip.calibrated_ut));

    mag_session_t session = {0};
    assert(!mag_session_active(&session, 100));
    mag_session_command(&session, &command, 100);
    assert(mag_session_active(&session, 200));
    int64_t expiry = session.expires_us;
    session.sequence = 10;
    mag_session_command(&session, &command, 1000000);
    assert(session.expires_us == expiry && session.sequence == 10);
    assert(!mag_session_active(&session, expiry));
    mag_session_command(&session, &command, expiry + 1);
    assert(!mag_session_active(&session, expiry + 2));
    command.request_id++;
    mag_session_command(&session, &command, expiry + 2);
    assert(mag_session_active(&session, expiry + 3) && session.sequence == 0);
    command.action = MAG_ACTION_STOP;
    command.duration_seconds = 0;
    command.request_id++;
    mag_session_command(&session, &command, expiry + 4);
    assert(session.active);
    command.request_id--;
    mag_session_command(&session, &command, expiry + 5);
    assert(!session.active);
    mag_command_encode(packet, &command);
    assert(mag_command_decode(packet, sizeof(packet), &decoded));
    command.duration_seconds = 30;
    mag_command_encode(packet, &command);
    assert(!mag_command_decode(packet, sizeof(packet), &decoded));

#ifdef TEST_RECEIVER
    receiver_session_t receiver;
    receiver_session_init(&receiver);
    const uint8_t macs[3][6] = {{1}, {2}, {3}};
    for (size_t i = 0; i < 2; ++i) {
        size_t slot;
        assert(receiver_session_assign_sender(&receiver, macs[i], &slot) && slot == i);
        uint8_t payload[1 + MAG_SAMPLE_SIZE], frame[FUSION_FRAME_SIZE], type;
        payload[0] = (uint8_t)slot;
        memcpy(payload + 1, data, sizeof(data));
        receiver_build_usb_frame(FUSION_MAG_SAMPLE, payload, sizeof(payload), frame);
        const uint8_t *recovered;
        size_t size;
        assert(receiver_decode_usb_frame(frame, &type, &recovered, &size));
        assert(type == FUSION_MAG_SAMPLE && size == sizeof(payload));
        assert(memcmp(recovered, payload, size) == 0);
        receiver_build_usb_frame(FUSION_MAG_COMMAND, packet, sizeof(packet), frame);
        assert(receiver_decode_usb_frame(frame, &type, &recovered, &size));
        assert(type == FUSION_MAG_COMMAND);
    }
    assert(!receiver_session_assign_sender(&receiver, macs[2], NULL));
#endif
    puts("magnetometer protocol tests passed");
    return 0;
}
