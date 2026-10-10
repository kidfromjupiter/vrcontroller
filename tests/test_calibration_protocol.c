#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef TEST_RECEIVER
#include "receiver_protocol.h"
#else
#include "tracker_protocol.h"
#endif

int main(void)
{
    for (unsigned corner = 0; corner < 8; ++corner) {
        tracker_calibration_command_t command = {
            .action=TRACKER_CAL_ACTION_CONFIRM,
            .expected_stage=(tracker_calibration_stage_t)(TRACKER_CAL_STAGE_WAIT_CORNER_0 + 2*corner),
            .request_id=0x1234,
        }, decoded;
        uint8_t packet[TRACKER_CALIBRATION_COMMAND_SIZE];
        tracker_calibration_command_encode(packet,&command);
        assert(packet[1] == 2);
        assert(tracker_calibration_command_decode(packet,sizeof(packet),&decoded));
        assert(decoded.expected_stage == command.expected_stage);
        packet[1]=1;
        assert(!tracker_calibration_command_decode(packet,sizeof(packet),&decoded));
        tracker_calibration_status_t status = {
            .stage=(tracker_calibration_stage_t)(TRACKER_CAL_STAGE_CAPTURE_CORNER_0 + 2*corner),
            .result=TRACKER_CAL_RESULT_MAP_AMBIGUOUS,
            .flags=TRACKER_CAL_STATUS_HAS_CALIBRATION | TRACKER_CAL_STATUS_HAS_MAPS,
        }, recovered;
        uint8_t status_packet[TRACKER_CALIBRATION_STATUS_SIZE];
        tracker_calibration_status_encode(status_packet,&status);
        assert(tracker_calibration_status_decode(status_packet,sizeof(status_packet),&recovered));
        assert(recovered.stage == status.stage && recovered.result == status.result);
        status_packet[1]=1;
        assert(!tracker_calibration_status_decode(status_packet,sizeof(status_packet),&recovered));
    }
    tracker_calibration_map_report_t report = {.request_id=0xabcd,
        .imu_axis_map={2,-1,3},.mag_axis_map={-1,-2,3},
        .flags=TRACKER_CAL_STATUS_HAS_CALIBRATION | TRACKER_CAL_STATUS_HAS_MAPS}, decoded;
    uint8_t packet[TRACKER_CALIBRATION_MAP_REPORT_SIZE];
    tracker_calibration_map_report_encode(packet,&report);
    assert(packet[0]==6 && packet[1]==2 && packet[2]==0xcd && packet[3]==0xab);
    assert(packet[5]==255 && packet[7]==255 && packet[8]==254 && packet[11]==0);
    assert(tracker_calibration_map_report_decode(packet,sizeof(packet),&decoded));
    assert(decoded.request_id==report.request_id && decoded.flags==report.flags);
    assert(memcmp(decoded.imu_axis_map,report.imu_axis_map,3)==0);
    assert(memcmp(decoded.mag_axis_map,report.mag_axis_map,3)==0);
    packet[11]=1; assert(!tracker_calibration_map_report_decode(packet,sizeof(packet),&decoded));
    packet[11]=0; packet[4]=0; assert(!tracker_calibration_map_report_decode(packet,sizeof(packet),&decoded));
    tracker_calibration_map_report_encode(packet,&report);
    assert(!tracker_calibration_map_report_decode(packet,sizeof(packet)-1,&decoded));
    assert(!tracker_calibration_map_report_decode(NULL,sizeof(packet),&decoded));
    packet[1]=1; assert(!tracker_calibration_map_report_decode(packet,sizeof(packet),&decoded));
    report.flags=TRACKER_CAL_STATUS_HAS_CALIBRATION;
    tracker_calibration_map_report_encode(packet,&report);
    assert(tracker_calibration_map_report_decode(packet,sizeof(packet),&decoded));
#ifdef TEST_RECEIVER
    for (uint8_t slot=0;slot<2;++slot) {
        uint8_t payload[1+TRACKER_CALIBRATION_MAP_REPORT_SIZE], frame[FUSION_FRAME_SIZE];
        payload[0]=slot; memcpy(payload+1,packet,sizeof(packet));
        receiver_build_usb_frame(FUSION_CALIBRATION_MAP_REPORT,payload,sizeof(payload),frame);
        uint8_t type; const uint8_t *forwarded; size_t size;
        assert(receiver_decode_usb_frame(frame,&type,&forwarded,&size));
        assert(type==0x84 && size==sizeof(payload) && memcmp(forwarded,payload,size)==0);
        frame[29]^=1;
        assert(!receiver_decode_usb_frame(frame,&type,&forwarded,&size));
    }
#endif
    puts("calibration v2 protocol tests passed");
}
