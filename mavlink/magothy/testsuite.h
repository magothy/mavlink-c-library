/** @file
 *    @brief MAVLink comm protocol testsuite generated from magothy.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef MAGOTHY_TESTSUITE_H
#define MAGOTHY_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_magothy(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_magothy(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_mag_cal_progress(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAG_CAL_PROGRESS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mag_cal_progress_t packet_in = {
        17.0,45.0,73.0,41,108,175,242,53,{ 120, 121, 122, 123, 124, 125, 126, 127, 128, 129 }
    };
    mavlink_mag_cal_progress_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.direction_x = packet_in.direction_x;
        packet1.direction_y = packet_in.direction_y;
        packet1.direction_z = packet_in.direction_z;
        packet1.compass_id = packet_in.compass_id;
        packet1.cal_mask = packet_in.cal_mask;
        packet1.cal_status = packet_in.cal_status;
        packet1.attempt = packet_in.attempt;
        packet1.completion_pct = packet_in.completion_pct;
        
        mav_array_memcpy(packet1.completion_mask, packet_in.completion_mask, sizeof(uint8_t)*10);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mag_cal_progress_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_pack(system_id, component_id, &msg , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.attempt , packet1.completion_pct , packet1.completion_mask , packet1.direction_x , packet1.direction_y , packet1.direction_z );
    mavlink_msg_mag_cal_progress_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.attempt , packet1.completion_pct , packet1.completion_mask , packet1.direction_x , packet1.direction_y , packet1.direction_z );
    mavlink_msg_mag_cal_progress_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mag_cal_progress_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_send(MAVLINK_COMM_1 , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.attempt , packet1.completion_pct , packet1.completion_mask , packet1.direction_x , packet1.direction_y , packet1.direction_z );
    mavlink_msg_mag_cal_progress_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mag_cal_report(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAG_CAL_REPORT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mag_cal_report_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,125,192,3,70,325.0,149,216
    };
    mavlink_mag_cal_report_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.fitness = packet_in.fitness;
        packet1.ofs_x = packet_in.ofs_x;
        packet1.ofs_y = packet_in.ofs_y;
        packet1.ofs_z = packet_in.ofs_z;
        packet1.diag_x = packet_in.diag_x;
        packet1.diag_y = packet_in.diag_y;
        packet1.diag_z = packet_in.diag_z;
        packet1.offdiag_x = packet_in.offdiag_x;
        packet1.offdiag_y = packet_in.offdiag_y;
        packet1.offdiag_z = packet_in.offdiag_z;
        packet1.compass_id = packet_in.compass_id;
        packet1.cal_mask = packet_in.cal_mask;
        packet1.cal_status = packet_in.cal_status;
        packet1.autosaved = packet_in.autosaved;
        packet1.orientation_confidence = packet_in.orientation_confidence;
        packet1.old_orientation = packet_in.old_orientation;
        packet1.new_orientation = packet_in.new_orientation;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAG_CAL_REPORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAG_CAL_REPORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_report_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mag_cal_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_report_pack(system_id, component_id, &msg , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.autosaved , packet1.fitness , packet1.ofs_x , packet1.ofs_y , packet1.ofs_z , packet1.diag_x , packet1.diag_y , packet1.diag_z , packet1.offdiag_x , packet1.offdiag_y , packet1.offdiag_z , packet1.orientation_confidence , packet1.old_orientation , packet1.new_orientation );
    mavlink_msg_mag_cal_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_report_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.autosaved , packet1.fitness , packet1.ofs_x , packet1.ofs_y , packet1.ofs_z , packet1.diag_x , packet1.diag_y , packet1.diag_z , packet1.offdiag_x , packet1.offdiag_y , packet1.offdiag_z , packet1.orientation_confidence , packet1.old_orientation , packet1.new_orientation );
    mavlink_msg_mag_cal_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mag_cal_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_report_send(MAVLINK_COMM_1 , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.autosaved , packet1.fitness , packet1.ofs_x , packet1.ofs_y , packet1.ofs_z , packet1.diag_x , packet1.diag_y , packet1.diag_z , packet1.offdiag_x , packet1.offdiag_y , packet1.offdiag_z , packet1.orientation_confidence , packet1.old_orientation , packet1.new_orientation );
    mavlink_msg_mag_cal_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_magothy(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_mag_cal_progress(system_id, component_id, last_msg);
    mavlink_test_mag_cal_report(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAGOTHY_TESTSUITE_H
