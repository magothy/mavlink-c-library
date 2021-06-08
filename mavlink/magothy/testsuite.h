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


static void mavlink_test_magothy_echosounder(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_echosounder_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0
    };
    mavlink_magothy_echosounder_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.water_depth_low_freq_m = packet_in.water_depth_low_freq_m;
        packet1.water_depth_high_freq_m = packet_in.water_depth_high_freq_m;
        packet1.water_temperature_C = packet_in.water_temperature_C;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_echosounder_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_echosounder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_echosounder_pack(system_id, component_id, &msg , packet1.time_usec , packet1.water_depth_low_freq_m , packet1.water_depth_high_freq_m , packet1.water_temperature_C );
    mavlink_msg_magothy_echosounder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_echosounder_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.water_depth_low_freq_m , packet1.water_depth_high_freq_m , packet1.water_temperature_C );
    mavlink_msg_magothy_echosounder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_echosounder_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_echosounder_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.water_depth_low_freq_m , packet1.water_depth_high_freq_m , packet1.water_temperature_C );
    mavlink_msg_magothy_echosounder_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_magothy_mission_telemetry(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_mission_telemetry_t packet_in = {
        93372036854775807ULL,963497880,101.0,129.0,157.0,185.0,213.0,241.0
    };
    mavlink_magothy_mission_telemetry_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.unix_time_usec = packet_in.unix_time_usec;
        packet1.uptime_msec = packet_in.uptime_msec;
        packet1.mission_item_time_elapsed_s = packet_in.mission_item_time_elapsed_s;
        packet1.mission_item_time_remaining_s = packet_in.mission_item_time_remaining_s;
        packet1.mission_time_elapsed_s = packet_in.mission_time_elapsed_s;
        packet1.mission_time_remaining_s = packet_in.mission_time_remaining_s;
        packet1.distance_to_target_m = packet_in.distance_to_target_m;
        packet1.cross_track_error_m = packet_in.cross_track_error_m;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_mission_telemetry_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_mission_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_mission_telemetry_pack(system_id, component_id, &msg , packet1.unix_time_usec , packet1.uptime_msec , packet1.mission_item_time_elapsed_s , packet1.mission_item_time_remaining_s , packet1.mission_time_elapsed_s , packet1.mission_time_remaining_s , packet1.distance_to_target_m , packet1.cross_track_error_m );
    mavlink_msg_magothy_mission_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_mission_telemetry_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.unix_time_usec , packet1.uptime_msec , packet1.mission_item_time_elapsed_s , packet1.mission_item_time_remaining_s , packet1.mission_time_elapsed_s , packet1.mission_time_remaining_s , packet1.distance_to_target_m , packet1.cross_track_error_m );
    mavlink_msg_magothy_mission_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_mission_telemetry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_mission_telemetry_send(MAVLINK_COMM_1 , packet1.unix_time_usec , packet1.uptime_msec , packet1.mission_item_time_elapsed_s , packet1.mission_item_time_remaining_s , packet1.mission_time_elapsed_s , packet1.mission_time_remaining_s , packet1.distance_to_target_m , packet1.cross_track_error_m );
    mavlink_msg_magothy_mission_telemetry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_magothy_capability(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_CAPABILITY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_capability_t packet_in = {
        17235,17339,17443,{ 151, 152, 153, 154 },"KLMNOPQRSTUVWXYZABCDEFGHIJKLMNO",{ 3, 4, 5, 6 },"UVWXYZABCDEFGHIJKLMNOPQRSTUVWXY",{ 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130 },171,"VWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ","BCDEFGHIJKLMNOP"
    };
    mavlink_magothy_capability_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.enablement_bitmask = packet_in.enablement_bitmask;
        packet1.log_management_port = packet_in.log_management_port;
        packet1.firmware_upload_port = packet_in.firmware_upload_port;
        packet1.firmware_sha1_dirty = packet_in.firmware_sha1_dirty;
        
        mav_array_memcpy(packet1.log_management_ip_address, packet_in.log_management_ip_address, sizeof(uint8_t)*4);
        mav_array_memcpy(packet1.log_management_endpoint, packet_in.log_management_endpoint, sizeof(char)*32);
        mav_array_memcpy(packet1.firmware_upload_ip_address, packet_in.firmware_upload_ip_address, sizeof(uint8_t)*4);
        mav_array_memcpy(packet1.firmware_upload_endpoint, packet_in.firmware_upload_endpoint, sizeof(char)*32);
        mav_array_memcpy(packet1.firmware_sha1, packet_in.firmware_sha1, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.firmware_version, packet_in.firmware_version, sizeof(char)*32);
        mav_array_memcpy(packet1.device_type, packet_in.device_type, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_capability_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_capability_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_capability_pack(system_id, component_id, &msg , packet1.enablement_bitmask , packet1.log_management_ip_address , packet1.log_management_port , packet1.log_management_endpoint , packet1.firmware_upload_ip_address , packet1.firmware_upload_port , packet1.firmware_upload_endpoint , packet1.firmware_sha1 , packet1.firmware_sha1_dirty , packet1.firmware_version , packet1.device_type );
    mavlink_msg_magothy_capability_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_capability_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.enablement_bitmask , packet1.log_management_ip_address , packet1.log_management_port , packet1.log_management_endpoint , packet1.firmware_upload_ip_address , packet1.firmware_upload_port , packet1.firmware_upload_endpoint , packet1.firmware_sha1 , packet1.firmware_sha1_dirty , packet1.firmware_version , packet1.device_type );
    mavlink_msg_magothy_capability_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_capability_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_capability_send(MAVLINK_COMM_1 , packet1.enablement_bitmask , packet1.log_management_ip_address , packet1.log_management_port , packet1.log_management_endpoint , packet1.firmware_upload_ip_address , packet1.firmware_upload_port , packet1.firmware_upload_endpoint , packet1.firmware_sha1 , packet1.firmware_sha1_dirty , packet1.firmware_version , packet1.device_type );
    mavlink_msg_magothy_capability_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_magothy(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_magothy_echosounder(system_id, component_id, last_msg);
    mavlink_test_magothy_mission_telemetry(system_id, component_id, last_msg);
    mavlink_test_magothy_capability(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAGOTHY_TESTSUITE_H
