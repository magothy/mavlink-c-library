/** @file
 *    @brief MAVLink comm protocol testsuite generated from magothy.xml
 *    @see https://mavlink.io/en/
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

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_ECHOSOUNDER") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER) != NULL);
#endif
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
        93372036854775807ULL,963497880,101.0,129.0,157.0,185.0,213.0,241.0,"KLMNOPQRSTUVWXY"
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
        
        mav_array_memcpy(packet1.gcs_set_mode_uuid, packet_in.gcs_set_mode_uuid, sizeof(char)*16);
        
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
    mavlink_msg_magothy_mission_telemetry_pack(system_id, component_id, &msg , packet1.unix_time_usec , packet1.uptime_msec , packet1.mission_item_time_elapsed_s , packet1.mission_item_time_remaining_s , packet1.mission_time_elapsed_s , packet1.mission_time_remaining_s , packet1.distance_to_target_m , packet1.cross_track_error_m , packet1.gcs_set_mode_uuid );
    mavlink_msg_magothy_mission_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_mission_telemetry_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.unix_time_usec , packet1.uptime_msec , packet1.mission_item_time_elapsed_s , packet1.mission_item_time_remaining_s , packet1.mission_time_elapsed_s , packet1.mission_time_remaining_s , packet1.distance_to_target_m , packet1.cross_track_error_m , packet1.gcs_set_mode_uuid );
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
    mavlink_msg_magothy_mission_telemetry_send(MAVLINK_COMM_1 , packet1.unix_time_usec , packet1.uptime_msec , packet1.mission_item_time_elapsed_s , packet1.mission_item_time_remaining_s , packet1.mission_time_elapsed_s , packet1.mission_time_remaining_s , packet1.distance_to_target_m , packet1.cross_track_error_m , packet1.gcs_set_mode_uuid );
    mavlink_msg_magothy_mission_telemetry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_MISSION_TELEMETRY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY) != NULL);
#endif
}

static void mavlink_test_magothy_water_current(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_water_current_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0
    };
    mavlink_magothy_water_current_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.water_current_speed_mps = packet_in.water_current_speed_mps;
        packet1.water_current_direction_deg = packet_in.water_current_direction_deg;
        packet1.water_current_age_s = packet_in.water_current_age_s;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_water_current_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_water_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_water_current_pack(system_id, component_id, &msg , packet1.time_usec , packet1.water_current_speed_mps , packet1.water_current_direction_deg , packet1.water_current_age_s );
    mavlink_msg_magothy_water_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_water_current_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.water_current_speed_mps , packet1.water_current_direction_deg , packet1.water_current_age_s );
    mavlink_msg_magothy_water_current_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_water_current_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_water_current_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.water_current_speed_mps , packet1.water_current_direction_deg , packet1.water_current_age_s );
    mavlink_msg_magothy_water_current_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_WATER_CURRENT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT) != NULL);
#endif
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
        17235,17339,17443,17547,{ 29, 30, 31, 32 },{ 41, 42, 43, 44 },"QRSTUVWXYZABCDEFGHIJKLMNOPQRSTU",{ 149, 150, 151, 152 },"ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE",{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 },61,"BCDEFGHIJKLMNOPQRSTUVWXYZABCDEF","HIJKLMNOPQRSTUV","XYZABCDEFGHIJKLMNOP","RSTUVWXYZABCDEF"
    };
    mavlink_magothy_capability_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.enablement_bitmask = packet_in.enablement_bitmask;
        packet1.spy_port = packet_in.spy_port;
        packet1.log_management_port = packet_in.log_management_port;
        packet1.firmware_upload_port = packet_in.firmware_upload_port;
        packet1.firmware_sha1_dirty = packet_in.firmware_sha1_dirty;
        
        mav_array_memcpy(packet1.spy_ip_address, packet_in.spy_ip_address, sizeof(uint8_t)*4);
        mav_array_memcpy(packet1.log_management_ip_address, packet_in.log_management_ip_address, sizeof(uint8_t)*4);
        mav_array_memcpy(packet1.log_management_endpoint, packet_in.log_management_endpoint, sizeof(char)*32);
        mav_array_memcpy(packet1.firmware_upload_ip_address, packet_in.firmware_upload_ip_address, sizeof(uint8_t)*4);
        mav_array_memcpy(packet1.firmware_upload_endpoint, packet_in.firmware_upload_endpoint, sizeof(char)*32);
        mav_array_memcpy(packet1.firmware_sha1, packet_in.firmware_sha1, sizeof(uint8_t)*20);
        mav_array_memcpy(packet1.firmware_version, packet_in.firmware_version, sizeof(char)*32);
        mav_array_memcpy(packet1.device_type, packet_in.device_type, sizeof(char)*16);
        mav_array_memcpy(packet1.log_date_time, packet_in.log_date_time, sizeof(char)*20);
        mav_array_memcpy(packet1.vehicle_name, packet_in.vehicle_name, sizeof(char)*16);
        
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
    mavlink_msg_magothy_capability_pack(system_id, component_id, &msg , packet1.enablement_bitmask , packet1.spy_ip_address , packet1.spy_port , packet1.log_management_ip_address , packet1.log_management_port , packet1.log_management_endpoint , packet1.firmware_upload_ip_address , packet1.firmware_upload_port , packet1.firmware_upload_endpoint , packet1.firmware_sha1 , packet1.firmware_sha1_dirty , packet1.firmware_version , packet1.device_type , packet1.log_date_time , packet1.vehicle_name );
    mavlink_msg_magothy_capability_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_capability_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.enablement_bitmask , packet1.spy_ip_address , packet1.spy_port , packet1.log_management_ip_address , packet1.log_management_port , packet1.log_management_endpoint , packet1.firmware_upload_ip_address , packet1.firmware_upload_port , packet1.firmware_upload_endpoint , packet1.firmware_sha1 , packet1.firmware_sha1_dirty , packet1.firmware_version , packet1.device_type , packet1.log_date_time , packet1.vehicle_name );
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
    mavlink_msg_magothy_capability_send(MAVLINK_COMM_1 , packet1.enablement_bitmask , packet1.spy_ip_address , packet1.spy_port , packet1.log_management_ip_address , packet1.log_management_port , packet1.log_management_endpoint , packet1.firmware_upload_ip_address , packet1.firmware_upload_port , packet1.firmware_upload_endpoint , packet1.firmware_sha1 , packet1.firmware_sha1_dirty , packet1.firmware_version , packet1.device_type , packet1.log_date_time , packet1.vehicle_name );
    mavlink_msg_magothy_capability_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_CAPABILITY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_CAPABILITY) != NULL);
#endif
}

static void mavlink_test_magothy_3d_mag_cal_params(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_3d_mag_cal_params_t packet_in = {
        { 17.0, 18.0, 19.0 },{ 101.0, 102.0, 103.0, 104.0, 105.0, 106.0, 107.0, 108.0, 109.0 },353.0,381.0,409.0,437.0,465.0,493.0
    };
    mavlink_magothy_3d_mag_cal_params_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.uncalibrated_norm_mean = packet_in.uncalibrated_norm_mean;
        packet1.uncalibrated_norm_std_dev = packet_in.uncalibrated_norm_std_dev;
        packet1.uncalibrated_norm_max_error = packet_in.uncalibrated_norm_max_error;
        packet1.calibrated_norm_mean = packet_in.calibrated_norm_mean;
        packet1.calibrated_norm_std_dev = packet_in.calibrated_norm_std_dev;
        packet1.calibrated_norm_max_error = packet_in.calibrated_norm_max_error;
        
        mav_array_memcpy(packet1.hard_iron, packet_in.hard_iron, sizeof(float)*3);
        mav_array_memcpy(packet1.soft_iron, packet_in.soft_iron, sizeof(float)*9);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_3d_mag_cal_params_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_3d_mag_cal_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_3d_mag_cal_params_pack(system_id, component_id, &msg , packet1.hard_iron , packet1.soft_iron , packet1.uncalibrated_norm_mean , packet1.uncalibrated_norm_std_dev , packet1.uncalibrated_norm_max_error , packet1.calibrated_norm_mean , packet1.calibrated_norm_std_dev , packet1.calibrated_norm_max_error );
    mavlink_msg_magothy_3d_mag_cal_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_3d_mag_cal_params_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.hard_iron , packet1.soft_iron , packet1.uncalibrated_norm_mean , packet1.uncalibrated_norm_std_dev , packet1.uncalibrated_norm_max_error , packet1.calibrated_norm_mean , packet1.calibrated_norm_std_dev , packet1.calibrated_norm_max_error );
    mavlink_msg_magothy_3d_mag_cal_params_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_3d_mag_cal_params_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_3d_mag_cal_params_send(MAVLINK_COMM_1 , packet1.hard_iron , packet1.soft_iron , packet1.uncalibrated_norm_mean , packet1.uncalibrated_norm_std_dev , packet1.uncalibrated_norm_max_error , packet1.calibrated_norm_mean , packet1.calibrated_norm_std_dev , packet1.calibrated_norm_max_error );
    mavlink_msg_magothy_3d_mag_cal_params_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_3D_MAG_CAL_PARAMS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS) != NULL);
#endif
}

static void mavlink_test_magothy_low_bandwidth(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_low_bandwidth_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,963498504,18483,18587,18691,18795,18899,19003,19107,19211,19315,3,70,137,204,963499856
    };
    mavlink_magothy_low_bandwidth_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.custom_mode = packet_in.custom_mode;
        packet1.onboard_control_sensors_present = packet_in.onboard_control_sensors_present;
        packet1.onboard_control_sensors_enabled = packet_in.onboard_control_sensors_enabled;
        packet1.onboard_control_sensors_health = packet_in.onboard_control_sensors_health;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.voltage_battery = packet_in.voltage_battery;
        packet1.current_battery = packet_in.current_battery;
        packet1.mission_seq = packet_in.mission_seq;
        packet1.speed = packet_in.speed;
        packet1.course = packet_in.course;
        packet1.heading = packet_in.heading;
        packet1.position_error = packet_in.position_error;
        packet1.desired_speed = packet_in.desired_speed;
        packet1.desired_course = packet_in.desired_course;
        packet1.type = packet_in.type;
        packet1.battery_remaining = packet_in.battery_remaining;
        packet1.satellites_visible = packet_in.satellites_visible;
        packet1.is_position_independent = packet_in.is_position_independent;
        packet1.gcs_set_mode_uuid_lsb = packet_in.gcs_set_mode_uuid_lsb;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_low_bandwidth_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_low_bandwidth_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_low_bandwidth_pack(system_id, component_id, &msg , packet1.type , packet1.custom_mode , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining , packet1.mission_seq , packet1.lat , packet1.lon , packet1.speed , packet1.course , packet1.satellites_visible , packet1.heading , packet1.is_position_independent , packet1.position_error , packet1.desired_speed , packet1.desired_course , packet1.gcs_set_mode_uuid_lsb );
    mavlink_msg_magothy_low_bandwidth_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_low_bandwidth_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.custom_mode , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining , packet1.mission_seq , packet1.lat , packet1.lon , packet1.speed , packet1.course , packet1.satellites_visible , packet1.heading , packet1.is_position_independent , packet1.position_error , packet1.desired_speed , packet1.desired_course , packet1.gcs_set_mode_uuid_lsb );
    mavlink_msg_magothy_low_bandwidth_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_low_bandwidth_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_low_bandwidth_send(MAVLINK_COMM_1 , packet1.type , packet1.custom_mode , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining , packet1.mission_seq , packet1.lat , packet1.lon , packet1.speed , packet1.course , packet1.satellites_visible , packet1.heading , packet1.is_position_independent , packet1.position_error , packet1.desired_speed , packet1.desired_course , packet1.gcs_set_mode_uuid_lsb );
    mavlink_msg_magothy_low_bandwidth_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_LOW_BANDWIDTH") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH) != NULL);
#endif
}

static void mavlink_test_magothy_protobuf_proxy(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_protobuf_proxy_t packet_in = {
        17235,139,206,{ 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
    };
    mavlink_magothy_protobuf_proxy_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.proto_id = packet_in.proto_id;
        packet1.is_compressed = packet_in.is_compressed;
        packet1.data_len = packet_in.data_len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*251);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_protobuf_proxy_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_protobuf_proxy_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_protobuf_proxy_pack(system_id, component_id, &msg , packet1.proto_id , packet1.is_compressed , packet1.data_len , packet1.data );
    mavlink_msg_magothy_protobuf_proxy_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_protobuf_proxy_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.proto_id , packet1.is_compressed , packet1.data_len , packet1.data );
    mavlink_msg_magothy_protobuf_proxy_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_protobuf_proxy_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_protobuf_proxy_send(MAVLINK_COMM_1 , packet1.proto_id , packet1.is_compressed , packet1.data_len , packet1.data );
    mavlink_msg_magothy_protobuf_proxy_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_PROTOBUF_PROXY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY) != NULL);
#endif
}

static void mavlink_test_magothy_license_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_license_info_t packet_in = {
        5,72,"CDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLM","OPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXY"
    };
    mavlink_magothy_license_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.is_set = packet_in.is_set;
        packet1.is_valid = packet_in.is_valid;
        
        mav_array_memcpy(packet1.product_key, packet_in.product_key, sizeof(char)*64);
        mav_array_memcpy(packet1.description, packet_in.description, sizeof(char)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_license_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_info_pack(system_id, component_id, &msg , packet1.is_set , packet1.is_valid , packet1.product_key , packet1.description );
    mavlink_msg_magothy_license_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.is_set , packet1.is_valid , packet1.product_key , packet1.description );
    mavlink_msg_magothy_license_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_license_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_info_send(MAVLINK_COMM_1 , packet1.is_set , packet1.is_valid , packet1.product_key , packet1.description );
    mavlink_msg_magothy_license_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_LICENSE_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO) != NULL);
#endif
}

static void mavlink_test_magothy_license_transfer_initialize(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_license_transfer_initialize_t packet_in = {
        5,72,"CDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLM","OPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXY"
    };
    mavlink_magothy_license_transfer_initialize_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.transfer_type = packet_in.transfer_type;
        
        mav_array_memcpy(packet1.product_key, packet_in.product_key, sizeof(char)*64);
        mav_array_memcpy(packet1.description, packet_in.description, sizeof(char)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_initialize_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_license_transfer_initialize_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_initialize_pack(system_id, component_id, &msg , packet1.target_system , packet1.transfer_type , packet1.product_key , packet1.description );
    mavlink_msg_magothy_license_transfer_initialize_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_initialize_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.transfer_type , packet1.product_key , packet1.description );
    mavlink_msg_magothy_license_transfer_initialize_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_license_transfer_initialize_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_initialize_send(MAVLINK_COMM_1 , packet1.target_system , packet1.transfer_type , packet1.product_key , packet1.description );
    mavlink_msg_magothy_license_transfer_initialize_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_LICENSE_TRANSFER_INITIALIZE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE) != NULL);
#endif
}

static void mavlink_test_magothy_license_transfer_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_license_transfer_request_t packet_in = {
        5,72,139
    };
    mavlink_magothy_license_transfer_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.transfer_type = packet_in.transfer_type;
        packet1.chunk_index = packet_in.chunk_index;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_license_transfer_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_request_pack(system_id, component_id, &msg , packet1.target_system , packet1.transfer_type , packet1.chunk_index );
    mavlink_msg_magothy_license_transfer_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.transfer_type , packet1.chunk_index );
    mavlink_msg_magothy_license_transfer_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_license_transfer_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_request_send(MAVLINK_COMM_1 , packet1.target_system , packet1.transfer_type , packet1.chunk_index );
    mavlink_msg_magothy_license_transfer_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_LICENSE_TRANSFER_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST) != NULL);
#endif
}

static void mavlink_test_magothy_license_transfer(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_license_transfer_t packet_in = {
        17235,139,206,17,84,151,{ 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209 }
    };
    mavlink_magothy_license_transfer_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.crc16 = packet_in.crc16;
        packet1.target_system = packet_in.target_system;
        packet1.transfer_type = packet_in.transfer_type;
        packet1.chunk_index = packet_in.chunk_index;
        packet1.num_chunk = packet_in.num_chunk;
        packet1.payload_len = packet_in.payload_len;
        
        mav_array_memcpy(packet1.payload, packet_in.payload, sizeof(uint8_t)*248);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_license_transfer_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_pack(system_id, component_id, &msg , packet1.target_system , packet1.transfer_type , packet1.chunk_index , packet1.num_chunk , packet1.crc16 , packet1.payload_len , packet1.payload );
    mavlink_msg_magothy_license_transfer_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.transfer_type , packet1.chunk_index , packet1.num_chunk , packet1.crc16 , packet1.payload_len , packet1.payload );
    mavlink_msg_magothy_license_transfer_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_license_transfer_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_send(MAVLINK_COMM_1 , packet1.target_system , packet1.transfer_type , packet1.chunk_index , packet1.num_chunk , packet1.crc16 , packet1.payload_len , packet1.payload );
    mavlink_msg_magothy_license_transfer_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_LICENSE_TRANSFER") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER) != NULL);
#endif
}

static void mavlink_test_magothy_license_transfer_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_magothy_license_transfer_ack_t packet_in = {
        17235,139,206,17,84,151
    };
    mavlink_magothy_license_transfer_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.crc16 = packet_in.crc16;
        packet1.target_system = packet_in.target_system;
        packet1.transfer_type = packet_in.transfer_type;
        packet1.chunk_index = packet_in.chunk_index;
        packet1.num_chunk = packet_in.num_chunk;
        packet1.result = packet_in.result;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_magothy_license_transfer_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_ack_pack(system_id, component_id, &msg , packet1.target_system , packet1.transfer_type , packet1.chunk_index , packet1.num_chunk , packet1.crc16 , packet1.result );
    mavlink_msg_magothy_license_transfer_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.transfer_type , packet1.chunk_index , packet1.num_chunk , packet1.crc16 , packet1.result );
    mavlink_msg_magothy_license_transfer_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_magothy_license_transfer_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_magothy_license_transfer_ack_send(MAVLINK_COMM_1 , packet1.target_system , packet1.transfer_type , packet1.chunk_index , packet1.num_chunk , packet1.crc16 , packet1.result );
    mavlink_msg_magothy_license_transfer_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAGOTHY_LICENSE_TRANSFER_ACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_ACK) != NULL);
#endif
}

static void mavlink_test_magothy(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_magothy_echosounder(system_id, component_id, last_msg);
    mavlink_test_magothy_mission_telemetry(system_id, component_id, last_msg);
    mavlink_test_magothy_water_current(system_id, component_id, last_msg);
    mavlink_test_magothy_capability(system_id, component_id, last_msg);
    mavlink_test_magothy_3d_mag_cal_params(system_id, component_id, last_msg);
    mavlink_test_magothy_low_bandwidth(system_id, component_id, last_msg);
    mavlink_test_magothy_protobuf_proxy(system_id, component_id, last_msg);
    mavlink_test_magothy_license_info(system_id, component_id, last_msg);
    mavlink_test_magothy_license_transfer_initialize(system_id, component_id, last_msg);
    mavlink_test_magothy_license_transfer_request(system_id, component_id, last_msg);
    mavlink_test_magothy_license_transfer(system_id, component_id, last_msg);
    mavlink_test_magothy_license_transfer_ack(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAGOTHY_TESTSUITE_H
