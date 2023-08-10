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

static void mavlink_test_magothy(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_magothy_echosounder(system_id, component_id, last_msg);
    mavlink_test_magothy_mission_telemetry(system_id, component_id, last_msg);
    mavlink_test_magothy_water_current(system_id, component_id, last_msg);
    mavlink_test_magothy_capability(system_id, component_id, last_msg);
    mavlink_test_magothy_3d_mag_cal_params(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAGOTHY_TESTSUITE_H
