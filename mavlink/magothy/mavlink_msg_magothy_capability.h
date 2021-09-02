#pragma once
// MESSAGE MAGOTHY_CAPABILITY PACKING

#define MAVLINK_MSG_ID_MAGOTHY_CAPABILITY 50100


typedef struct __mavlink_magothy_capability_t {
 uint16_t enablement_bitmask; /*<  Additional Capability Bitmask*/
 uint16_t spy_port; /*<  Port to Spy server*/
 uint16_t log_management_port; /*<  Port to Log WebDAV server*/
 uint16_t firmware_upload_port; /*<  Port to Firmware Update web server*/
 uint8_t spy_ip_address[4]; /*<  IP Address to Spy server*/
 uint8_t log_management_ip_address[4]; /*<  IP Address to Log WebDAV server*/
 char log_management_endpoint[32]; /*<  Base URL prefix to Log WebDAV server*/
 uint8_t firmware_upload_ip_address[4]; /*<  IP Address to Firmware Update web server*/
 char firmware_upload_endpoint[32]; /*<  Base URL prefix to Firmware Update web server*/
 uint8_t firmware_sha1[20]; /*<  sha1 of meta-magothy*/
 uint8_t firmware_sha1_dirty; /*<  sha1 is clean if 0, else dirty*/
 char firmware_version[32]; /*<  Pretty version string*/
 char device_type[16]; /*<  Identifier to distinguish between vehicle types*/
 char log_date_time[20]; /*<  Log file date time prefix*/
 char vehicle_name[16]; /*<  Vehicle Name - Log file suffix*/
} mavlink_magothy_capability_t;

#define MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN 189
#define MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN 189
#define MAVLINK_MSG_ID_50100_LEN 189
#define MAVLINK_MSG_ID_50100_MIN_LEN 189

#define MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_CRC 126
#define MAVLINK_MSG_ID_50100_CRC 126

#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_SPY_IP_ADDRESS_LEN 4
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_LOG_MANAGEMENT_IP_ADDRESS_LEN 4
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_LOG_MANAGEMENT_ENDPOINT_LEN 32
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_FIRMWARE_UPLOAD_IP_ADDRESS_LEN 4
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_FIRMWARE_UPLOAD_ENDPOINT_LEN 32
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_FIRMWARE_SHA1_LEN 20
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_FIRMWARE_VERSION_LEN 32
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_DEVICE_TYPE_LEN 16
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_LOG_DATE_TIME_LEN 20
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_VEHICLE_NAME_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_CAPABILITY { \
    50100, \
    "MAGOTHY_CAPABILITY", \
    15, \
    {  { "enablement_bitmask", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_magothy_capability_t, enablement_bitmask) }, \
         { "spy_ip_address", NULL, MAVLINK_TYPE_UINT8_T, 4, 8, offsetof(mavlink_magothy_capability_t, spy_ip_address) }, \
         { "spy_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_magothy_capability_t, spy_port) }, \
         { "log_management_ip_address", NULL, MAVLINK_TYPE_UINT8_T, 4, 12, offsetof(mavlink_magothy_capability_t, log_management_ip_address) }, \
         { "log_management_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_magothy_capability_t, log_management_port) }, \
         { "log_management_endpoint", NULL, MAVLINK_TYPE_CHAR, 32, 16, offsetof(mavlink_magothy_capability_t, log_management_endpoint) }, \
         { "firmware_upload_ip_address", NULL, MAVLINK_TYPE_UINT8_T, 4, 48, offsetof(mavlink_magothy_capability_t, firmware_upload_ip_address) }, \
         { "firmware_upload_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_magothy_capability_t, firmware_upload_port) }, \
         { "firmware_upload_endpoint", NULL, MAVLINK_TYPE_CHAR, 32, 52, offsetof(mavlink_magothy_capability_t, firmware_upload_endpoint) }, \
         { "firmware_sha1", NULL, MAVLINK_TYPE_UINT8_T, 20, 84, offsetof(mavlink_magothy_capability_t, firmware_sha1) }, \
         { "firmware_sha1_dirty", NULL, MAVLINK_TYPE_UINT8_T, 0, 104, offsetof(mavlink_magothy_capability_t, firmware_sha1_dirty) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_CHAR, 32, 105, offsetof(mavlink_magothy_capability_t, firmware_version) }, \
         { "device_type", NULL, MAVLINK_TYPE_CHAR, 16, 137, offsetof(mavlink_magothy_capability_t, device_type) }, \
         { "log_date_time", NULL, MAVLINK_TYPE_CHAR, 20, 153, offsetof(mavlink_magothy_capability_t, log_date_time) }, \
         { "vehicle_name", NULL, MAVLINK_TYPE_CHAR, 16, 173, offsetof(mavlink_magothy_capability_t, vehicle_name) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_CAPABILITY { \
    "MAGOTHY_CAPABILITY", \
    15, \
    {  { "enablement_bitmask", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_magothy_capability_t, enablement_bitmask) }, \
         { "spy_ip_address", NULL, MAVLINK_TYPE_UINT8_T, 4, 8, offsetof(mavlink_magothy_capability_t, spy_ip_address) }, \
         { "spy_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_magothy_capability_t, spy_port) }, \
         { "log_management_ip_address", NULL, MAVLINK_TYPE_UINT8_T, 4, 12, offsetof(mavlink_magothy_capability_t, log_management_ip_address) }, \
         { "log_management_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_magothy_capability_t, log_management_port) }, \
         { "log_management_endpoint", NULL, MAVLINK_TYPE_CHAR, 32, 16, offsetof(mavlink_magothy_capability_t, log_management_endpoint) }, \
         { "firmware_upload_ip_address", NULL, MAVLINK_TYPE_UINT8_T, 4, 48, offsetof(mavlink_magothy_capability_t, firmware_upload_ip_address) }, \
         { "firmware_upload_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_magothy_capability_t, firmware_upload_port) }, \
         { "firmware_upload_endpoint", NULL, MAVLINK_TYPE_CHAR, 32, 52, offsetof(mavlink_magothy_capability_t, firmware_upload_endpoint) }, \
         { "firmware_sha1", NULL, MAVLINK_TYPE_UINT8_T, 20, 84, offsetof(mavlink_magothy_capability_t, firmware_sha1) }, \
         { "firmware_sha1_dirty", NULL, MAVLINK_TYPE_UINT8_T, 0, 104, offsetof(mavlink_magothy_capability_t, firmware_sha1_dirty) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_CHAR, 32, 105, offsetof(mavlink_magothy_capability_t, firmware_version) }, \
         { "device_type", NULL, MAVLINK_TYPE_CHAR, 16, 137, offsetof(mavlink_magothy_capability_t, device_type) }, \
         { "log_date_time", NULL, MAVLINK_TYPE_CHAR, 20, 153, offsetof(mavlink_magothy_capability_t, log_date_time) }, \
         { "vehicle_name", NULL, MAVLINK_TYPE_CHAR, 16, 173, offsetof(mavlink_magothy_capability_t, vehicle_name) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_capability message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param enablement_bitmask  Additional Capability Bitmask
 * @param spy_ip_address  IP Address to Spy server
 * @param spy_port  Port to Spy server
 * @param log_management_ip_address  IP Address to Log WebDAV server
 * @param log_management_port  Port to Log WebDAV server
 * @param log_management_endpoint  Base URL prefix to Log WebDAV server
 * @param firmware_upload_ip_address  IP Address to Firmware Update web server
 * @param firmware_upload_port  Port to Firmware Update web server
 * @param firmware_upload_endpoint  Base URL prefix to Firmware Update web server
 * @param firmware_sha1  sha1 of meta-magothy
 * @param firmware_sha1_dirty  sha1 is clean if 0, else dirty
 * @param firmware_version  Pretty version string
 * @param device_type  Identifier to distinguish between vehicle types
 * @param log_date_time  Log file date time prefix
 * @param vehicle_name  Vehicle Name - Log file suffix
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_capability_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t enablement_bitmask, const uint8_t *spy_ip_address, uint16_t spy_port, const uint8_t *log_management_ip_address, uint16_t log_management_port, const char *log_management_endpoint, const uint8_t *firmware_upload_ip_address, uint16_t firmware_upload_port, const char *firmware_upload_endpoint, const uint8_t *firmware_sha1, uint8_t firmware_sha1_dirty, const char *firmware_version, const char *device_type, const char *log_date_time, const char *vehicle_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN];
    _mav_put_uint16_t(buf, 0, enablement_bitmask);
    _mav_put_uint16_t(buf, 2, spy_port);
    _mav_put_uint16_t(buf, 4, log_management_port);
    _mav_put_uint16_t(buf, 6, firmware_upload_port);
    _mav_put_uint8_t(buf, 104, firmware_sha1_dirty);
    _mav_put_uint8_t_array(buf, 8, spy_ip_address, 4);
    _mav_put_uint8_t_array(buf, 12, log_management_ip_address, 4);
    _mav_put_char_array(buf, 16, log_management_endpoint, 32);
    _mav_put_uint8_t_array(buf, 48, firmware_upload_ip_address, 4);
    _mav_put_char_array(buf, 52, firmware_upload_endpoint, 32);
    _mav_put_uint8_t_array(buf, 84, firmware_sha1, 20);
    _mav_put_char_array(buf, 105, firmware_version, 32);
    _mav_put_char_array(buf, 137, device_type, 16);
    _mav_put_char_array(buf, 153, log_date_time, 20);
    _mav_put_char_array(buf, 173, vehicle_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN);
#else
    mavlink_magothy_capability_t packet;
    packet.enablement_bitmask = enablement_bitmask;
    packet.spy_port = spy_port;
    packet.log_management_port = log_management_port;
    packet.firmware_upload_port = firmware_upload_port;
    packet.firmware_sha1_dirty = firmware_sha1_dirty;
    mav_array_memcpy(packet.spy_ip_address, spy_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.log_management_ip_address, log_management_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.log_management_endpoint, log_management_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_upload_ip_address, firmware_upload_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.firmware_upload_endpoint, firmware_upload_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_sha1, firmware_sha1, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.firmware_version, firmware_version, sizeof(char)*32);
    mav_array_memcpy(packet.device_type, device_type, sizeof(char)*16);
    mav_array_memcpy(packet.log_date_time, log_date_time, sizeof(char)*20);
    mav_array_memcpy(packet.vehicle_name, vehicle_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_CAPABILITY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_CRC);
}

/**
 * @brief Pack a magothy_capability message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param enablement_bitmask  Additional Capability Bitmask
 * @param spy_ip_address  IP Address to Spy server
 * @param spy_port  Port to Spy server
 * @param log_management_ip_address  IP Address to Log WebDAV server
 * @param log_management_port  Port to Log WebDAV server
 * @param log_management_endpoint  Base URL prefix to Log WebDAV server
 * @param firmware_upload_ip_address  IP Address to Firmware Update web server
 * @param firmware_upload_port  Port to Firmware Update web server
 * @param firmware_upload_endpoint  Base URL prefix to Firmware Update web server
 * @param firmware_sha1  sha1 of meta-magothy
 * @param firmware_sha1_dirty  sha1 is clean if 0, else dirty
 * @param firmware_version  Pretty version string
 * @param device_type  Identifier to distinguish between vehicle types
 * @param log_date_time  Log file date time prefix
 * @param vehicle_name  Vehicle Name - Log file suffix
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_capability_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t enablement_bitmask,const uint8_t *spy_ip_address,uint16_t spy_port,const uint8_t *log_management_ip_address,uint16_t log_management_port,const char *log_management_endpoint,const uint8_t *firmware_upload_ip_address,uint16_t firmware_upload_port,const char *firmware_upload_endpoint,const uint8_t *firmware_sha1,uint8_t firmware_sha1_dirty,const char *firmware_version,const char *device_type,const char *log_date_time,const char *vehicle_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN];
    _mav_put_uint16_t(buf, 0, enablement_bitmask);
    _mav_put_uint16_t(buf, 2, spy_port);
    _mav_put_uint16_t(buf, 4, log_management_port);
    _mav_put_uint16_t(buf, 6, firmware_upload_port);
    _mav_put_uint8_t(buf, 104, firmware_sha1_dirty);
    _mav_put_uint8_t_array(buf, 8, spy_ip_address, 4);
    _mav_put_uint8_t_array(buf, 12, log_management_ip_address, 4);
    _mav_put_char_array(buf, 16, log_management_endpoint, 32);
    _mav_put_uint8_t_array(buf, 48, firmware_upload_ip_address, 4);
    _mav_put_char_array(buf, 52, firmware_upload_endpoint, 32);
    _mav_put_uint8_t_array(buf, 84, firmware_sha1, 20);
    _mav_put_char_array(buf, 105, firmware_version, 32);
    _mav_put_char_array(buf, 137, device_type, 16);
    _mav_put_char_array(buf, 153, log_date_time, 20);
    _mav_put_char_array(buf, 173, vehicle_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN);
#else
    mavlink_magothy_capability_t packet;
    packet.enablement_bitmask = enablement_bitmask;
    packet.spy_port = spy_port;
    packet.log_management_port = log_management_port;
    packet.firmware_upload_port = firmware_upload_port;
    packet.firmware_sha1_dirty = firmware_sha1_dirty;
    mav_array_memcpy(packet.spy_ip_address, spy_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.log_management_ip_address, log_management_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.log_management_endpoint, log_management_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_upload_ip_address, firmware_upload_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.firmware_upload_endpoint, firmware_upload_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_sha1, firmware_sha1, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.firmware_version, firmware_version, sizeof(char)*32);
    mav_array_memcpy(packet.device_type, device_type, sizeof(char)*16);
    mav_array_memcpy(packet.log_date_time, log_date_time, sizeof(char)*20);
    mav_array_memcpy(packet.vehicle_name, vehicle_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_CAPABILITY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_CRC);
}

/**
 * @brief Encode a magothy_capability struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_capability C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_capability_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_capability_t* magothy_capability)
{
    return mavlink_msg_magothy_capability_pack(system_id, component_id, msg, magothy_capability->enablement_bitmask, magothy_capability->spy_ip_address, magothy_capability->spy_port, magothy_capability->log_management_ip_address, magothy_capability->log_management_port, magothy_capability->log_management_endpoint, magothy_capability->firmware_upload_ip_address, magothy_capability->firmware_upload_port, magothy_capability->firmware_upload_endpoint, magothy_capability->firmware_sha1, magothy_capability->firmware_sha1_dirty, magothy_capability->firmware_version, magothy_capability->device_type, magothy_capability->log_date_time, magothy_capability->vehicle_name);
}

/**
 * @brief Encode a magothy_capability struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_capability C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_capability_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_capability_t* magothy_capability)
{
    return mavlink_msg_magothy_capability_pack_chan(system_id, component_id, chan, msg, magothy_capability->enablement_bitmask, magothy_capability->spy_ip_address, magothy_capability->spy_port, magothy_capability->log_management_ip_address, magothy_capability->log_management_port, magothy_capability->log_management_endpoint, magothy_capability->firmware_upload_ip_address, magothy_capability->firmware_upload_port, magothy_capability->firmware_upload_endpoint, magothy_capability->firmware_sha1, magothy_capability->firmware_sha1_dirty, magothy_capability->firmware_version, magothy_capability->device_type, magothy_capability->log_date_time, magothy_capability->vehicle_name);
}

/**
 * @brief Send a magothy_capability message
 * @param chan MAVLink channel to send the message
 *
 * @param enablement_bitmask  Additional Capability Bitmask
 * @param spy_ip_address  IP Address to Spy server
 * @param spy_port  Port to Spy server
 * @param log_management_ip_address  IP Address to Log WebDAV server
 * @param log_management_port  Port to Log WebDAV server
 * @param log_management_endpoint  Base URL prefix to Log WebDAV server
 * @param firmware_upload_ip_address  IP Address to Firmware Update web server
 * @param firmware_upload_port  Port to Firmware Update web server
 * @param firmware_upload_endpoint  Base URL prefix to Firmware Update web server
 * @param firmware_sha1  sha1 of meta-magothy
 * @param firmware_sha1_dirty  sha1 is clean if 0, else dirty
 * @param firmware_version  Pretty version string
 * @param device_type  Identifier to distinguish between vehicle types
 * @param log_date_time  Log file date time prefix
 * @param vehicle_name  Vehicle Name - Log file suffix
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_capability_send(mavlink_channel_t chan, uint16_t enablement_bitmask, const uint8_t *spy_ip_address, uint16_t spy_port, const uint8_t *log_management_ip_address, uint16_t log_management_port, const char *log_management_endpoint, const uint8_t *firmware_upload_ip_address, uint16_t firmware_upload_port, const char *firmware_upload_endpoint, const uint8_t *firmware_sha1, uint8_t firmware_sha1_dirty, const char *firmware_version, const char *device_type, const char *log_date_time, const char *vehicle_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN];
    _mav_put_uint16_t(buf, 0, enablement_bitmask);
    _mav_put_uint16_t(buf, 2, spy_port);
    _mav_put_uint16_t(buf, 4, log_management_port);
    _mav_put_uint16_t(buf, 6, firmware_upload_port);
    _mav_put_uint8_t(buf, 104, firmware_sha1_dirty);
    _mav_put_uint8_t_array(buf, 8, spy_ip_address, 4);
    _mav_put_uint8_t_array(buf, 12, log_management_ip_address, 4);
    _mav_put_char_array(buf, 16, log_management_endpoint, 32);
    _mav_put_uint8_t_array(buf, 48, firmware_upload_ip_address, 4);
    _mav_put_char_array(buf, 52, firmware_upload_endpoint, 32);
    _mav_put_uint8_t_array(buf, 84, firmware_sha1, 20);
    _mav_put_char_array(buf, 105, firmware_version, 32);
    _mav_put_char_array(buf, 137, device_type, 16);
    _mav_put_char_array(buf, 153, log_date_time, 20);
    _mav_put_char_array(buf, 173, vehicle_name, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY, buf, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_CRC);
#else
    mavlink_magothy_capability_t packet;
    packet.enablement_bitmask = enablement_bitmask;
    packet.spy_port = spy_port;
    packet.log_management_port = log_management_port;
    packet.firmware_upload_port = firmware_upload_port;
    packet.firmware_sha1_dirty = firmware_sha1_dirty;
    mav_array_memcpy(packet.spy_ip_address, spy_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.log_management_ip_address, log_management_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.log_management_endpoint, log_management_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_upload_ip_address, firmware_upload_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.firmware_upload_endpoint, firmware_upload_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_sha1, firmware_sha1, sizeof(uint8_t)*20);
    mav_array_memcpy(packet.firmware_version, firmware_version, sizeof(char)*32);
    mav_array_memcpy(packet.device_type, device_type, sizeof(char)*16);
    mav_array_memcpy(packet.log_date_time, log_date_time, sizeof(char)*20);
    mav_array_memcpy(packet.vehicle_name, vehicle_name, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_CRC);
#endif
}

/**
 * @brief Send a magothy_capability message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_capability_send_struct(mavlink_channel_t chan, const mavlink_magothy_capability_t* magothy_capability)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_capability_send(chan, magothy_capability->enablement_bitmask, magothy_capability->spy_ip_address, magothy_capability->spy_port, magothy_capability->log_management_ip_address, magothy_capability->log_management_port, magothy_capability->log_management_endpoint, magothy_capability->firmware_upload_ip_address, magothy_capability->firmware_upload_port, magothy_capability->firmware_upload_endpoint, magothy_capability->firmware_sha1, magothy_capability->firmware_sha1_dirty, magothy_capability->firmware_version, magothy_capability->device_type, magothy_capability->log_date_time, magothy_capability->vehicle_name);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY, (const char *)magothy_capability, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_capability_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t enablement_bitmask, const uint8_t *spy_ip_address, uint16_t spy_port, const uint8_t *log_management_ip_address, uint16_t log_management_port, const char *log_management_endpoint, const uint8_t *firmware_upload_ip_address, uint16_t firmware_upload_port, const char *firmware_upload_endpoint, const uint8_t *firmware_sha1, uint8_t firmware_sha1_dirty, const char *firmware_version, const char *device_type, const char *log_date_time, const char *vehicle_name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, enablement_bitmask);
    _mav_put_uint16_t(buf, 2, spy_port);
    _mav_put_uint16_t(buf, 4, log_management_port);
    _mav_put_uint16_t(buf, 6, firmware_upload_port);
    _mav_put_uint8_t(buf, 104, firmware_sha1_dirty);
    _mav_put_uint8_t_array(buf, 8, spy_ip_address, 4);
    _mav_put_uint8_t_array(buf, 12, log_management_ip_address, 4);
    _mav_put_char_array(buf, 16, log_management_endpoint, 32);
    _mav_put_uint8_t_array(buf, 48, firmware_upload_ip_address, 4);
    _mav_put_char_array(buf, 52, firmware_upload_endpoint, 32);
    _mav_put_uint8_t_array(buf, 84, firmware_sha1, 20);
    _mav_put_char_array(buf, 105, firmware_version, 32);
    _mav_put_char_array(buf, 137, device_type, 16);
    _mav_put_char_array(buf, 153, log_date_time, 20);
    _mav_put_char_array(buf, 173, vehicle_name, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY, buf, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_CRC);
#else
    mavlink_magothy_capability_t *packet = (mavlink_magothy_capability_t *)msgbuf;
    packet->enablement_bitmask = enablement_bitmask;
    packet->spy_port = spy_port;
    packet->log_management_port = log_management_port;
    packet->firmware_upload_port = firmware_upload_port;
    packet->firmware_sha1_dirty = firmware_sha1_dirty;
    mav_array_memcpy(packet->spy_ip_address, spy_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet->log_management_ip_address, log_management_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet->log_management_endpoint, log_management_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet->firmware_upload_ip_address, firmware_upload_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet->firmware_upload_endpoint, firmware_upload_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet->firmware_sha1, firmware_sha1, sizeof(uint8_t)*20);
    mav_array_memcpy(packet->firmware_version, firmware_version, sizeof(char)*32);
    mav_array_memcpy(packet->device_type, device_type, sizeof(char)*16);
    mav_array_memcpy(packet->log_date_time, log_date_time, sizeof(char)*20);
    mav_array_memcpy(packet->vehicle_name, vehicle_name, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_CAPABILITY UNPACKING


/**
 * @brief Get field enablement_bitmask from magothy_capability message
 *
 * @return  Additional Capability Bitmask
 */
static inline uint16_t mavlink_msg_magothy_capability_get_enablement_bitmask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field spy_ip_address from magothy_capability message
 *
 * @return  IP Address to Spy server
 */
static inline uint16_t mavlink_msg_magothy_capability_get_spy_ip_address(const mavlink_message_t* msg, uint8_t *spy_ip_address)
{
    return _MAV_RETURN_uint8_t_array(msg, spy_ip_address, 4,  8);
}

/**
 * @brief Get field spy_port from magothy_capability message
 *
 * @return  Port to Spy server
 */
static inline uint16_t mavlink_msg_magothy_capability_get_spy_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field log_management_ip_address from magothy_capability message
 *
 * @return  IP Address to Log WebDAV server
 */
static inline uint16_t mavlink_msg_magothy_capability_get_log_management_ip_address(const mavlink_message_t* msg, uint8_t *log_management_ip_address)
{
    return _MAV_RETURN_uint8_t_array(msg, log_management_ip_address, 4,  12);
}

/**
 * @brief Get field log_management_port from magothy_capability message
 *
 * @return  Port to Log WebDAV server
 */
static inline uint16_t mavlink_msg_magothy_capability_get_log_management_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field log_management_endpoint from magothy_capability message
 *
 * @return  Base URL prefix to Log WebDAV server
 */
static inline uint16_t mavlink_msg_magothy_capability_get_log_management_endpoint(const mavlink_message_t* msg, char *log_management_endpoint)
{
    return _MAV_RETURN_char_array(msg, log_management_endpoint, 32,  16);
}

/**
 * @brief Get field firmware_upload_ip_address from magothy_capability message
 *
 * @return  IP Address to Firmware Update web server
 */
static inline uint16_t mavlink_msg_magothy_capability_get_firmware_upload_ip_address(const mavlink_message_t* msg, uint8_t *firmware_upload_ip_address)
{
    return _MAV_RETURN_uint8_t_array(msg, firmware_upload_ip_address, 4,  48);
}

/**
 * @brief Get field firmware_upload_port from magothy_capability message
 *
 * @return  Port to Firmware Update web server
 */
static inline uint16_t mavlink_msg_magothy_capability_get_firmware_upload_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field firmware_upload_endpoint from magothy_capability message
 *
 * @return  Base URL prefix to Firmware Update web server
 */
static inline uint16_t mavlink_msg_magothy_capability_get_firmware_upload_endpoint(const mavlink_message_t* msg, char *firmware_upload_endpoint)
{
    return _MAV_RETURN_char_array(msg, firmware_upload_endpoint, 32,  52);
}

/**
 * @brief Get field firmware_sha1 from magothy_capability message
 *
 * @return  sha1 of meta-magothy
 */
static inline uint16_t mavlink_msg_magothy_capability_get_firmware_sha1(const mavlink_message_t* msg, uint8_t *firmware_sha1)
{
    return _MAV_RETURN_uint8_t_array(msg, firmware_sha1, 20,  84);
}

/**
 * @brief Get field firmware_sha1_dirty from magothy_capability message
 *
 * @return  sha1 is clean if 0, else dirty
 */
static inline uint8_t mavlink_msg_magothy_capability_get_firmware_sha1_dirty(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  104);
}

/**
 * @brief Get field firmware_version from magothy_capability message
 *
 * @return  Pretty version string
 */
static inline uint16_t mavlink_msg_magothy_capability_get_firmware_version(const mavlink_message_t* msg, char *firmware_version)
{
    return _MAV_RETURN_char_array(msg, firmware_version, 32,  105);
}

/**
 * @brief Get field device_type from magothy_capability message
 *
 * @return  Identifier to distinguish between vehicle types
 */
static inline uint16_t mavlink_msg_magothy_capability_get_device_type(const mavlink_message_t* msg, char *device_type)
{
    return _MAV_RETURN_char_array(msg, device_type, 16,  137);
}

/**
 * @brief Get field log_date_time from magothy_capability message
 *
 * @return  Log file date time prefix
 */
static inline uint16_t mavlink_msg_magothy_capability_get_log_date_time(const mavlink_message_t* msg, char *log_date_time)
{
    return _MAV_RETURN_char_array(msg, log_date_time, 20,  153);
}

/**
 * @brief Get field vehicle_name from magothy_capability message
 *
 * @return  Vehicle Name - Log file suffix
 */
static inline uint16_t mavlink_msg_magothy_capability_get_vehicle_name(const mavlink_message_t* msg, char *vehicle_name)
{
    return _MAV_RETURN_char_array(msg, vehicle_name, 16,  173);
}

/**
 * @brief Decode a magothy_capability message into a struct
 *
 * @param msg The message to decode
 * @param magothy_capability C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_capability_decode(const mavlink_message_t* msg, mavlink_magothy_capability_t* magothy_capability)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_capability->enablement_bitmask = mavlink_msg_magothy_capability_get_enablement_bitmask(msg);
    magothy_capability->spy_port = mavlink_msg_magothy_capability_get_spy_port(msg);
    magothy_capability->log_management_port = mavlink_msg_magothy_capability_get_log_management_port(msg);
    magothy_capability->firmware_upload_port = mavlink_msg_magothy_capability_get_firmware_upload_port(msg);
    mavlink_msg_magothy_capability_get_spy_ip_address(msg, magothy_capability->spy_ip_address);
    mavlink_msg_magothy_capability_get_log_management_ip_address(msg, magothy_capability->log_management_ip_address);
    mavlink_msg_magothy_capability_get_log_management_endpoint(msg, magothy_capability->log_management_endpoint);
    mavlink_msg_magothy_capability_get_firmware_upload_ip_address(msg, magothy_capability->firmware_upload_ip_address);
    mavlink_msg_magothy_capability_get_firmware_upload_endpoint(msg, magothy_capability->firmware_upload_endpoint);
    mavlink_msg_magothy_capability_get_firmware_sha1(msg, magothy_capability->firmware_sha1);
    magothy_capability->firmware_sha1_dirty = mavlink_msg_magothy_capability_get_firmware_sha1_dirty(msg);
    mavlink_msg_magothy_capability_get_firmware_version(msg, magothy_capability->firmware_version);
    mavlink_msg_magothy_capability_get_device_type(msg, magothy_capability->device_type);
    mavlink_msg_magothy_capability_get_log_date_time(msg, magothy_capability->log_date_time);
    mavlink_msg_magothy_capability_get_vehicle_name(msg, magothy_capability->vehicle_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN;
        memset(magothy_capability, 0, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN);
    memcpy(magothy_capability, _MAV_PAYLOAD(msg), len);
#endif
}
