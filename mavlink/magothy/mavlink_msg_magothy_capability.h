#pragma once
// MESSAGE MAGOTHY_CAPABILITY PACKING

#define MAVLINK_MSG_ID_MAGOTHY_CAPABILITY 50100


typedef struct __mavlink_magothy_capability_t {
 uint16_t enablement_bitmask; /*<  Additional Capability Bitmask*/
 uint16_t log_management_port; /*<  */
 uint16_t firmware_upload_port; /*<  */
 uint8_t log_management_ip_address[4]; /*<  */
 char log_management_endpoint[32]; /*<  */
 uint8_t firmware_upload_ip_address[4]; /*<  */
 char firmware_upload_endpoint[32]; /*<  */
} mavlink_magothy_capability_t;

#define MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN 78
#define MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN 78
#define MAVLINK_MSG_ID_50100_LEN 78
#define MAVLINK_MSG_ID_50100_MIN_LEN 78

#define MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_CRC 104
#define MAVLINK_MSG_ID_50100_CRC 104

#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_LOG_MANAGEMENT_IP_ADDRESS_LEN 4
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_LOG_MANAGEMENT_ENDPOINT_LEN 32
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_FIRMWARE_UPLOAD_IP_ADDRESS_LEN 4
#define MAVLINK_MSG_MAGOTHY_CAPABILITY_FIELD_FIRMWARE_UPLOAD_ENDPOINT_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_CAPABILITY { \
    50100, \
    "MAGOTHY_CAPABILITY", \
    7, \
    {  { "enablement_bitmask", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_magothy_capability_t, enablement_bitmask) }, \
         { "log_management_ip_address", NULL, MAVLINK_TYPE_UINT8_T, 4, 6, offsetof(mavlink_magothy_capability_t, log_management_ip_address) }, \
         { "log_management_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_magothy_capability_t, log_management_port) }, \
         { "log_management_endpoint", NULL, MAVLINK_TYPE_CHAR, 32, 10, offsetof(mavlink_magothy_capability_t, log_management_endpoint) }, \
         { "firmware_upload_ip_address", NULL, MAVLINK_TYPE_UINT8_T, 4, 42, offsetof(mavlink_magothy_capability_t, firmware_upload_ip_address) }, \
         { "firmware_upload_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_magothy_capability_t, firmware_upload_port) }, \
         { "firmware_upload_endpoint", NULL, MAVLINK_TYPE_CHAR, 32, 46, offsetof(mavlink_magothy_capability_t, firmware_upload_endpoint) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_CAPABILITY { \
    "MAGOTHY_CAPABILITY", \
    7, \
    {  { "enablement_bitmask", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_magothy_capability_t, enablement_bitmask) }, \
         { "log_management_ip_address", NULL, MAVLINK_TYPE_UINT8_T, 4, 6, offsetof(mavlink_magothy_capability_t, log_management_ip_address) }, \
         { "log_management_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_magothy_capability_t, log_management_port) }, \
         { "log_management_endpoint", NULL, MAVLINK_TYPE_CHAR, 32, 10, offsetof(mavlink_magothy_capability_t, log_management_endpoint) }, \
         { "firmware_upload_ip_address", NULL, MAVLINK_TYPE_UINT8_T, 4, 42, offsetof(mavlink_magothy_capability_t, firmware_upload_ip_address) }, \
         { "firmware_upload_port", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_magothy_capability_t, firmware_upload_port) }, \
         { "firmware_upload_endpoint", NULL, MAVLINK_TYPE_CHAR, 32, 46, offsetof(mavlink_magothy_capability_t, firmware_upload_endpoint) }, \
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
 * @param log_management_ip_address  
 * @param log_management_port  
 * @param log_management_endpoint  
 * @param firmware_upload_ip_address  
 * @param firmware_upload_port  
 * @param firmware_upload_endpoint  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_capability_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t enablement_bitmask, const uint8_t *log_management_ip_address, uint16_t log_management_port, const char *log_management_endpoint, const uint8_t *firmware_upload_ip_address, uint16_t firmware_upload_port, const char *firmware_upload_endpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN];
    _mav_put_uint16_t(buf, 0, enablement_bitmask);
    _mav_put_uint16_t(buf, 2, log_management_port);
    _mav_put_uint16_t(buf, 4, firmware_upload_port);
    _mav_put_uint8_t_array(buf, 6, log_management_ip_address, 4);
    _mav_put_char_array(buf, 10, log_management_endpoint, 32);
    _mav_put_uint8_t_array(buf, 42, firmware_upload_ip_address, 4);
    _mav_put_char_array(buf, 46, firmware_upload_endpoint, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN);
#else
    mavlink_magothy_capability_t packet;
    packet.enablement_bitmask = enablement_bitmask;
    packet.log_management_port = log_management_port;
    packet.firmware_upload_port = firmware_upload_port;
    mav_array_memcpy(packet.log_management_ip_address, log_management_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.log_management_endpoint, log_management_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_upload_ip_address, firmware_upload_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.firmware_upload_endpoint, firmware_upload_endpoint, sizeof(char)*32);
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
 * @param log_management_ip_address  
 * @param log_management_port  
 * @param log_management_endpoint  
 * @param firmware_upload_ip_address  
 * @param firmware_upload_port  
 * @param firmware_upload_endpoint  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_capability_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t enablement_bitmask,const uint8_t *log_management_ip_address,uint16_t log_management_port,const char *log_management_endpoint,const uint8_t *firmware_upload_ip_address,uint16_t firmware_upload_port,const char *firmware_upload_endpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN];
    _mav_put_uint16_t(buf, 0, enablement_bitmask);
    _mav_put_uint16_t(buf, 2, log_management_port);
    _mav_put_uint16_t(buf, 4, firmware_upload_port);
    _mav_put_uint8_t_array(buf, 6, log_management_ip_address, 4);
    _mav_put_char_array(buf, 10, log_management_endpoint, 32);
    _mav_put_uint8_t_array(buf, 42, firmware_upload_ip_address, 4);
    _mav_put_char_array(buf, 46, firmware_upload_endpoint, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN);
#else
    mavlink_magothy_capability_t packet;
    packet.enablement_bitmask = enablement_bitmask;
    packet.log_management_port = log_management_port;
    packet.firmware_upload_port = firmware_upload_port;
    mav_array_memcpy(packet.log_management_ip_address, log_management_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.log_management_endpoint, log_management_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_upload_ip_address, firmware_upload_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.firmware_upload_endpoint, firmware_upload_endpoint, sizeof(char)*32);
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
    return mavlink_msg_magothy_capability_pack(system_id, component_id, msg, magothy_capability->enablement_bitmask, magothy_capability->log_management_ip_address, magothy_capability->log_management_port, magothy_capability->log_management_endpoint, magothy_capability->firmware_upload_ip_address, magothy_capability->firmware_upload_port, magothy_capability->firmware_upload_endpoint);
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
    return mavlink_msg_magothy_capability_pack_chan(system_id, component_id, chan, msg, magothy_capability->enablement_bitmask, magothy_capability->log_management_ip_address, magothy_capability->log_management_port, magothy_capability->log_management_endpoint, magothy_capability->firmware_upload_ip_address, magothy_capability->firmware_upload_port, magothy_capability->firmware_upload_endpoint);
}

/**
 * @brief Send a magothy_capability message
 * @param chan MAVLink channel to send the message
 *
 * @param enablement_bitmask  Additional Capability Bitmask
 * @param log_management_ip_address  
 * @param log_management_port  
 * @param log_management_endpoint  
 * @param firmware_upload_ip_address  
 * @param firmware_upload_port  
 * @param firmware_upload_endpoint  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_capability_send(mavlink_channel_t chan, uint16_t enablement_bitmask, const uint8_t *log_management_ip_address, uint16_t log_management_port, const char *log_management_endpoint, const uint8_t *firmware_upload_ip_address, uint16_t firmware_upload_port, const char *firmware_upload_endpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN];
    _mav_put_uint16_t(buf, 0, enablement_bitmask);
    _mav_put_uint16_t(buf, 2, log_management_port);
    _mav_put_uint16_t(buf, 4, firmware_upload_port);
    _mav_put_uint8_t_array(buf, 6, log_management_ip_address, 4);
    _mav_put_char_array(buf, 10, log_management_endpoint, 32);
    _mav_put_uint8_t_array(buf, 42, firmware_upload_ip_address, 4);
    _mav_put_char_array(buf, 46, firmware_upload_endpoint, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY, buf, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_CRC);
#else
    mavlink_magothy_capability_t packet;
    packet.enablement_bitmask = enablement_bitmask;
    packet.log_management_port = log_management_port;
    packet.firmware_upload_port = firmware_upload_port;
    mav_array_memcpy(packet.log_management_ip_address, log_management_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.log_management_endpoint, log_management_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet.firmware_upload_ip_address, firmware_upload_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.firmware_upload_endpoint, firmware_upload_endpoint, sizeof(char)*32);
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
    mavlink_msg_magothy_capability_send(chan, magothy_capability->enablement_bitmask, magothy_capability->log_management_ip_address, magothy_capability->log_management_port, magothy_capability->log_management_endpoint, magothy_capability->firmware_upload_ip_address, magothy_capability->firmware_upload_port, magothy_capability->firmware_upload_endpoint);
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
static inline void mavlink_msg_magothy_capability_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t enablement_bitmask, const uint8_t *log_management_ip_address, uint16_t log_management_port, const char *log_management_endpoint, const uint8_t *firmware_upload_ip_address, uint16_t firmware_upload_port, const char *firmware_upload_endpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, enablement_bitmask);
    _mav_put_uint16_t(buf, 2, log_management_port);
    _mav_put_uint16_t(buf, 4, firmware_upload_port);
    _mav_put_uint8_t_array(buf, 6, log_management_ip_address, 4);
    _mav_put_char_array(buf, 10, log_management_endpoint, 32);
    _mav_put_uint8_t_array(buf, 42, firmware_upload_ip_address, 4);
    _mav_put_char_array(buf, 46, firmware_upload_endpoint, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY, buf, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_CRC);
#else
    mavlink_magothy_capability_t *packet = (mavlink_magothy_capability_t *)msgbuf;
    packet->enablement_bitmask = enablement_bitmask;
    packet->log_management_port = log_management_port;
    packet->firmware_upload_port = firmware_upload_port;
    mav_array_memcpy(packet->log_management_ip_address, log_management_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet->log_management_endpoint, log_management_endpoint, sizeof(char)*32);
    mav_array_memcpy(packet->firmware_upload_ip_address, firmware_upload_ip_address, sizeof(uint8_t)*4);
    mav_array_memcpy(packet->firmware_upload_endpoint, firmware_upload_endpoint, sizeof(char)*32);
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
 * @brief Get field log_management_ip_address from magothy_capability message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_magothy_capability_get_log_management_ip_address(const mavlink_message_t* msg, uint8_t *log_management_ip_address)
{
    return _MAV_RETURN_uint8_t_array(msg, log_management_ip_address, 4,  6);
}

/**
 * @brief Get field log_management_port from magothy_capability message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_magothy_capability_get_log_management_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field log_management_endpoint from magothy_capability message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_magothy_capability_get_log_management_endpoint(const mavlink_message_t* msg, char *log_management_endpoint)
{
    return _MAV_RETURN_char_array(msg, log_management_endpoint, 32,  10);
}

/**
 * @brief Get field firmware_upload_ip_address from magothy_capability message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_magothy_capability_get_firmware_upload_ip_address(const mavlink_message_t* msg, uint8_t *firmware_upload_ip_address)
{
    return _MAV_RETURN_uint8_t_array(msg, firmware_upload_ip_address, 4,  42);
}

/**
 * @brief Get field firmware_upload_port from magothy_capability message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_magothy_capability_get_firmware_upload_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field firmware_upload_endpoint from magothy_capability message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_magothy_capability_get_firmware_upload_endpoint(const mavlink_message_t* msg, char *firmware_upload_endpoint)
{
    return _MAV_RETURN_char_array(msg, firmware_upload_endpoint, 32,  46);
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
    magothy_capability->log_management_port = mavlink_msg_magothy_capability_get_log_management_port(msg);
    magothy_capability->firmware_upload_port = mavlink_msg_magothy_capability_get_firmware_upload_port(msg);
    mavlink_msg_magothy_capability_get_log_management_ip_address(msg, magothy_capability->log_management_ip_address);
    mavlink_msg_magothy_capability_get_log_management_endpoint(msg, magothy_capability->log_management_endpoint);
    mavlink_msg_magothy_capability_get_firmware_upload_ip_address(msg, magothy_capability->firmware_upload_ip_address);
    mavlink_msg_magothy_capability_get_firmware_upload_endpoint(msg, magothy_capability->firmware_upload_endpoint);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN;
        memset(magothy_capability, 0, MAVLINK_MSG_ID_MAGOTHY_CAPABILITY_LEN);
    memcpy(magothy_capability, _MAV_PAYLOAD(msg), len);
#endif
}
