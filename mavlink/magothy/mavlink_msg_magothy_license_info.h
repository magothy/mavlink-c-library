#pragma once
// MESSAGE MAGOTHY_LICENSE_INFO PACKING

#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO 50150


typedef struct __mavlink_magothy_license_info_t {
 uint8_t is_set; /*<  1 if license key is set, else 0*/
 uint8_t is_valid; /*<  1 if license key is valid, else 0*/
 char product_key[64]; /*<  license product_key - must be null terminated if length is less than 64*/
} mavlink_magothy_license_info_t;

#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN 66
#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_MIN_LEN 66
#define MAVLINK_MSG_ID_50150_LEN 66
#define MAVLINK_MSG_ID_50150_MIN_LEN 66

#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_CRC 168
#define MAVLINK_MSG_ID_50150_CRC 168

#define MAVLINK_MSG_MAGOTHY_LICENSE_INFO_FIELD_PRODUCT_KEY_LEN 64

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_LICENSE_INFO { \
    50150, \
    "MAGOTHY_LICENSE_INFO", \
    3, \
    {  { "is_set", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_magothy_license_info_t, is_set) }, \
         { "is_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_magothy_license_info_t, is_valid) }, \
         { "product_key", NULL, MAVLINK_TYPE_CHAR, 64, 2, offsetof(mavlink_magothy_license_info_t, product_key) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_LICENSE_INFO { \
    "MAGOTHY_LICENSE_INFO", \
    3, \
    {  { "is_set", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_magothy_license_info_t, is_set) }, \
         { "is_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_magothy_license_info_t, is_valid) }, \
         { "product_key", NULL, MAVLINK_TYPE_CHAR, 64, 2, offsetof(mavlink_magothy_license_info_t, product_key) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_license_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param is_set  1 if license key is set, else 0
 * @param is_valid  1 if license key is valid, else 0
 * @param product_key  license product_key - must be null terminated if length is less than 64
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_license_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t is_set, uint8_t is_valid, const char *product_key)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN];
    _mav_put_uint8_t(buf, 0, is_set);
    _mav_put_uint8_t(buf, 1, is_valid);
    _mav_put_char_array(buf, 2, product_key, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN);
#else
    mavlink_magothy_license_info_t packet;
    packet.is_set = is_set;
    packet.is_valid = is_valid;
    mav_array_memcpy(packet.product_key, product_key, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_CRC);
}

/**
 * @brief Pack a magothy_license_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param is_set  1 if license key is set, else 0
 * @param is_valid  1 if license key is valid, else 0
 * @param product_key  license product_key - must be null terminated if length is less than 64
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_license_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t is_set,uint8_t is_valid,const char *product_key)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN];
    _mav_put_uint8_t(buf, 0, is_set);
    _mav_put_uint8_t(buf, 1, is_valid);
    _mav_put_char_array(buf, 2, product_key, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN);
#else
    mavlink_magothy_license_info_t packet;
    packet.is_set = is_set;
    packet.is_valid = is_valid;
    mav_array_memcpy(packet.product_key, product_key, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_CRC);
}

/**
 * @brief Encode a magothy_license_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_license_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_license_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_license_info_t* magothy_license_info)
{
    return mavlink_msg_magothy_license_info_pack(system_id, component_id, msg, magothy_license_info->is_set, magothy_license_info->is_valid, magothy_license_info->product_key);
}

/**
 * @brief Encode a magothy_license_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_license_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_license_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_license_info_t* magothy_license_info)
{
    return mavlink_msg_magothy_license_info_pack_chan(system_id, component_id, chan, msg, magothy_license_info->is_set, magothy_license_info->is_valid, magothy_license_info->product_key);
}

/**
 * @brief Send a magothy_license_info message
 * @param chan MAVLink channel to send the message
 *
 * @param is_set  1 if license key is set, else 0
 * @param is_valid  1 if license key is valid, else 0
 * @param product_key  license product_key - must be null terminated if length is less than 64
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_license_info_send(mavlink_channel_t chan, uint8_t is_set, uint8_t is_valid, const char *product_key)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN];
    _mav_put_uint8_t(buf, 0, is_set);
    _mav_put_uint8_t(buf, 1, is_valid);
    _mav_put_char_array(buf, 2, product_key, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO, buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_CRC);
#else
    mavlink_magothy_license_info_t packet;
    packet.is_set = is_set;
    packet.is_valid = is_valid;
    mav_array_memcpy(packet.product_key, product_key, sizeof(char)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_CRC);
#endif
}

/**
 * @brief Send a magothy_license_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_license_info_send_struct(mavlink_channel_t chan, const mavlink_magothy_license_info_t* magothy_license_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_license_info_send(chan, magothy_license_info->is_set, magothy_license_info->is_valid, magothy_license_info->product_key);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO, (const char *)magothy_license_info, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_license_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t is_set, uint8_t is_valid, const char *product_key)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, is_set);
    _mav_put_uint8_t(buf, 1, is_valid);
    _mav_put_char_array(buf, 2, product_key, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO, buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_CRC);
#else
    mavlink_magothy_license_info_t *packet = (mavlink_magothy_license_info_t *)msgbuf;
    packet->is_set = is_set;
    packet->is_valid = is_valid;
    mav_array_memcpy(packet->product_key, product_key, sizeof(char)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_LICENSE_INFO UNPACKING


/**
 * @brief Get field is_set from magothy_license_info message
 *
 * @return  1 if license key is set, else 0
 */
static inline uint8_t mavlink_msg_magothy_license_info_get_is_set(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field is_valid from magothy_license_info message
 *
 * @return  1 if license key is valid, else 0
 */
static inline uint8_t mavlink_msg_magothy_license_info_get_is_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field product_key from magothy_license_info message
 *
 * @return  license product_key - must be null terminated if length is less than 64
 */
static inline uint16_t mavlink_msg_magothy_license_info_get_product_key(const mavlink_message_t* msg, char *product_key)
{
    return _MAV_RETURN_char_array(msg, product_key, 64,  2);
}

/**
 * @brief Decode a magothy_license_info message into a struct
 *
 * @param msg The message to decode
 * @param magothy_license_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_license_info_decode(const mavlink_message_t* msg, mavlink_magothy_license_info_t* magothy_license_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_license_info->is_set = mavlink_msg_magothy_license_info_get_is_set(msg);
    magothy_license_info->is_valid = mavlink_msg_magothy_license_info_get_is_valid(msg);
    mavlink_msg_magothy_license_info_get_product_key(msg, magothy_license_info->product_key);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN;
        memset(magothy_license_info, 0, MAVLINK_MSG_ID_MAGOTHY_LICENSE_INFO_LEN);
    memcpy(magothy_license_info, _MAV_PAYLOAD(msg), len);
#endif
}
