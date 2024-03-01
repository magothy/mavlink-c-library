#pragma once
// MESSAGE MAGOTHY_LICENSE_TRANSFER_INITIALIZE PACKING

#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE 50151


typedef struct __mavlink_magothy_license_transfer_initialize_t {
 uint8_t target_system; /*<  system id of target system*/
 uint8_t transfer_type; /*<  file type*/
 char product_key[64]; /*<  license product_key - must be null terminated if length is less than 64*/
 char description[64]; /*<  user provided description - must be null terminated if length is less than 64*/
} mavlink_magothy_license_transfer_initialize_t;

#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN 130
#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_MIN_LEN 130
#define MAVLINK_MSG_ID_50151_LEN 130
#define MAVLINK_MSG_ID_50151_MIN_LEN 130

#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_CRC 163
#define MAVLINK_MSG_ID_50151_CRC 163

#define MAVLINK_MSG_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_FIELD_PRODUCT_KEY_LEN 64
#define MAVLINK_MSG_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_FIELD_DESCRIPTION_LEN 64

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_LICENSE_TRANSFER_INITIALIZE { \
    50151, \
    "MAGOTHY_LICENSE_TRANSFER_INITIALIZE", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_magothy_license_transfer_initialize_t, target_system) }, \
         { "transfer_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_magothy_license_transfer_initialize_t, transfer_type) }, \
         { "product_key", NULL, MAVLINK_TYPE_CHAR, 64, 2, offsetof(mavlink_magothy_license_transfer_initialize_t, product_key) }, \
         { "description", NULL, MAVLINK_TYPE_CHAR, 64, 66, offsetof(mavlink_magothy_license_transfer_initialize_t, description) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_LICENSE_TRANSFER_INITIALIZE { \
    "MAGOTHY_LICENSE_TRANSFER_INITIALIZE", \
    4, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_magothy_license_transfer_initialize_t, target_system) }, \
         { "transfer_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_magothy_license_transfer_initialize_t, transfer_type) }, \
         { "product_key", NULL, MAVLINK_TYPE_CHAR, 64, 2, offsetof(mavlink_magothy_license_transfer_initialize_t, product_key) }, \
         { "description", NULL, MAVLINK_TYPE_CHAR, 64, 66, offsetof(mavlink_magothy_license_transfer_initialize_t, description) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_license_transfer_initialize message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  system id of target system
 * @param transfer_type  file type
 * @param product_key  license product_key - must be null terminated if length is less than 64
 * @param description  user provided description - must be null terminated if length is less than 64
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_license_transfer_initialize_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t transfer_type, const char *product_key, const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, transfer_type);
    _mav_put_char_array(buf, 2, product_key, 64);
    _mav_put_char_array(buf, 66, description, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN);
#else
    mavlink_magothy_license_transfer_initialize_t packet;
    packet.target_system = target_system;
    packet.transfer_type = transfer_type;
    mav_array_memcpy(packet.product_key, product_key, sizeof(char)*64);
    mav_array_memcpy(packet.description, description, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_CRC);
}

/**
 * @brief Pack a magothy_license_transfer_initialize message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  system id of target system
 * @param transfer_type  file type
 * @param product_key  license product_key - must be null terminated if length is less than 64
 * @param description  user provided description - must be null terminated if length is less than 64
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_license_transfer_initialize_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t transfer_type,const char *product_key,const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, transfer_type);
    _mav_put_char_array(buf, 2, product_key, 64);
    _mav_put_char_array(buf, 66, description, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN);
#else
    mavlink_magothy_license_transfer_initialize_t packet;
    packet.target_system = target_system;
    packet.transfer_type = transfer_type;
    mav_array_memcpy(packet.product_key, product_key, sizeof(char)*64);
    mav_array_memcpy(packet.description, description, sizeof(char)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_CRC);
}

/**
 * @brief Encode a magothy_license_transfer_initialize struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_license_transfer_initialize C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_license_transfer_initialize_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_license_transfer_initialize_t* magothy_license_transfer_initialize)
{
    return mavlink_msg_magothy_license_transfer_initialize_pack(system_id, component_id, msg, magothy_license_transfer_initialize->target_system, magothy_license_transfer_initialize->transfer_type, magothy_license_transfer_initialize->product_key, magothy_license_transfer_initialize->description);
}

/**
 * @brief Encode a magothy_license_transfer_initialize struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_license_transfer_initialize C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_license_transfer_initialize_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_license_transfer_initialize_t* magothy_license_transfer_initialize)
{
    return mavlink_msg_magothy_license_transfer_initialize_pack_chan(system_id, component_id, chan, msg, magothy_license_transfer_initialize->target_system, magothy_license_transfer_initialize->transfer_type, magothy_license_transfer_initialize->product_key, magothy_license_transfer_initialize->description);
}

/**
 * @brief Send a magothy_license_transfer_initialize message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  system id of target system
 * @param transfer_type  file type
 * @param product_key  license product_key - must be null terminated if length is less than 64
 * @param description  user provided description - must be null terminated if length is less than 64
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_license_transfer_initialize_send(mavlink_channel_t chan, uint8_t target_system, uint8_t transfer_type, const char *product_key, const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, transfer_type);
    _mav_put_char_array(buf, 2, product_key, 64);
    _mav_put_char_array(buf, 66, description, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE, buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_CRC);
#else
    mavlink_magothy_license_transfer_initialize_t packet;
    packet.target_system = target_system;
    packet.transfer_type = transfer_type;
    mav_array_memcpy(packet.product_key, product_key, sizeof(char)*64);
    mav_array_memcpy(packet.description, description, sizeof(char)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_CRC);
#endif
}

/**
 * @brief Send a magothy_license_transfer_initialize message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_license_transfer_initialize_send_struct(mavlink_channel_t chan, const mavlink_magothy_license_transfer_initialize_t* magothy_license_transfer_initialize)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_license_transfer_initialize_send(chan, magothy_license_transfer_initialize->target_system, magothy_license_transfer_initialize->transfer_type, magothy_license_transfer_initialize->product_key, magothy_license_transfer_initialize->description);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE, (const char *)magothy_license_transfer_initialize, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_license_transfer_initialize_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t transfer_type, const char *product_key, const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, transfer_type);
    _mav_put_char_array(buf, 2, product_key, 64);
    _mav_put_char_array(buf, 66, description, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE, buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_CRC);
#else
    mavlink_magothy_license_transfer_initialize_t *packet = (mavlink_magothy_license_transfer_initialize_t *)msgbuf;
    packet->target_system = target_system;
    packet->transfer_type = transfer_type;
    mav_array_memcpy(packet->product_key, product_key, sizeof(char)*64);
    mav_array_memcpy(packet->description, description, sizeof(char)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_LICENSE_TRANSFER_INITIALIZE UNPACKING


/**
 * @brief Get field target_system from magothy_license_transfer_initialize message
 *
 * @return  system id of target system
 */
static inline uint8_t mavlink_msg_magothy_license_transfer_initialize_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field transfer_type from magothy_license_transfer_initialize message
 *
 * @return  file type
 */
static inline uint8_t mavlink_msg_magothy_license_transfer_initialize_get_transfer_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field product_key from magothy_license_transfer_initialize message
 *
 * @return  license product_key - must be null terminated if length is less than 64
 */
static inline uint16_t mavlink_msg_magothy_license_transfer_initialize_get_product_key(const mavlink_message_t* msg, char *product_key)
{
    return _MAV_RETURN_char_array(msg, product_key, 64,  2);
}

/**
 * @brief Get field description from magothy_license_transfer_initialize message
 *
 * @return  user provided description - must be null terminated if length is less than 64
 */
static inline uint16_t mavlink_msg_magothy_license_transfer_initialize_get_description(const mavlink_message_t* msg, char *description)
{
    return _MAV_RETURN_char_array(msg, description, 64,  66);
}

/**
 * @brief Decode a magothy_license_transfer_initialize message into a struct
 *
 * @param msg The message to decode
 * @param magothy_license_transfer_initialize C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_license_transfer_initialize_decode(const mavlink_message_t* msg, mavlink_magothy_license_transfer_initialize_t* magothy_license_transfer_initialize)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_license_transfer_initialize->target_system = mavlink_msg_magothy_license_transfer_initialize_get_target_system(msg);
    magothy_license_transfer_initialize->transfer_type = mavlink_msg_magothy_license_transfer_initialize_get_transfer_type(msg);
    mavlink_msg_magothy_license_transfer_initialize_get_product_key(msg, magothy_license_transfer_initialize->product_key);
    mavlink_msg_magothy_license_transfer_initialize_get_description(msg, magothy_license_transfer_initialize->description);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN;
        memset(magothy_license_transfer_initialize, 0, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_INITIALIZE_LEN);
    memcpy(magothy_license_transfer_initialize, _MAV_PAYLOAD(msg), len);
#endif
}
