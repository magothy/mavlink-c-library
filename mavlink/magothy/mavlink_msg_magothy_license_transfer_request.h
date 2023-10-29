#pragma once
// MESSAGE MAGOTHY_LICENSE_TRANSFER_REQUEST PACKING

#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST 50153


typedef struct __mavlink_magothy_license_transfer_request_t {
 uint8_t target_system; /*<  system id of target system*/
 uint8_t transfer_type; /*<  file type*/
 uint8_t chunk_index; /*<  chunk index of license transfer*/
} mavlink_magothy_license_transfer_request_t;

#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN 3
#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_MIN_LEN 3
#define MAVLINK_MSG_ID_50153_LEN 3
#define MAVLINK_MSG_ID_50153_MIN_LEN 3

#define MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_CRC 141
#define MAVLINK_MSG_ID_50153_CRC 141



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_LICENSE_TRANSFER_REQUEST { \
    50153, \
    "MAGOTHY_LICENSE_TRANSFER_REQUEST", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_magothy_license_transfer_request_t, target_system) }, \
         { "transfer_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_magothy_license_transfer_request_t, transfer_type) }, \
         { "chunk_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_magothy_license_transfer_request_t, chunk_index) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_LICENSE_TRANSFER_REQUEST { \
    "MAGOTHY_LICENSE_TRANSFER_REQUEST", \
    3, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_magothy_license_transfer_request_t, target_system) }, \
         { "transfer_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_magothy_license_transfer_request_t, transfer_type) }, \
         { "chunk_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_magothy_license_transfer_request_t, chunk_index) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_license_transfer_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  system id of target system
 * @param transfer_type  file type
 * @param chunk_index  chunk index of license transfer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_license_transfer_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t transfer_type, uint8_t chunk_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, transfer_type);
    _mav_put_uint8_t(buf, 2, chunk_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN);
#else
    mavlink_magothy_license_transfer_request_t packet;
    packet.target_system = target_system;
    packet.transfer_type = transfer_type;
    packet.chunk_index = chunk_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_CRC);
}

/**
 * @brief Pack a magothy_license_transfer_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  system id of target system
 * @param transfer_type  file type
 * @param chunk_index  chunk index of license transfer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_license_transfer_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t transfer_type,uint8_t chunk_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, transfer_type);
    _mav_put_uint8_t(buf, 2, chunk_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN);
#else
    mavlink_magothy_license_transfer_request_t packet;
    packet.target_system = target_system;
    packet.transfer_type = transfer_type;
    packet.chunk_index = chunk_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_CRC);
}

/**
 * @brief Encode a magothy_license_transfer_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_license_transfer_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_license_transfer_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_license_transfer_request_t* magothy_license_transfer_request)
{
    return mavlink_msg_magothy_license_transfer_request_pack(system_id, component_id, msg, magothy_license_transfer_request->target_system, magothy_license_transfer_request->transfer_type, magothy_license_transfer_request->chunk_index);
}

/**
 * @brief Encode a magothy_license_transfer_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_license_transfer_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_license_transfer_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_license_transfer_request_t* magothy_license_transfer_request)
{
    return mavlink_msg_magothy_license_transfer_request_pack_chan(system_id, component_id, chan, msg, magothy_license_transfer_request->target_system, magothy_license_transfer_request->transfer_type, magothy_license_transfer_request->chunk_index);
}

/**
 * @brief Send a magothy_license_transfer_request message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  system id of target system
 * @param transfer_type  file type
 * @param chunk_index  chunk index of license transfer
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_license_transfer_request_send(mavlink_channel_t chan, uint8_t target_system, uint8_t transfer_type, uint8_t chunk_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, transfer_type);
    _mav_put_uint8_t(buf, 2, chunk_index);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST, buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_CRC);
#else
    mavlink_magothy_license_transfer_request_t packet;
    packet.target_system = target_system;
    packet.transfer_type = transfer_type;
    packet.chunk_index = chunk_index;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_CRC);
#endif
}

/**
 * @brief Send a magothy_license_transfer_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_license_transfer_request_send_struct(mavlink_channel_t chan, const mavlink_magothy_license_transfer_request_t* magothy_license_transfer_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_license_transfer_request_send(chan, magothy_license_transfer_request->target_system, magothy_license_transfer_request->transfer_type, magothy_license_transfer_request->chunk_index);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST, (const char *)magothy_license_transfer_request, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_license_transfer_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t transfer_type, uint8_t chunk_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, target_system);
    _mav_put_uint8_t(buf, 1, transfer_type);
    _mav_put_uint8_t(buf, 2, chunk_index);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST, buf, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_CRC);
#else
    mavlink_magothy_license_transfer_request_t *packet = (mavlink_magothy_license_transfer_request_t *)msgbuf;
    packet->target_system = target_system;
    packet->transfer_type = transfer_type;
    packet->chunk_index = chunk_index;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_LICENSE_TRANSFER_REQUEST UNPACKING


/**
 * @brief Get field target_system from magothy_license_transfer_request message
 *
 * @return  system id of target system
 */
static inline uint8_t mavlink_msg_magothy_license_transfer_request_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field transfer_type from magothy_license_transfer_request message
 *
 * @return  file type
 */
static inline uint8_t mavlink_msg_magothy_license_transfer_request_get_transfer_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field chunk_index from magothy_license_transfer_request message
 *
 * @return  chunk index of license transfer
 */
static inline uint8_t mavlink_msg_magothy_license_transfer_request_get_chunk_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a magothy_license_transfer_request message into a struct
 *
 * @param msg The message to decode
 * @param magothy_license_transfer_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_license_transfer_request_decode(const mavlink_message_t* msg, mavlink_magothy_license_transfer_request_t* magothy_license_transfer_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_license_transfer_request->target_system = mavlink_msg_magothy_license_transfer_request_get_target_system(msg);
    magothy_license_transfer_request->transfer_type = mavlink_msg_magothy_license_transfer_request_get_transfer_type(msg);
    magothy_license_transfer_request->chunk_index = mavlink_msg_magothy_license_transfer_request_get_chunk_index(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN;
        memset(magothy_license_transfer_request, 0, MAVLINK_MSG_ID_MAGOTHY_LICENSE_TRANSFER_REQUEST_LEN);
    memcpy(magothy_license_transfer_request, _MAV_PAYLOAD(msg), len);
#endif
}
