#pragma once
// MESSAGE MAGOTHY_PROTOBUF_PROXY PACKING

#define MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY 50005


typedef struct __mavlink_magothy_protobuf_proxy_t {
 uint16_t proto_id; /*<  Message ID to corresponding to protobuf message type*/
 uint8_t is_compressed; /*<  0 - not compressed, 1 - compressed with zstd*/
 uint8_t data_len; /*<  length of serialized protobuf message */
 uint8_t data[251]; /*<  serialized protobuf message buffer*/
} mavlink_magothy_protobuf_proxy_t;

#define MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN 255
#define MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_MIN_LEN 255
#define MAVLINK_MSG_ID_50005_LEN 255
#define MAVLINK_MSG_ID_50005_MIN_LEN 255

#define MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_CRC 224
#define MAVLINK_MSG_ID_50005_CRC 224

#define MAVLINK_MSG_MAGOTHY_PROTOBUF_PROXY_FIELD_DATA_LEN 251

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_PROTOBUF_PROXY { \
    50005, \
    "MAGOTHY_PROTOBUF_PROXY", \
    4, \
    {  { "proto_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_magothy_protobuf_proxy_t, proto_id) }, \
         { "is_compressed", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_magothy_protobuf_proxy_t, is_compressed) }, \
         { "data_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_magothy_protobuf_proxy_t, data_len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 251, 4, offsetof(mavlink_magothy_protobuf_proxy_t, data) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_PROTOBUF_PROXY { \
    "MAGOTHY_PROTOBUF_PROXY", \
    4, \
    {  { "proto_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_magothy_protobuf_proxy_t, proto_id) }, \
         { "is_compressed", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_magothy_protobuf_proxy_t, is_compressed) }, \
         { "data_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_magothy_protobuf_proxy_t, data_len) }, \
         { "data", NULL, MAVLINK_TYPE_UINT8_T, 251, 4, offsetof(mavlink_magothy_protobuf_proxy_t, data) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_protobuf_proxy message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param proto_id  Message ID to corresponding to protobuf message type
 * @param is_compressed  0 - not compressed, 1 - compressed with zstd
 * @param data_len  length of serialized protobuf message 
 * @param data  serialized protobuf message buffer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_protobuf_proxy_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t proto_id, uint8_t is_compressed, uint8_t data_len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN];
    _mav_put_uint16_t(buf, 0, proto_id);
    _mav_put_uint8_t(buf, 2, is_compressed);
    _mav_put_uint8_t(buf, 3, data_len);
    _mav_put_uint8_t_array(buf, 4, data, 251);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN);
#else
    mavlink_magothy_protobuf_proxy_t packet;
    packet.proto_id = proto_id;
    packet.is_compressed = is_compressed;
    packet.data_len = data_len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*251);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_CRC);
}

/**
 * @brief Pack a magothy_protobuf_proxy message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param proto_id  Message ID to corresponding to protobuf message type
 * @param is_compressed  0 - not compressed, 1 - compressed with zstd
 * @param data_len  length of serialized protobuf message 
 * @param data  serialized protobuf message buffer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_protobuf_proxy_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t proto_id,uint8_t is_compressed,uint8_t data_len,const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN];
    _mav_put_uint16_t(buf, 0, proto_id);
    _mav_put_uint8_t(buf, 2, is_compressed);
    _mav_put_uint8_t(buf, 3, data_len);
    _mav_put_uint8_t_array(buf, 4, data, 251);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN);
#else
    mavlink_magothy_protobuf_proxy_t packet;
    packet.proto_id = proto_id;
    packet.is_compressed = is_compressed;
    packet.data_len = data_len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*251);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_CRC);
}

/**
 * @brief Encode a magothy_protobuf_proxy struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_protobuf_proxy C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_protobuf_proxy_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_protobuf_proxy_t* magothy_protobuf_proxy)
{
    return mavlink_msg_magothy_protobuf_proxy_pack(system_id, component_id, msg, magothy_protobuf_proxy->proto_id, magothy_protobuf_proxy->is_compressed, magothy_protobuf_proxy->data_len, magothy_protobuf_proxy->data);
}

/**
 * @brief Encode a magothy_protobuf_proxy struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_protobuf_proxy C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_protobuf_proxy_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_protobuf_proxy_t* magothy_protobuf_proxy)
{
    return mavlink_msg_magothy_protobuf_proxy_pack_chan(system_id, component_id, chan, msg, magothy_protobuf_proxy->proto_id, magothy_protobuf_proxy->is_compressed, magothy_protobuf_proxy->data_len, magothy_protobuf_proxy->data);
}

/**
 * @brief Send a magothy_protobuf_proxy message
 * @param chan MAVLink channel to send the message
 *
 * @param proto_id  Message ID to corresponding to protobuf message type
 * @param is_compressed  0 - not compressed, 1 - compressed with zstd
 * @param data_len  length of serialized protobuf message 
 * @param data  serialized protobuf message buffer
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_protobuf_proxy_send(mavlink_channel_t chan, uint16_t proto_id, uint8_t is_compressed, uint8_t data_len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN];
    _mav_put_uint16_t(buf, 0, proto_id);
    _mav_put_uint8_t(buf, 2, is_compressed);
    _mav_put_uint8_t(buf, 3, data_len);
    _mav_put_uint8_t_array(buf, 4, data, 251);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY, buf, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_CRC);
#else
    mavlink_magothy_protobuf_proxy_t packet;
    packet.proto_id = proto_id;
    packet.is_compressed = is_compressed;
    packet.data_len = data_len;
    mav_array_memcpy(packet.data, data, sizeof(uint8_t)*251);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_CRC);
#endif
}

/**
 * @brief Send a magothy_protobuf_proxy message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_protobuf_proxy_send_struct(mavlink_channel_t chan, const mavlink_magothy_protobuf_proxy_t* magothy_protobuf_proxy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_protobuf_proxy_send(chan, magothy_protobuf_proxy->proto_id, magothy_protobuf_proxy->is_compressed, magothy_protobuf_proxy->data_len, magothy_protobuf_proxy->data);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY, (const char *)magothy_protobuf_proxy, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_protobuf_proxy_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t proto_id, uint8_t is_compressed, uint8_t data_len, const uint8_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, proto_id);
    _mav_put_uint8_t(buf, 2, is_compressed);
    _mav_put_uint8_t(buf, 3, data_len);
    _mav_put_uint8_t_array(buf, 4, data, 251);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY, buf, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_CRC);
#else
    mavlink_magothy_protobuf_proxy_t *packet = (mavlink_magothy_protobuf_proxy_t *)msgbuf;
    packet->proto_id = proto_id;
    packet->is_compressed = is_compressed;
    packet->data_len = data_len;
    mav_array_memcpy(packet->data, data, sizeof(uint8_t)*251);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_PROTOBUF_PROXY UNPACKING


/**
 * @brief Get field proto_id from magothy_protobuf_proxy message
 *
 * @return  Message ID to corresponding to protobuf message type
 */
static inline uint16_t mavlink_msg_magothy_protobuf_proxy_get_proto_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field is_compressed from magothy_protobuf_proxy message
 *
 * @return  0 - not compressed, 1 - compressed with zstd
 */
static inline uint8_t mavlink_msg_magothy_protobuf_proxy_get_is_compressed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field data_len from magothy_protobuf_proxy message
 *
 * @return  length of serialized protobuf message 
 */
static inline uint8_t mavlink_msg_magothy_protobuf_proxy_get_data_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field data from magothy_protobuf_proxy message
 *
 * @return  serialized protobuf message buffer
 */
static inline uint16_t mavlink_msg_magothy_protobuf_proxy_get_data(const mavlink_message_t* msg, uint8_t *data)
{
    return _MAV_RETURN_uint8_t_array(msg, data, 251,  4);
}

/**
 * @brief Decode a magothy_protobuf_proxy message into a struct
 *
 * @param msg The message to decode
 * @param magothy_protobuf_proxy C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_protobuf_proxy_decode(const mavlink_message_t* msg, mavlink_magothy_protobuf_proxy_t* magothy_protobuf_proxy)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_protobuf_proxy->proto_id = mavlink_msg_magothy_protobuf_proxy_get_proto_id(msg);
    magothy_protobuf_proxy->is_compressed = mavlink_msg_magothy_protobuf_proxy_get_is_compressed(msg);
    magothy_protobuf_proxy->data_len = mavlink_msg_magothy_protobuf_proxy_get_data_len(msg);
    mavlink_msg_magothy_protobuf_proxy_get_data(msg, magothy_protobuf_proxy->data);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN;
        memset(magothy_protobuf_proxy, 0, MAVLINK_MSG_ID_MAGOTHY_PROTOBUF_PROXY_LEN);
    memcpy(magothy_protobuf_proxy, _MAV_PAYLOAD(msg), len);
#endif
}
