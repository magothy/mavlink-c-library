#pragma once
// MESSAGE MAGOTHY_MISSION_ACK PACKING

#define MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK 50252


typedef struct __mavlink_magothy_mission_ack_t {
 uint16_t crc16; /*<  CRC-16/CCITT-FALSE of complete payload*/
 uint8_t target_system; /*<  system id of target system*/
 uint8_t session_id; /*<  increment at the beginning of each "session", can be used to correlate different chunks or detect new sessions*/
 uint8_t chunk_index; /*<  chunk index*/
 uint8_t num_chunk; /*<  total number of chunks*/
 uint8_t result; /*<  result of command*/
} mavlink_magothy_mission_ack_t;

#define MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN 7
#define MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_MIN_LEN 7
#define MAVLINK_MSG_ID_50252_LEN 7
#define MAVLINK_MSG_ID_50252_MIN_LEN 7

#define MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_CRC 175
#define MAVLINK_MSG_ID_50252_CRC 175



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_MISSION_ACK { \
    50252, \
    "MAGOTHY_MISSION_ACK", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_magothy_mission_ack_t, target_system) }, \
         { "session_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_magothy_mission_ack_t, session_id) }, \
         { "chunk_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_magothy_mission_ack_t, chunk_index) }, \
         { "num_chunk", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_magothy_mission_ack_t, num_chunk) }, \
         { "crc16", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_magothy_mission_ack_t, crc16) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_magothy_mission_ack_t, result) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_MISSION_ACK { \
    "MAGOTHY_MISSION_ACK", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_magothy_mission_ack_t, target_system) }, \
         { "session_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_magothy_mission_ack_t, session_id) }, \
         { "chunk_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_magothy_mission_ack_t, chunk_index) }, \
         { "num_chunk", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_magothy_mission_ack_t, num_chunk) }, \
         { "crc16", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_magothy_mission_ack_t, crc16) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_magothy_mission_ack_t, result) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_mission_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  system id of target system
 * @param session_id  increment at the beginning of each "session", can be used to correlate different chunks or detect new sessions
 * @param chunk_index  chunk index
 * @param num_chunk  total number of chunks
 * @param crc16  CRC-16/CCITT-FALSE of complete payload
 * @param result  result of command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_mission_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t session_id, uint8_t chunk_index, uint8_t num_chunk, uint16_t crc16, uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN];
    _mav_put_uint16_t(buf, 0, crc16);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, session_id);
    _mav_put_uint8_t(buf, 4, chunk_index);
    _mav_put_uint8_t(buf, 5, num_chunk);
    _mav_put_uint8_t(buf, 6, result);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN);
#else
    mavlink_magothy_mission_ack_t packet;
    packet.crc16 = crc16;
    packet.target_system = target_system;
    packet.session_id = session_id;
    packet.chunk_index = chunk_index;
    packet.num_chunk = num_chunk;
    packet.result = result;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_CRC);
}

/**
 * @brief Pack a magothy_mission_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  system id of target system
 * @param session_id  increment at the beginning of each "session", can be used to correlate different chunks or detect new sessions
 * @param chunk_index  chunk index
 * @param num_chunk  total number of chunks
 * @param crc16  CRC-16/CCITT-FALSE of complete payload
 * @param result  result of command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_mission_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t session_id,uint8_t chunk_index,uint8_t num_chunk,uint16_t crc16,uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN];
    _mav_put_uint16_t(buf, 0, crc16);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, session_id);
    _mav_put_uint8_t(buf, 4, chunk_index);
    _mav_put_uint8_t(buf, 5, num_chunk);
    _mav_put_uint8_t(buf, 6, result);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN);
#else
    mavlink_magothy_mission_ack_t packet;
    packet.crc16 = crc16;
    packet.target_system = target_system;
    packet.session_id = session_id;
    packet.chunk_index = chunk_index;
    packet.num_chunk = num_chunk;
    packet.result = result;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_CRC);
}

/**
 * @brief Encode a magothy_mission_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_mission_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_mission_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_mission_ack_t* magothy_mission_ack)
{
    return mavlink_msg_magothy_mission_ack_pack(system_id, component_id, msg, magothy_mission_ack->target_system, magothy_mission_ack->session_id, magothy_mission_ack->chunk_index, magothy_mission_ack->num_chunk, magothy_mission_ack->crc16, magothy_mission_ack->result);
}

/**
 * @brief Encode a magothy_mission_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_mission_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_mission_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_mission_ack_t* magothy_mission_ack)
{
    return mavlink_msg_magothy_mission_ack_pack_chan(system_id, component_id, chan, msg, magothy_mission_ack->target_system, magothy_mission_ack->session_id, magothy_mission_ack->chunk_index, magothy_mission_ack->num_chunk, magothy_mission_ack->crc16, magothy_mission_ack->result);
}

/**
 * @brief Send a magothy_mission_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  system id of target system
 * @param session_id  increment at the beginning of each "session", can be used to correlate different chunks or detect new sessions
 * @param chunk_index  chunk index
 * @param num_chunk  total number of chunks
 * @param crc16  CRC-16/CCITT-FALSE of complete payload
 * @param result  result of command
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_mission_ack_send(mavlink_channel_t chan, uint8_t target_system, uint8_t session_id, uint8_t chunk_index, uint8_t num_chunk, uint16_t crc16, uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN];
    _mav_put_uint16_t(buf, 0, crc16);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, session_id);
    _mav_put_uint8_t(buf, 4, chunk_index);
    _mav_put_uint8_t(buf, 5, num_chunk);
    _mav_put_uint8_t(buf, 6, result);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK, buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_CRC);
#else
    mavlink_magothy_mission_ack_t packet;
    packet.crc16 = crc16;
    packet.target_system = target_system;
    packet.session_id = session_id;
    packet.chunk_index = chunk_index;
    packet.num_chunk = num_chunk;
    packet.result = result;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_CRC);
#endif
}

/**
 * @brief Send a magothy_mission_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_mission_ack_send_struct(mavlink_channel_t chan, const mavlink_magothy_mission_ack_t* magothy_mission_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_mission_ack_send(chan, magothy_mission_ack->target_system, magothy_mission_ack->session_id, magothy_mission_ack->chunk_index, magothy_mission_ack->num_chunk, magothy_mission_ack->crc16, magothy_mission_ack->result);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK, (const char *)magothy_mission_ack, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_mission_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t session_id, uint8_t chunk_index, uint8_t num_chunk, uint16_t crc16, uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, crc16);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, session_id);
    _mav_put_uint8_t(buf, 4, chunk_index);
    _mav_put_uint8_t(buf, 5, num_chunk);
    _mav_put_uint8_t(buf, 6, result);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK, buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_CRC);
#else
    mavlink_magothy_mission_ack_t *packet = (mavlink_magothy_mission_ack_t *)msgbuf;
    packet->crc16 = crc16;
    packet->target_system = target_system;
    packet->session_id = session_id;
    packet->chunk_index = chunk_index;
    packet->num_chunk = num_chunk;
    packet->result = result;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_MISSION_ACK UNPACKING


/**
 * @brief Get field target_system from magothy_mission_ack message
 *
 * @return  system id of target system
 */
static inline uint8_t mavlink_msg_magothy_mission_ack_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field session_id from magothy_mission_ack message
 *
 * @return  increment at the beginning of each "session", can be used to correlate different chunks or detect new sessions
 */
static inline uint8_t mavlink_msg_magothy_mission_ack_get_session_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field chunk_index from magothy_mission_ack message
 *
 * @return  chunk index
 */
static inline uint8_t mavlink_msg_magothy_mission_ack_get_chunk_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field num_chunk from magothy_mission_ack message
 *
 * @return  total number of chunks
 */
static inline uint8_t mavlink_msg_magothy_mission_ack_get_num_chunk(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field crc16 from magothy_mission_ack message
 *
 * @return  CRC-16/CCITT-FALSE of complete payload
 */
static inline uint16_t mavlink_msg_magothy_mission_ack_get_crc16(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field result from magothy_mission_ack message
 *
 * @return  result of command
 */
static inline uint8_t mavlink_msg_magothy_mission_ack_get_result(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a magothy_mission_ack message into a struct
 *
 * @param msg The message to decode
 * @param magothy_mission_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_mission_ack_decode(const mavlink_message_t* msg, mavlink_magothy_mission_ack_t* magothy_mission_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_mission_ack->crc16 = mavlink_msg_magothy_mission_ack_get_crc16(msg);
    magothy_mission_ack->target_system = mavlink_msg_magothy_mission_ack_get_target_system(msg);
    magothy_mission_ack->session_id = mavlink_msg_magothy_mission_ack_get_session_id(msg);
    magothy_mission_ack->chunk_index = mavlink_msg_magothy_mission_ack_get_chunk_index(msg);
    magothy_mission_ack->num_chunk = mavlink_msg_magothy_mission_ack_get_num_chunk(msg);
    magothy_mission_ack->result = mavlink_msg_magothy_mission_ack_get_result(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN;
        memset(magothy_mission_ack, 0, MAVLINK_MSG_ID_MAGOTHY_MISSION_ACK_LEN);
    memcpy(magothy_mission_ack, _MAV_PAYLOAD(msg), len);
#endif
}
