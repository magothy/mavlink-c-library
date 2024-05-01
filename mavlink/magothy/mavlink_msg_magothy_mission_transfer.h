#pragma once
// MESSAGE MAGOTHY_MISSION_TRANSFER PACKING

#define MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER 50251


typedef struct __mavlink_magothy_mission_transfer_t {
 uint16_t crc16; /*<  CRC-16/CCITT-FALSE of complete payload*/
 uint8_t target_system; /*<  system id of target system*/
 uint8_t session_id; /*<  increment at the beginning of each "session", can be used to correlate different chunks or detect new sessions*/
 uint8_t filename_index; /*<  save uploaded mission to file mission{index}.yml. if value == 255, skip this step*/
 uint8_t chunk_index; /*<  chunk index*/
 uint8_t num_chunk; /*<  total number of chunks*/
 uint8_t payload_len; /*<  length of this chunk payload*/
 uint8_t payload[247]; /*<  payload of this chunk*/
} mavlink_magothy_mission_transfer_t;

#define MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN 255
#define MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_MIN_LEN 255
#define MAVLINK_MSG_ID_50251_LEN 255
#define MAVLINK_MSG_ID_50251_MIN_LEN 255

#define MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_CRC 162
#define MAVLINK_MSG_ID_50251_CRC 162

#define MAVLINK_MSG_MAGOTHY_MISSION_TRANSFER_FIELD_PAYLOAD_LEN 247

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_MISSION_TRANSFER { \
    50251, \
    "MAGOTHY_MISSION_TRANSFER", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_magothy_mission_transfer_t, target_system) }, \
         { "session_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_magothy_mission_transfer_t, session_id) }, \
         { "filename_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_magothy_mission_transfer_t, filename_index) }, \
         { "chunk_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_magothy_mission_transfer_t, chunk_index) }, \
         { "num_chunk", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_magothy_mission_transfer_t, num_chunk) }, \
         { "crc16", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_magothy_mission_transfer_t, crc16) }, \
         { "payload_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_magothy_mission_transfer_t, payload_len) }, \
         { "payload", NULL, MAVLINK_TYPE_UINT8_T, 247, 8, offsetof(mavlink_magothy_mission_transfer_t, payload) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_MISSION_TRANSFER { \
    "MAGOTHY_MISSION_TRANSFER", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_magothy_mission_transfer_t, target_system) }, \
         { "session_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_magothy_mission_transfer_t, session_id) }, \
         { "filename_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_magothy_mission_transfer_t, filename_index) }, \
         { "chunk_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_magothy_mission_transfer_t, chunk_index) }, \
         { "num_chunk", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_magothy_mission_transfer_t, num_chunk) }, \
         { "crc16", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_magothy_mission_transfer_t, crc16) }, \
         { "payload_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_magothy_mission_transfer_t, payload_len) }, \
         { "payload", NULL, MAVLINK_TYPE_UINT8_T, 247, 8, offsetof(mavlink_magothy_mission_transfer_t, payload) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_mission_transfer message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  system id of target system
 * @param session_id  increment at the beginning of each "session", can be used to correlate different chunks or detect new sessions
 * @param filename_index  save uploaded mission to file mission{index}.yml. if value == 255, skip this step
 * @param chunk_index  chunk index
 * @param num_chunk  total number of chunks
 * @param crc16  CRC-16/CCITT-FALSE of complete payload
 * @param payload_len  length of this chunk payload
 * @param payload  payload of this chunk
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_mission_transfer_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t session_id, uint8_t filename_index, uint8_t chunk_index, uint8_t num_chunk, uint16_t crc16, uint8_t payload_len, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN];
    _mav_put_uint16_t(buf, 0, crc16);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, session_id);
    _mav_put_uint8_t(buf, 4, filename_index);
    _mav_put_uint8_t(buf, 5, chunk_index);
    _mav_put_uint8_t(buf, 6, num_chunk);
    _mav_put_uint8_t(buf, 7, payload_len);
    _mav_put_uint8_t_array(buf, 8, payload, 247);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN);
#else
    mavlink_magothy_mission_transfer_t packet;
    packet.crc16 = crc16;
    packet.target_system = target_system;
    packet.session_id = session_id;
    packet.filename_index = filename_index;
    packet.chunk_index = chunk_index;
    packet.num_chunk = num_chunk;
    packet.payload_len = payload_len;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*247);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_CRC);
}

/**
 * @brief Pack a magothy_mission_transfer message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  system id of target system
 * @param session_id  increment at the beginning of each "session", can be used to correlate different chunks or detect new sessions
 * @param filename_index  save uploaded mission to file mission{index}.yml. if value == 255, skip this step
 * @param chunk_index  chunk index
 * @param num_chunk  total number of chunks
 * @param crc16  CRC-16/CCITT-FALSE of complete payload
 * @param payload_len  length of this chunk payload
 * @param payload  payload of this chunk
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_mission_transfer_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t session_id,uint8_t filename_index,uint8_t chunk_index,uint8_t num_chunk,uint16_t crc16,uint8_t payload_len,const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN];
    _mav_put_uint16_t(buf, 0, crc16);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, session_id);
    _mav_put_uint8_t(buf, 4, filename_index);
    _mav_put_uint8_t(buf, 5, chunk_index);
    _mav_put_uint8_t(buf, 6, num_chunk);
    _mav_put_uint8_t(buf, 7, payload_len);
    _mav_put_uint8_t_array(buf, 8, payload, 247);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN);
#else
    mavlink_magothy_mission_transfer_t packet;
    packet.crc16 = crc16;
    packet.target_system = target_system;
    packet.session_id = session_id;
    packet.filename_index = filename_index;
    packet.chunk_index = chunk_index;
    packet.num_chunk = num_chunk;
    packet.payload_len = payload_len;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*247);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_CRC);
}

/**
 * @brief Encode a magothy_mission_transfer struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_mission_transfer C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_mission_transfer_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_mission_transfer_t* magothy_mission_transfer)
{
    return mavlink_msg_magothy_mission_transfer_pack(system_id, component_id, msg, magothy_mission_transfer->target_system, magothy_mission_transfer->session_id, magothy_mission_transfer->filename_index, magothy_mission_transfer->chunk_index, magothy_mission_transfer->num_chunk, magothy_mission_transfer->crc16, magothy_mission_transfer->payload_len, magothy_mission_transfer->payload);
}

/**
 * @brief Encode a magothy_mission_transfer struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_mission_transfer C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_mission_transfer_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_mission_transfer_t* magothy_mission_transfer)
{
    return mavlink_msg_magothy_mission_transfer_pack_chan(system_id, component_id, chan, msg, magothy_mission_transfer->target_system, magothy_mission_transfer->session_id, magothy_mission_transfer->filename_index, magothy_mission_transfer->chunk_index, magothy_mission_transfer->num_chunk, magothy_mission_transfer->crc16, magothy_mission_transfer->payload_len, magothy_mission_transfer->payload);
}

/**
 * @brief Send a magothy_mission_transfer message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  system id of target system
 * @param session_id  increment at the beginning of each "session", can be used to correlate different chunks or detect new sessions
 * @param filename_index  save uploaded mission to file mission{index}.yml. if value == 255, skip this step
 * @param chunk_index  chunk index
 * @param num_chunk  total number of chunks
 * @param crc16  CRC-16/CCITT-FALSE of complete payload
 * @param payload_len  length of this chunk payload
 * @param payload  payload of this chunk
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_mission_transfer_send(mavlink_channel_t chan, uint8_t target_system, uint8_t session_id, uint8_t filename_index, uint8_t chunk_index, uint8_t num_chunk, uint16_t crc16, uint8_t payload_len, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN];
    _mav_put_uint16_t(buf, 0, crc16);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, session_id);
    _mav_put_uint8_t(buf, 4, filename_index);
    _mav_put_uint8_t(buf, 5, chunk_index);
    _mav_put_uint8_t(buf, 6, num_chunk);
    _mav_put_uint8_t(buf, 7, payload_len);
    _mav_put_uint8_t_array(buf, 8, payload, 247);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER, buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_CRC);
#else
    mavlink_magothy_mission_transfer_t packet;
    packet.crc16 = crc16;
    packet.target_system = target_system;
    packet.session_id = session_id;
    packet.filename_index = filename_index;
    packet.chunk_index = chunk_index;
    packet.num_chunk = num_chunk;
    packet.payload_len = payload_len;
    mav_array_memcpy(packet.payload, payload, sizeof(uint8_t)*247);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_CRC);
#endif
}

/**
 * @brief Send a magothy_mission_transfer message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_mission_transfer_send_struct(mavlink_channel_t chan, const mavlink_magothy_mission_transfer_t* magothy_mission_transfer)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_mission_transfer_send(chan, magothy_mission_transfer->target_system, magothy_mission_transfer->session_id, magothy_mission_transfer->filename_index, magothy_mission_transfer->chunk_index, magothy_mission_transfer->num_chunk, magothy_mission_transfer->crc16, magothy_mission_transfer->payload_len, magothy_mission_transfer->payload);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER, (const char *)magothy_mission_transfer, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_mission_transfer_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t session_id, uint8_t filename_index, uint8_t chunk_index, uint8_t num_chunk, uint16_t crc16, uint8_t payload_len, const uint8_t *payload)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, crc16);
    _mav_put_uint8_t(buf, 2, target_system);
    _mav_put_uint8_t(buf, 3, session_id);
    _mav_put_uint8_t(buf, 4, filename_index);
    _mav_put_uint8_t(buf, 5, chunk_index);
    _mav_put_uint8_t(buf, 6, num_chunk);
    _mav_put_uint8_t(buf, 7, payload_len);
    _mav_put_uint8_t_array(buf, 8, payload, 247);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER, buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_CRC);
#else
    mavlink_magothy_mission_transfer_t *packet = (mavlink_magothy_mission_transfer_t *)msgbuf;
    packet->crc16 = crc16;
    packet->target_system = target_system;
    packet->session_id = session_id;
    packet->filename_index = filename_index;
    packet->chunk_index = chunk_index;
    packet->num_chunk = num_chunk;
    packet->payload_len = payload_len;
    mav_array_memcpy(packet->payload, payload, sizeof(uint8_t)*247);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_MISSION_TRANSFER UNPACKING


/**
 * @brief Get field target_system from magothy_mission_transfer message
 *
 * @return  system id of target system
 */
static inline uint8_t mavlink_msg_magothy_mission_transfer_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field session_id from magothy_mission_transfer message
 *
 * @return  increment at the beginning of each "session", can be used to correlate different chunks or detect new sessions
 */
static inline uint8_t mavlink_msg_magothy_mission_transfer_get_session_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field filename_index from magothy_mission_transfer message
 *
 * @return  save uploaded mission to file mission{index}.yml. if value == 255, skip this step
 */
static inline uint8_t mavlink_msg_magothy_mission_transfer_get_filename_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field chunk_index from magothy_mission_transfer message
 *
 * @return  chunk index
 */
static inline uint8_t mavlink_msg_magothy_mission_transfer_get_chunk_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field num_chunk from magothy_mission_transfer message
 *
 * @return  total number of chunks
 */
static inline uint8_t mavlink_msg_magothy_mission_transfer_get_num_chunk(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field crc16 from magothy_mission_transfer message
 *
 * @return  CRC-16/CCITT-FALSE of complete payload
 */
static inline uint16_t mavlink_msg_magothy_mission_transfer_get_crc16(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field payload_len from magothy_mission_transfer message
 *
 * @return  length of this chunk payload
 */
static inline uint8_t mavlink_msg_magothy_mission_transfer_get_payload_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field payload from magothy_mission_transfer message
 *
 * @return  payload of this chunk
 */
static inline uint16_t mavlink_msg_magothy_mission_transfer_get_payload(const mavlink_message_t* msg, uint8_t *payload)
{
    return _MAV_RETURN_uint8_t_array(msg, payload, 247,  8);
}

/**
 * @brief Decode a magothy_mission_transfer message into a struct
 *
 * @param msg The message to decode
 * @param magothy_mission_transfer C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_mission_transfer_decode(const mavlink_message_t* msg, mavlink_magothy_mission_transfer_t* magothy_mission_transfer)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_mission_transfer->crc16 = mavlink_msg_magothy_mission_transfer_get_crc16(msg);
    magothy_mission_transfer->target_system = mavlink_msg_magothy_mission_transfer_get_target_system(msg);
    magothy_mission_transfer->session_id = mavlink_msg_magothy_mission_transfer_get_session_id(msg);
    magothy_mission_transfer->filename_index = mavlink_msg_magothy_mission_transfer_get_filename_index(msg);
    magothy_mission_transfer->chunk_index = mavlink_msg_magothy_mission_transfer_get_chunk_index(msg);
    magothy_mission_transfer->num_chunk = mavlink_msg_magothy_mission_transfer_get_num_chunk(msg);
    magothy_mission_transfer->payload_len = mavlink_msg_magothy_mission_transfer_get_payload_len(msg);
    mavlink_msg_magothy_mission_transfer_get_payload(msg, magothy_mission_transfer->payload);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN;
        memset(magothy_mission_transfer, 0, MAVLINK_MSG_ID_MAGOTHY_MISSION_TRANSFER_LEN);
    memcpy(magothy_mission_transfer, _MAV_PAYLOAD(msg), len);
#endif
}
