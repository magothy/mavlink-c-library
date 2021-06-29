#pragma once
// MESSAGE MAGOTHY_WATER_CURRENT PACKING

#define MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT 50003


typedef struct __mavlink_magothy_water_current_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float water_current_speed_mps; /*<  Water Current Speed (m/s)*/
 float water_current_direction_deg; /*<  Water Current Direction (deg)*/
 float water_current_age_s; /*<  Water Current Age (s)*/
} mavlink_magothy_water_current_t;

#define MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN 20
#define MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_MIN_LEN 20
#define MAVLINK_MSG_ID_50003_LEN 20
#define MAVLINK_MSG_ID_50003_MIN_LEN 20

#define MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_CRC 186
#define MAVLINK_MSG_ID_50003_CRC 186



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_WATER_CURRENT { \
    50003, \
    "MAGOTHY_WATER_CURRENT", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_magothy_water_current_t, time_usec) }, \
         { "water_current_speed_mps", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_magothy_water_current_t, water_current_speed_mps) }, \
         { "water_current_direction_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_magothy_water_current_t, water_current_direction_deg) }, \
         { "water_current_age_s", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_magothy_water_current_t, water_current_age_s) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_WATER_CURRENT { \
    "MAGOTHY_WATER_CURRENT", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_magothy_water_current_t, time_usec) }, \
         { "water_current_speed_mps", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_magothy_water_current_t, water_current_speed_mps) }, \
         { "water_current_direction_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_magothy_water_current_t, water_current_direction_deg) }, \
         { "water_current_age_s", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_magothy_water_current_t, water_current_age_s) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_water_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param water_current_speed_mps  Water Current Speed (m/s)
 * @param water_current_direction_deg  Water Current Direction (deg)
 * @param water_current_age_s  Water Current Age (s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_water_current_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float water_current_speed_mps, float water_current_direction_deg, float water_current_age_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, water_current_speed_mps);
    _mav_put_float(buf, 12, water_current_direction_deg);
    _mav_put_float(buf, 16, water_current_age_s);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN);
#else
    mavlink_magothy_water_current_t packet;
    packet.time_usec = time_usec;
    packet.water_current_speed_mps = water_current_speed_mps;
    packet.water_current_direction_deg = water_current_direction_deg;
    packet.water_current_age_s = water_current_age_s;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_CRC);
}

/**
 * @brief Pack a magothy_water_current message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param water_current_speed_mps  Water Current Speed (m/s)
 * @param water_current_direction_deg  Water Current Direction (deg)
 * @param water_current_age_s  Water Current Age (s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_water_current_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float water_current_speed_mps,float water_current_direction_deg,float water_current_age_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, water_current_speed_mps);
    _mav_put_float(buf, 12, water_current_direction_deg);
    _mav_put_float(buf, 16, water_current_age_s);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN);
#else
    mavlink_magothy_water_current_t packet;
    packet.time_usec = time_usec;
    packet.water_current_speed_mps = water_current_speed_mps;
    packet.water_current_direction_deg = water_current_direction_deg;
    packet.water_current_age_s = water_current_age_s;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_CRC);
}

/**
 * @brief Encode a magothy_water_current struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_water_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_water_current_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_water_current_t* magothy_water_current)
{
    return mavlink_msg_magothy_water_current_pack(system_id, component_id, msg, magothy_water_current->time_usec, magothy_water_current->water_current_speed_mps, magothy_water_current->water_current_direction_deg, magothy_water_current->water_current_age_s);
}

/**
 * @brief Encode a magothy_water_current struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_water_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_water_current_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_water_current_t* magothy_water_current)
{
    return mavlink_msg_magothy_water_current_pack_chan(system_id, component_id, chan, msg, magothy_water_current->time_usec, magothy_water_current->water_current_speed_mps, magothy_water_current->water_current_direction_deg, magothy_water_current->water_current_age_s);
}

/**
 * @brief Send a magothy_water_current message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param water_current_speed_mps  Water Current Speed (m/s)
 * @param water_current_direction_deg  Water Current Direction (deg)
 * @param water_current_age_s  Water Current Age (s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_water_current_send(mavlink_channel_t chan, uint64_t time_usec, float water_current_speed_mps, float water_current_direction_deg, float water_current_age_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, water_current_speed_mps);
    _mav_put_float(buf, 12, water_current_direction_deg);
    _mav_put_float(buf, 16, water_current_age_s);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT, buf, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_CRC);
#else
    mavlink_magothy_water_current_t packet;
    packet.time_usec = time_usec;
    packet.water_current_speed_mps = water_current_speed_mps;
    packet.water_current_direction_deg = water_current_direction_deg;
    packet.water_current_age_s = water_current_age_s;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_CRC);
#endif
}

/**
 * @brief Send a magothy_water_current message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_water_current_send_struct(mavlink_channel_t chan, const mavlink_magothy_water_current_t* magothy_water_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_water_current_send(chan, magothy_water_current->time_usec, magothy_water_current->water_current_speed_mps, magothy_water_current->water_current_direction_deg, magothy_water_current->water_current_age_s);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT, (const char *)magothy_water_current, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_water_current_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float water_current_speed_mps, float water_current_direction_deg, float water_current_age_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, water_current_speed_mps);
    _mav_put_float(buf, 12, water_current_direction_deg);
    _mav_put_float(buf, 16, water_current_age_s);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT, buf, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_CRC);
#else
    mavlink_magothy_water_current_t *packet = (mavlink_magothy_water_current_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->water_current_speed_mps = water_current_speed_mps;
    packet->water_current_direction_deg = water_current_direction_deg;
    packet->water_current_age_s = water_current_age_s;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_WATER_CURRENT UNPACKING


/**
 * @brief Get field time_usec from magothy_water_current message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_magothy_water_current_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field water_current_speed_mps from magothy_water_current message
 *
 * @return  Water Current Speed (m/s)
 */
static inline float mavlink_msg_magothy_water_current_get_water_current_speed_mps(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field water_current_direction_deg from magothy_water_current message
 *
 * @return  Water Current Direction (deg)
 */
static inline float mavlink_msg_magothy_water_current_get_water_current_direction_deg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field water_current_age_s from magothy_water_current message
 *
 * @return  Water Current Age (s)
 */
static inline float mavlink_msg_magothy_water_current_get_water_current_age_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a magothy_water_current message into a struct
 *
 * @param msg The message to decode
 * @param magothy_water_current C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_water_current_decode(const mavlink_message_t* msg, mavlink_magothy_water_current_t* magothy_water_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_water_current->time_usec = mavlink_msg_magothy_water_current_get_time_usec(msg);
    magothy_water_current->water_current_speed_mps = mavlink_msg_magothy_water_current_get_water_current_speed_mps(msg);
    magothy_water_current->water_current_direction_deg = mavlink_msg_magothy_water_current_get_water_current_direction_deg(msg);
    magothy_water_current->water_current_age_s = mavlink_msg_magothy_water_current_get_water_current_age_s(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN;
        memset(magothy_water_current, 0, MAVLINK_MSG_ID_MAGOTHY_WATER_CURRENT_LEN);
    memcpy(magothy_water_current, _MAV_PAYLOAD(msg), len);
#endif
}
