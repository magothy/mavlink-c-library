#pragma once
// MESSAGE MAGOTHY_ECHOSOUNDER PACKING

#define MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER 50001


typedef struct __mavlink_magothy_echosounder_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float water_depth_low_freq_m; /*<  Water Depth (Low Frequency) (m)*/
 float water_depth_high_freq_m; /*<  Water Depth (High Frequency) (m)*/
 float water_temperature_C; /*<  Water Temperature (C)*/
} mavlink_magothy_echosounder_t;

#define MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN 20
#define MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_MIN_LEN 20
#define MAVLINK_MSG_ID_50001_LEN 20
#define MAVLINK_MSG_ID_50001_MIN_LEN 20

#define MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_CRC 234
#define MAVLINK_MSG_ID_50001_CRC 234



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_ECHOSOUNDER { \
    50001, \
    "MAGOTHY_ECHOSOUNDER", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_magothy_echosounder_t, time_usec) }, \
         { "water_depth_low_freq_m", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_magothy_echosounder_t, water_depth_low_freq_m) }, \
         { "water_depth_high_freq_m", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_magothy_echosounder_t, water_depth_high_freq_m) }, \
         { "water_temperature_C", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_magothy_echosounder_t, water_temperature_C) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_ECHOSOUNDER { \
    "MAGOTHY_ECHOSOUNDER", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_magothy_echosounder_t, time_usec) }, \
         { "water_depth_low_freq_m", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_magothy_echosounder_t, water_depth_low_freq_m) }, \
         { "water_depth_high_freq_m", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_magothy_echosounder_t, water_depth_high_freq_m) }, \
         { "water_temperature_C", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_magothy_echosounder_t, water_temperature_C) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_echosounder message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param water_depth_low_freq_m  Water Depth (Low Frequency) (m)
 * @param water_depth_high_freq_m  Water Depth (High Frequency) (m)
 * @param water_temperature_C  Water Temperature (C)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_echosounder_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float water_depth_low_freq_m, float water_depth_high_freq_m, float water_temperature_C)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, water_depth_low_freq_m);
    _mav_put_float(buf, 12, water_depth_high_freq_m);
    _mav_put_float(buf, 16, water_temperature_C);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN);
#else
    mavlink_magothy_echosounder_t packet;
    packet.time_usec = time_usec;
    packet.water_depth_low_freq_m = water_depth_low_freq_m;
    packet.water_depth_high_freq_m = water_depth_high_freq_m;
    packet.water_temperature_C = water_temperature_C;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_CRC);
}

/**
 * @brief Pack a magothy_echosounder message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param water_depth_low_freq_m  Water Depth (Low Frequency) (m)
 * @param water_depth_high_freq_m  Water Depth (High Frequency) (m)
 * @param water_temperature_C  Water Temperature (C)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_echosounder_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float water_depth_low_freq_m,float water_depth_high_freq_m,float water_temperature_C)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, water_depth_low_freq_m);
    _mav_put_float(buf, 12, water_depth_high_freq_m);
    _mav_put_float(buf, 16, water_temperature_C);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN);
#else
    mavlink_magothy_echosounder_t packet;
    packet.time_usec = time_usec;
    packet.water_depth_low_freq_m = water_depth_low_freq_m;
    packet.water_depth_high_freq_m = water_depth_high_freq_m;
    packet.water_temperature_C = water_temperature_C;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_CRC);
}

/**
 * @brief Encode a magothy_echosounder struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_echosounder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_echosounder_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_echosounder_t* magothy_echosounder)
{
    return mavlink_msg_magothy_echosounder_pack(system_id, component_id, msg, magothy_echosounder->time_usec, magothy_echosounder->water_depth_low_freq_m, magothy_echosounder->water_depth_high_freq_m, magothy_echosounder->water_temperature_C);
}

/**
 * @brief Encode a magothy_echosounder struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_echosounder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_echosounder_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_echosounder_t* magothy_echosounder)
{
    return mavlink_msg_magothy_echosounder_pack_chan(system_id, component_id, chan, msg, magothy_echosounder->time_usec, magothy_echosounder->water_depth_low_freq_m, magothy_echosounder->water_depth_high_freq_m, magothy_echosounder->water_temperature_C);
}

/**
 * @brief Send a magothy_echosounder message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param water_depth_low_freq_m  Water Depth (Low Frequency) (m)
 * @param water_depth_high_freq_m  Water Depth (High Frequency) (m)
 * @param water_temperature_C  Water Temperature (C)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_echosounder_send(mavlink_channel_t chan, uint64_t time_usec, float water_depth_low_freq_m, float water_depth_high_freq_m, float water_temperature_C)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, water_depth_low_freq_m);
    _mav_put_float(buf, 12, water_depth_high_freq_m);
    _mav_put_float(buf, 16, water_temperature_C);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER, buf, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_CRC);
#else
    mavlink_magothy_echosounder_t packet;
    packet.time_usec = time_usec;
    packet.water_depth_low_freq_m = water_depth_low_freq_m;
    packet.water_depth_high_freq_m = water_depth_high_freq_m;
    packet.water_temperature_C = water_temperature_C;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_CRC);
#endif
}

/**
 * @brief Send a magothy_echosounder message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_echosounder_send_struct(mavlink_channel_t chan, const mavlink_magothy_echosounder_t* magothy_echosounder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_echosounder_send(chan, magothy_echosounder->time_usec, magothy_echosounder->water_depth_low_freq_m, magothy_echosounder->water_depth_high_freq_m, magothy_echosounder->water_temperature_C);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER, (const char *)magothy_echosounder, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_echosounder_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float water_depth_low_freq_m, float water_depth_high_freq_m, float water_temperature_C)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, water_depth_low_freq_m);
    _mav_put_float(buf, 12, water_depth_high_freq_m);
    _mav_put_float(buf, 16, water_temperature_C);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER, buf, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_CRC);
#else
    mavlink_magothy_echosounder_t *packet = (mavlink_magothy_echosounder_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->water_depth_low_freq_m = water_depth_low_freq_m;
    packet->water_depth_high_freq_m = water_depth_high_freq_m;
    packet->water_temperature_C = water_temperature_C;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_ECHOSOUNDER UNPACKING


/**
 * @brief Get field time_usec from magothy_echosounder message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_magothy_echosounder_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field water_depth_low_freq_m from magothy_echosounder message
 *
 * @return  Water Depth (Low Frequency) (m)
 */
static inline float mavlink_msg_magothy_echosounder_get_water_depth_low_freq_m(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field water_depth_high_freq_m from magothy_echosounder message
 *
 * @return  Water Depth (High Frequency) (m)
 */
static inline float mavlink_msg_magothy_echosounder_get_water_depth_high_freq_m(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field water_temperature_C from magothy_echosounder message
 *
 * @return  Water Temperature (C)
 */
static inline float mavlink_msg_magothy_echosounder_get_water_temperature_C(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a magothy_echosounder message into a struct
 *
 * @param msg The message to decode
 * @param magothy_echosounder C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_echosounder_decode(const mavlink_message_t* msg, mavlink_magothy_echosounder_t* magothy_echosounder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_echosounder->time_usec = mavlink_msg_magothy_echosounder_get_time_usec(msg);
    magothy_echosounder->water_depth_low_freq_m = mavlink_msg_magothy_echosounder_get_water_depth_low_freq_m(msg);
    magothy_echosounder->water_depth_high_freq_m = mavlink_msg_magothy_echosounder_get_water_depth_high_freq_m(msg);
    magothy_echosounder->water_temperature_C = mavlink_msg_magothy_echosounder_get_water_temperature_C(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN;
        memset(magothy_echosounder, 0, MAVLINK_MSG_ID_MAGOTHY_ECHOSOUNDER_LEN);
    memcpy(magothy_echosounder, _MAV_PAYLOAD(msg), len);
#endif
}
