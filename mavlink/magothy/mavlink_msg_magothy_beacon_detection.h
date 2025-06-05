#pragma once
// MESSAGE MAGOTHY_BEACON_DETECTION PACKING

#define MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION 50006


typedef struct __mavlink_magothy_beacon_detection_t {
 float lat_deg; /*<  Latitude (deg)*/
 float lon_deg; /*<  Longitude (deg)*/
 float speed_mps; /*<  Speed (m/s)*/
 float course_deg; /*<  Course (deg)*/
 float error_m; /*<  Estimated Position Error (m)*/
 uint16_t position_mask; /*<  12-bit bitmask, 30 deg increments, bit 0 == 0deg, bit 1 == 30deg, ...*/
 uint8_t beacon_id; /*<  beacon id of detection*/
} mavlink_magothy_beacon_detection_t;

#define MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN 23
#define MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_MIN_LEN 23
#define MAVLINK_MSG_ID_50006_LEN 23
#define MAVLINK_MSG_ID_50006_MIN_LEN 23

#define MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_CRC 89
#define MAVLINK_MSG_ID_50006_CRC 89



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_BEACON_DETECTION { \
    50006, \
    "MAGOTHY_BEACON_DETECTION", \
    7, \
    {  { "beacon_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_magothy_beacon_detection_t, beacon_id) }, \
         { "position_mask", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_magothy_beacon_detection_t, position_mask) }, \
         { "lat_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_magothy_beacon_detection_t, lat_deg) }, \
         { "lon_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_magothy_beacon_detection_t, lon_deg) }, \
         { "speed_mps", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_magothy_beacon_detection_t, speed_mps) }, \
         { "course_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_magothy_beacon_detection_t, course_deg) }, \
         { "error_m", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_magothy_beacon_detection_t, error_m) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_BEACON_DETECTION { \
    "MAGOTHY_BEACON_DETECTION", \
    7, \
    {  { "beacon_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_magothy_beacon_detection_t, beacon_id) }, \
         { "position_mask", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_magothy_beacon_detection_t, position_mask) }, \
         { "lat_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_magothy_beacon_detection_t, lat_deg) }, \
         { "lon_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_magothy_beacon_detection_t, lon_deg) }, \
         { "speed_mps", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_magothy_beacon_detection_t, speed_mps) }, \
         { "course_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_magothy_beacon_detection_t, course_deg) }, \
         { "error_m", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_magothy_beacon_detection_t, error_m) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_beacon_detection message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param beacon_id  beacon id of detection
 * @param position_mask  12-bit bitmask, 30 deg increments, bit 0 == 0deg, bit 1 == 30deg, ...
 * @param lat_deg  Latitude (deg)
 * @param lon_deg  Longitude (deg)
 * @param speed_mps  Speed (m/s)
 * @param course_deg  Course (deg)
 * @param error_m  Estimated Position Error (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_beacon_detection_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t beacon_id, uint16_t position_mask, float lat_deg, float lon_deg, float speed_mps, float course_deg, float error_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN];
    _mav_put_float(buf, 0, lat_deg);
    _mav_put_float(buf, 4, lon_deg);
    _mav_put_float(buf, 8, speed_mps);
    _mav_put_float(buf, 12, course_deg);
    _mav_put_float(buf, 16, error_m);
    _mav_put_uint16_t(buf, 20, position_mask);
    _mav_put_uint8_t(buf, 22, beacon_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN);
#else
    mavlink_magothy_beacon_detection_t packet;
    packet.lat_deg = lat_deg;
    packet.lon_deg = lon_deg;
    packet.speed_mps = speed_mps;
    packet.course_deg = course_deg;
    packet.error_m = error_m;
    packet.position_mask = position_mask;
    packet.beacon_id = beacon_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_CRC);
}

/**
 * @brief Pack a magothy_beacon_detection message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param beacon_id  beacon id of detection
 * @param position_mask  12-bit bitmask, 30 deg increments, bit 0 == 0deg, bit 1 == 30deg, ...
 * @param lat_deg  Latitude (deg)
 * @param lon_deg  Longitude (deg)
 * @param speed_mps  Speed (m/s)
 * @param course_deg  Course (deg)
 * @param error_m  Estimated Position Error (m)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_beacon_detection_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t beacon_id,uint16_t position_mask,float lat_deg,float lon_deg,float speed_mps,float course_deg,float error_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN];
    _mav_put_float(buf, 0, lat_deg);
    _mav_put_float(buf, 4, lon_deg);
    _mav_put_float(buf, 8, speed_mps);
    _mav_put_float(buf, 12, course_deg);
    _mav_put_float(buf, 16, error_m);
    _mav_put_uint16_t(buf, 20, position_mask);
    _mav_put_uint8_t(buf, 22, beacon_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN);
#else
    mavlink_magothy_beacon_detection_t packet;
    packet.lat_deg = lat_deg;
    packet.lon_deg = lon_deg;
    packet.speed_mps = speed_mps;
    packet.course_deg = course_deg;
    packet.error_m = error_m;
    packet.position_mask = position_mask;
    packet.beacon_id = beacon_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_CRC);
}

/**
 * @brief Encode a magothy_beacon_detection struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_beacon_detection C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_beacon_detection_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_beacon_detection_t* magothy_beacon_detection)
{
    return mavlink_msg_magothy_beacon_detection_pack(system_id, component_id, msg, magothy_beacon_detection->beacon_id, magothy_beacon_detection->position_mask, magothy_beacon_detection->lat_deg, magothy_beacon_detection->lon_deg, magothy_beacon_detection->speed_mps, magothy_beacon_detection->course_deg, magothy_beacon_detection->error_m);
}

/**
 * @brief Encode a magothy_beacon_detection struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_beacon_detection C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_beacon_detection_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_beacon_detection_t* magothy_beacon_detection)
{
    return mavlink_msg_magothy_beacon_detection_pack_chan(system_id, component_id, chan, msg, magothy_beacon_detection->beacon_id, magothy_beacon_detection->position_mask, magothy_beacon_detection->lat_deg, magothy_beacon_detection->lon_deg, magothy_beacon_detection->speed_mps, magothy_beacon_detection->course_deg, magothy_beacon_detection->error_m);
}

/**
 * @brief Send a magothy_beacon_detection message
 * @param chan MAVLink channel to send the message
 *
 * @param beacon_id  beacon id of detection
 * @param position_mask  12-bit bitmask, 30 deg increments, bit 0 == 0deg, bit 1 == 30deg, ...
 * @param lat_deg  Latitude (deg)
 * @param lon_deg  Longitude (deg)
 * @param speed_mps  Speed (m/s)
 * @param course_deg  Course (deg)
 * @param error_m  Estimated Position Error (m)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_beacon_detection_send(mavlink_channel_t chan, uint8_t beacon_id, uint16_t position_mask, float lat_deg, float lon_deg, float speed_mps, float course_deg, float error_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN];
    _mav_put_float(buf, 0, lat_deg);
    _mav_put_float(buf, 4, lon_deg);
    _mav_put_float(buf, 8, speed_mps);
    _mav_put_float(buf, 12, course_deg);
    _mav_put_float(buf, 16, error_m);
    _mav_put_uint16_t(buf, 20, position_mask);
    _mav_put_uint8_t(buf, 22, beacon_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION, buf, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_CRC);
#else
    mavlink_magothy_beacon_detection_t packet;
    packet.lat_deg = lat_deg;
    packet.lon_deg = lon_deg;
    packet.speed_mps = speed_mps;
    packet.course_deg = course_deg;
    packet.error_m = error_m;
    packet.position_mask = position_mask;
    packet.beacon_id = beacon_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_CRC);
#endif
}

/**
 * @brief Send a magothy_beacon_detection message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_beacon_detection_send_struct(mavlink_channel_t chan, const mavlink_magothy_beacon_detection_t* magothy_beacon_detection)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_beacon_detection_send(chan, magothy_beacon_detection->beacon_id, magothy_beacon_detection->position_mask, magothy_beacon_detection->lat_deg, magothy_beacon_detection->lon_deg, magothy_beacon_detection->speed_mps, magothy_beacon_detection->course_deg, magothy_beacon_detection->error_m);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION, (const char *)magothy_beacon_detection, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_beacon_detection_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t beacon_id, uint16_t position_mask, float lat_deg, float lon_deg, float speed_mps, float course_deg, float error_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, lat_deg);
    _mav_put_float(buf, 4, lon_deg);
    _mav_put_float(buf, 8, speed_mps);
    _mav_put_float(buf, 12, course_deg);
    _mav_put_float(buf, 16, error_m);
    _mav_put_uint16_t(buf, 20, position_mask);
    _mav_put_uint8_t(buf, 22, beacon_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION, buf, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_CRC);
#else
    mavlink_magothy_beacon_detection_t *packet = (mavlink_magothy_beacon_detection_t *)msgbuf;
    packet->lat_deg = lat_deg;
    packet->lon_deg = lon_deg;
    packet->speed_mps = speed_mps;
    packet->course_deg = course_deg;
    packet->error_m = error_m;
    packet->position_mask = position_mask;
    packet->beacon_id = beacon_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_BEACON_DETECTION UNPACKING


/**
 * @brief Get field beacon_id from magothy_beacon_detection message
 *
 * @return  beacon id of detection
 */
static inline uint8_t mavlink_msg_magothy_beacon_detection_get_beacon_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field position_mask from magothy_beacon_detection message
 *
 * @return  12-bit bitmask, 30 deg increments, bit 0 == 0deg, bit 1 == 30deg, ...
 */
static inline uint16_t mavlink_msg_magothy_beacon_detection_get_position_mask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field lat_deg from magothy_beacon_detection message
 *
 * @return  Latitude (deg)
 */
static inline float mavlink_msg_magothy_beacon_detection_get_lat_deg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field lon_deg from magothy_beacon_detection message
 *
 * @return  Longitude (deg)
 */
static inline float mavlink_msg_magothy_beacon_detection_get_lon_deg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field speed_mps from magothy_beacon_detection message
 *
 * @return  Speed (m/s)
 */
static inline float mavlink_msg_magothy_beacon_detection_get_speed_mps(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field course_deg from magothy_beacon_detection message
 *
 * @return  Course (deg)
 */
static inline float mavlink_msg_magothy_beacon_detection_get_course_deg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field error_m from magothy_beacon_detection message
 *
 * @return  Estimated Position Error (m)
 */
static inline float mavlink_msg_magothy_beacon_detection_get_error_m(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a magothy_beacon_detection message into a struct
 *
 * @param msg The message to decode
 * @param magothy_beacon_detection C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_beacon_detection_decode(const mavlink_message_t* msg, mavlink_magothy_beacon_detection_t* magothy_beacon_detection)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_beacon_detection->lat_deg = mavlink_msg_magothy_beacon_detection_get_lat_deg(msg);
    magothy_beacon_detection->lon_deg = mavlink_msg_magothy_beacon_detection_get_lon_deg(msg);
    magothy_beacon_detection->speed_mps = mavlink_msg_magothy_beacon_detection_get_speed_mps(msg);
    magothy_beacon_detection->course_deg = mavlink_msg_magothy_beacon_detection_get_course_deg(msg);
    magothy_beacon_detection->error_m = mavlink_msg_magothy_beacon_detection_get_error_m(msg);
    magothy_beacon_detection->position_mask = mavlink_msg_magothy_beacon_detection_get_position_mask(msg);
    magothy_beacon_detection->beacon_id = mavlink_msg_magothy_beacon_detection_get_beacon_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN;
        memset(magothy_beacon_detection, 0, MAVLINK_MSG_ID_MAGOTHY_BEACON_DETECTION_LEN);
    memcpy(magothy_beacon_detection, _MAV_PAYLOAD(msg), len);
#endif
}
