#pragma once
// MESSAGE MAGOTHY_TELEOP_SAFETY PACKING

#define MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY 50007

MAVPACKED(
typedef struct __mavlink_magothy_teleop_safety_t {
 float distance_to_obstacle_m; /*< [m] Distance to nearest obstacle (m), NaN if no obstacle detected*/
 uint8_t is_enabled; /*<  1 if teleop safety is enabled, 0 if disabled*/
 uint8_t is_safe; /*<  1 is teleop is currently safe, 0 if unsafe*/
 uint8_t is_safe_latched; /*<  1 if teleop safety is latched unsafe, 0 if not latched*/
 float min_safe_distance_m; /*< [m] Minimum distance to maintain from obstacles (m)*/
 float field_of_view_deg; /*< [deg] Field of view (deg)*/
 float bearing_to_obstacle_deg; /*< [deg] Bearing to nearest obstacle (deg) clockwise from north, NaN if no obstacle detected*/
}) mavlink_magothy_teleop_safety_t;

#define MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN 19
#define MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_MIN_LEN 7
#define MAVLINK_MSG_ID_50007_LEN 19
#define MAVLINK_MSG_ID_50007_MIN_LEN 7

#define MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_CRC 155
#define MAVLINK_MSG_ID_50007_CRC 155



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_TELEOP_SAFETY { \
    50007, \
    "MAGOTHY_TELEOP_SAFETY", \
    7, \
    {  { "is_enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_magothy_teleop_safety_t, is_enabled) }, \
         { "is_safe", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_magothy_teleop_safety_t, is_safe) }, \
         { "is_safe_latched", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_magothy_teleop_safety_t, is_safe_latched) }, \
         { "distance_to_obstacle_m", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_magothy_teleop_safety_t, distance_to_obstacle_m) }, \
         { "min_safe_distance_m", NULL, MAVLINK_TYPE_FLOAT, 0, 7, offsetof(mavlink_magothy_teleop_safety_t, min_safe_distance_m) }, \
         { "field_of_view_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 11, offsetof(mavlink_magothy_teleop_safety_t, field_of_view_deg) }, \
         { "bearing_to_obstacle_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 15, offsetof(mavlink_magothy_teleop_safety_t, bearing_to_obstacle_deg) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_TELEOP_SAFETY { \
    "MAGOTHY_TELEOP_SAFETY", \
    7, \
    {  { "is_enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_magothy_teleop_safety_t, is_enabled) }, \
         { "is_safe", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_magothy_teleop_safety_t, is_safe) }, \
         { "is_safe_latched", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_magothy_teleop_safety_t, is_safe_latched) }, \
         { "distance_to_obstacle_m", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_magothy_teleop_safety_t, distance_to_obstacle_m) }, \
         { "min_safe_distance_m", NULL, MAVLINK_TYPE_FLOAT, 0, 7, offsetof(mavlink_magothy_teleop_safety_t, min_safe_distance_m) }, \
         { "field_of_view_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 11, offsetof(mavlink_magothy_teleop_safety_t, field_of_view_deg) }, \
         { "bearing_to_obstacle_deg", NULL, MAVLINK_TYPE_FLOAT, 0, 15, offsetof(mavlink_magothy_teleop_safety_t, bearing_to_obstacle_deg) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_teleop_safety message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param is_enabled  1 if teleop safety is enabled, 0 if disabled
 * @param is_safe  1 is teleop is currently safe, 0 if unsafe
 * @param is_safe_latched  1 if teleop safety is latched unsafe, 0 if not latched
 * @param distance_to_obstacle_m [m] Distance to nearest obstacle (m), NaN if no obstacle detected
 * @param min_safe_distance_m [m] Minimum distance to maintain from obstacles (m)
 * @param field_of_view_deg [deg] Field of view (deg)
 * @param bearing_to_obstacle_deg [deg] Bearing to nearest obstacle (deg) clockwise from north, NaN if no obstacle detected
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_teleop_safety_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t is_enabled, uint8_t is_safe, uint8_t is_safe_latched, float distance_to_obstacle_m, float min_safe_distance_m, float field_of_view_deg, float bearing_to_obstacle_deg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN];
    _mav_put_float(buf, 0, distance_to_obstacle_m);
    _mav_put_uint8_t(buf, 4, is_enabled);
    _mav_put_uint8_t(buf, 5, is_safe);
    _mav_put_uint8_t(buf, 6, is_safe_latched);
    _mav_put_float(buf, 7, min_safe_distance_m);
    _mav_put_float(buf, 11, field_of_view_deg);
    _mav_put_float(buf, 15, bearing_to_obstacle_deg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN);
#else
    mavlink_magothy_teleop_safety_t packet;
    packet.distance_to_obstacle_m = distance_to_obstacle_m;
    packet.is_enabled = is_enabled;
    packet.is_safe = is_safe;
    packet.is_safe_latched = is_safe_latched;
    packet.min_safe_distance_m = min_safe_distance_m;
    packet.field_of_view_deg = field_of_view_deg;
    packet.bearing_to_obstacle_deg = bearing_to_obstacle_deg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_CRC);
}

/**
 * @brief Pack a magothy_teleop_safety message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param is_enabled  1 if teleop safety is enabled, 0 if disabled
 * @param is_safe  1 is teleop is currently safe, 0 if unsafe
 * @param is_safe_latched  1 if teleop safety is latched unsafe, 0 if not latched
 * @param distance_to_obstacle_m [m] Distance to nearest obstacle (m), NaN if no obstacle detected
 * @param min_safe_distance_m [m] Minimum distance to maintain from obstacles (m)
 * @param field_of_view_deg [deg] Field of view (deg)
 * @param bearing_to_obstacle_deg [deg] Bearing to nearest obstacle (deg) clockwise from north, NaN if no obstacle detected
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_teleop_safety_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t is_enabled,uint8_t is_safe,uint8_t is_safe_latched,float distance_to_obstacle_m,float min_safe_distance_m,float field_of_view_deg,float bearing_to_obstacle_deg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN];
    _mav_put_float(buf, 0, distance_to_obstacle_m);
    _mav_put_uint8_t(buf, 4, is_enabled);
    _mav_put_uint8_t(buf, 5, is_safe);
    _mav_put_uint8_t(buf, 6, is_safe_latched);
    _mav_put_float(buf, 7, min_safe_distance_m);
    _mav_put_float(buf, 11, field_of_view_deg);
    _mav_put_float(buf, 15, bearing_to_obstacle_deg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN);
#else
    mavlink_magothy_teleop_safety_t packet;
    packet.distance_to_obstacle_m = distance_to_obstacle_m;
    packet.is_enabled = is_enabled;
    packet.is_safe = is_safe;
    packet.is_safe_latched = is_safe_latched;
    packet.min_safe_distance_m = min_safe_distance_m;
    packet.field_of_view_deg = field_of_view_deg;
    packet.bearing_to_obstacle_deg = bearing_to_obstacle_deg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_CRC);
}

/**
 * @brief Encode a magothy_teleop_safety struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_teleop_safety C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_teleop_safety_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_teleop_safety_t* magothy_teleop_safety)
{
    return mavlink_msg_magothy_teleop_safety_pack(system_id, component_id, msg, magothy_teleop_safety->is_enabled, magothy_teleop_safety->is_safe, magothy_teleop_safety->is_safe_latched, magothy_teleop_safety->distance_to_obstacle_m, magothy_teleop_safety->min_safe_distance_m, magothy_teleop_safety->field_of_view_deg, magothy_teleop_safety->bearing_to_obstacle_deg);
}

/**
 * @brief Encode a magothy_teleop_safety struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_teleop_safety C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_teleop_safety_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_teleop_safety_t* magothy_teleop_safety)
{
    return mavlink_msg_magothy_teleop_safety_pack_chan(system_id, component_id, chan, msg, magothy_teleop_safety->is_enabled, magothy_teleop_safety->is_safe, magothy_teleop_safety->is_safe_latched, magothy_teleop_safety->distance_to_obstacle_m, magothy_teleop_safety->min_safe_distance_m, magothy_teleop_safety->field_of_view_deg, magothy_teleop_safety->bearing_to_obstacle_deg);
}

/**
 * @brief Send a magothy_teleop_safety message
 * @param chan MAVLink channel to send the message
 *
 * @param is_enabled  1 if teleop safety is enabled, 0 if disabled
 * @param is_safe  1 is teleop is currently safe, 0 if unsafe
 * @param is_safe_latched  1 if teleop safety is latched unsafe, 0 if not latched
 * @param distance_to_obstacle_m [m] Distance to nearest obstacle (m), NaN if no obstacle detected
 * @param min_safe_distance_m [m] Minimum distance to maintain from obstacles (m)
 * @param field_of_view_deg [deg] Field of view (deg)
 * @param bearing_to_obstacle_deg [deg] Bearing to nearest obstacle (deg) clockwise from north, NaN if no obstacle detected
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_teleop_safety_send(mavlink_channel_t chan, uint8_t is_enabled, uint8_t is_safe, uint8_t is_safe_latched, float distance_to_obstacle_m, float min_safe_distance_m, float field_of_view_deg, float bearing_to_obstacle_deg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN];
    _mav_put_float(buf, 0, distance_to_obstacle_m);
    _mav_put_uint8_t(buf, 4, is_enabled);
    _mav_put_uint8_t(buf, 5, is_safe);
    _mav_put_uint8_t(buf, 6, is_safe_latched);
    _mav_put_float(buf, 7, min_safe_distance_m);
    _mav_put_float(buf, 11, field_of_view_deg);
    _mav_put_float(buf, 15, bearing_to_obstacle_deg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY, buf, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_CRC);
#else
    mavlink_magothy_teleop_safety_t packet;
    packet.distance_to_obstacle_m = distance_to_obstacle_m;
    packet.is_enabled = is_enabled;
    packet.is_safe = is_safe;
    packet.is_safe_latched = is_safe_latched;
    packet.min_safe_distance_m = min_safe_distance_m;
    packet.field_of_view_deg = field_of_view_deg;
    packet.bearing_to_obstacle_deg = bearing_to_obstacle_deg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_CRC);
#endif
}

/**
 * @brief Send a magothy_teleop_safety message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_teleop_safety_send_struct(mavlink_channel_t chan, const mavlink_magothy_teleop_safety_t* magothy_teleop_safety)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_teleop_safety_send(chan, magothy_teleop_safety->is_enabled, magothy_teleop_safety->is_safe, magothy_teleop_safety->is_safe_latched, magothy_teleop_safety->distance_to_obstacle_m, magothy_teleop_safety->min_safe_distance_m, magothy_teleop_safety->field_of_view_deg, magothy_teleop_safety->bearing_to_obstacle_deg);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY, (const char *)magothy_teleop_safety, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_teleop_safety_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t is_enabled, uint8_t is_safe, uint8_t is_safe_latched, float distance_to_obstacle_m, float min_safe_distance_m, float field_of_view_deg, float bearing_to_obstacle_deg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, distance_to_obstacle_m);
    _mav_put_uint8_t(buf, 4, is_enabled);
    _mav_put_uint8_t(buf, 5, is_safe);
    _mav_put_uint8_t(buf, 6, is_safe_latched);
    _mav_put_float(buf, 7, min_safe_distance_m);
    _mav_put_float(buf, 11, field_of_view_deg);
    _mav_put_float(buf, 15, bearing_to_obstacle_deg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY, buf, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_CRC);
#else
    mavlink_magothy_teleop_safety_t *packet = (mavlink_magothy_teleop_safety_t *)msgbuf;
    packet->distance_to_obstacle_m = distance_to_obstacle_m;
    packet->is_enabled = is_enabled;
    packet->is_safe = is_safe;
    packet->is_safe_latched = is_safe_latched;
    packet->min_safe_distance_m = min_safe_distance_m;
    packet->field_of_view_deg = field_of_view_deg;
    packet->bearing_to_obstacle_deg = bearing_to_obstacle_deg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_TELEOP_SAFETY UNPACKING


/**
 * @brief Get field is_enabled from magothy_teleop_safety message
 *
 * @return  1 if teleop safety is enabled, 0 if disabled
 */
static inline uint8_t mavlink_msg_magothy_teleop_safety_get_is_enabled(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field is_safe from magothy_teleop_safety message
 *
 * @return  1 is teleop is currently safe, 0 if unsafe
 */
static inline uint8_t mavlink_msg_magothy_teleop_safety_get_is_safe(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field is_safe_latched from magothy_teleop_safety message
 *
 * @return  1 if teleop safety is latched unsafe, 0 if not latched
 */
static inline uint8_t mavlink_msg_magothy_teleop_safety_get_is_safe_latched(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field distance_to_obstacle_m from magothy_teleop_safety message
 *
 * @return [m] Distance to nearest obstacle (m), NaN if no obstacle detected
 */
static inline float mavlink_msg_magothy_teleop_safety_get_distance_to_obstacle_m(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field min_safe_distance_m from magothy_teleop_safety message
 *
 * @return [m] Minimum distance to maintain from obstacles (m)
 */
static inline float mavlink_msg_magothy_teleop_safety_get_min_safe_distance_m(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  7);
}

/**
 * @brief Get field field_of_view_deg from magothy_teleop_safety message
 *
 * @return [deg] Field of view (deg)
 */
static inline float mavlink_msg_magothy_teleop_safety_get_field_of_view_deg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  11);
}

/**
 * @brief Get field bearing_to_obstacle_deg from magothy_teleop_safety message
 *
 * @return [deg] Bearing to nearest obstacle (deg) clockwise from north, NaN if no obstacle detected
 */
static inline float mavlink_msg_magothy_teleop_safety_get_bearing_to_obstacle_deg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  15);
}

/**
 * @brief Decode a magothy_teleop_safety message into a struct
 *
 * @param msg The message to decode
 * @param magothy_teleop_safety C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_teleop_safety_decode(const mavlink_message_t* msg, mavlink_magothy_teleop_safety_t* magothy_teleop_safety)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_teleop_safety->distance_to_obstacle_m = mavlink_msg_magothy_teleop_safety_get_distance_to_obstacle_m(msg);
    magothy_teleop_safety->is_enabled = mavlink_msg_magothy_teleop_safety_get_is_enabled(msg);
    magothy_teleop_safety->is_safe = mavlink_msg_magothy_teleop_safety_get_is_safe(msg);
    magothy_teleop_safety->is_safe_latched = mavlink_msg_magothy_teleop_safety_get_is_safe_latched(msg);
    magothy_teleop_safety->min_safe_distance_m = mavlink_msg_magothy_teleop_safety_get_min_safe_distance_m(msg);
    magothy_teleop_safety->field_of_view_deg = mavlink_msg_magothy_teleop_safety_get_field_of_view_deg(msg);
    magothy_teleop_safety->bearing_to_obstacle_deg = mavlink_msg_magothy_teleop_safety_get_bearing_to_obstacle_deg(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN;
        memset(magothy_teleop_safety, 0, MAVLINK_MSG_ID_MAGOTHY_TELEOP_SAFETY_LEN);
    memcpy(magothy_teleop_safety, _MAV_PAYLOAD(msg), len);
#endif
}
