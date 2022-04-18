#pragma once
// MESSAGE MAGOTHY_MISSION_TELEMETRY PACKING

#define MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY 50002


typedef struct __mavlink_magothy_mission_telemetry_t {
 uint64_t unix_time_usec; /*< [us] Unix Timestamp*/
 uint32_t uptime_msec; /*< [ms] Vehicle Controller uptime*/
 float mission_item_time_elapsed_s; /*< [s] Elasped time of currently executing mission item (s) - NaN if not in mission*/
 float mission_item_time_remaining_s; /*< [s] Remaining time of currently executing mission item (s) - NaN if not in mission*/
 float mission_time_elapsed_s; /*< [s] Elasped time of entire mission (s) - NaN if not in mission*/
 float mission_time_remaining_s; /*< [s] Remaining time of entire mission (s) - NaN if not in mission*/
 float distance_to_target_m; /*< [m] Distance to target (waypoint/loiter/etc) (m) - NaN if not in mission or not geo based*/
 float cross_track_error_m; /*< [m] Trackline offtrack error (m) - NaN if not in mission or not in trackline*/
 char gcs_set_mode_uuid[16]; /*<  UUID of most recent mode change*/
} mavlink_magothy_mission_telemetry_t;

#define MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN 52
#define MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_MIN_LEN 52
#define MAVLINK_MSG_ID_50002_LEN 52
#define MAVLINK_MSG_ID_50002_MIN_LEN 52

#define MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_CRC 189
#define MAVLINK_MSG_ID_50002_CRC 189

#define MAVLINK_MSG_MAGOTHY_MISSION_TELEMETRY_FIELD_GCS_SET_MODE_UUID_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_MISSION_TELEMETRY { \
    50002, \
    "MAGOTHY_MISSION_TELEMETRY", \
    9, \
    {  { "unix_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_magothy_mission_telemetry_t, unix_time_usec) }, \
         { "uptime_msec", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_magothy_mission_telemetry_t, uptime_msec) }, \
         { "mission_item_time_elapsed_s", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_magothy_mission_telemetry_t, mission_item_time_elapsed_s) }, \
         { "mission_item_time_remaining_s", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_magothy_mission_telemetry_t, mission_item_time_remaining_s) }, \
         { "mission_time_elapsed_s", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_magothy_mission_telemetry_t, mission_time_elapsed_s) }, \
         { "mission_time_remaining_s", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_magothy_mission_telemetry_t, mission_time_remaining_s) }, \
         { "distance_to_target_m", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_magothy_mission_telemetry_t, distance_to_target_m) }, \
         { "cross_track_error_m", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_magothy_mission_telemetry_t, cross_track_error_m) }, \
         { "gcs_set_mode_uuid", NULL, MAVLINK_TYPE_CHAR, 16, 36, offsetof(mavlink_magothy_mission_telemetry_t, gcs_set_mode_uuid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_MISSION_TELEMETRY { \
    "MAGOTHY_MISSION_TELEMETRY", \
    9, \
    {  { "unix_time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_magothy_mission_telemetry_t, unix_time_usec) }, \
         { "uptime_msec", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_magothy_mission_telemetry_t, uptime_msec) }, \
         { "mission_item_time_elapsed_s", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_magothy_mission_telemetry_t, mission_item_time_elapsed_s) }, \
         { "mission_item_time_remaining_s", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_magothy_mission_telemetry_t, mission_item_time_remaining_s) }, \
         { "mission_time_elapsed_s", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_magothy_mission_telemetry_t, mission_time_elapsed_s) }, \
         { "mission_time_remaining_s", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_magothy_mission_telemetry_t, mission_time_remaining_s) }, \
         { "distance_to_target_m", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_magothy_mission_telemetry_t, distance_to_target_m) }, \
         { "cross_track_error_m", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_magothy_mission_telemetry_t, cross_track_error_m) }, \
         { "gcs_set_mode_uuid", NULL, MAVLINK_TYPE_CHAR, 16, 36, offsetof(mavlink_magothy_mission_telemetry_t, gcs_set_mode_uuid) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_mission_telemetry message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param unix_time_usec [us] Unix Timestamp
 * @param uptime_msec [ms] Vehicle Controller uptime
 * @param mission_item_time_elapsed_s [s] Elasped time of currently executing mission item (s) - NaN if not in mission
 * @param mission_item_time_remaining_s [s] Remaining time of currently executing mission item (s) - NaN if not in mission
 * @param mission_time_elapsed_s [s] Elasped time of entire mission (s) - NaN if not in mission
 * @param mission_time_remaining_s [s] Remaining time of entire mission (s) - NaN if not in mission
 * @param distance_to_target_m [m] Distance to target (waypoint/loiter/etc) (m) - NaN if not in mission or not geo based
 * @param cross_track_error_m [m] Trackline offtrack error (m) - NaN if not in mission or not in trackline
 * @param gcs_set_mode_uuid  UUID of most recent mode change
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_mission_telemetry_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t unix_time_usec, uint32_t uptime_msec, float mission_item_time_elapsed_s, float mission_item_time_remaining_s, float mission_time_elapsed_s, float mission_time_remaining_s, float distance_to_target_m, float cross_track_error_m, const char *gcs_set_mode_uuid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN];
    _mav_put_uint64_t(buf, 0, unix_time_usec);
    _mav_put_uint32_t(buf, 8, uptime_msec);
    _mav_put_float(buf, 12, mission_item_time_elapsed_s);
    _mav_put_float(buf, 16, mission_item_time_remaining_s);
    _mav_put_float(buf, 20, mission_time_elapsed_s);
    _mav_put_float(buf, 24, mission_time_remaining_s);
    _mav_put_float(buf, 28, distance_to_target_m);
    _mav_put_float(buf, 32, cross_track_error_m);
    _mav_put_char_array(buf, 36, gcs_set_mode_uuid, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN);
#else
    mavlink_magothy_mission_telemetry_t packet;
    packet.unix_time_usec = unix_time_usec;
    packet.uptime_msec = uptime_msec;
    packet.mission_item_time_elapsed_s = mission_item_time_elapsed_s;
    packet.mission_item_time_remaining_s = mission_item_time_remaining_s;
    packet.mission_time_elapsed_s = mission_time_elapsed_s;
    packet.mission_time_remaining_s = mission_time_remaining_s;
    packet.distance_to_target_m = distance_to_target_m;
    packet.cross_track_error_m = cross_track_error_m;
    mav_array_memcpy(packet.gcs_set_mode_uuid, gcs_set_mode_uuid, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_CRC);
}

/**
 * @brief Pack a magothy_mission_telemetry message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param unix_time_usec [us] Unix Timestamp
 * @param uptime_msec [ms] Vehicle Controller uptime
 * @param mission_item_time_elapsed_s [s] Elasped time of currently executing mission item (s) - NaN if not in mission
 * @param mission_item_time_remaining_s [s] Remaining time of currently executing mission item (s) - NaN if not in mission
 * @param mission_time_elapsed_s [s] Elasped time of entire mission (s) - NaN if not in mission
 * @param mission_time_remaining_s [s] Remaining time of entire mission (s) - NaN if not in mission
 * @param distance_to_target_m [m] Distance to target (waypoint/loiter/etc) (m) - NaN if not in mission or not geo based
 * @param cross_track_error_m [m] Trackline offtrack error (m) - NaN if not in mission or not in trackline
 * @param gcs_set_mode_uuid  UUID of most recent mode change
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_mission_telemetry_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t unix_time_usec,uint32_t uptime_msec,float mission_item_time_elapsed_s,float mission_item_time_remaining_s,float mission_time_elapsed_s,float mission_time_remaining_s,float distance_to_target_m,float cross_track_error_m,const char *gcs_set_mode_uuid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN];
    _mav_put_uint64_t(buf, 0, unix_time_usec);
    _mav_put_uint32_t(buf, 8, uptime_msec);
    _mav_put_float(buf, 12, mission_item_time_elapsed_s);
    _mav_put_float(buf, 16, mission_item_time_remaining_s);
    _mav_put_float(buf, 20, mission_time_elapsed_s);
    _mav_put_float(buf, 24, mission_time_remaining_s);
    _mav_put_float(buf, 28, distance_to_target_m);
    _mav_put_float(buf, 32, cross_track_error_m);
    _mav_put_char_array(buf, 36, gcs_set_mode_uuid, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN);
#else
    mavlink_magothy_mission_telemetry_t packet;
    packet.unix_time_usec = unix_time_usec;
    packet.uptime_msec = uptime_msec;
    packet.mission_item_time_elapsed_s = mission_item_time_elapsed_s;
    packet.mission_item_time_remaining_s = mission_item_time_remaining_s;
    packet.mission_time_elapsed_s = mission_time_elapsed_s;
    packet.mission_time_remaining_s = mission_time_remaining_s;
    packet.distance_to_target_m = distance_to_target_m;
    packet.cross_track_error_m = cross_track_error_m;
    mav_array_memcpy(packet.gcs_set_mode_uuid, gcs_set_mode_uuid, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_CRC);
}

/**
 * @brief Encode a magothy_mission_telemetry struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_mission_telemetry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_mission_telemetry_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_mission_telemetry_t* magothy_mission_telemetry)
{
    return mavlink_msg_magothy_mission_telemetry_pack(system_id, component_id, msg, magothy_mission_telemetry->unix_time_usec, magothy_mission_telemetry->uptime_msec, magothy_mission_telemetry->mission_item_time_elapsed_s, magothy_mission_telemetry->mission_item_time_remaining_s, magothy_mission_telemetry->mission_time_elapsed_s, magothy_mission_telemetry->mission_time_remaining_s, magothy_mission_telemetry->distance_to_target_m, magothy_mission_telemetry->cross_track_error_m, magothy_mission_telemetry->gcs_set_mode_uuid);
}

/**
 * @brief Encode a magothy_mission_telemetry struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_mission_telemetry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_mission_telemetry_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_mission_telemetry_t* magothy_mission_telemetry)
{
    return mavlink_msg_magothy_mission_telemetry_pack_chan(system_id, component_id, chan, msg, magothy_mission_telemetry->unix_time_usec, magothy_mission_telemetry->uptime_msec, magothy_mission_telemetry->mission_item_time_elapsed_s, magothy_mission_telemetry->mission_item_time_remaining_s, magothy_mission_telemetry->mission_time_elapsed_s, magothy_mission_telemetry->mission_time_remaining_s, magothy_mission_telemetry->distance_to_target_m, magothy_mission_telemetry->cross_track_error_m, magothy_mission_telemetry->gcs_set_mode_uuid);
}

/**
 * @brief Send a magothy_mission_telemetry message
 * @param chan MAVLink channel to send the message
 *
 * @param unix_time_usec [us] Unix Timestamp
 * @param uptime_msec [ms] Vehicle Controller uptime
 * @param mission_item_time_elapsed_s [s] Elasped time of currently executing mission item (s) - NaN if not in mission
 * @param mission_item_time_remaining_s [s] Remaining time of currently executing mission item (s) - NaN if not in mission
 * @param mission_time_elapsed_s [s] Elasped time of entire mission (s) - NaN if not in mission
 * @param mission_time_remaining_s [s] Remaining time of entire mission (s) - NaN if not in mission
 * @param distance_to_target_m [m] Distance to target (waypoint/loiter/etc) (m) - NaN if not in mission or not geo based
 * @param cross_track_error_m [m] Trackline offtrack error (m) - NaN if not in mission or not in trackline
 * @param gcs_set_mode_uuid  UUID of most recent mode change
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_mission_telemetry_send(mavlink_channel_t chan, uint64_t unix_time_usec, uint32_t uptime_msec, float mission_item_time_elapsed_s, float mission_item_time_remaining_s, float mission_time_elapsed_s, float mission_time_remaining_s, float distance_to_target_m, float cross_track_error_m, const char *gcs_set_mode_uuid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN];
    _mav_put_uint64_t(buf, 0, unix_time_usec);
    _mav_put_uint32_t(buf, 8, uptime_msec);
    _mav_put_float(buf, 12, mission_item_time_elapsed_s);
    _mav_put_float(buf, 16, mission_item_time_remaining_s);
    _mav_put_float(buf, 20, mission_time_elapsed_s);
    _mav_put_float(buf, 24, mission_time_remaining_s);
    _mav_put_float(buf, 28, distance_to_target_m);
    _mav_put_float(buf, 32, cross_track_error_m);
    _mav_put_char_array(buf, 36, gcs_set_mode_uuid, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY, buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_CRC);
#else
    mavlink_magothy_mission_telemetry_t packet;
    packet.unix_time_usec = unix_time_usec;
    packet.uptime_msec = uptime_msec;
    packet.mission_item_time_elapsed_s = mission_item_time_elapsed_s;
    packet.mission_item_time_remaining_s = mission_item_time_remaining_s;
    packet.mission_time_elapsed_s = mission_time_elapsed_s;
    packet.mission_time_remaining_s = mission_time_remaining_s;
    packet.distance_to_target_m = distance_to_target_m;
    packet.cross_track_error_m = cross_track_error_m;
    mav_array_memcpy(packet.gcs_set_mode_uuid, gcs_set_mode_uuid, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_CRC);
#endif
}

/**
 * @brief Send a magothy_mission_telemetry message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_mission_telemetry_send_struct(mavlink_channel_t chan, const mavlink_magothy_mission_telemetry_t* magothy_mission_telemetry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_mission_telemetry_send(chan, magothy_mission_telemetry->unix_time_usec, magothy_mission_telemetry->uptime_msec, magothy_mission_telemetry->mission_item_time_elapsed_s, magothy_mission_telemetry->mission_item_time_remaining_s, magothy_mission_telemetry->mission_time_elapsed_s, magothy_mission_telemetry->mission_time_remaining_s, magothy_mission_telemetry->distance_to_target_m, magothy_mission_telemetry->cross_track_error_m, magothy_mission_telemetry->gcs_set_mode_uuid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY, (const char *)magothy_mission_telemetry, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_mission_telemetry_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t unix_time_usec, uint32_t uptime_msec, float mission_item_time_elapsed_s, float mission_item_time_remaining_s, float mission_time_elapsed_s, float mission_time_remaining_s, float distance_to_target_m, float cross_track_error_m, const char *gcs_set_mode_uuid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, unix_time_usec);
    _mav_put_uint32_t(buf, 8, uptime_msec);
    _mav_put_float(buf, 12, mission_item_time_elapsed_s);
    _mav_put_float(buf, 16, mission_item_time_remaining_s);
    _mav_put_float(buf, 20, mission_time_elapsed_s);
    _mav_put_float(buf, 24, mission_time_remaining_s);
    _mav_put_float(buf, 28, distance_to_target_m);
    _mav_put_float(buf, 32, cross_track_error_m);
    _mav_put_char_array(buf, 36, gcs_set_mode_uuid, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY, buf, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_CRC);
#else
    mavlink_magothy_mission_telemetry_t *packet = (mavlink_magothy_mission_telemetry_t *)msgbuf;
    packet->unix_time_usec = unix_time_usec;
    packet->uptime_msec = uptime_msec;
    packet->mission_item_time_elapsed_s = mission_item_time_elapsed_s;
    packet->mission_item_time_remaining_s = mission_item_time_remaining_s;
    packet->mission_time_elapsed_s = mission_time_elapsed_s;
    packet->mission_time_remaining_s = mission_time_remaining_s;
    packet->distance_to_target_m = distance_to_target_m;
    packet->cross_track_error_m = cross_track_error_m;
    mav_array_memcpy(packet->gcs_set_mode_uuid, gcs_set_mode_uuid, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_MISSION_TELEMETRY UNPACKING


/**
 * @brief Get field unix_time_usec from magothy_mission_telemetry message
 *
 * @return [us] Unix Timestamp
 */
static inline uint64_t mavlink_msg_magothy_mission_telemetry_get_unix_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field uptime_msec from magothy_mission_telemetry message
 *
 * @return [ms] Vehicle Controller uptime
 */
static inline uint32_t mavlink_msg_magothy_mission_telemetry_get_uptime_msec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field mission_item_time_elapsed_s from magothy_mission_telemetry message
 *
 * @return [s] Elasped time of currently executing mission item (s) - NaN if not in mission
 */
static inline float mavlink_msg_magothy_mission_telemetry_get_mission_item_time_elapsed_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field mission_item_time_remaining_s from magothy_mission_telemetry message
 *
 * @return [s] Remaining time of currently executing mission item (s) - NaN if not in mission
 */
static inline float mavlink_msg_magothy_mission_telemetry_get_mission_item_time_remaining_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field mission_time_elapsed_s from magothy_mission_telemetry message
 *
 * @return [s] Elasped time of entire mission (s) - NaN if not in mission
 */
static inline float mavlink_msg_magothy_mission_telemetry_get_mission_time_elapsed_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field mission_time_remaining_s from magothy_mission_telemetry message
 *
 * @return [s] Remaining time of entire mission (s) - NaN if not in mission
 */
static inline float mavlink_msg_magothy_mission_telemetry_get_mission_time_remaining_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field distance_to_target_m from magothy_mission_telemetry message
 *
 * @return [m] Distance to target (waypoint/loiter/etc) (m) - NaN if not in mission or not geo based
 */
static inline float mavlink_msg_magothy_mission_telemetry_get_distance_to_target_m(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field cross_track_error_m from magothy_mission_telemetry message
 *
 * @return [m] Trackline offtrack error (m) - NaN if not in mission or not in trackline
 */
static inline float mavlink_msg_magothy_mission_telemetry_get_cross_track_error_m(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field gcs_set_mode_uuid from magothy_mission_telemetry message
 *
 * @return  UUID of most recent mode change
 */
static inline uint16_t mavlink_msg_magothy_mission_telemetry_get_gcs_set_mode_uuid(const mavlink_message_t* msg, char *gcs_set_mode_uuid)
{
    return _MAV_RETURN_char_array(msg, gcs_set_mode_uuid, 16,  36);
}

/**
 * @brief Decode a magothy_mission_telemetry message into a struct
 *
 * @param msg The message to decode
 * @param magothy_mission_telemetry C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_mission_telemetry_decode(const mavlink_message_t* msg, mavlink_magothy_mission_telemetry_t* magothy_mission_telemetry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_mission_telemetry->unix_time_usec = mavlink_msg_magothy_mission_telemetry_get_unix_time_usec(msg);
    magothy_mission_telemetry->uptime_msec = mavlink_msg_magothy_mission_telemetry_get_uptime_msec(msg);
    magothy_mission_telemetry->mission_item_time_elapsed_s = mavlink_msg_magothy_mission_telemetry_get_mission_item_time_elapsed_s(msg);
    magothy_mission_telemetry->mission_item_time_remaining_s = mavlink_msg_magothy_mission_telemetry_get_mission_item_time_remaining_s(msg);
    magothy_mission_telemetry->mission_time_elapsed_s = mavlink_msg_magothy_mission_telemetry_get_mission_time_elapsed_s(msg);
    magothy_mission_telemetry->mission_time_remaining_s = mavlink_msg_magothy_mission_telemetry_get_mission_time_remaining_s(msg);
    magothy_mission_telemetry->distance_to_target_m = mavlink_msg_magothy_mission_telemetry_get_distance_to_target_m(msg);
    magothy_mission_telemetry->cross_track_error_m = mavlink_msg_magothy_mission_telemetry_get_cross_track_error_m(msg);
    mavlink_msg_magothy_mission_telemetry_get_gcs_set_mode_uuid(msg, magothy_mission_telemetry->gcs_set_mode_uuid);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN;
        memset(magothy_mission_telemetry, 0, MAVLINK_MSG_ID_MAGOTHY_MISSION_TELEMETRY_LEN);
    memcpy(magothy_mission_telemetry, _MAV_PAYLOAD(msg), len);
#endif
}
