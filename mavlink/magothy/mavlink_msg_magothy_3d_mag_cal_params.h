#pragma once
// MESSAGE MAGOTHY_3D_MAG_CAL_PARAMS PACKING

#define MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS 50101


typedef struct __mavlink_magothy_3d_mag_cal_params_t {
 float hard_iron[3]; /*<  Hard Iron Offsets*/
 float soft_iron[9]; /*<  Soft Iron Rotation Matrix*/
 float uncalibrated_norm_mean; /*<  Norm mean of raw mag data*/
 float uncalibrated_norm_std_dev; /*<  Norm standard deviation of raw mag data*/
 float uncalibrated_norm_max_error; /*<  Maximum norm error (difference from 1.0) of raw mag data*/
 float calibrated_norm_mean; /*<  Norm mean of calibrated mag data*/
 float calibrated_norm_std_dev; /*<  Norm standard deviation of calibrated mag data*/
 float calibrated_norm_max_error; /*<  Maximum norm error (difference from 1.0) of calibrated mag data*/
} mavlink_magothy_3d_mag_cal_params_t;

#define MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN 72
#define MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_MIN_LEN 72
#define MAVLINK_MSG_ID_50101_LEN 72
#define MAVLINK_MSG_ID_50101_MIN_LEN 72

#define MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_CRC 238
#define MAVLINK_MSG_ID_50101_CRC 238

#define MAVLINK_MSG_MAGOTHY_3D_MAG_CAL_PARAMS_FIELD_HARD_IRON_LEN 3
#define MAVLINK_MSG_MAGOTHY_3D_MAG_CAL_PARAMS_FIELD_SOFT_IRON_LEN 9

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_3D_MAG_CAL_PARAMS { \
    50101, \
    "MAGOTHY_3D_MAG_CAL_PARAMS", \
    8, \
    {  { "hard_iron", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_magothy_3d_mag_cal_params_t, hard_iron) }, \
         { "soft_iron", NULL, MAVLINK_TYPE_FLOAT, 9, 12, offsetof(mavlink_magothy_3d_mag_cal_params_t, soft_iron) }, \
         { "uncalibrated_norm_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_magothy_3d_mag_cal_params_t, uncalibrated_norm_mean) }, \
         { "uncalibrated_norm_std_dev", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_magothy_3d_mag_cal_params_t, uncalibrated_norm_std_dev) }, \
         { "uncalibrated_norm_max_error", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_magothy_3d_mag_cal_params_t, uncalibrated_norm_max_error) }, \
         { "calibrated_norm_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_magothy_3d_mag_cal_params_t, calibrated_norm_mean) }, \
         { "calibrated_norm_std_dev", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_magothy_3d_mag_cal_params_t, calibrated_norm_std_dev) }, \
         { "calibrated_norm_max_error", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_magothy_3d_mag_cal_params_t, calibrated_norm_max_error) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_3D_MAG_CAL_PARAMS { \
    "MAGOTHY_3D_MAG_CAL_PARAMS", \
    8, \
    {  { "hard_iron", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_magothy_3d_mag_cal_params_t, hard_iron) }, \
         { "soft_iron", NULL, MAVLINK_TYPE_FLOAT, 9, 12, offsetof(mavlink_magothy_3d_mag_cal_params_t, soft_iron) }, \
         { "uncalibrated_norm_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_magothy_3d_mag_cal_params_t, uncalibrated_norm_mean) }, \
         { "uncalibrated_norm_std_dev", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_magothy_3d_mag_cal_params_t, uncalibrated_norm_std_dev) }, \
         { "uncalibrated_norm_max_error", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_magothy_3d_mag_cal_params_t, uncalibrated_norm_max_error) }, \
         { "calibrated_norm_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_magothy_3d_mag_cal_params_t, calibrated_norm_mean) }, \
         { "calibrated_norm_std_dev", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_magothy_3d_mag_cal_params_t, calibrated_norm_std_dev) }, \
         { "calibrated_norm_max_error", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_magothy_3d_mag_cal_params_t, calibrated_norm_max_error) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_3d_mag_cal_params message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param hard_iron  Hard Iron Offsets
 * @param soft_iron  Soft Iron Rotation Matrix
 * @param uncalibrated_norm_mean  Norm mean of raw mag data
 * @param uncalibrated_norm_std_dev  Norm standard deviation of raw mag data
 * @param uncalibrated_norm_max_error  Maximum norm error (difference from 1.0) of raw mag data
 * @param calibrated_norm_mean  Norm mean of calibrated mag data
 * @param calibrated_norm_std_dev  Norm standard deviation of calibrated mag data
 * @param calibrated_norm_max_error  Maximum norm error (difference from 1.0) of calibrated mag data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_3d_mag_cal_params_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *hard_iron, const float *soft_iron, float uncalibrated_norm_mean, float uncalibrated_norm_std_dev, float uncalibrated_norm_max_error, float calibrated_norm_mean, float calibrated_norm_std_dev, float calibrated_norm_max_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN];
    _mav_put_float(buf, 48, uncalibrated_norm_mean);
    _mav_put_float(buf, 52, uncalibrated_norm_std_dev);
    _mav_put_float(buf, 56, uncalibrated_norm_max_error);
    _mav_put_float(buf, 60, calibrated_norm_mean);
    _mav_put_float(buf, 64, calibrated_norm_std_dev);
    _mav_put_float(buf, 68, calibrated_norm_max_error);
    _mav_put_float_array(buf, 0, hard_iron, 3);
    _mav_put_float_array(buf, 12, soft_iron, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN);
#else
    mavlink_magothy_3d_mag_cal_params_t packet;
    packet.uncalibrated_norm_mean = uncalibrated_norm_mean;
    packet.uncalibrated_norm_std_dev = uncalibrated_norm_std_dev;
    packet.uncalibrated_norm_max_error = uncalibrated_norm_max_error;
    packet.calibrated_norm_mean = calibrated_norm_mean;
    packet.calibrated_norm_std_dev = calibrated_norm_std_dev;
    packet.calibrated_norm_max_error = calibrated_norm_max_error;
    mav_array_memcpy(packet.hard_iron, hard_iron, sizeof(float)*3);
    mav_array_memcpy(packet.soft_iron, soft_iron, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_CRC);
}

/**
 * @brief Pack a magothy_3d_mag_cal_params message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hard_iron  Hard Iron Offsets
 * @param soft_iron  Soft Iron Rotation Matrix
 * @param uncalibrated_norm_mean  Norm mean of raw mag data
 * @param uncalibrated_norm_std_dev  Norm standard deviation of raw mag data
 * @param uncalibrated_norm_max_error  Maximum norm error (difference from 1.0) of raw mag data
 * @param calibrated_norm_mean  Norm mean of calibrated mag data
 * @param calibrated_norm_std_dev  Norm standard deviation of calibrated mag data
 * @param calibrated_norm_max_error  Maximum norm error (difference from 1.0) of calibrated mag data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_3d_mag_cal_params_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *hard_iron,const float *soft_iron,float uncalibrated_norm_mean,float uncalibrated_norm_std_dev,float uncalibrated_norm_max_error,float calibrated_norm_mean,float calibrated_norm_std_dev,float calibrated_norm_max_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN];
    _mav_put_float(buf, 48, uncalibrated_norm_mean);
    _mav_put_float(buf, 52, uncalibrated_norm_std_dev);
    _mav_put_float(buf, 56, uncalibrated_norm_max_error);
    _mav_put_float(buf, 60, calibrated_norm_mean);
    _mav_put_float(buf, 64, calibrated_norm_std_dev);
    _mav_put_float(buf, 68, calibrated_norm_max_error);
    _mav_put_float_array(buf, 0, hard_iron, 3);
    _mav_put_float_array(buf, 12, soft_iron, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN);
#else
    mavlink_magothy_3d_mag_cal_params_t packet;
    packet.uncalibrated_norm_mean = uncalibrated_norm_mean;
    packet.uncalibrated_norm_std_dev = uncalibrated_norm_std_dev;
    packet.uncalibrated_norm_max_error = uncalibrated_norm_max_error;
    packet.calibrated_norm_mean = calibrated_norm_mean;
    packet.calibrated_norm_std_dev = calibrated_norm_std_dev;
    packet.calibrated_norm_max_error = calibrated_norm_max_error;
    mav_array_memcpy(packet.hard_iron, hard_iron, sizeof(float)*3);
    mav_array_memcpy(packet.soft_iron, soft_iron, sizeof(float)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_CRC);
}

/**
 * @brief Encode a magothy_3d_mag_cal_params struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_3d_mag_cal_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_3d_mag_cal_params_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_3d_mag_cal_params_t* magothy_3d_mag_cal_params)
{
    return mavlink_msg_magothy_3d_mag_cal_params_pack(system_id, component_id, msg, magothy_3d_mag_cal_params->hard_iron, magothy_3d_mag_cal_params->soft_iron, magothy_3d_mag_cal_params->uncalibrated_norm_mean, magothy_3d_mag_cal_params->uncalibrated_norm_std_dev, magothy_3d_mag_cal_params->uncalibrated_norm_max_error, magothy_3d_mag_cal_params->calibrated_norm_mean, magothy_3d_mag_cal_params->calibrated_norm_std_dev, magothy_3d_mag_cal_params->calibrated_norm_max_error);
}

/**
 * @brief Encode a magothy_3d_mag_cal_params struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_3d_mag_cal_params C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_3d_mag_cal_params_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_3d_mag_cal_params_t* magothy_3d_mag_cal_params)
{
    return mavlink_msg_magothy_3d_mag_cal_params_pack_chan(system_id, component_id, chan, msg, magothy_3d_mag_cal_params->hard_iron, magothy_3d_mag_cal_params->soft_iron, magothy_3d_mag_cal_params->uncalibrated_norm_mean, magothy_3d_mag_cal_params->uncalibrated_norm_std_dev, magothy_3d_mag_cal_params->uncalibrated_norm_max_error, magothy_3d_mag_cal_params->calibrated_norm_mean, magothy_3d_mag_cal_params->calibrated_norm_std_dev, magothy_3d_mag_cal_params->calibrated_norm_max_error);
}

/**
 * @brief Send a magothy_3d_mag_cal_params message
 * @param chan MAVLink channel to send the message
 *
 * @param hard_iron  Hard Iron Offsets
 * @param soft_iron  Soft Iron Rotation Matrix
 * @param uncalibrated_norm_mean  Norm mean of raw mag data
 * @param uncalibrated_norm_std_dev  Norm standard deviation of raw mag data
 * @param uncalibrated_norm_max_error  Maximum norm error (difference from 1.0) of raw mag data
 * @param calibrated_norm_mean  Norm mean of calibrated mag data
 * @param calibrated_norm_std_dev  Norm standard deviation of calibrated mag data
 * @param calibrated_norm_max_error  Maximum norm error (difference from 1.0) of calibrated mag data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_3d_mag_cal_params_send(mavlink_channel_t chan, const float *hard_iron, const float *soft_iron, float uncalibrated_norm_mean, float uncalibrated_norm_std_dev, float uncalibrated_norm_max_error, float calibrated_norm_mean, float calibrated_norm_std_dev, float calibrated_norm_max_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN];
    _mav_put_float(buf, 48, uncalibrated_norm_mean);
    _mav_put_float(buf, 52, uncalibrated_norm_std_dev);
    _mav_put_float(buf, 56, uncalibrated_norm_max_error);
    _mav_put_float(buf, 60, calibrated_norm_mean);
    _mav_put_float(buf, 64, calibrated_norm_std_dev);
    _mav_put_float(buf, 68, calibrated_norm_max_error);
    _mav_put_float_array(buf, 0, hard_iron, 3);
    _mav_put_float_array(buf, 12, soft_iron, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS, buf, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_CRC);
#else
    mavlink_magothy_3d_mag_cal_params_t packet;
    packet.uncalibrated_norm_mean = uncalibrated_norm_mean;
    packet.uncalibrated_norm_std_dev = uncalibrated_norm_std_dev;
    packet.uncalibrated_norm_max_error = uncalibrated_norm_max_error;
    packet.calibrated_norm_mean = calibrated_norm_mean;
    packet.calibrated_norm_std_dev = calibrated_norm_std_dev;
    packet.calibrated_norm_max_error = calibrated_norm_max_error;
    mav_array_memcpy(packet.hard_iron, hard_iron, sizeof(float)*3);
    mav_array_memcpy(packet.soft_iron, soft_iron, sizeof(float)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_CRC);
#endif
}

/**
 * @brief Send a magothy_3d_mag_cal_params message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_3d_mag_cal_params_send_struct(mavlink_channel_t chan, const mavlink_magothy_3d_mag_cal_params_t* magothy_3d_mag_cal_params)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_3d_mag_cal_params_send(chan, magothy_3d_mag_cal_params->hard_iron, magothy_3d_mag_cal_params->soft_iron, magothy_3d_mag_cal_params->uncalibrated_norm_mean, magothy_3d_mag_cal_params->uncalibrated_norm_std_dev, magothy_3d_mag_cal_params->uncalibrated_norm_max_error, magothy_3d_mag_cal_params->calibrated_norm_mean, magothy_3d_mag_cal_params->calibrated_norm_std_dev, magothy_3d_mag_cal_params->calibrated_norm_max_error);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS, (const char *)magothy_3d_mag_cal_params, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_3d_mag_cal_params_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *hard_iron, const float *soft_iron, float uncalibrated_norm_mean, float uncalibrated_norm_std_dev, float uncalibrated_norm_max_error, float calibrated_norm_mean, float calibrated_norm_std_dev, float calibrated_norm_max_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 48, uncalibrated_norm_mean);
    _mav_put_float(buf, 52, uncalibrated_norm_std_dev);
    _mav_put_float(buf, 56, uncalibrated_norm_max_error);
    _mav_put_float(buf, 60, calibrated_norm_mean);
    _mav_put_float(buf, 64, calibrated_norm_std_dev);
    _mav_put_float(buf, 68, calibrated_norm_max_error);
    _mav_put_float_array(buf, 0, hard_iron, 3);
    _mav_put_float_array(buf, 12, soft_iron, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS, buf, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_CRC);
#else
    mavlink_magothy_3d_mag_cal_params_t *packet = (mavlink_magothy_3d_mag_cal_params_t *)msgbuf;
    packet->uncalibrated_norm_mean = uncalibrated_norm_mean;
    packet->uncalibrated_norm_std_dev = uncalibrated_norm_std_dev;
    packet->uncalibrated_norm_max_error = uncalibrated_norm_max_error;
    packet->calibrated_norm_mean = calibrated_norm_mean;
    packet->calibrated_norm_std_dev = calibrated_norm_std_dev;
    packet->calibrated_norm_max_error = calibrated_norm_max_error;
    mav_array_memcpy(packet->hard_iron, hard_iron, sizeof(float)*3);
    mav_array_memcpy(packet->soft_iron, soft_iron, sizeof(float)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_3D_MAG_CAL_PARAMS UNPACKING


/**
 * @brief Get field hard_iron from magothy_3d_mag_cal_params message
 *
 * @return  Hard Iron Offsets
 */
static inline uint16_t mavlink_msg_magothy_3d_mag_cal_params_get_hard_iron(const mavlink_message_t* msg, float *hard_iron)
{
    return _MAV_RETURN_float_array(msg, hard_iron, 3,  0);
}

/**
 * @brief Get field soft_iron from magothy_3d_mag_cal_params message
 *
 * @return  Soft Iron Rotation Matrix
 */
static inline uint16_t mavlink_msg_magothy_3d_mag_cal_params_get_soft_iron(const mavlink_message_t* msg, float *soft_iron)
{
    return _MAV_RETURN_float_array(msg, soft_iron, 9,  12);
}

/**
 * @brief Get field uncalibrated_norm_mean from magothy_3d_mag_cal_params message
 *
 * @return  Norm mean of raw mag data
 */
static inline float mavlink_msg_magothy_3d_mag_cal_params_get_uncalibrated_norm_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field uncalibrated_norm_std_dev from magothy_3d_mag_cal_params message
 *
 * @return  Norm standard deviation of raw mag data
 */
static inline float mavlink_msg_magothy_3d_mag_cal_params_get_uncalibrated_norm_std_dev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field uncalibrated_norm_max_error from magothy_3d_mag_cal_params message
 *
 * @return  Maximum norm error (difference from 1.0) of raw mag data
 */
static inline float mavlink_msg_magothy_3d_mag_cal_params_get_uncalibrated_norm_max_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field calibrated_norm_mean from magothy_3d_mag_cal_params message
 *
 * @return  Norm mean of calibrated mag data
 */
static inline float mavlink_msg_magothy_3d_mag_cal_params_get_calibrated_norm_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field calibrated_norm_std_dev from magothy_3d_mag_cal_params message
 *
 * @return  Norm standard deviation of calibrated mag data
 */
static inline float mavlink_msg_magothy_3d_mag_cal_params_get_calibrated_norm_std_dev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field calibrated_norm_max_error from magothy_3d_mag_cal_params message
 *
 * @return  Maximum norm error (difference from 1.0) of calibrated mag data
 */
static inline float mavlink_msg_magothy_3d_mag_cal_params_get_calibrated_norm_max_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Decode a magothy_3d_mag_cal_params message into a struct
 *
 * @param msg The message to decode
 * @param magothy_3d_mag_cal_params C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_3d_mag_cal_params_decode(const mavlink_message_t* msg, mavlink_magothy_3d_mag_cal_params_t* magothy_3d_mag_cal_params)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_3d_mag_cal_params_get_hard_iron(msg, magothy_3d_mag_cal_params->hard_iron);
    mavlink_msg_magothy_3d_mag_cal_params_get_soft_iron(msg, magothy_3d_mag_cal_params->soft_iron);
    magothy_3d_mag_cal_params->uncalibrated_norm_mean = mavlink_msg_magothy_3d_mag_cal_params_get_uncalibrated_norm_mean(msg);
    magothy_3d_mag_cal_params->uncalibrated_norm_std_dev = mavlink_msg_magothy_3d_mag_cal_params_get_uncalibrated_norm_std_dev(msg);
    magothy_3d_mag_cal_params->uncalibrated_norm_max_error = mavlink_msg_magothy_3d_mag_cal_params_get_uncalibrated_norm_max_error(msg);
    magothy_3d_mag_cal_params->calibrated_norm_mean = mavlink_msg_magothy_3d_mag_cal_params_get_calibrated_norm_mean(msg);
    magothy_3d_mag_cal_params->calibrated_norm_std_dev = mavlink_msg_magothy_3d_mag_cal_params_get_calibrated_norm_std_dev(msg);
    magothy_3d_mag_cal_params->calibrated_norm_max_error = mavlink_msg_magothy_3d_mag_cal_params_get_calibrated_norm_max_error(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN;
        memset(magothy_3d_mag_cal_params, 0, MAVLINK_MSG_ID_MAGOTHY_3D_MAG_CAL_PARAMS_LEN);
    memcpy(magothy_3d_mag_cal_params, _MAV_PAYLOAD(msg), len);
#endif
}
