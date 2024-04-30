#pragma once
// MESSAGE MAGOTHY_LOW_BANDWIDTH PACKING

#define MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH 50004

MAVPACKED(
typedef struct __mavlink_magothy_low_bandwidth_t {
 uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
 uint32_t onboard_control_sensors_present; /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.*/
 uint32_t onboard_control_sensors_enabled; /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.*/
 uint32_t onboard_control_sensors_health; /*<  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.*/
 int32_t lat; /*< [degE7] Latitude (WGS84, EGM96 ellipsoid)*/
 int32_t lon; /*< [degE7] Longitude (WGS84, EGM96 ellipsoid)*/
 uint16_t voltage_battery; /*< [mV] Battery voltage, UINT16_MAX: Voltage not sent by autopilot*/
 int16_t current_battery; /*< [cA] Battery current, -1: Current not sent by autopilot*/
 uint16_t mission_seq; /*<  Sequence number of the current active mission item. UINT16_MAX: not in mission*/
 uint16_t speed; /*< [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX*/
 uint16_t course; /*< [cdeg] Course over ground in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 uint16_t heading; /*< [cdeg] Heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 uint16_t position_error; /*< [cm] Position estimate error. If unknown, set to: UINT16_MAX*/
 uint16_t desired_speed; /*< [cm/s] Desired ground speed. If unknown, set to: UINT16_MAX*/
 uint16_t desired_course; /*< [cdeg] Desired course over ground in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 uint8_t type; /*<  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.*/
 int8_t battery_remaining; /*< [%] Battery energy remaining, -1: Battery remaining energy not sent by autopilot*/
 uint8_t satellites_visible; /*<  Number of satellites visible. If unknown, set to 255*/
 uint8_t is_position_independent; /*<  1 if position measurement is independent (gps), else 0*/
 uint32_t gcs_set_mode_uuid_lsb; /*<  UUID of most recent mode change*/
 uint16_t mission_crc; /*<  CRC-16/CCITT-FALSE of serialized loaded mission*/
}) mavlink_magothy_low_bandwidth_t;

#define MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN 52
#define MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_MIN_LEN 46
#define MAVLINK_MSG_ID_50004_LEN 52
#define MAVLINK_MSG_ID_50004_MIN_LEN 46

#define MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_CRC 243
#define MAVLINK_MSG_ID_50004_CRC 243



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAGOTHY_LOW_BANDWIDTH { \
    50004, \
    "MAGOTHY_LOW_BANDWIDTH", \
    21, \
    {  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_magothy_low_bandwidth_t, type) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_magothy_low_bandwidth_t, custom_mode) }, \
         { "onboard_control_sensors_present", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_magothy_low_bandwidth_t, onboard_control_sensors_present) }, \
         { "onboard_control_sensors_enabled", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_magothy_low_bandwidth_t, onboard_control_sensors_enabled) }, \
         { "onboard_control_sensors_health", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_magothy_low_bandwidth_t, onboard_control_sensors_health) }, \
         { "voltage_battery", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_magothy_low_bandwidth_t, voltage_battery) }, \
         { "current_battery", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_magothy_low_bandwidth_t, current_battery) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_INT8_T, 0, 43, offsetof(mavlink_magothy_low_bandwidth_t, battery_remaining) }, \
         { "mission_seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_magothy_low_bandwidth_t, mission_seq) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_magothy_low_bandwidth_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_magothy_low_bandwidth_t, lon) }, \
         { "speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_magothy_low_bandwidth_t, speed) }, \
         { "course", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_magothy_low_bandwidth_t, course) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_magothy_low_bandwidth_t, satellites_visible) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_magothy_low_bandwidth_t, heading) }, \
         { "is_position_independent", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_magothy_low_bandwidth_t, is_position_independent) }, \
         { "position_error", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_magothy_low_bandwidth_t, position_error) }, \
         { "desired_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_magothy_low_bandwidth_t, desired_speed) }, \
         { "desired_course", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_magothy_low_bandwidth_t, desired_course) }, \
         { "gcs_set_mode_uuid_lsb", NULL, MAVLINK_TYPE_UINT32_T, 0, 46, offsetof(mavlink_magothy_low_bandwidth_t, gcs_set_mode_uuid_lsb) }, \
         { "mission_crc", NULL, MAVLINK_TYPE_UINT16_T, 0, 50, offsetof(mavlink_magothy_low_bandwidth_t, mission_crc) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAGOTHY_LOW_BANDWIDTH { \
    "MAGOTHY_LOW_BANDWIDTH", \
    21, \
    {  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_magothy_low_bandwidth_t, type) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_magothy_low_bandwidth_t, custom_mode) }, \
         { "onboard_control_sensors_present", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_magothy_low_bandwidth_t, onboard_control_sensors_present) }, \
         { "onboard_control_sensors_enabled", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_magothy_low_bandwidth_t, onboard_control_sensors_enabled) }, \
         { "onboard_control_sensors_health", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_magothy_low_bandwidth_t, onboard_control_sensors_health) }, \
         { "voltage_battery", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_magothy_low_bandwidth_t, voltage_battery) }, \
         { "current_battery", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_magothy_low_bandwidth_t, current_battery) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_INT8_T, 0, 43, offsetof(mavlink_magothy_low_bandwidth_t, battery_remaining) }, \
         { "mission_seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_magothy_low_bandwidth_t, mission_seq) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_magothy_low_bandwidth_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_magothy_low_bandwidth_t, lon) }, \
         { "speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_magothy_low_bandwidth_t, speed) }, \
         { "course", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_magothy_low_bandwidth_t, course) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_magothy_low_bandwidth_t, satellites_visible) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_magothy_low_bandwidth_t, heading) }, \
         { "is_position_independent", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_magothy_low_bandwidth_t, is_position_independent) }, \
         { "position_error", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_magothy_low_bandwidth_t, position_error) }, \
         { "desired_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_magothy_low_bandwidth_t, desired_speed) }, \
         { "desired_course", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_magothy_low_bandwidth_t, desired_course) }, \
         { "gcs_set_mode_uuid_lsb", NULL, MAVLINK_TYPE_UINT32_T, 0, 46, offsetof(mavlink_magothy_low_bandwidth_t, gcs_set_mode_uuid_lsb) }, \
         { "mission_crc", NULL, MAVLINK_TYPE_UINT16_T, 0, 50, offsetof(mavlink_magothy_low_bandwidth_t, mission_crc) }, \
         } \
}
#endif

/**
 * @brief Pack a magothy_low_bandwidth message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.
 * @param custom_mode  A bitfield for use for autopilot-specific flags
 * @param onboard_control_sensors_present  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
 * @param onboard_control_sensors_enabled  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
 * @param onboard_control_sensors_health  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.
 * @param voltage_battery [mV] Battery voltage, UINT16_MAX: Voltage not sent by autopilot
 * @param current_battery [cA] Battery current, -1: Current not sent by autopilot
 * @param battery_remaining [%] Battery energy remaining, -1: Battery remaining energy not sent by autopilot
 * @param mission_seq  Sequence number of the current active mission item. UINT16_MAX: not in mission
 * @param lat [degE7] Latitude (WGS84, EGM96 ellipsoid)
 * @param lon [degE7] Longitude (WGS84, EGM96 ellipsoid)
 * @param speed [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 * @param course [cdeg] Course over ground in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible  Number of satellites visible. If unknown, set to 255
 * @param heading [cdeg] Heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param is_position_independent  1 if position measurement is independent (gps), else 0
 * @param position_error [cm] Position estimate error. If unknown, set to: UINT16_MAX
 * @param desired_speed [cm/s] Desired ground speed. If unknown, set to: UINT16_MAX
 * @param desired_course [cdeg] Desired course over ground in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param gcs_set_mode_uuid_lsb  UUID of most recent mode change
 * @param mission_crc  CRC-16/CCITT-FALSE of serialized loaded mission
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t type, uint32_t custom_mode, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t mission_seq, int32_t lat, int32_t lon, uint16_t speed, uint16_t course, uint8_t satellites_visible, uint16_t heading, uint8_t is_position_independent, uint16_t position_error, uint16_t desired_speed, uint16_t desired_course, uint32_t gcs_set_mode_uuid_lsb, uint16_t mission_crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 12, onboard_control_sensors_health);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lon);
    _mav_put_uint16_t(buf, 24, voltage_battery);
    _mav_put_int16_t(buf, 26, current_battery);
    _mav_put_uint16_t(buf, 28, mission_seq);
    _mav_put_uint16_t(buf, 30, speed);
    _mav_put_uint16_t(buf, 32, course);
    _mav_put_uint16_t(buf, 34, heading);
    _mav_put_uint16_t(buf, 36, position_error);
    _mav_put_uint16_t(buf, 38, desired_speed);
    _mav_put_uint16_t(buf, 40, desired_course);
    _mav_put_uint8_t(buf, 42, type);
    _mav_put_int8_t(buf, 43, battery_remaining);
    _mav_put_uint8_t(buf, 44, satellites_visible);
    _mav_put_uint8_t(buf, 45, is_position_independent);
    _mav_put_uint32_t(buf, 46, gcs_set_mode_uuid_lsb);
    _mav_put_uint16_t(buf, 50, mission_crc);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN);
#else
    mavlink_magothy_low_bandwidth_t packet;
    packet.custom_mode = custom_mode;
    packet.onboard_control_sensors_present = onboard_control_sensors_present;
    packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.lat = lat;
    packet.lon = lon;
    packet.voltage_battery = voltage_battery;
    packet.current_battery = current_battery;
    packet.mission_seq = mission_seq;
    packet.speed = speed;
    packet.course = course;
    packet.heading = heading;
    packet.position_error = position_error;
    packet.desired_speed = desired_speed;
    packet.desired_course = desired_course;
    packet.type = type;
    packet.battery_remaining = battery_remaining;
    packet.satellites_visible = satellites_visible;
    packet.is_position_independent = is_position_independent;
    packet.gcs_set_mode_uuid_lsb = gcs_set_mode_uuid_lsb;
    packet.mission_crc = mission_crc;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_CRC);
}

/**
 * @brief Pack a magothy_low_bandwidth message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.
 * @param custom_mode  A bitfield for use for autopilot-specific flags
 * @param onboard_control_sensors_present  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
 * @param onboard_control_sensors_enabled  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
 * @param onboard_control_sensors_health  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.
 * @param voltage_battery [mV] Battery voltage, UINT16_MAX: Voltage not sent by autopilot
 * @param current_battery [cA] Battery current, -1: Current not sent by autopilot
 * @param battery_remaining [%] Battery energy remaining, -1: Battery remaining energy not sent by autopilot
 * @param mission_seq  Sequence number of the current active mission item. UINT16_MAX: not in mission
 * @param lat [degE7] Latitude (WGS84, EGM96 ellipsoid)
 * @param lon [degE7] Longitude (WGS84, EGM96 ellipsoid)
 * @param speed [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 * @param course [cdeg] Course over ground in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible  Number of satellites visible. If unknown, set to 255
 * @param heading [cdeg] Heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param is_position_independent  1 if position measurement is independent (gps), else 0
 * @param position_error [cm] Position estimate error. If unknown, set to: UINT16_MAX
 * @param desired_speed [cm/s] Desired ground speed. If unknown, set to: UINT16_MAX
 * @param desired_course [cdeg] Desired course over ground in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param gcs_set_mode_uuid_lsb  UUID of most recent mode change
 * @param mission_crc  CRC-16/CCITT-FALSE of serialized loaded mission
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t type,uint32_t custom_mode,uint32_t onboard_control_sensors_present,uint32_t onboard_control_sensors_enabled,uint32_t onboard_control_sensors_health,uint16_t voltage_battery,int16_t current_battery,int8_t battery_remaining,uint16_t mission_seq,int32_t lat,int32_t lon,uint16_t speed,uint16_t course,uint8_t satellites_visible,uint16_t heading,uint8_t is_position_independent,uint16_t position_error,uint16_t desired_speed,uint16_t desired_course,uint32_t gcs_set_mode_uuid_lsb,uint16_t mission_crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 12, onboard_control_sensors_health);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lon);
    _mav_put_uint16_t(buf, 24, voltage_battery);
    _mav_put_int16_t(buf, 26, current_battery);
    _mav_put_uint16_t(buf, 28, mission_seq);
    _mav_put_uint16_t(buf, 30, speed);
    _mav_put_uint16_t(buf, 32, course);
    _mav_put_uint16_t(buf, 34, heading);
    _mav_put_uint16_t(buf, 36, position_error);
    _mav_put_uint16_t(buf, 38, desired_speed);
    _mav_put_uint16_t(buf, 40, desired_course);
    _mav_put_uint8_t(buf, 42, type);
    _mav_put_int8_t(buf, 43, battery_remaining);
    _mav_put_uint8_t(buf, 44, satellites_visible);
    _mav_put_uint8_t(buf, 45, is_position_independent);
    _mav_put_uint32_t(buf, 46, gcs_set_mode_uuid_lsb);
    _mav_put_uint16_t(buf, 50, mission_crc);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN);
#else
    mavlink_magothy_low_bandwidth_t packet;
    packet.custom_mode = custom_mode;
    packet.onboard_control_sensors_present = onboard_control_sensors_present;
    packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.lat = lat;
    packet.lon = lon;
    packet.voltage_battery = voltage_battery;
    packet.current_battery = current_battery;
    packet.mission_seq = mission_seq;
    packet.speed = speed;
    packet.course = course;
    packet.heading = heading;
    packet.position_error = position_error;
    packet.desired_speed = desired_speed;
    packet.desired_course = desired_course;
    packet.type = type;
    packet.battery_remaining = battery_remaining;
    packet.satellites_visible = satellites_visible;
    packet.is_position_independent = is_position_independent;
    packet.gcs_set_mode_uuid_lsb = gcs_set_mode_uuid_lsb;
    packet.mission_crc = mission_crc;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_CRC);
}

/**
 * @brief Encode a magothy_low_bandwidth struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param magothy_low_bandwidth C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_magothy_low_bandwidth_t* magothy_low_bandwidth)
{
    return mavlink_msg_magothy_low_bandwidth_pack(system_id, component_id, msg, magothy_low_bandwidth->type, magothy_low_bandwidth->custom_mode, magothy_low_bandwidth->onboard_control_sensors_present, magothy_low_bandwidth->onboard_control_sensors_enabled, magothy_low_bandwidth->onboard_control_sensors_health, magothy_low_bandwidth->voltage_battery, magothy_low_bandwidth->current_battery, magothy_low_bandwidth->battery_remaining, magothy_low_bandwidth->mission_seq, magothy_low_bandwidth->lat, magothy_low_bandwidth->lon, magothy_low_bandwidth->speed, magothy_low_bandwidth->course, magothy_low_bandwidth->satellites_visible, magothy_low_bandwidth->heading, magothy_low_bandwidth->is_position_independent, magothy_low_bandwidth->position_error, magothy_low_bandwidth->desired_speed, magothy_low_bandwidth->desired_course, magothy_low_bandwidth->gcs_set_mode_uuid_lsb, magothy_low_bandwidth->mission_crc);
}

/**
 * @brief Encode a magothy_low_bandwidth struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magothy_low_bandwidth C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_magothy_low_bandwidth_t* magothy_low_bandwidth)
{
    return mavlink_msg_magothy_low_bandwidth_pack_chan(system_id, component_id, chan, msg, magothy_low_bandwidth->type, magothy_low_bandwidth->custom_mode, magothy_low_bandwidth->onboard_control_sensors_present, magothy_low_bandwidth->onboard_control_sensors_enabled, magothy_low_bandwidth->onboard_control_sensors_health, magothy_low_bandwidth->voltage_battery, magothy_low_bandwidth->current_battery, magothy_low_bandwidth->battery_remaining, magothy_low_bandwidth->mission_seq, magothy_low_bandwidth->lat, magothy_low_bandwidth->lon, magothy_low_bandwidth->speed, magothy_low_bandwidth->course, magothy_low_bandwidth->satellites_visible, magothy_low_bandwidth->heading, magothy_low_bandwidth->is_position_independent, magothy_low_bandwidth->position_error, magothy_low_bandwidth->desired_speed, magothy_low_bandwidth->desired_course, magothy_low_bandwidth->gcs_set_mode_uuid_lsb, magothy_low_bandwidth->mission_crc);
}

/**
 * @brief Send a magothy_low_bandwidth message
 * @param chan MAVLink channel to send the message
 *
 * @param type  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.
 * @param custom_mode  A bitfield for use for autopilot-specific flags
 * @param onboard_control_sensors_present  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
 * @param onboard_control_sensors_enabled  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
 * @param onboard_control_sensors_health  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.
 * @param voltage_battery [mV] Battery voltage, UINT16_MAX: Voltage not sent by autopilot
 * @param current_battery [cA] Battery current, -1: Current not sent by autopilot
 * @param battery_remaining [%] Battery energy remaining, -1: Battery remaining energy not sent by autopilot
 * @param mission_seq  Sequence number of the current active mission item. UINT16_MAX: not in mission
 * @param lat [degE7] Latitude (WGS84, EGM96 ellipsoid)
 * @param lon [degE7] Longitude (WGS84, EGM96 ellipsoid)
 * @param speed [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 * @param course [cdeg] Course over ground in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible  Number of satellites visible. If unknown, set to 255
 * @param heading [cdeg] Heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param is_position_independent  1 if position measurement is independent (gps), else 0
 * @param position_error [cm] Position estimate error. If unknown, set to: UINT16_MAX
 * @param desired_speed [cm/s] Desired ground speed. If unknown, set to: UINT16_MAX
 * @param desired_course [cdeg] Desired course over ground in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param gcs_set_mode_uuid_lsb  UUID of most recent mode change
 * @param mission_crc  CRC-16/CCITT-FALSE of serialized loaded mission
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_magothy_low_bandwidth_send(mavlink_channel_t chan, uint8_t type, uint32_t custom_mode, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t mission_seq, int32_t lat, int32_t lon, uint16_t speed, uint16_t course, uint8_t satellites_visible, uint16_t heading, uint8_t is_position_independent, uint16_t position_error, uint16_t desired_speed, uint16_t desired_course, uint32_t gcs_set_mode_uuid_lsb, uint16_t mission_crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 12, onboard_control_sensors_health);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lon);
    _mav_put_uint16_t(buf, 24, voltage_battery);
    _mav_put_int16_t(buf, 26, current_battery);
    _mav_put_uint16_t(buf, 28, mission_seq);
    _mav_put_uint16_t(buf, 30, speed);
    _mav_put_uint16_t(buf, 32, course);
    _mav_put_uint16_t(buf, 34, heading);
    _mav_put_uint16_t(buf, 36, position_error);
    _mav_put_uint16_t(buf, 38, desired_speed);
    _mav_put_uint16_t(buf, 40, desired_course);
    _mav_put_uint8_t(buf, 42, type);
    _mav_put_int8_t(buf, 43, battery_remaining);
    _mav_put_uint8_t(buf, 44, satellites_visible);
    _mav_put_uint8_t(buf, 45, is_position_independent);
    _mav_put_uint32_t(buf, 46, gcs_set_mode_uuid_lsb);
    _mav_put_uint16_t(buf, 50, mission_crc);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH, buf, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_CRC);
#else
    mavlink_magothy_low_bandwidth_t packet;
    packet.custom_mode = custom_mode;
    packet.onboard_control_sensors_present = onboard_control_sensors_present;
    packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.lat = lat;
    packet.lon = lon;
    packet.voltage_battery = voltage_battery;
    packet.current_battery = current_battery;
    packet.mission_seq = mission_seq;
    packet.speed = speed;
    packet.course = course;
    packet.heading = heading;
    packet.position_error = position_error;
    packet.desired_speed = desired_speed;
    packet.desired_course = desired_course;
    packet.type = type;
    packet.battery_remaining = battery_remaining;
    packet.satellites_visible = satellites_visible;
    packet.is_position_independent = is_position_independent;
    packet.gcs_set_mode_uuid_lsb = gcs_set_mode_uuid_lsb;
    packet.mission_crc = mission_crc;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH, (const char *)&packet, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_CRC);
#endif
}

/**
 * @brief Send a magothy_low_bandwidth message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_magothy_low_bandwidth_send_struct(mavlink_channel_t chan, const mavlink_magothy_low_bandwidth_t* magothy_low_bandwidth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_magothy_low_bandwidth_send(chan, magothy_low_bandwidth->type, magothy_low_bandwidth->custom_mode, magothy_low_bandwidth->onboard_control_sensors_present, magothy_low_bandwidth->onboard_control_sensors_enabled, magothy_low_bandwidth->onboard_control_sensors_health, magothy_low_bandwidth->voltage_battery, magothy_low_bandwidth->current_battery, magothy_low_bandwidth->battery_remaining, magothy_low_bandwidth->mission_seq, magothy_low_bandwidth->lat, magothy_low_bandwidth->lon, magothy_low_bandwidth->speed, magothy_low_bandwidth->course, magothy_low_bandwidth->satellites_visible, magothy_low_bandwidth->heading, magothy_low_bandwidth->is_position_independent, magothy_low_bandwidth->position_error, magothy_low_bandwidth->desired_speed, magothy_low_bandwidth->desired_course, magothy_low_bandwidth->gcs_set_mode_uuid_lsb, magothy_low_bandwidth->mission_crc);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH, (const char *)magothy_low_bandwidth, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_magothy_low_bandwidth_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type, uint32_t custom_mode, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t mission_seq, int32_t lat, int32_t lon, uint16_t speed, uint16_t course, uint8_t satellites_visible, uint16_t heading, uint8_t is_position_independent, uint16_t position_error, uint16_t desired_speed, uint16_t desired_course, uint32_t gcs_set_mode_uuid_lsb, uint16_t mission_crc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_uint32_t(buf, 4, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 12, onboard_control_sensors_health);
    _mav_put_int32_t(buf, 16, lat);
    _mav_put_int32_t(buf, 20, lon);
    _mav_put_uint16_t(buf, 24, voltage_battery);
    _mav_put_int16_t(buf, 26, current_battery);
    _mav_put_uint16_t(buf, 28, mission_seq);
    _mav_put_uint16_t(buf, 30, speed);
    _mav_put_uint16_t(buf, 32, course);
    _mav_put_uint16_t(buf, 34, heading);
    _mav_put_uint16_t(buf, 36, position_error);
    _mav_put_uint16_t(buf, 38, desired_speed);
    _mav_put_uint16_t(buf, 40, desired_course);
    _mav_put_uint8_t(buf, 42, type);
    _mav_put_int8_t(buf, 43, battery_remaining);
    _mav_put_uint8_t(buf, 44, satellites_visible);
    _mav_put_uint8_t(buf, 45, is_position_independent);
    _mav_put_uint32_t(buf, 46, gcs_set_mode_uuid_lsb);
    _mav_put_uint16_t(buf, 50, mission_crc);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH, buf, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_CRC);
#else
    mavlink_magothy_low_bandwidth_t *packet = (mavlink_magothy_low_bandwidth_t *)msgbuf;
    packet->custom_mode = custom_mode;
    packet->onboard_control_sensors_present = onboard_control_sensors_present;
    packet->onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet->onboard_control_sensors_health = onboard_control_sensors_health;
    packet->lat = lat;
    packet->lon = lon;
    packet->voltage_battery = voltage_battery;
    packet->current_battery = current_battery;
    packet->mission_seq = mission_seq;
    packet->speed = speed;
    packet->course = course;
    packet->heading = heading;
    packet->position_error = position_error;
    packet->desired_speed = desired_speed;
    packet->desired_course = desired_course;
    packet->type = type;
    packet->battery_remaining = battery_remaining;
    packet->satellites_visible = satellites_visible;
    packet->is_position_independent = is_position_independent;
    packet->gcs_set_mode_uuid_lsb = gcs_set_mode_uuid_lsb;
    packet->mission_crc = mission_crc;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH, (const char *)packet, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_MIN_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_CRC);
#endif
}
#endif

#endif

// MESSAGE MAGOTHY_LOW_BANDWIDTH UNPACKING


/**
 * @brief Get field type from magothy_low_bandwidth message
 *
 * @return  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.
 */
static inline uint8_t mavlink_msg_magothy_low_bandwidth_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field custom_mode from magothy_low_bandwidth message
 *
 * @return  A bitfield for use for autopilot-specific flags
 */
static inline uint32_t mavlink_msg_magothy_low_bandwidth_get_custom_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field onboard_control_sensors_present from magothy_low_bandwidth message
 *
 * @return  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
 */
static inline uint32_t mavlink_msg_magothy_low_bandwidth_get_onboard_control_sensors_present(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field onboard_control_sensors_enabled from magothy_low_bandwidth message
 *
 * @return  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
 */
static inline uint32_t mavlink_msg_magothy_low_bandwidth_get_onboard_control_sensors_enabled(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field onboard_control_sensors_health from magothy_low_bandwidth message
 *
 * @return  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.
 */
static inline uint32_t mavlink_msg_magothy_low_bandwidth_get_onboard_control_sensors_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field voltage_battery from magothy_low_bandwidth message
 *
 * @return [mV] Battery voltage, UINT16_MAX: Voltage not sent by autopilot
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_get_voltage_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field current_battery from magothy_low_bandwidth message
 *
 * @return [cA] Battery current, -1: Current not sent by autopilot
 */
static inline int16_t mavlink_msg_magothy_low_bandwidth_get_current_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field battery_remaining from magothy_low_bandwidth message
 *
 * @return [%] Battery energy remaining, -1: Battery remaining energy not sent by autopilot
 */
static inline int8_t mavlink_msg_magothy_low_bandwidth_get_battery_remaining(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  43);
}

/**
 * @brief Get field mission_seq from magothy_low_bandwidth message
 *
 * @return  Sequence number of the current active mission item. UINT16_MAX: not in mission
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_get_mission_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field lat from magothy_low_bandwidth message
 *
 * @return [degE7] Latitude (WGS84, EGM96 ellipsoid)
 */
static inline int32_t mavlink_msg_magothy_low_bandwidth_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field lon from magothy_low_bandwidth message
 *
 * @return [degE7] Longitude (WGS84, EGM96 ellipsoid)
 */
static inline int32_t mavlink_msg_magothy_low_bandwidth_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field speed from magothy_low_bandwidth message
 *
 * @return [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field course from magothy_low_bandwidth message
 *
 * @return [cdeg] Course over ground in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_get_course(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field satellites_visible from magothy_low_bandwidth message
 *
 * @return  Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mavlink_msg_magothy_low_bandwidth_get_satellites_visible(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field heading from magothy_low_bandwidth message
 *
 * @return [cdeg] Heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field is_position_independent from magothy_low_bandwidth message
 *
 * @return  1 if position measurement is independent (gps), else 0
 */
static inline uint8_t mavlink_msg_magothy_low_bandwidth_get_is_position_independent(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field position_error from magothy_low_bandwidth message
 *
 * @return [cm] Position estimate error. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_get_position_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field desired_speed from magothy_low_bandwidth message
 *
 * @return [cm/s] Desired ground speed. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_get_desired_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  38);
}

/**
 * @brief Get field desired_course from magothy_low_bandwidth message
 *
 * @return [cdeg] Desired course over ground in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_get_desired_course(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field gcs_set_mode_uuid_lsb from magothy_low_bandwidth message
 *
 * @return  UUID of most recent mode change
 */
static inline uint32_t mavlink_msg_magothy_low_bandwidth_get_gcs_set_mode_uuid_lsb(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  46);
}

/**
 * @brief Get field mission_crc from magothy_low_bandwidth message
 *
 * @return  CRC-16/CCITT-FALSE of serialized loaded mission
 */
static inline uint16_t mavlink_msg_magothy_low_bandwidth_get_mission_crc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  50);
}

/**
 * @brief Decode a magothy_low_bandwidth message into a struct
 *
 * @param msg The message to decode
 * @param magothy_low_bandwidth C-struct to decode the message contents into
 */
static inline void mavlink_msg_magothy_low_bandwidth_decode(const mavlink_message_t* msg, mavlink_magothy_low_bandwidth_t* magothy_low_bandwidth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    magothy_low_bandwidth->custom_mode = mavlink_msg_magothy_low_bandwidth_get_custom_mode(msg);
    magothy_low_bandwidth->onboard_control_sensors_present = mavlink_msg_magothy_low_bandwidth_get_onboard_control_sensors_present(msg);
    magothy_low_bandwidth->onboard_control_sensors_enabled = mavlink_msg_magothy_low_bandwidth_get_onboard_control_sensors_enabled(msg);
    magothy_low_bandwidth->onboard_control_sensors_health = mavlink_msg_magothy_low_bandwidth_get_onboard_control_sensors_health(msg);
    magothy_low_bandwidth->lat = mavlink_msg_magothy_low_bandwidth_get_lat(msg);
    magothy_low_bandwidth->lon = mavlink_msg_magothy_low_bandwidth_get_lon(msg);
    magothy_low_bandwidth->voltage_battery = mavlink_msg_magothy_low_bandwidth_get_voltage_battery(msg);
    magothy_low_bandwidth->current_battery = mavlink_msg_magothy_low_bandwidth_get_current_battery(msg);
    magothy_low_bandwidth->mission_seq = mavlink_msg_magothy_low_bandwidth_get_mission_seq(msg);
    magothy_low_bandwidth->speed = mavlink_msg_magothy_low_bandwidth_get_speed(msg);
    magothy_low_bandwidth->course = mavlink_msg_magothy_low_bandwidth_get_course(msg);
    magothy_low_bandwidth->heading = mavlink_msg_magothy_low_bandwidth_get_heading(msg);
    magothy_low_bandwidth->position_error = mavlink_msg_magothy_low_bandwidth_get_position_error(msg);
    magothy_low_bandwidth->desired_speed = mavlink_msg_magothy_low_bandwidth_get_desired_speed(msg);
    magothy_low_bandwidth->desired_course = mavlink_msg_magothy_low_bandwidth_get_desired_course(msg);
    magothy_low_bandwidth->type = mavlink_msg_magothy_low_bandwidth_get_type(msg);
    magothy_low_bandwidth->battery_remaining = mavlink_msg_magothy_low_bandwidth_get_battery_remaining(msg);
    magothy_low_bandwidth->satellites_visible = mavlink_msg_magothy_low_bandwidth_get_satellites_visible(msg);
    magothy_low_bandwidth->is_position_independent = mavlink_msg_magothy_low_bandwidth_get_is_position_independent(msg);
    magothy_low_bandwidth->gcs_set_mode_uuid_lsb = mavlink_msg_magothy_low_bandwidth_get_gcs_set_mode_uuid_lsb(msg);
    magothy_low_bandwidth->mission_crc = mavlink_msg_magothy_low_bandwidth_get_mission_crc(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN? msg->len : MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN;
        memset(magothy_low_bandwidth, 0, MAVLINK_MSG_ID_MAGOTHY_LOW_BANDWIDTH_LEN);
    memcpy(magothy_low_bandwidth, _MAV_PAYLOAD(msg), len);
#endif
}
