/*! \file mavlink_lookup.hpp
 *  This file was automatically generated from magothy.xml
 *  Copyright 2023 Magothy River Technologies
 */

#pragma once

#include <cstdint>
#include <sstream>
#include <string>
#include <unordered_map>

inline const char* mavlink_lookup(uint32_t key, const std::unordered_map<uint32_t, const char*>& map) {
    auto it = map.find(key);
    if (it != map.end()) {
        return it->second;
    }
    return "Unknown";
}


inline std::string mavlink_pretty(uint32_t key, const std::unordered_map<uint32_t, const char*>& map) {
    std::stringstream ss;
    ss << mavlink_lookup(key, map) << "(" << key << ")";
    return ss.str();
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MSG_ID_LOOKUP = {
    {0, "HEARTBEAT"},
    {1, "SYS_STATUS"},
    {2, "SYSTEM_TIME"},
    {4, "PING"},
    {5, "CHANGE_OPERATOR_CONTROL"},
    {6, "CHANGE_OPERATOR_CONTROL_ACK"},
    {7, "AUTH_KEY"},
    {8, "LINK_NODE_STATUS"},
    {11, "SET_MODE"},
    {19, "PARAM_ACK_TRANSACTION"},
    {20, "PARAM_REQUEST_READ"},
    {21, "PARAM_REQUEST_LIST"},
    {22, "PARAM_VALUE"},
    {23, "PARAM_SET"},
    {24, "GPS_RAW_INT"},
    {25, "GPS_STATUS"},
    {26, "SCALED_IMU"},
    {27, "RAW_IMU"},
    {28, "RAW_PRESSURE"},
    {29, "SCALED_PRESSURE"},
    {30, "ATTITUDE"},
    {31, "ATTITUDE_QUATERNION"},
    {32, "LOCAL_POSITION_NED"},
    {33, "GLOBAL_POSITION_INT"},
    {34, "RC_CHANNELS_SCALED"},
    {35, "RC_CHANNELS_RAW"},
    {36, "SERVO_OUTPUT_RAW"},
    {37, "MISSION_REQUEST_PARTIAL_LIST"},
    {38, "MISSION_WRITE_PARTIAL_LIST"},
    {39, "MISSION_ITEM"},
    {40, "MISSION_REQUEST"},
    {41, "MISSION_SET_CURRENT"},
    {42, "MISSION_CURRENT"},
    {43, "MISSION_REQUEST_LIST"},
    {44, "MISSION_COUNT"},
    {45, "MISSION_CLEAR_ALL"},
    {46, "MISSION_ITEM_REACHED"},
    {47, "MISSION_ACK"},
    {48, "SET_GPS_GLOBAL_ORIGIN"},
    {49, "GPS_GLOBAL_ORIGIN"},
    {50, "PARAM_MAP_RC"},
    {51, "MISSION_REQUEST_INT"},
    {52, "MISSION_CHANGED"},
    {54, "SAFETY_SET_ALLOWED_AREA"},
    {55, "SAFETY_ALLOWED_AREA"},
    {61, "ATTITUDE_QUATERNION_COV"},
    {62, "NAV_CONTROLLER_OUTPUT"},
    {63, "GLOBAL_POSITION_INT_COV"},
    {64, "LOCAL_POSITION_NED_COV"},
    {65, "RC_CHANNELS"},
    {66, "REQUEST_DATA_STREAM"},
    {67, "DATA_STREAM"},
    {69, "MANUAL_CONTROL"},
    {70, "RC_CHANNELS_OVERRIDE"},
    {73, "MISSION_ITEM_INT"},
    {74, "VFR_HUD"},
    {75, "COMMAND_INT"},
    {76, "COMMAND_LONG"},
    {77, "COMMAND_ACK"},
    {80, "COMMAND_CANCEL"},
    {81, "MANUAL_SETPOINT"},
    {82, "SET_ATTITUDE_TARGET"},
    {83, "ATTITUDE_TARGET"},
    {84, "SET_POSITION_TARGET_LOCAL_NED"},
    {85, "POSITION_TARGET_LOCAL_NED"},
    {86, "SET_POSITION_TARGET_GLOBAL_INT"},
    {87, "POSITION_TARGET_GLOBAL_INT"},
    {89, "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET"},
    {90, "HIL_STATE"},
    {91, "HIL_CONTROLS"},
    {92, "HIL_RC_INPUTS_RAW"},
    {93, "HIL_ACTUATOR_CONTROLS"},
    {100, "OPTICAL_FLOW"},
    {101, "GLOBAL_VISION_POSITION_ESTIMATE"},
    {102, "VISION_POSITION_ESTIMATE"},
    {103, "VISION_SPEED_ESTIMATE"},
    {104, "VICON_POSITION_ESTIMATE"},
    {105, "HIGHRES_IMU"},
    {106, "OPTICAL_FLOW_RAD"},
    {107, "HIL_SENSOR"},
    {108, "SIM_STATE"},
    {109, "RADIO_STATUS"},
    {110, "FILE_TRANSFER_PROTOCOL"},
    {111, "TIMESYNC"},
    {112, "CAMERA_TRIGGER"},
    {113, "HIL_GPS"},
    {114, "HIL_OPTICAL_FLOW"},
    {115, "HIL_STATE_QUATERNION"},
    {116, "SCALED_IMU2"},
    {117, "LOG_REQUEST_LIST"},
    {118, "LOG_ENTRY"},
    {119, "LOG_REQUEST_DATA"},
    {120, "LOG_DATA"},
    {121, "LOG_ERASE"},
    {122, "LOG_REQUEST_END"},
    {123, "GPS_INJECT_DATA"},
    {124, "GPS2_RAW"},
    {125, "POWER_STATUS"},
    {126, "SERIAL_CONTROL"},
    {127, "GPS_RTK"},
    {128, "GPS2_RTK"},
    {129, "SCALED_IMU3"},
    {130, "DATA_TRANSMISSION_HANDSHAKE"},
    {131, "ENCAPSULATED_DATA"},
    {132, "DISTANCE_SENSOR"},
    {133, "TERRAIN_REQUEST"},
    {134, "TERRAIN_DATA"},
    {135, "TERRAIN_CHECK"},
    {136, "TERRAIN_REPORT"},
    {137, "SCALED_PRESSURE2"},
    {138, "ATT_POS_MOCAP"},
    {139, "SET_ACTUATOR_CONTROL_TARGET"},
    {140, "ACTUATOR_CONTROL_TARGET"},
    {141, "ALTITUDE"},
    {142, "RESOURCE_REQUEST"},
    {143, "SCALED_PRESSURE3"},
    {144, "FOLLOW_TARGET"},
    {146, "CONTROL_SYSTEM_STATE"},
    {147, "BATTERY_STATUS"},
    {148, "AUTOPILOT_VERSION"},
    {149, "LANDING_TARGET"},
    {162, "FENCE_STATUS"},
    {192, "MAG_CAL_REPORT"},
    {225, "EFI_STATUS"},
    {230, "ESTIMATOR_STATUS"},
    {231, "WIND_COV"},
    {232, "GPS_INPUT"},
    {233, "GPS_RTCM_DATA"},
    {234, "HIGH_LATENCY"},
    {235, "HIGH_LATENCY2"},
    {241, "VIBRATION"},
    {242, "HOME_POSITION"},
    {243, "SET_HOME_POSITION"},
    {244, "MESSAGE_INTERVAL"},
    {245, "EXTENDED_SYS_STATE"},
    {246, "ADSB_VEHICLE"},
    {247, "COLLISION"},
    {248, "V2_EXTENSION"},
    {249, "MEMORY_VECT"},
    {250, "DEBUG_VECT"},
    {251, "NAMED_VALUE_FLOAT"},
    {252, "NAMED_VALUE_INT"},
    {253, "STATUSTEXT"},
    {254, "DEBUG"},
    {256, "SETUP_SIGNING"},
    {257, "BUTTON_CHANGE"},
    {258, "PLAY_TUNE"},
    {259, "CAMERA_INFORMATION"},
    {260, "CAMERA_SETTINGS"},
    {261, "STORAGE_INFORMATION"},
    {262, "CAMERA_CAPTURE_STATUS"},
    {263, "CAMERA_IMAGE_CAPTURED"},
    {264, "FLIGHT_INFORMATION"},
    {265, "MOUNT_ORIENTATION"},
    {266, "LOGGING_DATA"},
    {267, "LOGGING_DATA_ACKED"},
    {268, "LOGGING_ACK"},
    {269, "VIDEO_STREAM_INFORMATION"},
    {270, "VIDEO_STREAM_STATUS"},
    {271, "CAMERA_FOV_STATUS"},
    {275, "CAMERA_TRACKING_IMAGE_STATUS"},
    {276, "CAMERA_TRACKING_GEO_STATUS"},
    {280, "GIMBAL_MANAGER_INFORMATION"},
    {281, "GIMBAL_MANAGER_STATUS"},
    {282, "GIMBAL_MANAGER_SET_ATTITUDE"},
    {283, "GIMBAL_DEVICE_INFORMATION"},
    {284, "GIMBAL_DEVICE_SET_ATTITUDE"},
    {285, "GIMBAL_DEVICE_ATTITUDE_STATUS"},
    {286, "AUTOPILOT_STATE_FOR_GIMBAL_DEVICE"},
    {287, "GIMBAL_MANAGER_SET_PITCHYAW"},
    {288, "GIMBAL_MANAGER_SET_MANUAL_CONTROL"},
    {290, "ESC_INFO"},
    {291, "ESC_STATUS"},
    {299, "WIFI_CONFIG_AP"},
    {300, "PROTOCOL_VERSION"},
    {301, "AIS_VESSEL"},
    {310, "UAVCAN_NODE_STATUS"},
    {311, "UAVCAN_NODE_INFO"},
    {320, "PARAM_EXT_REQUEST_READ"},
    {321, "PARAM_EXT_REQUEST_LIST"},
    {322, "PARAM_EXT_VALUE"},
    {323, "PARAM_EXT_SET"},
    {324, "PARAM_EXT_ACK"},
    {325, "PARAM_EXT_VALUE_TRIMMED"},
    {326, "PARAM_EXT_SET_TRIMMED"},
    {327, "PARAM_EXT_ACK_TRIMMED"},
    {330, "OBSTACLE_DISTANCE"},
    {331, "ODOMETRY"},
    {332, "TRAJECTORY_REPRESENTATION_WAYPOINTS"},
    {333, "TRAJECTORY_REPRESENTATION_BEZIER"},
    {334, "CELLULAR_STATUS"},
    {335, "ISBD_LINK_STATUS"},
    {336, "CELLULAR_CONFIG"},
    {339, "RAW_RPM"},
    {340, "UTM_GLOBAL_POSITION"},
    {350, "DEBUG_FLOAT_ARRAY"},
    {360, "ORBIT_EXECUTION_STATUS"},
    {370, "SMART_BATTERY_INFO"},
    {373, "GENERATOR_STATUS"},
    {375, "ACTUATOR_OUTPUT_STATUS"},
    {380, "TIME_ESTIMATE_TO_TARGET"},
    {385, "TUNNEL"},
    {390, "ONBOARD_COMPUTER_STATUS"},
    {395, "COMPONENT_INFORMATION"},
    {400, "PLAY_TUNE_V2"},
    {401, "SUPPORTED_TUNES"},
    {9000, "WHEEL_DISTANCE"},
    {9005, "WINCH_STATUS"},
    {12900, "OPEN_DRONE_ID_BASIC_ID"},
    {12901, "OPEN_DRONE_ID_LOCATION"},
    {12902, "OPEN_DRONE_ID_AUTHENTICATION"},
    {12903, "OPEN_DRONE_ID_SELF_ID"},
    {12904, "OPEN_DRONE_ID_SYSTEM"},
    {12905, "OPEN_DRONE_ID_OPERATOR_ID"},
    {12915, "OPEN_DRONE_ID_MESSAGE_PACK"},
    {50001, "MAGOTHY_ECHOSOUNDER"},
    {50002, "MAGOTHY_MISSION_TELEMETRY"},
    {50003, "MAGOTHY_WATER_CURRENT"},
    {50004, "MAGOTHY_LOW_BANDWIDTH"},
    {50100, "MAGOTHY_CAPABILITY"},
    {50101, "MAGOTHY_3D_MAG_CAL_PARAMS"},
    {50150, "MAGOTHY_LICENSE_INFO"},
    {50151, "MAGOTHY_LICENSE_TRANSFER_INITIALIZE"},
    {50153, "MAGOTHY_LICENSE_TRANSFER_REQUEST"},
    {50154, "MAGOTHY_LICENSE_TRANSFER"},
    {50155, "MAGOTHY_LICENSE_TRANSFER_ACK"},
};

inline const char* mavlink_msg_id_str(uint32_t msg_id) {
    return mavlink_lookup(msg_id, MAVLINK_MSG_ID_LOOKUP);
}

inline std::string mavlink_msg_id_pretty(uint32_t msg_id) {
    return mavlink_pretty(msg_id, MAVLINK_MSG_ID_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_ADSB_ALTITUDE_TYPE_LOOKUP = {
    {0, "ADSB_ALTITUDE_TYPE_PRESSURE_QNH"},
    {1, "ADSB_ALTITUDE_TYPE_GEOMETRIC"},
};

inline const char* mavlink_adsb_altitude_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_ADSB_ALTITUDE_TYPE_LOOKUP);
}

inline std::string mavlink_adsb_altitude_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_ADSB_ALTITUDE_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_ADSB_EMITTER_TYPE_LOOKUP = {
    {0, "ADSB_EMITTER_TYPE_NO_INFO"},
    {1, "ADSB_EMITTER_TYPE_LIGHT"},
    {2, "ADSB_EMITTER_TYPE_SMALL"},
    {3, "ADSB_EMITTER_TYPE_LARGE"},
    {4, "ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE"},
    {5, "ADSB_EMITTER_TYPE_HEAVY"},
    {6, "ADSB_EMITTER_TYPE_HIGHLY_MANUV"},
    {7, "ADSB_EMITTER_TYPE_ROTOCRAFT"},
    {8, "ADSB_EMITTER_TYPE_UNASSIGNED"},
    {9, "ADSB_EMITTER_TYPE_GLIDER"},
    {10, "ADSB_EMITTER_TYPE_LIGHTER_AIR"},
    {11, "ADSB_EMITTER_TYPE_PARACHUTE"},
    {12, "ADSB_EMITTER_TYPE_ULTRA_LIGHT"},
    {13, "ADSB_EMITTER_TYPE_UNASSIGNED2"},
    {14, "ADSB_EMITTER_TYPE_UAV"},
    {15, "ADSB_EMITTER_TYPE_SPACE"},
    {16, "ADSB_EMITTER_TYPE_UNASSGINED3"},
    {17, "ADSB_EMITTER_TYPE_EMERGENCY_SURFACE"},
    {18, "ADSB_EMITTER_TYPE_SERVICE_SURFACE"},
    {19, "ADSB_EMITTER_TYPE_POINT_OBSTACLE"},
};

inline const char* mavlink_adsb_emitter_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_ADSB_EMITTER_TYPE_LOOKUP);
}

inline std::string mavlink_adsb_emitter_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_ADSB_EMITTER_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_ADSB_FLAGS_LOOKUP = {
    {1, "ADSB_FLAGS_VALID_COORDS"},
    {2, "ADSB_FLAGS_VALID_ALTITUDE"},
    {4, "ADSB_FLAGS_VALID_HEADING"},
    {8, "ADSB_FLAGS_VALID_VELOCITY"},
    {16, "ADSB_FLAGS_VALID_CALLSIGN"},
    {32, "ADSB_FLAGS_VALID_SQUAWK"},
    {64, "ADSB_FLAGS_SIMULATED"},
    {128, "ADSB_FLAGS_VERTICAL_VELOCITY_VALID"},
    {256, "ADSB_FLAGS_BARO_VALID"},
    {32768, "ADSB_FLAGS_SOURCE_UAT"},
};

inline const char* mavlink_adsb_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_ADSB_FLAGS_LOOKUP);
}

inline std::string mavlink_adsb_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_ADSB_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_AIS_FLAGS_LOOKUP = {
    {1, "AIS_FLAGS_POSITION_ACCURACY"},
    {2, "AIS_FLAGS_VALID_COG"},
    {4, "AIS_FLAGS_VALID_VELOCITY"},
    {8, "AIS_FLAGS_HIGH_VELOCITY"},
    {16, "AIS_FLAGS_VALID_TURN_RATE"},
    {32, "AIS_FLAGS_TURN_RATE_SIGN_ONLY"},
    {64, "AIS_FLAGS_VALID_DIMENSIONS"},
    {128, "AIS_FLAGS_LARGE_BOW_DIMENSION"},
    {256, "AIS_FLAGS_LARGE_STERN_DIMENSION"},
    {512, "AIS_FLAGS_LARGE_PORT_DIMENSION"},
    {1024, "AIS_FLAGS_LARGE_STARBOARD_DIMENSION"},
    {2048, "AIS_FLAGS_VALID_CALLSIGN"},
    {4096, "AIS_FLAGS_VALID_NAME"},
};

inline const char* mavlink_ais_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_AIS_FLAGS_LOOKUP);
}

inline std::string mavlink_ais_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_AIS_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_AIS_NAV_STATUS_LOOKUP = {
    {0, "UNDER_WAY"},
    {1, "AIS_NAV_ANCHORED"},
    {2, "AIS_NAV_UN_COMMANDED"},
    {3, "AIS_NAV_RESTRICTED_MANOEUVERABILITY"},
    {4, "AIS_NAV_DRAUGHT_CONSTRAINED"},
    {5, "AIS_NAV_MOORED"},
    {6, "AIS_NAV_AGROUND"},
    {7, "AIS_NAV_FISHING"},
    {8, "AIS_NAV_SAILING"},
    {9, "AIS_NAV_RESERVED_HSC"},
    {10, "AIS_NAV_RESERVED_WIG"},
    {11, "AIS_NAV_RESERVED_1"},
    {12, "AIS_NAV_RESERVED_2"},
    {13, "AIS_NAV_RESERVED_3"},
    {14, "AIS_NAV_AIS_SART"},
    {15, "AIS_NAV_UNKNOWN"},
};

inline const char* mavlink_ais_nav_status_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_AIS_NAV_STATUS_LOOKUP);
}

inline std::string mavlink_ais_nav_status_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_AIS_NAV_STATUS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_AIS_TYPE_LOOKUP = {
    {0, "AIS_TYPE_UNKNOWN"},
    {1, "AIS_TYPE_RESERVED_1"},
    {2, "AIS_TYPE_RESERVED_2"},
    {3, "AIS_TYPE_RESERVED_3"},
    {4, "AIS_TYPE_RESERVED_4"},
    {5, "AIS_TYPE_RESERVED_5"},
    {6, "AIS_TYPE_RESERVED_6"},
    {7, "AIS_TYPE_RESERVED_7"},
    {8, "AIS_TYPE_RESERVED_8"},
    {9, "AIS_TYPE_RESERVED_9"},
    {10, "AIS_TYPE_RESERVED_10"},
    {11, "AIS_TYPE_RESERVED_11"},
    {12, "AIS_TYPE_RESERVED_12"},
    {13, "AIS_TYPE_RESERVED_13"},
    {14, "AIS_TYPE_RESERVED_14"},
    {15, "AIS_TYPE_RESERVED_15"},
    {16, "AIS_TYPE_RESERVED_16"},
    {17, "AIS_TYPE_RESERVED_17"},
    {18, "AIS_TYPE_RESERVED_18"},
    {19, "AIS_TYPE_RESERVED_19"},
    {20, "AIS_TYPE_WIG"},
    {21, "AIS_TYPE_WIG_HAZARDOUS_A"},
    {22, "AIS_TYPE_WIG_HAZARDOUS_B"},
    {23, "AIS_TYPE_WIG_HAZARDOUS_C"},
    {24, "AIS_TYPE_WIG_HAZARDOUS_D"},
    {25, "AIS_TYPE_WIG_RESERVED_1"},
    {26, "AIS_TYPE_WIG_RESERVED_2"},
    {27, "AIS_TYPE_WIG_RESERVED_3"},
    {28, "AIS_TYPE_WIG_RESERVED_4"},
    {29, "AIS_TYPE_WIG_RESERVED_5"},
    {30, "AIS_TYPE_FISHING"},
    {31, "AIS_TYPE_TOWING"},
    {32, "AIS_TYPE_TOWING_LARGE"},
    {33, "AIS_TYPE_DREDGING"},
    {34, "AIS_TYPE_DIVING"},
    {35, "AIS_TYPE_MILITARY"},
    {36, "AIS_TYPE_SAILING"},
    {37, "AIS_TYPE_PLEASURE"},
    {38, "AIS_TYPE_RESERVED_20"},
    {39, "AIS_TYPE_RESERVED_21"},
    {40, "AIS_TYPE_HSC"},
    {41, "AIS_TYPE_HSC_HAZARDOUS_A"},
    {42, "AIS_TYPE_HSC_HAZARDOUS_B"},
    {43, "AIS_TYPE_HSC_HAZARDOUS_C"},
    {44, "AIS_TYPE_HSC_HAZARDOUS_D"},
    {45, "AIS_TYPE_HSC_RESERVED_1"},
    {46, "AIS_TYPE_HSC_RESERVED_2"},
    {47, "AIS_TYPE_HSC_RESERVED_3"},
    {48, "AIS_TYPE_HSC_RESERVED_4"},
    {49, "AIS_TYPE_HSC_UNKNOWN"},
    {50, "AIS_TYPE_PILOT"},
    {51, "AIS_TYPE_SAR"},
    {52, "AIS_TYPE_TUG"},
    {53, "AIS_TYPE_PORT_TENDER"},
    {54, "AIS_TYPE_ANTI_POLLUTION"},
    {55, "AIS_TYPE_LAW_ENFORCEMENT"},
    {56, "AIS_TYPE_SPARE_LOCAL_1"},
    {57, "AIS_TYPE_SPARE_LOCAL_2"},
    {58, "AIS_TYPE_MEDICAL_TRANSPORT"},
    {59, "AIS_TYPE_NONECOMBATANT"},
    {60, "AIS_TYPE_PASSENGER"},
    {61, "AIS_TYPE_PASSENGER_HAZARDOUS_A"},
    {62, "AIS_TYPE_PASSENGER_HAZARDOUS_B"},
    {63, "AIS_TYPE_AIS_TYPE_PASSENGER_HAZARDOUS_C"},
    {64, "AIS_TYPE_PASSENGER_HAZARDOUS_D"},
    {65, "AIS_TYPE_PASSENGER_RESERVED_1"},
    {66, "AIS_TYPE_PASSENGER_RESERVED_2"},
    {67, "AIS_TYPE_PASSENGER_RESERVED_3"},
    {68, "AIS_TYPE_AIS_TYPE_PASSENGER_RESERVED_4"},
    {69, "AIS_TYPE_PASSENGER_UNKNOWN"},
    {70, "AIS_TYPE_CARGO"},
    {71, "AIS_TYPE_CARGO_HAZARDOUS_A"},
    {72, "AIS_TYPE_CARGO_HAZARDOUS_B"},
    {73, "AIS_TYPE_CARGO_HAZARDOUS_C"},
    {74, "AIS_TYPE_CARGO_HAZARDOUS_D"},
    {75, "AIS_TYPE_CARGO_RESERVED_1"},
    {76, "AIS_TYPE_CARGO_RESERVED_2"},
    {77, "AIS_TYPE_CARGO_RESERVED_3"},
    {78, "AIS_TYPE_CARGO_RESERVED_4"},
    {79, "AIS_TYPE_CARGO_UNKNOWN"},
    {80, "AIS_TYPE_TANKER"},
    {81, "AIS_TYPE_TANKER_HAZARDOUS_A"},
    {82, "AIS_TYPE_TANKER_HAZARDOUS_B"},
    {83, "AIS_TYPE_TANKER_HAZARDOUS_C"},
    {84, "AIS_TYPE_TANKER_HAZARDOUS_D"},
    {85, "AIS_TYPE_TANKER_RESERVED_1"},
    {86, "AIS_TYPE_TANKER_RESERVED_2"},
    {87, "AIS_TYPE_TANKER_RESERVED_3"},
    {88, "AIS_TYPE_TANKER_RESERVED_4"},
    {89, "AIS_TYPE_TANKER_UNKNOWN"},
    {90, "AIS_TYPE_OTHER"},
    {91, "AIS_TYPE_OTHER_HAZARDOUS_A"},
    {92, "AIS_TYPE_OTHER_HAZARDOUS_B"},
    {93, "AIS_TYPE_OTHER_HAZARDOUS_C"},
    {94, "AIS_TYPE_OTHER_HAZARDOUS_D"},
    {95, "AIS_TYPE_OTHER_RESERVED_1"},
    {96, "AIS_TYPE_OTHER_RESERVED_2"},
    {97, "AIS_TYPE_OTHER_RESERVED_3"},
    {98, "AIS_TYPE_OTHER_RESERVED_4"},
    {99, "AIS_TYPE_OTHER_UNKNOWN"},
};

inline const char* mavlink_ais_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_AIS_TYPE_LOOKUP);
}

inline std::string mavlink_ais_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_AIS_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_ATTITUDE_TARGET_TYPEMASK_LOOKUP = {
    {1, "ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE"},
    {2, "ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE"},
    {4, "ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE"},
    {64, "ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE"},
    {128, "ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE"},
};

inline const char* mavlink_attitude_target_typemask_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_ATTITUDE_TARGET_TYPEMASK_LOOKUP);
}

inline std::string mavlink_attitude_target_typemask_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_ATTITUDE_TARGET_TYPEMASK_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_CAMERA_CAP_FLAGS_LOOKUP = {
    {1, "CAMERA_CAP_FLAGS_CAPTURE_VIDEO"},
    {2, "CAMERA_CAP_FLAGS_CAPTURE_IMAGE"},
    {4, "CAMERA_CAP_FLAGS_HAS_MODES"},
    {8, "CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE"},
    {16, "CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE"},
    {32, "CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE"},
    {64, "CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM"},
    {128, "CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS"},
    {256, "CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM"},
    {512, "CAMERA_CAP_FLAGS_HAS_TRACKING_POINT"},
    {1024, "CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE"},
    {2048, "CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS"},
};

inline const char* mavlink_camera_cap_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_CAMERA_CAP_FLAGS_LOOKUP);
}

inline std::string mavlink_camera_cap_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_CAMERA_CAP_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_CAMERA_MODE_LOOKUP = {
    {0, "CAMERA_MODE_IMAGE"},
    {1, "CAMERA_MODE_VIDEO"},
    {2, "CAMERA_MODE_IMAGE_SURVEY"},
};

inline const char* mavlink_camera_mode_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_CAMERA_MODE_LOOKUP);
}

inline std::string mavlink_camera_mode_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_CAMERA_MODE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_CAMERA_TRACKING_MODE_LOOKUP = {
    {0, "CAMERA_TRACKING_NONE"},
    {1, "CAMERA_TRACKING_POINT"},
    {2, "CAMERA_TRACKING_RECTANGLE"},
};

inline const char* mavlink_camera_tracking_mode_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_CAMERA_TRACKING_MODE_LOOKUP);
}

inline std::string mavlink_camera_tracking_mode_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_CAMERA_TRACKING_MODE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_CAMERA_TRACKING_STATUS_FLAGS_LOOKUP = {
    {0, "CAMERA_TRACKING_STATUS_FLAGS_IDLE"},
    {1, "CAMERA_TRACKING_STATUS_FLAGS_ACTIVE"},
    {2, "CAMERA_TRACKING_STATUS_FLAGS_ERROR"},
};

inline const char* mavlink_camera_tracking_status_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_CAMERA_TRACKING_STATUS_FLAGS_LOOKUP);
}

inline std::string mavlink_camera_tracking_status_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_CAMERA_TRACKING_STATUS_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_CAMERA_TRACKING_TARGET_DATA_LOOKUP = {
    {0, "CAMERA_TRACKING_TARGET_NONE"},
    {1, "CAMERA_TRACKING_TARGET_EMBEDDED"},
    {2, "CAMERA_TRACKING_TARGET_RENDERED"},
    {4, "CAMERA_TRACKING_TARGET_IN_STATUS"},
};

inline const char* mavlink_camera_tracking_target_data_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_CAMERA_TRACKING_TARGET_DATA_LOOKUP);
}

inline std::string mavlink_camera_tracking_target_data_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_CAMERA_TRACKING_TARGET_DATA_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_CAMERA_ZOOM_TYPE_LOOKUP = {
    {0, "ZOOM_TYPE_STEP"},
    {1, "ZOOM_TYPE_CONTINUOUS"},
    {2, "ZOOM_TYPE_RANGE"},
    {3, "ZOOM_TYPE_FOCAL_LENGTH"},
};

inline const char* mavlink_camera_zoom_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_CAMERA_ZOOM_TYPE_LOOKUP);
}

inline std::string mavlink_camera_zoom_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_CAMERA_ZOOM_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_CELLULAR_CONFIG_RESPONSE_LOOKUP = {
    {0, "CELLULAR_CONFIG_RESPONSE_ACCEPTED"},
    {1, "CELLULAR_CONFIG_RESPONSE_APN_ERROR"},
    {2, "CELLULAR_CONFIG_RESPONSE_PIN_ERROR"},
    {3, "CELLULAR_CONFIG_RESPONSE_REJECTED"},
    {4, "CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED"},
};

inline const char* mavlink_cellular_config_response_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_CELLULAR_CONFIG_RESPONSE_LOOKUP);
}

inline std::string mavlink_cellular_config_response_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_CELLULAR_CONFIG_RESPONSE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_CELLULAR_NETWORK_FAILED_REASON_LOOKUP = {
    {0, "CELLULAR_NETWORK_FAILED_REASON_NONE"},
    {1, "CELLULAR_NETWORK_FAILED_REASON_UNKNOWN"},
    {2, "CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING"},
    {3, "CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR"},
};

inline const char* mavlink_cellular_network_failed_reason_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_CELLULAR_NETWORK_FAILED_REASON_LOOKUP);
}

inline std::string mavlink_cellular_network_failed_reason_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_CELLULAR_NETWORK_FAILED_REASON_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_CELLULAR_NETWORK_RADIO_TYPE_LOOKUP = {
    {0, "CELLULAR_NETWORK_RADIO_TYPE_NONE"},
    {1, "CELLULAR_NETWORK_RADIO_TYPE_GSM"},
    {2, "CELLULAR_NETWORK_RADIO_TYPE_CDMA"},
    {3, "CELLULAR_NETWORK_RADIO_TYPE_WCDMA"},
    {4, "CELLULAR_NETWORK_RADIO_TYPE_LTE"},
};

inline const char* mavlink_cellular_network_radio_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_CELLULAR_NETWORK_RADIO_TYPE_LOOKUP);
}

inline std::string mavlink_cellular_network_radio_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_CELLULAR_NETWORK_RADIO_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_CELLULAR_STATUS_FLAG_LOOKUP = {
    {0, "CELLULAR_STATUS_FLAG_UNKNOWN"},
    {1, "CELLULAR_STATUS_FLAG_FAILED"},
    {2, "CELLULAR_STATUS_FLAG_INITIALIZING"},
    {3, "CELLULAR_STATUS_FLAG_LOCKED"},
    {4, "CELLULAR_STATUS_FLAG_DISABLED"},
    {5, "CELLULAR_STATUS_FLAG_DISABLING"},
    {6, "CELLULAR_STATUS_FLAG_ENABLING"},
    {7, "CELLULAR_STATUS_FLAG_ENABLED"},
    {8, "CELLULAR_STATUS_FLAG_SEARCHING"},
    {9, "CELLULAR_STATUS_FLAG_REGISTERED"},
    {10, "CELLULAR_STATUS_FLAG_DISCONNECTING"},
    {11, "CELLULAR_STATUS_FLAG_CONNECTING"},
    {12, "CELLULAR_STATUS_FLAG_CONNECTED"},
};

inline const char* mavlink_cellular_status_flag_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_CELLULAR_STATUS_FLAG_LOOKUP);
}

inline std::string mavlink_cellular_status_flag_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_CELLULAR_STATUS_FLAG_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_COMPONENT_CAP_FLAGS_LOOKUP = {
    {1, "COMPONENT_CAP_FLAGS_PARAM"},
    {2, "COMPONENT_CAP_FLAGS_PARAM_EXT"},
};

inline const char* mavlink_component_cap_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_COMPONENT_CAP_FLAGS_LOOKUP);
}

inline std::string mavlink_component_cap_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_COMPONENT_CAP_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_COMP_METADATA_TYPE_LOOKUP = {
    {0, "COMP_METADATA_TYPE_VERSION"},
    {1, "COMP_METADATA_TYPE_PARAMETER"},
    {2, "COMP_METADATA_TYPE_COMMANDS"},
};

inline const char* mavlink_comp_metadata_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_COMP_METADATA_TYPE_LOOKUP);
}

inline std::string mavlink_comp_metadata_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_COMP_METADATA_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_ESC_CONNECTION_TYPE_LOOKUP = {
    {0, "ESC_CONNECTION_TYPE_PPM"},
    {1, "ESC_CONNECTION_TYPE_SERIAL"},
    {2, "ESC_CONNECTION_TYPE_ONESHOT"},
    {3, "ESC_CONNECTION_TYPE_I2C"},
    {4, "ESC_CONNECTION_TYPE_CAN"},
    {5, "ESC_CONNECTION_TYPE_DSHOT"},
};

inline const char* mavlink_esc_connection_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_ESC_CONNECTION_TYPE_LOOKUP);
}

inline std::string mavlink_esc_connection_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_ESC_CONNECTION_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_ESC_FAILURE_FLAGS_LOOKUP = {
    {0, "ESC_FAILURE_NONE"},
    {1, "ESC_FAILURE_OVER_CURRENT"},
    {2, "ESC_FAILURE_OVER_VOLTAGE"},
    {4, "ESC_FAILURE_OVER_TEMPERATURE"},
    {8, "ESC_FAILURE_OVER_RPM"},
    {16, "ESC_FAILURE_INCONSISTENT_CMD"},
    {32, "ESC_FAILURE_MOTOR_STUCK"},
    {64, "ESC_FAILURE_GENERIC"},
};

inline const char* mavlink_esc_failure_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_ESC_FAILURE_FLAGS_LOOKUP);
}

inline std::string mavlink_esc_failure_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_ESC_FAILURE_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_ESTIMATOR_STATUS_FLAGS_LOOKUP = {
    {1, "ESTIMATOR_ATTITUDE"},
    {2, "ESTIMATOR_VELOCITY_HORIZ"},
    {4, "ESTIMATOR_VELOCITY_VERT"},
    {8, "ESTIMATOR_POS_HORIZ_REL"},
    {16, "ESTIMATOR_POS_HORIZ_ABS"},
    {32, "ESTIMATOR_POS_VERT_ABS"},
    {64, "ESTIMATOR_POS_VERT_AGL"},
    {128, "ESTIMATOR_CONST_POS_MODE"},
    {256, "ESTIMATOR_PRED_POS_HORIZ_REL"},
    {512, "ESTIMATOR_PRED_POS_HORIZ_ABS"},
    {1024, "ESTIMATOR_GPS_GLITCH"},
    {2048, "ESTIMATOR_ACCEL_ERROR"},
};

inline const char* mavlink_estimator_status_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_ESTIMATOR_STATUS_FLAGS_LOOKUP);
}

inline std::string mavlink_estimator_status_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_ESTIMATOR_STATUS_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_FAILURE_TYPE_LOOKUP = {
    {0, "FAILURE_TYPE_OK"},
    {1, "FAILURE_TYPE_OFF"},
    {2, "FAILURE_TYPE_STUCK"},
    {3, "FAILURE_TYPE_GARBAGE"},
    {4, "FAILURE_TYPE_WRONG"},
    {5, "FAILURE_TYPE_SLOW"},
    {6, "FAILURE_TYPE_DELAYED"},
    {7, "FAILURE_TYPE_INTERMITTENT"},
};

inline const char* mavlink_failure_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_FAILURE_TYPE_LOOKUP);
}

inline std::string mavlink_failure_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_FAILURE_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_FAILURE_UNIT_LOOKUP = {
    {0, "FAILURE_UNIT_SENSOR_GYRO"},
    {1, "FAILURE_UNIT_SENSOR_ACCEL"},
    {2, "FAILURE_UNIT_SENSOR_MAG"},
    {3, "FAILURE_UNIT_SENSOR_BARO"},
    {4, "FAILURE_UNIT_SENSOR_GPS"},
    {5, "FAILURE_UNIT_SENSOR_OPTICAL_FLOW"},
    {6, "FAILURE_UNIT_SENSOR_VIO"},
    {7, "FAILURE_UNIT_SENSOR_DISTANCE_SENSOR"},
    {8, "FAILURE_UNIT_SENSOR_AIRSPEED"},
    {100, "FAILURE_UNIT_SYSTEM_BATTERY"},
    {101, "FAILURE_UNIT_SYSTEM_MOTOR"},
    {102, "FAILURE_UNIT_SYSTEM_SERVO"},
    {103, "FAILURE_UNIT_SYSTEM_AVOIDANCE"},
    {104, "FAILURE_UNIT_SYSTEM_RC_SIGNAL"},
    {105, "FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL"},
};

inline const char* mavlink_failure_unit_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_FAILURE_UNIT_LOOKUP);
}

inline std::string mavlink_failure_unit_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_FAILURE_UNIT_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_FENCE_ACTION_LOOKUP = {
    {0, "FENCE_ACTION_NONE"},
    {1, "FENCE_ACTION_GUIDED"},
    {2, "FENCE_ACTION_REPORT"},
    {3, "FENCE_ACTION_GUIDED_THR_PASS"},
    {4, "FENCE_ACTION_RTL"},
};

inline const char* mavlink_fence_action_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_FENCE_ACTION_LOOKUP);
}

inline std::string mavlink_fence_action_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_FENCE_ACTION_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_FENCE_BREACH_LOOKUP = {
    {0, "FENCE_BREACH_NONE"},
    {1, "FENCE_BREACH_MINALT"},
    {2, "FENCE_BREACH_MAXALT"},
    {3, "FENCE_BREACH_BOUNDARY"},
};

inline const char* mavlink_fence_breach_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_FENCE_BREACH_LOOKUP);
}

inline std::string mavlink_fence_breach_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_FENCE_BREACH_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_FENCE_MITIGATE_LOOKUP = {
    {0, "FENCE_MITIGATE_UNKNOWN"},
    {1, "FENCE_MITIGATE_NONE"},
    {2, "FENCE_MITIGATE_VEL_LIMIT"},
};

inline const char* mavlink_fence_mitigate_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_FENCE_MITIGATE_LOOKUP);
}

inline std::string mavlink_fence_mitigate_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_FENCE_MITIGATE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_FIRMWARE_VERSION_TYPE_LOOKUP = {
    {0, "FIRMWARE_VERSION_TYPE_DEV"},
    {64, "FIRMWARE_VERSION_TYPE_ALPHA"},
    {128, "FIRMWARE_VERSION_TYPE_BETA"},
    {192, "FIRMWARE_VERSION_TYPE_RC"},
    {255, "FIRMWARE_VERSION_TYPE_OFFICIAL"},
};

inline const char* mavlink_firmware_version_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_FIRMWARE_VERSION_TYPE_LOOKUP);
}

inline std::string mavlink_firmware_version_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_FIRMWARE_VERSION_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_GIMBAL_DEVICE_CAP_FLAGS_LOOKUP = {
    {1, "GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT"},
    {2, "GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL"},
    {4, "GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS"},
    {8, "GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW"},
    {16, "GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK"},
    {32, "GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS"},
    {64, "GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW"},
    {128, "GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK"},
    {256, "GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS"},
    {512, "GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW"},
    {1024, "GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK"},
    {2048, "GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW"},
};

inline const char* mavlink_gimbal_device_cap_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_GIMBAL_DEVICE_CAP_FLAGS_LOOKUP);
}

inline std::string mavlink_gimbal_device_cap_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_GIMBAL_DEVICE_CAP_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_GIMBAL_DEVICE_ERROR_FLAGS_LOOKUP = {
    {1, "GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT"},
    {2, "GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT"},
    {4, "GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT"},
    {8, "GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR"},
    {16, "GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR"},
    {32, "GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR"},
    {64, "GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR"},
    {128, "GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR"},
    {256, "GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING"},
};

inline const char* mavlink_gimbal_device_error_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_GIMBAL_DEVICE_ERROR_FLAGS_LOOKUP);
}

inline std::string mavlink_gimbal_device_error_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_GIMBAL_DEVICE_ERROR_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_GIMBAL_DEVICE_FLAGS_LOOKUP = {
    {1, "GIMBAL_DEVICE_FLAGS_RETRACT"},
    {2, "GIMBAL_DEVICE_FLAGS_NEUTRAL"},
    {4, "GIMBAL_DEVICE_FLAGS_ROLL_LOCK"},
    {8, "GIMBAL_DEVICE_FLAGS_PITCH_LOCK"},
    {16, "GIMBAL_DEVICE_FLAGS_YAW_LOCK"},
};

inline const char* mavlink_gimbal_device_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_GIMBAL_DEVICE_FLAGS_LOOKUP);
}

inline std::string mavlink_gimbal_device_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_GIMBAL_DEVICE_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_GIMBAL_MANAGER_CAP_FLAGS_LOOKUP = {
    {1, "GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT"},
    {2, "GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL"},
    {4, "GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS"},
    {8, "GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW"},
    {16, "GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK"},
    {32, "GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS"},
    {64, "GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW"},
    {128, "GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK"},
    {256, "GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS"},
    {512, "GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW"},
    {1024, "GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK"},
    {2048, "GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW"},
    {65536, "GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL"},
    {131072, "GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL"},
};

inline const char* mavlink_gimbal_manager_cap_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_GIMBAL_MANAGER_CAP_FLAGS_LOOKUP);
}

inline std::string mavlink_gimbal_manager_cap_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_GIMBAL_MANAGER_CAP_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_GIMBAL_MANAGER_FLAGS_LOOKUP = {
    {1, "GIMBAL_MANAGER_FLAGS_RETRACT"},
    {2, "GIMBAL_MANAGER_FLAGS_NEUTRAL"},
    {4, "GIMBAL_MANAGER_FLAGS_ROLL_LOCK"},
    {8, "GIMBAL_MANAGER_FLAGS_PITCH_LOCK"},
    {16, "GIMBAL_MANAGER_FLAGS_YAW_LOCK"},
};

inline const char* mavlink_gimbal_manager_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_GIMBAL_MANAGER_FLAGS_LOOKUP);
}

inline std::string mavlink_gimbal_manager_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_GIMBAL_MANAGER_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_GPS_FIX_TYPE_LOOKUP = {
    {0, "GPS_FIX_TYPE_NO_GPS"},
    {1, "GPS_FIX_TYPE_NO_FIX"},
    {2, "GPS_FIX_TYPE_2D_FIX"},
    {3, "GPS_FIX_TYPE_3D_FIX"},
    {4, "GPS_FIX_TYPE_DGPS"},
    {5, "GPS_FIX_TYPE_RTK_FLOAT"},
    {6, "GPS_FIX_TYPE_RTK_FIXED"},
    {7, "GPS_FIX_TYPE_STATIC"},
    {8, "GPS_FIX_TYPE_PPP"},
};

inline const char* mavlink_gps_fix_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_GPS_FIX_TYPE_LOOKUP);
}

inline std::string mavlink_gps_fix_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_GPS_FIX_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_GPS_INPUT_IGNORE_FLAGS_LOOKUP = {
    {1, "GPS_INPUT_IGNORE_FLAG_ALT"},
    {2, "GPS_INPUT_IGNORE_FLAG_HDOP"},
    {4, "GPS_INPUT_IGNORE_FLAG_VDOP"},
    {8, "GPS_INPUT_IGNORE_FLAG_VEL_HORIZ"},
    {16, "GPS_INPUT_IGNORE_FLAG_VEL_VERT"},
    {32, "GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY"},
    {64, "GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY"},
    {128, "GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY"},
};

inline const char* mavlink_gps_input_ignore_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_GPS_INPUT_IGNORE_FLAGS_LOOKUP);
}

inline std::string mavlink_gps_input_ignore_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_GPS_INPUT_IGNORE_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_GRIPPER_ACTIONS_LOOKUP = {
    {0, "GRIPPER_ACTION_RELEASE"},
    {1, "GRIPPER_ACTION_GRAB"},
};

inline const char* mavlink_gripper_actions_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_GRIPPER_ACTIONS_LOOKUP);
}

inline std::string mavlink_gripper_actions_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_GRIPPER_ACTIONS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_HL_FAILURE_FLAG_LOOKUP = {
    {1, "HL_FAILURE_FLAG_GPS"},
    {2, "HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE"},
    {4, "HL_FAILURE_FLAG_ABSOLUTE_PRESSURE"},
    {8, "HL_FAILURE_FLAG_3D_ACCEL"},
    {16, "HL_FAILURE_FLAG_3D_GYRO"},
    {32, "HL_FAILURE_FLAG_3D_MAG"},
    {64, "HL_FAILURE_FLAG_TERRAIN"},
    {128, "HL_FAILURE_FLAG_BATTERY"},
    {256, "HL_FAILURE_FLAG_RC_RECEIVER"},
    {512, "HL_FAILURE_FLAG_OFFBOARD_LINK"},
    {1024, "HL_FAILURE_FLAG_ENGINE"},
    {2048, "HL_FAILURE_FLAG_GEOFENCE"},
    {4096, "HL_FAILURE_FLAG_ESTIMATOR"},
    {8192, "HL_FAILURE_FLAG_MISSION"},
};

inline const char* mavlink_hl_failure_flag_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_HL_FAILURE_FLAG_LOOKUP);
}

inline std::string mavlink_hl_failure_flag_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_HL_FAILURE_FLAG_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_LANDING_TARGET_TYPE_LOOKUP = {
    {0, "LANDING_TARGET_TYPE_LIGHT_BEACON"},
    {1, "LANDING_TARGET_TYPE_RADIO_BEACON"},
    {2, "LANDING_TARGET_TYPE_VISION_FIDUCIAL"},
    {3, "LANDING_TARGET_TYPE_VISION_OTHER"},
};

inline const char* mavlink_landing_target_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_LANDING_TARGET_TYPE_LOOKUP);
}

inline std::string mavlink_landing_target_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_LANDING_TARGET_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAGOTHY_CAPABILITY_LOOKUP = {
    {1, "MAGOTHY_CAPABILITY_LOG_MANAGEMENT"},
    {2, "MAGOTHY_CAPABILITY_FIRMWARE_UPDATE"},
    {4, "MAGOTHY_CAPABILITY_GYRO_CAL"},
    {8, "MAGOTHY_CAPABILITY_MAG_CAL_2D"},
    {16, "MAGOTHY_CAPABILITY_MAG_CAL_3D"},
    {32, "MAGOTHY_CAPABILITY_SPY"},
};

inline const char* mavlink_magothy_capability_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAGOTHY_CAPABILITY_LOOKUP);
}

inline std::string mavlink_magothy_capability_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAGOTHY_CAPABILITY_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAGOTHY_FAULT_RESPONSE_LOOKUP = {
    {0, "MAGOTHY_FAULT_RESPONSE_IGNORE"},
    {1, "MAGOTHY_FAULT_RESPONSE_DRIFT"},
    {2, "MAGOTHY_FAULT_RESPONSE_LOITER"},
    {3, "MAGOTHY_FAULT_RESPONSE_RALLY"},
    {4, "MAGOTHY_FAULT_RESPONSE_FIRST"},
    {5, "MAGOTHY_FAULT_RESPONSE_FINAL"},
    {6, "MAGOTHY_FAULT_RESPONSE_LAUNCH"},
    {7, "MAGOTHY_FAULT_RESPONSE_CUSTOM"},
};

inline const char* mavlink_magothy_fault_response_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAGOTHY_FAULT_RESPONSE_LOOKUP);
}

inline std::string mavlink_magothy_fault_response_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAGOTHY_FAULT_RESPONSE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAGOTHY_LICENSE_TRANSFER_TYPE_LOOKUP = {
    {0, "MAGOTHY_LICENSE_TRANSFER_TYPE_ACTIVATION"},
    {1, "MAGOTHY_LICENSE_TRANSFER_TYPE_LICENSE_FILE"},
    {2, "MAGOTHY_LICENSE_TRANSFER_TYPE_DEACTIVATION"},
    {3, "MAGOTHY_LICENSE_TRANSFER_TYPE_DEACTIVATION_PERMISSION"},
};

inline const char* mavlink_magothy_license_transfer_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAGOTHY_LICENSE_TRANSFER_TYPE_LOOKUP);
}

inline std::string mavlink_magothy_license_transfer_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAGOTHY_LICENSE_TRANSFER_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAG_CAL_STATUS_LOOKUP = {
    {0, "MAG_CAL_NOT_STARTED"},
    {1, "MAG_CAL_WAITING_TO_START"},
    {2, "MAG_CAL_RUNNING_STEP_ONE"},
    {3, "MAG_CAL_RUNNING_STEP_TWO"},
    {4, "MAG_CAL_SUCCESS"},
    {5, "MAG_CAL_FAILED"},
    {6, "MAG_CAL_BAD_ORIENTATION"},
    {7, "MAG_CAL_BAD_RADIUS"},
};

inline const char* mavlink_mag_cal_status_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAG_CAL_STATUS_LOOKUP);
}

inline std::string mavlink_mag_cal_status_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAG_CAL_STATUS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAVLINK_DATA_STREAM_TYPE_LOOKUP = {
    {0, "MAVLINK_DATA_STREAM_IMG_JPEG"},
    {1, "MAVLINK_DATA_STREAM_IMG_BMP"},
    {2, "MAVLINK_DATA_STREAM_IMG_RAW8U"},
    {3, "MAVLINK_DATA_STREAM_IMG_RAW32U"},
    {4, "MAVLINK_DATA_STREAM_IMG_PGM"},
    {5, "MAVLINK_DATA_STREAM_IMG_PNG"},
};

inline const char* mavlink_mavlink_data_stream_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAVLINK_DATA_STREAM_TYPE_LOOKUP);
}

inline std::string mavlink_mavlink_data_stream_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAVLINK_DATA_STREAM_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ARM_AUTH_DENIED_REASON_LOOKUP = {
    {0, "MAV_ARM_AUTH_DENIED_REASON_GENERIC"},
    {1, "MAV_ARM_AUTH_DENIED_REASON_NONE"},
    {2, "MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT"},
    {3, "MAV_ARM_AUTH_DENIED_REASON_TIMEOUT"},
    {4, "MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE"},
    {5, "MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER"},
};

inline const char* mavlink_mav_arm_auth_denied_reason_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ARM_AUTH_DENIED_REASON_LOOKUP);
}

inline std::string mavlink_mav_arm_auth_denied_reason_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ARM_AUTH_DENIED_REASON_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_AUTOPILOT_LOOKUP = {
    {0, "MAV_AUTOPILOT_GENERIC"},
    {1, "MAV_AUTOPILOT_RESERVED"},
    {2, "MAV_AUTOPILOT_SLUGS"},
    {3, "MAV_AUTOPILOT_ARDUPILOTMEGA"},
    {4, "MAV_AUTOPILOT_OPENPILOT"},
    {5, "MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY"},
    {6, "MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY"},
    {7, "MAV_AUTOPILOT_GENERIC_MISSION_FULL"},
    {8, "MAV_AUTOPILOT_INVALID"},
    {9, "MAV_AUTOPILOT_PPZ"},
    {10, "MAV_AUTOPILOT_UDB"},
    {11, "MAV_AUTOPILOT_FP"},
    {12, "MAV_AUTOPILOT_PX4"},
    {13, "MAV_AUTOPILOT_SMACCMPILOT"},
    {14, "MAV_AUTOPILOT_AUTOQUAD"},
    {15, "MAV_AUTOPILOT_ARMAZILA"},
    {16, "MAV_AUTOPILOT_AEROB"},
    {17, "MAV_AUTOPILOT_ASLUAV"},
    {18, "MAV_AUTOPILOT_SMARTAP"},
    {19, "MAV_AUTOPILOT_AIRRAILS"},
};

inline const char* mavlink_mav_autopilot_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_AUTOPILOT_LOOKUP);
}

inline std::string mavlink_mav_autopilot_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_AUTOPILOT_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_BATTERY_CHARGE_STATE_LOOKUP = {
    {0, "MAV_BATTERY_CHARGE_STATE_UNDEFINED"},
    {1, "MAV_BATTERY_CHARGE_STATE_OK"},
    {2, "MAV_BATTERY_CHARGE_STATE_LOW"},
    {3, "MAV_BATTERY_CHARGE_STATE_CRITICAL"},
    {4, "MAV_BATTERY_CHARGE_STATE_EMERGENCY"},
    {5, "MAV_BATTERY_CHARGE_STATE_FAILED"},
    {6, "MAV_BATTERY_CHARGE_STATE_UNHEALTHY"},
    {7, "MAV_BATTERY_CHARGE_STATE_CHARGING"},
};

inline const char* mavlink_mav_battery_charge_state_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_BATTERY_CHARGE_STATE_LOOKUP);
}

inline std::string mavlink_mav_battery_charge_state_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_BATTERY_CHARGE_STATE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_BATTERY_FAULT_LOOKUP = {
    {1, "MAV_BATTERY_FAULT_DEEP_DISCHARGE"},
    {2, "MAV_BATTERY_FAULT_SPIKES"},
    {4, "MAV_BATTERY_FAULT_CELL_FAIL"},
    {8, "MAV_BATTERY_FAULT_OVER_CURRENT"},
    {16, "MAV_BATTERY_FAULT_OVER_TEMPERATURE"},
    {32, "MAV_BATTERY_FAULT_UNDER_TEMPERATURE"},
    {64, "MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE"},
};

inline const char* mavlink_mav_battery_fault_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_BATTERY_FAULT_LOOKUP);
}

inline std::string mavlink_mav_battery_fault_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_BATTERY_FAULT_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_BATTERY_FUNCTION_LOOKUP = {
    {0, "MAV_BATTERY_FUNCTION_UNKNOWN"},
    {1, "MAV_BATTERY_FUNCTION_ALL"},
    {2, "MAV_BATTERY_FUNCTION_PROPULSION"},
    {3, "MAV_BATTERY_FUNCTION_AVIONICS"},
    {4, "MAV_BATTERY_TYPE_PAYLOAD"},
};

inline const char* mavlink_mav_battery_function_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_BATTERY_FUNCTION_LOOKUP);
}

inline std::string mavlink_mav_battery_function_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_BATTERY_FUNCTION_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_BATTERY_MODE_LOOKUP = {
    {0, "MAV_BATTERY_MODE_UNKNOWN"},
    {1, "MAV_BATTERY_MODE_AUTO_DISCHARGING"},
    {2, "MAV_BATTERY_MODE_HOT_SWAP"},
};

inline const char* mavlink_mav_battery_mode_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_BATTERY_MODE_LOOKUP);
}

inline std::string mavlink_mav_battery_mode_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_BATTERY_MODE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_BATTERY_TYPE_LOOKUP = {
    {0, "MAV_BATTERY_TYPE_UNKNOWN"},
    {1, "MAV_BATTERY_TYPE_LIPO"},
    {2, "MAV_BATTERY_TYPE_LIFE"},
    {3, "MAV_BATTERY_TYPE_LION"},
    {4, "MAV_BATTERY_TYPE_NIMH"},
};

inline const char* mavlink_mav_battery_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_BATTERY_TYPE_LOOKUP);
}

inline std::string mavlink_mav_battery_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_BATTERY_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_CMD_LOOKUP = {
    {16, "MAV_CMD_NAV_WAYPOINT"},
    {17, "MAV_CMD_NAV_LOITER_UNLIM"},
    {18, "MAV_CMD_NAV_LOITER_TURNS"},
    {19, "MAV_CMD_NAV_LOITER_TIME"},
    {20, "MAV_CMD_NAV_RETURN_TO_LAUNCH"},
    {21, "MAV_CMD_NAV_LAND"},
    {22, "MAV_CMD_NAV_TAKEOFF"},
    {23, "MAV_CMD_NAV_LAND_LOCAL"},
    {24, "MAV_CMD_NAV_TAKEOFF_LOCAL"},
    {25, "MAV_CMD_NAV_FOLLOW"},
    {30, "MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT"},
    {31, "MAV_CMD_NAV_LOITER_TO_ALT"},
    {32, "MAV_CMD_DO_FOLLOW"},
    {33, "MAV_CMD_DO_FOLLOW_REPOSITION"},
    {34, "MAV_CMD_DO_ORBIT"},
    {80, "MAV_CMD_NAV_ROI"},
    {81, "MAV_CMD_NAV_PATHPLANNING"},
    {82, "MAV_CMD_NAV_SPLINE_WAYPOINT"},
    {84, "MAV_CMD_NAV_VTOL_TAKEOFF"},
    {85, "MAV_CMD_NAV_VTOL_LAND"},
    {92, "MAV_CMD_NAV_GUIDED_ENABLE"},
    {93, "MAV_CMD_NAV_DELAY"},
    {94, "MAV_CMD_NAV_PAYLOAD_PLACE"},
    {95, "MAV_CMD_NAV_LAST"},
    {112, "MAV_CMD_CONDITION_DELAY"},
    {113, "MAV_CMD_CONDITION_CHANGE_ALT"},
    {114, "MAV_CMD_CONDITION_DISTANCE"},
    {115, "MAV_CMD_CONDITION_YAW"},
    {159, "MAV_CMD_CONDITION_LAST"},
    {176, "MAV_CMD_DO_SET_MODE"},
    {177, "MAV_CMD_DO_JUMP"},
    {178, "MAV_CMD_DO_CHANGE_SPEED"},
    {179, "MAV_CMD_DO_SET_HOME"},
    {180, "MAV_CMD_DO_SET_PARAMETER"},
    {181, "MAV_CMD_DO_SET_RELAY"},
    {182, "MAV_CMD_DO_REPEAT_RELAY"},
    {183, "MAV_CMD_DO_SET_SERVO"},
    {184, "MAV_CMD_DO_REPEAT_SERVO"},
    {185, "MAV_CMD_DO_FLIGHTTERMINATION"},
    {186, "MAV_CMD_DO_CHANGE_ALTITUDE"},
    {187, "MAV_CMD_DO_SET_ACTUATOR"},
    {189, "MAV_CMD_DO_LAND_START"},
    {190, "MAV_CMD_DO_RALLY_LAND"},
    {191, "MAV_CMD_DO_GO_AROUND"},
    {192, "MAV_CMD_DO_REPOSITION"},
    {193, "MAV_CMD_DO_PAUSE_CONTINUE"},
    {194, "MAV_CMD_DO_SET_REVERSE"},
    {195, "MAV_CMD_DO_SET_ROI_LOCATION"},
    {196, "MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET"},
    {197, "MAV_CMD_DO_SET_ROI_NONE"},
    {198, "MAV_CMD_DO_SET_ROI_SYSID"},
    {200, "MAV_CMD_DO_CONTROL_VIDEO"},
    {201, "MAV_CMD_DO_SET_ROI"},
    {202, "MAV_CMD_DO_DIGICAM_CONFIGURE"},
    {203, "MAV_CMD_DO_DIGICAM_CONTROL"},
    {204, "MAV_CMD_DO_MOUNT_CONFIGURE"},
    {205, "MAV_CMD_DO_MOUNT_CONTROL"},
    {206, "MAV_CMD_DO_SET_CAM_TRIGG_DIST"},
    {207, "MAV_CMD_DO_FENCE_ENABLE"},
    {208, "MAV_CMD_DO_PARACHUTE"},
    {209, "MAV_CMD_DO_MOTOR_TEST"},
    {210, "MAV_CMD_DO_INVERTED_FLIGHT"},
    {211, "MAV_CMD_DO_GRIPPER"},
    {212, "MAV_CMD_DO_AUTOTUNE_ENABLE"},
    {213, "MAV_CMD_NAV_SET_YAW_SPEED"},
    {214, "MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL"},
    {220, "MAV_CMD_DO_MOUNT_CONTROL_QUAT"},
    {221, "MAV_CMD_DO_GUIDED_MASTER"},
    {222, "MAV_CMD_DO_GUIDED_LIMITS"},
    {223, "MAV_CMD_DO_ENGINE_CONTROL"},
    {224, "MAV_CMD_DO_SET_MISSION_CURRENT"},
    {240, "MAV_CMD_DO_LAST"},
    {241, "MAV_CMD_PREFLIGHT_CALIBRATION"},
    {242, "MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS"},
    {243, "MAV_CMD_PREFLIGHT_UAVCAN"},
    {245, "MAV_CMD_PREFLIGHT_STORAGE"},
    {246, "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN"},
    {247, "MAV_CMD_DO_UPGRADE"},
    {252, "MAV_CMD_OVERRIDE_GOTO"},
    {260, "MAV_CMD_OBLIQUE_SURVEY"},
    {300, "MAV_CMD_MISSION_START"},
    {400, "MAV_CMD_COMPONENT_ARM_DISARM"},
    {405, "MAV_CMD_ILLUMINATOR_ON_OFF"},
    {410, "MAV_CMD_GET_HOME_POSITION"},
    {420, "MAV_CMD_INJECT_FAILURE"},
    {500, "MAV_CMD_START_RX_PAIR"},
    {510, "MAV_CMD_GET_MESSAGE_INTERVAL"},
    {511, "MAV_CMD_SET_MESSAGE_INTERVAL"},
    {512, "MAV_CMD_REQUEST_MESSAGE"},
    {519, "MAV_CMD_REQUEST_PROTOCOL_VERSION"},
    {520, "MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES"},
    {521, "MAV_CMD_REQUEST_CAMERA_INFORMATION"},
    {522, "MAV_CMD_REQUEST_CAMERA_SETTINGS"},
    {525, "MAV_CMD_REQUEST_STORAGE_INFORMATION"},
    {526, "MAV_CMD_STORAGE_FORMAT"},
    {527, "MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS"},
    {528, "MAV_CMD_REQUEST_FLIGHT_INFORMATION"},
    {529, "MAV_CMD_RESET_CAMERA_SETTINGS"},
    {530, "MAV_CMD_SET_CAMERA_MODE"},
    {531, "MAV_CMD_SET_CAMERA_ZOOM"},
    {532, "MAV_CMD_SET_CAMERA_FOCUS"},
    {600, "MAV_CMD_JUMP_TAG"},
    {601, "MAV_CMD_DO_JUMP_TAG"},
    {900, "MAV_CMD_PARAM_TRANSACTION"},
    {1000, "MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW"},
    {1001, "MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE"},
    {2000, "MAV_CMD_IMAGE_START_CAPTURE"},
    {2001, "MAV_CMD_IMAGE_STOP_CAPTURE"},
    {2002, "MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE"},
    {2003, "MAV_CMD_DO_TRIGGER_CONTROL"},
    {2004, "MAV_CMD_CAMERA_TRACK_POINT"},
    {2005, "MAV_CMD_CAMERA_TRACK_RECTANGLE"},
    {2010, "MAV_CMD_CAMERA_STOP_TRACKING"},
    {2500, "MAV_CMD_VIDEO_START_CAPTURE"},
    {2501, "MAV_CMD_VIDEO_STOP_CAPTURE"},
    {2502, "MAV_CMD_VIDEO_START_STREAMING"},
    {2503, "MAV_CMD_VIDEO_STOP_STREAMING"},
    {2504, "MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION"},
    {2505, "MAV_CMD_REQUEST_VIDEO_STREAM_STATUS"},
    {2510, "MAV_CMD_LOGGING_START"},
    {2511, "MAV_CMD_LOGGING_STOP"},
    {2520, "MAV_CMD_AIRFRAME_CONFIGURATION"},
    {2600, "MAV_CMD_CONTROL_HIGH_LATENCY"},
    {2800, "MAV_CMD_PANORAMA_CREATE"},
    {3000, "MAV_CMD_DO_VTOL_TRANSITION"},
    {3001, "MAV_CMD_ARM_AUTHORIZATION_REQUEST"},
    {4000, "MAV_CMD_SET_GUIDED_SUBMODE_STANDARD"},
    {4001, "MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE"},
    {4501, "MAV_CMD_CONDITION_GATE"},
    {5000, "MAV_CMD_NAV_FENCE_RETURN_POINT"},
    {5001, "MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION"},
    {5002, "MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION"},
    {5003, "MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION"},
    {5004, "MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION"},
    {5100, "MAV_CMD_NAV_RALLY_POINT"},
    {5200, "MAV_CMD_UAVCAN_GET_NODE_INFO"},
    {30001, "MAV_CMD_PAYLOAD_PREPARE_DEPLOY"},
    {30002, "MAV_CMD_PAYLOAD_CONTROL_DEPLOY"},
    {31000, "MAV_CMD_WAYPOINT_USER_1"},
    {31001, "MAV_CMD_WAYPOINT_USER_2"},
    {31002, "MAV_CMD_WAYPOINT_USER_3"},
    {31003, "MAV_CMD_WAYPOINT_USER_4"},
    {31004, "MAV_CMD_WAYPOINT_USER_5"},
    {31005, "MAV_CMD_SPATIAL_USER_1"},
    {31006, "MAV_CMD_SPATIAL_USER_2"},
    {31007, "MAV_CMD_SPATIAL_USER_3"},
    {31008, "MAV_CMD_SPATIAL_USER_4"},
    {31009, "MAV_CMD_SPATIAL_USER_5"},
    {31010, "MAV_CMD_USER_1"},
    {31011, "MAV_CMD_USER_2"},
    {31012, "MAV_CMD_USER_3"},
    {31013, "MAV_CMD_USER_4"},
    {31014, "MAV_CMD_USER_5"},
    {42006, "MAV_CMD_FIXED_MAG_CAL_YAW"},
    {42600, "MAV_CMD_DO_WINCH"},
    {50200, "MAV_CMD_DO_REBOOT"},
    {50201, "MAV_CMD_DO_START_FIRMWARE_UPDATE"},
    {50202, "MAV_CMD_DO_START_GYRO_CAL"},
    {50203, "MAV_CMD_DO_START_MAG_CAL_2D"},
    {50204, "MAV_CMD_DO_SET_FAULT_RESPONSE"},
    {50205, "MAV_CMD_DO_SET_FAULT_RESPONSE_PARAMS"},
    {50206, "MAV_CMD_DO_START_MAG_CAL_3D"},
    {50207, "MAV_CMD_DO_STOP_MAG_CAL_3D"},
    {50208, "MAV_CMD_DO_GENERIC_COMMAND"},
    {50209, "MAV_CMD_DO_ZIG_ZIG_COMMAND"},
    {50210, "MAV_CMD_DO_FILE_MISSION_COMMAND"},
    {50211, "MAV_CMD_DO_SET_FILE_MISSION_COMMAND"},
    {50300, "MAV_CMD_DO_MAGOTHY_CUSTOM_0"},
    {50301, "MAV_CMD_DO_MAGOTHY_CUSTOM_1"},
    {50302, "MAV_CMD_DO_MAGOTHY_CUSTOM_2"},
};

inline const char* mavlink_mav_cmd_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_CMD_LOOKUP);
}

inline std::string mavlink_mav_cmd_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_CMD_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_CMD_ACK_LOOKUP = {
    {0, "MAV_CMD_ACK_OK"},
    {1, "MAV_CMD_ACK_ERR_FAIL"},
    {2, "MAV_CMD_ACK_ERR_ACCESS_DENIED"},
    {3, "MAV_CMD_ACK_ERR_NOT_SUPPORTED"},
    {4, "MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED"},
    {5, "MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE"},
    {6, "MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE"},
    {7, "MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE"},
    {8, "MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE"},
};

inline const char* mavlink_mav_cmd_ack_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_CMD_ACK_LOOKUP);
}

inline std::string mavlink_mav_cmd_ack_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_CMD_ACK_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_COLLISION_ACTION_LOOKUP = {
    {0, "MAV_COLLISION_ACTION_NONE"},
    {1, "MAV_COLLISION_ACTION_REPORT"},
    {2, "MAV_COLLISION_ACTION_ASCEND_OR_DESCEND"},
    {3, "MAV_COLLISION_ACTION_MOVE_HORIZONTALLY"},
    {4, "MAV_COLLISION_ACTION_MOVE_PERPENDICULAR"},
    {5, "MAV_COLLISION_ACTION_RTL"},
    {6, "MAV_COLLISION_ACTION_HOVER"},
};

inline const char* mavlink_mav_collision_action_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_COLLISION_ACTION_LOOKUP);
}

inline std::string mavlink_mav_collision_action_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_COLLISION_ACTION_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_COLLISION_SRC_LOOKUP = {
    {0, "MAV_COLLISION_SRC_ADSB"},
    {1, "MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT"},
};

inline const char* mavlink_mav_collision_src_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_COLLISION_SRC_LOOKUP);
}

inline std::string mavlink_mav_collision_src_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_COLLISION_SRC_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_COLLISION_THREAT_LEVEL_LOOKUP = {
    {0, "MAV_COLLISION_THREAT_LEVEL_NONE"},
    {1, "MAV_COLLISION_THREAT_LEVEL_LOW"},
    {2, "MAV_COLLISION_THREAT_LEVEL_HIGH"},
};

inline const char* mavlink_mav_collision_threat_level_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_COLLISION_THREAT_LEVEL_LOOKUP);
}

inline std::string mavlink_mav_collision_threat_level_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_COLLISION_THREAT_LEVEL_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_COMPONENT_LOOKUP = {
    {0, "MAV_COMP_ID_ALL"},
    {1, "MAV_COMP_ID_AUTOPILOT1"},
    {25, "MAV_COMP_ID_USER1"},
    {26, "MAV_COMP_ID_USER2"},
    {27, "MAV_COMP_ID_USER3"},
    {28, "MAV_COMP_ID_USER4"},
    {29, "MAV_COMP_ID_USER5"},
    {30, "MAV_COMP_ID_USER6"},
    {31, "MAV_COMP_ID_USER7"},
    {32, "MAV_COMP_ID_USER8"},
    {33, "MAV_COMP_ID_USER9"},
    {34, "MAV_COMP_ID_USER10"},
    {35, "MAV_COMP_ID_USER11"},
    {36, "MAV_COMP_ID_USER12"},
    {37, "MAV_COMP_ID_USER13"},
    {38, "MAV_COMP_ID_USER14"},
    {39, "MAV_COMP_ID_USER15"},
    {40, "MAV_COMP_ID_USER16"},
    {41, "MAV_COMP_ID_USER17"},
    {42, "MAV_COMP_ID_USER18"},
    {43, "MAV_COMP_ID_USER19"},
    {44, "MAV_COMP_ID_USER20"},
    {45, "MAV_COMP_ID_USER21"},
    {46, "MAV_COMP_ID_USER22"},
    {47, "MAV_COMP_ID_USER23"},
    {48, "MAV_COMP_ID_USER24"},
    {49, "MAV_COMP_ID_USER25"},
    {50, "MAV_COMP_ID_USER26"},
    {51, "MAV_COMP_ID_USER27"},
    {52, "MAV_COMP_ID_USER28"},
    {53, "MAV_COMP_ID_USER29"},
    {54, "MAV_COMP_ID_USER30"},
    {55, "MAV_COMP_ID_USER31"},
    {56, "MAV_COMP_ID_USER32"},
    {57, "MAV_COMP_ID_USER33"},
    {58, "MAV_COMP_ID_USER34"},
    {59, "MAV_COMP_ID_USER35"},
    {60, "MAV_COMP_ID_USER36"},
    {61, "MAV_COMP_ID_USER37"},
    {62, "MAV_COMP_ID_USER38"},
    {63, "MAV_COMP_ID_USER39"},
    {64, "MAV_COMP_ID_USER40"},
    {65, "MAV_COMP_ID_USER41"},
    {66, "MAV_COMP_ID_USER42"},
    {67, "MAV_COMP_ID_USER43"},
    {68, "MAV_COMP_ID_TELEMETRY_RADIO"},
    {69, "MAV_COMP_ID_USER45"},
    {70, "MAV_COMP_ID_USER46"},
    {71, "MAV_COMP_ID_USER47"},
    {72, "MAV_COMP_ID_USER48"},
    {73, "MAV_COMP_ID_USER49"},
    {74, "MAV_COMP_ID_USER50"},
    {75, "MAV_COMP_ID_USER51"},
    {76, "MAV_COMP_ID_USER52"},
    {77, "MAV_COMP_ID_USER53"},
    {78, "MAV_COMP_ID_USER54"},
    {79, "MAV_COMP_ID_USER55"},
    {80, "MAV_COMP_ID_USER56"},
    {81, "MAV_COMP_ID_USER57"},
    {82, "MAV_COMP_ID_USER58"},
    {83, "MAV_COMP_ID_USER59"},
    {84, "MAV_COMP_ID_USER60"},
    {85, "MAV_COMP_ID_USER61"},
    {86, "MAV_COMP_ID_USER62"},
    {87, "MAV_COMP_ID_USER63"},
    {88, "MAV_COMP_ID_USER64"},
    {89, "MAV_COMP_ID_USER65"},
    {90, "MAV_COMP_ID_USER66"},
    {91, "MAV_COMP_ID_USER67"},
    {92, "MAV_COMP_ID_USER68"},
    {93, "MAV_COMP_ID_USER69"},
    {94, "MAV_COMP_ID_USER70"},
    {95, "MAV_COMP_ID_USER71"},
    {96, "MAV_COMP_ID_USER72"},
    {97, "MAV_COMP_ID_USER73"},
    {98, "MAV_COMP_ID_USER74"},
    {99, "MAV_COMP_ID_USER75"},
    {100, "MAV_COMP_ID_CAMERA"},
    {101, "MAV_COMP_ID_CAMERA2"},
    {102, "MAV_COMP_ID_CAMERA3"},
    {103, "MAV_COMP_ID_CAMERA4"},
    {104, "MAV_COMP_ID_CAMERA5"},
    {105, "MAV_COMP_ID_CAMERA6"},
    {140, "MAV_COMP_ID_SERVO1"},
    {141, "MAV_COMP_ID_SERVO2"},
    {142, "MAV_COMP_ID_SERVO3"},
    {143, "MAV_COMP_ID_SERVO4"},
    {144, "MAV_COMP_ID_SERVO5"},
    {145, "MAV_COMP_ID_SERVO6"},
    {146, "MAV_COMP_ID_SERVO7"},
    {147, "MAV_COMP_ID_SERVO8"},
    {148, "MAV_COMP_ID_SERVO9"},
    {149, "MAV_COMP_ID_SERVO10"},
    {150, "MAV_COMP_ID_SERVO11"},
    {151, "MAV_COMP_ID_SERVO12"},
    {152, "MAV_COMP_ID_SERVO13"},
    {153, "MAV_COMP_ID_SERVO14"},
    {154, "MAV_COMP_ID_GIMBAL"},
    {155, "MAV_COMP_ID_LOG"},
    {156, "MAV_COMP_ID_ADSB"},
    {157, "MAV_COMP_ID_OSD"},
    {158, "MAV_COMP_ID_PERIPHERAL"},
    {159, "MAV_COMP_ID_QX1_GIMBAL"},
    {160, "MAV_COMP_ID_FLARM"},
    {171, "MAV_COMP_ID_GIMBAL2"},
    {172, "MAV_COMP_ID_GIMBAL3"},
    {173, "MAV_COMP_ID_GIMBAL4"},
    {174, "MAV_COMP_ID_GIMBAL5"},
    {175, "MAV_COMP_ID_GIMBAL6"},
    {190, "MAV_COMP_ID_MISSIONPLANNER"},
    {191, "MAV_COMP_ID_ONBOARD_COMPUTER"},
    {195, "MAV_COMP_ID_PATHPLANNER"},
    {196, "MAV_COMP_ID_OBSTACLE_AVOIDANCE"},
    {197, "MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY"},
    {198, "MAV_COMP_ID_PAIRING_MANAGER"},
    {200, "MAV_COMP_ID_IMU"},
    {201, "MAV_COMP_ID_IMU_2"},
    {202, "MAV_COMP_ID_IMU_3"},
    {220, "MAV_COMP_ID_GPS"},
    {221, "MAV_COMP_ID_GPS2"},
    {236, "MAV_COMP_ID_ODID_TXRX_1"},
    {237, "MAV_COMP_ID_ODID_TXRX_2"},
    {238, "MAV_COMP_ID_ODID_TXRX_3"},
    {240, "MAV_COMP_ID_UDP_BRIDGE"},
    {241, "MAV_COMP_ID_UART_BRIDGE"},
    {242, "MAV_COMP_ID_TUNNEL_NODE"},
    {250, "MAV_COMP_ID_SYSTEM_CONTROL"},
};

inline const char* mavlink_mav_component_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_COMPONENT_LOOKUP);
}

inline std::string mavlink_mav_component_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_COMPONENT_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_DATA_STREAM_LOOKUP = {
    {0, "MAV_DATA_STREAM_ALL"},
    {1, "MAV_DATA_STREAM_RAW_SENSORS"},
    {2, "MAV_DATA_STREAM_EXTENDED_STATUS"},
    {3, "MAV_DATA_STREAM_RC_CHANNELS"},
    {4, "MAV_DATA_STREAM_RAW_CONTROLLER"},
    {6, "MAV_DATA_STREAM_POSITION"},
    {10, "MAV_DATA_STREAM_EXTRA1"},
    {11, "MAV_DATA_STREAM_EXTRA2"},
    {12, "MAV_DATA_STREAM_EXTRA3"},
};

inline const char* mavlink_mav_data_stream_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_DATA_STREAM_LOOKUP);
}

inline std::string mavlink_mav_data_stream_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_DATA_STREAM_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_DISTANCE_SENSOR_LOOKUP = {
    {0, "MAV_DISTANCE_SENSOR_LASER"},
    {1, "MAV_DISTANCE_SENSOR_ULTRASOUND"},
    {2, "MAV_DISTANCE_SENSOR_INFRARED"},
    {3, "MAV_DISTANCE_SENSOR_RADAR"},
    {4, "MAV_DISTANCE_SENSOR_UNKNOWN"},
};

inline const char* mavlink_mav_distance_sensor_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_DISTANCE_SENSOR_LOOKUP);
}

inline std::string mavlink_mav_distance_sensor_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_DISTANCE_SENSOR_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_DO_REPOSITION_FLAGS_LOOKUP = {
    {1, "MAV_DO_REPOSITION_FLAGS_CHANGE_MODE"},
};

inline const char* mavlink_mav_do_reposition_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_DO_REPOSITION_FLAGS_LOOKUP);
}

inline std::string mavlink_mav_do_reposition_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_DO_REPOSITION_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ESTIMATOR_TYPE_LOOKUP = {
    {0, "MAV_ESTIMATOR_TYPE_UNKNOWN"},
    {1, "MAV_ESTIMATOR_TYPE_NAIVE"},
    {2, "MAV_ESTIMATOR_TYPE_VISION"},
    {3, "MAV_ESTIMATOR_TYPE_VIO"},
    {4, "MAV_ESTIMATOR_TYPE_GPS"},
    {5, "MAV_ESTIMATOR_TYPE_GPS_INS"},
    {6, "MAV_ESTIMATOR_TYPE_MOCAP"},
    {7, "MAV_ESTIMATOR_TYPE_LIDAR"},
    {8, "MAV_ESTIMATOR_TYPE_AUTOPILOT"},
};

inline const char* mavlink_mav_estimator_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ESTIMATOR_TYPE_LOOKUP);
}

inline std::string mavlink_mav_estimator_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ESTIMATOR_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_FRAME_LOOKUP = {
    {0, "MAV_FRAME_GLOBAL"},
    {1, "MAV_FRAME_LOCAL_NED"},
    {2, "MAV_FRAME_MISSION"},
    {3, "MAV_FRAME_GLOBAL_RELATIVE_ALT"},
    {4, "MAV_FRAME_LOCAL_ENU"},
    {5, "MAV_FRAME_GLOBAL_INT"},
    {6, "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT"},
    {7, "MAV_FRAME_LOCAL_OFFSET_NED"},
    {8, "MAV_FRAME_BODY_NED"},
    {9, "MAV_FRAME_BODY_OFFSET_NED"},
    {10, "MAV_FRAME_GLOBAL_TERRAIN_ALT"},
    {11, "MAV_FRAME_GLOBAL_TERRAIN_ALT_INT"},
    {12, "MAV_FRAME_BODY_FRD"},
    {13, "MAV_FRAME_RESERVED_13"},
    {14, "MAV_FRAME_RESERVED_14"},
    {15, "MAV_FRAME_RESERVED_15"},
    {16, "MAV_FRAME_RESERVED_16"},
    {17, "MAV_FRAME_RESERVED_17"},
    {18, "MAV_FRAME_RESERVED_18"},
    {19, "MAV_FRAME_RESERVED_19"},
    {20, "MAV_FRAME_LOCAL_FRD"},
    {21, "MAV_FRAME_LOCAL_FLU"},
};

inline const char* mavlink_mav_frame_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_FRAME_LOOKUP);
}

inline std::string mavlink_mav_frame_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_FRAME_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_GENERATOR_STATUS_FLAG_LOOKUP = {
    {1, "MAV_GENERATOR_STATUS_FLAG_OFF"},
    {2, "MAV_GENERATOR_STATUS_FLAG_READY"},
    {4, "MAV_GENERATOR_STATUS_FLAG_GENERATING"},
    {8, "MAV_GENERATOR_STATUS_FLAG_CHARGING"},
    {16, "MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER"},
    {32, "MAV_GENERATOR_STATUS_FLAG_MAXPOWER"},
    {64, "MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING"},
    {128, "MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT"},
    {256, "MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING"},
    {512, "MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT"},
    {1024, "MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT"},
    {2048, "MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT"},
    {4096, "MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING"},
    {8192, "MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING"},
    {16384, "MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT"},
    {32768, "MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT"},
    {65536, "MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT"},
    {131072, "MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT"},
    {262144, "MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT"},
    {524288, "MAV_GENERATOR_STATUS_FLAG_START_INHIBITED"},
    {1048576, "MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED"},
    {2097152, "MAV_GENERATOR_STATUS_FLAG_WARMING_UP"},
    {4194304, "MAV_GENERATOR_STATUS_FLAG_IDLE"},
};

inline const char* mavlink_mav_generator_status_flag_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_GENERATOR_STATUS_FLAG_LOOKUP);
}

inline std::string mavlink_mav_generator_status_flag_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_GENERATOR_STATUS_FLAG_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_GOTO_LOOKUP = {
    {0, "MAV_GOTO_DO_HOLD"},
    {1, "MAV_GOTO_DO_CONTINUE"},
    {2, "MAV_GOTO_HOLD_AT_CURRENT_POSITION"},
    {3, "MAV_GOTO_HOLD_AT_SPECIFIED_POSITION"},
};

inline const char* mavlink_mav_goto_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_GOTO_LOOKUP);
}

inline std::string mavlink_mav_goto_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_GOTO_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_LANDED_STATE_LOOKUP = {
    {0, "MAV_LANDED_STATE_UNDEFINED"},
    {1, "MAV_LANDED_STATE_ON_GROUND"},
    {2, "MAV_LANDED_STATE_IN_AIR"},
    {3, "MAV_LANDED_STATE_TAKEOFF"},
    {4, "MAV_LANDED_STATE_LANDING"},
};

inline const char* mavlink_mav_landed_state_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_LANDED_STATE_LOOKUP);
}

inline std::string mavlink_mav_landed_state_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_LANDED_STATE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_MISSION_RESULT_LOOKUP = {
    {0, "MAV_MISSION_ACCEPTED"},
    {1, "MAV_MISSION_ERROR"},
    {2, "MAV_MISSION_UNSUPPORTED_FRAME"},
    {3, "MAV_MISSION_UNSUPPORTED"},
    {4, "MAV_MISSION_NO_SPACE"},
    {5, "MAV_MISSION_INVALID"},
    {6, "MAV_MISSION_INVALID_PARAM1"},
    {7, "MAV_MISSION_INVALID_PARAM2"},
    {8, "MAV_MISSION_INVALID_PARAM3"},
    {9, "MAV_MISSION_INVALID_PARAM4"},
    {10, "MAV_MISSION_INVALID_PARAM5_X"},
    {11, "MAV_MISSION_INVALID_PARAM6_Y"},
    {12, "MAV_MISSION_INVALID_PARAM7"},
    {13, "MAV_MISSION_INVALID_SEQUENCE"},
    {14, "MAV_MISSION_DENIED"},
    {15, "MAV_MISSION_OPERATION_CANCELLED"},
};

inline const char* mavlink_mav_mission_result_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_MISSION_RESULT_LOOKUP);
}

inline std::string mavlink_mav_mission_result_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_MISSION_RESULT_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_MISSION_TYPE_LOOKUP = {
    {0, "MAV_MISSION_TYPE_MISSION"},
    {1, "MAV_MISSION_TYPE_FENCE"},
    {2, "MAV_MISSION_TYPE_RALLY"},
    {255, "MAV_MISSION_TYPE_ALL"},
};

inline const char* mavlink_mav_mission_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_MISSION_TYPE_LOOKUP);
}

inline std::string mavlink_mav_mission_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_MISSION_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_MODE_LOOKUP = {
    {0, "MAV_MODE_PREFLIGHT"},
    {64, "MAV_MODE_MANUAL_DISARMED"},
    {66, "MAV_MODE_TEST_DISARMED"},
    {80, "MAV_MODE_STABILIZE_DISARMED"},
    {88, "MAV_MODE_GUIDED_DISARMED"},
    {92, "MAV_MODE_AUTO_DISARMED"},
    {192, "MAV_MODE_MANUAL_ARMED"},
    {194, "MAV_MODE_TEST_ARMED"},
    {208, "MAV_MODE_STABILIZE_ARMED"},
    {216, "MAV_MODE_GUIDED_ARMED"},
    {220, "MAV_MODE_AUTO_ARMED"},
};

inline const char* mavlink_mav_mode_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_MODE_LOOKUP);
}

inline std::string mavlink_mav_mode_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_MODE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_MODE_FLAG_LOOKUP = {
    {1, "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED"},
    {2, "MAV_MODE_FLAG_TEST_ENABLED"},
    {4, "MAV_MODE_FLAG_AUTO_ENABLED"},
    {8, "MAV_MODE_FLAG_GUIDED_ENABLED"},
    {16, "MAV_MODE_FLAG_STABILIZE_ENABLED"},
    {32, "MAV_MODE_FLAG_HIL_ENABLED"},
    {64, "MAV_MODE_FLAG_MANUAL_INPUT_ENABLED"},
    {128, "MAV_MODE_FLAG_SAFETY_ARMED"},
};

inline const char* mavlink_mav_mode_flag_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_MODE_FLAG_LOOKUP);
}

inline std::string mavlink_mav_mode_flag_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_MODE_FLAG_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_MODE_FLAG_DECODE_POSITION_LOOKUP = {
    {1, "MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE"},
    {2, "MAV_MODE_FLAG_DECODE_POSITION_TEST"},
    {4, "MAV_MODE_FLAG_DECODE_POSITION_AUTO"},
    {8, "MAV_MODE_FLAG_DECODE_POSITION_GUIDED"},
    {16, "MAV_MODE_FLAG_DECODE_POSITION_STABILIZE"},
    {32, "MAV_MODE_FLAG_DECODE_POSITION_HIL"},
    {64, "MAV_MODE_FLAG_DECODE_POSITION_MANUAL"},
    {128, "MAV_MODE_FLAG_DECODE_POSITION_SAFETY"},
};

inline const char* mavlink_mav_mode_flag_decode_position_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_MODE_FLAG_DECODE_POSITION_LOOKUP);
}

inline std::string mavlink_mav_mode_flag_decode_position_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_MODE_FLAG_DECODE_POSITION_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_MOUNT_MODE_LOOKUP = {
    {0, "MAV_MOUNT_MODE_RETRACT"},
    {1, "MAV_MOUNT_MODE_NEUTRAL"},
    {2, "MAV_MOUNT_MODE_MAVLINK_TARGETING"},
    {3, "MAV_MOUNT_MODE_RC_TARGETING"},
    {4, "MAV_MOUNT_MODE_GPS_POINT"},
    {5, "MAV_MOUNT_MODE_SYSID_TARGET"},
    {6, "MAV_MOUNT_MODE_HOME_LOCATION"},
};

inline const char* mavlink_mav_mount_mode_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_MOUNT_MODE_LOOKUP);
}

inline std::string mavlink_mav_mount_mode_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_MOUNT_MODE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_AUTH_TYPE_LOOKUP = {
    {0, "MAV_ODID_AUTH_TYPE_NONE"},
    {1, "MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE"},
    {2, "MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE"},
    {3, "MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE"},
    {4, "MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID"},
};

inline const char* mavlink_mav_odid_auth_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_AUTH_TYPE_LOOKUP);
}

inline std::string mavlink_mav_odid_auth_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_AUTH_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_CATEGORY_EU_LOOKUP = {
    {0, "MAV_ODID_CATEGORY_EU_UNDECLARED"},
    {1, "MAV_ODID_CATEGORY_EU_OPEN"},
    {2, "MAV_ODID_CATEGORY_EU_SPECIFIC"},
    {3, "MAV_ODID_CATEGORY_EU_CERTIFIED"},
};

inline const char* mavlink_mav_odid_category_eu_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_CATEGORY_EU_LOOKUP);
}

inline std::string mavlink_mav_odid_category_eu_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_CATEGORY_EU_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_CLASSIFICATION_TYPE_LOOKUP = {
    {0, "MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED"},
    {1, "MAV_ODID_CLASSIFICATION_TYPE_EU"},
};

inline const char* mavlink_mav_odid_classification_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_CLASSIFICATION_TYPE_LOOKUP);
}

inline std::string mavlink_mav_odid_classification_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_CLASSIFICATION_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_CLASS_EU_LOOKUP = {
    {0, "MAV_ODID_CLASS_EU_UNDECLARED"},
    {1, "MAV_ODID_CLASS_EU_CLASS_0"},
    {2, "MAV_ODID_CLASS_EU_CLASS_1"},
    {3, "MAV_ODID_CLASS_EU_CLASS_2"},
    {4, "MAV_ODID_CLASS_EU_CLASS_3"},
    {5, "MAV_ODID_CLASS_EU_CLASS_4"},
    {6, "MAV_ODID_CLASS_EU_CLASS_5"},
    {7, "MAV_ODID_CLASS_EU_CLASS_6"},
};

inline const char* mavlink_mav_odid_class_eu_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_CLASS_EU_LOOKUP);
}

inline std::string mavlink_mav_odid_class_eu_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_CLASS_EU_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_DESC_TYPE_LOOKUP = {
    {0, "MAV_ODID_DESC_TYPE_TEXT"},
};

inline const char* mavlink_mav_odid_desc_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_DESC_TYPE_LOOKUP);
}

inline std::string mavlink_mav_odid_desc_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_DESC_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_HEIGHT_REF_LOOKUP = {
    {0, "MAV_ODID_HEIGHT_REF_OVER_TAKEOFF"},
    {1, "MAV_ODID_HEIGHT_REF_OVER_GROUND"},
};

inline const char* mavlink_mav_odid_height_ref_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_HEIGHT_REF_LOOKUP);
}

inline std::string mavlink_mav_odid_height_ref_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_HEIGHT_REF_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_HOR_ACC_LOOKUP = {
    {0, "MAV_ODID_HOR_ACC_UNKNOWN"},
    {1, "MAV_ODID_HOR_ACC_10NM"},
    {2, "MAV_ODID_HOR_ACC_4NM"},
    {3, "MAV_ODID_HOR_ACC_2NM"},
    {4, "MAV_ODID_HOR_ACC_1NM"},
    {5, "MAV_ODID_HOR_ACC_0_5NM"},
    {6, "MAV_ODID_HOR_ACC_0_3NM"},
    {7, "MAV_ODID_HOR_ACC_0_1NM"},
    {8, "MAV_ODID_HOR_ACC_0_05NM"},
    {9, "MAV_ODID_HOR_ACC_30_METER"},
    {10, "MAV_ODID_HOR_ACC_10_METER"},
    {11, "MAV_ODID_HOR_ACC_3_METER"},
    {12, "MAV_ODID_HOR_ACC_1_METER"},
};

inline const char* mavlink_mav_odid_hor_acc_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_HOR_ACC_LOOKUP);
}

inline std::string mavlink_mav_odid_hor_acc_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_HOR_ACC_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_ID_TYPE_LOOKUP = {
    {0, "MAV_ODID_ID_TYPE_NONE"},
    {1, "MAV_ODID_ID_TYPE_SERIAL_NUMBER"},
    {2, "MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID"},
    {3, "MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID"},
};

inline const char* mavlink_mav_odid_id_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_ID_TYPE_LOOKUP);
}

inline std::string mavlink_mav_odid_id_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_ID_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_OPERATOR_ID_TYPE_LOOKUP = {
    {0, "MAV_ODID_OPERATOR_ID_TYPE_CAA"},
};

inline const char* mavlink_mav_odid_operator_id_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_OPERATOR_ID_TYPE_LOOKUP);
}

inline std::string mavlink_mav_odid_operator_id_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_OPERATOR_ID_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_OPERATOR_LOCATION_TYPE_LOOKUP = {
    {0, "MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF"},
    {1, "MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS"},
    {2, "MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED"},
};

inline const char* mavlink_mav_odid_operator_location_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_OPERATOR_LOCATION_TYPE_LOOKUP);
}

inline std::string mavlink_mav_odid_operator_location_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_OPERATOR_LOCATION_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_SPEED_ACC_LOOKUP = {
    {0, "MAV_ODID_SPEED_ACC_UNKNOWN"},
    {1, "MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND"},
    {2, "MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND"},
    {3, "MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND"},
    {4, "MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND"},
};

inline const char* mavlink_mav_odid_speed_acc_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_SPEED_ACC_LOOKUP);
}

inline std::string mavlink_mav_odid_speed_acc_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_SPEED_ACC_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_STATUS_LOOKUP = {
    {0, "MAV_ODID_STATUS_UNDECLARED"},
    {1, "MAV_ODID_STATUS_GROUND"},
    {2, "MAV_ODID_STATUS_AIRBORNE"},
    {3, "MAV_ODID_STATUS_EMERGENCY"},
};

inline const char* mavlink_mav_odid_status_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_STATUS_LOOKUP);
}

inline std::string mavlink_mav_odid_status_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_STATUS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_TIME_ACC_LOOKUP = {
    {0, "MAV_ODID_TIME_ACC_UNKNOWN"},
    {1, "MAV_ODID_TIME_ACC_0_1_SECOND"},
    {2, "MAV_ODID_TIME_ACC_0_2_SECOND"},
    {3, "MAV_ODID_TIME_ACC_0_3_SECOND"},
    {4, "MAV_ODID_TIME_ACC_0_4_SECOND"},
    {5, "MAV_ODID_TIME_ACC_0_5_SECOND"},
    {6, "MAV_ODID_TIME_ACC_0_6_SECOND"},
    {7, "MAV_ODID_TIME_ACC_0_7_SECOND"},
    {8, "MAV_ODID_TIME_ACC_0_8_SECOND"},
    {9, "MAV_ODID_TIME_ACC_0_9_SECOND"},
    {10, "MAV_ODID_TIME_ACC_1_0_SECOND"},
    {11, "MAV_ODID_TIME_ACC_1_1_SECOND"},
    {12, "MAV_ODID_TIME_ACC_1_2_SECOND"},
    {13, "MAV_ODID_TIME_ACC_1_3_SECOND"},
    {14, "MAV_ODID_TIME_ACC_1_4_SECOND"},
    {15, "MAV_ODID_TIME_ACC_1_5_SECOND"},
};

inline const char* mavlink_mav_odid_time_acc_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_TIME_ACC_LOOKUP);
}

inline std::string mavlink_mav_odid_time_acc_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_TIME_ACC_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_UA_TYPE_LOOKUP = {
    {0, "MAV_ODID_UA_TYPE_NONE"},
    {1, "MAV_ODID_UA_TYPE_AEROPLANE"},
    {2, "MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR"},
    {3, "MAV_ODID_UA_TYPE_GYROPLANE"},
    {4, "MAV_ODID_UA_TYPE_HYBRID_LIFT"},
    {5, "MAV_ODID_UA_TYPE_ORNITHOPTER"},
    {6, "MAV_ODID_UA_TYPE_GLIDER"},
    {7, "MAV_ODID_UA_TYPE_KITE"},
    {8, "MAV_ODID_UA_TYPE_FREE_BALLOON"},
    {9, "MAV_ODID_UA_TYPE_CAPTIVE_BALLOON"},
    {10, "MAV_ODID_UA_TYPE_AIRSHIP"},
    {11, "MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE"},
    {12, "MAV_ODID_UA_TYPE_ROCKET"},
    {13, "MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT"},
    {14, "MAV_ODID_UA_TYPE_GROUND_OBSTACLE"},
    {15, "MAV_ODID_UA_TYPE_OTHER"},
};

inline const char* mavlink_mav_odid_ua_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_UA_TYPE_LOOKUP);
}

inline std::string mavlink_mav_odid_ua_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_UA_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ODID_VER_ACC_LOOKUP = {
    {0, "MAV_ODID_VER_ACC_UNKNOWN"},
    {1, "MAV_ODID_VER_ACC_150_METER"},
    {2, "MAV_ODID_VER_ACC_45_METER"},
    {3, "MAV_ODID_VER_ACC_25_METER"},
    {4, "MAV_ODID_VER_ACC_10_METER"},
    {5, "MAV_ODID_VER_ACC_3_METER"},
    {6, "MAV_ODID_VER_ACC_1_METER"},
};

inline const char* mavlink_mav_odid_ver_acc_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ODID_VER_ACC_LOOKUP);
}

inline std::string mavlink_mav_odid_ver_acc_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ODID_VER_ACC_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_PARAM_EXT_TYPE_LOOKUP = {
    {1, "MAV_PARAM_EXT_TYPE_UINT8"},
    {2, "MAV_PARAM_EXT_TYPE_INT8"},
    {3, "MAV_PARAM_EXT_TYPE_UINT16"},
    {4, "MAV_PARAM_EXT_TYPE_INT16"},
    {5, "MAV_PARAM_EXT_TYPE_UINT32"},
    {6, "MAV_PARAM_EXT_TYPE_INT32"},
    {7, "MAV_PARAM_EXT_TYPE_UINT64"},
    {8, "MAV_PARAM_EXT_TYPE_INT64"},
    {9, "MAV_PARAM_EXT_TYPE_REAL32"},
    {10, "MAV_PARAM_EXT_TYPE_REAL64"},
    {11, "MAV_PARAM_EXT_TYPE_CUSTOM"},
};

inline const char* mavlink_mav_param_ext_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_PARAM_EXT_TYPE_LOOKUP);
}

inline std::string mavlink_mav_param_ext_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_PARAM_EXT_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_PARAM_TYPE_LOOKUP = {
    {1, "MAV_PARAM_TYPE_UINT8"},
    {2, "MAV_PARAM_TYPE_INT8"},
    {3, "MAV_PARAM_TYPE_UINT16"},
    {4, "MAV_PARAM_TYPE_INT16"},
    {5, "MAV_PARAM_TYPE_UINT32"},
    {6, "MAV_PARAM_TYPE_INT32"},
    {7, "MAV_PARAM_TYPE_UINT64"},
    {8, "MAV_PARAM_TYPE_INT64"},
    {9, "MAV_PARAM_TYPE_REAL32"},
    {10, "MAV_PARAM_TYPE_REAL64"},
};

inline const char* mavlink_mav_param_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_PARAM_TYPE_LOOKUP);
}

inline std::string mavlink_mav_param_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_PARAM_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_POWER_STATUS_LOOKUP = {
    {1, "MAV_POWER_STATUS_BRICK_VALID"},
    {2, "MAV_POWER_STATUS_SERVO_VALID"},
    {4, "MAV_POWER_STATUS_USB_CONNECTED"},
    {8, "MAV_POWER_STATUS_PERIPH_OVERCURRENT"},
    {16, "MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT"},
    {32, "MAV_POWER_STATUS_CHANGED"},
};

inline const char* mavlink_mav_power_status_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_POWER_STATUS_LOOKUP);
}

inline std::string mavlink_mav_power_status_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_POWER_STATUS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_PROTOCOL_CAPABILITY_LOOKUP = {
    {1, "MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT"},
    {2, "MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT"},
    {4, "MAV_PROTOCOL_CAPABILITY_MISSION_INT"},
    {8, "MAV_PROTOCOL_CAPABILITY_COMMAND_INT"},
    {16, "MAV_PROTOCOL_CAPABILITY_PARAM_UNION"},
    {32, "MAV_PROTOCOL_CAPABILITY_FTP"},
    {64, "MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET"},
    {128, "MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED"},
    {256, "MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT"},
    {512, "MAV_PROTOCOL_CAPABILITY_TERRAIN"},
    {1024, "MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET"},
    {2048, "MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION"},
    {4096, "MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION"},
    {8192, "MAV_PROTOCOL_CAPABILITY_MAVLINK2"},
    {16384, "MAV_PROTOCOL_CAPABILITY_MISSION_FENCE"},
    {32768, "MAV_PROTOCOL_CAPABILITY_MISSION_RALLY"},
    {65536, "MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION"},
};

inline const char* mavlink_mav_protocol_capability_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_PROTOCOL_CAPABILITY_LOOKUP);
}

inline std::string mavlink_mav_protocol_capability_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_PROTOCOL_CAPABILITY_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_RESULT_LOOKUP = {
    {0, "MAV_RESULT_ACCEPTED"},
    {1, "MAV_RESULT_TEMPORARILY_REJECTED"},
    {2, "MAV_RESULT_DENIED"},
    {3, "MAV_RESULT_UNSUPPORTED"},
    {4, "MAV_RESULT_FAILED"},
    {5, "MAV_RESULT_IN_PROGRESS"},
    {6, "MAV_RESULT_CANCELLED"},
};

inline const char* mavlink_mav_result_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_RESULT_LOOKUP);
}

inline std::string mavlink_mav_result_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_RESULT_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_ROI_LOOKUP = {
    {0, "MAV_ROI_NONE"},
    {1, "MAV_ROI_WPNEXT"},
    {2, "MAV_ROI_WPINDEX"},
    {3, "MAV_ROI_LOCATION"},
    {4, "MAV_ROI_TARGET"},
};

inline const char* mavlink_mav_roi_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_ROI_LOOKUP);
}

inline std::string mavlink_mav_roi_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_ROI_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_SENSOR_ORIENTATION_LOOKUP = {
    {0, "MAV_SENSOR_ROTATION_NONE"},
    {1, "MAV_SENSOR_ROTATION_YAW_45"},
    {2, "MAV_SENSOR_ROTATION_YAW_90"},
    {3, "MAV_SENSOR_ROTATION_YAW_135"},
    {4, "MAV_SENSOR_ROTATION_YAW_180"},
    {5, "MAV_SENSOR_ROTATION_YAW_225"},
    {6, "MAV_SENSOR_ROTATION_YAW_270"},
    {7, "MAV_SENSOR_ROTATION_YAW_315"},
    {8, "MAV_SENSOR_ROTATION_ROLL_180"},
    {9, "MAV_SENSOR_ROTATION_ROLL_180_YAW_45"},
    {10, "MAV_SENSOR_ROTATION_ROLL_180_YAW_90"},
    {11, "MAV_SENSOR_ROTATION_ROLL_180_YAW_135"},
    {12, "MAV_SENSOR_ROTATION_PITCH_180"},
    {13, "MAV_SENSOR_ROTATION_ROLL_180_YAW_225"},
    {14, "MAV_SENSOR_ROTATION_ROLL_180_YAW_270"},
    {15, "MAV_SENSOR_ROTATION_ROLL_180_YAW_315"},
    {16, "MAV_SENSOR_ROTATION_ROLL_90"},
    {17, "MAV_SENSOR_ROTATION_ROLL_90_YAW_45"},
    {18, "MAV_SENSOR_ROTATION_ROLL_90_YAW_90"},
    {19, "MAV_SENSOR_ROTATION_ROLL_90_YAW_135"},
    {20, "MAV_SENSOR_ROTATION_ROLL_270"},
    {21, "MAV_SENSOR_ROTATION_ROLL_270_YAW_45"},
    {22, "MAV_SENSOR_ROTATION_ROLL_270_YAW_90"},
    {23, "MAV_SENSOR_ROTATION_ROLL_270_YAW_135"},
    {24, "MAV_SENSOR_ROTATION_PITCH_90"},
    {25, "MAV_SENSOR_ROTATION_PITCH_270"},
    {26, "MAV_SENSOR_ROTATION_PITCH_180_YAW_90"},
    {27, "MAV_SENSOR_ROTATION_PITCH_180_YAW_270"},
    {28, "MAV_SENSOR_ROTATION_ROLL_90_PITCH_90"},
    {29, "MAV_SENSOR_ROTATION_ROLL_180_PITCH_90"},
    {30, "MAV_SENSOR_ROTATION_ROLL_270_PITCH_90"},
    {31, "MAV_SENSOR_ROTATION_ROLL_90_PITCH_180"},
    {32, "MAV_SENSOR_ROTATION_ROLL_270_PITCH_180"},
    {33, "MAV_SENSOR_ROTATION_ROLL_90_PITCH_270"},
    {34, "MAV_SENSOR_ROTATION_ROLL_180_PITCH_270"},
    {35, "MAV_SENSOR_ROTATION_ROLL_270_PITCH_270"},
    {36, "MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90"},
    {37, "MAV_SENSOR_ROTATION_ROLL_90_YAW_270"},
    {38, "MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293"},
    {39, "MAV_SENSOR_ROTATION_PITCH_315"},
    {40, "MAV_SENSOR_ROTATION_ROLL_90_PITCH_315"},
    {41, "MAV_SENSOR_ROTATION_ROLL_270_YAW_180"},
    {100, "MAV_SENSOR_ROTATION_CUSTOM"},
};

inline const char* mavlink_mav_sensor_orientation_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_SENSOR_ORIENTATION_LOOKUP);
}

inline std::string mavlink_mav_sensor_orientation_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_SENSOR_ORIENTATION_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_SEVERITY_LOOKUP = {
    {0, "MAV_SEVERITY_EMERGENCY"},
    {1, "MAV_SEVERITY_ALERT"},
    {2, "MAV_SEVERITY_CRITICAL"},
    {3, "MAV_SEVERITY_ERROR"},
    {4, "MAV_SEVERITY_WARNING"},
    {5, "MAV_SEVERITY_NOTICE"},
    {6, "MAV_SEVERITY_INFO"},
    {7, "MAV_SEVERITY_DEBUG"},
};

inline const char* mavlink_mav_severity_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_SEVERITY_LOOKUP);
}

inline std::string mavlink_mav_severity_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_SEVERITY_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_STATE_LOOKUP = {
    {0, "MAV_STATE_UNINIT"},
    {1, "MAV_STATE_BOOT"},
    {2, "MAV_STATE_CALIBRATING"},
    {3, "MAV_STATE_STANDBY"},
    {4, "MAV_STATE_ACTIVE"},
    {5, "MAV_STATE_CRITICAL"},
    {6, "MAV_STATE_EMERGENCY"},
    {7, "MAV_STATE_POWEROFF"},
    {8, "MAV_STATE_FLIGHT_TERMINATION"},
};

inline const char* mavlink_mav_state_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_STATE_LOOKUP);
}

inline std::string mavlink_mav_state_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_STATE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_SYS_STATUS_SENSOR_LOOKUP = {
    {1, "MAV_SYS_STATUS_SENSOR_3D_GYRO"},
    {2, "MAV_SYS_STATUS_SENSOR_3D_ACCEL"},
    {4, "MAV_SYS_STATUS_SENSOR_3D_MAG"},
    {8, "MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE"},
    {16, "MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE"},
    {32, "MAV_SYS_STATUS_SENSOR_GPS"},
    {64, "MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW"},
    {128, "MAV_SYS_STATUS_SENSOR_VISION_POSITION"},
    {256, "MAV_SYS_STATUS_SENSOR_LASER_POSITION"},
    {512, "MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH"},
    {1024, "MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL"},
    {2048, "MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION"},
    {4096, "MAV_SYS_STATUS_SENSOR_YAW_POSITION"},
    {8192, "MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL"},
    {16384, "MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL"},
    {32768, "MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS"},
    {65536, "MAV_SYS_STATUS_SENSOR_RC_RECEIVER"},
    {131072, "MAV_SYS_STATUS_SENSOR_3D_GYRO2"},
    {262144, "MAV_SYS_STATUS_SENSOR_3D_ACCEL2"},
    {524288, "MAV_SYS_STATUS_SENSOR_3D_MAG2"},
    {1048576, "MAV_SYS_STATUS_GEOFENCE"},
    {2097152, "MAV_SYS_STATUS_AHRS"},
    {4194304, "MAV_SYS_STATUS_TERRAIN"},
    {8388608, "MAV_SYS_STATUS_REVERSE_MOTOR"},
    {16777216, "MAV_SYS_STATUS_LOGGING"},
    {33554432, "MAV_SYS_STATUS_SENSOR_BATTERY"},
    {67108864, "MAV_SYS_STATUS_SENSOR_PROXIMITY"},
    {134217728, "MAV_SYS_STATUS_SENSOR_SATCOM"},
    {268435456, "MAV_SYS_STATUS_PREARM_CHECK"},
    {536870912, "MAV_SYS_STATUS_OBSTACLE_AVOIDANCE"},
};

inline const char* mavlink_mav_sys_status_sensor_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_SYS_STATUS_SENSOR_LOOKUP);
}

inline std::string mavlink_mav_sys_status_sensor_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_SYS_STATUS_SENSOR_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_TUNNEL_PAYLOAD_TYPE_LOOKUP = {
    {0, "MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN"},
    {200, "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0"},
    {201, "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1"},
    {202, "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2"},
    {203, "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3"},
    {204, "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4"},
    {205, "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5"},
    {206, "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6"},
    {207, "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7"},
    {208, "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8"},
    {209, "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9"},
};

inline const char* mavlink_mav_tunnel_payload_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_TUNNEL_PAYLOAD_TYPE_LOOKUP);
}

inline std::string mavlink_mav_tunnel_payload_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_TUNNEL_PAYLOAD_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_TYPE_LOOKUP = {
    {0, "MAV_TYPE_GENERIC"},
    {1, "MAV_TYPE_FIXED_WING"},
    {2, "MAV_TYPE_QUADROTOR"},
    {3, "MAV_TYPE_COAXIAL"},
    {4, "MAV_TYPE_HELICOPTER"},
    {5, "MAV_TYPE_ANTENNA_TRACKER"},
    {6, "MAV_TYPE_GCS"},
    {7, "MAV_TYPE_AIRSHIP"},
    {8, "MAV_TYPE_FREE_BALLOON"},
    {9, "MAV_TYPE_ROCKET"},
    {10, "MAV_TYPE_GROUND_ROVER"},
    {11, "MAV_TYPE_SURFACE_BOAT"},
    {12, "MAV_TYPE_SUBMARINE"},
    {13, "MAV_TYPE_HEXAROTOR"},
    {14, "MAV_TYPE_OCTOROTOR"},
    {15, "MAV_TYPE_TRICOPTER"},
    {16, "MAV_TYPE_FLAPPING_WING"},
    {17, "MAV_TYPE_KITE"},
    {18, "MAV_TYPE_ONBOARD_CONTROLLER"},
    {19, "MAV_TYPE_VTOL_DUOROTOR"},
    {20, "MAV_TYPE_VTOL_QUADROTOR"},
    {21, "MAV_TYPE_VTOL_TILTROTOR"},
    {22, "MAV_TYPE_VTOL_RESERVED2"},
    {23, "MAV_TYPE_VTOL_RESERVED3"},
    {24, "MAV_TYPE_VTOL_RESERVED4"},
    {25, "MAV_TYPE_VTOL_RESERVED5"},
    {26, "MAV_TYPE_GIMBAL"},
    {27, "MAV_TYPE_ADSB"},
    {28, "MAV_TYPE_PARAFOIL"},
    {29, "MAV_TYPE_DODECAROTOR"},
    {30, "MAV_TYPE_CAMERA"},
    {31, "MAV_TYPE_CHARGING_STATION"},
    {32, "MAV_TYPE_FLARM"},
    {33, "MAV_TYPE_SERVO"},
    {34, "MAV_TYPE_ODID"},
    {35, "MAV_TYPE_DECAROTOR"},
};

inline const char* mavlink_mav_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_TYPE_LOOKUP);
}

inline std::string mavlink_mav_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_VTOL_STATE_LOOKUP = {
    {0, "MAV_VTOL_STATE_UNDEFINED"},
    {1, "MAV_VTOL_STATE_TRANSITION_TO_FW"},
    {2, "MAV_VTOL_STATE_TRANSITION_TO_MC"},
    {3, "MAV_VTOL_STATE_MC"},
    {4, "MAV_VTOL_STATE_FW"},
};

inline const char* mavlink_mav_vtol_state_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_VTOL_STATE_LOOKUP);
}

inline std::string mavlink_mav_vtol_state_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_VTOL_STATE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MAV_WINCH_STATUS_FLAG_LOOKUP = {
    {1, "MAV_WINCH_STATUS_HEALTHY"},
    {2, "MAV_WINCH_STATUS_FULLY_RETRACTED"},
    {4, "MAV_WINCH_STATUS_MOVING"},
    {8, "MAV_WINCH_STATUS_CLUTCH_ENGAGED"},
};

inline const char* mavlink_mav_winch_status_flag_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MAV_WINCH_STATUS_FLAG_LOOKUP);
}

inline std::string mavlink_mav_winch_status_flag_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MAV_WINCH_STATUS_FLAG_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MOTOR_TEST_ORDER_LOOKUP = {
    {0, "MOTOR_TEST_ORDER_DEFAULT"},
    {1, "MOTOR_TEST_ORDER_SEQUENCE"},
    {2, "MOTOR_TEST_ORDER_BOARD"},
};

inline const char* mavlink_motor_test_order_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MOTOR_TEST_ORDER_LOOKUP);
}

inline std::string mavlink_motor_test_order_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MOTOR_TEST_ORDER_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_MOTOR_TEST_THROTTLE_TYPE_LOOKUP = {
    {0, "MOTOR_TEST_THROTTLE_PERCENT"},
    {1, "MOTOR_TEST_THROTTLE_PWM"},
    {2, "MOTOR_TEST_THROTTLE_PILOT"},
    {3, "MOTOR_TEST_COMPASS_CAL"},
};

inline const char* mavlink_motor_test_throttle_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_MOTOR_TEST_THROTTLE_TYPE_LOOKUP);
}

inline std::string mavlink_motor_test_throttle_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_MOTOR_TEST_THROTTLE_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_ORBIT_YAW_BEHAVIOUR_LOOKUP = {
    {0, "ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER"},
    {1, "ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING"},
    {2, "ORBIT_YAW_BEHAVIOUR_UNCONTROLLED"},
    {3, "ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE"},
    {4, "ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED"},
};

inline const char* mavlink_orbit_yaw_behaviour_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_ORBIT_YAW_BEHAVIOUR_LOOKUP);
}

inline std::string mavlink_orbit_yaw_behaviour_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_ORBIT_YAW_BEHAVIOUR_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_PARACHUTE_ACTION_LOOKUP = {
    {0, "PARACHUTE_DISABLE"},
    {1, "PARACHUTE_ENABLE"},
    {2, "PARACHUTE_RELEASE"},
};

inline const char* mavlink_parachute_action_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_PARACHUTE_ACTION_LOOKUP);
}

inline std::string mavlink_parachute_action_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_PARACHUTE_ACTION_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_PARAM_ACK_LOOKUP = {
    {0, "PARAM_ACK_ACCEPTED"},
    {1, "PARAM_ACK_VALUE_UNSUPPORTED"},
    {2, "PARAM_ACK_FAILED"},
    {3, "PARAM_ACK_IN_PROGRESS"},
};

inline const char* mavlink_param_ack_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_PARAM_ACK_LOOKUP);
}

inline std::string mavlink_param_ack_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_PARAM_ACK_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_PARAM_TRANSACTION_ACTION_LOOKUP = {
    {0, "PARAM_TRANSACTION_ACTION_START"},
    {1, "PARAM_TRANSACTION_ACTION_COMMIT"},
    {2, "PARAM_TRANSACTION_ACTION_CANCEL"},
};

inline const char* mavlink_param_transaction_action_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_PARAM_TRANSACTION_ACTION_LOOKUP);
}

inline std::string mavlink_param_transaction_action_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_PARAM_TRANSACTION_ACTION_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_PARAM_TRANSACTION_TRANSPORT_LOOKUP = {
    {0, "PARAM_TRANSACTION_TRANSPORT_PARAM"},
    {1, "PARAM_TRANSACTION_TRANSPORT_PARAM_EXT"},
};

inline const char* mavlink_param_transaction_transport_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_PARAM_TRANSACTION_TRANSPORT_LOOKUP);
}

inline std::string mavlink_param_transaction_transport_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_PARAM_TRANSACTION_TRANSPORT_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_POSITION_TARGET_TYPEMASK_LOOKUP = {
    {1, "POSITION_TARGET_TYPEMASK_X_IGNORE"},
    {2, "POSITION_TARGET_TYPEMASK_Y_IGNORE"},
    {4, "POSITION_TARGET_TYPEMASK_Z_IGNORE"},
    {8, "POSITION_TARGET_TYPEMASK_VX_IGNORE"},
    {16, "POSITION_TARGET_TYPEMASK_VY_IGNORE"},
    {32, "POSITION_TARGET_TYPEMASK_VZ_IGNORE"},
    {64, "POSITION_TARGET_TYPEMASK_AX_IGNORE"},
    {128, "POSITION_TARGET_TYPEMASK_AY_IGNORE"},
    {256, "POSITION_TARGET_TYPEMASK_AZ_IGNORE"},
    {512, "POSITION_TARGET_TYPEMASK_FORCE_SET"},
    {1024, "POSITION_TARGET_TYPEMASK_YAW_IGNORE"},
    {2048, "POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE"},
};

inline const char* mavlink_position_target_typemask_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_POSITION_TARGET_TYPEMASK_LOOKUP);
}

inline std::string mavlink_position_target_typemask_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_POSITION_TARGET_TYPEMASK_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_PRECISION_LAND_MODE_LOOKUP = {
    {0, "PRECISION_LAND_MODE_DISABLED"},
    {1, "PRECISION_LAND_MODE_OPPORTUNISTIC"},
    {2, "PRECISION_LAND_MODE_REQUIRED"},
};

inline const char* mavlink_precision_land_mode_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_PRECISION_LAND_MODE_LOOKUP);
}

inline std::string mavlink_precision_land_mode_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_PRECISION_LAND_MODE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_RC_TYPE_LOOKUP = {
    {0, "RC_TYPE_SPEKTRUM_DSM2"},
    {1, "RC_TYPE_SPEKTRUM_DSMX"},
};

inline const char* mavlink_rc_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_RC_TYPE_LOOKUP);
}

inline std::string mavlink_rc_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_RC_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_RTK_BASELINE_COORDINATE_SYSTEM_LOOKUP = {
    {0, "RTK_BASELINE_COORDINATE_SYSTEM_ECEF"},
    {1, "RTK_BASELINE_COORDINATE_SYSTEM_NED"},
};

inline const char* mavlink_rtk_baseline_coordinate_system_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_RTK_BASELINE_COORDINATE_SYSTEM_LOOKUP);
}

inline std::string mavlink_rtk_baseline_coordinate_system_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_RTK_BASELINE_COORDINATE_SYSTEM_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_SERIAL_CONTROL_DEV_LOOKUP = {
    {0, "SERIAL_CONTROL_DEV_TELEM1"},
    {1, "SERIAL_CONTROL_DEV_TELEM2"},
    {2, "SERIAL_CONTROL_DEV_GPS1"},
    {3, "SERIAL_CONTROL_DEV_GPS2"},
    {10, "SERIAL_CONTROL_DEV_SHELL"},
    {100, "SERIAL_CONTROL_SERIAL0"},
    {101, "SERIAL_CONTROL_SERIAL1"},
    {102, "SERIAL_CONTROL_SERIAL2"},
    {103, "SERIAL_CONTROL_SERIAL3"},
    {104, "SERIAL_CONTROL_SERIAL4"},
    {105, "SERIAL_CONTROL_SERIAL5"},
    {106, "SERIAL_CONTROL_SERIAL6"},
    {107, "SERIAL_CONTROL_SERIAL7"},
    {108, "SERIAL_CONTROL_SERIAL8"},
    {109, "SERIAL_CONTROL_SERIAL9"},
};

inline const char* mavlink_serial_control_dev_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_SERIAL_CONTROL_DEV_LOOKUP);
}

inline std::string mavlink_serial_control_dev_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_SERIAL_CONTROL_DEV_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_SERIAL_CONTROL_FLAG_LOOKUP = {
    {1, "SERIAL_CONTROL_FLAG_REPLY"},
    {2, "SERIAL_CONTROL_FLAG_RESPOND"},
    {4, "SERIAL_CONTROL_FLAG_EXCLUSIVE"},
    {8, "SERIAL_CONTROL_FLAG_BLOCKING"},
    {16, "SERIAL_CONTROL_FLAG_MULTI"},
};

inline const char* mavlink_serial_control_flag_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_SERIAL_CONTROL_FLAG_LOOKUP);
}

inline std::string mavlink_serial_control_flag_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_SERIAL_CONTROL_FLAG_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_SET_FOCUS_TYPE_LOOKUP = {
    {0, "FOCUS_TYPE_STEP"},
    {1, "FOCUS_TYPE_CONTINUOUS"},
    {2, "FOCUS_TYPE_RANGE"},
    {3, "FOCUS_TYPE_METERS"},
};

inline const char* mavlink_set_focus_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_SET_FOCUS_TYPE_LOOKUP);
}

inline std::string mavlink_set_focus_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_SET_FOCUS_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_STORAGE_STATUS_LOOKUP = {
    {0, "STORAGE_STATUS_EMPTY"},
    {1, "STORAGE_STATUS_UNFORMATTED"},
    {2, "STORAGE_STATUS_READY"},
    {3, "STORAGE_STATUS_NOT_SUPPORTED"},
};

inline const char* mavlink_storage_status_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_STORAGE_STATUS_LOOKUP);
}

inline std::string mavlink_storage_status_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_STORAGE_STATUS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_STORAGE_TYPE_LOOKUP = {
    {0, "STORAGE_TYPE_UNKNOWN"},
    {1, "STORAGE_TYPE_USB_STICK"},
    {2, "STORAGE_TYPE_SD"},
    {3, "STORAGE_TYPE_MICROSD"},
    {4, "STORAGE_TYPE_CF"},
    {5, "STORAGE_TYPE_CFE"},
    {6, "STORAGE_TYPE_XQD"},
    {7, "STORAGE_TYPE_HD"},
    {254, "STORAGE_TYPE_OTHER"},
};

inline const char* mavlink_storage_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_STORAGE_TYPE_LOOKUP);
}

inline std::string mavlink_storage_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_STORAGE_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_TUNE_FORMAT_LOOKUP = {
    {1, "TUNE_FORMAT_QBASIC1_1"},
    {2, "TUNE_FORMAT_MML_MODERN"},
};

inline const char* mavlink_tune_format_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_TUNE_FORMAT_LOOKUP);
}

inline std::string mavlink_tune_format_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_TUNE_FORMAT_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_UAVCAN_NODE_HEALTH_LOOKUP = {
    {0, "UAVCAN_NODE_HEALTH_OK"},
    {1, "UAVCAN_NODE_HEALTH_WARNING"},
    {2, "UAVCAN_NODE_HEALTH_ERROR"},
    {3, "UAVCAN_NODE_HEALTH_CRITICAL"},
};

inline const char* mavlink_uavcan_node_health_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_UAVCAN_NODE_HEALTH_LOOKUP);
}

inline std::string mavlink_uavcan_node_health_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_UAVCAN_NODE_HEALTH_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_UAVCAN_NODE_MODE_LOOKUP = {
    {0, "UAVCAN_NODE_MODE_OPERATIONAL"},
    {1, "UAVCAN_NODE_MODE_INITIALIZATION"},
    {2, "UAVCAN_NODE_MODE_MAINTENANCE"},
    {3, "UAVCAN_NODE_MODE_SOFTWARE_UPDATE"},
    {7, "UAVCAN_NODE_MODE_OFFLINE"},
};

inline const char* mavlink_uavcan_node_mode_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_UAVCAN_NODE_MODE_LOOKUP);
}

inline std::string mavlink_uavcan_node_mode_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_UAVCAN_NODE_MODE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_UTM_DATA_AVAIL_FLAGS_LOOKUP = {
    {1, "UTM_DATA_AVAIL_FLAGS_TIME_VALID"},
    {2, "UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE"},
    {4, "UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE"},
    {8, "UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE"},
    {16, "UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE"},
    {32, "UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE"},
    {64, "UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE"},
    {128, "UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE"},
};

inline const char* mavlink_utm_data_avail_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_UTM_DATA_AVAIL_FLAGS_LOOKUP);
}

inline std::string mavlink_utm_data_avail_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_UTM_DATA_AVAIL_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_UTM_FLIGHT_STATE_LOOKUP = {
    {1, "UTM_FLIGHT_STATE_UNKNOWN"},
    {2, "UTM_FLIGHT_STATE_GROUND"},
    {3, "UTM_FLIGHT_STATE_AIRBORNE"},
    {16, "UTM_FLIGHT_STATE_EMERGENCY"},
    {32, "UTM_FLIGHT_STATE_NOCTRL"},
};

inline const char* mavlink_utm_flight_state_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_UTM_FLIGHT_STATE_LOOKUP);
}

inline std::string mavlink_utm_flight_state_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_UTM_FLIGHT_STATE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_VIDEO_STREAM_STATUS_FLAGS_LOOKUP = {
    {1, "VIDEO_STREAM_STATUS_FLAGS_RUNNING"},
    {2, "VIDEO_STREAM_STATUS_FLAGS_THERMAL"},
};

inline const char* mavlink_video_stream_status_flags_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_VIDEO_STREAM_STATUS_FLAGS_LOOKUP);
}

inline std::string mavlink_video_stream_status_flags_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_VIDEO_STREAM_STATUS_FLAGS_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_VIDEO_STREAM_TYPE_LOOKUP = {
    {0, "VIDEO_STREAM_TYPE_RTSP"},
    {1, "VIDEO_STREAM_TYPE_RTPUDP"},
    {2, "VIDEO_STREAM_TYPE_TCP_MPEG"},
    {3, "VIDEO_STREAM_TYPE_MPEG_TS_H264"},
};

inline const char* mavlink_video_stream_type_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_VIDEO_STREAM_TYPE_LOOKUP);
}

inline std::string mavlink_video_stream_type_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_VIDEO_STREAM_TYPE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_VTOL_TRANSITION_HEADING_LOOKUP = {
    {0, "VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT"},
    {1, "VTOL_TRANSITION_HEADING_NEXT_WAYPOINT"},
    {2, "VTOL_TRANSITION_HEADING_TAKEOFF"},
    {3, "VTOL_TRANSITION_HEADING_SPECIFIED"},
    {4, "VTOL_TRANSITION_HEADING_ANY"},
};

inline const char* mavlink_vtol_transition_heading_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_VTOL_TRANSITION_HEADING_LOOKUP);
}

inline std::string mavlink_vtol_transition_heading_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_VTOL_TRANSITION_HEADING_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_WIFI_CONFIG_AP_MODE_LOOKUP = {
    {0, "WIFI_CONFIG_AP_MODE_UNDEFINED"},
    {1, "WIFI_CONFIG_AP_MODE_AP"},
    {2, "WIFI_CONFIG_AP_MODE_STATION"},
    {3, "WIFI_CONFIG_AP_MODE_DISABLED"},
};

inline const char* mavlink_wifi_config_ap_mode_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_WIFI_CONFIG_AP_MODE_LOOKUP);
}

inline std::string mavlink_wifi_config_ap_mode_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_WIFI_CONFIG_AP_MODE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_WIFI_CONFIG_AP_RESPONSE_LOOKUP = {
    {0, "WIFI_CONFIG_AP_RESPONSE_UNDEFINED"},
    {1, "WIFI_CONFIG_AP_RESPONSE_ACCEPTED"},
    {2, "WIFI_CONFIG_AP_RESPONSE_REJECTED"},
    {3, "WIFI_CONFIG_AP_RESPONSE_MODE_ERROR"},
    {4, "WIFI_CONFIG_AP_RESPONSE_SSID_ERROR"},
    {5, "WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR"},
};

inline const char* mavlink_wifi_config_ap_response_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_WIFI_CONFIG_AP_RESPONSE_LOOKUP);
}

inline std::string mavlink_wifi_config_ap_response_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_WIFI_CONFIG_AP_RESPONSE_LOOKUP);
}

static const std::unordered_map<uint32_t, const char*> MAVLINK_WINCH_ACTIONS_LOOKUP = {
    {0, "WINCH_RELAXED"},
    {1, "WINCH_RELATIVE_LENGTH_CONTROL"},
    {2, "WINCH_RATE_CONTROL"},
};

inline const char* mavlink_winch_actions_str(uint32_t val) {
    return mavlink_lookup(val, MAVLINK_WINCH_ACTIONS_LOOKUP);
}

inline std::string mavlink_winch_actions_pretty(uint32_t val) {
    return mavlink_pretty(val, MAVLINK_WINCH_ACTIONS_LOOKUP);
}

