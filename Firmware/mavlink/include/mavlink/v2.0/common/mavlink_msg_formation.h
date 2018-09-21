#pragma once
// MESSAGE FORMATION PACKING

#define MAVLINK_MSG_ID_FORMATION 237

MAVPACKED(
typedef struct __mavlink_formation_t {
 int32_t lon; /*< WGS84 lontitude, expressed as degrees * 1E7*/
 int32_t lat; /*< WGS84 latitude, expressed as degrees * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
 float vel_n; /*< velocity in north, expressed as m/s*/
 float vel_e; /*< velocity in east, expressed as m/s*/
 float vel_d; /*< velocity down, expressed as m/s*/
 int8_t status; /*< status, 0:on ground, 1:take off, 2:hover, 3:formation, 4:docking, 5:return, 6:land*/
 int8_t kill; /*< kill 0:normal, 1:kill(stop motors)*/
}) mavlink_formation_t;

#define MAVLINK_MSG_ID_FORMATION_LEN 26
#define MAVLINK_MSG_ID_FORMATION_MIN_LEN 26
#define MAVLINK_MSG_ID_237_LEN 26
#define MAVLINK_MSG_ID_237_MIN_LEN 26

#define MAVLINK_MSG_ID_FORMATION_CRC 44
#define MAVLINK_MSG_ID_237_CRC 44



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FORMATION { \
    237, \
    "FORMATION", \
    8, \
    {  { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_formation_t, lon) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_formation_t, lat) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_formation_t, alt) }, \
         { "vel_n", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_formation_t, vel_n) }, \
         { "vel_e", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_formation_t, vel_e) }, \
         { "vel_d", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_formation_t, vel_d) }, \
         { "status", NULL, MAVLINK_TYPE_INT8_T, 0, 24, offsetof(mavlink_formation_t, status) }, \
         { "kill", NULL, MAVLINK_TYPE_INT8_T, 0, 25, offsetof(mavlink_formation_t, kill) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FORMATION { \
    "FORMATION", \
    8, \
    {  { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_formation_t, lon) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_formation_t, lat) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_formation_t, alt) }, \
         { "vel_n", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_formation_t, vel_n) }, \
         { "vel_e", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_formation_t, vel_e) }, \
         { "vel_d", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_formation_t, vel_d) }, \
         { "status", NULL, MAVLINK_TYPE_INT8_T, 0, 24, offsetof(mavlink_formation_t, status) }, \
         { "kill", NULL, MAVLINK_TYPE_INT8_T, 0, 25, offsetof(mavlink_formation_t, kill) }, \
         } \
}
#endif

/**
 * @brief Pack a formation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lon WGS84 lontitude, expressed as degrees * 1E7
 * @param lat WGS84 latitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param vel_n velocity in north, expressed as m/s
 * @param vel_e velocity in east, expressed as m/s
 * @param vel_d velocity down, expressed as m/s
 * @param status status, 0:on ground, 1:take off, 2:hover, 3:formation, 4:docking, 5:return, 6:land
 * @param kill kill 0:normal, 1:kill(stop motors)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lon, int32_t lat, int32_t alt, float vel_n, float vel_e, float vel_d, int8_t status, int8_t kill)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_LEN];
    _mav_put_int32_t(buf, 0, lon);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_float(buf, 12, vel_n);
    _mav_put_float(buf, 16, vel_e);
    _mav_put_float(buf, 20, vel_d);
    _mav_put_int8_t(buf, 24, status);
    _mav_put_int8_t(buf, 25, kill);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_LEN);
#else
    mavlink_formation_t packet;
    packet.lon = lon;
    packet.lat = lat;
    packet.alt = alt;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.status = status;
    packet.kill = kill;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FORMATION_MIN_LEN, MAVLINK_MSG_ID_FORMATION_LEN, MAVLINK_MSG_ID_FORMATION_CRC);
}

/**
 * @brief Pack a formation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lon WGS84 lontitude, expressed as degrees * 1E7
 * @param lat WGS84 latitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param vel_n velocity in north, expressed as m/s
 * @param vel_e velocity in east, expressed as m/s
 * @param vel_d velocity down, expressed as m/s
 * @param status status, 0:on ground, 1:take off, 2:hover, 3:formation, 4:docking, 5:return, 6:land
 * @param kill kill 0:normal, 1:kill(stop motors)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lon,int32_t lat,int32_t alt,float vel_n,float vel_e,float vel_d,int8_t status,int8_t kill)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_LEN];
    _mav_put_int32_t(buf, 0, lon);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_float(buf, 12, vel_n);
    _mav_put_float(buf, 16, vel_e);
    _mav_put_float(buf, 20, vel_d);
    _mav_put_int8_t(buf, 24, status);
    _mav_put_int8_t(buf, 25, kill);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_LEN);
#else
    mavlink_formation_t packet;
    packet.lon = lon;
    packet.lat = lat;
    packet.alt = alt;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.status = status;
    packet.kill = kill;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FORMATION_MIN_LEN, MAVLINK_MSG_ID_FORMATION_LEN, MAVLINK_MSG_ID_FORMATION_CRC);
}

/**
 * @brief Encode a formation struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param formation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_formation_t* formation)
{
    return mavlink_msg_formation_pack(system_id, component_id, msg, formation->lon, formation->lat, formation->alt, formation->vel_n, formation->vel_e, formation->vel_d, formation->status, formation->kill);
}

/**
 * @brief Encode a formation struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param formation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_formation_t* formation)
{
    return mavlink_msg_formation_pack_chan(system_id, component_id, chan, msg, formation->lon, formation->lat, formation->alt, formation->vel_n, formation->vel_e, formation->vel_d, formation->status, formation->kill);
}

/**
 * @brief Send a formation message
 * @param chan MAVLink channel to send the message
 *
 * @param lon WGS84 lontitude, expressed as degrees * 1E7
 * @param lat WGS84 latitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param vel_n velocity in north, expressed as m/s
 * @param vel_e velocity in east, expressed as m/s
 * @param vel_d velocity down, expressed as m/s
 * @param status status, 0:on ground, 1:take off, 2:hover, 3:formation, 4:docking, 5:return, 6:land
 * @param kill kill 0:normal, 1:kill(stop motors)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_formation_send(mavlink_channel_t chan, int32_t lon, int32_t lat, int32_t alt, float vel_n, float vel_e, float vel_d, int8_t status, int8_t kill)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_LEN];
    _mav_put_int32_t(buf, 0, lon);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_float(buf, 12, vel_n);
    _mav_put_float(buf, 16, vel_e);
    _mav_put_float(buf, 20, vel_d);
    _mav_put_int8_t(buf, 24, status);
    _mav_put_int8_t(buf, 25, kill);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION, buf, MAVLINK_MSG_ID_FORMATION_MIN_LEN, MAVLINK_MSG_ID_FORMATION_LEN, MAVLINK_MSG_ID_FORMATION_CRC);
#else
    mavlink_formation_t packet;
    packet.lon = lon;
    packet.lat = lat;
    packet.alt = alt;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.status = status;
    packet.kill = kill;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION, (const char *)&packet, MAVLINK_MSG_ID_FORMATION_MIN_LEN, MAVLINK_MSG_ID_FORMATION_LEN, MAVLINK_MSG_ID_FORMATION_CRC);
#endif
}

/**
 * @brief Send a formation message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_formation_send_struct(mavlink_channel_t chan, const mavlink_formation_t* formation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_formation_send(chan, formation->lon, formation->lat, formation->alt, formation->vel_n, formation->vel_e, formation->vel_d, formation->status, formation->kill);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION, (const char *)formation, MAVLINK_MSG_ID_FORMATION_MIN_LEN, MAVLINK_MSG_ID_FORMATION_LEN, MAVLINK_MSG_ID_FORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_FORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_formation_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lon, int32_t lat, int32_t alt, float vel_n, float vel_e, float vel_d, int8_t status, int8_t kill)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lon);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_float(buf, 12, vel_n);
    _mav_put_float(buf, 16, vel_e);
    _mav_put_float(buf, 20, vel_d);
    _mav_put_int8_t(buf, 24, status);
    _mav_put_int8_t(buf, 25, kill);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION, buf, MAVLINK_MSG_ID_FORMATION_MIN_LEN, MAVLINK_MSG_ID_FORMATION_LEN, MAVLINK_MSG_ID_FORMATION_CRC);
#else
    mavlink_formation_t *packet = (mavlink_formation_t *)msgbuf;
    packet->lon = lon;
    packet->lat = lat;
    packet->alt = alt;
    packet->vel_n = vel_n;
    packet->vel_e = vel_e;
    packet->vel_d = vel_d;
    packet->status = status;
    packet->kill = kill;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION, (const char *)packet, MAVLINK_MSG_ID_FORMATION_MIN_LEN, MAVLINK_MSG_ID_FORMATION_LEN, MAVLINK_MSG_ID_FORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE FORMATION UNPACKING


/**
 * @brief Get field lon from formation message
 *
 * @return WGS84 lontitude, expressed as degrees * 1E7
 */
static inline int32_t mavlink_msg_formation_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lat from formation message
 *
 * @return WGS84 latitude, expressed as degrees * 1E7
 */
static inline int32_t mavlink_msg_formation_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from formation message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 */
static inline int32_t mavlink_msg_formation_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field vel_n from formation message
 *
 * @return velocity in north, expressed as m/s
 */
static inline float mavlink_msg_formation_get_vel_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vel_e from formation message
 *
 * @return velocity in east, expressed as m/s
 */
static inline float mavlink_msg_formation_get_vel_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vel_d from formation message
 *
 * @return velocity down, expressed as m/s
 */
static inline float mavlink_msg_formation_get_vel_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field status from formation message
 *
 * @return status, 0:on ground, 1:take off, 2:hover, 3:formation, 4:docking, 5:return, 6:land
 */
static inline int8_t mavlink_msg_formation_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  24);
}

/**
 * @brief Get field kill from formation message
 *
 * @return kill 0:normal, 1:kill(stop motors)
 */
static inline int8_t mavlink_msg_formation_get_kill(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  25);
}

/**
 * @brief Decode a formation message into a struct
 *
 * @param msg The message to decode
 * @param formation C-struct to decode the message contents into
 */
static inline void mavlink_msg_formation_decode(const mavlink_message_t* msg, mavlink_formation_t* formation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    formation->lon = mavlink_msg_formation_get_lon(msg);
    formation->lat = mavlink_msg_formation_get_lat(msg);
    formation->alt = mavlink_msg_formation_get_alt(msg);
    formation->vel_n = mavlink_msg_formation_get_vel_n(msg);
    formation->vel_e = mavlink_msg_formation_get_vel_e(msg);
    formation->vel_d = mavlink_msg_formation_get_vel_d(msg);
    formation->status = mavlink_msg_formation_get_status(msg);
    formation->kill = mavlink_msg_formation_get_kill(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FORMATION_LEN? msg->len : MAVLINK_MSG_ID_FORMATION_LEN;
        memset(formation, 0, MAVLINK_MSG_ID_FORMATION_LEN);
    memcpy(formation, _MAV_PAYLOAD(msg), len);
#endif
}
