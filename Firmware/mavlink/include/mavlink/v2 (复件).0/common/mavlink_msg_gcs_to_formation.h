#pragma once
// MESSAGE GCS_TO_FORMATION PACKING

#define MAVLINK_MSG_ID_GCS_TO_FORMATION 239

MAVPACKED(
typedef struct __mavlink_gcs_to_formation_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 int32_t lon; /*< WGS84 lontitude, expressed as degrees * 1E7*/
 int32_t lat; /*< WGS84 latitude, expressed as degrees * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
}) mavlink_gcs_to_formation_t;

#define MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN 20
#define MAVLINK_MSG_ID_GCS_TO_FORMATION_MIN_LEN 20
#define MAVLINK_MSG_ID_239_LEN 20
#define MAVLINK_MSG_ID_239_MIN_LEN 20

#define MAVLINK_MSG_ID_GCS_TO_FORMATION_CRC 115
#define MAVLINK_MSG_ID_239_CRC 115



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GCS_TO_FORMATION { \
    239, \
    "GCS_TO_FORMATION", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gcs_to_formation_t, time_usec) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gcs_to_formation_t, lon) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gcs_to_formation_t, lat) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gcs_to_formation_t, alt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GCS_TO_FORMATION { \
    "GCS_TO_FORMATION", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gcs_to_formation_t, time_usec) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gcs_to_formation_t, lon) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gcs_to_formation_t, lat) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gcs_to_formation_t, alt) }, \
         } \
}
#endif

/**
 * @brief Pack a gcs_to_formation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param lon WGS84 lontitude, expressed as degrees * 1E7
 * @param lat WGS84 latitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_formation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, int32_t lon, int32_t lat, int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN);
#else
    mavlink_gcs_to_formation_t packet;
    packet.time_usec = time_usec;
    packet.lon = lon;
    packet.lat = lat;
    packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_FORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_TO_FORMATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_CRC);
}

/**
 * @brief Pack a gcs_to_formation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param lon WGS84 lontitude, expressed as degrees * 1E7
 * @param lat WGS84 latitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_to_formation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,int32_t lon,int32_t lat,int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN);
#else
    mavlink_gcs_to_formation_t packet;
    packet.time_usec = time_usec;
    packet.lon = lon;
    packet.lat = lat;
    packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_TO_FORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_TO_FORMATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_CRC);
}

/**
 * @brief Encode a gcs_to_formation struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_formation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_formation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gcs_to_formation_t* gcs_to_formation)
{
    return mavlink_msg_gcs_to_formation_pack(system_id, component_id, msg, gcs_to_formation->time_usec, gcs_to_formation->lon, gcs_to_formation->lat, gcs_to_formation->alt);
}

/**
 * @brief Encode a gcs_to_formation struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_to_formation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_to_formation_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gcs_to_formation_t* gcs_to_formation)
{
    return mavlink_msg_gcs_to_formation_pack_chan(system_id, component_id, chan, msg, gcs_to_formation->time_usec, gcs_to_formation->lon, gcs_to_formation->lat, gcs_to_formation->alt);
}

/**
 * @brief Send a gcs_to_formation message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param lon WGS84 lontitude, expressed as degrees * 1E7
 * @param lat WGS84 latitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gcs_to_formation_send(mavlink_channel_t chan, uint64_t time_usec, int32_t lon, int32_t lat, int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_FORMATION, buf, MAVLINK_MSG_ID_GCS_TO_FORMATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_CRC);
#else
    mavlink_gcs_to_formation_t packet;
    packet.time_usec = time_usec;
    packet.lon = lon;
    packet.lat = lat;
    packet.alt = alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_FORMATION, (const char *)&packet, MAVLINK_MSG_ID_GCS_TO_FORMATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_CRC);
#endif
}

/**
 * @brief Send a gcs_to_formation message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gcs_to_formation_send_struct(mavlink_channel_t chan, const mavlink_gcs_to_formation_t* gcs_to_formation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gcs_to_formation_send(chan, gcs_to_formation->time_usec, gcs_to_formation->lon, gcs_to_formation->lat, gcs_to_formation->alt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_FORMATION, (const char *)gcs_to_formation, MAVLINK_MSG_ID_GCS_TO_FORMATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gcs_to_formation_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, int32_t lon, int32_t lat, int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_FORMATION, buf, MAVLINK_MSG_ID_GCS_TO_FORMATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_CRC);
#else
    mavlink_gcs_to_formation_t *packet = (mavlink_gcs_to_formation_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->lon = lon;
    packet->lat = lat;
    packet->alt = alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_TO_FORMATION, (const char *)packet, MAVLINK_MSG_ID_GCS_TO_FORMATION_MIN_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN, MAVLINK_MSG_ID_GCS_TO_FORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE GCS_TO_FORMATION UNPACKING


/**
 * @brief Get field time_usec from gcs_to_formation message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_gcs_to_formation_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lon from gcs_to_formation message
 *
 * @return WGS84 lontitude, expressed as degrees * 1E7
 */
static inline int32_t mavlink_msg_gcs_to_formation_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lat from gcs_to_formation message
 *
 * @return WGS84 latitude, expressed as degrees * 1E7
 */
static inline int32_t mavlink_msg_gcs_to_formation_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from gcs_to_formation message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 */
static inline int32_t mavlink_msg_gcs_to_formation_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Decode a gcs_to_formation message into a struct
 *
 * @param msg The message to decode
 * @param gcs_to_formation C-struct to decode the message contents into
 */
static inline void mavlink_msg_gcs_to_formation_decode(const mavlink_message_t* msg, mavlink_gcs_to_formation_t* gcs_to_formation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gcs_to_formation->time_usec = mavlink_msg_gcs_to_formation_get_time_usec(msg);
    gcs_to_formation->lon = mavlink_msg_gcs_to_formation_get_lon(msg);
    gcs_to_formation->lat = mavlink_msg_gcs_to_formation_get_lat(msg);
    gcs_to_formation->alt = mavlink_msg_gcs_to_formation_get_alt(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN? msg->len : MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN;
        memset(gcs_to_formation, 0, MAVLINK_MSG_ID_GCS_TO_FORMATION_LEN);
    memcpy(gcs_to_formation, _MAV_PAYLOAD(msg), len);
#endif
}
