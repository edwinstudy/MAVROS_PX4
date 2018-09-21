#pragma once
// MESSAGE FORMATION_TO_GCS PACKING

#define MAVLINK_MSG_ID_FORMATION_TO_GCS 238

MAVPACKED(
typedef struct __mavlink_formation_to_gcs_t {
 float dock_time; /*< docking time*/
 int8_t dock_status; /*< -1:dock failed, 0:dock timeout, 1:dock success*/
 int8_t status; /*< status, 0:on ground, 1:take off, 2:hover, 3:formation, 4:docking, 5:return, 6:land*/
}) mavlink_formation_to_gcs_t;

#define MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN 6
#define MAVLINK_MSG_ID_FORMATION_TO_GCS_MIN_LEN 6
#define MAVLINK_MSG_ID_238_LEN 6
#define MAVLINK_MSG_ID_238_MIN_LEN 6

#define MAVLINK_MSG_ID_FORMATION_TO_GCS_CRC 46
#define MAVLINK_MSG_ID_238_CRC 46



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FORMATION_TO_GCS { \
    238, \
    "FORMATION_TO_GCS", \
    3, \
    {  { "dock_time", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_formation_to_gcs_t, dock_time) }, \
         { "dock_status", NULL, MAVLINK_TYPE_INT8_T, 0, 4, offsetof(mavlink_formation_to_gcs_t, dock_status) }, \
         { "status", NULL, MAVLINK_TYPE_INT8_T, 0, 5, offsetof(mavlink_formation_to_gcs_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FORMATION_TO_GCS { \
    "FORMATION_TO_GCS", \
    3, \
    {  { "dock_time", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_formation_to_gcs_t, dock_time) }, \
         { "dock_status", NULL, MAVLINK_TYPE_INT8_T, 0, 4, offsetof(mavlink_formation_to_gcs_t, dock_status) }, \
         { "status", NULL, MAVLINK_TYPE_INT8_T, 0, 5, offsetof(mavlink_formation_to_gcs_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a formation_to_gcs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param dock_time docking time
 * @param dock_status -1:dock failed, 0:dock timeout, 1:dock success
 * @param status status, 0:on ground, 1:take off, 2:hover, 3:formation, 4:docking, 5:return, 6:land
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_to_gcs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float dock_time, int8_t dock_status, int8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN];
    _mav_put_float(buf, 0, dock_time);
    _mav_put_int8_t(buf, 4, dock_status);
    _mav_put_int8_t(buf, 5, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN);
#else
    mavlink_formation_to_gcs_t packet;
    packet.dock_time = dock_time;
    packet.dock_status = dock_status;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_TO_GCS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FORMATION_TO_GCS_MIN_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_CRC);
}

/**
 * @brief Pack a formation_to_gcs message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dock_time docking time
 * @param dock_status -1:dock failed, 0:dock timeout, 1:dock success
 * @param status status, 0:on ground, 1:take off, 2:hover, 3:formation, 4:docking, 5:return, 6:land
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_formation_to_gcs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float dock_time,int8_t dock_status,int8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN];
    _mav_put_float(buf, 0, dock_time);
    _mav_put_int8_t(buf, 4, dock_status);
    _mav_put_int8_t(buf, 5, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN);
#else
    mavlink_formation_to_gcs_t packet;
    packet.dock_time = dock_time;
    packet.dock_status = dock_status;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FORMATION_TO_GCS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FORMATION_TO_GCS_MIN_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_CRC);
}

/**
 * @brief Encode a formation_to_gcs struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param formation_to_gcs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_to_gcs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_formation_to_gcs_t* formation_to_gcs)
{
    return mavlink_msg_formation_to_gcs_pack(system_id, component_id, msg, formation_to_gcs->dock_time, formation_to_gcs->dock_status, formation_to_gcs->status);
}

/**
 * @brief Encode a formation_to_gcs struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param formation_to_gcs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_formation_to_gcs_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_formation_to_gcs_t* formation_to_gcs)
{
    return mavlink_msg_formation_to_gcs_pack_chan(system_id, component_id, chan, msg, formation_to_gcs->dock_time, formation_to_gcs->dock_status, formation_to_gcs->status);
}

/**
 * @brief Send a formation_to_gcs message
 * @param chan MAVLink channel to send the message
 *
 * @param dock_time docking time
 * @param dock_status -1:dock failed, 0:dock timeout, 1:dock success
 * @param status status, 0:on ground, 1:take off, 2:hover, 3:formation, 4:docking, 5:return, 6:land
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_formation_to_gcs_send(mavlink_channel_t chan, float dock_time, int8_t dock_status, int8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN];
    _mav_put_float(buf, 0, dock_time);
    _mav_put_int8_t(buf, 4, dock_status);
    _mav_put_int8_t(buf, 5, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_TO_GCS, buf, MAVLINK_MSG_ID_FORMATION_TO_GCS_MIN_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_CRC);
#else
    mavlink_formation_to_gcs_t packet;
    packet.dock_time = dock_time;
    packet.dock_status = dock_status;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_TO_GCS, (const char *)&packet, MAVLINK_MSG_ID_FORMATION_TO_GCS_MIN_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_CRC);
#endif
}

/**
 * @brief Send a formation_to_gcs message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_formation_to_gcs_send_struct(mavlink_channel_t chan, const mavlink_formation_to_gcs_t* formation_to_gcs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_formation_to_gcs_send(chan, formation_to_gcs->dock_time, formation_to_gcs->dock_status, formation_to_gcs->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_TO_GCS, (const char *)formation_to_gcs, MAVLINK_MSG_ID_FORMATION_TO_GCS_MIN_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_CRC);
#endif
}

#if MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_formation_to_gcs_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float dock_time, int8_t dock_status, int8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, dock_time);
    _mav_put_int8_t(buf, 4, dock_status);
    _mav_put_int8_t(buf, 5, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_TO_GCS, buf, MAVLINK_MSG_ID_FORMATION_TO_GCS_MIN_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_CRC);
#else
    mavlink_formation_to_gcs_t *packet = (mavlink_formation_to_gcs_t *)msgbuf;
    packet->dock_time = dock_time;
    packet->dock_status = dock_status;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FORMATION_TO_GCS, (const char *)packet, MAVLINK_MSG_ID_FORMATION_TO_GCS_MIN_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN, MAVLINK_MSG_ID_FORMATION_TO_GCS_CRC);
#endif
}
#endif

#endif

// MESSAGE FORMATION_TO_GCS UNPACKING


/**
 * @brief Get field dock_time from formation_to_gcs message
 *
 * @return docking time
 */
static inline float mavlink_msg_formation_to_gcs_get_dock_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field dock_status from formation_to_gcs message
 *
 * @return -1:dock failed, 0:dock timeout, 1:dock success
 */
static inline int8_t mavlink_msg_formation_to_gcs_get_dock_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  4);
}

/**
 * @brief Get field status from formation_to_gcs message
 *
 * @return status, 0:on ground, 1:take off, 2:hover, 3:formation, 4:docking, 5:return, 6:land
 */
static inline int8_t mavlink_msg_formation_to_gcs_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  5);
}

/**
 * @brief Decode a formation_to_gcs message into a struct
 *
 * @param msg The message to decode
 * @param formation_to_gcs C-struct to decode the message contents into
 */
static inline void mavlink_msg_formation_to_gcs_decode(const mavlink_message_t* msg, mavlink_formation_to_gcs_t* formation_to_gcs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    formation_to_gcs->dock_time = mavlink_msg_formation_to_gcs_get_dock_time(msg);
    formation_to_gcs->dock_status = mavlink_msg_formation_to_gcs_get_dock_status(msg);
    formation_to_gcs->status = mavlink_msg_formation_to_gcs_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN? msg->len : MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN;
        memset(formation_to_gcs, 0, MAVLINK_MSG_ID_FORMATION_TO_GCS_LEN);
    memcpy(formation_to_gcs, _MAV_PAYLOAD(msg), len);
#endif
}
