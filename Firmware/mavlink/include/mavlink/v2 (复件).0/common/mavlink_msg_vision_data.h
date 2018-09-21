#pragma once
// MESSAGE VISION_DATA PACKING

#define MAVLINK_MSG_ID_VISION_DATA 236

MAVPACKED(
typedef struct __mavlink_vision_data_t {
 float vision_x; /*< x position while dual camera is valid*/
 float vision_y; /*< y position while dual camera is valid*/
 float vision_z; /*< z position while dual camera is valid*/
 float vision_vx; /*< vx while dual camera is valid*/
 float vision_vy; /*< vy while dual camera is valid*/
 float vision_vz; /*< vz while dual camera is valid*/
 uint32_t pixel_x; /*< target is x pixels away from the center of a picture. It is only used while dual camera is invalid.*/
 uint32_t pixel_y; /*< target is y pixels away from the center of a picture. It is only used while dual camera is invalid.*/
 int8_t status; /*< is the vision data valid. 0: invalid, 1:valid*/
}) mavlink_vision_data_t;

#define MAVLINK_MSG_ID_VISION_DATA_LEN 33
#define MAVLINK_MSG_ID_VISION_DATA_MIN_LEN 33
#define MAVLINK_MSG_ID_236_LEN 33
#define MAVLINK_MSG_ID_236_MIN_LEN 33

#define MAVLINK_MSG_ID_VISION_DATA_CRC 103
#define MAVLINK_MSG_ID_236_CRC 103



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_DATA { \
    236, \
    "VISION_DATA", \
    9, \
    {  { "vision_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_vision_data_t, vision_x) }, \
         { "vision_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_vision_data_t, vision_y) }, \
         { "vision_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_data_t, vision_z) }, \
         { "vision_vx", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_data_t, vision_vx) }, \
         { "vision_vy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_data_t, vision_vy) }, \
         { "vision_vz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_vision_data_t, vision_vz) }, \
         { "pixel_x", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_vision_data_t, pixel_x) }, \
         { "pixel_y", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_vision_data_t, pixel_y) }, \
         { "status", NULL, MAVLINK_TYPE_INT8_T, 0, 32, offsetof(mavlink_vision_data_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_DATA { \
    "VISION_DATA", \
    9, \
    {  { "vision_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_vision_data_t, vision_x) }, \
         { "vision_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_vision_data_t, vision_y) }, \
         { "vision_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_data_t, vision_z) }, \
         { "vision_vx", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_data_t, vision_vx) }, \
         { "vision_vy", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_data_t, vision_vy) }, \
         { "vision_vz", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_vision_data_t, vision_vz) }, \
         { "pixel_x", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_vision_data_t, pixel_x) }, \
         { "pixel_y", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_vision_data_t, pixel_y) }, \
         { "status", NULL, MAVLINK_TYPE_INT8_T, 0, 32, offsetof(mavlink_vision_data_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vision_x x position while dual camera is valid
 * @param vision_y y position while dual camera is valid
 * @param vision_z z position while dual camera is valid
 * @param vision_vx vx while dual camera is valid
 * @param vision_vy vy while dual camera is valid
 * @param vision_vz vz while dual camera is valid
 * @param pixel_x target is x pixels away from the center of a picture. It is only used while dual camera is invalid.
 * @param pixel_y target is y pixels away from the center of a picture. It is only used while dual camera is invalid.
 * @param status is the vision data valid. 0: invalid, 1:valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float vision_x, float vision_y, float vision_z, float vision_vx, float vision_vy, float vision_vz, uint32_t pixel_x, uint32_t pixel_y, int8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_DATA_LEN];
    _mav_put_float(buf, 0, vision_x);
    _mav_put_float(buf, 4, vision_y);
    _mav_put_float(buf, 8, vision_z);
    _mav_put_float(buf, 12, vision_vx);
    _mav_put_float(buf, 16, vision_vy);
    _mav_put_float(buf, 20, vision_vz);
    _mav_put_uint32_t(buf, 24, pixel_x);
    _mav_put_uint32_t(buf, 28, pixel_y);
    _mav_put_int8_t(buf, 32, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_DATA_LEN);
#else
    mavlink_vision_data_t packet;
    packet.vision_x = vision_x;
    packet.vision_y = vision_y;
    packet.vision_z = vision_z;
    packet.vision_vx = vision_vx;
    packet.vision_vy = vision_vy;
    packet.vision_vz = vision_vz;
    packet.pixel_x = pixel_x;
    packet.pixel_y = pixel_y;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_DATA_MIN_LEN, MAVLINK_MSG_ID_VISION_DATA_LEN, MAVLINK_MSG_ID_VISION_DATA_CRC);
}

/**
 * @brief Pack a vision_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_x x position while dual camera is valid
 * @param vision_y y position while dual camera is valid
 * @param vision_z z position while dual camera is valid
 * @param vision_vx vx while dual camera is valid
 * @param vision_vy vy while dual camera is valid
 * @param vision_vz vz while dual camera is valid
 * @param pixel_x target is x pixels away from the center of a picture. It is only used while dual camera is invalid.
 * @param pixel_y target is y pixels away from the center of a picture. It is only used while dual camera is invalid.
 * @param status is the vision data valid. 0: invalid, 1:valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float vision_x,float vision_y,float vision_z,float vision_vx,float vision_vy,float vision_vz,uint32_t pixel_x,uint32_t pixel_y,int8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_DATA_LEN];
    _mav_put_float(buf, 0, vision_x);
    _mav_put_float(buf, 4, vision_y);
    _mav_put_float(buf, 8, vision_z);
    _mav_put_float(buf, 12, vision_vx);
    _mav_put_float(buf, 16, vision_vy);
    _mav_put_float(buf, 20, vision_vz);
    _mav_put_uint32_t(buf, 24, pixel_x);
    _mav_put_uint32_t(buf, 28, pixel_y);
    _mav_put_int8_t(buf, 32, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_DATA_LEN);
#else
    mavlink_vision_data_t packet;
    packet.vision_x = vision_x;
    packet.vision_y = vision_y;
    packet.vision_z = vision_z;
    packet.vision_vx = vision_vx;
    packet.vision_vy = vision_vy;
    packet.vision_vz = vision_vz;
    packet.pixel_x = pixel_x;
    packet.pixel_y = pixel_y;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_DATA_MIN_LEN, MAVLINK_MSG_ID_VISION_DATA_LEN, MAVLINK_MSG_ID_VISION_DATA_CRC);
}

/**
 * @brief Encode a vision_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_data_t* vision_data)
{
    return mavlink_msg_vision_data_pack(system_id, component_id, msg, vision_data->vision_x, vision_data->vision_y, vision_data->vision_z, vision_data->vision_vx, vision_data->vision_vy, vision_data->vision_vz, vision_data->pixel_x, vision_data->pixel_y, vision_data->status);
}

/**
 * @brief Encode a vision_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_data_t* vision_data)
{
    return mavlink_msg_vision_data_pack_chan(system_id, component_id, chan, msg, vision_data->vision_x, vision_data->vision_y, vision_data->vision_z, vision_data->vision_vx, vision_data->vision_vy, vision_data->vision_vz, vision_data->pixel_x, vision_data->pixel_y, vision_data->status);
}

/**
 * @brief Send a vision_data message
 * @param chan MAVLink channel to send the message
 *
 * @param vision_x x position while dual camera is valid
 * @param vision_y y position while dual camera is valid
 * @param vision_z z position while dual camera is valid
 * @param vision_vx vx while dual camera is valid
 * @param vision_vy vy while dual camera is valid
 * @param vision_vz vz while dual camera is valid
 * @param pixel_x target is x pixels away from the center of a picture. It is only used while dual camera is invalid.
 * @param pixel_y target is y pixels away from the center of a picture. It is only used while dual camera is invalid.
 * @param status is the vision data valid. 0: invalid, 1:valid
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_data_send(mavlink_channel_t chan, float vision_x, float vision_y, float vision_z, float vision_vx, float vision_vy, float vision_vz, uint32_t pixel_x, uint32_t pixel_y, int8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISION_DATA_LEN];
    _mav_put_float(buf, 0, vision_x);
    _mav_put_float(buf, 4, vision_y);
    _mav_put_float(buf, 8, vision_z);
    _mav_put_float(buf, 12, vision_vx);
    _mav_put_float(buf, 16, vision_vy);
    _mav_put_float(buf, 20, vision_vz);
    _mav_put_uint32_t(buf, 24, pixel_x);
    _mav_put_uint32_t(buf, 28, pixel_y);
    _mav_put_int8_t(buf, 32, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_DATA, buf, MAVLINK_MSG_ID_VISION_DATA_MIN_LEN, MAVLINK_MSG_ID_VISION_DATA_LEN, MAVLINK_MSG_ID_VISION_DATA_CRC);
#else
    mavlink_vision_data_t packet;
    packet.vision_x = vision_x;
    packet.vision_y = vision_y;
    packet.vision_z = vision_z;
    packet.vision_vx = vision_vx;
    packet.vision_vy = vision_vy;
    packet.vision_vz = vision_vz;
    packet.pixel_x = pixel_x;
    packet.pixel_y = pixel_y;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_DATA, (const char *)&packet, MAVLINK_MSG_ID_VISION_DATA_MIN_LEN, MAVLINK_MSG_ID_VISION_DATA_LEN, MAVLINK_MSG_ID_VISION_DATA_CRC);
#endif
}

/**
 * @brief Send a vision_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_data_send_struct(mavlink_channel_t chan, const mavlink_vision_data_t* vision_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_data_send(chan, vision_data->vision_x, vision_data->vision_y, vision_data->vision_z, vision_data->vision_vx, vision_data->vision_vy, vision_data->vision_vz, vision_data->pixel_x, vision_data->pixel_y, vision_data->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_DATA, (const char *)vision_data, MAVLINK_MSG_ID_VISION_DATA_MIN_LEN, MAVLINK_MSG_ID_VISION_DATA_LEN, MAVLINK_MSG_ID_VISION_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float vision_x, float vision_y, float vision_z, float vision_vx, float vision_vy, float vision_vz, uint32_t pixel_x, uint32_t pixel_y, int8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vision_x);
    _mav_put_float(buf, 4, vision_y);
    _mav_put_float(buf, 8, vision_z);
    _mav_put_float(buf, 12, vision_vx);
    _mav_put_float(buf, 16, vision_vy);
    _mav_put_float(buf, 20, vision_vz);
    _mav_put_uint32_t(buf, 24, pixel_x);
    _mav_put_uint32_t(buf, 28, pixel_y);
    _mav_put_int8_t(buf, 32, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_DATA, buf, MAVLINK_MSG_ID_VISION_DATA_MIN_LEN, MAVLINK_MSG_ID_VISION_DATA_LEN, MAVLINK_MSG_ID_VISION_DATA_CRC);
#else
    mavlink_vision_data_t *packet = (mavlink_vision_data_t *)msgbuf;
    packet->vision_x = vision_x;
    packet->vision_y = vision_y;
    packet->vision_z = vision_z;
    packet->vision_vx = vision_vx;
    packet->vision_vy = vision_vy;
    packet->vision_vz = vision_vz;
    packet->pixel_x = pixel_x;
    packet->pixel_y = pixel_y;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_DATA, (const char *)packet, MAVLINK_MSG_ID_VISION_DATA_MIN_LEN, MAVLINK_MSG_ID_VISION_DATA_LEN, MAVLINK_MSG_ID_VISION_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_DATA UNPACKING


/**
 * @brief Get field vision_x from vision_data message
 *
 * @return x position while dual camera is valid
 */
static inline float mavlink_msg_vision_data_get_vision_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vision_y from vision_data message
 *
 * @return y position while dual camera is valid
 */
static inline float mavlink_msg_vision_data_get_vision_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vision_z from vision_data message
 *
 * @return z position while dual camera is valid
 */
static inline float mavlink_msg_vision_data_get_vision_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vision_vx from vision_data message
 *
 * @return vx while dual camera is valid
 */
static inline float mavlink_msg_vision_data_get_vision_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vision_vy from vision_data message
 *
 * @return vy while dual camera is valid
 */
static inline float mavlink_msg_vision_data_get_vision_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vision_vz from vision_data message
 *
 * @return vz while dual camera is valid
 */
static inline float mavlink_msg_vision_data_get_vision_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pixel_x from vision_data message
 *
 * @return target is x pixels away from the center of a picture. It is only used while dual camera is invalid.
 */
static inline uint32_t mavlink_msg_vision_data_get_pixel_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field pixel_y from vision_data message
 *
 * @return target is y pixels away from the center of a picture. It is only used while dual camera is invalid.
 */
static inline uint32_t mavlink_msg_vision_data_get_pixel_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field status from vision_data message
 *
 * @return is the vision data valid. 0: invalid, 1:valid
 */
static inline int8_t mavlink_msg_vision_data_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  32);
}

/**
 * @brief Decode a vision_data message into a struct
 *
 * @param msg The message to decode
 * @param vision_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_data_decode(const mavlink_message_t* msg, mavlink_vision_data_t* vision_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    vision_data->vision_x = mavlink_msg_vision_data_get_vision_x(msg);
    vision_data->vision_y = mavlink_msg_vision_data_get_vision_y(msg);
    vision_data->vision_z = mavlink_msg_vision_data_get_vision_z(msg);
    vision_data->vision_vx = mavlink_msg_vision_data_get_vision_vx(msg);
    vision_data->vision_vy = mavlink_msg_vision_data_get_vision_vy(msg);
    vision_data->vision_vz = mavlink_msg_vision_data_get_vision_vz(msg);
    vision_data->pixel_x = mavlink_msg_vision_data_get_pixel_x(msg);
    vision_data->pixel_y = mavlink_msg_vision_data_get_pixel_y(msg);
    vision_data->status = mavlink_msg_vision_data_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_DATA_LEN? msg->len : MAVLINK_MSG_ID_VISION_DATA_LEN;
        memset(vision_data, 0, MAVLINK_MSG_ID_VISION_DATA_LEN);
    memcpy(vision_data, _MAV_PAYLOAD(msg), len);
#endif
}
