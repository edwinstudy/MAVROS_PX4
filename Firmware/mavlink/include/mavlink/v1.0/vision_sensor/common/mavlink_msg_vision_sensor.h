// MESSAGE VISION_SENSOR PACKING

#define MAVLINK_MSG_ID_VISION_SENSOR 156

MAVPACKED(
typedef struct __mavlink_vision_sensor_t {
 float vision_x; /*< vision x velocity(m/s). */
 float vision_y; /*< vision y velocity(m/s). */
 float vision_z; /*< vision z velocity(m/s).*/
 float vision_distortion_x; /*< vision y velocity(m/s). */
 float vision_distortion_y; /*< vision z velocity(m/s).*/
}) mavlink_vision_sensor_t;

#define MAVLINK_MSG_ID_VISION_SENSOR_LEN 20
#define MAVLINK_MSG_ID_VISION_SENSOR_MIN_LEN 20
#define MAVLINK_MSG_ID_156_LEN 20
#define MAVLINK_MSG_ID_156_MIN_LEN 20

#define MAVLINK_MSG_ID_VISION_SENSOR_CRC 154
#define MAVLINK_MSG_ID_156_CRC 154



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_SENSOR { \
	156, \
	"VISION_SENSOR", \
	5, \
	{  { "vision_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_vision_sensor_t, vision_x) }, \
         { "vision_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_vision_sensor_t, vision_y) }, \
         { "vision_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_sensor_t, vision_z) }, \
         { "vision_distortion_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_sensor_t, vision_distortion_x) }, \
         { "vision_distortion_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_sensor_t, vision_distortion_y) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_SENSOR { \
	"VISION_SENSOR", \
	5, \
	{  { "vision_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_vision_sensor_t, vision_x) }, \
         { "vision_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_vision_sensor_t, vision_y) }, \
         { "vision_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_sensor_t, vision_z) }, \
         { "vision_distortion_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_sensor_t, vision_distortion_x) }, \
         { "vision_distortion_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_sensor_t, vision_distortion_y) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vision_x vision x velocity(m/s). 
 * @param vision_y vision y velocity(m/s). 
 * @param vision_z vision z velocity(m/s).
 * @param vision_distortion_x vision y velocity(m/s). 
 * @param vision_distortion_y vision z velocity(m/s).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float vision_x, float vision_y, float vision_z, float vision_distortion_x, float vision_distortion_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_SENSOR_LEN];
	_mav_put_float(buf, 0, vision_x);
	_mav_put_float(buf, 4, vision_y);
	_mav_put_float(buf, 8, vision_z);
	_mav_put_float(buf, 12, vision_distortion_x);
	_mav_put_float(buf, 16, vision_distortion_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_SENSOR_LEN);
#else
	mavlink_vision_sensor_t packet;
	packet.vision_x = vision_x;
	packet.vision_y = vision_y;
	packet.vision_z = vision_z;
	packet.vision_distortion_x = vision_distortion_x;
	packet.vision_distortion_y = vision_distortion_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_SENSOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_SENSOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VISION_SENSOR_LEN, MAVLINK_MSG_ID_VISION_SENSOR_CRC);
}

/**
 * @brief Pack a vision_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_x vision x velocity(m/s). 
 * @param vision_y vision y velocity(m/s). 
 * @param vision_z vision z velocity(m/s).
 * @param vision_distortion_x vision y velocity(m/s). 
 * @param vision_distortion_y vision z velocity(m/s).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float vision_x,float vision_y,float vision_z,float vision_distortion_x,float vision_distortion_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_SENSOR_LEN];
	_mav_put_float(buf, 0, vision_x);
	_mav_put_float(buf, 4, vision_y);
	_mav_put_float(buf, 8, vision_z);
	_mav_put_float(buf, 12, vision_distortion_x);
	_mav_put_float(buf, 16, vision_distortion_y);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_SENSOR_LEN);
#else
	mavlink_vision_sensor_t packet;
	packet.vision_x = vision_x;
	packet.vision_y = vision_y;
	packet.vision_z = vision_z;
	packet.vision_distortion_x = vision_distortion_x;
	packet.vision_distortion_y = vision_distortion_y;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_SENSOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_SENSOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VISION_SENSOR_LEN, MAVLINK_MSG_ID_VISION_SENSOR_CRC);
}

/**
 * @brief Encode a vision_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_sensor_t* vision_sensor)
{
	return mavlink_msg_vision_sensor_pack(system_id, component_id, msg, vision_sensor->vision_x, vision_sensor->vision_y, vision_sensor->vision_z, vision_sensor->vision_distortion_x, vision_sensor->vision_distortion_y);
}

/**
 * @brief Encode a vision_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_sensor_t* vision_sensor)
{
	return mavlink_msg_vision_sensor_pack_chan(system_id, component_id, chan, msg, vision_sensor->vision_x, vision_sensor->vision_y, vision_sensor->vision_z, vision_sensor->vision_distortion_x, vision_sensor->vision_distortion_y);
}

/**
 * @brief Send a vision_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param vision_x vision x velocity(m/s). 
 * @param vision_y vision y velocity(m/s). 
 * @param vision_z vision z velocity(m/s).
 * @param vision_distortion_x vision y velocity(m/s). 
 * @param vision_distortion_y vision z velocity(m/s).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_sensor_send(mavlink_channel_t chan, float vision_x, float vision_y, float vision_z, float vision_distortion_x, float vision_distortion_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_SENSOR_LEN];
	_mav_put_float(buf, 0, vision_x);
	_mav_put_float(buf, 4, vision_y);
	_mav_put_float(buf, 8, vision_z);
	_mav_put_float(buf, 12, vision_distortion_x);
	_mav_put_float(buf, 16, vision_distortion_y);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SENSOR, buf, MAVLINK_MSG_ID_VISION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VISION_SENSOR_LEN, MAVLINK_MSG_ID_VISION_SENSOR_CRC);
#else
	mavlink_vision_sensor_t packet;
	packet.vision_x = vision_x;
	packet.vision_y = vision_y;
	packet.vision_z = vision_z;
	packet.vision_distortion_x = vision_distortion_x;
	packet.vision_distortion_y = vision_distortion_y;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_VISION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VISION_SENSOR_LEN, MAVLINK_MSG_ID_VISION_SENSOR_CRC);
#endif
}

/**
 * @brief Send a vision_sensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_sensor_send_struct(mavlink_channel_t chan, const mavlink_vision_sensor_t* vision_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_sensor_send(chan, vision_sensor->vision_x, vision_sensor->vision_y, vision_sensor->vision_z, vision_sensor->vision_distortion_x, vision_sensor->vision_distortion_y);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SENSOR, (const char *)vision_sensor, MAVLINK_MSG_ID_VISION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VISION_SENSOR_LEN, MAVLINK_MSG_ID_VISION_SENSOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float vision_x, float vision_y, float vision_z, float vision_distortion_x, float vision_distortion_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, vision_x);
	_mav_put_float(buf, 4, vision_y);
	_mav_put_float(buf, 8, vision_z);
	_mav_put_float(buf, 12, vision_distortion_x);
	_mav_put_float(buf, 16, vision_distortion_y);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SENSOR, buf, MAVLINK_MSG_ID_VISION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VISION_SENSOR_LEN, MAVLINK_MSG_ID_VISION_SENSOR_CRC);
#else
	mavlink_vision_sensor_t *packet = (mavlink_vision_sensor_t *)msgbuf;
	packet->vision_x = vision_x;
	packet->vision_y = vision_y;
	packet->vision_z = vision_z;
	packet->vision_distortion_x = vision_distortion_x;
	packet->vision_distortion_y = vision_distortion_y;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SENSOR, (const char *)packet, MAVLINK_MSG_ID_VISION_SENSOR_MIN_LEN, MAVLINK_MSG_ID_VISION_SENSOR_LEN, MAVLINK_MSG_ID_VISION_SENSOR_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_SENSOR UNPACKING


/**
 * @brief Get field vision_x from vision_sensor message
 *
 * @return vision x velocity(m/s). 
 */
static inline float mavlink_msg_vision_sensor_get_vision_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vision_y from vision_sensor message
 *
 * @return vision y velocity(m/s). 
 */
static inline float mavlink_msg_vision_sensor_get_vision_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vision_z from vision_sensor message
 *
 * @return vision z velocity(m/s).
 */
static inline float mavlink_msg_vision_sensor_get_vision_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vision_distortion_x from vision_sensor message
 *
 * @return vision y velocity(m/s). 
 */
static inline float mavlink_msg_vision_sensor_get_vision_distortion_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vision_distortion_y from vision_sensor message
 *
 * @return vision z velocity(m/s).
 */
static inline float mavlink_msg_vision_sensor_get_vision_distortion_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a vision_sensor message into a struct
 *
 * @param msg The message to decode
 * @param vision_sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_sensor_decode(const mavlink_message_t* msg, mavlink_vision_sensor_t* vision_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	vision_sensor->vision_x = mavlink_msg_vision_sensor_get_vision_x(msg);
	vision_sensor->vision_y = mavlink_msg_vision_sensor_get_vision_y(msg);
	vision_sensor->vision_z = mavlink_msg_vision_sensor_get_vision_z(msg);
	vision_sensor->vision_distortion_x = mavlink_msg_vision_sensor_get_vision_distortion_x(msg);
	vision_sensor->vision_distortion_y = mavlink_msg_vision_sensor_get_vision_distortion_y(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_SENSOR_LEN? msg->len : MAVLINK_MSG_ID_VISION_SENSOR_LEN;
        memset(vision_sensor, 0, MAVLINK_MSG_ID_VISION_SENSOR_LEN);
	memcpy(vision_sensor, _MAV_PAYLOAD(msg), len);
#endif
}
