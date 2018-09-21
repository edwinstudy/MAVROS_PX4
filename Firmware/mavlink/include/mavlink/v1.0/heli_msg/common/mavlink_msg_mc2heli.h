#ifndef _MAVLINK_MSG_MC2HELI_H_
#define _MAVLINK_MSG_MC2HELI_H_
// MESSAGE MC2HELI PACKING

#define MAVLINK_MSG_ID_MC2HELI 153

MAVPACKED(
typedef struct __mavlink_mc2heli_t {
 uint8_t trackReady; /*< at desired position and velocity*/
 uint8_t catchSuccess; /*< the craw had catched the MC*/
}) mavlink_mc2heli_t;

#define MAVLINK_MSG_ID_MC2HELI_LEN 2
#define MAVLINK_MSG_ID_MC2HELI_MIN_LEN 2
#define MAVLINK_MSG_ID_153_LEN 2
#define MAVLINK_MSG_ID_153_MIN_LEN 2

#define MAVLINK_MSG_ID_MC2HELI_CRC 148
#define MAVLINK_MSG_ID_153_CRC 148



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MC2HELI { \
	153, \
	"MC2HELI", \
	2, \
	{  { "trackReady", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mc2heli_t, trackReady) }, \
         { "catchSuccess", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mc2heli_t, catchSuccess) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MC2HELI { \
	"MC2HELI", \
	2, \
	{  { "trackReady", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mc2heli_t, trackReady) }, \
         { "catchSuccess", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mc2heli_t, catchSuccess) }, \
         } \
}
#endif

/**
 * @brief Pack a mc2heli message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param trackReady at desired position and velocity
 * @param catchSuccess the craw had catched the MC
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mc2heli_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t trackReady, uint8_t catchSuccess)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MC2HELI_LEN];
	_mav_put_uint8_t(buf, 0, trackReady);
	_mav_put_uint8_t(buf, 1, catchSuccess);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MC2HELI_LEN);
#else
	mavlink_mc2heli_t packet;
	packet.trackReady = trackReady;
	packet.catchSuccess = catchSuccess;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MC2HELI_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MC2HELI;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MC2HELI_MIN_LEN, MAVLINK_MSG_ID_MC2HELI_LEN, MAVLINK_MSG_ID_MC2HELI_CRC);
}

/**
 * @brief Pack a mc2heli message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param trackReady at desired position and velocity
 * @param catchSuccess the craw had catched the MC
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mc2heli_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t trackReady,uint8_t catchSuccess)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MC2HELI_LEN];
	_mav_put_uint8_t(buf, 0, trackReady);
	_mav_put_uint8_t(buf, 1, catchSuccess);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MC2HELI_LEN);
#else
	mavlink_mc2heli_t packet;
	packet.trackReady = trackReady;
	packet.catchSuccess = catchSuccess;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MC2HELI_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MC2HELI;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MC2HELI_MIN_LEN, MAVLINK_MSG_ID_MC2HELI_LEN, MAVLINK_MSG_ID_MC2HELI_CRC);
}

/**
 * @brief Encode a mc2heli struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mc2heli C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mc2heli_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mc2heli_t* mc2heli)
{
	return mavlink_msg_mc2heli_pack(system_id, component_id, msg, mc2heli->trackReady, mc2heli->catchSuccess);
}

/**
 * @brief Encode a mc2heli struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mc2heli C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mc2heli_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mc2heli_t* mc2heli)
{
	return mavlink_msg_mc2heli_pack_chan(system_id, component_id, chan, msg, mc2heli->trackReady, mc2heli->catchSuccess);
}

/**
 * @brief Send a mc2heli message
 * @param chan MAVLink channel to send the message
 *
 * @param trackReady at desired position and velocity
 * @param catchSuccess the craw had catched the MC
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mc2heli_send(mavlink_channel_t chan, uint8_t trackReady, uint8_t catchSuccess)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MC2HELI_LEN];
	_mav_put_uint8_t(buf, 0, trackReady);
	_mav_put_uint8_t(buf, 1, catchSuccess);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MC2HELI, buf, MAVLINK_MSG_ID_MC2HELI_MIN_LEN, MAVLINK_MSG_ID_MC2HELI_LEN, MAVLINK_MSG_ID_MC2HELI_CRC);
#else
	mavlink_mc2heli_t packet;
	packet.trackReady = trackReady;
	packet.catchSuccess = catchSuccess;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MC2HELI, (const char *)&packet, MAVLINK_MSG_ID_MC2HELI_MIN_LEN, MAVLINK_MSG_ID_MC2HELI_LEN, MAVLINK_MSG_ID_MC2HELI_CRC);
#endif
}

/**
 * @brief Send a mc2heli message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mc2heli_send_struct(mavlink_channel_t chan, const mavlink_mc2heli_t* mc2heli)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mc2heli_send(chan, mc2heli->trackReady, mc2heli->catchSuccess);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MC2HELI, (const char *)mc2heli, MAVLINK_MSG_ID_MC2HELI_MIN_LEN, MAVLINK_MSG_ID_MC2HELI_LEN, MAVLINK_MSG_ID_MC2HELI_CRC);
#endif
}

#if MAVLINK_MSG_ID_MC2HELI_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mc2heli_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t trackReady, uint8_t catchSuccess)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, trackReady);
	_mav_put_uint8_t(buf, 1, catchSuccess);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MC2HELI, buf, MAVLINK_MSG_ID_MC2HELI_MIN_LEN, MAVLINK_MSG_ID_MC2HELI_LEN, MAVLINK_MSG_ID_MC2HELI_CRC);
#else
	mavlink_mc2heli_t *packet = (mavlink_mc2heli_t *)msgbuf;
	packet->trackReady = trackReady;
	packet->catchSuccess = catchSuccess;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MC2HELI, (const char *)packet, MAVLINK_MSG_ID_MC2HELI_MIN_LEN, MAVLINK_MSG_ID_MC2HELI_LEN, MAVLINK_MSG_ID_MC2HELI_CRC);
#endif
}
#endif

#endif

// MESSAGE MC2HELI UNPACKING


/**
 * @brief Get field trackReady from mc2heli message
 *
 * @return at desired position and velocity
 */
static inline uint8_t mavlink_msg_mc2heli_get_trackReady(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field catchSuccess from mc2heli message
 *
 * @return the craw had catched the MC
 */
static inline uint8_t mavlink_msg_mc2heli_get_catchSuccess(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a mc2heli message into a struct
 *
 * @param msg The message to decode
 * @param mc2heli C-struct to decode the message contents into
 */
static inline void mavlink_msg_mc2heli_decode(const mavlink_message_t* msg, mavlink_mc2heli_t* mc2heli)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	mc2heli->trackReady = mavlink_msg_mc2heli_get_trackReady(msg);
	mc2heli->catchSuccess = mavlink_msg_mc2heli_get_catchSuccess(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MC2HELI_LEN? msg->len : MAVLINK_MSG_ID_MC2HELI_LEN;
        memset(mc2heli, 0, MAVLINK_MSG_ID_MC2HELI_LEN);
	memcpy(mc2heli, _MAV_PAYLOAD(msg), len);
#endif
}
#endif //_MAVLINK_MSG_MC2HELI_H_
