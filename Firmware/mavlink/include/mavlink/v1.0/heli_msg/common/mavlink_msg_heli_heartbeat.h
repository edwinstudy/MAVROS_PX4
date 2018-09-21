#ifndef _MAVLINK_MSG_HELI_HeartBeat_H_
#define _MAVLINK_MSG_HELI_HeartBeat_H_

// MESSAGE HELI_HeartBeat PACKING

#define MAVLINK_MSG_ID_HELI_HeartBeat 151

MAVPACKED(
typedef struct __mavlink_heli_heartbeat_t {
 uint8_t heartBeat; /*< 1 byte(unused)*/
}) mavlink_heli_heartbeat_t;

#define MAVLINK_MSG_ID_HELI_HeartBeat_LEN 1
#define MAVLINK_MSG_ID_HELI_HeartBeat_MIN_LEN 1
#define MAVLINK_MSG_ID_151_LEN 1
#define MAVLINK_MSG_ID_151_MIN_LEN 1

#define MAVLINK_MSG_ID_HELI_HeartBeat_CRC 235
#define MAVLINK_MSG_ID_151_CRC 235



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HELI_HeartBeat { \
	151, \
	"HELI_HeartBeat", \
	1, \
	{  { "heartBeat", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_heli_heartbeat_t, heartBeat) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HELI_HeartBeat { \
	"HELI_HeartBeat", \
	1, \
	{  { "heartBeat", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_heli_heartbeat_t, heartBeat) }, \
         } \
}
#endif

/**
 * @brief Pack a heli_heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param heartBeat 1 byte(unused)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heli_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t heartBeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HELI_HeartBeat_LEN];
	_mav_put_uint8_t(buf, 0, heartBeat);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HELI_HeartBeat_LEN);
#else
	mavlink_heli_heartbeat_t packet;
	packet.heartBeat = heartBeat;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HELI_HeartBeat_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HELI_HeartBeat;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HELI_HeartBeat_MIN_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_CRC);
}

/**
 * @brief Pack a heli_heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param heartBeat 1 byte(unused)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heli_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t heartBeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HELI_HeartBeat_LEN];
	_mav_put_uint8_t(buf, 0, heartBeat);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HELI_HeartBeat_LEN);
#else
	mavlink_heli_heartbeat_t packet;
	packet.heartBeat = heartBeat;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HELI_HeartBeat_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HELI_HeartBeat;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HELI_HeartBeat_MIN_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_CRC);
}

/**
 * @brief Encode a heli_heartbeat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heli_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heli_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heli_heartbeat_t* heli_heartbeat)
{
	return mavlink_msg_heli_heartbeat_pack(system_id, component_id, msg, heli_heartbeat->heartBeat);
}

/**
 * @brief Encode a heli_heartbeat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param heli_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heli_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_heli_heartbeat_t* heli_heartbeat)
{
	return mavlink_msg_heli_heartbeat_pack_chan(system_id, component_id, chan, msg, heli_heartbeat->heartBeat);
}

/**
 * @brief Send a heli_heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param heartBeat 1 byte(unused)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heli_heartbeat_send(mavlink_channel_t chan, uint8_t heartBeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HELI_HeartBeat_LEN];
	_mav_put_uint8_t(buf, 0, heartBeat);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HELI_HeartBeat, buf, MAVLINK_MSG_ID_HELI_HeartBeat_MIN_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_CRC);
#else
	mavlink_heli_heartbeat_t packet;
	packet.heartBeat = heartBeat;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HELI_HeartBeat, (const char *)&packet, MAVLINK_MSG_ID_HELI_HeartBeat_MIN_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_CRC);
#endif
}

/**
 * @brief Send a heli_heartbeat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_heli_heartbeat_send_struct(mavlink_channel_t chan, const mavlink_heli_heartbeat_t* heli_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_heli_heartbeat_send(chan, heli_heartbeat->heartBeat);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HELI_HeartBeat, (const char *)heli_heartbeat, MAVLINK_MSG_ID_HELI_HeartBeat_MIN_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_CRC);
#endif
}

#if MAVLINK_MSG_ID_HELI_HeartBeat_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_heli_heartbeat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t heartBeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, heartBeat);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HELI_HeartBeat, buf, MAVLINK_MSG_ID_HELI_HeartBeat_MIN_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_CRC);
#else
	mavlink_heli_heartbeat_t *packet = (mavlink_heli_heartbeat_t *)msgbuf;
	packet->heartBeat = heartBeat;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HELI_HeartBeat, (const char *)packet, MAVLINK_MSG_ID_HELI_HeartBeat_MIN_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_LEN, MAVLINK_MSG_ID_HELI_HeartBeat_CRC);
#endif
}
#endif

#endif

// MESSAGE HELI_HeartBeat UNPACKING


/**
 * @brief Get field heartBeat from heli_heartbeat message
 *
 * @return 1 byte(unused)
 */
static inline uint8_t mavlink_msg_heli_heartbeat_get_heartBeat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a heli_heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heli_heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_heli_heartbeat_decode(const mavlink_message_t* msg, mavlink_heli_heartbeat_t* heli_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	heli_heartbeat->heartBeat = mavlink_msg_heli_heartbeat_get_heartBeat(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HELI_HeartBeat_LEN? msg->len : MAVLINK_MSG_ID_HELI_HeartBeat_LEN;
        memset(heli_heartbeat, 0, MAVLINK_MSG_ID_HELI_HeartBeat_LEN);
	memcpy(heli_heartbeat, _MAV_PAYLOAD(msg), len);
#endif
}
#endif //_MAVLINK_MSG_HELI_HeartBeat_H_
