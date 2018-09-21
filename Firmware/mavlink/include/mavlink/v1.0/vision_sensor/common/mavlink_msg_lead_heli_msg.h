// MESSAGE LEAD_HELI_MSG PACKING

#define MAVLINK_MSG_ID_LEAD_HELI_MSG 155

MAVPACKED(
typedef struct __mavlink_lead_heli_msg_t {
 int32_t lat; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t lon; /*< Longitude (WGS84), in degrees * 1E7*/
 int32_t alt; /*< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.*/
 float yaw; /*< yaw angle in rad(0-2*pi). */
 float velN; /*< GPS north velocity(m/s). */
 float velE; /*< GPS east velocity(m/s). */
 float velD; /*< GPS downward velocity(m/s).*/
 float x; /*< target position in vision*/
 float y; /*< target position in vision*/
 float z; /*< target position in vision*/
 float velX; /*< target velocity in vision*/
 float velY; /*< target velocity in vision*/
 float velZ; /*< target velocity in vision*/
 uint8_t trackEnalbled; /*< used to judge helicoptor GPS state*/
 uint8_t catched; /*< true: craw is closed, false: craw is opened*/
}) mavlink_lead_heli_msg_t;

#define MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN 54
#define MAVLINK_MSG_ID_LEAD_HELI_MSG_MIN_LEN 54
#define MAVLINK_MSG_ID_155_LEN 54
#define MAVLINK_MSG_ID_155_MIN_LEN 54

#define MAVLINK_MSG_ID_LEAD_HELI_MSG_CRC 248
#define MAVLINK_MSG_ID_155_CRC 248



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LEAD_HELI_MSG { \
	155, \
	"LEAD_HELI_MSG", \
	15, \
	{  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_lead_heli_msg_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_lead_heli_msg_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_lead_heli_msg_t, alt) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_lead_heli_msg_t, yaw) }, \
         { "velN", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_lead_heli_msg_t, velN) }, \
         { "velE", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_lead_heli_msg_t, velE) }, \
         { "velD", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_lead_heli_msg_t, velD) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_lead_heli_msg_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_lead_heli_msg_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_lead_heli_msg_t, z) }, \
         { "velX", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_lead_heli_msg_t, velX) }, \
         { "velY", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_lead_heli_msg_t, velY) }, \
         { "velZ", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_lead_heli_msg_t, velZ) }, \
         { "trackEnalbled", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_lead_heli_msg_t, trackEnalbled) }, \
         { "catched", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_lead_heli_msg_t, catched) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LEAD_HELI_MSG { \
	"LEAD_HELI_MSG", \
	15, \
	{  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_lead_heli_msg_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_lead_heli_msg_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_lead_heli_msg_t, alt) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_lead_heli_msg_t, yaw) }, \
         { "velN", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_lead_heli_msg_t, velN) }, \
         { "velE", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_lead_heli_msg_t, velE) }, \
         { "velD", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_lead_heli_msg_t, velD) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_lead_heli_msg_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_lead_heli_msg_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_lead_heli_msg_t, z) }, \
         { "velX", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_lead_heli_msg_t, velX) }, \
         { "velY", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_lead_heli_msg_t, velY) }, \
         { "velZ", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_lead_heli_msg_t, velZ) }, \
         { "trackEnalbled", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_lead_heli_msg_t, trackEnalbled) }, \
         { "catched", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_lead_heli_msg_t, catched) }, \
         } \
}
#endif

/**
 * @brief Pack a lead_heli_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
 * @param yaw yaw angle in rad(0-2*pi). 
 * @param velN GPS north velocity(m/s). 
 * @param velE GPS east velocity(m/s). 
 * @param velD GPS downward velocity(m/s).
 * @param x target position in vision
 * @param y target position in vision
 * @param z target position in vision
 * @param velX target velocity in vision
 * @param velY target velocity in vision
 * @param velZ target velocity in vision
 * @param trackEnalbled used to judge helicoptor GPS state
 * @param catched true: craw is closed, false: craw is opened
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lead_heli_msg_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t lat, int32_t lon, int32_t alt, float yaw, float velN, float velE, float velD, float x, float y, float z, float velX, float velY, float velZ, uint8_t trackEnalbled, uint8_t catched)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, alt);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, velN);
	_mav_put_float(buf, 20, velE);
	_mav_put_float(buf, 24, velD);
	_mav_put_float(buf, 28, x);
	_mav_put_float(buf, 32, y);
	_mav_put_float(buf, 36, z);
	_mav_put_float(buf, 40, velX);
	_mav_put_float(buf, 44, velY);
	_mav_put_float(buf, 48, velZ);
	_mav_put_uint8_t(buf, 52, trackEnalbled);
	_mav_put_uint8_t(buf, 53, catched);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN);
#else
	mavlink_lead_heli_msg_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.yaw = yaw;
	packet.velN = velN;
	packet.velE = velE;
	packet.velD = velD;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.velX = velX;
	packet.velY = velY;
	packet.velZ = velZ;
	packet.trackEnalbled = trackEnalbled;
	packet.catched = catched;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LEAD_HELI_MSG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LEAD_HELI_MSG_MIN_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_CRC);
}

/**
 * @brief Pack a lead_heli_msg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
 * @param yaw yaw angle in rad(0-2*pi). 
 * @param velN GPS north velocity(m/s). 
 * @param velE GPS east velocity(m/s). 
 * @param velD GPS downward velocity(m/s).
 * @param x target position in vision
 * @param y target position in vision
 * @param z target position in vision
 * @param velX target velocity in vision
 * @param velY target velocity in vision
 * @param velZ target velocity in vision
 * @param trackEnalbled used to judge helicoptor GPS state
 * @param catched true: craw is closed, false: craw is opened
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lead_heli_msg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t lat,int32_t lon,int32_t alt,float yaw,float velN,float velE,float velD,float x,float y,float z,float velX,float velY,float velZ,uint8_t trackEnalbled,uint8_t catched)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, alt);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, velN);
	_mav_put_float(buf, 20, velE);
	_mav_put_float(buf, 24, velD);
	_mav_put_float(buf, 28, x);
	_mav_put_float(buf, 32, y);
	_mav_put_float(buf, 36, z);
	_mav_put_float(buf, 40, velX);
	_mav_put_float(buf, 44, velY);
	_mav_put_float(buf, 48, velZ);
	_mav_put_uint8_t(buf, 52, trackEnalbled);
	_mav_put_uint8_t(buf, 53, catched);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN);
#else
	mavlink_lead_heli_msg_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.yaw = yaw;
	packet.velN = velN;
	packet.velE = velE;
	packet.velD = velD;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.velX = velX;
	packet.velY = velY;
	packet.velZ = velZ;
	packet.trackEnalbled = trackEnalbled;
	packet.catched = catched;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LEAD_HELI_MSG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LEAD_HELI_MSG_MIN_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_CRC);
}

/**
 * @brief Encode a lead_heli_msg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lead_heli_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lead_heli_msg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lead_heli_msg_t* lead_heli_msg)
{
	return mavlink_msg_lead_heli_msg_pack(system_id, component_id, msg, lead_heli_msg->lat, lead_heli_msg->lon, lead_heli_msg->alt, lead_heli_msg->yaw, lead_heli_msg->velN, lead_heli_msg->velE, lead_heli_msg->velD, lead_heli_msg->x, lead_heli_msg->y, lead_heli_msg->z, lead_heli_msg->velX, lead_heli_msg->velY, lead_heli_msg->velZ, lead_heli_msg->trackEnalbled, lead_heli_msg->catched);
}

/**
 * @brief Encode a lead_heli_msg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lead_heli_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lead_heli_msg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lead_heli_msg_t* lead_heli_msg)
{
	return mavlink_msg_lead_heli_msg_pack_chan(system_id, component_id, chan, msg, lead_heli_msg->lat, lead_heli_msg->lon, lead_heli_msg->alt, lead_heli_msg->yaw, lead_heli_msg->velN, lead_heli_msg->velE, lead_heli_msg->velD, lead_heli_msg->x, lead_heli_msg->y, lead_heli_msg->z, lead_heli_msg->velX, lead_heli_msg->velY, lead_heli_msg->velZ, lead_heli_msg->trackEnalbled, lead_heli_msg->catched);
}

/**
 * @brief Send a lead_heli_msg message
 * @param chan MAVLink channel to send the message
 *
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
 * @param yaw yaw angle in rad(0-2*pi). 
 * @param velN GPS north velocity(m/s). 
 * @param velE GPS east velocity(m/s). 
 * @param velD GPS downward velocity(m/s).
 * @param x target position in vision
 * @param y target position in vision
 * @param z target position in vision
 * @param velX target velocity in vision
 * @param velY target velocity in vision
 * @param velZ target velocity in vision
 * @param trackEnalbled used to judge helicoptor GPS state
 * @param catched true: craw is closed, false: craw is opened
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lead_heli_msg_send(mavlink_channel_t chan, int32_t lat, int32_t lon, int32_t alt, float yaw, float velN, float velE, float velD, float x, float y, float z, float velX, float velY, float velZ, uint8_t trackEnalbled, uint8_t catched)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, alt);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, velN);
	_mav_put_float(buf, 20, velE);
	_mav_put_float(buf, 24, velD);
	_mav_put_float(buf, 28, x);
	_mav_put_float(buf, 32, y);
	_mav_put_float(buf, 36, z);
	_mav_put_float(buf, 40, velX);
	_mav_put_float(buf, 44, velY);
	_mav_put_float(buf, 48, velZ);
	_mav_put_uint8_t(buf, 52, trackEnalbled);
	_mav_put_uint8_t(buf, 53, catched);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LEAD_HELI_MSG, buf, MAVLINK_MSG_ID_LEAD_HELI_MSG_MIN_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_CRC);
#else
	mavlink_lead_heli_msg_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.yaw = yaw;
	packet.velN = velN;
	packet.velE = velE;
	packet.velD = velD;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.velX = velX;
	packet.velY = velY;
	packet.velZ = velZ;
	packet.trackEnalbled = trackEnalbled;
	packet.catched = catched;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LEAD_HELI_MSG, (const char *)&packet, MAVLINK_MSG_ID_LEAD_HELI_MSG_MIN_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_CRC);
#endif
}

/**
 * @brief Send a lead_heli_msg message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lead_heli_msg_send_struct(mavlink_channel_t chan, const mavlink_lead_heli_msg_t* lead_heli_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lead_heli_msg_send(chan, lead_heli_msg->lat, lead_heli_msg->lon, lead_heli_msg->alt, lead_heli_msg->yaw, lead_heli_msg->velN, lead_heli_msg->velE, lead_heli_msg->velD, lead_heli_msg->x, lead_heli_msg->y, lead_heli_msg->z, lead_heli_msg->velX, lead_heli_msg->velY, lead_heli_msg->velZ, lead_heli_msg->trackEnalbled, lead_heli_msg->catched);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LEAD_HELI_MSG, (const char *)lead_heli_msg, MAVLINK_MSG_ID_LEAD_HELI_MSG_MIN_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_CRC);
#endif
}

#if MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lead_heli_msg_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat, int32_t lon, int32_t alt, float yaw, float velN, float velE, float velD, float x, float y, float z, float velX, float velY, float velZ, uint8_t trackEnalbled, uint8_t catched)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, alt);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, velN);
	_mav_put_float(buf, 20, velE);
	_mav_put_float(buf, 24, velD);
	_mav_put_float(buf, 28, x);
	_mav_put_float(buf, 32, y);
	_mav_put_float(buf, 36, z);
	_mav_put_float(buf, 40, velX);
	_mav_put_float(buf, 44, velY);
	_mav_put_float(buf, 48, velZ);
	_mav_put_uint8_t(buf, 52, trackEnalbled);
	_mav_put_uint8_t(buf, 53, catched);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LEAD_HELI_MSG, buf, MAVLINK_MSG_ID_LEAD_HELI_MSG_MIN_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_CRC);
#else
	mavlink_lead_heli_msg_t *packet = (mavlink_lead_heli_msg_t *)msgbuf;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;
	packet->yaw = yaw;
	packet->velN = velN;
	packet->velE = velE;
	packet->velD = velD;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->velX = velX;
	packet->velY = velY;
	packet->velZ = velZ;
	packet->trackEnalbled = trackEnalbled;
	packet->catched = catched;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LEAD_HELI_MSG, (const char *)packet, MAVLINK_MSG_ID_LEAD_HELI_MSG_MIN_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN, MAVLINK_MSG_ID_LEAD_HELI_MSG_CRC);
#endif
}
#endif

#endif

// MESSAGE LEAD_HELI_MSG UNPACKING


/**
 * @brief Get field lat from lead_heli_msg message
 *
 * @return Latitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_lead_heli_msg_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from lead_heli_msg message
 *
 * @return Longitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_lead_heli_msg_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from lead_heli_msg message
 *
 * @return Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
 */
static inline int32_t mavlink_msg_lead_heli_msg_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field yaw from lead_heli_msg message
 *
 * @return yaw angle in rad(0-2*pi). 
 */
static inline float mavlink_msg_lead_heli_msg_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field velN from lead_heli_msg message
 *
 * @return GPS north velocity(m/s). 
 */
static inline float mavlink_msg_lead_heli_msg_get_velN(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field velE from lead_heli_msg message
 *
 * @return GPS east velocity(m/s). 
 */
static inline float mavlink_msg_lead_heli_msg_get_velE(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field velD from lead_heli_msg message
 *
 * @return GPS downward velocity(m/s).
 */
static inline float mavlink_msg_lead_heli_msg_get_velD(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field x from lead_heli_msg message
 *
 * @return target position in vision
 */
static inline float mavlink_msg_lead_heli_msg_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field y from lead_heli_msg message
 *
 * @return target position in vision
 */
static inline float mavlink_msg_lead_heli_msg_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field z from lead_heli_msg message
 *
 * @return target position in vision
 */
static inline float mavlink_msg_lead_heli_msg_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field velX from lead_heli_msg message
 *
 * @return target velocity in vision
 */
static inline float mavlink_msg_lead_heli_msg_get_velX(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field velY from lead_heli_msg message
 *
 * @return target velocity in vision
 */
static inline float mavlink_msg_lead_heli_msg_get_velY(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field velZ from lead_heli_msg message
 *
 * @return target velocity in vision
 */
static inline float mavlink_msg_lead_heli_msg_get_velZ(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field trackEnalbled from lead_heli_msg message
 *
 * @return used to judge helicoptor GPS state
 */
static inline uint8_t mavlink_msg_lead_heli_msg_get_trackEnalbled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field catched from lead_heli_msg message
 *
 * @return true: craw is closed, false: craw is opened
 */
static inline uint8_t mavlink_msg_lead_heli_msg_get_catched(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Decode a lead_heli_msg message into a struct
 *
 * @param msg The message to decode
 * @param lead_heli_msg C-struct to decode the message contents into
 */
static inline void mavlink_msg_lead_heli_msg_decode(const mavlink_message_t* msg, mavlink_lead_heli_msg_t* lead_heli_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	lead_heli_msg->lat = mavlink_msg_lead_heli_msg_get_lat(msg);
	lead_heli_msg->lon = mavlink_msg_lead_heli_msg_get_lon(msg);
	lead_heli_msg->alt = mavlink_msg_lead_heli_msg_get_alt(msg);
	lead_heli_msg->yaw = mavlink_msg_lead_heli_msg_get_yaw(msg);
	lead_heli_msg->velN = mavlink_msg_lead_heli_msg_get_velN(msg);
	lead_heli_msg->velE = mavlink_msg_lead_heli_msg_get_velE(msg);
	lead_heli_msg->velD = mavlink_msg_lead_heli_msg_get_velD(msg);
	lead_heli_msg->x = mavlink_msg_lead_heli_msg_get_x(msg);
	lead_heli_msg->y = mavlink_msg_lead_heli_msg_get_y(msg);
	lead_heli_msg->z = mavlink_msg_lead_heli_msg_get_z(msg);
	lead_heli_msg->velX = mavlink_msg_lead_heli_msg_get_velX(msg);
	lead_heli_msg->velY = mavlink_msg_lead_heli_msg_get_velY(msg);
	lead_heli_msg->velZ = mavlink_msg_lead_heli_msg_get_velZ(msg);
	lead_heli_msg->trackEnalbled = mavlink_msg_lead_heli_msg_get_trackEnalbled(msg);
	lead_heli_msg->catched = mavlink_msg_lead_heli_msg_get_catched(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN? msg->len : MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN;
        memset(lead_heli_msg, 0, MAVLINK_MSG_ID_LEAD_HELI_MSG_LEN);
	memcpy(lead_heli_msg, _MAV_PAYLOAD(msg), len);
#endif
}
