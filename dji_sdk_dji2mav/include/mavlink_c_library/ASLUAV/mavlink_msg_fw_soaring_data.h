// MESSAGE FW_SOARING_DATA PACKING

#define MAVLINK_MSG_ID_FW_SOARING_DATA 210

typedef struct __mavlink_fw_soaring_data_t
{
 uint64_t timestamp; /*< Timestamp [ms]*/
 uint64_t timestampModeChanged; /*< Timestamp since last mode change[ms]*/
 float CurrentUpdraftSpeed; /*< Updraft speed at current/local airplane position [m/s]*/
 float xW; /*< Thermal core updraft strength [m/s]*/
 float xR; /*< Thermal radius [m]*/
 float xLat; /*< Thermal center latitude [deg]*/
 float xLon; /*< Thermal center longitude [deg]*/
 float VarW; /*< Variance W*/
 float VarR; /*< Variance R*/
 float VarLat; /*< Variance Lat*/
 float VarLon; /*< Variance Lon */
 float LoiterRadius; /*< Suggested loiter radius [m]*/
 uint8_t ControlMode; /*< Control Mode [-]*/
 uint8_t valid; /*< Data valid [-]*/
} mavlink_fw_soaring_data_t;

#define MAVLINK_MSG_ID_FW_SOARING_DATA_LEN 58
#define MAVLINK_MSG_ID_210_LEN 58

#define MAVLINK_MSG_ID_FW_SOARING_DATA_CRC 129
#define MAVLINK_MSG_ID_210_CRC 129



#define MAVLINK_MESSAGE_INFO_FW_SOARING_DATA { \
	"FW_SOARING_DATA", \
	14, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_fw_soaring_data_t, timestamp) }, \
         { "timestampModeChanged", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_fw_soaring_data_t, timestampModeChanged) }, \
         { "CurrentUpdraftSpeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_fw_soaring_data_t, CurrentUpdraftSpeed) }, \
         { "xW", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_fw_soaring_data_t, xW) }, \
         { "xR", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_fw_soaring_data_t, xR) }, \
         { "xLat", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_fw_soaring_data_t, xLat) }, \
         { "xLon", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_fw_soaring_data_t, xLon) }, \
         { "VarW", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_fw_soaring_data_t, VarW) }, \
         { "VarR", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_fw_soaring_data_t, VarR) }, \
         { "VarLat", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_fw_soaring_data_t, VarLat) }, \
         { "VarLon", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_fw_soaring_data_t, VarLon) }, \
         { "LoiterRadius", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_fw_soaring_data_t, LoiterRadius) }, \
         { "ControlMode", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_fw_soaring_data_t, ControlMode) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 57, offsetof(mavlink_fw_soaring_data_t, valid) }, \
         } \
}


/**
 * @brief Pack a fw_soaring_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp [ms]
 * @param timestampModeChanged Timestamp since last mode change[ms]
 * @param CurrentUpdraftSpeed Updraft speed at current/local airplane position [m/s]
 * @param xW Thermal core updraft strength [m/s]
 * @param xR Thermal radius [m]
 * @param xLat Thermal center latitude [deg]
 * @param xLon Thermal center longitude [deg]
 * @param VarW Variance W
 * @param VarR Variance R
 * @param VarLat Variance Lat
 * @param VarLon Variance Lon 
 * @param LoiterRadius Suggested loiter radius [m]
 * @param ControlMode Control Mode [-]
 * @param valid Data valid [-]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fw_soaring_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, uint64_t timestampModeChanged, float CurrentUpdraftSpeed, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, uint8_t ControlMode, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FW_SOARING_DATA_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint64_t(buf, 8, timestampModeChanged);
	_mav_put_float(buf, 16, CurrentUpdraftSpeed);
	_mav_put_float(buf, 20, xW);
	_mav_put_float(buf, 24, xR);
	_mav_put_float(buf, 28, xLat);
	_mav_put_float(buf, 32, xLon);
	_mav_put_float(buf, 36, VarW);
	_mav_put_float(buf, 40, VarR);
	_mav_put_float(buf, 44, VarLat);
	_mav_put_float(buf, 48, VarLon);
	_mav_put_float(buf, 52, LoiterRadius);
	_mav_put_uint8_t(buf, 56, ControlMode);
	_mav_put_uint8_t(buf, 57, valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#else
	mavlink_fw_soaring_data_t packet;
	packet.timestamp = timestamp;
	packet.timestampModeChanged = timestampModeChanged;
	packet.CurrentUpdraftSpeed = CurrentUpdraftSpeed;
	packet.xW = xW;
	packet.xR = xR;
	packet.xLat = xLat;
	packet.xLon = xLon;
	packet.VarW = VarW;
	packet.VarR = VarR;
	packet.VarLat = VarLat;
	packet.VarLon = VarLon;
	packet.LoiterRadius = LoiterRadius;
	packet.ControlMode = ControlMode;
	packet.valid = valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FW_SOARING_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif
}

/**
 * @brief Pack a fw_soaring_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp [ms]
 * @param timestampModeChanged Timestamp since last mode change[ms]
 * @param CurrentUpdraftSpeed Updraft speed at current/local airplane position [m/s]
 * @param xW Thermal core updraft strength [m/s]
 * @param xR Thermal radius [m]
 * @param xLat Thermal center latitude [deg]
 * @param xLon Thermal center longitude [deg]
 * @param VarW Variance W
 * @param VarR Variance R
 * @param VarLat Variance Lat
 * @param VarLon Variance Lon 
 * @param LoiterRadius Suggested loiter radius [m]
 * @param ControlMode Control Mode [-]
 * @param valid Data valid [-]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fw_soaring_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,uint64_t timestampModeChanged,float CurrentUpdraftSpeed,float xW,float xR,float xLat,float xLon,float VarW,float VarR,float VarLat,float VarLon,float LoiterRadius,uint8_t ControlMode,uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FW_SOARING_DATA_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint64_t(buf, 8, timestampModeChanged);
	_mav_put_float(buf, 16, CurrentUpdraftSpeed);
	_mav_put_float(buf, 20, xW);
	_mav_put_float(buf, 24, xR);
	_mav_put_float(buf, 28, xLat);
	_mav_put_float(buf, 32, xLon);
	_mav_put_float(buf, 36, VarW);
	_mav_put_float(buf, 40, VarR);
	_mav_put_float(buf, 44, VarLat);
	_mav_put_float(buf, 48, VarLon);
	_mav_put_float(buf, 52, LoiterRadius);
	_mav_put_uint8_t(buf, 56, ControlMode);
	_mav_put_uint8_t(buf, 57, valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#else
	mavlink_fw_soaring_data_t packet;
	packet.timestamp = timestamp;
	packet.timestampModeChanged = timestampModeChanged;
	packet.CurrentUpdraftSpeed = CurrentUpdraftSpeed;
	packet.xW = xW;
	packet.xR = xR;
	packet.xLat = xLat;
	packet.xLon = xLon;
	packet.VarW = VarW;
	packet.VarR = VarR;
	packet.VarLat = VarLat;
	packet.VarLon = VarLon;
	packet.LoiterRadius = LoiterRadius;
	packet.ControlMode = ControlMode;
	packet.valid = valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FW_SOARING_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif
}

/**
 * @brief Encode a fw_soaring_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fw_soaring_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fw_soaring_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fw_soaring_data_t* fw_soaring_data)
{
	return mavlink_msg_fw_soaring_data_pack(system_id, component_id, msg, fw_soaring_data->timestamp, fw_soaring_data->timestampModeChanged, fw_soaring_data->CurrentUpdraftSpeed, fw_soaring_data->xW, fw_soaring_data->xR, fw_soaring_data->xLat, fw_soaring_data->xLon, fw_soaring_data->VarW, fw_soaring_data->VarR, fw_soaring_data->VarLat, fw_soaring_data->VarLon, fw_soaring_data->LoiterRadius, fw_soaring_data->ControlMode, fw_soaring_data->valid);
}

/**
 * @brief Encode a fw_soaring_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fw_soaring_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fw_soaring_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fw_soaring_data_t* fw_soaring_data)
{
	return mavlink_msg_fw_soaring_data_pack_chan(system_id, component_id, chan, msg, fw_soaring_data->timestamp, fw_soaring_data->timestampModeChanged, fw_soaring_data->CurrentUpdraftSpeed, fw_soaring_data->xW, fw_soaring_data->xR, fw_soaring_data->xLat, fw_soaring_data->xLon, fw_soaring_data->VarW, fw_soaring_data->VarR, fw_soaring_data->VarLat, fw_soaring_data->VarLon, fw_soaring_data->LoiterRadius, fw_soaring_data->ControlMode, fw_soaring_data->valid);
}

/**
 * @brief Send a fw_soaring_data message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp [ms]
 * @param timestampModeChanged Timestamp since last mode change[ms]
 * @param CurrentUpdraftSpeed Updraft speed at current/local airplane position [m/s]
 * @param xW Thermal core updraft strength [m/s]
 * @param xR Thermal radius [m]
 * @param xLat Thermal center latitude [deg]
 * @param xLon Thermal center longitude [deg]
 * @param VarW Variance W
 * @param VarR Variance R
 * @param VarLat Variance Lat
 * @param VarLon Variance Lon 
 * @param LoiterRadius Suggested loiter radius [m]
 * @param ControlMode Control Mode [-]
 * @param valid Data valid [-]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fw_soaring_data_send(mavlink_channel_t chan, uint64_t timestamp, uint64_t timestampModeChanged, float CurrentUpdraftSpeed, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, uint8_t ControlMode, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FW_SOARING_DATA_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint64_t(buf, 8, timestampModeChanged);
	_mav_put_float(buf, 16, CurrentUpdraftSpeed);
	_mav_put_float(buf, 20, xW);
	_mav_put_float(buf, 24, xR);
	_mav_put_float(buf, 28, xLat);
	_mav_put_float(buf, 32, xLon);
	_mav_put_float(buf, 36, VarW);
	_mav_put_float(buf, 40, VarR);
	_mav_put_float(buf, 44, VarLat);
	_mav_put_float(buf, 48, VarLon);
	_mav_put_float(buf, 52, LoiterRadius);
	_mav_put_uint8_t(buf, 56, ControlMode);
	_mav_put_uint8_t(buf, 57, valid);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, buf, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, buf, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif
#else
	mavlink_fw_soaring_data_t packet;
	packet.timestamp = timestamp;
	packet.timestampModeChanged = timestampModeChanged;
	packet.CurrentUpdraftSpeed = CurrentUpdraftSpeed;
	packet.xW = xW;
	packet.xR = xR;
	packet.xLat = xLat;
	packet.xLon = xLon;
	packet.VarW = VarW;
	packet.VarR = VarR;
	packet.VarLat = VarLat;
	packet.VarLon = VarLon;
	packet.LoiterRadius = LoiterRadius;
	packet.ControlMode = ControlMode;
	packet.valid = valid;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, (const char *)&packet, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, (const char *)&packet, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_FW_SOARING_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fw_soaring_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint64_t timestampModeChanged, float CurrentUpdraftSpeed, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, uint8_t ControlMode, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint64_t(buf, 8, timestampModeChanged);
	_mav_put_float(buf, 16, CurrentUpdraftSpeed);
	_mav_put_float(buf, 20, xW);
	_mav_put_float(buf, 24, xR);
	_mav_put_float(buf, 28, xLat);
	_mav_put_float(buf, 32, xLon);
	_mav_put_float(buf, 36, VarW);
	_mav_put_float(buf, 40, VarR);
	_mav_put_float(buf, 44, VarLat);
	_mav_put_float(buf, 48, VarLon);
	_mav_put_float(buf, 52, LoiterRadius);
	_mav_put_uint8_t(buf, 56, ControlMode);
	_mav_put_uint8_t(buf, 57, valid);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, buf, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, buf, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif
#else
	mavlink_fw_soaring_data_t *packet = (mavlink_fw_soaring_data_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->timestampModeChanged = timestampModeChanged;
	packet->CurrentUpdraftSpeed = CurrentUpdraftSpeed;
	packet->xW = xW;
	packet->xR = xR;
	packet->xLat = xLat;
	packet->xLon = xLon;
	packet->VarW = VarW;
	packet->VarR = VarR;
	packet->VarLat = VarLat;
	packet->VarLon = VarLon;
	packet->LoiterRadius = LoiterRadius;
	packet->ControlMode = ControlMode;
	packet->valid = valid;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, (const char *)packet, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, (const char *)packet, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE FW_SOARING_DATA UNPACKING


/**
 * @brief Get field timestamp from fw_soaring_data message
 *
 * @return Timestamp [ms]
 */
static inline uint64_t mavlink_msg_fw_soaring_data_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field timestampModeChanged from fw_soaring_data message
 *
 * @return Timestamp since last mode change[ms]
 */
static inline uint64_t mavlink_msg_fw_soaring_data_get_timestampModeChanged(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field CurrentUpdraftSpeed from fw_soaring_data message
 *
 * @return Updraft speed at current/local airplane position [m/s]
 */
static inline float mavlink_msg_fw_soaring_data_get_CurrentUpdraftSpeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field xW from fw_soaring_data message
 *
 * @return Thermal core updraft strength [m/s]
 */
static inline float mavlink_msg_fw_soaring_data_get_xW(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field xR from fw_soaring_data message
 *
 * @return Thermal radius [m]
 */
static inline float mavlink_msg_fw_soaring_data_get_xR(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field xLat from fw_soaring_data message
 *
 * @return Thermal center latitude [deg]
 */
static inline float mavlink_msg_fw_soaring_data_get_xLat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field xLon from fw_soaring_data message
 *
 * @return Thermal center longitude [deg]
 */
static inline float mavlink_msg_fw_soaring_data_get_xLon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field VarW from fw_soaring_data message
 *
 * @return Variance W
 */
static inline float mavlink_msg_fw_soaring_data_get_VarW(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field VarR from fw_soaring_data message
 *
 * @return Variance R
 */
static inline float mavlink_msg_fw_soaring_data_get_VarR(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field VarLat from fw_soaring_data message
 *
 * @return Variance Lat
 */
static inline float mavlink_msg_fw_soaring_data_get_VarLat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field VarLon from fw_soaring_data message
 *
 * @return Variance Lon 
 */
static inline float mavlink_msg_fw_soaring_data_get_VarLon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field LoiterRadius from fw_soaring_data message
 *
 * @return Suggested loiter radius [m]
 */
static inline float mavlink_msg_fw_soaring_data_get_LoiterRadius(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field ControlMode from fw_soaring_data message
 *
 * @return Control Mode [-]
 */
static inline uint8_t mavlink_msg_fw_soaring_data_get_ControlMode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  56);
}

/**
 * @brief Get field valid from fw_soaring_data message
 *
 * @return Data valid [-]
 */
static inline uint8_t mavlink_msg_fw_soaring_data_get_valid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  57);
}

/**
 * @brief Decode a fw_soaring_data message into a struct
 *
 * @param msg The message to decode
 * @param fw_soaring_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_fw_soaring_data_decode(const mavlink_message_t* msg, mavlink_fw_soaring_data_t* fw_soaring_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	fw_soaring_data->timestamp = mavlink_msg_fw_soaring_data_get_timestamp(msg);
	fw_soaring_data->timestampModeChanged = mavlink_msg_fw_soaring_data_get_timestampModeChanged(msg);
	fw_soaring_data->CurrentUpdraftSpeed = mavlink_msg_fw_soaring_data_get_CurrentUpdraftSpeed(msg);
	fw_soaring_data->xW = mavlink_msg_fw_soaring_data_get_xW(msg);
	fw_soaring_data->xR = mavlink_msg_fw_soaring_data_get_xR(msg);
	fw_soaring_data->xLat = mavlink_msg_fw_soaring_data_get_xLat(msg);
	fw_soaring_data->xLon = mavlink_msg_fw_soaring_data_get_xLon(msg);
	fw_soaring_data->VarW = mavlink_msg_fw_soaring_data_get_VarW(msg);
	fw_soaring_data->VarR = mavlink_msg_fw_soaring_data_get_VarR(msg);
	fw_soaring_data->VarLat = mavlink_msg_fw_soaring_data_get_VarLat(msg);
	fw_soaring_data->VarLon = mavlink_msg_fw_soaring_data_get_VarLon(msg);
	fw_soaring_data->LoiterRadius = mavlink_msg_fw_soaring_data_get_LoiterRadius(msg);
	fw_soaring_data->ControlMode = mavlink_msg_fw_soaring_data_get_ControlMode(msg);
	fw_soaring_data->valid = mavlink_msg_fw_soaring_data_get_valid(msg);
#else
	memcpy(fw_soaring_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif
}
