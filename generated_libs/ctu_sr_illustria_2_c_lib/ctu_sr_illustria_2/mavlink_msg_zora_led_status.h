#pragma once
// MESSAGE ZORA_LED_STATUS PACKING

#define MAVLINK_MSG_ID_ZORA_LED_STATUS 10


typedef struct __mavlink_zora_led_status_t {
 uint8_t LED_GREEN_STATUS; /*<  LED Green status*/
 uint8_t LED_ORANGE_STATUS; /*<  LED Green status*/
 uint8_t LED_RED_STATUS; /*<  LED Green status*/
} mavlink_zora_led_status_t;

#define MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN 3
#define MAVLINK_MSG_ID_ZORA_LED_STATUS_MIN_LEN 3
#define MAVLINK_MSG_ID_10_LEN 3
#define MAVLINK_MSG_ID_10_MIN_LEN 3

#define MAVLINK_MSG_ID_ZORA_LED_STATUS_CRC 53
#define MAVLINK_MSG_ID_10_CRC 53



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ZORA_LED_STATUS { \
    10, \
    "ZORA_LED_STATUS", \
    3, \
    {  { "LED_GREEN_STATUS", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_zora_led_status_t, LED_GREEN_STATUS) }, \
         { "LED_ORANGE_STATUS", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_zora_led_status_t, LED_ORANGE_STATUS) }, \
         { "LED_RED_STATUS", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_zora_led_status_t, LED_RED_STATUS) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ZORA_LED_STATUS { \
    "ZORA_LED_STATUS", \
    3, \
    {  { "LED_GREEN_STATUS", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_zora_led_status_t, LED_GREEN_STATUS) }, \
         { "LED_ORANGE_STATUS", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_zora_led_status_t, LED_ORANGE_STATUS) }, \
         { "LED_RED_STATUS", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_zora_led_status_t, LED_RED_STATUS) }, \
         } \
}
#endif

/**
 * @brief Pack a zora_led_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param LED_GREEN_STATUS  LED Green status
 * @param LED_ORANGE_STATUS  LED Green status
 * @param LED_RED_STATUS  LED Green status
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zora_led_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t LED_GREEN_STATUS, uint8_t LED_ORANGE_STATUS, uint8_t LED_RED_STATUS)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, LED_GREEN_STATUS);
    _mav_put_uint8_t(buf, 1, LED_ORANGE_STATUS);
    _mav_put_uint8_t(buf, 2, LED_RED_STATUS);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN);
#else
    mavlink_zora_led_status_t packet;
    packet.LED_GREEN_STATUS = LED_GREEN_STATUS;
    packet.LED_ORANGE_STATUS = LED_ORANGE_STATUS;
    packet.LED_RED_STATUS = LED_RED_STATUS;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZORA_LED_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ZORA_LED_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_CRC);
}

/**
 * @brief Pack a zora_led_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param LED_GREEN_STATUS  LED Green status
 * @param LED_ORANGE_STATUS  LED Green status
 * @param LED_RED_STATUS  LED Green status
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_zora_led_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t LED_GREEN_STATUS,uint8_t LED_ORANGE_STATUS,uint8_t LED_RED_STATUS)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, LED_GREEN_STATUS);
    _mav_put_uint8_t(buf, 1, LED_ORANGE_STATUS);
    _mav_put_uint8_t(buf, 2, LED_RED_STATUS);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN);
#else
    mavlink_zora_led_status_t packet;
    packet.LED_GREEN_STATUS = LED_GREEN_STATUS;
    packet.LED_ORANGE_STATUS = LED_ORANGE_STATUS;
    packet.LED_RED_STATUS = LED_RED_STATUS;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ZORA_LED_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ZORA_LED_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_CRC);
}

/**
 * @brief Encode a zora_led_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param zora_led_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zora_led_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_zora_led_status_t* zora_led_status)
{
    return mavlink_msg_zora_led_status_pack(system_id, component_id, msg, zora_led_status->LED_GREEN_STATUS, zora_led_status->LED_ORANGE_STATUS, zora_led_status->LED_RED_STATUS);
}

/**
 * @brief Encode a zora_led_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param zora_led_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_zora_led_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_zora_led_status_t* zora_led_status)
{
    return mavlink_msg_zora_led_status_pack_chan(system_id, component_id, chan, msg, zora_led_status->LED_GREEN_STATUS, zora_led_status->LED_ORANGE_STATUS, zora_led_status->LED_RED_STATUS);
}

/**
 * @brief Send a zora_led_status message
 * @param chan MAVLink channel to send the message
 *
 * @param LED_GREEN_STATUS  LED Green status
 * @param LED_ORANGE_STATUS  LED Green status
 * @param LED_RED_STATUS  LED Green status
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_zora_led_status_send(mavlink_channel_t chan, uint8_t LED_GREEN_STATUS, uint8_t LED_ORANGE_STATUS, uint8_t LED_RED_STATUS)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, LED_GREEN_STATUS);
    _mav_put_uint8_t(buf, 1, LED_ORANGE_STATUS);
    _mav_put_uint8_t(buf, 2, LED_RED_STATUS);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZORA_LED_STATUS, buf, MAVLINK_MSG_ID_ZORA_LED_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_CRC);
#else
    mavlink_zora_led_status_t packet;
    packet.LED_GREEN_STATUS = LED_GREEN_STATUS;
    packet.LED_ORANGE_STATUS = LED_ORANGE_STATUS;
    packet.LED_RED_STATUS = LED_RED_STATUS;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZORA_LED_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ZORA_LED_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_CRC);
#endif
}

/**
 * @brief Send a zora_led_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_zora_led_status_send_struct(mavlink_channel_t chan, const mavlink_zora_led_status_t* zora_led_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_zora_led_status_send(chan, zora_led_status->LED_GREEN_STATUS, zora_led_status->LED_ORANGE_STATUS, zora_led_status->LED_RED_STATUS);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZORA_LED_STATUS, (const char *)zora_led_status, MAVLINK_MSG_ID_ZORA_LED_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_zora_led_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t LED_GREEN_STATUS, uint8_t LED_ORANGE_STATUS, uint8_t LED_RED_STATUS)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, LED_GREEN_STATUS);
    _mav_put_uint8_t(buf, 1, LED_ORANGE_STATUS);
    _mav_put_uint8_t(buf, 2, LED_RED_STATUS);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZORA_LED_STATUS, buf, MAVLINK_MSG_ID_ZORA_LED_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_CRC);
#else
    mavlink_zora_led_status_t *packet = (mavlink_zora_led_status_t *)msgbuf;
    packet->LED_GREEN_STATUS = LED_GREEN_STATUS;
    packet->LED_ORANGE_STATUS = LED_ORANGE_STATUS;
    packet->LED_RED_STATUS = LED_RED_STATUS;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ZORA_LED_STATUS, (const char *)packet, MAVLINK_MSG_ID_ZORA_LED_STATUS_MIN_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN, MAVLINK_MSG_ID_ZORA_LED_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ZORA_LED_STATUS UNPACKING


/**
 * @brief Get field LED_GREEN_STATUS from zora_led_status message
 *
 * @return  LED Green status
 */
static inline uint8_t mavlink_msg_zora_led_status_get_LED_GREEN_STATUS(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field LED_ORANGE_STATUS from zora_led_status message
 *
 * @return  LED Green status
 */
static inline uint8_t mavlink_msg_zora_led_status_get_LED_ORANGE_STATUS(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field LED_RED_STATUS from zora_led_status message
 *
 * @return  LED Green status
 */
static inline uint8_t mavlink_msg_zora_led_status_get_LED_RED_STATUS(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a zora_led_status message into a struct
 *
 * @param msg The message to decode
 * @param zora_led_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_zora_led_status_decode(const mavlink_message_t* msg, mavlink_zora_led_status_t* zora_led_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    zora_led_status->LED_GREEN_STATUS = mavlink_msg_zora_led_status_get_LED_GREEN_STATUS(msg);
    zora_led_status->LED_ORANGE_STATUS = mavlink_msg_zora_led_status_get_LED_ORANGE_STATUS(msg);
    zora_led_status->LED_RED_STATUS = mavlink_msg_zora_led_status_get_LED_RED_STATUS(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN;
        memset(zora_led_status, 0, MAVLINK_MSG_ID_ZORA_LED_STATUS_LEN);
    memcpy(zora_led_status, _MAV_PAYLOAD(msg), len);
#endif
}
