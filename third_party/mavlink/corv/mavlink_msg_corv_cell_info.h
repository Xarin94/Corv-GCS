#pragma once
// MESSAGE CORV_CELL_INFO PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_CORV_CELL_INFO 52200


typedef struct __mavlink_corv_cell_info_t {
 uint32_t eci; /*<  E-UTRAN cell identity (eNB id * 256 + sector).*/
 uint32_t tac; /*<  Tracking area code.*/
 uint32_t earfcn; /*<  Serving cell EARFCN.*/
 uint32_t handovers; /*<  Cell changes since boot.*/
 uint32_t nb_earfcn[4]; /*<  Neighbour EARFCNs.*/
 uint16_t mcc; /*<  Mobile country code.*/
 uint16_t mnc; /*<  Mobile network code.*/
 uint16_t pci; /*<  Physical cell id.*/
 int16_t rsrp; /*< [dBm] RSRP, INT16_MAX = unknown.*/
 int16_t rssi; /*< [dBm] RSSI, INT16_MAX = unknown.*/
 uint16_t nb_pci[4]; /*<  Neighbour PCIs, UINT16_MAX = none.*/
 int16_t nb_rsrp[4]; /*< [dBm] Neighbour RSRP.*/
 int8_t rsrq; /*< [dB] RSRQ, INT8_MAX = unknown.*/
 int8_t sinr; /*< [dB] SINR, INT8_MAX = unknown.*/
 uint8_t band; /*<  LTE band.*/
 uint8_t state; /*<  0 unknown, 1 search, 2 limited, 3 camped, 4 connected.*/
 uint8_t n_neighbours; /*<  Neighbour cells seen.*/
} mavlink_corv_cell_info_t;

#define MAVLINK_MSG_ID_CORV_CELL_INFO_LEN 63
#define MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN 63
#define MAVLINK_MSG_ID_52200_LEN 63
#define MAVLINK_MSG_ID_52200_MIN_LEN 63

#define MAVLINK_MSG_ID_CORV_CELL_INFO_CRC 119
#define MAVLINK_MSG_ID_52200_CRC 119

#define MAVLINK_MSG_CORV_CELL_INFO_FIELD_NB_EARFCN_LEN 4
#define MAVLINK_MSG_CORV_CELL_INFO_FIELD_NB_PCI_LEN 4
#define MAVLINK_MSG_CORV_CELL_INFO_FIELD_NB_RSRP_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CORV_CELL_INFO { \
    52200, \
    "CORV_CELL_INFO", \
    17, \
    {  { "eci", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_corv_cell_info_t, eci) }, \
         { "tac", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_corv_cell_info_t, tac) }, \
         { "earfcn", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_corv_cell_info_t, earfcn) }, \
         { "handovers", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_corv_cell_info_t, handovers) }, \
         { "nb_earfcn", NULL, MAVLINK_TYPE_UINT32_T, 4, 16, offsetof(mavlink_corv_cell_info_t, nb_earfcn) }, \
         { "mcc", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_corv_cell_info_t, mcc) }, \
         { "mnc", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_corv_cell_info_t, mnc) }, \
         { "pci", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_corv_cell_info_t, pci) }, \
         { "rsrp", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_corv_cell_info_t, rsrp) }, \
         { "rssi", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_corv_cell_info_t, rssi) }, \
         { "nb_pci", NULL, MAVLINK_TYPE_UINT16_T, 4, 42, offsetof(mavlink_corv_cell_info_t, nb_pci) }, \
         { "nb_rsrp", NULL, MAVLINK_TYPE_INT16_T, 4, 50, offsetof(mavlink_corv_cell_info_t, nb_rsrp) }, \
         { "rsrq", NULL, MAVLINK_TYPE_INT8_T, 0, 58, offsetof(mavlink_corv_cell_info_t, rsrq) }, \
         { "sinr", NULL, MAVLINK_TYPE_INT8_T, 0, 59, offsetof(mavlink_corv_cell_info_t, sinr) }, \
         { "band", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_corv_cell_info_t, band) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_corv_cell_info_t, state) }, \
         { "n_neighbours", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_corv_cell_info_t, n_neighbours) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CORV_CELL_INFO { \
    "CORV_CELL_INFO", \
    17, \
    {  { "eci", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_corv_cell_info_t, eci) }, \
         { "tac", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_corv_cell_info_t, tac) }, \
         { "earfcn", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_corv_cell_info_t, earfcn) }, \
         { "handovers", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_corv_cell_info_t, handovers) }, \
         { "nb_earfcn", NULL, MAVLINK_TYPE_UINT32_T, 4, 16, offsetof(mavlink_corv_cell_info_t, nb_earfcn) }, \
         { "mcc", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_corv_cell_info_t, mcc) }, \
         { "mnc", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_corv_cell_info_t, mnc) }, \
         { "pci", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_corv_cell_info_t, pci) }, \
         { "rsrp", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_corv_cell_info_t, rsrp) }, \
         { "rssi", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_corv_cell_info_t, rssi) }, \
         { "nb_pci", NULL, MAVLINK_TYPE_UINT16_T, 4, 42, offsetof(mavlink_corv_cell_info_t, nb_pci) }, \
         { "nb_rsrp", NULL, MAVLINK_TYPE_INT16_T, 4, 50, offsetof(mavlink_corv_cell_info_t, nb_rsrp) }, \
         { "rsrq", NULL, MAVLINK_TYPE_INT8_T, 0, 58, offsetof(mavlink_corv_cell_info_t, rsrq) }, \
         { "sinr", NULL, MAVLINK_TYPE_INT8_T, 0, 59, offsetof(mavlink_corv_cell_info_t, sinr) }, \
         { "band", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_corv_cell_info_t, band) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_corv_cell_info_t, state) }, \
         { "n_neighbours", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_corv_cell_info_t, n_neighbours) }, \
         } \
}
#endif

/**
 * @brief Pack a corv_cell_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param eci  E-UTRAN cell identity (eNB id * 256 + sector).
 * @param tac  Tracking area code.
 * @param earfcn  Serving cell EARFCN.
 * @param handovers  Cell changes since boot.
 * @param nb_earfcn  Neighbour EARFCNs.
 * @param mcc  Mobile country code.
 * @param mnc  Mobile network code.
 * @param pci  Physical cell id.
 * @param rsrp [dBm] RSRP, INT16_MAX = unknown.
 * @param rssi [dBm] RSSI, INT16_MAX = unknown.
 * @param nb_pci  Neighbour PCIs, UINT16_MAX = none.
 * @param nb_rsrp [dBm] Neighbour RSRP.
 * @param rsrq [dB] RSRQ, INT8_MAX = unknown.
 * @param sinr [dB] SINR, INT8_MAX = unknown.
 * @param band  LTE band.
 * @param state  0 unknown, 1 search, 2 limited, 3 camped, 4 connected.
 * @param n_neighbours  Neighbour cells seen.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_corv_cell_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t eci, uint32_t tac, uint32_t earfcn, uint32_t handovers, const uint32_t *nb_earfcn, uint16_t mcc, uint16_t mnc, uint16_t pci, int16_t rsrp, int16_t rssi, const uint16_t *nb_pci, const int16_t *nb_rsrp, int8_t rsrq, int8_t sinr, uint8_t band, uint8_t state, uint8_t n_neighbours)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CORV_CELL_INFO_LEN];
    _mav_put_uint32_t(buf, 0, eci);
    _mav_put_uint32_t(buf, 4, tac);
    _mav_put_uint32_t(buf, 8, earfcn);
    _mav_put_uint32_t(buf, 12, handovers);
    _mav_put_uint16_t(buf, 32, mcc);
    _mav_put_uint16_t(buf, 34, mnc);
    _mav_put_uint16_t(buf, 36, pci);
    _mav_put_int16_t(buf, 38, rsrp);
    _mav_put_int16_t(buf, 40, rssi);
    _mav_put_int8_t(buf, 58, rsrq);
    _mav_put_int8_t(buf, 59, sinr);
    _mav_put_uint8_t(buf, 60, band);
    _mav_put_uint8_t(buf, 61, state);
    _mav_put_uint8_t(buf, 62, n_neighbours);
    _mav_put_uint32_t_array(buf, 16, nb_earfcn, 4);
    _mav_put_uint16_t_array(buf, 42, nb_pci, 4);
    _mav_put_int16_t_array(buf, 50, nb_rsrp, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN);
#else
    mavlink_corv_cell_info_t packet;
    packet.eci = eci;
    packet.tac = tac;
    packet.earfcn = earfcn;
    packet.handovers = handovers;
    packet.mcc = mcc;
    packet.mnc = mnc;
    packet.pci = pci;
    packet.rsrp = rsrp;
    packet.rssi = rssi;
    packet.rsrq = rsrq;
    packet.sinr = sinr;
    packet.band = band;
    packet.state = state;
    packet.n_neighbours = n_neighbours;
    mav_array_memcpy(packet.nb_earfcn, nb_earfcn, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.nb_pci, nb_pci, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.nb_rsrp, nb_rsrp, sizeof(int16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CORV_CELL_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_CRC);
}

/**
 * @brief Pack a corv_cell_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param eci  E-UTRAN cell identity (eNB id * 256 + sector).
 * @param tac  Tracking area code.
 * @param earfcn  Serving cell EARFCN.
 * @param handovers  Cell changes since boot.
 * @param nb_earfcn  Neighbour EARFCNs.
 * @param mcc  Mobile country code.
 * @param mnc  Mobile network code.
 * @param pci  Physical cell id.
 * @param rsrp [dBm] RSRP, INT16_MAX = unknown.
 * @param rssi [dBm] RSSI, INT16_MAX = unknown.
 * @param nb_pci  Neighbour PCIs, UINT16_MAX = none.
 * @param nb_rsrp [dBm] Neighbour RSRP.
 * @param rsrq [dB] RSRQ, INT8_MAX = unknown.
 * @param sinr [dB] SINR, INT8_MAX = unknown.
 * @param band  LTE band.
 * @param state  0 unknown, 1 search, 2 limited, 3 camped, 4 connected.
 * @param n_neighbours  Neighbour cells seen.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_corv_cell_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t eci, uint32_t tac, uint32_t earfcn, uint32_t handovers, const uint32_t *nb_earfcn, uint16_t mcc, uint16_t mnc, uint16_t pci, int16_t rsrp, int16_t rssi, const uint16_t *nb_pci, const int16_t *nb_rsrp, int8_t rsrq, int8_t sinr, uint8_t band, uint8_t state, uint8_t n_neighbours)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CORV_CELL_INFO_LEN];
    _mav_put_uint32_t(buf, 0, eci);
    _mav_put_uint32_t(buf, 4, tac);
    _mav_put_uint32_t(buf, 8, earfcn);
    _mav_put_uint32_t(buf, 12, handovers);
    _mav_put_uint16_t(buf, 32, mcc);
    _mav_put_uint16_t(buf, 34, mnc);
    _mav_put_uint16_t(buf, 36, pci);
    _mav_put_int16_t(buf, 38, rsrp);
    _mav_put_int16_t(buf, 40, rssi);
    _mav_put_int8_t(buf, 58, rsrq);
    _mav_put_int8_t(buf, 59, sinr);
    _mav_put_uint8_t(buf, 60, band);
    _mav_put_uint8_t(buf, 61, state);
    _mav_put_uint8_t(buf, 62, n_neighbours);
    _mav_put_uint32_t_array(buf, 16, nb_earfcn, 4);
    _mav_put_uint16_t_array(buf, 42, nb_pci, 4);
    _mav_put_int16_t_array(buf, 50, nb_rsrp, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN);
#else
    mavlink_corv_cell_info_t packet;
    packet.eci = eci;
    packet.tac = tac;
    packet.earfcn = earfcn;
    packet.handovers = handovers;
    packet.mcc = mcc;
    packet.mnc = mnc;
    packet.pci = pci;
    packet.rsrp = rsrp;
    packet.rssi = rssi;
    packet.rsrq = rsrq;
    packet.sinr = sinr;
    packet.band = band;
    packet.state = state;
    packet.n_neighbours = n_neighbours;
    mav_array_memcpy(packet.nb_earfcn, nb_earfcn, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.nb_pci, nb_pci, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.nb_rsrp, nb_rsrp, sizeof(int16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CORV_CELL_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN);
#endif
}

/**
 * @brief Pack a corv_cell_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param eci  E-UTRAN cell identity (eNB id * 256 + sector).
 * @param tac  Tracking area code.
 * @param earfcn  Serving cell EARFCN.
 * @param handovers  Cell changes since boot.
 * @param nb_earfcn  Neighbour EARFCNs.
 * @param mcc  Mobile country code.
 * @param mnc  Mobile network code.
 * @param pci  Physical cell id.
 * @param rsrp [dBm] RSRP, INT16_MAX = unknown.
 * @param rssi [dBm] RSSI, INT16_MAX = unknown.
 * @param nb_pci  Neighbour PCIs, UINT16_MAX = none.
 * @param nb_rsrp [dBm] Neighbour RSRP.
 * @param rsrq [dB] RSRQ, INT8_MAX = unknown.
 * @param sinr [dB] SINR, INT8_MAX = unknown.
 * @param band  LTE band.
 * @param state  0 unknown, 1 search, 2 limited, 3 camped, 4 connected.
 * @param n_neighbours  Neighbour cells seen.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_corv_cell_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t eci,uint32_t tac,uint32_t earfcn,uint32_t handovers,const uint32_t *nb_earfcn,uint16_t mcc,uint16_t mnc,uint16_t pci,int16_t rsrp,int16_t rssi,const uint16_t *nb_pci,const int16_t *nb_rsrp,int8_t rsrq,int8_t sinr,uint8_t band,uint8_t state,uint8_t n_neighbours)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CORV_CELL_INFO_LEN];
    _mav_put_uint32_t(buf, 0, eci);
    _mav_put_uint32_t(buf, 4, tac);
    _mav_put_uint32_t(buf, 8, earfcn);
    _mav_put_uint32_t(buf, 12, handovers);
    _mav_put_uint16_t(buf, 32, mcc);
    _mav_put_uint16_t(buf, 34, mnc);
    _mav_put_uint16_t(buf, 36, pci);
    _mav_put_int16_t(buf, 38, rsrp);
    _mav_put_int16_t(buf, 40, rssi);
    _mav_put_int8_t(buf, 58, rsrq);
    _mav_put_int8_t(buf, 59, sinr);
    _mav_put_uint8_t(buf, 60, band);
    _mav_put_uint8_t(buf, 61, state);
    _mav_put_uint8_t(buf, 62, n_neighbours);
    _mav_put_uint32_t_array(buf, 16, nb_earfcn, 4);
    _mav_put_uint16_t_array(buf, 42, nb_pci, 4);
    _mav_put_int16_t_array(buf, 50, nb_rsrp, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN);
#else
    mavlink_corv_cell_info_t packet;
    packet.eci = eci;
    packet.tac = tac;
    packet.earfcn = earfcn;
    packet.handovers = handovers;
    packet.mcc = mcc;
    packet.mnc = mnc;
    packet.pci = pci;
    packet.rsrp = rsrp;
    packet.rssi = rssi;
    packet.rsrq = rsrq;
    packet.sinr = sinr;
    packet.band = band;
    packet.state = state;
    packet.n_neighbours = n_neighbours;
    mav_array_memcpy(packet.nb_earfcn, nb_earfcn, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.nb_pci, nb_pci, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.nb_rsrp, nb_rsrp, sizeof(int16_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CORV_CELL_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_CRC);
}

/**
 * @brief Encode a corv_cell_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param corv_cell_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_corv_cell_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_corv_cell_info_t* corv_cell_info)
{
    return mavlink_msg_corv_cell_info_pack(system_id, component_id, msg, corv_cell_info->eci, corv_cell_info->tac, corv_cell_info->earfcn, corv_cell_info->handovers, corv_cell_info->nb_earfcn, corv_cell_info->mcc, corv_cell_info->mnc, corv_cell_info->pci, corv_cell_info->rsrp, corv_cell_info->rssi, corv_cell_info->nb_pci, corv_cell_info->nb_rsrp, corv_cell_info->rsrq, corv_cell_info->sinr, corv_cell_info->band, corv_cell_info->state, corv_cell_info->n_neighbours);
}

/**
 * @brief Encode a corv_cell_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param corv_cell_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_corv_cell_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_corv_cell_info_t* corv_cell_info)
{
    return mavlink_msg_corv_cell_info_pack_chan(system_id, component_id, chan, msg, corv_cell_info->eci, corv_cell_info->tac, corv_cell_info->earfcn, corv_cell_info->handovers, corv_cell_info->nb_earfcn, corv_cell_info->mcc, corv_cell_info->mnc, corv_cell_info->pci, corv_cell_info->rsrp, corv_cell_info->rssi, corv_cell_info->nb_pci, corv_cell_info->nb_rsrp, corv_cell_info->rsrq, corv_cell_info->sinr, corv_cell_info->band, corv_cell_info->state, corv_cell_info->n_neighbours);
}

/**
 * @brief Encode a corv_cell_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param corv_cell_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_corv_cell_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_corv_cell_info_t* corv_cell_info)
{
    return mavlink_msg_corv_cell_info_pack_status(system_id, component_id, _status, msg,  corv_cell_info->eci, corv_cell_info->tac, corv_cell_info->earfcn, corv_cell_info->handovers, corv_cell_info->nb_earfcn, corv_cell_info->mcc, corv_cell_info->mnc, corv_cell_info->pci, corv_cell_info->rsrp, corv_cell_info->rssi, corv_cell_info->nb_pci, corv_cell_info->nb_rsrp, corv_cell_info->rsrq, corv_cell_info->sinr, corv_cell_info->band, corv_cell_info->state, corv_cell_info->n_neighbours);
}

/**
 * @brief Send a corv_cell_info message
 * @param chan MAVLink channel to send the message
 *
 * @param eci  E-UTRAN cell identity (eNB id * 256 + sector).
 * @param tac  Tracking area code.
 * @param earfcn  Serving cell EARFCN.
 * @param handovers  Cell changes since boot.
 * @param nb_earfcn  Neighbour EARFCNs.
 * @param mcc  Mobile country code.
 * @param mnc  Mobile network code.
 * @param pci  Physical cell id.
 * @param rsrp [dBm] RSRP, INT16_MAX = unknown.
 * @param rssi [dBm] RSSI, INT16_MAX = unknown.
 * @param nb_pci  Neighbour PCIs, UINT16_MAX = none.
 * @param nb_rsrp [dBm] Neighbour RSRP.
 * @param rsrq [dB] RSRQ, INT8_MAX = unknown.
 * @param sinr [dB] SINR, INT8_MAX = unknown.
 * @param band  LTE band.
 * @param state  0 unknown, 1 search, 2 limited, 3 camped, 4 connected.
 * @param n_neighbours  Neighbour cells seen.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_corv_cell_info_send(mavlink_channel_t chan, uint32_t eci, uint32_t tac, uint32_t earfcn, uint32_t handovers, const uint32_t *nb_earfcn, uint16_t mcc, uint16_t mnc, uint16_t pci, int16_t rsrp, int16_t rssi, const uint16_t *nb_pci, const int16_t *nb_rsrp, int8_t rsrq, int8_t sinr, uint8_t band, uint8_t state, uint8_t n_neighbours)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CORV_CELL_INFO_LEN];
    _mav_put_uint32_t(buf, 0, eci);
    _mav_put_uint32_t(buf, 4, tac);
    _mav_put_uint32_t(buf, 8, earfcn);
    _mav_put_uint32_t(buf, 12, handovers);
    _mav_put_uint16_t(buf, 32, mcc);
    _mav_put_uint16_t(buf, 34, mnc);
    _mav_put_uint16_t(buf, 36, pci);
    _mav_put_int16_t(buf, 38, rsrp);
    _mav_put_int16_t(buf, 40, rssi);
    _mav_put_int8_t(buf, 58, rsrq);
    _mav_put_int8_t(buf, 59, sinr);
    _mav_put_uint8_t(buf, 60, band);
    _mav_put_uint8_t(buf, 61, state);
    _mav_put_uint8_t(buf, 62, n_neighbours);
    _mav_put_uint32_t_array(buf, 16, nb_earfcn, 4);
    _mav_put_uint16_t_array(buf, 42, nb_pci, 4);
    _mav_put_int16_t_array(buf, 50, nb_rsrp, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CORV_CELL_INFO, buf, MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_CRC);
#else
    mavlink_corv_cell_info_t packet;
    packet.eci = eci;
    packet.tac = tac;
    packet.earfcn = earfcn;
    packet.handovers = handovers;
    packet.mcc = mcc;
    packet.mnc = mnc;
    packet.pci = pci;
    packet.rsrp = rsrp;
    packet.rssi = rssi;
    packet.rsrq = rsrq;
    packet.sinr = sinr;
    packet.band = band;
    packet.state = state;
    packet.n_neighbours = n_neighbours;
    mav_array_memcpy(packet.nb_earfcn, nb_earfcn, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.nb_pci, nb_pci, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.nb_rsrp, nb_rsrp, sizeof(int16_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CORV_CELL_INFO, (const char *)&packet, MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_CRC);
#endif
}

/**
 * @brief Send a corv_cell_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_corv_cell_info_send_struct(mavlink_channel_t chan, const mavlink_corv_cell_info_t* corv_cell_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_corv_cell_info_send(chan, corv_cell_info->eci, corv_cell_info->tac, corv_cell_info->earfcn, corv_cell_info->handovers, corv_cell_info->nb_earfcn, corv_cell_info->mcc, corv_cell_info->mnc, corv_cell_info->pci, corv_cell_info->rsrp, corv_cell_info->rssi, corv_cell_info->nb_pci, corv_cell_info->nb_rsrp, corv_cell_info->rsrq, corv_cell_info->sinr, corv_cell_info->band, corv_cell_info->state, corv_cell_info->n_neighbours);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CORV_CELL_INFO, (const char *)corv_cell_info, MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_CORV_CELL_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_corv_cell_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t eci, uint32_t tac, uint32_t earfcn, uint32_t handovers, const uint32_t *nb_earfcn, uint16_t mcc, uint16_t mnc, uint16_t pci, int16_t rsrp, int16_t rssi, const uint16_t *nb_pci, const int16_t *nb_rsrp, int8_t rsrq, int8_t sinr, uint8_t band, uint8_t state, uint8_t n_neighbours)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, eci);
    _mav_put_uint32_t(buf, 4, tac);
    _mav_put_uint32_t(buf, 8, earfcn);
    _mav_put_uint32_t(buf, 12, handovers);
    _mav_put_uint16_t(buf, 32, mcc);
    _mav_put_uint16_t(buf, 34, mnc);
    _mav_put_uint16_t(buf, 36, pci);
    _mav_put_int16_t(buf, 38, rsrp);
    _mav_put_int16_t(buf, 40, rssi);
    _mav_put_int8_t(buf, 58, rsrq);
    _mav_put_int8_t(buf, 59, sinr);
    _mav_put_uint8_t(buf, 60, band);
    _mav_put_uint8_t(buf, 61, state);
    _mav_put_uint8_t(buf, 62, n_neighbours);
    _mav_put_uint32_t_array(buf, 16, nb_earfcn, 4);
    _mav_put_uint16_t_array(buf, 42, nb_pci, 4);
    _mav_put_int16_t_array(buf, 50, nb_rsrp, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CORV_CELL_INFO, buf, MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_CRC);
#else
    mavlink_corv_cell_info_t *packet = (mavlink_corv_cell_info_t *)msgbuf;
    packet->eci = eci;
    packet->tac = tac;
    packet->earfcn = earfcn;
    packet->handovers = handovers;
    packet->mcc = mcc;
    packet->mnc = mnc;
    packet->pci = pci;
    packet->rsrp = rsrp;
    packet->rssi = rssi;
    packet->rsrq = rsrq;
    packet->sinr = sinr;
    packet->band = band;
    packet->state = state;
    packet->n_neighbours = n_neighbours;
    mav_array_memcpy(packet->nb_earfcn, nb_earfcn, sizeof(uint32_t)*4);
    mav_array_memcpy(packet->nb_pci, nb_pci, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->nb_rsrp, nb_rsrp, sizeof(int16_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CORV_CELL_INFO, (const char *)packet, MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN, MAVLINK_MSG_ID_CORV_CELL_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE CORV_CELL_INFO UNPACKING


/**
 * @brief Get field eci from corv_cell_info message
 *
 * @return  E-UTRAN cell identity (eNB id * 256 + sector).
 */
static inline uint32_t mavlink_msg_corv_cell_info_get_eci(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field tac from corv_cell_info message
 *
 * @return  Tracking area code.
 */
static inline uint32_t mavlink_msg_corv_cell_info_get_tac(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field earfcn from corv_cell_info message
 *
 * @return  Serving cell EARFCN.
 */
static inline uint32_t mavlink_msg_corv_cell_info_get_earfcn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field handovers from corv_cell_info message
 *
 * @return  Cell changes since boot.
 */
static inline uint32_t mavlink_msg_corv_cell_info_get_handovers(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field nb_earfcn from corv_cell_info message
 *
 * @return  Neighbour EARFCNs.
 */
static inline uint16_t mavlink_msg_corv_cell_info_get_nb_earfcn(const mavlink_message_t* msg, uint32_t *nb_earfcn)
{
    return _MAV_RETURN_uint32_t_array(msg, nb_earfcn, 4,  16);
}

/**
 * @brief Get field mcc from corv_cell_info message
 *
 * @return  Mobile country code.
 */
static inline uint16_t mavlink_msg_corv_cell_info_get_mcc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field mnc from corv_cell_info message
 *
 * @return  Mobile network code.
 */
static inline uint16_t mavlink_msg_corv_cell_info_get_mnc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field pci from corv_cell_info message
 *
 * @return  Physical cell id.
 */
static inline uint16_t mavlink_msg_corv_cell_info_get_pci(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field rsrp from corv_cell_info message
 *
 * @return [dBm] RSRP, INT16_MAX = unknown.
 */
static inline int16_t mavlink_msg_corv_cell_info_get_rsrp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  38);
}

/**
 * @brief Get field rssi from corv_cell_info message
 *
 * @return [dBm] RSSI, INT16_MAX = unknown.
 */
static inline int16_t mavlink_msg_corv_cell_info_get_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  40);
}

/**
 * @brief Get field nb_pci from corv_cell_info message
 *
 * @return  Neighbour PCIs, UINT16_MAX = none.
 */
static inline uint16_t mavlink_msg_corv_cell_info_get_nb_pci(const mavlink_message_t* msg, uint16_t *nb_pci)
{
    return _MAV_RETURN_uint16_t_array(msg, nb_pci, 4,  42);
}

/**
 * @brief Get field nb_rsrp from corv_cell_info message
 *
 * @return [dBm] Neighbour RSRP.
 */
static inline uint16_t mavlink_msg_corv_cell_info_get_nb_rsrp(const mavlink_message_t* msg, int16_t *nb_rsrp)
{
    return _MAV_RETURN_int16_t_array(msg, nb_rsrp, 4,  50);
}

/**
 * @brief Get field rsrq from corv_cell_info message
 *
 * @return [dB] RSRQ, INT8_MAX = unknown.
 */
static inline int8_t mavlink_msg_corv_cell_info_get_rsrq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  58);
}

/**
 * @brief Get field sinr from corv_cell_info message
 *
 * @return [dB] SINR, INT8_MAX = unknown.
 */
static inline int8_t mavlink_msg_corv_cell_info_get_sinr(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  59);
}

/**
 * @brief Get field band from corv_cell_info message
 *
 * @return  LTE band.
 */
static inline uint8_t mavlink_msg_corv_cell_info_get_band(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Get field state from corv_cell_info message
 *
 * @return  0 unknown, 1 search, 2 limited, 3 camped, 4 connected.
 */
static inline uint8_t mavlink_msg_corv_cell_info_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  61);
}

/**
 * @brief Get field n_neighbours from corv_cell_info message
 *
 * @return  Neighbour cells seen.
 */
static inline uint8_t mavlink_msg_corv_cell_info_get_n_neighbours(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  62);
}

/**
 * @brief Decode a corv_cell_info message into a struct
 *
 * @param msg The message to decode
 * @param corv_cell_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_corv_cell_info_decode(const mavlink_message_t* msg, mavlink_corv_cell_info_t* corv_cell_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    corv_cell_info->eci = mavlink_msg_corv_cell_info_get_eci(msg);
    corv_cell_info->tac = mavlink_msg_corv_cell_info_get_tac(msg);
    corv_cell_info->earfcn = mavlink_msg_corv_cell_info_get_earfcn(msg);
    corv_cell_info->handovers = mavlink_msg_corv_cell_info_get_handovers(msg);
    mavlink_msg_corv_cell_info_get_nb_earfcn(msg, corv_cell_info->nb_earfcn);
    corv_cell_info->mcc = mavlink_msg_corv_cell_info_get_mcc(msg);
    corv_cell_info->mnc = mavlink_msg_corv_cell_info_get_mnc(msg);
    corv_cell_info->pci = mavlink_msg_corv_cell_info_get_pci(msg);
    corv_cell_info->rsrp = mavlink_msg_corv_cell_info_get_rsrp(msg);
    corv_cell_info->rssi = mavlink_msg_corv_cell_info_get_rssi(msg);
    mavlink_msg_corv_cell_info_get_nb_pci(msg, corv_cell_info->nb_pci);
    mavlink_msg_corv_cell_info_get_nb_rsrp(msg, corv_cell_info->nb_rsrp);
    corv_cell_info->rsrq = mavlink_msg_corv_cell_info_get_rsrq(msg);
    corv_cell_info->sinr = mavlink_msg_corv_cell_info_get_sinr(msg);
    corv_cell_info->band = mavlink_msg_corv_cell_info_get_band(msg);
    corv_cell_info->state = mavlink_msg_corv_cell_info_get_state(msg);
    corv_cell_info->n_neighbours = mavlink_msg_corv_cell_info_get_n_neighbours(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CORV_CELL_INFO_LEN? msg->len : MAVLINK_MSG_ID_CORV_CELL_INFO_LEN;
        memset(corv_cell_info, 0, MAVLINK_MSG_ID_CORV_CELL_INFO_LEN);
    memcpy(corv_cell_info, _MAV_PAYLOAD(msg), len);
#endif
}
