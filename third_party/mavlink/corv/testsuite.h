/** @file
 *    @brief MAVLink comm protocol testsuite generated from corv.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef CORV_TESTSUITE_H
#define CORV_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_ardupilotmega(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_development(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_corv(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ardupilotmega(system_id, component_id, last_msg);
    mavlink_test_development(system_id, component_id, last_msg);
    mavlink_test_corv(system_id, component_id, last_msg);
}
#endif

#include "../ardupilotmega/testsuite.h"
#include "../development/testsuite.h"


static void mavlink_test_corv_cell_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CORV_CELL_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_corv_cell_info_t packet_in = {
        963497464,963497672,963497880,963498088,{ 963498296, 963498297, 963498298, 963498299 },18899,19003,19107,19211,19315,{ 19419, 19420, 19421, 19422 },{ 19835, 19836, 19837, 19838 },51,118,185,252,63
    };
    mavlink_corv_cell_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.eci = packet_in.eci;
        packet1.tac = packet_in.tac;
        packet1.earfcn = packet_in.earfcn;
        packet1.handovers = packet_in.handovers;
        packet1.mcc = packet_in.mcc;
        packet1.mnc = packet_in.mnc;
        packet1.pci = packet_in.pci;
        packet1.rsrp = packet_in.rsrp;
        packet1.rssi = packet_in.rssi;
        packet1.rsrq = packet_in.rsrq;
        packet1.sinr = packet_in.sinr;
        packet1.band = packet_in.band;
        packet1.state = packet_in.state;
        packet1.n_neighbours = packet_in.n_neighbours;
        
        mav_array_memcpy(packet1.nb_earfcn, packet_in.nb_earfcn, sizeof(uint32_t)*4);
        mav_array_memcpy(packet1.nb_pci, packet_in.nb_pci, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.nb_rsrp, packet_in.nb_rsrp, sizeof(int16_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CORV_CELL_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_corv_cell_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_corv_cell_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_corv_cell_info_pack(system_id, component_id, &msg , packet1.eci , packet1.tac , packet1.earfcn , packet1.handovers , packet1.nb_earfcn , packet1.mcc , packet1.mnc , packet1.pci , packet1.rsrp , packet1.rssi , packet1.nb_pci , packet1.nb_rsrp , packet1.rsrq , packet1.sinr , packet1.band , packet1.state , packet1.n_neighbours );
    mavlink_msg_corv_cell_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_corv_cell_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.eci , packet1.tac , packet1.earfcn , packet1.handovers , packet1.nb_earfcn , packet1.mcc , packet1.mnc , packet1.pci , packet1.rsrp , packet1.rssi , packet1.nb_pci , packet1.nb_rsrp , packet1.rsrq , packet1.sinr , packet1.band , packet1.state , packet1.n_neighbours );
    mavlink_msg_corv_cell_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_corv_cell_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_corv_cell_info_send(MAVLINK_COMM_1 , packet1.eci , packet1.tac , packet1.earfcn , packet1.handovers , packet1.nb_earfcn , packet1.mcc , packet1.mnc , packet1.pci , packet1.rsrp , packet1.rssi , packet1.nb_pci , packet1.nb_rsrp , packet1.rsrq , packet1.sinr , packet1.band , packet1.state , packet1.n_neighbours );
    mavlink_msg_corv_cell_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CORV_CELL_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CORV_CELL_INFO) != NULL);
#endif
}

static void mavlink_test_corv(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_corv_cell_info(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CORV_TESTSUITE_H
