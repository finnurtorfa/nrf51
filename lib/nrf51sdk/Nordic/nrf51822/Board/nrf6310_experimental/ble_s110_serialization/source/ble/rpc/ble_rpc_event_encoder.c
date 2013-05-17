/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <string.h>

#include "app_util.h"
#include "ble_gap.h"
#include "nrf_error.h"
#include "ble_rpc_event_encoder.h"
#include "rpc_transport.h"
#include "ble_rpc_cmd_defines.h"

static uint32_t peer_address_encode(uint8_t *                    p_packet,
                                    const ble_gap_addr_t * const p_peer_address)
{
    uint32_t i     = 0;
    uint32_t index = 0;

    p_packet[index++] = p_peer_address->addr_type;
    for (; i < BLE_GAP_ADDR_LEN; i++)
    {
        p_packet[index++] = p_peer_address->addr[i];
    }

    return index;
}


void rpc_event_encoder_write(ble_evt_t * p_ble_evt)
{
    /*lint -esym(438, index ) Last value assigned to index not used*/
    uint_fast8_t   index              = 0;
    uint8_t        packet[MAX_EVENT_LEN];
    uint32_t       err_code;
    uint32_t       i                  = 0;

    // Encode header
    index += uint16_encode(p_ble_evt->header.evt_id,  &packet[index]);

    // Encode common part of the event
    index += uint16_encode(p_ble_evt->evt.gap_evt.conn_handle, &packet[index]);

    // Encode events
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            index += peer_address_encode(&packet[index],
                                         &(p_ble_evt->evt.gap_evt.params.connected.peer_addr));

            packet[index]  = (p_ble_evt->evt.gap_evt.params.connected.irk_match_idx << 1) & 0xFE;
            packet[index] |= p_ble_evt->evt.gap_evt.params.connected.irk_match;
            index++;

            ble_gap_conn_params_t * p_conn_params;
            p_conn_params = &(p_ble_evt->evt.gap_evt.params.connected.conn_params);

            index += uint16_encode(p_conn_params->min_conn_interval, &packet[index]);
            index += uint16_encode(p_conn_params->max_conn_interval, &packet[index]);
            index += uint16_encode(p_conn_params->slave_latency,     &packet[index]);
            index += uint16_encode(p_conn_params->conn_sup_timeout,  &packet[index]);
            break;
        }

        case BLE_GAP_EVT_DISCONNECTED:
            index += peer_address_encode(&packet[index],
                                         &(p_ble_evt->evt.gap_evt.params.disconnected.peer_addr));
            packet[index++] = p_ble_evt->evt.gap_evt.params.disconnected.reason;
            break;

        case BLE_GAP_EVT_TIMEOUT:
            packet[index++] = p_ble_evt->evt.gap_evt.params.timeout.src;
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {
            ble_gap_conn_params_t * p_conn_params;
            p_conn_params = &(p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params);

            index += uint16_encode(p_conn_params->min_conn_interval, &packet[index]);
            index += uint16_encode(p_conn_params->max_conn_interval, &packet[index]);
            index += uint16_encode(p_conn_params->slave_latency,     &packet[index]);
            index += uint16_encode(p_conn_params->conn_sup_timeout,  &packet[index]);
            break;
        }

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        {
            ble_gap_evt_sec_params_request_t * p_sec_params_request;

            p_sec_params_request = &(p_ble_evt->evt.gap_evt.params.sec_params_request);

            index += uint16_encode(p_sec_params_request->peer_params.timeout, &packet[index]);

            packet[index++] = (p_sec_params_request->peer_params.oob     << 5) |
                              (p_sec_params_request->peer_params.io_caps << 2) |
                              (p_sec_params_request->peer_params.mitm    << 1) |
                              (p_sec_params_request->peer_params.bond);

            packet[index++] = p_sec_params_request->peer_params.min_key_size;
            packet[index++] = p_sec_params_request->peer_params.max_key_size;
            break;
        }

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
        {
            ble_gap_evt_sec_info_request_t * p_sec_info_request;
            p_sec_info_request = &(p_ble_evt->evt.gap_evt.params.sec_info_request);

            index += peer_address_encode(&packet[index], &(p_sec_info_request->peer_addr));
            index += uint16_encode(p_sec_info_request->div, &packet[index]);

            packet[index++] = (p_sec_info_request->sign_info << 2 ) |
                              (p_sec_info_request->id_info   << 1 ) |
                              (p_sec_info_request->enc_info);
            break;
        }

        case BLE_GAP_EVT_AUTH_STATUS:
        {
            ble_gap_evt_auth_status_t * p_auth_status;
            p_auth_status = &(p_ble_evt->evt.gap_evt.params.auth_status);

            packet[index++] = p_auth_status->auth_status;
            packet[index++] = p_auth_status->error_src;

            packet[index++] = (p_auth_status->sm1_levels.lv3 << 5) |
                              (p_auth_status->sm1_levels.lv2 << 4) |
                              (p_auth_status->sm1_levels.lv1 << 3) |
                              (p_auth_status->sm2_levels.lv3 << 2) |
                              (p_auth_status->sm2_levels.lv2 << 1) |
                              (p_auth_status->sm2_levels.lv1);

            packet[index++] = (p_auth_status->periph_kex.csrk       << 4) |
                              (p_auth_status->periph_kex.address    << 3) |
                              (p_auth_status->periph_kex.irk        << 2) |
                              (p_auth_status->periph_kex.ediv_rand  << 1) |
                              (p_auth_status->periph_kex.ltk);

            packet[index++] = (p_auth_status->central_kex.csrk      << 4) |
                              (p_auth_status->central_kex.address   << 3) |
                              (p_auth_status->central_kex.irk       << 2) |
                              (p_auth_status->central_kex.ediv_rand << 1) |
                              (p_auth_status->central_kex.ltk);

            index += uint16_encode(p_auth_status->periph_keys.enc_info.div, &packet[index]);

            for (i = 0; i < BLE_GAP_SEC_KEY_LEN; i++)
            {
                packet[index++] = p_auth_status->periph_keys.enc_info.ltk[i];
            }

            packet[index++] = (p_auth_status->periph_keys.enc_info.ltk_len << 1) |
                              (p_auth_status->periph_keys.enc_info.auth);

            for (i = 0; i < BLE_GAP_SEC_KEY_LEN; i++)
            {
                packet[index++] = (p_auth_status->central_keys.irk.irk[i]);
            }

            packet[index++] = p_auth_status->central_keys.id_info.addr_type;

            for (i = 0; i < BLE_GAP_ADDR_LEN; i++)
            {
                packet[index++] = (p_auth_status->central_keys.id_info.addr[i]);
            }
            break;
        }

        case BLE_GAP_EVT_CONN_SEC_UPDATE:
        {
            ble_gap_evt_conn_sec_update_t * p_conn_sec_update;
            p_conn_sec_update = &(p_ble_evt->evt.gap_evt.params.conn_sec_update);

            packet[index++] = p_conn_sec_update->conn_sec.sec_mode.sm |
                             (p_conn_sec_update->conn_sec.sec_mode.lv << 4);
            packet[index++] = p_conn_sec_update->conn_sec.encr_key_size;
            break;
        }

        case BLE_GATTS_EVT_WRITE:
        {
            ble_gatts_evt_write_t * p_evt_write;
            p_evt_write = &(p_ble_evt->evt.gatts_evt.params.write);

            index          += uint16_encode(p_evt_write->handle, &packet[index]);
            packet[index++] = p_evt_write->op;
            index          += uint16_encode(p_evt_write->context.srvc_uuid.uuid, &packet[index]);
            packet[index++] = p_evt_write->context.srvc_uuid.type;
            index          += uint16_encode(p_evt_write->context.char_uuid.uuid, &packet[index]);
            packet[index++] = p_evt_write->context.char_uuid.type;
            index          += uint16_encode(p_evt_write->context.desc_uuid.uuid, &packet[index]);
            packet[index++] = p_evt_write->context.desc_uuid.type;
            index          += uint16_encode(p_evt_write->context.srvc_handle, &packet[index]);
            index          += uint16_encode(p_evt_write->context.value_handle, &packet[index]);
            packet[index++] = p_evt_write->context.type;
            index          += uint16_encode(p_evt_write->offset, &packet[index]);
            index          += uint16_encode(p_evt_write->len, &packet[index]);

            if (p_evt_write->len != 0)
            {
                memcpy(&(packet[index]), p_evt_write->data, p_evt_write->len);
                index += p_evt_write->len;
            }

            break;
        }
                
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        {
            packet[index++] = p_ble_evt->evt.gatts_evt.params.sys_attr_missing.hint;            
            break;
        }
        
        default:
            break;

    }
    //lint -restore
    err_code = rpc_transport_packet_write(RPC_TRANSPORT_EVT, packet, index);
    APP_ERROR_CHECK(err_code);
}

