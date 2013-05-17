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

#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "nrf51.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "rpc_transport.h"
#include "ble_rpc_cmd_defines.h" 
#include "ble.h"
#include "app_util.h"
#include "app_fifo.h"

#ifndef SVCALL_AS_NORMAL_FUNCTION
#error "The compiler define SVCALL_AS_NORMAL_FUNCTION is not defined."
#endif

/** @brief Macro for checking if a pointer is aligned to the required boundary. */
#define IS_ALIGNED(ptr, byte_boundary) (                                         \
                                        (                                        \
                                         (ptr + (byte_boundary - 1))             \
                                         &                                       \
                                         (~(byte_boundary - 1))                  \
                                        )                                        \
                                        ==                                       \
                                        ptr                                      \
                                       )

/** @todo: Use queue for BLE Events. */
/** @todo: Address problems regarding thread safety.*/
static uint32_t    m_event_buffer[MAX_EVENT_LEN / sizeof(uint32_t)];  /**< Using 32-bit array to force memory alignment. */
static ble_evt_t * mp_ble_evt = (ble_evt_t *)m_event_buffer;          /**< BLE Event Structure that is passed to the application when an event is received. */


/** @brief Function for decoding a Bluetooth device address.
 *
 * @param[in]       p_encoded_ble_addr The pointer to the encoded Bluetooth device address.
 * @param[in, out]  p_addr             The pointer to the decoded address.
 * @return          Number of bytes decoded.
 */
static uint8_t ble_addr_decode(const uint8_t * const  p_encoded_ble_addr,
                               ble_gap_addr_t * const p_addr)
{
    uint8_t index = 0;

    p_addr->addr_type = p_encoded_ble_addr[index++];

    memcpy(p_addr->addr, &(p_encoded_ble_addr[index]), BLE_GAP_ADDR_LEN);
    index += BLE_GAP_ADDR_LEN;

    return index;
}


/** @brief Function for decoding a connected event.
 *
 * @param[in]       p_evt_data      The pointer to the encoded event (excluding the event
 *                                  header).
 * @param[in, out]  p_decoded_evt   The pointer to the decoded event.
 */
static void connected_evt_decode(const uint8_t * const           p_evt_data,
                                 ble_gap_evt_connected_t * const p_decoded_evt)
{
    uint8_t index = 0;

    index += ble_addr_decode(p_evt_data, &(p_decoded_evt->peer_addr));

    p_decoded_evt->irk_match      = (p_evt_data[index] & 0x01);
    p_decoded_evt->irk_match_idx  = (p_evt_data[index] & 0xFE) >> 1;
    index++;

    p_decoded_evt->conn_params.min_conn_interval = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    p_decoded_evt->conn_params.max_conn_interval = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    p_decoded_evt->conn_params.slave_latency     = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    p_decoded_evt->conn_params.conn_sup_timeout  = uint16_decode(&p_evt_data[index]);
}


/** @brief Function for decoding a disconnected event.
 *
 * @param[in]       p_evt_data      The pointer to the encoded event (excluding the event header).
 * @param[in, out]  p_decoded_evt   The pointer to the decoded event.
 */
static void disconnected_evt_decode(const uint8_t * const              p_evt_data,
                                    ble_gap_evt_disconnected_t * const p_decoded_evt)
{
    uint8_t index = 0;

    index += ble_addr_decode(p_evt_data, &(p_decoded_evt->peer_addr));

    p_decoded_evt->reason = p_evt_data[index];
}


/** @brief Function for decoding a GAP timeout event.
 *
 * @param[in]       p_evt_data      The pointer to the encoded event (excluding the event header).
 * @param[in, out]  p_decoded_evt   The pointer to the decoded event.
 */
static void gap_timeout_evt_decode(const uint8_t * const          p_evt_data,
                                   ble_gap_evt_timeout_t * const  p_decoded_evt)
{
    mp_ble_evt->evt.gap_evt.params.timeout.src = p_evt_data[0];
}


/** @brief Function for decoding a GAP connection parameter update event.
 *
 * @param[in]       p_evt_data      The pointer to the encoded event (excluding the event header).
 * @param[in, out]  p_decoded_evt   The pointer to the decoded event.
 */
static void gap_conn_param_update_evt_decode(const uint8_t * const                    p_evt_data,
                                             ble_gap_evt_conn_param_update_t * const  p_decoded_evt)
{
    uint8_t index = 0;

    p_decoded_evt->conn_params.min_conn_interval = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    p_decoded_evt->conn_params.max_conn_interval = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    p_decoded_evt->conn_params.slave_latency     = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    p_decoded_evt->conn_params.conn_sup_timeout  = uint16_decode(&p_evt_data[index]);
}


/** @brief Function for decoding a GAP security parameters request event.
 *
 * @param[in]       p_evt_data      The pointer to the encoded event (excluding the event header).
 * @param[in, out]  p_decoded_evt   The pointer to the decoded event.
 */
static void gap_sec_params_request_evt_decode(const uint8_t * const            p_evt_data,
                                      ble_gap_evt_sec_params_request_t * const p_decoded_evt)
{
    uint8_t index = 0;

    // (oob) << 5 | (io_caps) << 2 | (mitm) << 1 | (bond)
    p_decoded_evt->peer_params.timeout = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    p_decoded_evt->peer_params.bond    = (p_evt_data[index] >> 0) & 0x1;
    p_decoded_evt->peer_params.mitm    = (p_evt_data[index] >> 1) & 0x1;
    p_decoded_evt->peer_params.io_caps = (p_evt_data[index] >> 2) & 0x7;
    p_decoded_evt->peer_params.oob     = (p_evt_data[index] >> 5) & 0x1;
    index += sizeof(uint8_t);

    p_decoded_evt->peer_params.min_key_size        = p_evt_data[index++];
    p_decoded_evt->peer_params.max_key_size        = p_evt_data[index++];


}

/** @brief Function for decoding a GAP authentication status request event.
 *
 * @param[in]       p_evt_data      The pointer to the encoded event (excluding the event header).
 * @param[in, out]  p_decoded_evt   The pointer to the decoded event.
 */
static void gap_auth_status_evt_decode(const uint8_t * const             p_evt_data,
                                      ble_gap_evt_auth_status_t * const  p_decoded_evt)
{
    uint8_t index = 0;

    p_decoded_evt->auth_status = p_evt_data[index++];
    p_decoded_evt->error_src = p_evt_data[index++];

    p_decoded_evt->sm1_levels.lv3 = (p_evt_data[index] >> 5) & 0x01;
    p_decoded_evt->sm1_levels.lv2 = (p_evt_data[index] >> 4) & 0x01;
    p_decoded_evt->sm1_levels.lv1 = (p_evt_data[index] >> 3) & 0x01;
    p_decoded_evt->sm2_levels.lv3 = (p_evt_data[index] >> 2) & 0x01;
    p_decoded_evt->sm2_levels.lv2 = (p_evt_data[index] >> 1) & 0x01;
    p_decoded_evt->sm2_levels.lv1 = (p_evt_data[index] >> 0) & 0x01;
    index += sizeof(uint8_t);    
  
    p_decoded_evt->periph_kex.csrk      = (p_evt_data[index] >> 4) & 0x01;
    p_decoded_evt->periph_kex.address   = (p_evt_data[index] >> 3) & 0x01;
    p_decoded_evt->periph_kex.irk       = (p_evt_data[index] >> 2) & 0x01;
    p_decoded_evt->periph_kex.ediv_rand = (p_evt_data[index] >> 1) & 0x01;
    p_decoded_evt->periph_kex.ltk       = (p_evt_data[index] >> 0) & 0x01;
    index += sizeof(uint8_t);  
    
    p_decoded_evt->central_kex.ltk        = (p_evt_data[index] >> 4) & 0x01;
    p_decoded_evt->central_kex.ediv_rand  = (p_evt_data[index] >> 3) & 0x01;
    p_decoded_evt->central_kex.irk        = (p_evt_data[index] >> 2) & 0x01;
    p_decoded_evt->central_kex.address    = (p_evt_data[index] >> 1) & 0x01;
    p_decoded_evt->central_kex.csrk       = (p_evt_data[index] >> 0) & 0x01;
    index += sizeof(uint8_t);  
    
    p_decoded_evt->periph_keys.enc_info.div = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);
    
    uint32_t i = 0;
    for (i = 0; i < BLE_GAP_SEC_KEY_LEN; i++)
    {
        p_decoded_evt->periph_keys.enc_info.ltk[i] = p_evt_data[index++];
    }
    
    p_decoded_evt->periph_keys.enc_info.ltk_len = (p_evt_data[index] >> 1);
    p_decoded_evt->periph_keys.enc_info.auth = (p_evt_data[index] >> 0) & 0x01;
    index += sizeof(uint8_t); 
    
    for (i = 0; i < BLE_GAP_SEC_KEY_LEN; i++)
    {
        p_decoded_evt->central_keys.irk.irk[i] = p_evt_data[index++];
    }
    
    p_decoded_evt->central_keys.id_info.addr_type = p_evt_data[index++];

    for (i = 0; i < BLE_GAP_ADDR_LEN; i++)
    {
        p_decoded_evt->central_keys.id_info.addr[i] = p_evt_data[index++];
    }
    
}


/** @brief Function for decoding a GAP security info request event.
 *
 * @param[in]       p_evt_data      The pointer to the encoded event (excluding the event header).
 * @param[in, out]  p_decoded_evt   The pointer to the decoded event.
 */
static void gap_sec_info_request_evt_decode(const uint8_t * const          p_evt_data,
                                    ble_gap_evt_sec_info_request_t * const p_decoded_evt)
{
    uint8_t index = 0;

    p_decoded_evt->peer_addr.addr_type = p_evt_data[index++];
    p_decoded_evt->peer_addr.addr[0]   = p_evt_data[index++];
    p_decoded_evt->peer_addr.addr[1]   = p_evt_data[index++];
    p_decoded_evt->peer_addr.addr[2]   = p_evt_data[index++];
    p_decoded_evt->peer_addr.addr[3]   = p_evt_data[index++];
    p_decoded_evt->peer_addr.addr[4]   = p_evt_data[index++];
    p_decoded_evt->peer_addr.addr[5]   = p_evt_data[index++];

    p_decoded_evt->div = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    // (enc_info) << 2 | (id_info) << 1 | (sign_info) << 0
    p_decoded_evt->enc_info            = (p_evt_data[index] >> 0) & 0x1;
    p_decoded_evt->id_info             = (p_evt_data[index] >> 1) & 0x1;
    p_decoded_evt->sign_info           = (p_evt_data[index] >> 2) & 0x1;
}


/** @brief Function for decoding a GAP connection security updated event.
 *
 * @param[in]       p_evt_data      The pointer to the encoded event (excluding the event header).
 * @param[in, out]  p_decoded_evt   The pointer to the decoded event.
 */
static void gap_conn_sec_update_evt_decode(const uint8_t * const                 p_evt_data,
                                           ble_gap_evt_conn_sec_update_t * const p_decoded_evt)
{
    uint8_t index = 0;

    p_decoded_evt->conn_sec.sec_mode.sm = p_evt_data[index] & 0x0F;
    p_decoded_evt->conn_sec.sec_mode.lv = (p_evt_data[index] >> 4) & 0x0F;
    index++;

    p_decoded_evt->conn_sec.encr_key_size = p_evt_data[index++];
}



/** @brief Function for decoding a GATTS write event.
 *
 * @param[in]       p_evt_data      The pointer to the encoded event (excluding the event header).
 * @param[in, out]  p_decoded_evt   The pointer to the decoded event.
 */
static void gatts_write_evt_decode(const uint8_t * const         p_evt_data,
                                   ble_gatts_evt_write_t * const p_decoded_evt)
{
    uint16_t data_index;
    uint8_t  index = 0;

    p_decoded_evt->handle = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    p_decoded_evt->op     = p_evt_data[index++];


    p_decoded_evt->context.srvc_uuid.uuid = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);
    p_decoded_evt->context.srvc_uuid.type = p_evt_data[index++];

    p_decoded_evt->context.char_uuid.uuid = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);
    p_decoded_evt->context.char_uuid.type = p_evt_data[index++];

    p_decoded_evt->context.desc_uuid.uuid = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);
    p_decoded_evt->context.desc_uuid.type = p_evt_data[index++];

    p_decoded_evt->context.srvc_handle = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    p_decoded_evt->context.value_handle = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    p_decoded_evt->context.type = p_evt_data[index++];

    p_decoded_evt->offset = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    p_decoded_evt->len = uint16_decode(&p_evt_data[index]);
    index += sizeof(uint16_t);

    for (data_index = 0; data_index < p_decoded_evt->len; data_index++)
    {
        p_decoded_evt->data[data_index] = p_evt_data[index + data_index];
    }

}


/** @brief Function for decoding the event header from an encoded event.
 *
 * @param[in]       p_evt       The pointer to the encoded event.
 * @param[in, out]  p_evt_hdr   The pointer to the decoded event header.
 *
 * @return          Number of bytes decoded.
 */
static uint8_t evt_header_decode(const uint8_t * const p_evt, ble_evt_hdr_t * const p_evt_hdr)
{
    uint8_t index = 0;

    p_evt_hdr->evt_id = uint16_decode(&(p_evt[index]));
    index += sizeof(uint16_t);

    return index;
}


/** @brief Function for handling an encoded event.
 *
 * @param[in] p_event   The pointer to the encoded event.
 * @param[in] evt_len   Length of the received event, in bytes.
 */
static void event_handle(const uint8_t * const p_event, const uint16_t evt_len)
{
    uint8_t index = 0;

    // TODO: Use memory pool.
    // Memset of evt_len is not enough as incomming data is not struct padded.
    // Using MAX_EVENT_LEN to solve this.
    memset(m_event_buffer, 0, MAX_EVENT_LEN);

    /** @todo Check if evt_len is greater than sizeof(ble_evt_hdr_t) before decoding header. */
    /** @todo Check if evt_len is big enough for each event. */

    uint8_t event_id = p_event[0];

    //lint -e526 -e628 -e516 -save // Symbol '__INTADDR__()' not defined
                                   // no argument information provided for function '__INTADDR__()'
                                   // Symbol '__INTADDR__()' has arg. type conflict
    switch(event_id)
    {
        // GAP events
        case BLE_GAP_EVT_CONNECTED:
            index += evt_header_decode(p_event, &(mp_ble_evt->header));

            mp_ble_evt->evt.gap_evt.conn_handle = uint16_decode(&(p_event[index]));
            index += sizeof(mp_ble_evt->evt.gap_evt.conn_handle);

            connected_evt_decode(&(p_event[index]), &(mp_ble_evt->evt.gap_evt.params.connected));

            mp_ble_evt->header.evt_len  = (uint16_t)(offsetof(ble_evt_t,
                                                              evt.gap_evt.params.connected));
            mp_ble_evt->header.evt_len += sizeof(ble_gap_evt_connected_t);
            mp_ble_evt->header.evt_len -= sizeof(ble_evt_hdr_t);

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            index += evt_header_decode(p_event, &(mp_ble_evt->header));

            mp_ble_evt->evt.gap_evt.conn_handle = uint16_decode(&(p_event[index]));
            index += sizeof(mp_ble_evt->evt.gap_evt.conn_handle);

            disconnected_evt_decode(&(p_event[index]),
                                    &(mp_ble_evt->evt.gap_evt.params.disconnected));

            mp_ble_evt->header.evt_len  = (uint16_t)(offsetof(ble_evt_t,
                                                              evt.gap_evt.params.disconnected));
            mp_ble_evt->header.evt_len += sizeof(ble_gap_evt_disconnected_t);
            mp_ble_evt->header.evt_len -= sizeof(ble_evt_hdr_t);

            break;

        case BLE_GAP_EVT_TIMEOUT:
            index += evt_header_decode(p_event, &(mp_ble_evt->header));

            mp_ble_evt->evt.gap_evt.conn_handle = uint16_decode(&(p_event[index]));
            index += sizeof(mp_ble_evt->evt.gap_evt.conn_handle);

            gap_timeout_evt_decode(&(p_event[index]), &(mp_ble_evt->evt.gap_evt.params.timeout));

            mp_ble_evt->header.evt_len  = (uint16_t)(offsetof(ble_evt_t,
                                                              evt.gap_evt.params.timeout));
            mp_ble_evt->header.evt_len += sizeof(ble_gap_evt_timeout_t);
            mp_ble_evt->header.evt_len -= sizeof(ble_evt_hdr_t);

            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            index += evt_header_decode(p_event, &(mp_ble_evt->header));

            mp_ble_evt->evt.gap_evt.conn_handle = uint16_decode(&(p_event[index]));
            index += sizeof(mp_ble_evt->evt.gap_evt.conn_handle);

            gap_conn_param_update_evt_decode(&(p_event[index]),
                                             &(mp_ble_evt->evt.gap_evt.params.conn_param_update));

            mp_ble_evt->header.evt_len  = (uint16_t)(offsetof(ble_evt_t,
                                                              evt.gap_evt.params.conn_param_update));
            mp_ble_evt->header.evt_len += sizeof(ble_gap_evt_conn_param_update_t);
            mp_ble_evt->header.evt_len -= sizeof(ble_evt_hdr_t);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            index += evt_header_decode(p_event, &(mp_ble_evt->header));

            mp_ble_evt->evt.gap_evt.conn_handle = uint16_decode(&(p_event[index]));
            index += sizeof(mp_ble_evt->evt.gap_evt.conn_handle);

            gap_sec_params_request_evt_decode(&(p_event[index]),
                                              &(mp_ble_evt->evt.gap_evt.params.sec_params_request));

            mp_ble_evt->header.evt_len  = (uint16_t)(offsetof(ble_evt_t,
                                                              evt.gap_evt.params.sec_params_request));
            mp_ble_evt->header.evt_len += sizeof(ble_gap_evt_sec_params_request_t);
            mp_ble_evt->header.evt_len -= sizeof(ble_evt_hdr_t);

            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            index += evt_header_decode(p_event, &(mp_ble_evt->header));

            mp_ble_evt->evt.gap_evt.conn_handle = uint16_decode(&(p_event[index]));
            index += sizeof(mp_ble_evt->evt.gap_evt.conn_handle);

            gap_sec_info_request_evt_decode(&(p_event[index]),
                                            &(mp_ble_evt->evt.gap_evt.params.sec_info_request));

            mp_ble_evt->header.evt_len  = (uint16_t)(offsetof(ble_evt_t,
                                                              evt.gap_evt.params.sec_info_request));
            mp_ble_evt->header.evt_len += sizeof(ble_gap_evt_sec_info_request_t);
            mp_ble_evt->header.evt_len -= sizeof(ble_evt_hdr_t);

            break;

        case BLE_GAP_EVT_CONN_SEC_UPDATE:
            index += evt_header_decode(p_event, &(mp_ble_evt->header));

            mp_ble_evt->evt.gap_evt.conn_handle = uint16_decode(&(p_event[index]));
            index += sizeof(mp_ble_evt->evt.gap_evt.conn_handle);

            gap_conn_sec_update_evt_decode(&(p_event[index]),
                                           &(mp_ble_evt->evt.gap_evt.params.conn_sec_update));

            mp_ble_evt->header.evt_len  = (uint16_t)(offsetof(ble_evt_t,
                                                              evt.gap_evt.params.conn_sec_update));
            mp_ble_evt->header.evt_len += sizeof(ble_gap_evt_conn_sec_update_t);
            mp_ble_evt->header.evt_len -= sizeof(ble_evt_hdr_t);

            break;

        
        case BLE_GAP_EVT_AUTH_STATUS:
            index += evt_header_decode(p_event, &(mp_ble_evt->header));
            
            mp_ble_evt->evt.gap_evt.conn_handle = uint16_decode(&(p_event[index]));
            index += sizeof(mp_ble_evt->evt.gap_evt.conn_handle);
            
            gap_auth_status_evt_decode(&(p_event[index]),
                                       &(mp_ble_evt->evt.gap_evt.params.auth_status));

            mp_ble_evt->header.evt_len  = (uint16_t)(offsetof(ble_evt_t,
                                                              evt.gap_evt.params.auth_status));
            mp_ble_evt->header.evt_len += sizeof(ble_gap_evt_auth_status_t);
            mp_ble_evt->header.evt_len -= sizeof(ble_evt_hdr_t);
            break;

        // GATTS events
        case BLE_GATTS_EVT_WRITE:
            index += evt_header_decode(p_event, &(mp_ble_evt->header));

            mp_ble_evt->evt.gatts_evt.conn_handle = uint16_decode(&(p_event[index]));
            index += sizeof(mp_ble_evt->evt.gatts_evt.conn_handle);

            gatts_write_evt_decode(&(p_event[index]), &(mp_ble_evt->evt.gatts_evt.params.write));

            mp_ble_evt->header.evt_len  = (uint16_t)(offsetof(ble_evt_t,
                                                              evt.gatts_evt.params.write.data));

            // Additional data, One byte is already counted in the struct itself.
            mp_ble_evt->header.evt_len += mp_ble_evt->evt.gatts_evt.params.write.len;
            mp_ble_evt->header.evt_len -= sizeof(ble_evt_hdr_t);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            index += evt_header_decode(p_event, &(mp_ble_evt->header));

            mp_ble_evt->evt.gatts_evt.conn_handle = uint16_decode(&(p_event[index]));
            index += sizeof(mp_ble_evt->evt.gatts_evt.conn_handle);

            mp_ble_evt->evt.gatts_evt.params.sys_attr_missing.hint = p_event[index++];

            mp_ble_evt->header.evt_len  = (uint16_t)(offsetof(ble_evt_t,
                                                              evt.gatts_evt.params.sys_attr_missing));
            mp_ble_evt->header.evt_len += sizeof(ble_gatts_evt_sys_attr_missing_t);
            mp_ble_evt->header.evt_len -= sizeof(ble_evt_hdr_t);

            break;

        default:
            // Unhandled event received.
            return;
    }

    //lint -restore

    NVIC_SetPendingIRQ(SWI2_IRQn);
}


void rpc_evt_handler(rpc_transport_evt_type_t pkt_type)
{
    if (pkt_type != RPC_TRANSPORT_EVT_READY)
    {
        // Unwanted packet received. This should never happen.
        return;
    }

    uint8_t   event[MAX_EVENT_LEN];
    uint32_t  rcvd_evt_size = 0;
    uint32_t  err_code;

    err_code = rpc_transport_packet_read(RPC_TRANSPORT_EVT, sizeof(event), event, &rcvd_evt_size);

    if (err_code != NRF_SUCCESS)
    {
        // Nothing other than assertion can be done.
        ASSERT(false);
    }

    event_handle(event, rcvd_evt_size);
}


uint32_t sd_ble_evt_get(uint8_t * p_dest, uint16_t * p_len)
{
    if (p_len == NULL)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    // Check pointer alignment.
    if (!IS_ALIGNED((uint32_t) p_dest, BLE_EVTS_PTR_ALIGNMENT))
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    uint16_t ble_evt_len = mp_ble_evt->header.evt_len + sizeof(ble_evt_hdr_t);

    // Check if there is any event received.
    if (mp_ble_evt->header.evt_len == 0)
    {
        // No event received.
        *p_len = 0;
        return NRF_ERROR_NOT_FOUND;
    }

    // Check that the provided buffer is large enough.
    if (*p_len < ble_evt_len)
    {
        // Not enough memory provided to fit the event.
        *p_len = 0;
        return NRF_ERROR_DATA_SIZE;
    }

    // Set the calculated length of the event as output.
    *p_len = ble_evt_len;

    if (p_dest != NULL)
    {
        memcpy(p_dest, mp_ble_evt, ble_evt_len);

        // Clear the event length to invalidate the sm_ble_evt.
        mp_ble_evt->header.evt_len = 0;
    }

    return NRF_SUCCESS;
}


uint32_t ble_rpc_event_decoder_init(void)
{
    // The caller is interested in BLE events. Register with transport layer for events.
    return rpc_transport_register(RPC_TRANSPORT_EVT, rpc_evt_handler);
}
