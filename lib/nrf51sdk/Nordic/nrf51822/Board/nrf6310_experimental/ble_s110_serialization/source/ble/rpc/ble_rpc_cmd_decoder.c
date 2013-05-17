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

#include "ble_rpc_cmd_decoder.h"
#include "rpc_transport.h"
#include "ble_rpc_op_codes.h"
#include "ble_rpc_cmd_defines.h"
#include "nrf_error.h"
#include "app_util.h"
#include "ble.h"
#include "ble_gap.h"
#include "nrf_soc.h"
#include "ble_gatts.h"
#include <stdio.h>
#include <string.h>


#define RPC_BLE_FIELD_PRESENT                       0x01                              /**< Value to indicate that an optional field is encoded in the serialized packet, e.g. white list. */
#define RPC_BLE_FIELD_NOT_PRESENT                   0x00                              /**< Value to indicate that an optional field is not encoded in the serialized packet. */

#define RPC_TRANSPORT_PACKET_READ_BUF_SIZE          256u                              /**< Size of the RPC transport packet read buffer size */


/**@brief Function to send command response to the application chip though the transport layer.
 *
 * @param[in] op_code   The op code of the command for which the response is sent.
 * @param[in] status    The status field to be encoded into the command response.
 * @return NRF_SUCCESS on successful write of command response, otherwise an error code.
 */
static uint32_t command_resp_send(const uint8_t op_code, const uint32_t status)
{
    uint8_t buffer[RPC_MAX_CMD_RSP_LEN];

    // Encode Op Code.
    buffer[RPC_CMD_RESP_OP_CODE_POS] = op_code;

    // Encode Status.
    (void)uint32_encode(status, &(buffer[RPC_CMD_RESP_STATUS_POS]));

    return rpc_transport_packet_write(RPC_TRANSPORT_RESP, buffer, RPC_MAX_CMD_RSP_LEN);
}


static uint32_t command_resp_send_with_data(const uint8_t op_code,
                                            const uint8_t status,
                                            const uint8_t * const p_data,
                                            uint16_t data_len)
{
    uint32_t index = 0;
    uint8_t buffer[RPC_MAX_CMD_RSP_DATA_LEN];

    // Encode Op Code.
    buffer[index++] = op_code;

    // Encode Status.
    index += uint32_encode(status, &(buffer[index]));

    // Additional data in response packet.
    memcpy(&buffer[index], p_data, data_len);

    return rpc_transport_packet_write(RPC_TRANSPORT_RESP, buffer, RPC_MAX_CMD_RSP_LEN + data_len);
}


/**@brief Function to decode ble_gap_conn_sec_mode_t from input buffer.
 *
 * @param[in, out]  p_conn_sec_mode The pointer to the decoded structure. This should be provided by
 *                                  the caller.
 * @param[in]       p_command       The input command.
 * @return          Number of bytes decoded.
 */
static uint8_t conn_sec_mode_decode(ble_gap_conn_sec_mode_t * const p_conn_sec_mode,
                                    const uint8_t * const           p_command)
{
    p_conn_sec_mode->sm = (p_command[0] & 0x0f);
    p_conn_sec_mode->lv = (p_command[0] & 0xf0) >> 4;

    return 1;
}


/**@brief Function to decode ble_gap_addr_t from input buffer.
 *
 * @param[in, out]  p_addr       The pointer to the decoded structure. This should be provided by
 *                               the caller.
 * @param[in]       p_buffer     The buffer containing the encoded ble_gap_addr_t.
 * @return          Number of bytes decoded.
 */
static uint8_t gap_addr_decode(ble_gap_addr_t * const p_addr, const uint8_t * const p_buffer)
{
    uint8_t index = 0;

    p_addr->addr_type = p_buffer[index++];
    memcpy(p_addr->addr, &(p_buffer[index]), BLE_GAP_ADDR_LEN);

    index += BLE_GAP_ADDR_LEN;

    return index;
}


/**@brief Function to decode ble_gap_whitelist_t from input buffer.
 *
 * @param[in, out]  p_wl         The pointer to the decoded structure. This should be provided by
 *                               the caller.
 * @param[in]       p_buffer     The buffer containing the encoded ble_gap_addr_t.
 * @return          Number of bytes decoded.
 */
static uint8_t gap_wl_decode(ble_gap_whitelist_t * const p_wl, const uint8_t * const p_buffer)
{
    uint8_t                 index = 0;
    uint8_t                 i;
    static ble_gap_addr_t * p_addresses[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    static ble_gap_addr_t   addresses[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    static ble_gap_irk_t  * p_irks[BLE_GAP_WHITELIST_IRK_MAX_COUNT];
    static ble_gap_irk_t    irks[BLE_GAP_WHITELIST_IRK_MAX_COUNT];

    p_wl->addr_count = p_buffer[index++];

    for (i = 0; i < p_wl->addr_count; i++)
    {
        index           += gap_addr_decode(&(addresses[i]), &(p_buffer[index]));
        p_addresses[i]  = &(addresses[i]);
    }

    p_wl->irk_count = p_buffer[index++];

    for (i = 0; i < p_wl->irk_count; i++)
    {
        // Reuse the memory used by the buffer because irk is also a byte array like the buffer.
        memcpy(irks[i].irk, &(p_buffer[index]), BLE_GAP_SEC_KEY_LEN);
        index       += BLE_GAP_SEC_KEY_LEN;
        p_irks[i]   = &irks[i];
    }

    p_wl->pp_addrs = (p_wl->addr_count != 0) ? p_addresses : NULL;
    p_wl->pp_irks  = (p_wl->irk_count != 0) ? p_irks : NULL;

    return index;
}


/**@brief Function to handle RPC_SD_BLE_GAP_DEVICE_NAME_GET command.
 *
 * This function will decode the command, call the BLE Stack API, and also send command response
 * to the peer through the transport layer. In the response packet there will also be additional
 * data with encoded UUID and encoded UUID length.
 *
 * @param  p_command     The encoded structure that needs to be decoded and passed on to the
 *                       BLE Stack API.
 * @return NRF_SUCCESS if the decoding of the command was successful, the SoftDevice API was
 *                      called, and the command response was sent to peer, otherwise an error code.
 */
static uint32_t uuid_encode_handle(uint8_t * p_command)
{
    uint8_t      response_buffer[sizeof(ble_uuid128_t) + sizeof(uint8_t)]; // uuid can be up to 16 bytes, and 1 byte length field
    ble_uuid_t   uuid_data;

    ble_uuid_t * p_uuid_data     = &uuid_data;
    uint8_t *    p_length        = &(response_buffer[0]);
    uint8_t *    p_result_buffer = &(response_buffer[1]);

    uint32_t     err_code;
    uint8_t      index           = 0;

    // UUID field present.
    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {
        uuid_data.uuid = uint16_decode(&(p_command[index]));
        index         += sizeof(uint16_t);
        uuid_data.type = p_command[index++];
    }
    else
    {
        p_uuid_data = NULL;
    }

    // Length field not present.
    if (p_command[index++] == RPC_BLE_FIELD_NOT_PRESENT)
    {
        p_length = NULL;
    }

    // Result buffer not present.
    if (p_command[index++] == RPC_BLE_FIELD_NOT_PRESENT)
    {
        p_result_buffer = NULL;
    }

    err_code = sd_ble_uuid_encode(p_uuid_data, p_length, p_result_buffer);
    if (err_code == NRF_SUCCESS)
    {
        return command_resp_send_with_data(RPC_SD_BLE_UUID_ENCODE, NRF_SUCCESS, response_buffer, response_buffer[0] + sizeof(uint8_t));
    }

    return command_resp_send(RPC_SD_BLE_UUID_ENCODE, err_code);
}


/**@brief Function to handle RPC_SD_BLE_GAP_DEVICE_NAME_SET command.
 *
 * This function will decode the command, call the BLE Stack API, and also send command response
 * to the peer through the transport layer.
 *
 * @param  p_command     The encoded structure that needs to be decoded and passed on to the
 *                       BLE Stack API.
 * @return NRF_SUCCESS if the decoding of the command was successful, the SoftDevice API was
 *                      called, and the command response was sent to peer, otherwise an error code.
 */
static uint32_t device_name_set_handle(uint8_t * p_command)
{
    ble_gap_conn_sec_mode_t dev_name_write_perm;
    uint16_t                dev_name_len;
    uint8_t *               device_name;
    uint8_t                 index = 0;

    index       += conn_sec_mode_decode(&dev_name_write_perm, &(p_command[0]));

    dev_name_len = uint16_decode(&(p_command[index]));
    index       += sizeof(uint16_t);

    device_name = &(p_command[index++]);

    uint32_t err_code = sd_ble_gap_device_name_set(&dev_name_write_perm, device_name, dev_name_len);

    return command_resp_send(RPC_SD_BLE_GAP_DEVICE_NAME_SET, err_code);
}


/**@brief Function to handle RPC_SD_BLE_GAP_DEVICE_NAME_GET command.
 *
 * This function will decode the command, call the BLE Stack API, and also send command response
 * to the peer through the transport layer. In the response packet there will also be additional
 * data with device name and device name length.
 *
 * @param  p_command     The encoded structure that needs to be decoded and passed on to the
 *                       BLE Stack API.
 * @return NRF_SUCCESS if the decoding of the command was successful, the SoftDevice API was
 *                      called, and the command response was sent to peer, otherwise an error code.
 */
static uint32_t device_name_get_handle(uint8_t * p_command)
{
    uint16_t max_dev_name_len  = 32;                                       // Local buffer size for storing device name data.
    uint8_t  response_buffer[max_dev_name_len + sizeof(max_dev_name_len)]; // Two bytes will be reserved for the length of dev name.

    uint32_t err_code;
    uint32_t index             = 0;
    uint16_t dev_name_len      = 0;                                        // Placeholder for the device name size.

    uint16_t * p_dev_name_len  = &(dev_name_len);                          // Pointer to the device name size variable.
    uint8_t *  p_dev_name      = &(response_buffer[2]);                    // Pointer to the device name target.


    /* If length present. */
    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {
        // Use the remote buffer size when calling the SoftDevice to make sure the caller has
        // enough memory allocated.
        *p_dev_name_len  = uint16_decode(&p_command[index]);
        index           += sizeof(uint16_t);

        // Verify that the remote buffer size is smaller or equal to the local buffer size.
        if (*p_dev_name_len > max_dev_name_len)
        {
            return command_resp_send(RPC_SD_BLE_GAP_DEVICE_NAME_GET, NRF_ERROR_DATA_SIZE);
        }
    }
    else
    {
        p_dev_name_len = NULL;
    }

    /* If remote result buffer is present */
    if (p_command[index++] == RPC_BLE_FIELD_NOT_PRESENT)
    {
        p_dev_name = NULL;
    }

    err_code = sd_ble_gap_device_name_get(p_dev_name, p_dev_name_len);

    if (err_code == NRF_SUCCESS)
    {

        // Encode the length to be sent over transport layer.
        (void)uint16_encode(*p_dev_name_len, &response_buffer[0]);

        // Calculate the total size of the device name and the device name length bytes.
        uint16_t total_len  = uint16_decode(&response_buffer[0]);
        total_len          += sizeof(total_len);

        return command_resp_send_with_data(RPC_SD_BLE_GAP_DEVICE_NAME_GET, NRF_SUCCESS, response_buffer, total_len);
    }

    return command_resp_send(RPC_SD_BLE_GAP_DEVICE_NAME_GET, err_code);
}


/**@brief Function to handle RPC_SD_BLE_GAP_APPEARANCE_SET command.
 *
 * This function will decode the command, call the BLE Stack API, and also send command response
 * to the peer through the the transport layer.
 *
 * @param p_command     The encoded structure that needs to be decoded and passed on to the
 *                      BLE Stack API.
 * @return NRF_SUCCESS if the decoding of the command was successful, the SoftDevice API was
 *                      called, and the command response was sent to peer, otherwise an error code.
 */
static uint32_t appearance_set_handle(uint8_t * p_command)
{
    uint16_t appearance = uint16_decode(&(p_command[0]));

    uint32_t err_code   = sd_ble_gap_appearance_set(appearance);

    return command_resp_send(RPC_SD_BLE_GAP_APPEARANCE_SET , err_code);
}
 

/** @brief Function to handle RPC_SD_BLE_GAP_APPEARANCE_GET command.
 * This function will decode the command, call the BLE Stack API, and also send command response
 * to the peer through the the transport layer.
 * @param p_command     The encoded structure that needs to be decoded and passed on to the
 *                      BLE Stack API.
 * @return NRF_SUCCESS If the decoding of the command was successful, the SoftDevice API was
 *                      called, and the command response was sent to peer, otherwise an error code.
 */
static uint32_t appearance_get_handle(uint8_t const * const p_command)
{
    uint32_t        err_code;
    uint16_t        index = 0;
    uint8_t         out_index = 0;
    uint8_t         resp_data[sizeof(uint16_t)];
    uint16_t        appearance;
    uint16_t * p_appearance = &appearance;
    
    /* If appearance present. */
    if (p_command[index++] == RPC_BLE_FIELD_NOT_PRESENT)
    {
        p_appearance = NULL;
    }
    
    err_code  = sd_ble_gap_appearance_get(p_appearance);
    
    if (err_code == NRF_SUCCESS)
    {
        out_index += uint16_encode(*p_appearance, resp_data);
        return command_resp_send_with_data(RPC_SD_BLE_GAP_APPEARANCE_GET, err_code, resp_data, out_index);
    }
    else
    {
        return command_resp_send(RPC_SD_BLE_GAP_APPEARANCE_GET, err_code);
    }       
}


/**@brief Function to handle RPC_SD_BLE_GAP_PPCP_SET command.
 *
 * This function will decode the command, call the BLE Stack API, and also send command response
 * to the peer through the the transport layer.
 *
 * @param p_command     The encoded structure that needs to be decoded and passed on to the
 *                      BLE Stack API.
 * @return NRF_SUCCESS if the decoding of the command was successful, the SoftDevice API was
 *                      called, and the command response was sent to peer, otherwise an error code.
 */
static uint32_t ppcp_set_handle(uint8_t * p_command)
{
    ble_gap_conn_params_t conn_params;
    uint8_t               index = 0;

    conn_params.min_conn_interval = uint16_decode(&p_command[index]);
    index += sizeof(uint16_t);

    conn_params.max_conn_interval = uint16_decode(&p_command[index]);
    index += sizeof(uint16_t);

    conn_params.slave_latency     = uint16_decode(&p_command[index]);
    index += sizeof(uint16_t);

    conn_params.conn_sup_timeout  = uint16_decode(&p_command[index]);

    uint32_t err_code = sd_ble_gap_ppcp_set(&conn_params);

    return command_resp_send(RPC_SD_BLE_GAP_PPCP_SET, err_code);
}


/**@brief Function to handle RPC_SD_BLE_GAP_PPCP_GET command.
 *
 * This function will decode the command, call the BLE Stack API, and also send command response
 * to the peer through the the transport layer.
 *
 * @param p_command     The encoded structure that needs to be decoded and passed on to the
 *                      BLE Stack API.
 * @return NRF_SUCCESS if the decoding of the command was successful, the SoftDevice API was
 *                      called, and the command response was sent to peer, otherwise an error code.
 */
static uint32_t ppcp_get_handle(uint8_t * p_command)
{
    // Structure to be used as output when calling the SoftDevice.
    ble_gap_conn_params_t conn_params;
    uint8_t buffer[sizeof(ble_gap_conn_params_t)];

    uint32_t err_code;
    uint8_t index = 0;
    uint8_t encode_index = 0;

    // Pointer to the connection parameters result buffer.
    ble_gap_conn_params_t * p_conn_params = &conn_params;

    /* If length present. */
    if (p_command[index++] == RPC_BLE_FIELD_NOT_PRESENT)
    {
        p_conn_params = NULL;
    }

    err_code = sd_ble_gap_ppcp_get(p_conn_params);

    if (err_code == NRF_SUCCESS)
    {

        encode_index += uint16_encode(p_conn_params->min_conn_interval, &buffer[encode_index]);
        encode_index += uint16_encode(p_conn_params->max_conn_interval, &buffer[encode_index]);
        encode_index += uint16_encode(p_conn_params->slave_latency, &buffer[encode_index]);
        encode_index += uint16_encode(p_conn_params->conn_sup_timeout, &buffer[encode_index]);

        return command_resp_send_with_data(RPC_SD_BLE_GAP_PPCP_GET, err_code, buffer, encode_index);
    }
    else
    {
        return command_resp_send(RPC_SD_BLE_GAP_PPCP_GET, err_code);
    }
}


/**@brief Function to handle RPC_SD_BLE_GAP_ADV_START command.
 *
 * This function will decode the command, call the BLE Stack API, and also send command response
 * to the peer through the the transport layer.
 *
 * @param p_command     The encoded structure that needs to be decoded and passed on to the
 *                      BLE Stack API.
 * @return NRF_SUCCESS if the decoding of the command was successful, the SoftDevice API was
 *                      called, and the command response was sent to peer, otherwise an error code.
 */
static uint32_t adv_start_handle(uint8_t * p_command)
{
    ble_gap_adv_params_t adv_params;
    ble_gap_addr_t       directed_peer_address;
    ble_gap_whitelist_t  white_list;
    uint8_t              index = 0;

    adv_params.type = p_command[index++];

    if (p_command[index++] == 0x01)
    {
        // Peer Address Present. Decode the peer address.
        index += gap_addr_decode(&directed_peer_address, &(p_command[index]));
        adv_params.p_peer_addr = &(directed_peer_address);
    }
    else
    {
        adv_params.p_peer_addr = NULL;
    }

    adv_params.fp = p_command[index++];

    if (p_command[index++] == 0x01)
    {
        // Whitelist present.
        index += gap_wl_decode(&white_list, &(p_command[index]));
        adv_params.p_whitelist = &white_list;
    }
    else
    {
        adv_params.p_whitelist = NULL;
    }

    adv_params.interval = uint16_decode(&p_command[index]);
    index              += sizeof(uint16_t);

    adv_params.timeout  = uint16_decode(&p_command[index]);

    uint32_t err_code   = sd_ble_gap_adv_start(&adv_params);

    return command_resp_send(RPC_SD_BLE_GAP_ADV_START, err_code);
}


/**@brief Function to handle RPC_SD_BLE_GAP_ADV_DATA_SET command.
 *
 * This function will decode the command, call the BLE Stack API, and also send command response
 * to the peer through the the transport layer.
 *
 * @param p_command     The encoded structure that needs to be decoded and passed on to the
 *                      BLE Stack API.
 * @return NRF_SUCCESS if the decoding of the command was successful, the SoftDevice API was
 *                      called, and the command response was sent to peer, otherwise an error code.
 */
static uint32_t adv_data_set_handle(const uint8_t * const p_command)
{
    const uint8_t * p_data;
    uint8_t         dlen;
    const uint8_t * p_sr_data;
    uint8_t         srdlen;
    int             index = 0;

    dlen        = p_command[index++];
    p_data      = &(p_command[index]);
    index      += dlen;
    srdlen      = p_command[index++];
    p_sr_data   = &(p_command[index]);

    uint32_t err_code = sd_ble_gap_adv_data_set(p_data, dlen, p_sr_data, srdlen);

    return command_resp_send(RPC_SD_BLE_GAP_ADV_DATA_SET, err_code);
}


static uint32_t gap_disconnect_handle(const uint8_t * const p_command)
{
    uint16_t    conn_handle;
    uint8_t     hci_status_code;
    uint32_t    err_code;
    uint32_t    index = 0;

    conn_handle      = uint16_decode(&p_command[index]);
    index           += sizeof(uint16_t);
    hci_status_code  = p_command[index++];

    err_code = sd_ble_gap_disconnect(conn_handle, hci_status_code);

    return command_resp_send(RPC_SD_BLE_GAP_DISCONNECT, err_code);
}


static uint32_t sec_params_decode(const uint8_t * const p_command, ble_gap_sec_params_t * p_params)
{
    uint32_t index = 0;
    uint8_t  bit_fields = 0;

    p_params->timeout       = uint16_decode(&p_command[index]);
    index                  += sizeof(uint16_t);

    bit_fields              = p_command[index++];
    p_params->bond          = (bit_fields >> 0) & 0x01;
    p_params->mitm          = (bit_fields >> 1) & 0x01;
    p_params->io_caps       = (bit_fields >> 2) & 0x07;
    p_params->oob           = (bit_fields >> 5) & 0x01;

    p_params->min_key_size  = p_command[index++];
    p_params->max_key_size  = p_command[index++];

    return index;
}


static uint32_t gap_sec_params_reply_handle(const uint8_t * const p_command)
{
    uint16_t                conn_handle;
    uint8_t                 sec_status;
    uint32_t                err_code;
    uint32_t                index = 0;
    ble_gap_sec_params_t    sec_params;

    conn_handle      = uint16_decode(&p_command[index]);
    index           += sizeof(uint16_t);
    sec_status       = p_command[index++];

    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {
        index   += sec_params_decode(&p_command[index], &sec_params);
        err_code = sd_ble_gap_sec_params_reply(conn_handle, sec_status, &sec_params);
    }
    else
    {
        err_code = sd_ble_gap_sec_params_reply(conn_handle, sec_status, NULL);
    }

    return command_resp_send(RPC_SD_BLE_GAP_SEC_PARAMS_REPLY, err_code);
}


static uint32_t gap_conn_param_update_handle(const uint8_t * const p_command)
{
    uint16_t                conn_handle;
    uint32_t                err_code;
    ble_gap_conn_params_t   conn_params;
    ble_gap_conn_params_t * p_conn_params = NULL;
    uint32_t                index = 0;

    conn_handle  = uint16_decode(&p_command[index]);
    index       += sizeof(uint16_t);

    // Check the Connection Parameters present field.
    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {
        conn_params.min_conn_interval = uint16_decode(&p_command[index]);
        index                        += sizeof(uint16_t);
        conn_params.max_conn_interval = uint16_decode(&p_command[index]);
        index                        += sizeof(uint16_t);
        conn_params.slave_latency     = uint16_decode(&p_command[index]);
        index                        += sizeof(uint16_t);
        conn_params.conn_sup_timeout  = uint16_decode(&p_command[index]);
        p_conn_params                 = &conn_params;
    }

    err_code =  sd_ble_gap_conn_param_update(conn_handle, p_conn_params);
    return command_resp_send(RPC_SD_BLE_GAP_CONN_PARAM_UPDATE, err_code);
}


static uint32_t gatts_service_add_handle(const uint8_t * const p_command)
{
    uint8_t     type;
    uint32_t    err_code;
    uint32_t    index = 0;
    uint16_t    handle;
    uint8_t     data[2];

    type = p_command[index++];

    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {
        ble_uuid_t uuid;

        uuid.uuid = uint16_decode(&p_command[index]);
        index    += sizeof(uint16_t);
        uuid.type = p_command[index++];

        err_code  = sd_ble_gatts_service_add(type, &uuid, &handle);
    }
    else
    {
        err_code = sd_ble_gatts_service_add(type, NULL, &handle);
    }

    if (err_code == NRF_SUCCESS)
    {
        (void) uint16_encode(handle, data);
        return command_resp_send_with_data(RPC_SD_BLE_GATTS_SERVICE_ADD, err_code, data, sizeof(data));
    }
    else
    {
        return command_resp_send(RPC_SD_BLE_GATTS_SERVICE_ADD, err_code);
    }
}


static uint32_t gatts_sys_attr_get_handle(uint8_t const * const p_command)
{
    uint32_t        err_code;
    uint32_t        index = 0;
    uint8_t         resp_data[512];

    uint16_t        conn_handle;
    uint16_t        attr_data_length;
    uint16_t *      p_attr_data_length = &attr_data_length;
    uint8_t *       p_attr_data = &resp_data[3];


    conn_handle = uint16_decode(&p_command[index]);
    index += sizeof(uint16_t);

    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {
        attr_data_length = uint16_decode(&p_command[index]);
        index           += sizeof(uint16_t);
    }
    else
    {
        p_attr_data_length = NULL;
    }

    if (p_command[index++] == RPC_BLE_FIELD_NOT_PRESENT)
    {
        p_attr_data = NULL;
    }

    err_code  = sd_ble_gatts_sys_attr_get(conn_handle, p_attr_data, p_attr_data_length);

    if (err_code == NRF_SUCCESS)
    {
        index = 0;
        index += uint16_encode(*p_attr_data_length, &resp_data[0]);

        if (p_attr_data == NULL)
        {
            resp_data[index++] = RPC_BLE_FIELD_NOT_PRESENT;
        }
        else
        {
            resp_data[index++]  = RPC_BLE_FIELD_PRESENT;
            index                  += *p_attr_data_length;
        }

        return command_resp_send_with_data(RPC_SD_BLE_GATTS_SYS_ATTR_GET, err_code, resp_data, index);
    }
    else
    {
        return command_resp_send(RPC_SD_BLE_GATTS_SYS_ATTR_GET, err_code);
    }

}


static uint32_t gatts_sys_attr_set_handle(uint8_t const * const p_command)
{
    uint16_t        conn_handle;
    uint16_t        attr_data_length;
    const uint8_t * p_attr_data;
    uint32_t        err_code;
    uint32_t        index = 0;

    conn_handle = uint16_decode(&p_command[index]);
    index += sizeof(uint16_t);

    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {
        attr_data_length = uint16_decode(&p_command[index]);
        index           += sizeof(uint16_t);
        p_attr_data      = &(p_command[index]);

        err_code  = sd_ble_gatts_sys_attr_set(conn_handle, p_attr_data, attr_data_length);
    }
    else
    {
        err_code  = sd_ble_gatts_sys_attr_set(conn_handle, NULL, 0);
    }
    return command_resp_send(RPC_SD_BLE_GATTS_SYS_ATTR_SET, err_code);
}


static uint32_t gatts_value_set_handle(uint8_t const * const p_command)
{
    uint16_t        handle;
    uint16_t        offset;
    uint16_t        length;
    uint16_t *      p_length = NULL;
    const uint8_t * p_value = NULL;

    uint32_t        err_code;
    uint32_t        index = 0;

    handle = uint16_decode(&p_command[index]);
    index += sizeof(uint16_t);

    offset = uint16_decode(&p_command[index]);
    index += sizeof(uint16_t);

    // Decode length value if Length field is present.
    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {
        length    = uint16_decode(&p_command[index]);
        p_length  = &length;
        index    += sizeof(uint16_t);
    }

    // Decode value date if Value Present field is 0x01.
    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {
        p_value = &p_command[index];
    }

    err_code  = sd_ble_gatts_value_set(handle, offset, p_length, p_value);

    if (err_code == NRF_SUCCESS)
    {
        uint8_t response_data[sizeof(uint16_t)];
        (void) uint16_encode(*p_length, response_data);

        return command_resp_send_with_data(RPC_SD_BLE_GATTS_VALUE_SET, err_code, response_data, sizeof(response_data));
    }
    else
    {
        return command_resp_send(RPC_SD_BLE_GATTS_VALUE_SET, err_code);
    }
}


static uint32_t gatts_hvx_handle(uint8_t  * const p_command)
{
    uint32_t                err_code;
    uint16_t                conn_handle;
    uint16_t                hvx_params_data_length;
    ble_gatts_hvx_params_t  hvx_params;
    uint32_t                index = 0;
    uint8_t                 data[2];

    conn_handle = uint16_decode(&p_command[index]);
    index += sizeof(uint16_t);

    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {
        hvx_params.handle       = uint16_decode(&p_command[index]);
        index                  += sizeof(uint16_t);
        hvx_params.type         = p_command[index++];
        hvx_params.offset       = uint16_decode(&p_command[index]);
        index                  += sizeof(uint16_t);

        if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
        {
            hvx_params_data_length  = uint16_decode(&p_command[index]);
            index                  += sizeof(uint16_t);
            hvx_params.p_len        = &hvx_params_data_length;
        }
        else
        {
            hvx_params.p_len = NULL;
        }

        if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
        {
            hvx_params.p_data = &(p_command[index]);
        }
        else
        {
            hvx_params.p_data = NULL;
        }

        err_code  = sd_ble_gatts_hvx(conn_handle, &hvx_params);

        if (err_code != NRF_SUCCESS)
        {
            return command_resp_send(RPC_SD_BLE_GATTS_HVX, err_code);
        }
        if (hvx_params.p_len != NULL)
        {
            (void) uint16_encode(*(hvx_params.p_len), data);

            return command_resp_send_with_data(RPC_SD_BLE_GATTS_HVX, err_code, data, sizeof(data));
        }

        return command_resp_send(RPC_SD_BLE_GATTS_HVX, err_code); 
    }
    else
    {
        err_code = sd_ble_gatts_hvx(conn_handle, NULL);
    }

    return command_resp_send(RPC_SD_BLE_GATTS_HVX, err_code);        
}


static uint32_t ble_gatts_attr_md_decode(uint8_t * p_buffer, ble_gatts_attr_md_t * p_attr_md)
{
	uint32_t index = 0;

	p_attr_md->read_perm.sm 	= ((p_buffer[index]   >> 0) & 0xF);
	p_attr_md->read_perm.lv 	= ((p_buffer[index++] >> 4) & 0xF);

	p_attr_md->write_perm.sm	= ((p_buffer[index]   >> 0) & 0xF);
	p_attr_md->write_perm.lv	= ((p_buffer[index++] >> 4) & 0xF);

	p_attr_md->vlen 			= ((p_buffer[index]   >> 0) & 0x1);
	p_attr_md->vloc 			= ((p_buffer[index]   >> 1) & 0x3);
	p_attr_md->rd_auth 			= ((p_buffer[index]   >> 3) & 0x1);
	p_attr_md->wr_auth 			= ((p_buffer[index++] >> 4) & 0x1);

	return index;
}


static uint32_t gatts_characteristic_add_handle(uint8_t * const p_command)
{
	uint32_t err_code;
	uint16_t service_handle;
	uint32_t index = 0;

	ble_gatts_char_md_t 		char_md;
	uint8_t 					char_md_char_user_desc;
	ble_gatts_attr_md_t 		char_md_user_desc_md;
	ble_gatts_attr_md_t 		char_md_cccd_md;
	ble_gatts_attr_md_t 		char_md_sccd_md;
	ble_gatts_char_pf_t 		char_md_char_pf;
	ble_gatts_char_md_t * 		p_char_md;

	ble_gatts_attr_t 			attr_char_value;
	ble_uuid_t 					attr_char_value_uuid;
	ble_gatts_attr_md_t 		attr_char_value_attr_md;
	uint8_t 					attr_char_value_value;
	ble_gatts_attr_t *			p_attr_char_value;

	ble_gatts_char_handles_t 	handles;
	ble_gatts_char_handles_t * 	p_handles;

	service_handle = uint16_decode(&p_command[index]);
	index += sizeof(uint16_t);

	if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
	{
		char_md.char_props.broadcast      	= ((p_command[index]   >> 0) & 1);
		char_md.char_props.read				= ((p_command[index]   >> 1) & 1);
		char_md.char_props.write_wo_resp	= ((p_command[index]   >> 2) & 1);
		char_md.char_props.write			= ((p_command[index]   >> 3) & 1);
		char_md.char_props.notify			= ((p_command[index]   >> 4) & 1);
		char_md.char_props.indicate			= ((p_command[index]   >> 5) & 1);
		char_md.char_props.auth_signed_wr	= ((p_command[index++] >> 6) & 1);

		char_md.char_ext_props.reliable_wr 	= ((p_command[index]   >> 0) & 1);
		char_md.char_ext_props.wr_aux      	= ((p_command[index++] >> 1) & 1);

		if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
		{
			char_md_char_user_desc		= p_command[index++];
			char_md.p_char_user_desc 	= &char_md_char_user_desc;
		}
		else
		{
			char_md.p_char_user_desc = NULL;
		}

		char_md.char_user_desc_max_size = uint16_decode(&p_command[index]);
		index += sizeof(uint16_t);
		char_md.char_user_desc_size 	= uint16_decode(&p_command[index]);
		index += sizeof(uint16_t);

		if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
		{
			char_md_char_pf.format 		= p_command[index++];
			char_md_char_pf.exponent 	= p_command[index++];
			char_md_char_pf.unit 		= uint16_decode(&p_command[index]);
			index += sizeof(uint16_t);
			char_md_char_pf.name_space 	= p_command[index++];
			char_md_char_pf.desc 		= uint16_decode(&p_command[index]);
			index += sizeof(uint16_t);
			char_md.p_char_pf 			= &char_md_char_pf;
		}
		else
		{
			char_md.p_char_pf = NULL;
		}

		if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
		{
			index += ble_gatts_attr_md_decode(&p_command[index], &char_md_user_desc_md);
			char_md.p_user_desc_md = &char_md_user_desc_md;
		}
		else
		{
			char_md.p_user_desc_md = NULL;
		}

		if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
		{
			index += ble_gatts_attr_md_decode(&p_command[index], &char_md_cccd_md);
			char_md.p_cccd_md = &char_md_cccd_md;
		}
		else
		{
			char_md.p_cccd_md = NULL;
		}

		if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
		{
			index += ble_gatts_attr_md_decode(&p_command[index], &char_md_sccd_md);
			char_md.p_sccd_md = &char_md_sccd_md;
		}
		else
		{
			char_md.p_sccd_md = NULL;
		}

		p_char_md = &char_md;
	}
	else
	{
		p_char_md = NULL;
	}
	if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
	{
		if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
		{
			attr_char_value_uuid.uuid = uint16_decode(&p_command[index]);
			index += sizeof(uint16_t);
			attr_char_value_uuid.type = p_command[index++];

			attr_char_value.p_uuid = &attr_char_value_uuid;
		}
		else
		{
			attr_char_value.p_uuid = NULL;
		}

		if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
		{
			index += ble_gatts_attr_md_decode(&p_command[index], &attr_char_value_attr_md);
			attr_char_value.p_attr_md = &attr_char_value_attr_md;
		}
		else
		{
			attr_char_value.p_attr_md = NULL;
		}
		attr_char_value.init_len	= uint16_decode(&p_command[index]);
		index += sizeof(uint16_t);
		attr_char_value.init_offs	= uint16_decode(&p_command[index]);
		index += sizeof(uint16_t);
		attr_char_value.max_len		= uint16_decode(&p_command[index]);
		index += sizeof(uint16_t);
		if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
		{
			attr_char_value_value		= p_command[index++];
			attr_char_value.p_value		= &attr_char_value_value;
		}
		else
		{
			attr_char_value.p_value = NULL;
		}

		p_attr_char_value = &attr_char_value;
	}

	else
	{
		p_attr_char_value = NULL;
	}

	if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
	{
		handles.value_handle 		= uint16_decode(&p_command[index]);
		index += sizeof(uint16_t);
		handles.user_desc_handle 	= uint16_decode(&p_command[index]);
		index += sizeof(uint16_t);
		handles.cccd_handle 		= uint16_decode(&p_command[index]);
		index += sizeof(uint16_t);
		handles.sccd_handle 		= uint16_decode(&p_command[index]);
		index += sizeof(uint16_t);
	}

	p_handles = &handles;

	err_code = sd_ble_gatts_characteristic_add(service_handle, p_char_md, p_attr_char_value, p_handles);

	if (p_handles != NULL)
	{
		index = 0;
		uint8_t data[sizeof(ble_gatts_char_handles_t)];
		index += uint16_encode(p_handles->value_handle, &data[index]);
		index += uint16_encode(p_handles->user_desc_handle, &data[index]);
		index += uint16_encode(p_handles->cccd_handle, &data[index]);
		index += uint16_encode(p_handles->sccd_handle, &data[index]);
		return command_resp_send_with_data(RPC_SD_BLE_GATTS_CHARACTERISTIC_ADD, err_code, data, sizeof(data));
	}

	return command_resp_send(RPC_SD_BLE_GATTS_CHARACTERISTIC_ADD, err_code);

}


/**@brief Function to handle RPC_SD_BLE_GAP_SEC_INFO_REPLY command.
 *
 * @param[in] p_command Begin of the input command after operation code field.
 * 
 * @return NRF_SUCCESS on successful write of command response, otherwise an error code.
 */
static uint32_t gap_sec_info_reply_handle(uint8_t * const p_command)
{
    uint32_t            err_code;
    bool                use_sign;
    
    //lint --e{645} "Symbol may not have been initialized"
    // @note Lint generates above warning for the 2 symbols below, this is understood and accepted 
    // side effect of the implementation in question.  
    ble_gap_enc_info_t  enc_info;
    ble_gap_sign_info_t sign_info;
    
    uint32_t       index       = 0;    
    const uint16_t conn_handle = uint16_decode(&p_command[index]);
    
    index += sizeof(uint16_t);    
    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {        
        enc_info.div    = uint16_decode(&p_command[index]);        
        index           += sizeof(uint16_t);            
        memcpy(enc_info.ltk, &p_command[index], sizeof(enc_info.ltk));
        index           += sizeof(enc_info.ltk);
        enc_info.auth    = (p_command[index] & 0x01);
        enc_info.ltk_len = ((p_command[index] & 0xfe) >> 1);           
        index += 1;
    }
    if (p_command[index++] == RPC_BLE_FIELD_PRESENT)
    {
        use_sign = true;    
        
        memcpy(sign_info.csrk, &p_command[index], sizeof(sign_info.csrk));        
    }
    else
    {
        use_sign = false;    
    }    
    
    err_code = sd_ble_gap_sec_info_reply(conn_handle, 
                                        (p_command[2] == RPC_BLE_FIELD_PRESENT) ? &enc_info : NULL, 
                                        (use_sign) ? &sign_info : NULL);
                                        
    return command_resp_send(RPC_SD_BLE_GAP_SEC_INFO_REPLY, err_code);
}


/**@brief Function to handle RPC_SD_POWER_SYSTEM_OFF command.
 *
 * This function will decode the command, call the SOC api and put the chip into system off
 * mode. It will always return error code NRF_SUCCESS.
 *
 * @return NRF_SUCCESS
 * 
 */
static uint32_t sd_power_system_off_handle(void)
{
    (void)sd_power_system_off();
    return NRF_SUCCESS;
}


void rpc_cmd_handle(void * p_event_data, uint16_t event_size)
{
    uint32_t err_code = NRF_SUCCESS;

    static uint8_t  rpc_cmd_buffer[RPC_TRANSPORT_PACKET_READ_BUF_SIZE];
    static uint32_t rpc_cmd_length_read;
    static uint32_t rpc_cmd_length = RPC_TRANSPORT_PACKET_READ_BUF_SIZE;

    err_code = rpc_transport_packet_read(RPC_TRANSPORT_CMD,
                                         rpc_cmd_length,
                                         rpc_cmd_buffer,
                                         &rpc_cmd_length_read);
    APP_ERROR_CHECK(err_code);

    err_code = command_process(rpc_cmd_buffer, rpc_cmd_length_read);
    APP_ERROR_CHECK(err_code);
}


uint32_t command_process(uint8_t * p_command, uint8_t command_len)
{
    uint32_t err_code;

    switch (p_command[RPC_CMD_OP_CODE_POS])
    {
        case RPC_SD_BLE_UUID_ENCODE:
            err_code = uuid_encode_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GAP_DEVICE_NAME_SET:
            err_code = device_name_set_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GAP_DEVICE_NAME_GET:
            err_code = device_name_get_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GAP_APPEARANCE_SET:
            err_code = appearance_set_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;
        
        case RPC_SD_BLE_GAP_APPEARANCE_GET:
            err_code = appearance_get_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GAP_PPCP_SET:
            err_code = ppcp_set_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GAP_PPCP_GET:
            err_code = ppcp_get_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GAP_ADV_START:
            err_code = adv_start_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GAP_ADV_DATA_SET:
            err_code = adv_data_set_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GAP_DISCONNECT:
            err_code = gap_disconnect_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GAP_SEC_PARAMS_REPLY:
            err_code = gap_sec_params_reply_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GAP_CONN_PARAM_UPDATE:
            err_code = gap_conn_param_update_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GATTS_SERVICE_ADD:
            err_code = gatts_service_add_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GATTS_SYS_ATTR_GET:
            err_code = gatts_sys_attr_get_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GATTS_SYS_ATTR_SET:
            err_code = gatts_sys_attr_set_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GATTS_VALUE_SET:
            err_code = gatts_value_set_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GATTS_HVX:
            err_code = gatts_hvx_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_BLE_GATTS_CHARACTERISTIC_ADD:
            err_code = gatts_characteristic_add_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;

        case RPC_SD_POWER_SYSTEM_OFF:
            err_code = sd_power_system_off_handle();
            break;

        case RPC_SD_BLE_GAP_SEC_INFO_REPLY:  
            err_code = gap_sec_info_reply_handle(&(p_command[RPC_CMD_DATA_POS]));
            break;
            
        default:
            err_code = command_resp_send(p_command[RPC_CMD_OP_CODE_POS], NRF_ERROR_NOT_SUPPORTED);
            break;
    }

    return err_code;
}
