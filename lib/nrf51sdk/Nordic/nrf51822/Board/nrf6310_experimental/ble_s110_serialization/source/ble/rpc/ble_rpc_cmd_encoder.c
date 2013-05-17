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
#include "nrf.h"
#include "nrf51.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "app_util.h"
#include "app_error.h"
#include "rpc_transport.h"
#include "ble_gap.h"
#include "ble_gatts.h"
#include "ble_rpc_op_codes.h"
#include "ble_rpc_cmd_encoder_internal.h"
#include "ble_rpc_cmd_encoder.h"

#define RPC_CMD_ENCODER_APPEARANCE_PACKET_SIZE          3                                 /**< Size of the Appearance Set Packet. */
#define RPC_CMD_ENCODER_PPCP_PACKET_SIZE                9                                 /**< Size of the PPCP Packet. */

#define RPC_CMD_ENCODER_ADV_START_PACKET_SIZE_MAX       202                               /**< Maximum length of the Advertising Start packet. */
#define RPC_CMD_ENCODER_DEVICE_NAME_PACKET_SIZE_MAX     (BLE_GAP_DEVNAME_MAX_LEN + 4)     /**< Maximum length of the Device Name Set packet. Length of Device Name + OpCode (1 byte), Security settings (1 byte), length encoding (2 byte). */
#define RPC_CMD_ENCODER_ADV_PACKET_SIZE_MAX             ((2 * BLE_GAP_ADV_MAX_SIZE) + 3)  /**< Maximum length of the Advertising Set packet. Length of Raw Data, Length of Scan Response Data, + OpCode (1 byte), Length encode raw data (1 byte), Length encoding Scan Response Data (1 byte).*/
#define RPC_CMD_ENCODER_SERVICE_ADD_PACKET_SIZE         9                                 /**< Maximum length of the Service Add packet. Command id (1 byte), Type (1 byte), ble_uuid_t (3 bytes), handle (2 bytes) and optional field markers (2 bytes). */ 
#define RPC_CMD_ENCODER_CHARACTERISTICS_ADD_PACKET_SIZE 74
#define RPC_CMD_ENCODER_SYSTEM_ATTR_SET_PACKET_SIZE     518                               /**< Maximum length of the System Attribute Set packet. Command id (1 byte), conn_handle (2 bytes), system_attr_data (512 byte), len (2 bytes) and optional field markers (1 byte). */ 
#define RPC_CMD_ENCODER_HVX_PACKET_SIZE                 512                               /**< Maximum length of the HVX packet. Command id (1 byte), conn_handle (2 bytes), hvx_params.handle (2 bytes), hvx_params.type (1 byte), hvx_params.offset (2 bytes), hvx_params.p_len (2 bytes), hvx_params.p_data (variable size) and optional field markers (3 bytes). */ 
#define RPC_CMD_ENCODER_DISCONNECT_PACKET_SIZE          4                                 /**< Maximum length of the Disconnect packet. Command id (1 byte), conn_handle (2 bytes) and hci status code (1 byte). */ 
#define RPC_CMD_ENCODER_SEC_PARAMS_PACKET_SIZE          10                                /**< Maximum length of the Sec Params Reply packet. Command id (1 byte), conn_handle (2 bytes), sec status (1 byte), timeout (2 byte), bit patterns (1 byte), min key size (1 byte), max key size (1 byte) and optional field markers (1 bytes). */ 
#define RPC_CMD_ENCODER_APPEARANCE_GET_PACKET_SIZE      2                                 /**< Maximum length of the Appearance Get Packet. Command id (1 byte). */ 
#define RPC_CMD_ENCODER_DEVICE_NAME_GET_PACKET_SIZE     5                                 /**< Maximum length of the Device Name Get Packet. Command id (1 byte), buffer size (2 bytes) and optional field markers (2 bytes). */ 
#define RPC_CMD_ENCODER_UUID_ENCODE_PACKET_SIZE         100                               /**< @todo trim my size down later: Maximum length of the Device Name Get Packet. Command id (1 byte), uuid value (2 bytes), uuid type (1 byte) and optional field markers (1 bytes). */ 
#define RPC_CMD_ENCODER_ADDRESS_SET_PACKET_SIZE         8                                 /**< Maximum length of the Address Set Packet. Command id (1 byte), addr_type (1 byte) and addr (6 bytes). */ 
#define RPC_CMD_ENCODER_ADDRESS_GET_PACKET_SIZE         8                                 /**< Maximum length of the Address Get Packet. Command id (1 byte), addr_type (1 byte) and addr (6 bytes). */ 
#define RPC_CMD_ENCODER_VALUE_SET_PACKET_SIZE           512                               /**< Maximum length of the Value Set packet. Command id (1 byte), attribute handle (2 bytes), offset (2 bytes), p_len (2 bytes), p_value (variable size) and optional field markers (2 bytes). */ 
#define RPC_CMD_ENCODER_CONN_PARAM_UPDATE_PACKET_SIZE   12                                /**< Maximum length of the Conn param Update packet. Command id (1 byte), connection handle (2 bytes), connection parameters (8 bytes) and optional field markers (1 bytes). */ 
#define RPC_CMD_ENCODER_SYS_ATTR_GET_PACKET_SIZE        7                                 /**< Maximum length of the Sys Attr Get packet. Command id (1 byte), connection handle (2 bytes), system attributes length (2 bytes) and optional field markers (2 bytes). */ 
#define RPC_CMD_ENCODER_PPCP_GET_PACKET_SIZE            2                                 /**< Maximum length of the PPCP Get Packet. Command id (1 byte) and optional field marker (1 byte). */ 
#define RPC_CMD_RESPONSE_PACKET_MIN_SIZE                5                                 /**< Minimum length of a command response. OpCode (uint8_t) + error code (uint32_t). */
#define RPC_CMD_RESPONSE_PACKET_MAX_SIZE                128                               /**< Current maximum length of a command response. */
#define RPC_CMD_ENCODER_SEC_INFO_REPLY_PACKET_SIZE      39                                /**< Maximum length of the gap_sec_info_reply command. Command id (1 byte), connection handle (2 bytes),  gap encoding info (19 bytes), gap sign info (17 bytes).*/

#define RPC_CMD_ENCODER_POWER_SYSTEM_OFF_PACKET_SIZE    1                                 /**< Maximum length of the Power System Off packet. Command id (1 byte). */ 
typedef struct
{
    rpc_ble_op_code_t op_code;                                                            /**< Operation code for which this response applies. */
    uint32_t          err_code;                                                           /**< Error code received for this response applies. */
} cmd_response_t;

static volatile cmd_response_t m_cmd_response;                                            /**< Response of last received command/response. */
static uint8_t                 m_cmd_response_buf[RPC_CMD_RESPONSE_PACKET_MAX_SIZE];      /**< Command response buffer. */

/**@brief   Event handler for the RPC Transport layer.
 *
 * @details  This function will be called when a command response is received in the transport
 *           layer and can be fetched by the Command Encoder module.
 *           The response is decoded and returned to the waiting caller.
 *
 * @param[in] event The event from the RPC Transport layer.
 */
static void rpc_transport_event_handler(rpc_transport_evt_type_t event)
{
    static uint32_t received = 0;
    uint32_t        err_code;

    // @todo Define correct behavior. Currently call APP_ERROR_CHECK(...) for the prototype.
    err_code = rpc_transport_packet_read(RPC_TRANSPORT_RESP,
                                         sizeof(m_cmd_response_buf),
                                         m_cmd_response_buf,
                                         &received);
    APP_ERROR_CHECK(err_code);

    m_cmd_response.op_code = (rpc_ble_op_code_t)m_cmd_response_buf[0];

    if (received >= RPC_CMD_RESPONSE_PACKET_MIN_SIZE)
    {
        m_cmd_response.err_code = uint32_decode(&m_cmd_response_buf[1]);
    }
    else
    {
        m_cmd_response.err_code = NRF_ERROR_INTERNAL;
    }
}


/**@brief This function will block in a loop, using WFE to allow low power mode, while awaiting a
 *        response from the connectivity chip.
 *
 * @param[in] op_code   The Operation Code for which a response message is expected.
 *
 * @return    The decoded error code received from the connectivity chip.
 */
static uint32_t wait_for_response(rpc_ble_op_code_t op_code)
{
    for(;;)
    {
        __WFE();

        if (m_cmd_response.op_code == op_code)
        {
            m_cmd_response.op_code = RPC_SD_BLE_INVALID_OPCODE;

            return m_cmd_response.err_code;
        }
    }
}


/**@brief       This functions encodes the peer address into the packet array provided and returns
 *              the number of bytes encoded.
 *
 * @param[in]   p_packet        Pointer to the memory location where the encoded peer address should
 *                              be stored. The memory location should hold minimum
 *                              @ref BLE_GAP_ADDR_LEN + addr_type of data (7 bytes total).
 * @param[in]   p_peer_address  Pointer to the peer address that should be encoded.
 *
 * @return      Number of bytes encoded.
 */
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


/**@brief       This functions encodes the white list into the packet array provided and returns
 *              the number of bytes encoded.
 *
 * @param[in]   p_packet     Pointer to the memory location where the encoded peer address should
 *                           be stored. The memory location should hold minimum the address and  irk
 *                           counters (2 bytes), (@ref BLE_GAP_ADDR_LEN + addr_type) *
 *                           @ref BLE_GAP_WHITELIST_ADDR_MAX_COUNT of data (56 bytes), and
 *                           @ref BLE_GAP_SEC_KEY_LEN * @ref BLE_GAP_WHITELIST_IRK_MAX_COUNT of data
 *                           (128 bytes). In total 186 bytes.
 * @param[in]   p_whitelist  Pointer to the peer address that should be encoded.
 *
 * @return      Number of bytes encoded.
 */
static uint32_t whitelist_encode(uint8_t *                         p_packet,
                                 const ble_gap_whitelist_t * const p_whitelist)
{
    uint32_t i;
    uint32_t j;
    uint32_t index = 0;

    p_packet[index++] = p_whitelist->addr_count;
    for (i = 0; i < p_whitelist->addr_count; i++)
    {
        p_packet[index++] = p_whitelist->pp_addrs[i]->addr_type;
        for (j = 0; j < BLE_GAP_ADDR_LEN; j++)
        {
            p_packet[index++] = p_whitelist->pp_addrs[i]->addr[j];
        }
    }

    p_packet[index++] = p_whitelist->irk_count;
    for (i = 0; i < p_whitelist->irk_count; i++)
    {
        for (j = 0; j < BLE_GAP_SEC_KEY_LEN; j++)
        {
            p_packet[index++] = p_whitelist->pp_irks[i]->irk[j];
        }
    }

    return index;
}


uint32_t sd_ble_gap_device_name_set(ble_gap_conn_sec_mode_t const * const p_write_perm,
                                    uint8_t const                 * const p_dev_name,
                                    uint16_t                              len)
{
    uint32_t i      = 0;
    uint32_t index  = 0;
    uint8_t  packet[RPC_CMD_ENCODER_DEVICE_NAME_PACKET_SIZE_MAX];

    packet[index++] = RPC_SD_BLE_GAP_DEVICE_NAME_SET;
    packet[index++] = (uint8_t) ((p_write_perm->sm) | (p_write_perm->lv << 4));
    index          += uint16_encode(len, &packet[index]);

    for(; i < len; ++i)
    {
        packet[index++] = p_dev_name[i];
    }

    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }

    return wait_for_response(RPC_SD_BLE_GAP_DEVICE_NAME_SET);
}


uint32_t sd_ble_gap_appearance_set(uint16_t appearance)
{
    uint32_t index = 0;
    uint8_t  packet[RPC_CMD_ENCODER_APPEARANCE_PACKET_SIZE];

    packet[index++] = RPC_SD_BLE_GAP_APPEARANCE_SET;
    index          += uint16_encode(appearance, &packet[index]);

    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }

    return wait_for_response(RPC_SD_BLE_GAP_APPEARANCE_SET);
}


uint32_t sd_ble_gap_ppcp_set(ble_gap_conn_params_t const * const p_conn_params)
{
    uint32_t index = 0;
    uint8_t  packet[RPC_CMD_ENCODER_PPCP_PACKET_SIZE];

    packet[index++] = RPC_SD_BLE_GAP_PPCP_SET;

    index += uint16_encode(p_conn_params->min_conn_interval, &packet[index]);
    index += uint16_encode(p_conn_params->max_conn_interval, &packet[index]);
    index += uint16_encode(p_conn_params->slave_latency, &packet[index]);
    index += uint16_encode(p_conn_params->conn_sup_timeout, &packet[index]);

    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }

    return wait_for_response(RPC_SD_BLE_GAP_PPCP_SET);
}


uint32_t sd_ble_gap_ppcp_get(ble_gap_conn_params_t * const p_conn_params)
{
    uint8_t  packet[RPC_CMD_ENCODER_PPCP_GET_PACKET_SIZE];
    uint32_t err_code; 

    uint32_t index = 0;    
        
    packet[index++] = RPC_SD_BLE_GAP_PPCP_GET;                        
    if (p_conn_params != NULL)
    {
        packet[index++] = RPC_BLE_FIELD_PRESENT;                   
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;                
    }

    err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }
        
    err_code = wait_for_response(RPC_SD_BLE_GAP_PPCP_GET);
    if (p_conn_params != NULL)
    {
        memcpy(p_conn_params, 
            &(m_cmd_response_buf[BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE]),
            sizeof(*p_conn_params)); 
    }
               
    return err_code;
}


uint32_t sd_ble_gap_adv_start(ble_gap_adv_params_t const * const p_adv_params)
{
    uint32_t index = 0;
    uint8_t  packet[RPC_CMD_ENCODER_ADV_START_PACKET_SIZE_MAX];

    packet[index++] = RPC_SD_BLE_GAP_ADV_START;
    packet[index++] = p_adv_params->type;

    if (p_adv_params->p_peer_addr == NULL)
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_PRESENT;
        index          += peer_address_encode(&packet[index], p_adv_params->p_peer_addr);
    }

    packet[index++] = p_adv_params->fp;

    if (p_adv_params->p_whitelist == NULL)
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_PRESENT;
        index          += whitelist_encode(&packet[index], p_adv_params->p_whitelist);
    }

    index += uint16_encode(p_adv_params->interval, &packet[index]);
    index += uint16_encode(p_adv_params->timeout, &packet[index]);
    

    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }

    return wait_for_response(RPC_SD_BLE_GAP_ADV_START);
}


uint32_t sd_ble_gap_adv_data_set(uint8_t const * const p_data,
                                 uint8_t               dlen,
                                 uint8_t const * const p_sr_data,
                                 uint8_t               srdlen)
{
    uint32_t i     = 0;
    uint32_t index = 0;
    uint8_t  packet[RPC_CMD_ENCODER_ADV_PACKET_SIZE_MAX];

    packet[index++] = RPC_SD_BLE_GAP_ADV_DATA_SET;
    packet[index++] = dlen;

    for(; i < dlen; ++i)
    {
        packet[index++] = p_data[i];
    }

    packet[index++] = srdlen;

    i = 0;
    for (; i < srdlen; ++i)
    {
        packet[index++] = p_sr_data[i];
    }

    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }

    return wait_for_response(RPC_SD_BLE_GAP_ADV_DATA_SET);
}


uint32_t sd_ble_gap_device_name_get(uint8_t * const p_dev_name, uint16_t * const p_len)
{
    uint8_t  packet[RPC_CMD_ENCODER_DEVICE_NAME_GET_PACKET_SIZE];        
        
    uint32_t index  = 0;        
    packet[index++] = RPC_SD_BLE_GAP_DEVICE_NAME_GET;
    if (p_len != NULL)
    {
        packet[index++] = RPC_BLE_FIELD_PRESENT;  
        index          += uint16_encode(*p_len, &packet[index]);              
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;  
    }
    packet[index++] = (p_dev_name != NULL) ? RPC_BLE_FIELD_PRESENT : RPC_BLE_FIELD_NOT_PRESENT;                
        
    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);    
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }   

    err_code = wait_for_response(RPC_SD_BLE_GAP_DEVICE_NAME_GET);    
    if (p_len != NULL)
    {
        *p_len   = uint16_decode(&(m_cmd_response_buf[BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE]));
        if (p_dev_name != NULL)
        {
            memcpy(p_dev_name, 
                   &(m_cmd_response_buf[BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE + sizeof(*p_len)]),
                   *p_len);
        }
    }

    return err_code;            
}


uint32_t sd_ble_gap_appearance_get(uint16_t * const p_appearance)
{
    uint8_t packet[RPC_CMD_ENCODER_APPEARANCE_GET_PACKET_SIZE];    
    uint32_t index  = 0;  
    packet[index++] = RPC_SD_BLE_GAP_APPEARANCE_GET;            
    if (p_appearance != NULL)
    {         
        packet[index++] = RPC_BLE_FIELD_PRESENT;
   
    }
    else
    {
       packet[index++] = RPC_BLE_FIELD_NOT_PRESENT; 
    }
    
    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD,packet,index);
    
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }    
        
    err_code      = wait_for_response(RPC_SD_BLE_GAP_APPEARANCE_GET);    
    *p_appearance = uint16_decode(&(m_cmd_response_buf[BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE]));
        
    return err_code;        
}


uint32_t sd_ble_uuid_encode(ble_uuid_t const * const p_uuid,
                            uint8_t          * const p_uuid_le_len,
                            uint8_t          * const p_uuid_le)
{
    uint8_t  packet[RPC_CMD_ENCODER_UUID_ENCODE_PACKET_SIZE];        
    uint32_t err_code;
    
    uint32_t index  = 0;    
    packet[index++] = RPC_SD_BLE_UUID_ENCODE;
    if (p_uuid != NULL)
    {    
        packet[index++] = RPC_BLE_FIELD_PRESENT;    
        index          += uint16_encode(p_uuid->uuid, &packet[index]);      
        packet[index++] = p_uuid->type;                    
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;         
    }
    if (p_uuid_le_len != NULL)
    {        
        packet[index++] = RPC_BLE_FIELD_PRESENT;         
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;         
    }        
    if (p_uuid_le != NULL)
    {        
        packet[index++] = RPC_BLE_FIELD_PRESENT;         
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;         
    }            

    err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }
        
    err_code = wait_for_response(RPC_SD_BLE_UUID_ENCODE);    
    if (p_uuid_le_len != NULL)
    {
        *p_uuid_le_len = m_cmd_response_buf[BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE];
    }
    if ((p_uuid_le != NULL) && (p_uuid_le_len != NULL))
    {
        memcpy(p_uuid_le, 
            &(m_cmd_response_buf[BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE + sizeof(*p_uuid_le_len)]),
            *p_uuid_le_len);        
    }
    
    return err_code;

}


uint32_t sd_ble_gatts_service_add(uint8_t                  type, 
                                  ble_uuid_t const * const p_uuid, 
                                  uint16_t         * const p_handle)
{
    uint8_t  packet[RPC_CMD_ENCODER_SERVICE_ADD_PACKET_SIZE];    
    uint32_t index = 0;
    
    packet[index++] = RPC_SD_BLE_GATTS_SERVICE_ADD;
    packet[index++] = type;
    
    if (p_uuid != NULL)
    {
        packet[index++] = RPC_BLE_FIELD_PRESENT;    
        index          += uint16_encode(p_uuid->uuid, &packet[index]);    
        packet[index++] = p_uuid->type;
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;    
    }
    
    if (p_handle != NULL)
    {
        packet[index++] = RPC_BLE_FIELD_PRESENT;    
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;        
    }
    
    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }

    err_code = wait_for_response(RPC_SD_BLE_GATTS_SERVICE_ADD);    
    if (p_handle != NULL)
    {
        *p_handle = uint16_decode(&(m_cmd_response_buf[BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE]));        
    }
    
    return err_code;
}


uint32_t sd_ble_gatts_sys_attr_set(uint16_t              conn_handle, 
                                   uint8_t const * const p_sys_attr_data, 
                                   uint16_t              len)
{
    uint8_t packet[RPC_CMD_ENCODER_SYSTEM_ATTR_SET_PACKET_SIZE];    
    
    if (len <= (sizeof(packet) - BLE_OP_CODE_SIZE - sizeof(conn_handle)))
    {
        uint32_t index = 0;
        
        packet[index++] = RPC_SD_BLE_GATTS_SYS_ATTR_SET;
        index          += uint16_encode(conn_handle, &packet[index]);
        
        if (p_sys_attr_data != NULL)
        {
            packet[index++] = RPC_BLE_FIELD_PRESENT;        
            index          += uint16_encode(len, &packet[index]);           
            memcpy(&(packet[index]), p_sys_attr_data, len);         
            index += len;        
        }
        else
        {
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;        
        }
        
        uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
        if (err_code != NRF_SUCCESS)
        {
            // @todo Define correct behavior. Should all errors be propagated to the application, or
            //       should some be handled here. E.g. full buffer.
            return err_code;
        }    
        
        return wait_for_response(RPC_SD_BLE_GATTS_SYS_ATTR_SET);        
    }
    else
    {
        return NRF_ERROR_NO_MEM;
    }    
}


uint32_t sd_ble_gatts_hvx(uint16_t conn_handle, ble_gatts_hvx_params_t const * const p_hvx_params)
{
    uint8_t  packet[RPC_CMD_ENCODER_HVX_PACKET_SIZE];    
    uint32_t index = 0;
    
    packet[index++] = RPC_SD_BLE_GATTS_HVX;
    index          += uint16_encode(conn_handle, &packet[index]);   

    if (p_hvx_params != NULL)
    {
        packet[index++] = RPC_BLE_FIELD_PRESENT;        
        index          += uint16_encode(p_hvx_params->handle, &packet[index]);           
        packet[index++] = p_hvx_params->type;    
        index          += uint16_encode(p_hvx_params->offset, &packet[index]);                   
        if (p_hvx_params->p_len != NULL)
        {        
            if (*(p_hvx_params->p_len) > (sizeof(packet) - 
                (BLE_OP_CODE_SIZE                        + 
                sizeof(conn_handle)                      + 
                RPC_BLE_FIELD_LEN                        +
                sizeof(p_hvx_params->handle)             +
                sizeof(p_hvx_params->type)               +                
                sizeof(p_hvx_params->offset)             +                                
                RPC_BLE_FIELD_LEN                        +                                                
                sizeof(*(p_hvx_params->p_len))           +
                RPC_BLE_FIELD_LEN)))
            {
                return NRF_ERROR_DATA_SIZE;
            }
          
            packet[index++] = RPC_BLE_FIELD_PRESENT;                
            index          += uint16_encode(*(p_hvx_params->p_len), &packet[index]);                           
            
            if (p_hvx_params->p_data != NULL)
            {        
                packet[index++] = RPC_BLE_FIELD_PRESENT;                        
                memcpy(&(packet[index]), p_hvx_params->p_data, *(p_hvx_params->p_len));        
                index += *(p_hvx_params->p_len);        
            }
            else
            {
                packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;                            
            }            
        }
        else
        {
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;                    
            // @note: Also data field is omitted if length field is omitted.
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;                                
        }
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;            
    }
    
    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }    
    
    err_code = wait_for_response(RPC_SD_BLE_GATTS_HVX);
    if ((p_hvx_params != NULL) && (p_hvx_params->p_len != NULL))
    {
        *(p_hvx_params->p_len) = uint16_decode(&
                                 (m_cmd_response_buf[BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE]));    
    }

    return err_code;
}


uint32_t sd_ble_gap_disconnect(uint16_t conn_handle, uint8_t hci_status_code)
{
    uint8_t  packet[RPC_CMD_ENCODER_DISCONNECT_PACKET_SIZE];    
    uint32_t index = 0;
    
    packet[index++] = RPC_SD_BLE_GAP_DISCONNECT;
    index          += uint16_encode(conn_handle, &packet[index]);   
    packet[index++] = hci_status_code;                                
    
    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }    
    
    return wait_for_response(RPC_SD_BLE_GAP_DISCONNECT);    
}


uint32_t sd_ble_gap_sec_params_reply(uint16_t                           conn_handle, 
                                     uint8_t                            sec_status, 
                                     ble_gap_sec_params_t const * const p_sec_params)
{
    uint8_t  packet[RPC_CMD_ENCODER_SEC_PARAMS_PACKET_SIZE];    
    uint32_t index = 0;
    
    packet[index++] = RPC_SD_BLE_GAP_SEC_PARAMS_REPLY;
    index          += uint16_encode(conn_handle, &packet[index]);   
    packet[index++] = sec_status;                                
    
    if (p_sec_params != NULL)
    {
        packet[index++] = RPC_BLE_FIELD_PRESENT;
        index          += uint16_encode(p_sec_params->timeout, &packet[index]);                   
        packet[index++] = (
                        (p_sec_params->oob << 5)     |
                        (p_sec_params->io_caps << 2) |
                        (p_sec_params->mitm << 1)    |
                        (p_sec_params->bond << 0)
                        );
        packet[index++] = p_sec_params->min_key_size;        
        packet[index++] = p_sec_params->max_key_size;                
    }
    else
    {   
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;                                
    }
    
    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }    
    
    return wait_for_response(RPC_SD_BLE_GAP_SEC_PARAMS_REPLY);    
}


uint32_t sd_ble_gap_address_set(ble_gap_addr_t const * const p_addr)
{
    uint8_t packet[RPC_CMD_ENCODER_ADDRESS_SET_PACKET_SIZE];  
    
    if (p_addr != NULL)
    {
        uint32_t index  = 0;    
        packet[index++] = RPC_SD_BLE_GAP_ADDRESS_SET;
        packet[index++] = p_addr->addr_type;
        packet[index++] = p_addr->addr[0];
        packet[index++] = p_addr->addr[1];        
        packet[index++] = p_addr->addr[2];                
        packet[index++] = p_addr->addr[3];                        
        packet[index++] = p_addr->addr[4];                                
        packet[index++] = p_addr->addr[5];                                        
        
        uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
        if (err_code != NRF_SUCCESS)
        {
            // @todo Define correct behavior. Should all errors be propagated to the application, or
            //       should some be handled here. E.g. full buffer.
            return err_code;
        }            
        
        return wait_for_response(RPC_SD_BLE_GAP_ADDRESS_SET);            
    }
    else
    {
        return NRF_ERROR_INVALID_ADDR;
    }
}


uint32_t sd_ble_gap_address_get(ble_gap_addr_t * const p_addr)
{
    uint8_t  packet[RPC_CMD_ENCODER_ADDRESS_GET_PACKET_SIZE];  
    uint32_t err_code;
    
    if (p_addr != NULL)
    {
        packet[0] = RPC_SD_BLE_GAP_ADDRESS_GET;    
        err_code  = rpc_transport_packet_write(RPC_TRANSPORT_CMD, 
                                                       packet, 
                                                       sizeof(packet[0]));
        if (err_code != NRF_SUCCESS)
        {
            // @todo Define correct behavior. Should all errors be propagated to the application, or
            //       should some be handled here. E.g. full buffer.
            return err_code;
        }            
        
        err_code = wait_for_response(RPC_SD_BLE_GAP_ADDRESS_GET);
        p_addr->addr_type = m_cmd_response_buf[BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE];
        memcpy(p_addr->addr, 
               &(m_cmd_response_buf[
               BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE + sizeof(p_addr->addr_type)]),
               sizeof(p_addr->addr));                        
    }
    else
    {       
        err_code = NRF_ERROR_INVALID_ADDR;
    }
    
    return err_code;
}

static uint32_t decode_gatts_char_md(ble_gatts_attr_md_t * gatts_attr_md, uint8_t * packet)
{
    uint32_t index = 0;
    packet[index++] = (gatts_attr_md->read_perm.sm  | gatts_attr_md->read_perm.lv << 4 );
    packet[index++] = (gatts_attr_md->write_perm.sm | gatts_attr_md->write_perm.lv << 4 );
    packet[index++] = (gatts_attr_md->vlen    << 0 |
                       gatts_attr_md->vloc    << 1 |
                       gatts_attr_md->rd_auth << 3 |
                       gatts_attr_md->wr_auth << 4);
    return index;
}

 uint32_t sd_ble_gatts_characteristic_add(uint16_t                         service_handle,
                                          ble_gatts_char_md_t const *const p_char_md,
                                          ble_gatts_attr_t const    *const p_attr_char_value,
                                          ble_gatts_char_handles_t  *const p_handles)
{
    uint8_t packet[RPC_CMD_ENCODER_CHARACTERISTICS_ADD_PACKET_SIZE];
    //memset(&packet, 0, sizeof(packet));
    uint32_t index = 0;
    
    packet[index++] = RPC_SD_BLE_GATTS_CHARACTERISTIC_ADD;
    index += uint16_encode(service_handle, &packet[index]);
    
    if(p_char_md !=NULL)
    {    
        packet[index++] = RPC_BLE_FIELD_PRESENT;

        packet[index++] = ((p_char_md->char_props.broadcast      << 0)|
                           (p_char_md->char_props.read           << 1)|
                           (p_char_md->char_props.write_wo_resp  << 2)|
                           (p_char_md->char_props.write          << 3)|
                           (p_char_md->char_props.notify         << 4)|
                           (p_char_md->char_props.indicate       << 5)|
                           (p_char_md->char_props.auth_signed_wr << 6));

        packet[index++] = ((p_char_md->char_ext_props.reliable_wr << 0)|
                           (p_char_md->char_ext_props.wr_aux << 1));

        if(p_char_md->p_char_user_desc != NULL)
        {
            packet[index++] = RPC_BLE_FIELD_PRESENT;
            packet[index++] = *(p_char_md->p_char_user_desc);
        }
        else
        {
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
        }
        index          += uint16_encode(p_char_md->char_user_desc_max_size, &packet[index]);
        index          += uint16_encode(p_char_md->char_user_desc_size, &packet[index]);
        
        if(p_char_md->p_char_pf != NULL)
        {
            packet[index++] = RPC_BLE_FIELD_PRESENT;
            packet[index++] = p_char_md->p_char_pf->format;
            packet[index++] = p_char_md->p_char_pf->exponent;
            index += uint16_encode(p_char_md->p_char_pf->unit, &packet[index]);
            packet[index++] = p_char_md->p_char_pf->name_space;
            index += uint16_encode(p_char_md->p_char_pf->desc, &packet[index]);
        }
        else
        {
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
        }
        if(p_char_md->p_user_desc_md != NULL)
        {
            packet[index++] = RPC_BLE_FIELD_PRESENT;
            index += decode_gatts_char_md(p_char_md->p_user_desc_md, &packet[index]);
        }
        else
        {
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
        }

        if(p_char_md->p_cccd_md != NULL)
        {
            packet[index++] = RPC_BLE_FIELD_PRESENT;
            index += decode_gatts_char_md(p_char_md->p_cccd_md, &packet[index]);
        }
        else
        {
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
        }

        if(p_char_md->p_sccd_md != NULL)
        {
            packet[index++] = RPC_BLE_FIELD_PRESENT;
            index += decode_gatts_char_md(p_char_md->p_sccd_md, &packet[index]);
        }
        else
        {
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
        }
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
    }
  
    if(p_attr_char_value !=NULL)
    {    
        packet[index++] = RPC_BLE_FIELD_PRESENT;
        if(p_attr_char_value->p_uuid !=NULL)
        {    
            packet[index++] = RPC_BLE_FIELD_PRESENT;
            index          += uint16_encode(p_attr_char_value->p_uuid->uuid, &packet[index]);
            packet[index++] = p_attr_char_value->p_uuid->type;
        }
        else
        {
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
        }

        if(p_attr_char_value->p_attr_md !=NULL)
        {
            packet[index++] = RPC_BLE_FIELD_PRESENT;
            index += decode_gatts_char_md(p_attr_char_value->p_attr_md, &packet[index]);
        }
        else
        {
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
        }

        index          += uint16_encode(p_attr_char_value->init_len, &packet[index]);
        index          += uint16_encode(p_attr_char_value->init_offs, &packet[index]);
        index          += uint16_encode(p_attr_char_value->max_len, &packet[index]);

        if(p_attr_char_value->p_value !=NULL)
        {    
            packet[index++] = RPC_BLE_FIELD_PRESENT;
            packet[index++] = *(p_attr_char_value->p_value);
        }
        else
        {
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
        }
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
    }     
    if(p_handles !=NULL)
    {    
        packet[index++] = RPC_BLE_FIELD_PRESENT;
        index          += uint16_encode(p_handles->value_handle, &packet[index]);
        index          += uint16_encode(p_handles->user_desc_handle, &packet[index]);
        index          += uint16_encode(p_handles->cccd_handle, &packet[index]);
        index          += uint16_encode(p_handles->sccd_handle, &packet[index]);
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
    }
    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }    
    
    err_code = wait_for_response(RPC_SD_BLE_GATTS_CHARACTERISTIC_ADD);
    //Put uint16 into struct
    index = BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE;
    p_handles->value_handle = uint16_decode(&m_cmd_response_buf[index]);
    index += sizeof(uint16_t);
    p_handles->user_desc_handle = uint16_decode(&m_cmd_response_buf[index]);
    index += sizeof(uint16_t);
    p_handles->cccd_handle = uint16_decode(&m_cmd_response_buf[index]);
    index += sizeof(uint16_t);
    p_handles->sccd_handle = uint16_decode(&m_cmd_response_buf[index]);
    
    return err_code;
}

            
uint32_t sd_ble_gatts_value_set(uint16_t              handle, 
                                uint16_t              offset, 
                                uint16_t * const      p_len, 
                                uint8_t const * const p_value)
{
    uint8_t packet[RPC_CMD_ENCODER_VALUE_SET_PACKET_SIZE];  
    
    uint32_t index  = 0;    
    packet[index++] = RPC_SD_BLE_GATTS_VALUE_SET;
    index          += uint16_encode(handle, &packet[index]);    
    index          += uint16_encode(offset, &packet[index]);        
    if (p_len != NULL)
    {
        if ((*p_len) > (sizeof(packet) - 
            (BLE_OP_CODE_SIZE          +     
            sizeof(handle)             +    
            sizeof(offset)             +            
            RPC_BLE_FIELD_LEN          +            
            sizeof(*p_len)             +            
            RPC_BLE_FIELD_LEN)))
        {            
            return NRF_ERROR_DATA_SIZE;
        }
        
        packet[index++] = RPC_BLE_FIELD_PRESENT;        
        index          += uint16_encode(*p_len, &packet[index]);     
        if (p_value != NULL)
        {
            packet[index++] = RPC_BLE_FIELD_PRESENT;    
            memcpy(&(packet[index]), p_value, *p_len);        
            index          += *p_len;                    
        }
        else
        {
            packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;                
        }
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;        
        // @note: If len field is omitted value field must also be omitted.        
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;                
    }
    
    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }         

    err_code = wait_for_response(RPC_SD_BLE_GATTS_VALUE_SET);       
    
    if (p_len != NULL)
    {
        *p_len   = m_cmd_response_buf[BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE];
    }
    
    return err_code;    
}


uint32_t sd_ble_gap_conn_param_update(uint16_t                            conn_handle, 
                                      ble_gap_conn_params_t const * const p_conn_params)
{
    uint8_t packet[RPC_CMD_ENCODER_CONN_PARAM_UPDATE_PACKET_SIZE]; 

    uint32_t index  = 0;    
    packet[index++] = RPC_SD_BLE_GAP_CONN_PARAM_UPDATE; 
    index          += uint16_encode(conn_handle, &packet[index]);        
    if (p_conn_params != NULL)
    {
        packet[index++] = RPC_BLE_FIELD_PRESENT;         
        index          += uint16_encode(p_conn_params->min_conn_interval, &packet[index]);            
        index          += uint16_encode(p_conn_params->max_conn_interval, &packet[index]);                
        index          += uint16_encode(p_conn_params->slave_latency, &packet[index]);                    
        index          += uint16_encode(p_conn_params->conn_sup_timeout, &packet[index]);                            
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;             
    }
    
    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }  

    return wait_for_response(RPC_SD_BLE_GAP_CONN_PARAM_UPDATE);           
}


/**@brief Decode sd_ble_gatts_sys_attr_get API response data. 
 *
 * @param[out] p_sys_attr_data Decode data buffer output.
 * @param[out] p_len           Decode data length output.
 */
static __INLINE void gatts_sys_attr_get_decode(uint8_t  * const p_sys_attr_data, 
                                               uint16_t * const p_len)
{
    if (p_len != NULL)
    {    
        *p_len = uint16_decode(&(m_cmd_response_buf[BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE]));    
        
        const uint32_t attributes_data_present_offset = BLE_OP_CODE_SIZE + BLE_ERR_CODE_SIZE + 2u;            
        const uint32_t attributes_offset              = attributes_data_present_offset + 1u;
            
        if ((p_sys_attr_data != NULL) && 
            (m_cmd_response_buf[attributes_data_present_offset] == RPC_BLE_FIELD_PRESENT))
        {
            if (*p_len <= (RPC_CMD_RESPONSE_PACKET_MAX_SIZE - attributes_offset))
            {
                memcpy(p_sys_attr_data, &(m_cmd_response_buf[attributes_offset]), *p_len);
            }
            else
            {
                // @note If this branch is executed it implies that for some reason length field has
                // errnous data as the packet reception API is length checked.
                APP_ERROR_HANDLER(*p_len);            
            }
        }            
    }    
}


uint32_t sd_ble_gatts_sys_attr_get(uint16_t         conn_handle, 
                                   uint8_t  * const p_sys_attr_data, 
                                   uint16_t * const p_len)
{
    uint8_t packet[RPC_CMD_ENCODER_SYS_ATTR_GET_PACKET_SIZE]; 
    
    uint32_t index  = 0;    
    packet[index++] = RPC_SD_BLE_GATTS_SYS_ATTR_GET;     
    index          += uint16_encode(conn_handle, &packet[index]);   
    if (p_len != NULL)
    {
        packet[index++] = RPC_BLE_FIELD_PRESENT;             
        index          += uint16_encode(*p_len, &packet[index]);       
    }
    else
    {
        packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;                 
    }
    packet[index++] = (p_sys_attr_data != NULL) ? RPC_BLE_FIELD_PRESENT : RPC_BLE_FIELD_NOT_PRESENT;
    
    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }  

    err_code = wait_for_response(RPC_SD_BLE_GATTS_SYS_ATTR_GET);  
    if (err_code == NRF_SUCCESS)
    {
        gatts_sys_attr_get_decode(p_sys_attr_data, p_len);
    }

    return err_code;
}



uint32_t sd_power_system_off(void)
{
    // @note: This function will only be placing the connectivity chip in System Off mode. 
    uint8_t packet[RPC_CMD_ENCODER_POWER_SYSTEM_OFF_PACKET_SIZE]; 
    
    uint32_t index  = 0;    
    packet[index++] = RPC_SD_POWER_SYSTEM_OFF;
    
    uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);
    if (err_code != NRF_SUCCESS)
    {
        // @todo Define correct behavior. Should all errors be propagated to the application, or
        //       should some be handled here. E.g. full buffer.
        return err_code;
    }  

    return err_code;
}

uint32_t sd_ble_gap_sec_info_reply(uint16_t                          conn_handle,
		                           ble_gap_enc_info_t  const * const p_enc_info,
		                           ble_gap_sign_info_t const * const p_sign_info)
{
	uint8_t packet[RPC_CMD_ENCODER_SEC_INFO_REPLY_PACKET_SIZE];

	uint32_t index = 0;
	packet[index++] = RPC_SD_BLE_GAP_SEC_INFO_REPLY;
	index += uint16_encode(conn_handle, &packet[index]);
	if (p_enc_info != NULL)
	{
		packet[index++] = RPC_BLE_FIELD_PRESENT;
		index += uint16_encode(p_enc_info->div, &packet[index]);
		memcpy(&packet[index], p_enc_info->ltk, BLE_GAP_SEC_KEY_LEN);
		index += BLE_GAP_SEC_KEY_LEN;
		packet[index++] = (p_enc_info->auth | (p_enc_info->ltk_len << 1));
	}
	else
	{
		packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
	}

	if (p_sign_info != NULL)
	{
		packet[index++] = RPC_BLE_FIELD_PRESENT;
		memcpy(&packet[index], p_sign_info->csrk, BLE_GAP_SEC_KEY_LEN);
		index += BLE_GAP_SEC_KEY_LEN;
	}
	else
	{
		packet[index++] = RPC_BLE_FIELD_NOT_PRESENT;
	}

	uint32_t err_code = rpc_transport_packet_write(RPC_TRANSPORT_CMD, packet, index);

	if (err_code != NRF_SUCCESS)
	{
		// @todo Define correct behavior. Should all errors be propagated to the application, or
		//       should some be handled here. E.g. full buffer.
		return err_code;
	}

	return wait_for_response(RPC_SD_BLE_GAP_SEC_INFO_REPLY);
}


uint32_t ble_rpc_cmd_encoder_init(void)
{
    uint32_t transport_count;
    uint32_t err_code;
    
    err_code = rpc_transport_open(&transport_count);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = rpc_transport_register(RPC_TRANSPORT_RESP, rpc_transport_event_handler);

    return err_code;
}


uint32_t sd_ble_gatts_descriptor_add(uint16_t                       char_handle, 
                                     ble_gatts_attr_t const * const p_attr, 
                                     uint16_t* const                p_handle)
{
    return NRF_ERROR_NOT_SUPPORTED;
}
