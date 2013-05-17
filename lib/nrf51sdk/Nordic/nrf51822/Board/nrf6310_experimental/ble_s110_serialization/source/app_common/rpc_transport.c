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
 
#include "rpc_transport.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "rpc_transport_internal.h"
#include "compiler_abstraction.h"
#include "app_transport.h"
#include "app_error.h"

/**@brief  States of the RX state machine. */
typedef enum
{
    RX_IDLE,                                                                                             /**< Idle state. */
    RX_READ_LENGTH,                                                                                      /**< Read length field state. */
    RX_READ_DATA,                                                                                        /**< Read data section state. */
    RX_W4_EXTRACT                                                                                        /**< Wait for the user to extract the received packet. */    
} rx_state_t;

/**@brief Rx state machine events. */
typedef enum
{
    RX_DATA_RDY                                                                                          /**< RX-data ready event received from the underlying layer. */
} rx_event_t;

static void rx_sm_state_change(rx_state_t new_state);

static rx_state_t                    m_rx_state                                                = RX_IDLE;/**< Current RX state. */
static bool                          m_is_rx_data_available                                    = false;  /**< True if unread packet is available to be read. */
static uint32_t                      m_rx_packet_size                                          = 0;      /**< The full size in bytes of the last received packet. */
static uint32_t                      m_channel_open_count                                      = 0;      /**< Channel open reference counter. */
static rpc_transport_event_handler_t m_rpc_transport_event_handler[RPC_TRANSPORT_PKT_TYPE_MAX] = {NULL}; /**< Registered event handlers. */
static uint8_t                       m_rx_buffer[RPC_RX_BUF_SIZE]                              = {NULL}; /**< RX-data buffer. */       
static uint32_t                      m_rx_data_read_count                                      = 0;      /**< Amount of RX-data read length that is read from the packet length field. */
    

/**@brief Reads the packet length amount of data and processes it.
 */
static void packet_length_field_read()
{    
    uint32_t actual_length;
    uint32_t err_code;
    
    static uint32_t i = 0;

    do
    {
        err_code = app_transport_read(&(m_rx_buffer[i]), 1u, &actual_length);
        switch (err_code)
        {
            case NRF_SUCCESS:    
                ++i;
                if (i == RPC_LENGTH_FIELD_SIZE)
                {
                    // Full length field read, extract it, reset internal state, set loop break 
                    // value and transit to next state.
                    i                    = 0;
                    err_code             = ~NRF_SUCCESS;
                    m_rx_data_read_count = *((uint32_t *)m_rx_buffer);
                    
                    rx_sm_state_change(RX_READ_DATA);                
                }
                break;
                
            default:
                // No data was extracted, silently discard.
                break;        
        }            
    } while (err_code == NRF_SUCCESS);
    
#if 0
    uint32_t err_code;
    uint32_t actual_length;
    
    err_code = app_transport_read(m_rx_buffer, RPC_LENGTH_FIELD_SIZE, &actual_length);
    switch (err_code)
    {
        case NRF_SUCCESS:
            // Full length field read, extract it and transit to next state.
            m_rx_data_read_count = *((uint32_t *)m_rx_buffer);
            rx_sm_state_change(RX_READ_DATA);
            break;

        case NRF_ERROR_DATA_SIZE:            
            // Actual read amount is not equal to requested read amount.
            
            break;
            
        default:
            APP_ERROR_HANDLER(err_code);
            break;
    }
#endif // 0    
}


/**@brief Decodes received packet type and indicates it to a registered handler.
 */
static void packet_rx_ready_indicate()
{
    rpc_transport_event_handler_t handler;
    
    const uint32_t packet_type = *((uint32_t *)m_rx_buffer);
    
    switch (packet_type)
    {
        case RPC_TRANSPORT_CMD:
        case RPC_TRANSPORT_RESP:
        case RPC_TRANSPORT_EVT:
            // Valid packet type decoded, set all relevant state and memory vaiables 
            // and indicate packet reception to a registered packet handler.
            m_is_rx_data_available = true;
            m_rx_packet_size       = m_rx_data_read_count + RPC_LENGTH_FIELD_SIZE;            
            handler                = m_rpc_transport_event_handler[packet_type];
            
            // @note: Checks that the application has registered a correct handler for the received 
            // packet type.
            APP_ERROR_CHECK_BOOL(handler != NULL);
            if (handler != NULL)
            {
                handler((rpc_transport_evt_type_t)packet_type);        
            }
            break;
            
        default:
            break;
    }    
}


/**@brief Reads packet data and processes it.
 */
static void packet_data_read(void)
{
    uint32_t actual_length;
    uint32_t err_code;
    
    static uint32_t i = 0;

    do
    {
        err_code = app_transport_read(&(m_rx_buffer[i]), 1u, &actual_length);
        switch (err_code)
        {
            case NRF_SUCCESS:    
                ++i;
                if (i == m_rx_data_read_count)
                {
                    // Full request amount read, reset internal state, 
                    // set loop break value and transit to next state.        
                    i        = 0;
                    err_code = ~NRF_SUCCESS;
                    rx_sm_state_change(RX_W4_EXTRACT);
                }
            
                break;
                
            default:
                // No data was extracted, silently discard.
                break;        
        }            
    } while (err_code == NRF_SUCCESS);
    
#if 0
    uint32_t actual_length;
    
    const uint32_t err_code = app_transport_read(m_rx_buffer, m_rx_data_read_count, &actual_length);
    switch (err_code)
    {
        case NRF_SUCCESS:
            // Full request amount read, transit to next state.        
            rx_sm_state_change(RX_READ_W4_EXTRACT);
            break;
        case RX_READ_LENGTH:
        
            break;
            
        default:
            break;
    }
#endif // 0    
}


/**@brief  Executes the RX state machine state entry action
 */
static __INLINE void rx_sm_state_entry_execute(void)
{
    switch (m_rx_state)
    {
        case RX_READ_LENGTH:
            packet_length_field_read();
            break;
    
        case RX_READ_DATA:
            packet_data_read();
            break;
            
        case RX_W4_EXTRACT:
            // Full request amount read, indicate rx-packet ready to upper layer. 
            packet_rx_ready_indicate();            
            break;
            
        default:
            break;
    }
}


/**@brief  Changes the current state of the RX state machine.
 *
 * @param[in] new_state State RX state machine changes to.
 */
static void rx_sm_state_change(rx_state_t new_state)
{
    m_rx_state = new_state;
    rx_sm_state_entry_execute();
}


/**@brief RX idle state event handler.
 *
 * @param[in] event Type of event that has occurred.
 */
static __INLINE void rx_idle_state_event_handle(rx_event_t event)
{
    switch (event)
    {
        case RX_DATA_RDY:
            rx_sm_state_change(RX_READ_LENGTH);
            break;
    
        default:
            break;
    }
}


/**@brief RX read length state event handler.
 *
 * @param[in] event Type of event occurred.
 */
static __INLINE void rx_read_len_state_event_handle(rx_event_t event)
{
    switch (event)
    {
        case RX_DATA_RDY:
            rx_sm_state_change(RX_READ_LENGTH);
            break;
    
        default:
            break;
    }
}


/**@brief Rx-read data state event handler.
 *
 * @param[in] event Type of event occurred.
 */
static __INLINE void rx_read_data_state_event_handle(rx_event_t event)
{
    switch (event)
    {
        case RX_DATA_RDY:
            rx_sm_state_change(RX_READ_DATA);
            break;
    
        default:
            break;
    }
}


/**@brief Rx-w4 data packet extract state event handler.
 *
 * @param[in] event Type of event occurred.
 */
static __INLINE void rx_w4_state_event_handle(rx_event_t event)
{
    switch (event)
    {
        case RX_DATA_RDY:
            // This is an error handler call implemented to catch an out-of-system definition 
            // implementation. This call should never happen, as it would mean that the upper layer 
            // has not extracted the received packet within the correct context.        
            APP_ERROR_HANDLER(event);
            break;
    
        default:
            break;
    }
}


/**@brief Rx statemachine event processing.
 *
 * @param[in] event Type of event occurred.
 */
static void rx_sm_event_handle(rx_event_t event)
{
    switch (m_rx_state)
    {
        case RX_IDLE:
            rx_idle_state_event_handle(event);
            break;
            
        case RX_READ_LENGTH:
            rx_read_len_state_event_handle(event);
            break;
    
        case RX_READ_DATA:
            rx_read_data_state_event_handle(event);
            break;
            
        case RX_W4_EXTRACT:
            rx_w4_state_event_handle(event);
            break;            
            
        default:
            break;
    }
}


/**@brief app_transport event handler callback function.
 *
 * @param[in] event Type of event occurred.
 */
void app_transport_event_handle(app_transport_evt_type_t event)
{
    switch (event)
    {
        case APP_TRANSPORT_RX_DATA_READY:
            rx_sm_event_handle(RX_DATA_RDY);
            break;
            
        case APP_TRANSPORT_ERROR:
            // For now call error handler for all other cases.
        default:
            APP_ERROR_HANDLER(event);            
            break;
    }
}


/**@brief Open a transport channel.
 * 
 * @retval NRF_SUCCESS        Operation success.
 * @retval NRF_ERROR_INTERNAL Operation failure. Internal error ocurred. 
 */
static __INLINE uint32_t channel_open(void)
{
    uint32_t err_code;
    
    err_code = app_transport_open(app_transport_event_handle);
    switch (err_code)
    {
        case NRF_SUCCESS:
        case NRF_ERROR_INTERNAL:
            // No interface conversion action needed as supported by the interface.
            break;
            
        case NRF_ERROR_NULL:
            // Interface conversion action needed as not supported by the interface.
            err_code = NRF_ERROR_INTERNAL;
            break;
            
        default:
            // No implementation needed. 
            break;
    }    
    
    return err_code;    
}


/**@brief Gets the current state of the channel.
 *
 * @return true if the channel is currently open.
 */
static __INLINE bool is_channel_open(void)
{
    return (m_channel_open_count != 0);
}


/**@brief Increment channel open reference count.
 */
static __INLINE void channel_count_increment(void)
{    
    ++m_channel_open_count;
}


uint32_t rpc_transport_open(uint32_t * p_channel_open_count)
{    
    uint32_t err_code;

    if (!is_channel_open())
    {
        // Channel is not open, try to open it.
        err_code = channel_open();
        if (err_code == NRF_SUCCESS)
        {
            m_channel_open_count = 1u;
        }
    }
    else
    {
        // Channel is allready open, increment the reference count.
        err_code = NRF_SUCCESS;
        channel_count_increment();
    }

    *p_channel_open_count = m_channel_open_count;
    
    return err_code;
}


/**@brief Decrement channel open reference count if channel is open.
 *
 * Internal state is reset when open reference count reaches 0. 
 */
static void channel_count_decrement(void)
{    
    if (is_channel_open())
    {
        --m_channel_open_count;
        if (m_channel_open_count == 0)
        {           
            m_is_rx_data_available   = false;
            m_rx_packet_size         = 0;
            m_rx_state               = RX_IDLE;
            memset(m_rpc_transport_event_handler, 0, sizeof(m_rpc_transport_event_handler));
        }
    }    
}


uint32_t rpc_transport_close(uint32_t * p_channel_open_count)
{   
    uint32_t return_value;
    
    channel_count_decrement();
    *p_channel_open_count = m_channel_open_count;
    
    return_value = (m_channel_open_count == 0) ? app_transport_close() : NRF_SUCCESS;
    
    return return_value;
}


uint32_t rpc_transport_register(rpc_transport_pkt_type_t      pkt_type,
                                rpc_transport_event_handler_t event_handler)
{
    m_rpc_transport_event_handler[pkt_type] = event_handler;
    
    return NRF_SUCCESS;
}


uint32_t rpc_transport_packet_write(rpc_transport_pkt_type_t pkt_type,
                                    const uint8_t *          p_buffer,
                                    uint32_t                 length)
{        
    uint32_t err_code;
    
    if (p_buffer == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (length > (RPC_TX_BUF_SIZE - RPC_HDR_SIZE))
    {
        return NRF_ERROR_DATA_SIZE;
    }
    
    if (is_channel_open())
    {
        // @todo make this implementation more robust.
        
        uint32_t *     p_offset;        
        static uint8_t tx_buffer[RPC_TX_BUF_SIZE] = {NULL};        
        
        // Copy payload after header.
        memcpy(&(tx_buffer[RPC_HDR_SIZE]), p_buffer, length);  

        // Set the header.                        
        length       += RPC_PACKET_TYPE_FIELD_SIZE;
        p_offset      = (uint32_t *)tx_buffer; 
        *p_offset     = length;
        *(++p_offset) = pkt_type; 
                        
        err_code = app_transport_write(tx_buffer, (length + RPC_LENGTH_FIELD_SIZE));            
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    
    return err_code;
}


uint32_t rpc_transport_packet_read(rpc_transport_pkt_type_t pkt_type,
                                   uint32_t                 length,
                                   uint8_t *                p_buffer,
                                   uint32_t *               p_read_length)
{
    uint32_t err_code; 
    
    if ((p_buffer == NULL) || (p_read_length == NULL))
    {
        return NRF_ERROR_NULL;
    }
    if (!is_channel_open())
    {    
        return NRF_ERROR_INVALID_STATE;
    }
    if (!m_is_rx_data_available)
    {
        return NRF_ERROR_NOT_FOUND;
    }    
    if (length < (m_rx_packet_size - RPC_HDR_SIZE))
    {
        return NRF_ERROR_DATA_SIZE;
    }    

    // @note Current system design defines exact state for packet extraction.
    if (m_rx_state == RX_W4_EXTRACT)
    {
        err_code       = NRF_SUCCESS;
        *p_read_length = m_rx_packet_size - RPC_HDR_SIZE;
        // @note m_rx_buffer begins with packet type field, 
        // thus offset is needed to begin of actual data.
        memcpy(p_buffer, &(m_rx_buffer[RPC_PACKET_TYPE_FIELD_SIZE]), *p_read_length); 
                
        rx_sm_state_change(RX_IDLE);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    
    return err_code;
}
