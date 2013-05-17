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
 
#include "app_transport.h"
#include <stdlib.h>
#include "app_uart.h"
#include "boards.h"
#include "nrf51_bitfields.h"
#include "app_error.h"
 
#define UART_TX_BUF_SIZE 1024u                                  /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1024u                                  /**< UART RX buffer size. */

static app_transport_event_handler_t m_event_callback = NULL;   /**< Event callback function. */


/**@brief app_uart event handler callback function.
 *
 * Forwards an event after appropriate interface conversion.
 *
 * @param[in] p_app_uart_event The type of event that has occurred.
 */
void uart_event_handle(app_uart_evt_t * p_app_uart_event)
{
    if (m_event_callback != NULL)
    {
        if (p_app_uart_event->evt_type == APP_UART_DATA_READY)
        {
            m_event_callback(APP_TRANSPORT_RX_DATA_READY);
        }
        else
        {
            uint32_t err_code;
            
            err_code = app_uart_flush();            
            APP_ERROR_CHECK(err_code);
#if 0            
            m_event_callback(APP_TRANSPORT_ERROR);
#endif //            
        }    
    }
}

 
uint32_t app_transport_open(app_transport_event_handler_t event_handler)
{    
    uint32_t err_code;
        
    if (event_handler != NULL)
    {                          
        // Configure and make UART ready for usage. Flow contrl enabled.
        app_uart_comm_params_t comm_params =  
        {
            RX_PIN_NUMBER, 
            TX_PIN_NUMBER, 
            RTS_PIN_NUMBER, 
            CTS_PIN_NUMBER, 
            APP_UART_FLOW_CONTROL_DISABLED, 
            false, 
            UART_BAUDRATE_BAUDRATE_Baud38400
        }; 
        
        // @note:  err_code variable is set by the macro below. 
        APP_UART_INIT(&comm_params, 
                      UART_RX_BUF_SIZE, 
                      UART_TX_BUF_SIZE, 
                      uart_event_handle, 
                      APP_IRQ_PRIORITY_HIGH,
                      err_code);

        if (err_code != NRF_SUCCESS)                    
        {
            // Adjust peripheral specific error code to interface supported error code.
            err_code = NRF_ERROR_INTERNAL;
        }
        
        m_event_callback = event_handler;                      
    }
    else
    {
        err_code = NRF_ERROR_NULL; 
    }
        
    return err_code;
}


uint32_t app_transport_close(void)
{
    m_event_callback = NULL;
    
    return NRF_SUCCESS;
}


uint32_t app_transport_write(const uint8_t * p_buffer, uint32_t length)
{
    uint32_t i;
    
    if (p_buffer == NULL)
    {
        return NRF_ERROR_NULL;
    }        
    
    // Write to the buffer byte-by-byte.
    i = 0;
    while (i != length)
    {
        if (app_uart_put(p_buffer[i]) != NRF_SUCCESS)
        {
            return NRF_ERROR_INTERNAL;
        }
        
        ++i;
    }   

    return NRF_SUCCESS;
}


uint32_t app_transport_read(uint8_t *  p_buffer, 
                            uint32_t   length, 
                            uint32_t * p_actual_length)
{
    uint32_t i;
    
    if ((p_buffer == NULL) || (p_actual_length == NULL))
    {
        return NRF_ERROR_NULL;
    }
    
    // Try to read to the buffer byte-by-byte.
    i = 0;
    while (i != length)
    {
        if (app_uart_get(&(p_buffer[i])) != NRF_SUCCESS)
        {
            // Update read size upon exit; not for each iteration.
            *p_actual_length = i;
            return NRF_ERROR_DATA_SIZE;
        }
        
        ++i;        
    }               
    
    // Update read size upon exit; not for each iteration.
    *p_actual_length = i;
    return NRF_SUCCESS;    
}
