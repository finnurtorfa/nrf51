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

/**@file
 *
 * @defgroup ble_sdk_app_connectivity_main main.c
 * @{
 * @ingroup ble_sdk_app_connectivity
 *
 * @brief BLE Connectivity application.
 */

#include <string.h>
#include <stddef.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf51_bitfields.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_stack_handler.h"
#include "ble_nrf6310_pins.h"
#include "ble_debug_assert_handler.h"
#include "app_error.h"
#include "app_gpiote.h"
#include "rpc_transport.h"
#include "ble_rpc_event_encoder.h"
#include "ble_rpc_cmd_decoder.h"

/** @todo Define in single header file. */
#define RPC_TRANSPORT_PACKET_READ_BUF_SIZE 256u                                           /**< Size of the RPC transport packet read buffer size */

#define SCHED_QUEUE_SIZE                   10u                                            /**< Maximum number of events in the scheduler queue. */
#define SCHED_MAX_EVENT_DATA_SIZE          MAX(sizeof(rpc_transport_evt_type_t),\
                                               BLE_STACK_HANDLER_SCHED_EVT_SIZE)          /**< Maximum size of scheduler events. */

#define DEAD_BEEF                          0xDEADBEEF                                     /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define MAIN_FUNCTION_LED_PIN_NO           NRF6310_LED_2                                  /**< LED indicating that the program has entered main function. */
#define FIRST_CMD_RECEIVED_LED_PIN_NO      NRF6310_LED_3                                  /**< LED indicating the first serialized command that is receieved. */


/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset.
    NVIC_SystemReset();
}


/**@brief Assert macro callback function.
 *
 * @details This function will be called if the ASSERT macro fails.
 *
 * @param[in]   line_num    Line number of the failing ASSERT call.
 * @param[in]   p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Schedule a transport layer event.
 * 
 * @details Transfer the given event to the scheduler so that this could be 
            processed in thread mode.
 * 
 * @param[in] event RPC transport layer event.
 */
static void rpc_evt_schedule(rpc_transport_evt_type_t event)
{
    // Transfer only Commands received from the transport layer. The events 
    // received from the BLE stack will be handled by the ble_stack_handler. 
    if (event == RPC_TRANSPORT_CMD_READY)
    {
        // Indicate the first command received.
        nrf_gpio_pin_set(FIRST_CMD_RECEIVED_LED_PIN_NO);
        
        uint32_t err_code = app_sched_event_put(NULL, 0, rpc_cmd_handle);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    GPIO_LED_CONFIG(MAIN_FUNCTION_LED_PIN_NO);
    GPIO_LED_CONFIG(FIRST_CMD_RECEIVED_LED_PIN_NO);
    GPIO_LED_CONFIG(ASSERT_LED_PIN_NO);
}


/**@brief Application main function.
 */
int main(void)
{
    leds_init();
    
    // Indicate that the application has started up. This can be used to verify that 
    // application resets if reset button is pressed, the chip has been reset with 
    // a debugger, or the remote application has issued a reset. The led will be off 
    // during reset.
    nrf_gpio_pin_set(MAIN_FUNCTION_LED_PIN_NO);
 
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    
    // Initialise SoftDevice, and enable BLE event interrupt.
    BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           rpc_event_encoder_write,
                           true);
    
    uint32_t err_code;
    uint32_t open_conn;
    
    // Configure gpiote with one user for the UART flow control feature. 
    APP_GPIOTE_INIT(1);
    
    // Open transport layer.
    err_code = rpc_transport_open(&open_conn);
    APP_ERROR_CHECK(err_code);
    
    // Register callback to be run when commands have been recieved by the transport layer.
    err_code = rpc_transport_register(RPC_TRANSPORT_CMD,
                                      rpc_evt_schedule);
    APP_ERROR_CHECK(err_code);
    
    // Enter main loop.
    for (;;)
    {   
        app_sched_execute();
        power_manage();   
    }
}
/** @} */
