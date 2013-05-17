/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 *
 * @defgroup ble_sdk_app_adv_serialized_main main.c
 * @{
 * @ingroup ble_sdk_app_adv_serialization
 * @brief Advertising Sample Application main file.
 *
 * This is a simple application for demonstrating how to set up and initiate advertising.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_nrf6310_pins.h"
#include "ble_stack_handler.h"
#include "app_gpiote.h"
#include "app_error.h"
#include "ble_debug_assert_handler.h"

#include "nrf_delay.h"

#define DEVICE_NAME                   "BLE Connectivity"          /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL              64                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS    180                         /**< The advertising timeout (in units of seconds). */

#define SECOND_1_25_MS_UNITS          800                         /**< Definition of 1 second, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS            100                         /**< Definition of 1 second, when 1 unit is 10 ms. */
#define MIN_CONN_INTERVAL             (SECOND_1_25_MS_UNITS / 2)  /**< Minimum acceptable connection interval (0.5 seconds), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL             (SECOND_1_25_MS_UNITS)      /**< Maximum acceptable connection interval (1 second), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                 0                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT              (4 * SECOND_10_MS_UNITS)    /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define DEAD_BEEF                     0xDEADBEEF                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define MAIN_FUNCTION_LED_PIN_NO      NRF6310_LED_2               /**< LED indicating that the program has entered main function. */

#define CONN_CHIP_RESET_PIN_NO        30                          /**< Pin used for reseting the connectivity chip. */
#define CONN_CHIP_RESET_TIME          50                          /**< The time to keep the reset line to the connectivity chip low (in milliseconds). */
#define CONN_CHIP_WAKEUP_TIME         500                         /**< The time for connectivity chip to reset and become ready to receive serialized commands (in milliseconds). */

static volatile bool        m_start_adv_flag;
static ble_gap_adv_params_t m_adv_params;


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


void assert_nrf_callback(uint16_t line_num, const uint8_t * file_name)
{
    app_error_handler(DEAD_BEEF, line_num, file_name);
}


/**@brief LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    // Initialize debug pins
    GPIO_LED_CONFIG(ADVERTISING_LED_PIN_NO);
    GPIO_LED_CONFIG(CONNECTED_LED_PIN_NO);
    GPIO_LED_CONFIG(MAIN_FUNCTION_LED_PIN_NO);
    GPIO_LED_CONFIG(ASSERT_LED_PIN_NO);
}


/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            m_start_adv_flag = true;
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            break;
            
        default:
            break;
    }
}


static void bluetooth_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gppcp;
    ble_advdata_t           advdata;
    ble_gap_conn_sec_mode_t sec_mode;
    uint8_t                 flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    
    // Initialise SoftDevice, and enable BLE event interrupt.
    BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           false);
    
    // Set GAP parameters
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_TAG);
    APP_ERROR_CHECK(err_code);
    
    memset(&gppcp, 0, sizeof(gppcp));
    
    gppcp.min_conn_interval = MIN_CONN_INTERVAL;
    gppcp.max_conn_interval = MAX_CONN_INTERVAL;
    gppcp.slave_latency     = SLAVE_LATENCY;
    gppcp.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    
    err_code = sd_ble_gap_ppcp_set(&gppcp);
    APP_ERROR_CHECK(err_code);
    
    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags.size         = sizeof(flags);
    advdata.flags.p_data       = &flags;
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
    
    // Initialise advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                           // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
    
    // Start advertising when entering main loop.
    m_start_adv_flag = true;
}


/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
}


static void connectivity_chip_reset(void)
{
    GPIO_PIN_CONFIG(CONN_CHIP_RESET_PIN_NO,                       
                    GPIO_PIN_CNF_DIR_Output,      
                    GPIO_PIN_CNF_INPUT_Disconnect,
                    GPIO_PIN_CNF_PULL_Pullup,   
                    GPIO_PIN_CNF_DRIVE_S0S1,      
                    GPIO_PIN_CNF_SENSE_Disabled);
    
    // Signal a reset to the connectivity chip by setting the 
    // reset pin on the connectivity chip low.
    nrf_gpio_pin_clear(CONN_CHIP_RESET_PIN_NO);
    nrf_delay_ms(CONN_CHIP_RESET_TIME);
    
    // Set the reset level to high again.
    nrf_gpio_pin_set(CONN_CHIP_RESET_PIN_NO);
    
    // Wait for connectivity chip to be ready.
    nrf_delay_ms(CONN_CHIP_WAKEUP_TIME);
}


int main(void)
{
    // If transport layer is using UART with flow control we need to get one 
    // GPIOTE user for the CTS wakeup pin.
    APP_GPIOTE_INIT(1); 

    leds_init();
    
    // Indicate that the application has started up. This can be used to verify that 
    // application resets if reset button is pressed or the chip has been reset with 
    // a debugger. The led will be off during reset.
    nrf_gpio_pin_set(MAIN_FUNCTION_LED_PIN_NO);
    
    connectivity_chip_reset();
    
    bluetooth_init();
   
    // Enter main loop.
    for (;;)
    {
        uint32_t err_code;
        
        // Start/restart advertising.
        if (m_start_adv_flag)
        {
            err_code = sd_ble_gap_adv_start(&m_adv_params);
            APP_ERROR_CHECK(err_code);
            nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
            m_start_adv_flag = false;
        }
        
        // Power management.
        power_manage();
    }
}
/*
@}
*/
