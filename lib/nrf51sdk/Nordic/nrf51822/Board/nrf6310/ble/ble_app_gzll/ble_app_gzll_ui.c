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

#include "ble_app_gzll_ui.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "app_util.h"
#include "app_button.h"
#include "app_timer.h"
#include "ble_app_gzll_common.h"


#define BUTTON_DETECTION_DELAY  APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */


void leds_init(void)
{
    GPIO_LED_CONFIG(ADVERTISING_LED_PIN_NO);
    GPIO_LED_CONFIG(CONNECTED_LED_PIN_NO);
    GPIO_LED_CONFIG(ASSERT_LED_PIN_NO);
    GPIO_LED_CONFIG(GZLL_TX_SUCCESS_LED_PIN_NO);
    GPIO_LED_CONFIG(GZLL_TX_FAIL_LED_PIN_NO);
}


/**@brief Button event handler.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */
static void button_event_handler(uint8_t pin_no)
{
    switch (pin_no)
    {
        case BLE_BUTTON_PIN_NO:
            running_mode = BLE;
            break;
            
        case GZLL_BUTTON_PIN_NO:
            running_mode = GAZELL;
            break;
            
        default:
            APP_ERROR_HANDLER(pin_no);
    }
}


/**@brief Initialize button handler module.
 */
void buttons_init(void)
{
    uint32_t err_code;
    
    // Configure buttons
    static app_button_cfg_t buttons[] =
    {
        {BLE_BUTTON_PIN_NO,  false, NRF_GPIO_PIN_NOPULL, button_event_handler},
        {GZLL_BUTTON_PIN_NO, false, NRF_GPIO_PIN_NOPULL, button_event_handler}
    };
    
    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);

    // Start handling button presses immediately
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}
