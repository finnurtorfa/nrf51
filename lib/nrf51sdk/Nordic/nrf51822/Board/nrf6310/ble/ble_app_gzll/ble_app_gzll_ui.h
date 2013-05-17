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
 * @defgroup ble_sdk_app_gzll_ui Multiprotocol Application User Interface
 * @{
 * @ingroup ble_sdk_app_gzll
 * @brief User Interface (buttons and LED) handling for the multiprotocol application
 */

#ifndef BLE_APP_GZLL_UI_H__
#define BLE_APP_GZLL_UI_H__

#include <stdbool.h>
#include "ble_nrf6310_pins.h"

#define BLE_BUTTON_PIN_NO              NRF6310_BUTTON_0        /**<  Button used for switching to Bluetooth Heart Rate example. */
#define GZLL_BUTTON_PIN_NO             NRF6310_BUTTON_1        /**<  Button used for switching to Gazell example. */
#define GZLL_TX_SUCCESS_LED_PIN_NO     NRF6310_LED_2           /**<  LED used to show successull Transmits.*/
#define GZLL_TX_FAIL_LED_PIN_NO        NRF6310_LED_3           /**<  LED used to show failed Transmits.*/

/**@brief Initialize GPIOTE module for detecting buttons.
 */
void buttons_init(void);

/**@brief LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
void leds_init(void);

#endif // BLE_APP_GZLL_UI_H__
/** @} */

