/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

/** @file
 *
 * @defgroup ble_sdk_alert_notification_main main.c
 * @{
 * @ingroup ble_sdk_app_alert_notification
 * @brief Alert Notification Client Sample Application main file.
 *
 * This file contains the source code for a sample application using the Alert Notification Profile Client.
 * This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "ble_ans_c.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_advdata.h"
#include "ble_error_log.h"
#include "nrf_gpio.h"
#include "ble_stack_handler.h"
#include "ble_srv_common.h"
#include "ble_conn_params.h"
#include "ble_nrf6310_pins.h"
#include "nrf51_bitfields.h"
#include "ble_bondmngr.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "app_timer.h"
#include "ble_radio_notification.h"
#include "ble_flash.h"
#include "ble_debug_assert_handler.h"


#define WAKEUP_BUTTON_PIN               NRF6310_BUTTON_0                                     /**< Button used to wake up the application. */
#define BONDMNGR_DELETE_BUTTON_PIN_NO   NRF6310_BUTTON_1                                     /**< Button used for deleting all bonded masters/services during startup. */
#define CLEAR_NEW_ALERT_BUTTON_PIN      NRF6310_BUTTON_2                                     /**< Button used to clear New Alert status/Enable/Disable New Alert Notifications. */
#define CLEAR_UNREAD_ALERT_BUTTON_PIN   NRF6310_BUTTON_3                                     /**< Button used to clear Unread Alert status/Enable/Disable New Alert Notifications. */
#define NOTIFY_ALL_ALERTS_BUTTON_PIN    NRF6310_BUTTON_4                                     /**< Button used to request peer to notify all current alert statuses. */

#define NEW_ALERT_NOTIF_LED_PIN_NO      NRF6310_LED_2                                        /**< Is on when a New Alert Notification has been received. */
#define UNREAD_ALERT_NOTIF_LED_PIN_NO   NRF6310_LED_3                                        /**< Is on when a Unread Alert Notification has been received. */

#define DEVICE_NAME                     "Nordic_Alert_Notif."                                /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                                /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                40                                                   /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_INTERVAL_SLOW           3200                                                 /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                                  /**< The advertising timeout in units of seconds. */
#define ADV_INTERVAL_FAST_PERIOD        30                                                   /**< The duration of the fast advertising period (in seconds). */

#define APP_TIMER_PRESCALER             0                                                    /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                                    /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                                    /**< Size of timer operation queues. */

#define SECOND_1_25_MS_UNITS            800                                                  /**< Definition of 1 second, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS              100                                                  /**< Definition of 1 second, when 1 unit is 10 ms. */
#define MIN_CONN_INTERVAL               (SECOND_1_25_MS_UNITS / 2)                           /**< Minimum acceptable connection interval (0.5 seconds), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               (SECOND_1_25_MS_UNITS)                               /**< Maximum acceptable connection interval (1 second), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT                (4 * SECOND_10_MS_UNITS)                             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20 * 1000, APP_TIMER_PRESCALER)      /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5 * 1000, APP_TIMER_PRESCALER)       /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                                    /**< Number of attempts before giving up the connection parameter negotiation. */

#define MESSAGE_BUFFER_SIZE             18                                                   /**< Size of buffer holding optional messages in notifications. */

#define APP_GPIOTE_MAX_USERS            1                                                    /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)             /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                                   /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                                    /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                                    /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                                 /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                                    /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                                    /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                                   /**< Maximum encryption key size. */

#define FLASH_PAGE_SYS_ATTR             (NRF_FICR->CODESIZE-3)                               /**< Flash page used for bond manager system attribute information. */
#define FLASH_PAGE_BOND                 (NRF_FICR->CODESIZE-1)                               /**< Flash page used for bond manager bonding information. */
#define ANS_FLASH_PAGE                  (NRF_FICR->CODESIZE-4)                               /**< Flash page used for Alert Notification Client. */

#define DEAD_BEEF                       0xDEADBEEF                                           /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

typedef enum
{
    BLE_NO_ADVERTISING,                                                                      /**< No advertising running. */
    BLE_SLOW_ADVERTISING,                                                                    /**< Slow advertising running. */
    BLE_FAST_ADVERTISING                                                                     /**< Fast advertising running. */
} ble_advertising_mode_t;

typedef enum
{
    ALERT_NOTIFICATION_DISABLED,                                                             /**< Alert Notifications has been disabled. */
    ALERT_NOTIFICATION_ENABLED,                                                              /**< Alert Notifications has been enabled. */
    ALERT_NOTIFICATION_ON,                                                                   /**< Alert State is on. */
} ble_ans_c_alert_state_t;

static ble_ans_c_t                      m_ans_c;                                             /**< Structure used to identify the Alert Notification Service Client. */
static uint8_t                          m_alert_message_buffer[MESSAGE_BUFFER_SIZE];         /**< Message buffer for optional notify messages. */

static ble_gap_adv_params_t             m_adv_params;                                        /**< Parameters to be passed to the stack when starting advertising. */
static ble_advertising_mode_t           m_advertising_mode;                                  /**< Variable to keep track of when we are advertising. */

static ble_ans_c_alert_state_t          m_new_alert_state    = ALERT_NOTIFICATION_DISABLED;  /**< State that holds the current state of New Alert Notifications, i.e. Enabled, Alert On, Disabled. */
static ble_ans_c_alert_state_t          m_unread_alert_state = ALERT_NOTIFICATION_DISABLED;  /**< State that holds the current state of Unread Alert Notifications, i.e. Enabled, Alert On, Disabled. */

static ble_gap_sec_params_t             m_sec_params;                                        /**< Security requirements for this application. */


/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
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
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset
    NVIC_SystemReset();
}


/**@brief Assert macro callback function.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Setup of alert notifications in master.
 *
 * @details This function will be called when a successful connection has been established.
 */
static void setup_alert_notification(void)
{
    uint32_t err_code;

    err_code = ble_ans_c_enable_notif_new_alert(&m_ans_c);
    APP_ERROR_CHECK(err_code);
    m_new_alert_state = ALERT_NOTIFICATION_ENABLED;

    err_code = ble_ans_c_enable_notif_unread_alert(&m_ans_c);
    APP_ERROR_CHECK(err_code);
    m_unread_alert_state = ALERT_NOTIFICATION_ENABLED;
}


/**@brief Setup of alert notifications in master.
 *
 * @details This function will be called when supported alert notification and
 *          supported unread alert notifications has been fetched.
 *
 * @param[in]   p_evt   Event containing the response with supported alert types.
 */
static void setup_control_point(ble_ans_c_evt_t * p_evt)
{
    uint32_t                err_code;
    ble_ans_control_point_t setting;

    if (p_evt->uuid.uuid == BLE_UUID_SUPPORTED_UNREAD_ALERT_CATEGORY_CHAR)
    {
        if (p_evt->data.settings.ans_notification_call_support)
        {
            setting.command  = ANS_ENABLE_UNREAD_CATEGORY_STATUS_NOTIFICATION;
            setting.category = ANS_TYPE_NOTIFICATION_CALL;
        }
        else if (p_evt->data.settings.ans_missed_call_support)
        {
            setting.command  = ANS_ENABLE_UNREAD_CATEGORY_STATUS_NOTIFICATION;
            setting.category = ANS_TYPE_MISSED_CALL;
        }
        else
        {
            // Don't configure the control point if the above alerts types are not supported.
            return;
        }
    }
    else
    {
        return;
    }

    err_code = ble_ans_c_control_point_write(&m_ans_c, &setting);
    APP_ERROR_CHECK(err_code);
}


/**@brief Read supported alert notifications in master.
 *
 * @details This function will be called when a connection has been established.
 */
static void read_supported_alert_notification(void)
{
    uint32_t err_code;

    err_code = ble_ans_c_new_alert_read(&m_ans_c);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ans_c_unread_alert_read(&m_ans_c);
    APP_ERROR_CHECK(err_code);
}


/**@brief Handle the key of the New Alert Notification button.
 *
 * @details This function check the current state of new alert and then, enable, disable or clear
 *          the alert depending on the state.
 */
static void handle_key_press_new_alert_button(void)
{
    uint32_t err_code = NRF_SUCCESS;

    if (m_new_alert_state == ALERT_NOTIFICATION_ON)
    {
        m_new_alert_state = ALERT_NOTIFICATION_ENABLED;
        nrf_gpio_pin_clear(NEW_ALERT_NOTIF_LED_PIN_NO);
    }
    else if (m_new_alert_state == ALERT_NOTIFICATION_ENABLED)
    {
        m_new_alert_state = ALERT_NOTIFICATION_DISABLED;
        err_code = ble_ans_c_disable_notif_new_alert(&m_ans_c);
    }
    else
    {
        m_new_alert_state = ALERT_NOTIFICATION_ENABLED;
        err_code = ble_ans_c_enable_notif_new_alert(&m_ans_c);
    }

    // If the user presses the button while we are not connected,
    // we will have NRF_ERROR_INVALID_STATE, thus we just ignore the error code.
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Handle the key of the Unread Alert button.
 *
 * @details This function check the current state of unread alert and then, enable, disable or
 *          clear the alert depending on the state.
 */
static void handle_key_press_unread_alert_button(void)
{
    uint32_t err_code = NRF_SUCCESS;

    if (m_unread_alert_state == ALERT_NOTIFICATION_ON)
    {
        m_unread_alert_state = ALERT_NOTIFICATION_ENABLED;
        nrf_gpio_pin_clear(UNREAD_ALERT_NOTIF_LED_PIN_NO);
    }
    else if (m_unread_alert_state == ALERT_NOTIFICATION_ENABLED)
    {
        m_unread_alert_state = ALERT_NOTIFICATION_DISABLED;
        err_code = ble_ans_c_disable_notif_unread_alert(&m_ans_c);
    }
    else
    {
        m_unread_alert_state = ALERT_NOTIFICATION_ENABLED;
        err_code = ble_ans_c_enable_notif_unread_alert(&m_ans_c);
    }

    // If the user presses the button while we are not connected,
    // we will have NRF_ERROR_INVALID_STATE, thus we just ignore the error code.
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Handle key presses of the All Alert Notify button.
 *
 * @details This function check the current state of the alert notifications and based on the state
 *          it will request the master to resend current alert counters.
 */
static void all_alert_notify_request(void)
{
    uint32_t err_code = NRF_SUCCESS;

    if (m_unread_alert_state == ALERT_NOTIFICATION_ON  ||
        m_unread_alert_state == ALERT_NOTIFICATION_ENABLED)
    {
        err_code = ble_ans_c_unread_alert_notify(&m_ans_c, ANS_TYPE_ALL_ALERTS);
        if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_HANDLER(err_code);
        }
    }

    if (m_new_alert_state == ALERT_NOTIFICATION_ON  ||
        m_new_alert_state == ALERT_NOTIFICATION_ENABLED)
    {
        err_code = ble_ans_c_new_alert_notify(&m_ans_c, ANS_TYPE_ALL_ALERTS);
        if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}


/**@brief This function lights up the LED corresponding to the notification received.
 *
 * @param[in]   p_evt   Event containing the notification.
 */
static void handle_alert_notification(ble_ans_c_evt_t * p_evt)
{
    if (p_evt->uuid.uuid == BLE_UUID_UNREAD_ALERT_CHAR)
    {
        if (m_unread_alert_state == ALERT_NOTIFICATION_ENABLED)
        {
            nrf_gpio_pin_set(UNREAD_ALERT_NOTIF_LED_PIN_NO);
            m_unread_alert_state = ALERT_NOTIFICATION_ON;
        }
    }
    else if (p_evt->uuid.uuid == BLE_UUID_NEW_ALERT_CHAR)
    {
        if (m_new_alert_state == ALERT_NOTIFICATION_ENABLED)
        {
            nrf_gpio_pin_set(NEW_ALERT_NOTIF_LED_PIN_NO);
            m_new_alert_state = ALERT_NOTIFICATION_ON;
        }
    }
}


/**@brief Alert Notification Service Client handler.
 *
 * @details This function will be called for all events in the Alert Notification Client which
 *          are passed to the application.
 *
 * @param[in]   p_evt   Event received from the Alert Notification Service Client.
 */
static void on_ans_c_evt(ble_ans_c_evt_t * p_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_evt->evt_type)
    {
        case BLE_ANS_C_EVT_NOTIFICATION:
            handle_alert_notification(p_evt);
            break;

        case BLE_ANS_C_EVT_DISCOVER_COMPLETE:
            read_supported_alert_notification();
            setup_alert_notification();

            // Start handling button presses
            break;

        case BLE_ANS_C_EVT_RECONNECT:
            break;

        case BLE_ANS_C_EVT_READ_RESP:
            setup_control_point(p_evt);
            break;

        case BLE_ANS_C_EVT_DISCONN_COMPLETE:
            m_new_alert_state    = ALERT_NOTIFICATION_DISABLED;
            m_unread_alert_state = ALERT_NOTIFICATION_DISABLED;

            nrf_gpio_pin_clear(NEW_ALERT_NOTIF_LED_PIN_NO);
            nrf_gpio_pin_clear(UNREAD_ALERT_NOTIF_LED_PIN_NO);
            break;

        // The BLE_ANS_C_EVT_DISCOVER_FAILED will occur if a master, not supporting the Alert
        // Notification Service, connects to the Alert Notification Client.
        case BLE_ANS_C_EVT_DISCOVER_FAILED:
        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Bond Manager handler.
 *
 * @details This function will be called for all events in the Bond Manager which
 *          are passed to the application.
 *
 * @param[in]   p_evt   Event received from the Bond Manager.
 */
static void on_bond_mgmr_evt(ble_bondmngr_evt_t * p_evt)
{
    ble_ans_c_on_bondmgmr_evt(&m_ans_c, p_evt);
}


/**@brief LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    GPIO_LED_CONFIG(ADVERTISING_LED_PIN_NO);
    GPIO_LED_CONFIG(CONNECTED_LED_PIN_NO);
    GPIO_LED_CONFIG(ASSERT_LED_PIN_NO);
    GPIO_LED_CONFIG(NEW_ALERT_NOTIF_LED_PIN_NO);
    GPIO_LED_CONFIG(UNREAD_ALERT_NOTIF_LED_PIN_NO);
}


/**@brief Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
}


/**@brief GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    
    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = 0;
    advdata.uuids_complete.p_uuids  = NULL;
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    if (m_advertising_mode == BLE_NO_ADVERTISING)
    {
        m_advertising_mode = BLE_FAST_ADVERTISING;
    }
    else
    {
        m_advertising_mode = BLE_SLOW_ADVERTISING;
    }

    // Initialize advertising parameters (used when starting advertising)
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                           // Undirected advertisement
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;

    if (m_advertising_mode == BLE_FAST_ADVERTISING)
    {
        m_adv_params.interval = APP_ADV_INTERVAL;
        m_adv_params.timeout  = ADV_INTERVAL_FAST_PERIOD;
    }
    else
    {
        m_adv_params.interval = APP_ADV_INTERVAL_SLOW;
        m_adv_params.timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
    }

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
    
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief Alert Notification Service Client module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void alert_notification_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Initialization of the Alert Notification Service Client.
 */
static void alert_notification_init(void)
{
    uint32_t         err_code;
    ble_ans_c_init_t ans_init_obj;
    bool             services_delete;

    memset(&ans_init_obj, 0, sizeof(ans_init_obj));
    memset(m_alert_message_buffer, 0, MESSAGE_BUFFER_SIZE);

    ans_init_obj.evt_handler            = on_ans_c_evt;
    ans_init_obj.message_buffer_size    = MESSAGE_BUFFER_SIZE;
    ans_init_obj.p_message_buffer       = m_alert_message_buffer;
    ans_init_obj.flash_page_num         = ANS_FLASH_PAGE;
    ans_init_obj.error_handler          = alert_notification_error_handler;

    err_code = ble_ans_c_init(&m_ans_c, &ans_init_obj);
    APP_ERROR_CHECK(err_code);

    // Clear all discovered and stored services if the "non-connectable advertisement start"
    // button is pushed
    err_code = app_button_is_pushed(BONDMNGR_DELETE_BUTTON_PIN_NO, &services_delete);
    APP_ERROR_CHECK(err_code);

    if (services_delete)
    {
        err_code = ble_ans_c_service_delete();
        APP_ERROR_CHECK(err_code);
    }

    err_code = ble_ans_c_service_load(&m_ans_c);
    APP_ERROR_CHECK(err_code);
}


/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Bond Manager module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void bond_manager_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Bond Manager initialization.
 */
static void bond_manager_init(void)
{
    uint32_t            err_code;
    ble_bondmngr_init_t bond_init_data;
    bool                bonds_delete;

    // Clear all bonded masters if the Bonds Delete button is pushed
    err_code = app_button_is_pushed(BONDMNGR_DELETE_BUTTON_PIN_NO, &bonds_delete);
    APP_ERROR_CHECK(err_code);

    // Initialize the Bond Manager
    bond_init_data.flash_page_num_bond     = FLASH_PAGE_BOND;
    bond_init_data.flash_page_num_sys_attr = FLASH_PAGE_SYS_ATTR;
    bond_init_data.evt_handler             = on_bond_mgmr_evt;
    bond_init_data.error_handler           = bond_manager_error_handler;
    bond_init_data.bonds_delete            = bonds_delete;
    
    err_code = ble_bondmngr_init(&bond_init_data);
    APP_ERROR_CHECK(err_code);
}


/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t        err_code = NRF_SUCCESS;
    static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            err_code = app_button_enable();
            m_advertising_mode = BLE_NO_ADVERTISING;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Since we are not in a connection and have not start to advertise either, store bonds.
            err_code = ble_bondmngr_bonded_masters_store();
            APP_ERROR_CHECK(err_code);

            // Stop detecting button presses when not connected
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);

            // Stop detecting button presses when not connected
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);

            err_code = ble_ans_c_service_store();

            advertising_start();
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            { 
                if (m_advertising_mode == BLE_FAST_ADVERTISING)
                {
                    advertising_start();
                }
                else
                {
                    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
                    m_advertising_mode = BLE_NO_ADVERTISING;

                    // Go to system-off mode (function will not return; wakeup will cause a reset)
                    GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
								  	GPIO_WAKEUP_BUTTON_CONFIG(BONDMNGR_DELETE_BUTTON_PIN_NO);
                    err_code = sd_power_system_off();
                }
            }
            break;
            
        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Button event handler.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */
static void button_event_handler(uint8_t pin_no)
{
    switch (pin_no)
    {
        case CLEAR_NEW_ALERT_BUTTON_PIN:
            handle_key_press_new_alert_button();
            break;
            
        case CLEAR_UNREAD_ALERT_BUTTON_PIN:
            handle_key_press_unread_alert_button();
            break;
            
        case NOTIFY_ALL_ALERTS_BUTTON_PIN:
            all_alert_notify_request();
            break;
            
        default:
            APP_ERROR_HANDLER(pin_no);
            break;
        }
    }


/**@brief Initialize GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Initialize button handler module.
 */
static void buttons_init(void)
{
    // Configure the button used to send alert to the peer. Configure it as wake up button too.
    // Configure Buttons. Buttons are used for:
    // - Clearing of Alerts
    // - Configuration of Alerts (CCCD)
    // - Wake-up application
    static app_button_cfg_t buttons[] =
    {
        {WAKEUP_BUTTON_PIN,             false, NRF_GPIO_PIN_NOPULL, NULL},
        {BONDMNGR_DELETE_BUTTON_PIN_NO, false, NRF_GPIO_PIN_NOPULL, NULL},
        {CLEAR_NEW_ALERT_BUTTON_PIN,    false, NRF_GPIO_PIN_NOPULL, button_event_handler},
        {CLEAR_UNREAD_ALERT_BUTTON_PIN, false, NRF_GPIO_PIN_NOPULL, button_event_handler},
        {NOTIFY_ALL_ALERTS_BUTTON_PIN,  false, NRF_GPIO_PIN_NOPULL, button_event_handler}
    };
    
    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, false);
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
    ble_bondmngr_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_ans_c_on_ble_evt(&m_ans_c, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief BLE stack initialization.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           false);
}


/**@brief Initialize Radio Notification event handler.
 */
static void radio_notification_init(void)
{
    uint32_t err_code;

    err_code = ble_radio_notification_init(NRF_APP_PRIORITY_HIGH,
                                           NRF_RADIO_NOTIFICATION_DISTANCE_4560US,
                                           ble_flash_on_radio_active_evt);
    APP_ERROR_CHECK(err_code);
}


/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    // Initialize
    leds_init();
    timers_init();
    gpiote_init();
    buttons_init();
    bond_manager_init();
    ble_stack_init();
    ble_error_log_init();
    gap_params_init();
    advertising_init();
    alert_notification_init();
    conn_params_init();
    sec_params_init();
    radio_notification_init();
    
    // Start execution
    advertising_start();

    // Enter main loop
    for (;;)
    {
        power_manage();
    }

}

/** 
 * @}
 */

