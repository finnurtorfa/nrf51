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
 * @defgroup ble_sdk_app_hids_mouse_main main.c
 * @{
 * @ingroup ble_sdk_app_hids_mouse
 * @brief HID Mouse Sample Application main file.
 *
 * This file contains is the source code for a sample application using the HID, Battery and Device
 * Information Service for implementing a simple mouse functionality. This application uses the
 * @ref app_scheduler.
 *
 * Also it would accept pairing requests from any peer device. This implementation of the
 * application will not know whether a connected master is a known device or not.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "ble_nrf6310_pins.h"
#include "ble_sensorsim.h"
#include "app_scheduler.h"
#include "ble_stack_handler.h"
#include "app_timer.h"
#include "ble_bondmngr.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_radio_notification.h"
#include "ble_flash.h"
#include "ble_debug_assert_handler.h"


#define LEFT_BUTTON_PIN_NO              NRF6310_BUTTON_0                            /**< Button used for moving the mouse pointer to the left. */
#define RIGHT_BUTTON_PIN_NO             NRF6310_BUTTON_1                            /**< Button used for moving the mouse pointer to the right. */
#define UP_BUTTON_PIN_NO                NRF6310_BUTTON_2                            /**< Button used for moving the mouse pointer upwards. */
#define DOWN_BUTTON_PIN_NO              NRF6310_BUTTON_3                            /**< Button used for moving the mouse pointer downwards. */
#define BONDMNGR_DELETE_BUTTON_PIN_NO   RIGHT_BUTTON_PIN_NO                         /**< Button used for deleting all bonded masters during startup. */

#define ADV_DIRECTED_LED_PIN_NO         NRF6310_LED_4                               /**< Is on when we are doing directed advertisement. */
#define ADV_WHITELIST_LED_PIN_NO        NRF6310_LED_5                               /**< Is on when we are doing advertising with whitelist. */
#define ADV_INTERVAL_SLOW_LED_PIN_NO    NRF6310_LED_6                               /**< Is on when we are doing slow advertising. */

#define DEVICE_NAME                     "Nordic_Mouse"                              /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            3                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)  /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Increment between each simulated battery level measurement. */

#define APP_ADV_INTERVAL_FAST           0x0028                                      /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_INTERVAL_SLOW           0x0C80                                      /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_FAST_ADV_TIMEOUT            30                                          /**< The duration of the fast advertising period (in seconds). */
#define APP_SLOW_ADV_TIMEOUT            180                                         /**< The duration of the slow advertising period (in seconds). */
#define APP_DIRECTED_ADV_TIMEOUT        5                                           /**< Number of direct advertisement (each lasting 1.28seconds). */

#define PNP_ID_VENDOR_ID_SOURCE         0x02                                        /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                0x1915                                      /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID               0xEEEE                                      /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION          0x0001                                      /**< Product Version. */

#define MIN_CONN_INTERVAL               6                                           /**< Minimum connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               12                                          /**< Maximum connection interval (15 ms). */
#define SLAVE_LATENCY                   25                                         /**< Slave latency. */
#define CONN_SUP_TIMEOUT                (3 * 100)                                   /**< Connection supervisory timeout (3 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAM_UPDATE_COUNT     3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define MOVEMENT_SPEED                  5                                           /**< Number of pixels by which the cursor is moved each time a button is pushed. */
#define INPUT_REPORT_COUNT              3                                           /**< Number of input reports in this application. */
#define INPUT_REP_BUTTONS_LEN           3                                           /**< Length of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_LEN          3                                           /**< Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1                                           /**< Length of Mouse Input Report containing media player data. */
#define INPUT_REP_BUTTONS_INDEX         0                                           /**< Index of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_INDEX        1                                           /**< Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MPLAYER_INDEX         2                                           /**< Index of Mouse Input Report containing media player data. */
#define INPUT_REP_REF_BUTTONS_ID        1                                           /**< Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_MOVEMENT_ID       2                                           /**< Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MPLAYER_ID        3                                           /**< Id of reference to Mouse Input Report containing media player data. */

#define FLASH_PAGE_SYS_ATTR             (NRF_FICR->CODESIZE-3)                      /**< Flash page used for bond manager system attribute information. */
#define FLASH_PAGE_BOND                 (NRF_FICR->CODESIZE-1)                      /**< Flash page used for bond manager bonding information. */

#define BASE_USB_HID_SPEC_VERSION       0x0101                                      /**< Version number of base USB HID Specification implemented by this application. */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE,\
                                            BLE_STACK_HANDLER_SCHED_EVT_SIZE)       /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
    
typedef enum    
{   
    BLE_NO_ADV,                                                                     /**< No advertising running. */
    BLE_DIRECTED_ADV,                                                               /**< Direct advertising to the latest master. */
    BLE_FAST_ADV_WHITELIST,                                                         /**< Advertising whith whitelist. */
    BLE_FAST_ADV,                                                                   /**< Fast advertising running. */
    BLE_SLOW_ADV,                                                                   /**< Slow advertising running. */
    BLE_SLEEP                                                                       /**< Go to system-off. */
} ble_advertising_mode_t;   
    
static ble_hids_t                       m_hids;                                     /**< Structure used to identify the HID service. */
static ble_bas_t                        m_bas;                                      /**< Structure used to identify the battery service. */
static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static bool                             m_in_boot_mode = false;                     /**< Current protocol mode. */
    
static ble_sensorsim_cfg_t              m_battery_sim_cfg;                          /**< Battery Level sensor simulator configuration. */
static ble_sensorsim_state_t            m_battery_sim_state;                        /**< Battery Level sensor simulator state. */
    
static app_timer_id_t                   m_battery_timer_id;                         /**< Battery timer. */
    
static ble_advertising_mode_t           m_advertising_mode;                         /**< Variable to keep track of when we are advertising. */
static int8_t                           m_last_connected_master;                    /**< BondManager reference handle to the last connected master. */ 
static uint8_t                          m_direct_adv_cnt;                           /**< Counter of direct advertisements. */


static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);


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


/**@brief Service error handler.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the 
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Perform battery measurement, and update Battery Level characteristic in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;
    
    battery_level = (uint8_t)ble_sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
    
    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if (
        (err_code != NRF_SUCCESS) 
        && 
        (err_code != NRF_ERROR_INVALID_STATE) 
        && 
        (err_code != BLE_ERROR_NO_TX_BUFFERS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Battery measurement timer timeout handler.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    GPIO_LED_CONFIG(ADV_WHITELIST_LED_PIN_NO);
    GPIO_LED_CONFIG(ADV_DIRECTED_LED_PIN_NO);
    GPIO_LED_CONFIG(ADV_INTERVAL_SLOW_LED_PIN_NO);
    GPIO_LED_CONFIG(ADVERTISING_LED_PIN_NO);
    GPIO_LED_CONFIG(CONNECTED_LED_PIN_NO);
    GPIO_LED_CONFIG(ASSERT_LED_PIN_NO);
}


/**@brief Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;
   
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

    // Create battery timer
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
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

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_MOUSE);
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
 *
 * @param[in]  adv_flags  Indicates which type of advertisement to use, see @ref BLE_GAP_DISC_MODES.
 */
static void advertising_init(uint8_t adv_flags)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    
    ble_uuid_t adv_uuids[] = {{BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}};

    m_advertising_mode = BLE_NO_ADV;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(adv_flags);
    advdata.flags.p_data            = &adv_flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize Device Information Service.
 */
static void dis_init(void)
{
    uint32_t         err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;
    
    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));
    
    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize Battery Service.
 */
static void bas_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init_obj;
    
    memset(&bas_init_obj, 0, sizeof(bas_init_obj));
    
    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;
    
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_report_read_perm);
    
    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize HID Service.
 */
static void hids_init(void)
{
    uint32_t                  err_code;
    ble_hids_init_t           hids_init_obj;
    ble_hids_inp_rep_init_t   inp_rep_array[INPUT_REPORT_COUNT];
    ble_hids_inp_rep_init_t * p_input_report;
    uint8_t                   hid_info_flags;
    
    static uint8_t rep_map_data[] =
    {
        0x05, 0x01,                     // Usage Page (Generic Desktop)
        0x09, 0x02,                     // Usage (Mouse)

        0xA1, 0x01,                     // Collection (Application)

        // Report ID 1: Mouse buttons + scroll/pan
        0x85, 0x01,                     //     Report Id 1 
        0x09, 0x01,                     //     Usage (Pointer)
        0xA1, 0x00,                     //     Collection (Physical)
        0x95, 0x05,                     //         Report Count (3)
        0x75, 0x01,                     //         Report Size (1)
        0x05, 0x09,                     //         Usage Page (Buttons)
        0x19, 0x01,                     //             Usage Minimum (01)
        0x29, 0x05,                     //             Usage Maximum (05)
        0x15, 0x00,                     //             Logical Minimum (0)
        0x25, 0x01,                     //             Logical Maximum (1)
        0x81, 0x02,                     //             Input (Data, Variable, Absolute)
        0x95, 0x01,                     //             Report Count (1)
        0x75, 0x03,                     //             Report Size (3)
        0x81, 0x01,                     //             Input (Constant) for padding
        0x75, 0x08,                     //             Report Size (8)
        0x95, 0x01,                     //             Report Count (1)
        0x05, 0x01,                     //         Usage Page (Generic Desktop)
        0x09, 0x38,                     //             Usage (Wheel)
        0x15, 0x81,                     //             Logical Minimum (-127)
        0x25, 0x7F,                     //             Logical Maximum (127)
        0x81, 0x06,                     //             Input (Data, Variable, Relative) 
        0x05, 0x0C,                     //         Usage Page (Consumer)
        0x0A, 0x38, 0x02,               //             Usage (AC Pan) 
        0x95, 0x01,                     //             Report Count (1)
        0x81, 0x06,                     //             Input (Data,Value,Relative,Bit Field)
        0xC0,                           //     End Collection (Physical)

        // Report ID 2: Mouse motion
        0x85, 0x02,                     //     Report Id 2 
        0x09, 0x01,                     //     Usage (Pointer)
        0xA1, 0x00,                     //     Collection (Physical)
        0x75, 0x0C,                     //         Report Size (12)
        0x95, 0x02,                     //         Report Count (2)
        0x05, 0x01,                     //         Usage Page (Generic Desktop)
        0x09, 0x30,                     //             Usage (X)                                                  
        0x09, 0x31,                     //             Usage (Y)
        0x16, 0x01, 0xF8,               //             Logical maximum (2047) 
        0x26, 0xFF, 0x07,               //             Logical minimum (-2047) 
        0x81, 0x06,                     //             Input (Data, Variable, Relative) 
        0xC0,                           //     End Collection (Physical) 
        0xC0,                           // End Collection (Application)

        // Report ID 3: Advanced buttons
        0x05, 0x0C,                     // Usage Page (Consumer)
        0x09, 0x01,                     // Usage (Consumer Control) 
        0xA1, 0x01,                     // Collection (Application)
        0x85, 0x03,                     //     Report Id (3) 
        0x15, 0x00,                     //     Logical minimum (0)
        0x25, 0x01,                     //     Logical maximum (1) 
        0x75, 0x01,                     //     Report Size (1)
        0x95, 0x01,                     //     Report Count (1)

        0x09, 0xCD,                     //     Usage (Play/Pause) 
        0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field)
        0x0A, 0x83, 0x01,               //     Usage (AL Consumer Control Configuration) 
        0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field)
        0x09, 0xB5,                     //     Usage (Scan Next Track)
        0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field)   
        0x09, 0xB6,                     //     Usage (Scan Previous Track)
        0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field) 

        0x09, 0xEA,                     //     Usage (Volume Down)
        0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field) 
        0x09, 0xE9,                     //     Usage (Volume Up) 
        0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field) 
        0x0A, 0x25, 0x02,               //     Usage (AC Forward) 
        0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field) 
        0x0A, 0x24, 0x02,               //     Usage (AC Back) 
        0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field) 
        0xC0                            // End Collection
    };

    // Initialize HID Service
    p_input_report                      = &inp_rep_array[INPUT_REP_BUTTONS_INDEX];
    p_input_report->max_len             = INPUT_REP_BUTTONS_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_BUTTONS_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;
    
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_input_report                      = &inp_rep_array[INPUT_REP_MOVEMENT_INDEX];
    p_input_report->max_len             = INPUT_REP_MOVEMENT_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MOVEMENT_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;
    
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_input_report                      = &inp_rep_array[INPUT_REP_MPLAYER_INDEX];
    p_input_report->max_len             = INPUT_REP_MEDIA_PLAYER_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MPLAYER_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;
    
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;
    
    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = false;
    hids_init_obj.is_mouse                       = true;
    hids_init_obj.inp_rep_count                  = INPUT_REPORT_COUNT;
    hids_init_obj.p_inp_rep_array                = inp_rep_array;
    hids_init_obj.outp_rep_count                 = 0;
    hids_init_obj.p_outp_rep_array               = NULL;
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
    hids_init_obj.rep_map.data_len               = sizeof(rep_map_data);
    hids_init_obj.rep_map.p_data                 = rep_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;
    
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_mouse_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_mouse_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_mouse_inp_rep.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize services that will be used by the application.
 */
static void services_init(void)
{
    dis_init();
    bas_init();
    hids_init();
}


/**@brief Initialize the battery sensor simulator.
 */
static void sensor_sim_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;
    
    ble_sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
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
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Start timers.
 */
static void timers_start(void)
{
    uint32_t err_code;
    
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    ble_gap_whitelist_t  whitelist;
    ble_gap_addr_t       peer_address;
    
    // Clear all advertising LEDs
    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
    nrf_gpio_pin_clear(ADV_DIRECTED_LED_PIN_NO);
    nrf_gpio_pin_clear(ADV_WHITELIST_LED_PIN_NO);
    nrf_gpio_pin_clear(ADV_INTERVAL_SLOW_LED_PIN_NO);

    // Initialize advertising parameters with defaults values
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.p_whitelist = NULL;
    
    // Configure advertisement according to current advertising state
    if (m_advertising_mode == BLE_DIRECTED_ADV)
    {
        err_code = ble_bondmngr_master_addr_get(m_last_connected_master, &peer_address);
        if (err_code != NRF_SUCCESS)
        {
            m_advertising_mode = BLE_FAST_ADV_WHITELIST;
        }
    }
    
    switch (m_advertising_mode)
    {
        case BLE_NO_ADV:
            m_advertising_mode = BLE_FAST_ADV_WHITELIST;
            // fall through

        case BLE_FAST_ADV_WHITELIST:
            err_code = ble_bondmngr_whitelist_get(&whitelist);
            APP_ERROR_CHECK(err_code);

            if ((whitelist.addr_count != 0) || (whitelist.irk_count != 0))
            {
                adv_params.fp          = BLE_GAP_ADV_FP_FILTER_CONNREQ;
                adv_params.p_whitelist = &whitelist;
                
                advertising_init(BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
                m_advertising_mode = BLE_FAST_ADV;
                nrf_gpio_pin_set(ADV_WHITELIST_LED_PIN_NO);
            }
            else
            {
                m_advertising_mode = BLE_SLOW_ADV;           
            }
            
            adv_params.interval = APP_ADV_INTERVAL_FAST;
            adv_params.timeout  = APP_FAST_ADV_TIMEOUT;
            break;
            
        case BLE_DIRECTED_ADV:
            adv_params.p_peer_addr = &peer_address;
            adv_params.type        = BLE_GAP_ADV_TYPE_ADV_DIRECT_IND;
            adv_params.timeout     = 0;
            
            m_direct_adv_cnt--;
            if (m_direct_adv_cnt == 0)
            {
                m_advertising_mode = BLE_FAST_ADV_WHITELIST;
            }
            nrf_gpio_pin_set(ADV_DIRECTED_LED_PIN_NO);      
            break;
            
        case BLE_FAST_ADV:
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);
            
            adv_params.interval = APP_ADV_INTERVAL_FAST;
            adv_params.timeout  = APP_FAST_ADV_TIMEOUT;
            m_advertising_mode  = BLE_SLOW_ADV;            
            break;
            
        case BLE_SLOW_ADV:
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);
            
            adv_params.interval = APP_ADV_INTERVAL_SLOW;
            adv_params.timeout  = APP_SLOW_ADV_TIMEOUT;
            m_advertising_mode  = BLE_SLEEP;
            
            nrf_gpio_pin_set(ADV_INTERVAL_SLOW_LED_PIN_NO);
            break;
            
        default:
            break;
    }

    // Start advertising
    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief HID event handler.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service stucture.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;
            
        case BLE_HIDS_EVT_NOTIF_ENABLED:
        {
            if (m_in_boot_mode)
            {
                // Protocol mode is Boot Protocol mode.
                if (
                    p_evt->params.notification.char_id.uuid 
                    == 
                    BLE_UUID_BOOT_MOUSE_INPUT_REPORT_CHAR
                   )
                {
                    // The notification of boot mouse input report has been enabled. 
                    // Save the system attribute (CCCD) information into the flash.
                    uint32_t err_code;
                    
                    err_code = ble_bondmngr_sys_attr_store();
                    if (err_code != NRF_ERROR_INVALID_STATE)
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                    else
                    {
                        // The bondmanager did not write the system attributes to the flash because
                        // the connected master is not a new master. The system attributes 
                        // will only be written to flash only when disconnected from this master.
                        // Do nothing now.
                    }
                }
                else
                {
                    // Do nothing
                }
            }
            else if (
                     (p_evt->params.notification.char_id.rep_index == INPUT_REP_MOVEMENT_INDEX)
                     &&
                     (p_evt->params.notification.char_id.rep_type == BLE_HIDS_REP_TYPE_INPUT)
                    )
            {
                // The protocol mode is Report Protocol mode. And the CCCD for the report ID 
                // INPUT_REP_MOVEMENT_INDEX is changed. Since this application uses this report to 
                // send mouse movements, it is now time to store all the CCCD information (system
                // attributes) into the flash.
                uint32_t err_code;
                
                err_code = ble_bondmngr_sys_attr_store();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                else
                {
                    // The bondmanager did not write the system attributes to the flash because
                    // the connected master is not a new master. The system attributes 
                    // will only be written to flash only when disconnected from this master.
                    // Do nothing now.
                }                
            }
            else
            {
                // The notification of the report that was enabled by the master is not interesting to this 
                // application. So do nothing.
            }
            break;
        }
                    
        default:
            break;
    }
}


/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t        err_code      = NRF_SUCCESS;
    static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            nrf_gpio_pin_clear(ADV_DIRECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADV_WHITELIST_LED_PIN_NO);
            nrf_gpio_pin_clear(ADV_INTERVAL_SLOW_LED_PIN_NO);
            // Start handling button presses
            err_code = app_button_enable();

            m_conn_handle      = p_ble_evt->evt.gap_evt.conn_handle;
            m_advertising_mode = BLE_NO_ADV;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Stop detecting button presses when not connected
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);

            // Since we are not in a connection and have not started advertising, store bonds
            err_code = ble_bondmngr_bonded_masters_store();
            APP_ERROR_CHECK(err_code);

            m_advertising_mode = BLE_DIRECTED_ADV;
            m_direct_adv_cnt   = APP_DIRECTED_ADV_TIMEOUT;
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
                if (m_advertising_mode == BLE_SLEEP)
                {
                    m_advertising_mode = BLE_NO_ADV;
                    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
                    nrf_gpio_pin_clear(ADV_DIRECTED_LED_PIN_NO);
                    nrf_gpio_pin_clear(ADV_WHITELIST_LED_PIN_NO);
                    nrf_gpio_pin_clear(ADV_INTERVAL_SLOW_LED_PIN_NO);
                    
                    // Go to system-off mode
                    // (this function will not return; wakeup will cause a reset)
                    GPIO_WAKEUP_BUTTON_CONFIG(LEFT_BUTTON_PIN_NO);
									  GPIO_WAKEUP_BUTTON_CONFIG(BONDMNGR_DELETE_BUTTON_PIN_NO);
                    err_code = sd_power_system_off();    
                }
                else
                {
                    advertising_start();
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


/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_bondmngr_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_hids_on_ble_evt(&m_hids, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
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
                           true);
}


/**@brief Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function to send Mouse Movement.
 *
 * @param[in]   x_delta   Horizontal movement.
 * @param[in]   y_delta   Vertical movement.
 */
static void mouse_movement_send(int16_t x_delta, int16_t y_delta)
{
    uint32_t err_code;
    
    if (m_in_boot_mode)
    {
        x_delta = MIN(x_delta, 0x00ff);
        y_delta = MIN(y_delta, 0x00ff);
        
        err_code = ble_hids_boot_mouse_inp_rep_send(&m_hids,
                                                    0x00, 
                                                    (int8_t)x_delta, 
                                                    (int8_t)y_delta,
                                                    0,
                                                    NULL);
    }
    else
    {
        uint8_t buffer[INPUT_REP_MOVEMENT_LEN];
        
        APP_ERROR_CHECK_BOOL(INPUT_REP_MOVEMENT_LEN == 3);
        
        x_delta = MIN(x_delta, 0x0fff);
        y_delta = MIN(y_delta, 0x0fff);

        buffer[0] = x_delta & 0x00ff;
        buffer[1] = ((y_delta & 0x000f) << 4) | ((x_delta & 0x0f00) >> 8);
        buffer[2] = (y_delta & 0x0ff0) >> 4;
        
        err_code = ble_hids_inp_rep_send(&m_hids,
                                         INPUT_REP_MOVEMENT_INDEX, 
                                         INPUT_REP_MOVEMENT_LEN, 
                                         buffer);
    }
    
    if (
        (err_code != NRF_SUCCESS)
        &&
        (err_code != NRF_ERROR_INVALID_STATE)
        &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS)
        &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Button event handler.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */
static void button_event_handler(uint8_t pin_no)
{
    switch (pin_no)
    {
        case LEFT_BUTTON_PIN_NO:
            mouse_movement_send(-MOVEMENT_SPEED, 0);
            break;
            
        case RIGHT_BUTTON_PIN_NO:
            mouse_movement_send(MOVEMENT_SPEED, 0);
            break;
            
        case UP_BUTTON_PIN_NO:
            mouse_movement_send(0, -MOVEMENT_SPEED);
            break;
            
        case DOWN_BUTTON_PIN_NO:
            mouse_movement_send(0, MOVEMENT_SPEED);
            break;
            
        default:
            APP_ERROR_HANDLER(pin_no);
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
    static app_button_cfg_t buttons[] =
    {
        {LEFT_BUTTON_PIN_NO,  false, NRF_GPIO_PIN_NOPULL, button_event_handler},
        {RIGHT_BUTTON_PIN_NO, false, NRF_GPIO_PIN_NOPULL, button_event_handler},   // Note: This pin is also BONDMNGR_DELETE_BUTTON_PIN_NO
        {UP_BUTTON_PIN_NO,    false, NRF_GPIO_PIN_NOPULL, button_event_handler},
        {DOWN_BUTTON_PIN_NO,  false, NRF_GPIO_PIN_NOPULL, button_event_handler}
    };
    
    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, true);
}


/**@brief Bond Manager module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void bond_manager_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Bond Manager module event handler.
 *
 * @param[in]   p_evt   Data associated to the bond manager event.
 */
static void bond_evt_handler(ble_bondmngr_evt_t * p_evt)
{
    m_last_connected_master = p_evt->master_handle;
    
    switch (p_evt->evt_type)
    {
        case BLE_BONDMNGR_EVT_ENCRYPTED:
        case BLE_BONDMNGR_EVT_NEW_BOND:
        case BLE_BONDMNGR_EVT_CONN_TO_BONDED_MASTER:
        default:
            break;
    }
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
    bond_init_data.evt_handler             = bond_evt_handler;
    bond_init_data.error_handler           = bond_manager_error_handler;
    bond_init_data.bonds_delete            = bonds_delete;
    
    err_code = ble_bondmngr_init(&bond_init_data);
    APP_ERROR_CHECK(err_code);
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
    scheduler_init();
    gap_params_init();
    advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);
    services_init();
    sensor_sim_init();
    conn_params_init();
    sec_params_init();
    radio_notification_init();
    
    // Start execution
    timers_start();
    advertising_start();
    
    // Enter main loop
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}

/** 
 * @}
 */
