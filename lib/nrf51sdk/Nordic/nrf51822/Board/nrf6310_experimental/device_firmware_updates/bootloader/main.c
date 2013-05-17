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

/** @file
 *
 * @defgroup ble_sdk_app_bootloader_main main.c
 * @{
 * @ingroup ble_sdk_app_bootloader
 * @brief Bootloader project main file.
 *
 * 1) Receive start data package.
 * 2) Based on start packet, prepare NVM area to store received data.
 * 3) Receive data packet
 * 4) Validate data packet
 * 5) Write Data packet to NVM
 * 6) If not finished - Wait for next packet
 * 7) Receive stop data packet.
 * 8) Activate Image, boot application.
 *
 */
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "nrf51.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_nrf6310_pins.h"
#include "app_scheduler.h"
#include "app_util.h"
#include "ble_stack_handler.h"
#include "app_timer.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "nrf_sdm.h"
#include "nrf_error.h"
#include "rpc_transport.h"


#define NRF_UICR_BOOT_START_ADDRESS     ((uint32_t *)(NRF_UICR_BASE + 0x14))                    /**< Register where the bootloader start address is stored in the UICR register. */

#define BOOTLOADER_BUTTON_PIN           NRF6310_BUTTON_7                                        /**< Button used to set in SW update mode. */
#define APP_GPIOTE_MAX_USERS            1

#define APP_TIMER_PRESCALER             0                                                       /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                                       /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                                       /**< Size of timer operation queues. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)                /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define PACKET_SIZE                     512                                                     /**< Size of each data packet. Also used for initial receiving of packets from transport layer. */
#define PACKET_HEADER_SIZE              sizeof(uint32_t)                                        /**< Size of the data packet header. */
#define CODE_REGION_1_START             0x00014000                                              /**< This field should correspond to the size of Code Region 0, (Which is identical to Start of Code Region 1), found in UICR.CLEN0 register. This value is used for compile safety, as the linker will fail if application expands into bootloader. Runtime, the bootloader will use the value found in UICR.CLEN0. */
#define BOOTLOADER_REGION_START         0x00038000                                              /**< This field should correspond to start address of the bootloader, found in UICR.RESERVED, 0x10001014, register. This value is used for sanity check, so the bootloader will fail immediately if this value differs from runtime value. The value is used to determine max application size for updating. */
#define BOOTLOADER_SETTINGS_ADDRESS     0x0003EC00                                              /**< The field specifies the page location of the bootloader settings address. */
#define APP_IMAGE_MAX_SIZE              BOOTLOADER_REGION_START - CODE_REGION_1_START           /**< Maximum size of data image we current support for update. */
#define CODE_PAGE_SIZE                  1024                                                    /**< Size of a flash codepage. Used for size of the reserved flash space in the bootloader region. Will be runtime checked against NRF_UICR->CODEPAGESIZE to ensure the region is correct. */
#define EMPTY_FLASH_MASK                0xFFFFFFFF                                              /**< Bit mask that defines an empty address in flash. */

#define INVALID_BANK                    0xFF                                                    /**< Invalid bank identifier. */
#define BANK_0                          0x1                                                     /**< Bank 0 identifier. */
#define BANK_1                          0x2                                                     /**< Bank 1 identifier. */
#define VALID_BANK                      0x1                                                     /**< Valid bank identifier. */

#define INVALID_PACKET                  0x00                                                    /**< Invalid packet identifies. */
#define INIT_PACKET                     0x01                                                    /**< Packet identifies for initialization packet. */
#define START_DATA_PACKET               0x02                                                    /**< Packet identifies for the Data Start Packet. */
#define DATA_PACKET                     0x03                                                    /**< Packet identifies for a Data Packet. */
#define STOP_DATA_PACKET                0x04                                                    /**< Packet identifies for the Data Stop Packet. */

/**@brief Structure holding bootloader settings for application and bank data.
 */
typedef struct
{
    uint8_t     current_bank;                                                                   /**< Current bank used for booting a valid application, @note only Bank 0 is supported. */
    uint8_t     bank_0;                                                                         /**< Variable to store if bank 0 contains a valid application. */
    uint8_t     bank_1;                                                                         /**< Variable to store if bank 1 contains a valid application, @note Not in use, as only Bank 0 is supported. */
    uint32_t    bank_0_address;                                                                 /**< Address where bank 0 starts, identical to start of Code Region 1 / NRF_UICR->CLENR0. */
    uint32_t    bank_1_address;                                                                 /**< Address where bank 1 starts, @note Not in use, as only Bank 0 is supported. */
} bootloader_settings_t;

/**@brief Structure holding a bootloader packet received on the UART.
 */
typedef struct
{
    volatile uint32_t   packet_type;                                                            /**< Packet type, used to identify the content of the received packet referenced by data packet. */
    volatile uint32_t   packet_length;                                                          /**< Packet length of the data packet. Each data is word size, meaning length of 4 is 4 words, not bytes. */
    volatile uint32_t * data_packet;                                                            /**< Data Packet received. Each data is a word size entry. */
} bootloader_update_packet_t;

static bootloader_update_packet_t   m_data_packet;                                              /**< Bootloader data packet used when processing data from the UART. */
static volatile uint32_t            m_receive_packet[PACKET_HEADER_SIZE / sizeof(uint32_t) +
                                                     PACKET_SIZE / sizeof(uint32_t)];           /**< Receive buffer provided to the transport layer for receiving data packets. */
static uint32_t                   * m_app_write_address;                                        /**< Pointer to the address in flash to write next word of data received. */

static bool                         m_bootloader_init_valid = true;                             /**< Field identifying if a valid data packet has been received. */
static bool                         m_flash_prepared        = false;                            /**< Field identifying if the flash has been prepared (erased) for new application. */

uint8_t m_app_image[APP_IMAGE_MAX_SIZE] __attribute__((at(CODE_REGION_1_START)));               /**< This variable defines the location and max supported size for the image to flash during update. */
uint8_t m_boot_settings[CODE_PAGE_SIZE] __attribute__((at(BOOTLOADER_SETTINGS_ADDRESS)));       /**< This variable reserves a codepage for bootloader specific settings, to ensure the compiler doesn't locate any code or variables at his location. */

const bootloader_settings_t * mp_ro_settings = (bootloader_settings_t *) &m_boot_settings[0];   /**< Read only pointer to bootloader settings in flash. */

uint32_t m_nrf_start_address __asm__("NRF_APP_START_ADDRESS") = CODE_REGION_1_START; /*lint -esym(552,m_nrf_start_address) "Symbol not accessed"  */            /**< Default start address, unless bank 1 is used. @note Bank 1 is currently not supported, all applications must be executed from bank 0. */

void StartApplication(void); /*lint -esym(526,StartApplication) "Symbol not defined"  */        /**< Function in assembly which will launch the application. */


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
    nrf_gpio_pin_set(NRF6310_LED_7);
    nrf_gpio_pin_set(NRF6310_LED_6);
    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);
    for (;;) ;
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
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


static void sd_assertion_handler(uint32_t pc, uint16_t line_num, const uint8_t * file_name)
{
    UNUSED_PARAMETER(pc);
    assert_nrf_callback(line_num, file_name);
}


/**@brief LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    GPIO_LED_CONFIG(NRF6310_LED_0);
    GPIO_LED_CONFIG(NRF6310_LED_1);
    GPIO_LED_CONFIG(NRF6310_LED_7);
}


/**@brief LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_off(void)
{
    nrf_gpio_pin_clear(NRF6310_LED_0);
    nrf_gpio_pin_clear(NRF6310_LED_1);
    nrf_gpio_pin_clear(NRF6310_LED_7);
}


/**@brief Initialize GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
}



/**@brief Initialize button handler module.
 */
static void buttons_init(void)
{
    // Note: Array must be static because a pointer to it will be saved in the Button handler
    //       module.
    static app_button_cfg_t buttons[] =
    {
        {BOOTLOADER_BUTTON_PIN, false, NRF_GPIO_PIN_NOPULL, NULL},
    };

    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, true);
}


static void erase_pages(uint32_t start_page, uint32_t no_of_pages)
{
    uint32_t i;
    uint32_t page_size = NRF_FICR->CODEPAGESIZE;

    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Erase pages.
    for (i = 0; i < no_of_pages; i++)
    {
        NRF_NVMC->ERASEPAGE = start_page;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
        {
            // Do nothing.
        }
        start_page += page_size;
    }

    // Turn off flash erase enable and wait until the NVMC is ready.
    NRF_NVMC->CONFIG &= ~(NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing
    }
}


static void write_data(uint32_t * p_start_address, uint32_t * p_data, uint32_t length)
{
    uint32_t index = 0;
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    for (; index < length;)
    {
        *p_start_address = p_data[index];
        ++p_start_address;
        ++index;
    }

    // Turn off flash write enable and wait until the NVMC is ready.
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing
    }
}


static void start_packet_handle(void)
{
    uint32_t image_size;
    uint32_t no_of_pages;
    uint32_t page_size = NRF_FICR->CODEPAGESIZE;

    image_size = uint32_decode((uint8_t *)m_data_packet.data_packet);

    no_of_pages = (image_size / page_size) + 1;

    erase_pages(NRF_UICR->CLENR0, no_of_pages);

    m_app_write_address = (uint32_t *)NRF_UICR->CLENR0;
    m_flash_prepared    = true;
}


static void stop_packet_handle(void)
{
    uint32_t              no_of_pages = 1;
    uint32_t *            page = (uint32_t *) mp_ro_settings;

    bootloader_settings_t settings = *mp_ro_settings;           // Copy existing settings to local variable, to ensure they survive after NVM erase.

    settings.current_bank          = BANK_0;                    // Everything has been written in bank 0.
    settings.bank_0                = VALID_BANK;                // Mark the bank valid.
    settings.bank_0_address        = NRF_UICR->CLENR0;          // Address of bank 0 is identical to code region 1.

    erase_pages((uint32_t) page, no_of_pages);

    write_data(page, (uint32_t *) &settings, sizeof(bootloader_settings_t) / sizeof(uint32_t));
}


static void data_packet_handle(void)
{
    if (m_bootloader_init_valid && m_flash_prepared)
    {
        // Here we should copy the packet to NVM.
        // 1) If m_bootloader_data_page_index == 0,
        //    - fetch UICR.CLENR0 and compare to CODE_REGION_1_START.
        //    - Erase flash page
        // 2)
        uint32_t * data  = (uint32_t *) m_data_packet.data_packet;
        uint32_t   data_length = m_data_packet.packet_length;

        write_data(m_app_write_address, data, m_data_packet.packet_length);
        m_app_write_address += data_length;
    }
    else
    {
        // Ignore, received a data packet without a valid init file or flash has not been prepared.
    }
}


/**@brief This function will block in a loop, using WFE to allow low power mode, while awaiting a
 *        response from the connectivity chip.
 *
 * @param[in] op_code   The Operation Code for which a response message is expected.
 *
 * @return    The decoded error code received from the connectivity chip.
 */
static void wait_for_packet(void)
{
    for(;;)
    {
        __WFE();

        switch (m_data_packet.packet_type)
        {
            case DATA_PACKET:
                data_packet_handle();
                break;

            case START_DATA_PACKET:
                start_packet_handle();
                break;

            case STOP_DATA_PACKET:
                stop_packet_handle();
                return;

            case INIT_PACKET:
                // Validate init packet.
                // We expect to receive the init packet in two rounds of 512 bytes.
                // If that fails, we abort, and boot the application.
                // @note Current proto-type doesn't handle an init packet.
                m_bootloader_init_valid   = true;
                break;

            default:
                break;
        }
        m_data_packet.packet_type = INVALID_PACKET;
    }
}


static void rpc_transport_event_handler(rpc_transport_evt_type_t event)
{
    uint32_t err_code;
    uint32_t index = 0;
    uint32_t length;
    
    // We only read a new packet from transport layer if the current packet has been processed
    // and the m_data_packet has been marked as invalid.
    if (m_data_packet.packet_type == INVALID_PACKET)
    {
        err_code = rpc_transport_packet_read(RPC_TRANSPORT_BOOTLOADER,
                                             sizeof(m_receive_packet),
                                             (uint8_t *) m_receive_packet,
                                             &length);
        APP_ERROR_CHECK(err_code);

        if (length < PACKET_HEADER_SIZE)
        {
            // @todo length is shorter than header, meaning invalid packet.
            // Implement error handling.
            return;
        }

        m_data_packet.packet_type   = m_receive_packet[index++];

        m_data_packet.data_packet   = &m_receive_packet[index];
        m_data_packet.packet_length = (length / sizeof(uint32_t)) - index;
    }
}


static uint32_t start_update(void)
{
    uint32_t err_code;
    uint32_t open_conn;

    // Open transport layer.
    err_code = rpc_transport_open(&open_conn);
    APP_ERROR_CHECK(err_code);

    // Register callback to be run when commands have been recieved by the transport layer.
    err_code = rpc_transport_register(RPC_TRANSPORT_BOOTLOADER,
                                      rpc_transport_event_handler);
    APP_ERROR_CHECK(err_code);

    wait_for_packet();

    return NRF_SUCCESS;
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool is_pushed = false;

    leds_init();

    // This checking ensure that the defined fields in the bootloader corresponds with actual
    // setting ins the nRF51 chip.
    APP_ERROR_CHECK_BOOL(NRF_UICR->CLENR0 == CODE_REGION_1_START);
    APP_ERROR_CHECK_BOOL(*(NRF_UICR_BOOT_START_ADDRESS) == BOOTLOADER_REGION_START);
    APP_ERROR_CHECK_BOOL(NRF_FICR->CODEPAGESIZE == CODE_PAGE_SIZE);

    // Initialize
    timers_init();
    gpiote_init();
    buttons_init();

    nrf_gpio_pin_set(NRF6310_LED_0);
    (void) app_button_is_pushed(BOOTLOADER_BUTTON_PIN, &is_pushed);

    if (is_pushed ||
        (mp_ro_settings->current_bank == INVALID_BANK) ||
        (*((uint32_t *)NRF_UICR->CLENR0) == EMPTY_FLASH_MASK))
    {
        err_code = sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, sd_assertion_handler);
        APP_ERROR_CHECK(err_code);

        nrf_gpio_pin_set(NRF6310_LED_1);

        err_code = start_update();
        APP_ERROR_CHECK(err_code);

        nrf_gpio_pin_clear(NRF6310_LED_1);

        err_code = sd_softdevice_disable();
        APP_ERROR_CHECK(err_code);
    }

    if (mp_ro_settings->current_bank == BANK_0)
    {
        // If the applications CRC has been checked and passed, the magic number will be written and we
        // can star the application safely.
        m_nrf_start_address = mp_ro_settings->bank_0_address;
        (void)sd_softdevice_forward_to_application();
        leds_off();
        StartApplication();
    }
/*
    else
    {
        // If the applications CRC has been checked and passed, the magic number will be written and we
        // can star the application safely.
        m_nrf_start_address = m_settings.bank_0_address;
        //m_nrf_start_address = settings.bank_1_address; // Bank 1 not supported yet.
        (void)sd_softdevice_forward_to_application();
        leds_off();
        StartApplication();
    }
*/               
    for (;;) ;
}



/** 
 * @}
 */
