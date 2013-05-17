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
 * @defgroup ble_flash_module Flash Manager
 * @{
 * @ingroup ble_sdk_lib
 * @brief Module for accessing flash memory.
 *
 * @details It contains functions for reading, writing and erasing one page in flash.
 *
 *          The module uses the first 32 bits of the flash page to write a magic number in order to
 *          determine if the page has been written or not.
 *
 * @note Be careful not to use a page number in the SoftDevice area (which currently occupies the
 *       range 0 to 127), or in your application space! In both cases, this would end up
 *       with a hard fault.
 */

#ifndef BLE_FLASH_H__
#define BLE_FLASH_H__

#include <stdint.h>
#include <stdbool.h>
#include <nrf51.h>

#define BLE_FLASH_PAGE_SIZE     ((uint16_t)NRF_FICR->CODEPAGESIZE)  /**< Size of one flash page. */
#define BLE_FLASH_MAGIC_NUMBER  0x45DE0000                          /**< Magic value to identify if flash contains valid data. */

/**@brief Erases the specified flash page, and then writes the given data to this page.
 *
 * @warning This operation blocks the CPU. DO NOT use while in a connection!
 *
 * @param[in]  page_num     Page number to update.
 * @param[in]  p_in_array   Pointer to a RAM area containing the elements to write in flash.
 *                          This area has to be 32 bits aligned.
 * @param[in]  word_count   Number of 32 bits words to write in flash.
 *
 * @return     NRF_SUCCESS on successful flash write, otherwise an error code.
 */
uint32_t ble_flash_page_write(uint8_t page_num, uint32_t * p_in_array, uint8_t word_count);

/**@brief Reads data from flash to RAM.
 *
 * @param[in]  page_num       Page number to read.
 * @param[out] p_out_array    Pointer to a RAM area where the found data will be written. 
 *                            This area has to be 32 bits aligned.
 * @param[out] p_word_count   Number of 32 bits words read.
 *
 * @return     NRF_SUCCESS on successful upload, NRF_ERROR_NOT_FOUND if no valid data has been found
 *             in flash (first 32 bits not equal to the MAGIC_NUMBER+CRC).
 */
uint32_t ble_flash_page_read(uint8_t page_num, uint32_t * p_out_array, uint8_t * p_word_count);

/**@brief Erases a flash page.
 *
 * @note This operation blocks the CPU, so it should not be done while the radio is running!
 *
 * @param[in]  page_num   Page number to erase.
 *
 * @return     NRF_SUCCESS on success, an error_code otherwise.
 */
uint32_t ble_flash_page_erase(uint8_t page_num);

/**@brief Write one word to flash.
 *
 * @note Flash location to be written must have been erased previously.
 *
 * @param[in]  p_address   Pointer to flash location to be written.
 * @param[in]  value       Value to write to flash.
 *
 * @return     NRF_SUCCESS.
 */
uint32_t ble_flash_word_write(uint32_t * p_address, uint32_t value);

/**@brief Write a data block to flash.
 *
 * @note Flash locations to be written must have been erased previously.
 *
 * @param[in]  p_address    Pointer to start of flash location to be written.
 * @param[in]  p_in_array   Pointer to start of flash block to be written.
 * @param[in]  word_count   Number of words to be written.
 *
 * @return     NRF_SUCCESS.
 */
uint32_t ble_flash_block_write(uint32_t * p_address, uint32_t * p_in_array, uint8_t word_count);

/**@brief Compute pointer to start of specified flash page.
 *
 * @param[in]  page_num       Page number.
 * @param[out] pp_page_addr   Pointer to start of flash page.
 *
 * @return     NRF_SUCCESS.
 */
uint32_t ble_flash_page_addr(uint8_t page_num, uint32_t ** pp_page_addr);

/**@brief Calculate a 16 bit CRC using the CRC-16-CCITT scheme.
 * 
 * @param[in]  p_data   Pointer to data on which the CRC is to be calulated.
 * @param[in]  size     Number of bytes on which the CRC is to be calulated.
 * @param[in]  p_crc    Initial CRC value (if NULL, a preset value is used as the initial value).
 *
 * @return     Calculated CRC.
 */
uint16_t ble_flash_crc16_compute(uint8_t * p_data, uint16_t size, uint16_t * p_crc);

/**@brief Flash module Radio Notification event handler.
 *
 * @note For flash writing to work safely while in a connection or while advertising, this function
 *       MUST be called from the Radio Notification module's event handler (see
 *       @ref ble_radio_notification for details).
 *
 * @param[in]  radio_active   TRUE if radio is active (or about to become active), FALSE otherwise.
 */
void ble_flash_on_radio_active_evt(bool radio_active);

#endif // BLE_FLASH_H__

/** @} */
