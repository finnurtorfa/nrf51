/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
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
 * @brief Flash write example
 * @defgroup flashwrite_example Non-volatile memory controller example
 * @{
 * @ingroup nrf_examples_nrf6310
 *
 * @brief Flash write template.
 *
 * This example erases a page in flash and then reads a pattern from pins 0-7,
 * writes it to flash if a new pattern is detected, reads it back and writes
 * it to pins 8-15. Each time a new pattern is detected the address in flash
 * is incremented.
 * @image html example_board_setup_a.png "Use board setup A for this example."
 */

#include <stdbool.h>
#include "nrf.h"
#include "nrf_gpio.h"

/** Configure pins 0-7 as inputs and 8-15 as outputs.
 *
 */
static void init(void)
{
  // Port0 (0-7 as inputs)
  nrf_gpio_range_cfg_input(0, 7, NRF_GPIO_PIN_NOPULL);

  // Port1 (8-15 as outputs)
  nrf_gpio_range_cfg_output(8, 15);
}

/** Erases a page in flash
 *
 * @param page_address Adress of first word in page to be erased
 */
static void flash_page_erase(uint32_t *page_address)
{
  // Turn on flash erase enable and wait until the NVMC is ready:
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
  }
  // Erase page:
  NRF_NVMC->ERASEPAGE = (uint32_t)page_address;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
  }
  // Turn off flash erase enable and wait until the NVMC is ready:
  NRF_NVMC->CONFIG &= ~(NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
  }
}

/** Fills a page in flash with a value
 *
 * @param address Adress of first word in page to be filled
 * @param value Value to write to flash
 */
static void flash_word_write(uint32_t *address, uint32_t value)
{
  // Turn on flash write enable and wait until the NVMC is ready:
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
  }
  *address = value;
  // Turn off flash write enable and wait until the NVMC is ready:
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
  }
}

/**
 * main function
 * @return int return type required by ANSI/ISO standard.
 */
int main(void)
{
  uint32_t *addr;
  uint8_t patwr;
  uint8_t patrd;
  uint8_t patold;
  uint32_t i;
  uint32_t pg_size;
  uint32_t pg_num;

  init();
  patold = 0;

  pg_size = NRF_FICR->CODEPAGESIZE;
  pg_num = NRF_FICR->CODESIZE - 1;    // Use last page in flash

  while (true)
  {
    // Start address:
    addr = (uint32_t *)(pg_size * pg_num);
    // Erase page:
    flash_page_erase(addr);
    i = 0;
    do
    {
      // Read pattern from port 0 (pins0-7), and write it to flash:
      patwr =  nrf_gpio_port_read(NRF_GPIO_PORT_SELECT_PORT0);
      if (patold != patwr)
      {
        patold = patwr;
        flash_word_write(++addr, (uint32_t)patwr);
        i++;
      }
      // Read pattern from flash and write it to port 1 (pins8-15):
      patrd = (uint8_t)*addr;
      nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, patrd);
      } while (i < pg_size);
  }
}

/**
 *@}
 **/
