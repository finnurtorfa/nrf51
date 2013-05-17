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
 * @brief Pin change interrupt example
 * @defgroup pin_change_int_example Pin change usage example
 * @{
 * @ingroup nrf_examples_nrf6310
 *
 * @brief Pin change interrupt example
 *
 * This example demonstrates interrupt on pin change on pin 0. Pin8 is configured
 * as output and toggled in the pin change interrupt handler.
 *
 * @image html example_board_setup_a.png "Use board setup A for this example."
 */

#include <stdbool.h>
#include "nrf.h"
#include "nrf_gpio.h"

/**
 * Configures pin 0 for input and pin 8 for output and
 * configures GPIOTE to give interrupt on pin change.
 */
static void gpio_init(void)
{
  *(uint32_t *)0x40000504 = 0xC007FFDF; // Workaround for PAN_028 rev1.1 anomaly 23 - System: Manual setup is required to enable use of peripherals

  nrf_gpio_cfg_input(0, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_output(8);

  nrf_gpio_pin_write(8, 0);

  // Enable interrupt:
  NVIC_EnableIRQ(GPIOTE_IRQn);
  NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)
              | (0 << GPIOTE_CONFIG_PSEL_Pos)
              | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;
}

/** GPIOTE interrupt handler.
 * Triggered on pin 0 change
 */
void GPIOTE_IRQHandler(void)
{
  // Event causing the interrupt must be cleared
  if ((NRF_GPIOTE->EVENTS_IN[0] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk))
  {
    NRF_GPIOTE->EVENTS_IN[0] = 0;
  }
  nrf_gpio_pin_toggle(8);
}

/**
 * main function
 * \return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  gpio_init();
  while (true)
  {
  }
}

/**
 *@}
 **/
