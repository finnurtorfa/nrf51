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
* @brief Random Number Generator
* @defgroup rng_example Random Number Generator example
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief This example generates random numbers from RNG peripheral and shows them on LEDs.
* @image html example_board_setup_a.png "Use board setup A for this example."
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"


/**
 * main function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  // Configure pins 8-15 (port1) for LEDs as outputs
  nrf_gpio_range_cfg_output(8, 15);

  nrf_gpio_port_set(NRF_GPIO_PORT_SELECT_PORT1, 0XFF);

  NRF_RNG->TASKS_START = 1; // start RNG
  while (true)
  {
    // Clear the VALRDY EVENT and clear gpio8 - 15 (port1)
    NRF_RNG->EVENTS_VALRDY = 0;

    // Wait until the value ready event is generated
    while ( NRF_RNG->EVENTS_VALRDY == 0){}

    // Set output according to the random value
    nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)NRF_RNG->VALUE);

    nrf_delay_ms(100);
  }
}


/**
 *@}
 **/
