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
* @brief Example project on GPIO usage to drive LEDs
* @defgroup gpio_example_pca10001 GPIO example
* @{
* @ingroup nrf_examples_pca10001
*
* @brief Example of GPIO output usage.
*
* This example shows how to configure GPIO pins as outputs which can also be used to drive LEDs.
* Each LED is set on one at a time and each state lasts 100 milliseconds.
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  uint8_t output_state = 0;

  // Configure LED-pins as outputs
  nrf_gpio_range_cfg_output(LED_START, LED_STOP);

  while(true)
  {
    nrf_gpio_port_write(LED_PORT, 1 << (output_state + LED_OFFSET));
    output_state = (output_state + 1) & BLINKY_STATE_MASK;
    nrf_delay_ms(100);
  }
}

/**
 *@}
 **/
