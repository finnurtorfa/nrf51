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
* @brief TWI- Two Wire Interface Master example.
*
* @defgroup twi_master_example TWI Master example
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief TWI Master example.
*
* Transfer of commands to initialize sensor registers and commands to read registers are done over TWI bus using TWI driver.
*
* @note By default this example uses the bit-banged software TWI driver but since 
* the API is exactly the same for the hardware TWI driver, the twi_sw_master.c 
* can be swapped out for the twi_hw_master.c to use on-chip hardware to achieve the same results.
*
* Temperature sensor ds1624:
*          This example initializes this sensor @ref ds1624_init. After initialization succeeded then we read the temperature from the sensor register @ref ds1624_temp_read.
*          Read value is written to gpio pins 14 to 8, where pin number 8 has the lsb value of temperature.
* Touchpad sensor:
*          This example initializes this sensor @ref touchpad_init. After initialization succeeded then we read the button status from sensor register @ref touchpad_read_regs.
*          Read value is written to gpio pin number 15 which can be connected to LED which lights up when a button is pressed on the touch sensor which is read through TWI bus.
*/

#include <stdbool.h>
#include <stdint.h>

#include "ds1624.h"
#include "twi_master.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "synaptics_touchpad.h"

#define DS1624_ADDRESS 0x07 //!< Bits [2:0] describing how pins A2, A1 and A0 are wired
#define TOUCHPAD_ADDRESS 0x20 //!< Touchpad TWI address in bits [6:0]

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard. 
 */
int main(void)
{
  bool touchpad_init_succeeded;
  bool ds1624_init_succeeded;
  bool m_conversion_in_progress = false;

  nrf_gpio_port_dir_set(NRF_GPIO_PORT_SELECT_PORT1, NRF_GPIO_PORT_DIR_OUTPUT);

  if (!twi_master_init())
  {
    nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0x55);
    while (1) {}
  }

  touchpad_init_succeeded = touchpad_init(TOUCHPAD_ADDRESS);
  ds1624_init_succeeded = ds1624_init(DS1624_ADDRESS);

  // If both failed to initialized, halt here
  if (!touchpad_init_succeeded && !ds1624_init_succeeded)
  {
    nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0x5F);
    while (1) {}
  }

  while(true)
  {
    if (ds1624_init_succeeded)
    {
      if (!m_conversion_in_progress) 
      {
        m_conversion_in_progress = ds1624_start_temp_conversion();
      }
      else
      {
        // Succeeded
        if (ds1624_is_temp_conversion_done())
        {
          m_conversion_in_progress = false;

          int8_t temperature;
          int8_t temperature_fraction;
      
          if (ds1624_temp_read(&temperature, &temperature_fraction))
          {
            NRF_GPIO->OUTCLR = 0x00007F00UL;
            NRF_GPIO->OUTSET = ((uint8_t)temperature << 8);
          }
        }
      }
    }

    if (touchpad_init_succeeded)
    {
      // Read button status
      uint8_t touchpad_button_status;
      if (touchpad_read_register(TOUCHPAD_BUTTON_STATUS, &touchpad_button_status))
      {
        // There's a active low button on the side of the touchpad, check the state and light up a LED if it's pressed
        NRF_GPIO->OUTCLR = 0x00008000UL;
        if (!(touchpad_button_status & 0x01))
        {
          NRF_GPIO->OUTSET = 0x00008000UL;
        }
      }
    }
  }
}

/**
 *@}
 **/
