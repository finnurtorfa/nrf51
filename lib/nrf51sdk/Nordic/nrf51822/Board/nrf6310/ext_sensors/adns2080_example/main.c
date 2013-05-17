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
* @brief Example project on ADNS2080 mouse sensor driver
* @defgroup adns2080_example ADNS2080 mouse sensor example
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief Example project on ADNS2080 mouse sensor driver
*
* This example demostrates how to use ADNS2080 mouse sensor (and driver) to read
* mouse movement. When mouse is moved left, four leftmost LEDs are lit and when mouse
* is moved right, four rightmost LEDs are lit.
*
* ADNS2080 clock line is connected to pin 24, data line to pin 25 and motion interrupt to pin 26.
* Note that on nRF2752 board, you need to to put the P4 jumpers to position closer to the center
* of the board in order to have the GPIO pin 26 routed to the chip.
*/

#include <stdbool.h>
#include <stdint.h>

#include "adns2080.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"

/* File sdio_config.h contains pin configuration for SDIO clock and data. It is included by sdio.c. */

#define MOTION_INTERRUPT_PIN_NUMBER (26) /*!< Pin number to used for ADNS2080 motion interrupt. If you change this, also remember to change the pin configuration in the main function. */
#define MOUSE_MOVEMENT_THRESHOLD (10) /*!< Set the deadzone for mouse movement before LEDs are lit */

static int16_t m_delta_x = 0; //!< Variable to store mouse X-axis movement deltas
static int16_t m_delta_y = 0; //!< Variable to store mouse Y-axis movement deltas

static bool volatile motion_interrupt_detected = false; //!< If set, motion interrupt has occurred. Clear after reading.

/** Initializes GPIO Tasks/Events peripheral.
*/
static void gpiote_init(void)
{
  *(uint32_t *)0x40000504 = 0xC007FFDF; // Workaround for PAN_028 rev1.1 anomaly 23 - System: Manual setup is required to enable use of peripherals

  // Configure GPIOTE channel 0 to generate event when MOTION_INTERRUPT_PIN_NUMBER goes from Low to High
  nrf_gpiote_event_config(0, MOTION_INTERRUPT_PIN_NUMBER, NRF_GPIOTE_POLARITY_LOTOHI);

  // Enable interrupt for NRF_GPIOTE->EVENTS_IN[0] event
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;
}

/** GPIOTE interrupt handler.
* Triggered on motion interrupt pin input low-to-high transition.
*/
void GPIOTE_IRQHandler(void)
{
  motion_interrupt_detected = true;

  // Event causing the interrupt must be cleared
  NRF_GPIOTE->EVENTS_IN[0] = 0;
}

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  // Configure pins 8-15 (Port1) as outputs
  nrf_gpio_range_cfg_output(8, 15);
  nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT1, 0XFF);

  // Configure motion interrupt pin
  nrf_gpio_cfg_input(MOTION_INTERRUPT_PIN_NUMBER, NRF_GPIO_PIN_PULLDOWN);

  gpiote_init();

  if (adns2080_init() != ADNS2080_OK)
  {
    // ADNS2080 init failed, set rightmost LED on
    nrf_gpio_pin_write(15, 1);
    while (true)
    {
    }
  }

  // By default, adns2080 motion interrupt output is active low, edge sensitive; make it active high
  if (adns2080_motion_interrupt_set(ADNS2080_MOTION_OUTPUT_POLARITY_HIGH, ADNS2080_MOTION_OUTPUT_SENSITIVITY_LEVEL) != ADNS2080_OK)
  {
    nrf_gpio_pin_write(14, 1);
    while (true)
    {
    }
  }

  // Read out movement to clear adns2080 interrupt flags
  if (adns2080_is_motion_detected())
  {
    int16_t dummy;
    adns2080_movement_read(&dummy, &dummy);
  }

  // Enable GPIOTE interrupt in Nested Vector Interrupt Controller
  NVIC_EnableIRQ(GPIOTE_IRQn);

  // Enable global interrupts
  __enable_irq();

  while(true)
  {
    if (motion_interrupt_detected)
    {
      // Toggle pin 12 to indicate that we've detected motion interrupt
      nrf_gpio_pin_toggle(12);
      
      motion_interrupt_detected = false;

      // On our Nordic reference design PCB, the chip orientation is not the same as in the ADNS2080 
      // datasheet diagram, so X and Y axis are reversed. This is corrected by passing the pointer
      // parameters in reversed order. */
      adns2080_movement_read(&m_delta_y, &m_delta_x);

      // If movement delta_x is above the threshold light up LEDs accordingly. So when mouse is moved
      // left, 4 leftmost LEDs are lit and when mouse is moved right, 4 rightmost LEDs are lit
      if (m_delta_x > MOUSE_MOVEMENT_THRESHOLD)
      {
        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0xF0);
      }
      else if (m_delta_x < (-MOUSE_MOVEMENT_THRESHOLD))
      {
        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0x0F);
      }
      else
      {
        nrf_gpio_port_clear(NRF_GPIO_PORT_SELECT_PORT1,  0xFF);
      }
    }
  }
}

/**
 *@}
 **/
