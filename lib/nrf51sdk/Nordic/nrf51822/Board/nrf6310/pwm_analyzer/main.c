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
* @brief Duty cycle analyzer
* @defgroup pwm_analyzer_example PWM analyzer example
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief This example uses Timer 1 to read the duty cycle from the input pin and
* outputs it on the output port (0-255).
*
*
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"

#define INPUT_PIN_NUMBER (BUTTON0) /*!< Pin number for input. */
#define DUTY_CYCLE_SCALE_VALUE (256UL) /*!< Defines upper limit of the duty cycle value. */

static void timer1_init(void);
static void gpiote_init(void);
static void ppi_init(void);

/**
 * main function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  gpiote_init();
  ppi_init();
  timer1_init();

  NVIC_EnableIRQ(GPIOTE_IRQn);
  __enable_irq();

  while (true)
  {
  }
}

/** Initializes Timer 1 peripheral.
*/
static void timer1_init(void)
{
  /* Start 16 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
  {
  }

  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER1->PRESCALER = 1;
  NRF_TIMER1->TASKS_START = 1; // Start clocks
}

/** GPIOTE interrupt handler.
* Triggered on input Low-to-high transition.
*/
void GPIOTE_IRQHandler(void)
{
  uint32_t duty_cycle;
  uint32_t cycle_duration;

  cycle_duration = NRF_TIMER1->CC[0];
  uint32_t active_time = NRF_TIMER1->CC[1];

  if (cycle_duration != 0)
  {
    duty_cycle = (DUTY_CYCLE_SCALE_VALUE * active_time) / cycle_duration;
    // Integer division always round down and the counting is not precise enough, add one.
    duty_cycle++;
  }
  else
  {
    duty_cycle = 0x00;
  }

  nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)duty_cycle);

  // Event causing the interrupt must be cleared
  NRF_GPIOTE->EVENTS_IN[0] = 0;
}

/** Initializes GPIO Tasks/Events peripheral.
*/
static void gpiote_init(void)
{
  *(uint32_t *)0x40000504 = 0xC007FFDF; // Workaround for PAN_028 rev1.1 anomaly 23 - System: Manual setup is required to enable use of peripherals

  // configure port1 (pins 8 -15) as outputs to show duty cycle
  nrf_gpio_range_cfg_output(8, 15);

  // configure pin as input
  nrf_gpio_cfg_input(INPUT_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);

  // Enable interrupt on input 0 event.
  NRF_GPIOTE->INTENSET = (1UL << GPIOTE_INTENSET_IN0_Pos);

  // Configure GPIOTE channel 0 to generate event on input pin low-to-high transition.
  // Note that we can connect multiple GPIOTE events to a single input pin.
  nrf_gpiote_event_config(0, INPUT_PIN_NUMBER, NRF_GPIOTE_POLARITY_LOTOHI);

  // Configure GPIOTE channel 1 to generate event on input pin high-to-low transition.
  // Note that we can connect multiple GPIOTE events to a single input pin.
  nrf_gpiote_event_config(1, INPUT_PIN_NUMBER, NRF_GPIOTE_POLARITY_HITOLO);
}

/** Initializes Programmable Peripheral Interconnect peripheral.
*/
static void ppi_init(void)
{
  // Configure PPI channel 0 to capture Timer 1 value into CC[0] register when GPIOTE detects Low-to-High transition on pin INPUT_PIN_NUMBER.
  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[0];
  NRF_PPI->CH[0].TEP = (uint32_t)&NRF_TIMER1->TASKS_CAPTURE[0];

  // Configure PPI channel 1 to clear Timer 1 when GPIOTE detects Low-to-High transition on pin INPUT_PIN_NUMBER.
  NRF_PPI->CH[1].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[0];
  NRF_PPI->CH[1].TEP = (uint32_t)&NRF_TIMER1->TASKS_CLEAR;

  // Configure PPI channel 2 to capture Timer 1 value into CC[1] register when GPIOTE detects High-to-Low transition on pin INPUT_PIN_NUMBER.
  NRF_PPI->CH[2].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[1];
  NRF_PPI->CH[2].TEP = (uint32_t)&NRF_TIMER1->TASKS_CAPTURE[1];

  // Enable only PPI channels 0, 1 and 2
  NRF_PPI->CHEN =    (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos)
                | (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos)
                | (PPI_CHEN_CH2_Enabled << PPI_CHEN_CH2_Pos);
}


/**
 *@}
 **/
