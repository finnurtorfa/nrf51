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
* @brief Example project to show gpiote functionality.
* @defgroup nrf_gpiote_example GPIOTE example
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief GPIOTE example
*
* This example shows the usage of GPIOTE peripheral. We use TIMER0 to generate 'EVENT_COMPARE0' event every
* 200 milliseconds. This will also clear the TIMER0 internal clock. This event is passed through PPI peripheral
* to generate a task. This task is used by GPIOTE (task mode) to toggle the pin. This means that every time we
* have EVENT_COMPARE event, the GPIO pin toggles automatically, which can only be achieved using the GPIOTE
* peripheral.
* [TIMER0]->EVENT_COMPARE->[PPI]->GPIOTE_TASK_OUT->[GPIOTE]->toggle_pin
*
* @verbatim
*          ----------------                       --------------------------
*  -Start-|    Timer0      |                     |   EEP |    PPI    |  TEP |-------
*  ------>|  CC[0] = 200ms |-----Timer0_Event--->|       | Channel 0 |      |       |
* |        ----------------                       --------------------------        |
* |                                                                                 |
* |    -----------------------------------------------------------------------------
* |   |
* |   |                 -----------------                      ---------------
* |    ---------TASK-->|     GPIO_TE     |-->Toggled_Output-->|  LED toggling |--
* |                     -----------------                     |  every 200ms  |  |
* |                                                            ---------------   |
*  ------------------------------------------------------------------------------
*
* @endverbatim
* @image html example_board_setup_a.png "Use board setup A for this example."
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"


#define GPIO_OUTPUT_PIN_NUMBER 8  /*!< Pin number for PWM output */
#define GPIOTE_CHANNEL_NUMBER  0  /*!< GPIOTE channel number */


/** Initializes GPIO Tasks and Events peripheral.
*/
static void gpiote_init(void)
{
  *(uint32_t *)0x40000504 = 0xC007FFDF; // Workaround for PAN_028 rev1.1 anomaly 23 - System: Manual setup is required to enable use of peripherals

  // Configure GPIO GPIO_OUTPUT_PIN_NUMBER as an output
  nrf_gpio_cfg_output(GPIO_OUTPUT_PIN_NUMBER);

  // Configure GPIOTE channel GPIOTE_CHANNEL_NUMBER to toggle the GPIO  pin state with input
  // Note that we can only connect a GPIOTE task to an output pin
  nrf_gpiote_task_config(GPIOTE_CHANNEL_NUMBER, GPIO_OUTPUT_PIN_NUMBER, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
}

/** Initializes Programmable Peripheral Interconnect (PPI) peripheral.
*   Need PPI to convert timer event into task
*/
static void ppi_init(void)
{
  // Configure PPI channel 0 to toggle GPIO_OUTPUT_PIN on every TIMER0 COMPARE[0] match, which is every 200 ms
  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER0->EVENTS_COMPARE[0];
  NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_CHANNEL_NUMBER];

  // Enable PPI channel 0
  NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos);
}

/** Initializes Timer 0 peripheral.
*/
static void timer0_init(void)
{
  /* Start 16 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
  {
  }

  // Clear TIMER0
  NRF_TIMER0->TASKS_CLEAR = 1;

  // Configure TIMER0 for compare[0] event every 200ms
  NRF_TIMER0->PRESCALER = 4;      // Prescaler 4 results in 1 tick == 1 microsecond
  NRF_TIMER0->CC[0] = 200*1000UL; // 1 tick == 1 microsecond, multiply by 1000 to get number in milliseconds
  NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;
  NRF_TIMER0->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
}

/**
 * main function
 * \return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  gpiote_init();                  // Configure a GPIO to toggle on a GPIOTE task,
  timer0_init();                  // using TIMER0 to generate events every 200 ms
  ppi_init();                     // and using a PPI channel to connect the event to the task automatically;

  NRF_TIMER0->TASKS_START = 1;    // after starting the event generation

  while (true)
  {
                                  // a GPIO can be toggled without SW intervention
  }
}

/**
 *@}
 **/
