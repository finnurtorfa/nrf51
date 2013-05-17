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
 * @brief Example project on GPIO usage to read GPIO pin states.
 * @defgroup debouncer_example Button debouncer example
 * @{
 * @ingroup nrf_examples_nrf6310
 *
 * @brief Example project on GPIO usage with integrator debouncer to read GPIO pin states.
 *
 * @image html example_board_setup_a.png "Use board setup A for this example. "
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "lib_debounce.h"

#define DEBOUNCE_TIME_IN_MS (50U) //!< Debounce timer in milliseconds
#define DEBOUNCE_INPUT_SAMPLING_FREQ (60U) //!< Input sampling frequency in Hertz

#define TIMER0_PRESCALER (9UL) /*!< Timer 0 prescaler */
#define TIMER0_CLOCK (SystemCoreClock >> TIMER0_PRESCALER) /*!< Timer clock frequency */

#define MS_TO_TIMER0_TICKS(ms) ((1000000UL * ms) / (TIMER0_CLOCK)) /*!< Converts milliseconds to timer ticks */

#define MAX_BUTTONS (8U) /*!< Maximum number of buttons in use */

static uint_fast16_t timer0_cc0_period; /*!< Period between debouncer input reads. */
static deb_t button[MAX_BUTTONS]; /*!< Debounced button state holder */

/** Initializes Timer 0 peripheral.
 */
static void timer0_init(void);

/** Timer 0 peripheral interrupt handler.
 */
void TIMER0_IRQHandler(void)
{
  if ((NRF_TIMER0->EVENTS_COMPARE[0] != 0) && ((NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
  {
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->CC[0] += timer0_cc0_period;
    for (uint_fast8_t button_index=0; button_index < MAX_BUTTONS; button_index++)
    {
      debounce(nrf_gpio_pin_read(button_index), &button[button_index]);
    }
  }
}

static void timer0_init(void)
{
  NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer; // Set the timer in Timer Mode
  NRF_TIMER0->PRESCALER = TIMER0_PRESCALER;
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;  // 24-bit mode

  // Enable interrupt for COMPARE[0]
  NRF_TIMER0->INTENSET = (1UL << TIMER_INTENSET_COMPARE0_Pos);
  NRF_TIMER0->CC[0] = timer0_cc0_period;
  NRF_TIMER0->TASKS_START = 1; // Start clocks
}


/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  // DEBOUNCE_INPUT_SAMPLING_FREQ is in hertz so we need to multiply by 1000 to get milliseconds
  timer0_cc0_period = MS_TO_TIMER0_TICKS((1000 * 1 / DEBOUNCE_INPUT_SAMPLING_FREQ));
  timer0_init();

  NVIC_EnableIRQ(TIMER0_IRQn); // Enable Interrupt for the timer in the core
  __enable_irq();

  // Configure pins 0-7 as inputs
  nrf_gpio_range_cfg_input(0, 7, NRF_GPIO_PIN_NOPULL);

  // Configure pins 8-15 as outputs
  nrf_gpio_range_cfg_output(8, 15);

  NRF_GPIO->OUT = 0UL;

  for (uint_fast8_t button_index=0; button_index < MAX_BUTTONS; button_index++)
  {
    debounce_init(&button[button_index], DEBOUNCE_TIME_IN_MS, DEBOUNCE_INPUT_SAMPLING_FREQ);
  }

  while (true)
  {
    for (uint_fast8_t button_index=0; button_index < MAX_BUTTONS; button_index++)
    {
      // 8+ to map buttons to LEDs
      nrf_gpio_pin_write(8+button_index, !button[button_index].output_state);
    }
  }
}
/**
 *@}
 **/
