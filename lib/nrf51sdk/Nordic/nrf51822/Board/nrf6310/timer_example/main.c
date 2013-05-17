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
 * @brief Timer example project.
 *
 * @defgroup nrf_dev_timer_example Timer example
 * @{
 * @ingroup nrf_examples_nrf6310
 *
 * @brief Example project on using the Timer0 peripheral.
 *
 * This example replaces the less accurate busy delay in blinky example with a
 * more accurate hardware timer based delay.
 * @image html example_board_setup_a.png "Use board setup A for this example."
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"

#define TIMER_DELAY_MS               (100UL)   /*!< Timer Delay in milli-seconds */
#define GPIO_TOGGLE_PIN              (8)       /*!< gpio pin to toggle after delay */

/** @def p_timer
 * Enumeration for the timers available for NRF device
 */
typedef enum
{
  TIMER0 = 0,  /*!< Timer0 module, base address at 0x40008000 */
  TIMER1,      /*!< Timer1 module, base address at 0x40009000 */
  TIMER2       /*!< Timer2 module, base address at 0x4000A000 */
} timer_t;

static void nrf_timer_delay_ms(timer_t timer, uint_fast16_t volatile number_of_ms);

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 * @sa nrf_timer_delay_ms
 */
int main(void)
{
  // Configure pins 8 to 15 as outputs
  nrf_gpio_cfg_output(GPIO_TOGGLE_PIN);

  while (true)
  {
    nrf_gpio_pin_toggle(GPIO_TOGGLE_PIN);
    // use hardware timer0 to generate 100ms delay
    nrf_timer_delay_ms(TIMER0, TIMER_DELAY_MS);

    nrf_gpio_pin_toggle(GPIO_TOGGLE_PIN);
    // use hardware timer1 to generate 100ms delay
    nrf_timer_delay_ms(TIMER1, TIMER_DELAY_MS);

    nrf_gpio_pin_toggle(GPIO_TOGGLE_PIN);
    // use hardware timer2 to generate 100ms delay
    nrf_timer_delay_ms(TIMER2, TIMER_DELAY_MS);
  }
}

static volatile NRF_TIMER_Type * timer_init(timer_t timer)
{
  volatile NRF_TIMER_Type * p_timer;

  /* Start 16 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
  {
  }

  switch (timer)
  {
    case TIMER0:
      p_timer = NRF_TIMER0;
      break;

    case TIMER1:
      p_timer = NRF_TIMER1;
      break;

    case TIMER2:
      p_timer = NRF_TIMER2;
      break;

    default:
      p_timer = 0;
      break;
  }

  return p_timer;
}

/** @def nrf_timer_delay_ms
 * This function uses hardware timers to generate an event after requested number of milliseconds.
 * Unlike nrf_delay_ms/nrf_delay_us, this function would have a very accurate delay that directly depends on the hardware latencies.
 * @param timer timer to be used for delay, values from @ref p_timer
 * @note This function will power ON the requested timer, wait until the delay and then power OFF that timer.
 */
static void nrf_timer_delay_ms(timer_t timer, uint_fast16_t volatile number_of_ms)
{
  volatile NRF_TIMER_Type * p_timer = timer_init(timer);

  if (p_timer == 0) while(true) {}

  p_timer->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Timer Mode
  p_timer->PRESCALER = 9;                 // Prescaler 9 produces 31250 Hz timer frequency => 1 tick = 32 us
  p_timer->BITMODE = TIMER_BITMODE_BITMODE_16Bit;  // 16-bit mode

  p_timer->TASKS_CLEAR = 1;               // clear the task first to be usable for later
  // With 32 us ticks, we need to multiply by 31.25 to get milliseconds
  p_timer->CC[0] = number_of_ms * 31;
  p_timer->CC[0] += number_of_ms / 4; 
  p_timer->TASKS_START = 1;               // Start clocks

  while (p_timer->EVENTS_COMPARE[0] == 0)
  {
  }

  p_timer->EVENTS_COMPARE[0] = 0;
  p_timer->TASKS_STOP = 1;                // Stop clocks
}

/**
 *@}
 **/
