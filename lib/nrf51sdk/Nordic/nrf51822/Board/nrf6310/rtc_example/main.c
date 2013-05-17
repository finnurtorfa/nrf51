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
 * @brief RTC example.
 * @defgroup rtc_example Real-time clock example
 * @{
 * @ingroup nrf_examples_nrf6310
 *
 * @brief RTC module example
 *
 * This example enables the RTC with a TICK frequency of 8Hz. It also configures
 * and enables the TICK- and the COMPARE0-interrupts. The compare interrupt
 * handler will be triggered 3 seconds after the RTC starts. The TICK interrupt
 * handler clears the interrupt request and toggles PIN8. The COMPARE0 handler
 * clears the interrupt request an set PIN9 to 1.
 * @image html example_board_setup_a.png "Use board setup A for this example."
 */

#include <stdbool.h>
#include "nrf.h"
#include "nrf_gpio.h"

#define GPIO_TOGGLE_TICK_EVENT    (8)                       /*!< Pin number to toggle when there is a tick event in RTC */
#define GPIO_TOGGLE_COMPARE_EVENT (9)                       /*!< Pin number to toggle when there is compare event in RTC */
#define LFCLK_FREQUENCY           (32768UL)                 /*!< LFCLK frequency in Hertz, constant */
#define RTC_FREQUENCY             (8UL)                     /*!< required RTC working clock RTC_FREQUENCY Hertz. Changable */
#define COMPARE_COUNTERTIME       (3UL)                     /*!< Get Compare event COMPARE_TIME seconds after the counter starts from 0 */

#define COUNTER_PRESCALER         ((LFCLK_FREQUENCY/RTC_FREQUENCY) - 1)  /* f = LFCLK/(prescaler + 1) */


/** Starts the internal LFCLK XTAL oscillator
 */
static void lfclk_config(void)
{
  NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_LFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
  {
  }
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
}

/** Configures the RTC with TICK for 100Hz and COMPARE0 to 10 sec
 */
static void rtc_config(void)
{
  NVIC_EnableIRQ(RTC0_IRQn);                                 // Enable Interrupt for the RTC in the core
  NRF_RTC0->PRESCALER = COUNTER_PRESCALER;                   // Set prescaler to a TICK of RTC_FREQUENCY
  NRF_RTC0->CC[0] = COMPARE_COUNTERTIME * RTC_FREQUENCY;     // Compare0 after approx COMPARE_COUNTERTIME seconds

  // Enable TICK event and TICK interrupt:
  NRF_RTC0->EVTENSET = RTC_EVTENSET_TICK_Msk;
  NRF_RTC0->INTENSET = RTC_INTENSET_TICK_Msk;

  // Enable COMPARE0 event and COMPARE0 interrupt:
  NRF_RTC0->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
  NRF_RTC0->INTENSET = RTC_INTENSET_COMPARE0_Msk;
}

/** Configures PIN8 and PIN9 as outputs
 */
static void gpio_config(void)
{
  nrf_gpio_cfg_output(GPIO_TOGGLE_TICK_EVENT);
  nrf_gpio_cfg_output(GPIO_TOGGLE_COMPARE_EVENT);

  nrf_gpio_pin_write(GPIO_TOGGLE_TICK_EVENT, 0);
  nrf_gpio_pin_write(GPIO_TOGGLE_COMPARE_EVENT, 0);
}

/** RTC0 interrupt handler.
 * Triggered on TICK and COMPARE0 match.
 */
void RTC0_IRQHandler()
{
  if ((NRF_RTC0->EVENTS_TICK != 0) && ((NRF_RTC0->INTENSET & RTC_INTENSET_TICK_Msk) != 0))
  {
    NRF_RTC0->EVENTS_TICK = 0;
    nrf_gpio_pin_toggle(GPIO_TOGGLE_TICK_EVENT);
  }
  if ((NRF_RTC0->EVENTS_COMPARE[0] != 0) && ((NRF_RTC0->INTENSET & RTC_INTENSET_COMPARE0_Msk) != 0))
  {
    NRF_RTC0->EVENTS_COMPARE[0] = 0;
    nrf_gpio_pin_write(GPIO_TOGGLE_COMPARE_EVENT, 1);
  }
}

/**
 * main function
 * \return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  gpio_config();
  lfclk_config();
  rtc_config();
  NRF_RTC0->TASKS_START = 1;
  while (true)
  {
  }
}

/**
 *@}
 **/
