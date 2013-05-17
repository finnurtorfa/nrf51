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
* @brief Simple motor control
*
* @defgroup simple_motor_control Motor control example
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief Example of PWM waveform generation where timer resets after each full period.
* Setup: Standard setup.
*
* Behaviour: This example generates variable duty cycle pulse train on selected GPIO pin.
* By default the pwm waveform generation will have a duty cycle of 1/256.
* If BUTTON1 is pressed the duty cycle is set to 224/256.
* If only BUTTON0 is pressed the waveform will set the duty cycle to 32/256.
*
* Expected Results: In the default state LED0 should have a very weak light.
* When BUTTON1 is pressed the strenght of LED0 is increased to a bright light.
* When only BBUTTON0 is pressed LED0 will give a medium strenght light.
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "boards.h"

#define PWM_OUTPUT_PIN_NUMBER (LED0) /*!< Pin number for PWM output */
#define TIMER_PRESCALER       (4) /*!< Prescaler setting for timers */
#define LED_INTENSITY_HIGH    (224U) /*!< High intentisty */
#define LED_INTENSITY_LOW     (32U) /*!< Low intensity */
#define LED_OFF               (1U) /*!< Led off */
#define LED_INTENSITY_HALF    (128U) /*!< Half intensity. Used to calculate timer parameters. */

/** Sets PWM duty cycle.
 * @param new_setting New duty cycle, where 128 results in 50% duty cycle.
*/
static void pwm_set(uint8_t new_setting)
{
  NRF_TIMER2->CC[0] = new_setting;
}

/** Initializes Timer 2 peripheral.
*/
static void timer2_init(void)
{
  /* Start 16 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  
  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
  {
  }
  
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->PRESCALER = 4;

  /* Load initial values to TIMER2 CC registers */
  /* Set initial  CC0 value to anything >1 */
  NRF_TIMER2->CC[0] = LED_INTENSITY_LOW;
  NRF_TIMER2->CC[1] = (LED_INTENSITY_HALF*2);

  /* Set up interrupt for CC2 */
  /* This interrupt is to force the change of CC0 to happen when it is safe */
  /* A safe time is after the highest possible CC0 value, but before the lowest one */
  NRF_TIMER2->CC[2] = LED_INTENSITY_HIGH;

  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE2_Enabled << TIMER_INTENSET_COMPARE2_Pos;

  /* Create an Event-Task shortcut to clear TIMER2 on COMPARE[1] event. */
  NRF_TIMER2->SHORTS = (TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos);
}

/** Initializes GPIO Tasks/Events peripheral.
*/
static void gpiote_init(void)
{
  NRF_GPIO->OUT = 0x00000000UL;
  NRF_GPIO->DIRSET = 0x0000FF00UL;
  NRF_GPIO->DIRCLR = 0x000000FFUL;

  /* Configuring Button 0 as input */
  nrf_gpio_cfg_input(BUTTON0, NRF_GPIO_PIN_NOPULL);

  /* Configuring Button 1 as input */
  /*lint -e{845} // A zero has been given as right argument to operator '|'" */
  nrf_gpio_cfg_input(BUTTON1, NRF_GPIO_PIN_NOPULL);

  /* Configuring Pin PWM_OUTPUT_PIN_NUMBER as output to be used for the PWM waveform */
  nrf_gpio_cfg_output(PWM_OUTPUT_PIN_NUMBER);
  
  *(uint32_t *)0x40000504 = 0xC007FFDF; // Workaround for PAN_028 rev1.1 anomaly 23 - System: Manual setup is required to enable use of peripherals

  /* Configure GPIOTE channel 0 to toggle the PWM pin state */
  /* Note that we can only connect one GPIOTE task to an output pin */
  nrf_gpiote_task_config(0, PWM_OUTPUT_PIN_NUMBER, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_HIGH);
}

/** Initializes Programmable Peripheral Interconnect peripheral.
*/
static void ppi_init(void)
{
  /* Configure PPI channel 0 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[0] match */
  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
  NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

  /* Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[1] match */
  NRF_PPI->CH[1].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[1];
  NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

  /* Enable only PPI channels 0 and 1 */
  NRF_PPI->CHEN =
    (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos) |
    (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos);
}

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */

void TIMER2_IRQHandler(void)
{
  // Clear interrupt
  if ((NRF_TIMER2->EVENTS_COMPARE[2] == 1) && (NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE2_Msk))
  {
    NRF_TIMER2->EVENTS_COMPARE[2] = 0;
  }
  // Process buttons
  if (nrf_gpio_pin_read(BUTTON1) == 0)
  {
    pwm_set(LED_INTENSITY_HIGH);
  }
  else if (nrf_gpio_pin_read(BUTTON0) == 0)
  {
    pwm_set(LED_INTENSITY_LOW);
  }
  else
  {
    pwm_set(LED_OFF);
  }
}

int main(void)
{
  /* Intitialization */
  gpiote_init();
  ppi_init();
  timer2_init();

  /* Enable interrupt on Timer 2*/
  NVIC_EnableIRQ(TIMER2_IRQn);
  __enable_irq();

  /* Start clock */
  NRF_TIMER2->TASKS_START = 1;
  while (true)
  {
  }
}

/**
 *@}
 **/

