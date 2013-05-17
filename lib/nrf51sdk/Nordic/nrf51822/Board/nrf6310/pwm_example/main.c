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
 * @brief Example project on PPI/Timer usage to output PWM waveform over GPIO.
 *
 * This example generates variable duty cycle pulse train on selected GPIO pin.
 *
 * @defgroup pwm_example PWM example
 * @{
 * @ingroup nrf_examples_nrf6310
 *
 * @brief Example of basic PWM generation using freerunning timer.
 * The GPIO pin could for example drive a small speaker.
 * Setup: Standard setup.
 *
 * Behaviour: This example generates variable duty cycle pulse train on selected GPIO pin.
 * The duty cycle is read as a byte taken from state of the buttons and will therefore vary between 1-255.
 * Notice that the value 0 is omitted, this is because of a problem with having two events trigger
 * at the same time only causing one toggle of the output pin. Timer 2 is used to dispatch
 * GPIOTE Toggle events on every compare match. CC1 will set the output pin high while CC0 and CC2
 * will toggle it low every other turn ending a duty cycle. These compare timings will be set by an
 * interrupt happening every CC1 at the 0(or 256) mark for the period. This interrupt will increase
 * CC1 with one more period and every turn change between setting CC0 and CC2 to the value retrieved
 * from state of the buttons.
 *
 * Expected Results: The program will emit light on LED0 corresponding to what value is pressed in
 * with buttons, where Button "BUTTON_START" is LSB bit and Button "BUTTON_STOP" is MSB bit of a byte. With a higher value
 * the LED will emit brighter light.
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"

#define PWM_OUTPUT_PIN_NUMBER (LED0)  /*!< Pin number for PWM output */

#define MAX_SAMPLE_LEVELS (256UL)     /*!< Maximum number of sample levels */
#define TIMER_PRESCALERS 4U           /*!< Prescaler setting for timer */

static uint32_t last_cc0_sample;      /*!< CC0 register value in the previous round */
static uint32_t last_cc2_sample;      /*!< CC2 register value in the previous round */

/** Get next sample.
\return sample_value Next sample value.
 */
static __INLINE uint32_t next_sample_get(void)
{
  uint8_t sample_value = 0;
  
  // Read button input
  sample_value = (~NRF_GPIO->IN & 0x000000FFUL);
  
  // This is to avoid having 2 CC events happen at the same time,
  // CC1 will always create an event on 0 so CC0 and CC2 should not
  if (sample_value == 0) sample_value = 1;

  return (uint32_t)sample_value;
}

/** Timer 2 peripheral interrupt handler.
 */
void TIMER2_IRQHandler(void)
{
  static bool cc0_turn = false; /*!< Variable to keep track which CC register is to be used */

  if ((NRF_TIMER2->EVENTS_COMPARE[1] != 0) && ((NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE1_Msk) != 0))
  {
    // Sets the next CC1 value
    NRF_TIMER2->EVENTS_COMPARE[1] = 0;
    NRF_TIMER2->CC[1] = (NRF_TIMER2->CC[1] + MAX_SAMPLE_LEVELS);

    // Every other interrupt CC0 and CC2 will be set to their next values
    // They each keep track of their last duty cycle so they can compute their next correctly
    uint32_t next_sample = next_sample_get();

    if (cc0_turn)
    {
      NRF_TIMER2->CC[0] = (NRF_TIMER2->CC[0] - last_cc0_sample + 2*MAX_SAMPLE_LEVELS + next_sample);
      last_cc0_sample = next_sample;
    }
    else
    {
      NRF_TIMER2->CC[2] = (NRF_TIMER2->CC[2] - last_cc2_sample + 2*MAX_SAMPLE_LEVELS + next_sample);
      last_cc2_sample = next_sample;
    }
    // Next turn the other CC will get its value
    cc0_turn = !cc0_turn;
  }
}


/** Initialises Timer 2 peripheral.
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
  NRF_TIMER2->PRESCALER = TIMER_PRESCALERS;

  // Clears the timer, sets it to 0
  NRF_TIMER2->TASKS_CLEAR = 1;

  // Load initial values to TIMER2 CC registers.
  // CC2 will be set on the first CC1 interrupt.
  // Timer compare events will only happen after the first 2 values
  last_cc0_sample = next_sample_get();
  last_cc2_sample = 0;
  NRF_TIMER2->CC[0] = MAX_SAMPLE_LEVELS + last_cc0_sample;
  NRF_TIMER2->CC[1] = MAX_SAMPLE_LEVELS;
  NRF_TIMER2->CC[2] = 0;

  // Interrupt setup
  NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);
}

/** Initialises GPIO Tasks/Events peripheral.
 */
static void gpiote_init(void)
{
  *(uint32_t *)0x40000504 = 0xC007FFDF; // Workaround for PAN_028 rev1.1 anomaly 23 - System: Manual setup is required to enable use of peripherals

  // Connect GPIO input buffers and configure PWM_OUTPUT_PIN_NUMBER as output
  nrf_gpio_range_cfg_input(BUTTON_START, BUTTON_STOP, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_output(PWM_OUTPUT_PIN_NUMBER);

  NRF_GPIO->OUT = 0x00000000UL;

  // Configure GPIOTE channel 0 to toggle the PWM pin state
  // Note that we can only connect one GPIOTE task to an output pin
  nrf_gpiote_task_config(0, PWM_OUTPUT_PIN_NUMBER, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
}

/** Initialises Programmable Peripheral Interconnect peripheral.
 */
static void ppi_init(void)
{
  // Configure PPI channel 0 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[0] match
  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
  NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

  // Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[1] match
  NRF_PPI->CH[1].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[1];
  NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

  // Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every TIMER2 COMPARE[2] match
  NRF_PPI->CH[2].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[2];
  NRF_PPI->CH[2].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

  // Enable PPI channels 0-2
  NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos)
                | (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos)
                | (PPI_CHEN_CH2_Enabled << PPI_CHEN_CH2_Pos);
}


/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  gpiote_init();
  ppi_init();
  timer2_init();

  // Enable interrupt on Timer 2
  NVIC_EnableIRQ(TIMER2_IRQn);
  __enable_irq();

  // Start clock
  NRF_TIMER2->TASKS_START = 1;

  while (true)
  {
  }
}

/**
 *@}
 **/
