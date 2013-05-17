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
* @brief Example project to use PPI module to communicate between timer.
* This example uses PPI module for the timer able to communicate with other timers using events
* @defgroup ppi_example  PPI example code
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief Example for using PPI module.
*
* This example uses PPI module for communication between three timer. One timer (T1) is used to generate events every odd seconds, that is at
* 1,3,5,7 ...etc seconds. This events are used for creating another timer (T0) start task through PPI which is running in counter mode. The other
* timer (T2) is used to generate events every even seconds that is at 2,4,6,8,..etc and are used to trigger stop task for timer T0 thorough another channel of PPI.
* Our main program loop is running to trigger the counter every 100 milliseconds. and the timer T0 coutner values are passed to GPIO port.
* This port can be used to light up LEDs in which case you will see that the LEDs change binary value every 100 milliseconds for a second then PAUSE for one second
* because of the timer T2 creating an event passed though PPI converting this to timer T0 stop task, in this start event the main loop triggers timer T0 counter increment
* nothing changes as timer T0 stopped and after one more seconds timer T1 generates event which is passed though PPI
* to create timer T0 start task and then the LEDS start changing again. This loops goes on for ever.
*
* @verbatim
*  ----------                                        -----------                 ------------------
* |  Timer1  |  CC[0] event at 2,4,6,8,.. seconds   |   PPI     |               |                  |
* |          |------------------------------------->|   CH0     |--STOP_TASK--->|     Timer0       |
*  ----------                                        -----------                |                  |
*                                                                               |  Counter Mode    |<---Increment every 100ms
*  ----------                                        -----------                |                  |
* |  Timer2  |  CC[0] event at 1,3,5,7,.. seconds   |   PPI     |               |                  |
* |          |------------------------------------->|   CH1     |--START_TASK-->|                  |
*  ----------                                        -----------                 ------------------
*                                                                                        |
*                                                                                        |
*                                                                                  ---------------
*                                                                                 | capture CC[0] |
*                                                                                  ---------------
*                                                                                        |
*                                                                                      CC[0]
*                                                                                        |
*                                                                                    GPIOs_Output
* @endverbatim
* @image html example_board_setup_a.png "Use board setup A for this example."
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"

/** Initializes Programmable Peripheral Interconnect peripheral.
*/
static void ppi_init(void)
{
  // Configure PPI channel 0 to stop Timer0 counter at TIMER1 COMPARE[0] match, which is every even number of seconds
  NRF_PPI->CH[0].EEP = (uint32_t)(&NRF_TIMER1->EVENTS_COMPARE[0]);
  NRF_PPI->CH[0].TEP = (uint32_t)(&NRF_TIMER0->TASKS_STOP);

  // Configure PPI channel 1 to start timer0 counter at TIMER2 COMPARE[0] match, which is every odd number of seconds
  NRF_PPI->CH[1].EEP = (uint32_t)(&NRF_TIMER2->EVENTS_COMPARE[0]);
  NRF_PPI->CH[1].TEP = (uint32_t)(&NRF_TIMER0->TASKS_START);

  // Enable only PPI channels 0 and 1
  NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos) | (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos);
}

/** Timer 0 init which will be started and stopped by timer1 and timer2 using PPI.
*/
static void timer0_init(void)
{
  NRF_TIMER0->MODE    = TIMER_MODE_MODE_Counter;      // Set the timer in counter Mode
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;  // 24-bit mode
}

/** Initializes Timer 1 peripheral, creates event and interrupt every 2 seconds,
 * by configuring CC[0] to timer overflow value, we create events at even number of seconds
 * for example, events are created at 2,4,6 .. seconds, this event can be used to stop Timer0
 * with Timer1->Event_Compare[0] triggering timer0 TASK_STOP through PPI
*/
static void timer1_init(void)
{
  // Configure timer1 to overflow every 2 seconds
  // SysClk = 16Mhz
  // BITMODE = 16 bit
  // PRESCALER = 9
  // now the overflow occurs every 0xFFFF/(SysClk/2^PRESCALER)
  // = 65535/31250 = 2.097 seconds
  NRF_TIMER1->BITMODE = (TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos);
  NRF_TIMER1->PRESCALER = 9;
  NRF_TIMER1->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);

  // Trigger interrupt for compare[0] event
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER1->CC[0] = 0xFFFFUL;  // match at even number of seconds
}

/** Initializes Timer 2 peripheral, creates event and interrupt every 2 seconds,
 * by configuring CC[0] to half of timer overflow value , we create events at odd number of seconds
 * for example, events are created at 1,3,5,... seconds,  this event can be used to start Timer0
 * with Timer2->Event_Compare[0] triggering timer0 TASK_START through PPI
*/
static void timer2_init(void)
{
  // generate interrupt/event when half time before timer overflows, that is at 1,3,5,7,... so on secs from start
  // SysClk = 16Mhz
  // BITMODE = 16 bit
  // PRESCALER = 9
  // now the overflow occurs every 0xFFFF/(SysClk/2^PRESCALER)
  // = 65535/31250 = 2.097 seconds */
  NRF_TIMER2->BITMODE = (TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos);
  NRF_TIMER2->PRESCALER = 9;
  NRF_TIMER2->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);


  // Trigger interrupt for compare[0] event
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->CC[0] = 0x8FFFUL;  // match at odd number of seconds
}
/**
 * main function
 * \return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  timer0_init();  // timer whose value is used to blink LEDs
  timer1_init();  // using timer to generate events even number of seconds
  timer2_init();  // using timer to generate events odd number of seconds
  ppi_init();     // use ppi to redirect the event to timer start/stop tasks

  // Start clock
  NRF_TIMER1->TASKS_START = 1;
  NRF_TIMER2->TASKS_START = 1;

  // Configure pins 8-15 as outputs
  nrf_gpio_range_cfg_output(8, 15);

  // loop, increment the timer count value and capture value into LEDs. Note counter is only incremented between TASK_START and TASK_STOP
  while (true)
  {
    /* increment the counter */
    NRF_TIMER0->TASKS_COUNT = 1;
    NRF_TIMER0->TASKS_CAPTURE[0] = 1;

    nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)NRF_TIMER0->CC[0]);

    nrf_delay_ms(100);
  }
}

/**
 *@}
 **/
