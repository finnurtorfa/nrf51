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
* @brief Radio Receiver example.
*
* @defgroup nrf_dev_led_radio_rx_example Radio Receiver example
* @{
* @ingroup nrf_examples_pca10001
*
* @brief Example project on receiving data using the NRF_RADIO peripheral.
*
* This project must be used together with the corresponding transmitter project
* for the PCA10000 board.
*
* 1) Flash led_radio_example in the PCA10001 board folder to PCA10001 board.
* 2) Flash led_radio_example in the PCA10000 board folder to PCA10000 board.
* 3) Use a terminal program on your PC to connect to the PCA10000 board. 38400 bps, 8 data bits, 1 stop bit.
* 4) Press '0' or '1' on your terminal. This will light up either LED0 or LED1 on PCA10001 board.
*
* The receiver is configured to continuously receive packets from the corresponding
* transmitter project. If the one-byte payload contains '0' LED0 is turned on and
* if it is '1' LED1 is turned on.
*/
#include <stdint.h>
#include <stdbool.h>
#include "radio_config.h"
#include "nrf_gpio.h"
#include "boards.h"

static uint8_t volatile packet[4];  ///< Received packet buffer

void init(void)
{
  /* Start 16 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
  {
  }

  nrf_gpio_range_cfg_output(LED_START, LED_STOP);

  // Set radio configuration parameters
  radio_configure();
}

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  init();

  while(true)
  {
    // Set payload pointer
    NRF_RADIO->PACKETPTR = (uint32_t)packet;
    NRF_RADIO->EVENTS_READY = 0U;
    // Enable radio and wait for ready
    NRF_RADIO->TASKS_RXEN = 1U;
    while(NRF_RADIO->EVENTS_READY == 0U)
    {
    }
    NRF_RADIO->EVENTS_END = 0U;
    // Start listening and wait for address received event
    NRF_RADIO->TASKS_START = 1U;
    // Wait for end of packet
    while(NRF_RADIO->EVENTS_END == 0U)
    {
    }
    // Write received data to LED0 and LED1 on CRC match
    if (NRF_RADIO->CRCSTATUS == 1U)
    {
      switch(packet[0])
      {
        case '0':
          nrf_gpio_pin_set(LED0);
          nrf_gpio_pin_clear(LED1);
          break;

        case '1':
          nrf_gpio_pin_set(LED1);
          nrf_gpio_pin_clear(LED0);
          break;
      }
    }
    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;
    while(NRF_RADIO->EVENTS_DISABLED == 0U)
    {
    }
  }
}

/**
 *@}
 **/
