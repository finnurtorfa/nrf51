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
* @brief Radio Transmitter example.
*
* @defgroup nrf_dev_led_radio_tx_example Radio Transmitter example
* @{
* @ingroup nrf_examples_pca10000
*
* @brief Example project on transmitting data using the NRF_RADIO peripheral.
*
* This project must be used together with the corresponding receiver project
* for the PCA10001 board.
*
* 1) Flash led_radio_example in the PCA10001 board folder to PCA10001 board.
* 2) Flash led_radio_example in the PCA10000 board folder to PCA10000 board.
* 3) Use a terminal program on your PC to connect to the PCA10000 board. 38400 bps, 8 data bits, 1 stop bit.
* 4) Press '0' or '1' on your terminal. This will light up either LED0 or LED1 on PCA10001 board.
*
* This example reads a character from the uart and if it is '0' or '1' the
* character is placed in a one-byte payload and transmitted using the radio.
*
*/

#include <stdint.h>
#include "radio_config.h"
#include "nrf_gpio.h"
#include "simple_uart.h"
#include "boards.h"

static uint8_t packet[4];  ///< Packet to transmit

void init(void)
{
  /* Start 16 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
  {
  }

  // Set radio configuration parameters
  radio_configure();
  
  simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);

  // Set payload pointer
  NRF_RADIO->PACKETPTR = (uint32_t)packet;  
}

/**
 * main function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  init();
  simple_uart_putstring((const uint8_t *)"\n\rPress '0' or '1': ");
  while(true)
  {
    uint8_t c = simple_uart_get();
    if (c != '0' && c != '1')
      continue;
    simple_uart_put(c);
    // Place the read character in the payload, enable the radio and
    // send the packet:
    packet[0] = c;
    NRF_RADIO->EVENTS_READY = 0U;
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0U)
    {
    }
    NRF_RADIO->TASKS_START = 1U;
    NRF_RADIO->EVENTS_END = 0U;  
    while(NRF_RADIO->EVENTS_END == 0U)
    {
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
