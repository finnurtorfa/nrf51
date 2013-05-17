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
* @defgroup nrf_dev_radio_rx_example Radio Receiver example
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief Example project on receiving data using the NRF_RADIO peripheral.
*
* The receiver is configured to continuously receive packets from the corresponding
* transmitter project. The first payload byte of the packet is sent to P1 which can
* be used, for example, to light up the LEDs on an nRFgo board.
*
* @image html example_board_setup_a.png "Use board setup A for this example."
*/
#include <stdint.h>
#include <stdbool.h>
#include "nrf_delay.h"
#include "radio_config.h"
#include "nrf_gpio.h"

static uint8_t volatile packet[PACKET_PAYLOAD_MAXSIZE];  ///< Received packet buffer

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  /* Start 16 MHz crystal oscillator */
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;

  /* Wait for the external oscillator to start up */
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
  {
  }
  
  // Set Port 1 as output
  nrf_gpio_range_cfg_output(8, 15);

  // Set radio configuration parameters
  radio_configure();

  nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, 0x55);

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

    // Write received data to port 1 on CRC match
    if (NRF_RADIO->CRCSTATUS == 1U)
    {
      nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, packet[0]);
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
