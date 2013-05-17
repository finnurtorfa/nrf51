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
* @defgroup nrf_dev_button_radio_tx_example Radio Transmitter example
* @{
* @ingroup nrf_examples_pca10001
*
* @brief Example project on transmitting data using the NRF_RADIO peripheral.
*
* This project must be used together with the corresponding receiver project 
* button_radio_example under PCA10000 board folder. 
*
* 1) Flash button_radio_example in the PCA10001 board folder to PCA10001 board.
* 2) Flash button_radio_example in the PCA10000 board folder to PCA10000 board.
* 3) Use a terminal program on your PC to connect to the PCA10000 board. 38400 bps, 8 data bits, 1 stop bit.
* 4) Press a button on PCA10001 board, you should see "Button 0/1 pressed" on your terminal.
* 
* This example continuously sends one-byte payload radio packets. The payload
* contains the button status of the PCA10001 board. If Button 0 is pushed the
* payload is 1, if Button 1 is pushed the payload is 2 and if both are pushed
* the payload is 3. When no button is pushed the payload is set to zero and only
* when a button's state goes from not pushed to pushed the corresponding bit in
* the payload is set.
*
*/

#include <stdint.h>
#include <stdbool.h>
#include "radio_config.h"
#include "nrf_gpio.h"
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
  
  // Set payload pointer
  NRF_RADIO->PACKETPTR = (uint32_t)packet;
  
  nrf_gpio_range_cfg_input(BUTTON_START, BUTTON_STOP, NRF_GPIO_PIN_PULLUP);
}

/**
 * main function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  init();
  
  uint8_t btn0_nstate;                              // Store new (current) state of button 0
  uint8_t btn1_nstate;                              // Store new (current) state of button 1
  uint8_t btn0_ostate = nrf_gpio_pin_read(BUTTON0); // Store old (previous) state of button 0
  uint8_t btn1_ostate = nrf_gpio_pin_read(BUTTON1); // Store old (previous) state of button 1
  
  while(true)
  {
    uint8_t btns = 0;
    btn0_nstate = nrf_gpio_pin_read(BUTTON0);
    btn1_nstate = nrf_gpio_pin_read(BUTTON1);
    if ((btn0_ostate == 1) && (btn0_nstate == 0))
    {
      btns |= 1;
    }
     
    if ((btn1_ostate == 1) && (btn1_nstate == 0))
    {
      btns |= 2;
    }
    
    btn0_ostate = btn0_nstate;
    btn1_ostate = btn1_nstate;
    
    // Place the read buttons in the payload, enable the radio and
    // send the packet:
    packet[0] = btns;
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
