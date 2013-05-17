/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
*
* The information contained herein is property of Nordic Semiconductor ASA.
* Terms and conditions of usage are described in detail in NORDIC
* SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
*
* Licensees are granted free, non-transferable use of the information. NO
* WARRANTY of ANY KIND is provided. This heading must NOT be removed from
* the file.
*/

#include "nrf.h"
#include "uart.h"
#include "nrf_gpio.h"

#define BUFFER_LENGTH 16

static volatile uint8_t rx_buffer[BUFFER_LENGTH];
static volatile uint8_t rxwp, rxrp, nrx;

void uart_put(uint8_t cr)
{
  NRF_UART0->TXD = cr;
  while(NRF_UART0->EVENTS_TXDRDY!=1)
    ;
  NRF_UART0->EVENTS_TXDRDY=0;
}

uint8_t uart_get(void)
{
  while(nrx == 0)
    ;
  nrx--;
  uint8_t tmp = rx_buffer[rxrp];
  rxrp = (rxrp + 1) & (BUFFER_LENGTH - 1);
  return tmp;
}

void uart_config(uint8_t txd_pin_number, uint8_t rxd_pin_number)
{
  rxwp = rxrp = nrx = 0;
  
  nrf_gpio_cfg_output(txd_pin_number);
  nrf_gpio_cfg_input(rxd_pin_number, NRF_GPIO_PIN_NOPULL);  

  NRF_UART0->PSELTXD = txd_pin_number;
  NRF_UART0->PSELRXD = rxd_pin_number;

  NRF_UART0->BAUDRATE         = (UART_BAUDRATE_BAUDRATE_Baud38400 << UART_BAUDRATE_BAUDRATE_Pos);
  NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
  NRF_UART0->TASKS_STARTTX    = 1;
  NRF_UART0->TASKS_STARTRX    = 1;
  NRF_UART0->EVENTS_RXDRDY    = 0; 

  NRF_UART0->INTENSET        |= (UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos );

  NVIC_EnableIRQ(UART0_IRQn);
}

void uart_putstring(const uint8_t* str)
{
    while(*str)
    {
        uart_put(*str++);
    }
}

void UART0_IRQHandler(void)
{
    if (nrx > BUFFER_LENGTH)
    {
        uart_putstring((const uint8_t *)"BUFFER_OVERFLOW");
        while(1);
    }
    nrx++;
    rx_buffer[rxwp] = NRF_UART0->RXD;
    rxwp = (rxwp + 1) & (BUFFER_LENGTH - 1);
    NRF_UART0->EVENTS_RXDRDY = 0;
}
