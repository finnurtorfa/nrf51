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
* @brief Example project to use temperature sensor and show the value on LEDs from port1 GPIO
* @defgroup temperature_example Temperature example
* @{
* @ingroup nrf_examples_nrf6310
*
* @brief Example for using temperature sensor and show the binary value on port1 GPIO,
* @note as port1 has only 8bits only positive temperature is shown
*
* This example shows how to get temperature from temperature sensor
*/

#include <stdbool.h>
#include <stdint.h>

#include <stdio.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_temp.h"
#include "nrf_gpio.h"

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
    // This function contains workaround for PAN_028 rev1.1 anomalies 24,25,26 and 27

    int32_t volatile temp;

    /* Init TEMP peripheral */
    nrf_temp_init();

    /* Configure pins 8-15 (port 1) as outputs */
    nrf_gpio_range_cfg_output(8, 15);

    while(true)
    {       
        NRF_TEMP->TASKS_START = 1; /* Start the temperature measurement */

        /* Busy wait while temperature measurement is not finished, you can skip waiting if you enable interrupt for DATARDY event and read the result in the interrupt. */
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        while ((NRF_TEMP->EVENTS_DATARDY & TEMP_INTENSET_DATARDY_Msk) != (TEMP_INTENSET_DATARDY_Set << TEMP_INTENSET_DATARDY_Pos))
        {
        }
        NRF_TEMP->EVENTS_DATARDY = 0;
        
        temp = (nrf_temp_read()/4);

        NRF_TEMP->TASKS_STOP = 1; /* Stop the temperature measurement */

        nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)(temp));

        nrf_delay_ms(500);
    }
}

/**
 *@}
 **/
