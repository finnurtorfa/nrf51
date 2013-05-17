/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
 
/**
A template file for system_device.c is provided by ARM but adapted by the 
silicon vendor to match their actual device. As a minimum requirement this file 
must provide a device specific system configuration function and a global 
variable that contains the system frequency. It configures the device and 
initializes typically the oscillator (PLL) that is part of the microcontroller 
device.
*/

#include <stdint.h>
#include "nrf.h"

/*lint ++flb "Enter library region" */

#define __SYSTEM_CLOCK      (__XTAL)

#define FLASH_RESET_VALUE   (0xFFFFFFFF)


/**
  System Core Clock Frequency (Core Clock).
  
  Contains the system core clock which is the system clock frequency supplied
  to the SysTick timer and the processor core clock. This variable can be used
  by the user application to setup the SysTick timer or configure other
  parameters. It may also be used by debugger to query the frequency of the
  debug timer or configure the trace clock speed. SystemCoreClock is
  initialized with a correct predefined value.

  The compiler must be configured to avoid the removal of this variable in case
  that the application program is not using it. It is important for debug
  systems that the variable is physically present in memory so that it can be
  examined to configure the debugger.
*/
uint32_t SystemCoreClock = __SYSTEM_CLOCK;

/**
  Sets up the microcontroller system.

  Typically this function configures the oscillator (PLL) that is part of the
  microcontroller device. For systems with variable clock speed it also updates
  the variable SystemCoreClock. SystemInit is called from startup_device file
  before entering main() function.
*/
void SystemInit (void) 
{
    /* Switch ON both RAM banks */
    NRF_POWER->RAMON |= (POWER_RAMON_ONRAM0_RAM0On << POWER_RAMON_ONRAM0_Pos) |
                        (POWER_RAMON_ONRAM1_RAM1On << POWER_RAMON_ONRAM1_Pos);

}

/**
  Updates the variable SystemCoreClock and must be called whenever the core
  clock is changed during program execution.
  
  SystemCoreClockUpdate() evaluates the clock register settings and calculates
  the current core clock.
*/
void SystemCoreClockUpdate (void)
{
  SystemCoreClock = __SYSTEM_CLOCK;
}

/*lint --flb "Leave library region" */
