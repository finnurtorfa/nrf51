/* 
Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.

The information contained herein is confidential property of Nordic
Semiconductor ASA.Terms and conditions of usage are described in detail
in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.

Licensees are granted free, non-transferable use of the information. NO
WARRANTY of ANY KIND is provided. This heading must NOT be removed from
the file.
 */

/*
Description message
 */

/* <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
*/
  .equ    Stack_Size, 2048
    .section ".stack", "w"
    .align  3
    .globl  __cs3_stack_mem
    .globl  __cs3_stack_size
__cs3_stack_mem:
    .if     Stack_Size
    .space  Stack_Size
    .endif
    .size   __cs3_stack_mem,  . - __cs3_stack_mem
    .set    __cs3_stack_size, . - __cs3_stack_mem



/* <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
*/

    .equ    Heap_Size,  2048

    .section ".heap", "w"
    .align  3
    .globl  __cs3_heap_start
    .globl  __cs3_heap_end
__cs3_heap_start:
    .if     Heap_Size
    .space  Heap_Size
    .endif
__cs3_heap_end:

/* Vector Table */

    .section ".cs3.interrupt_vector"
    .globl  __cs3_interrupt_vector_cortex_m
    .type   __cs3_interrupt_vector_cortex_m, %object

__cs3_interrupt_vector_cortex_m:
    .long   __cs3_stack                 /* Top of Stack */
    .long   __cs3_reset                 /* Reset Handler */
    .long   NMI_Handler                 /* NMI Handler */
    .long   HardFault_Handler           /* Hard Fault Handler */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   SVC_Handler                 /* SVCall Handler */
    .long   0                           /* Reserved */
    .long   0                           /* Reserved */
    .long   PendSV_Handler              /* PendSV Handler */
    .long   SysTick_Handler             /* SysTick Handler */

  /* External Interrupts */
    .long 	POWER_CLOCK_IRQHandler		 /*POWER_CLOCK */
    .long 	RADIO_IRQHandler		 /*RADIO */
    .long 	UART0_IRQHandler		 /*UART0 */
    .long 	SPI0_TWI0_IRQHandler		 /*SPI0_TWI0 */
    .long 	SPI1_TWI1_IRQHandler		 /*SPI1_TWI1 */
    .long 	0		 /*Reserved */
    .long 	GPIOTE_IRQHandler		 /*GPIOTE */
    .long 	ADC_IRQHandler		 /*ADC */
    .long 	TIMER0_IRQHandler		 /*TIMER0 */
    .long 	TIMER1_IRQHandler		 /*TIMER1 */
    .long 	TIMER2_IRQHandler		 /*TIMER2 */
    .long 	RTC0_IRQHandler		 /*RTC0 */
    .long 	TEMP_IRQHandler		 /*TEMP */
    .long 	RNG_IRQHandler		 /*RNG */
    .long 	ECB_IRQHandler		 /*ECB */
    .long 	CCM_AAR_IRQHandler		 /*CCM_AAR */
    .long 	WDT_IRQHandler		 /*WDT */
    .long 	RTC1_IRQHandler		 /*RTC1 */
    .long 	QDEC_IRQHandler		 /*QDEC */
    .long 	WUCOMP_COMP_IRQHandler		 /*WUCOMP_COMP */
    .long 	SWI0_IRQHandler		 /*SWI0 */
    .long 	SWI1_IRQHandler		 /*SWI1 */
    .long 	SWI2_IRQHandler		 /*SWI2 */
    .long 	SWI3_IRQHandler		 /*SWI3 */
    .long 	SWI4_IRQHandler		 /*SWI4 */
    .long 	SWI5_IRQHandler		 /*SWI5 */
    .long 	0		 /*Reserved */
    .long 	0		 /*Reserved */
    .long 	0		 /*Reserved */
    .long 	0		 /*Reserved */
    .long 	0		 /*Reserved */
    .long 	0		 /*Reserved */


    .size   __cs3_interrupt_vector_cortex_m, . - __cs3_interrupt_vector_cortex_m

    .thumb

/* Reset Handler */

    .equ    NRF_POWER_RAMON_ADDRESS,            0x40000524
    .equ    NRF_POWER_RAMON_RAMxON_ONMODE_Msk,  0xF  

    .section .cs3.reset,"x",%progbits
    .thumb_func
    .globl  __cs3_reset_cortex_m
    .type   __cs3_reset_cortex_m, %function
__cs3_reset_cortex_m:
    .fnstart
    LDR     R0, =NRF_POWER_RAMON_ADDRESS
    LDR     R2, [R0]
    MOV     R1, #NRF_POWER_RAMON_RAMxON_ONMODE_Msk
    ORR     R2, R1
    STR     R2, [R0]
    LDR     R0, =SystemInit
    BLX     R0
    LDR     R0, =__cs3_start_asm
    BX      R0

    .pool
    .cantunwind
    .fnend
    .size   __cs3_reset_cortex_m,.-__cs3_reset_cortex_m

    .section ".text"


/* Dummy Exception Handlers (infinite loops which can be modified) */

    .weak   NMI_Handler
    .type   NMI_Handler, %function
NMI_Handler:
    B       .
    .size   NMI_Handler, . - NMI_Handler


    .weak   HardFault_Handler
    .type   HardFault_Handler, %function
HardFault_Handler:
    B       .
    .size   HardFault_Handler, . - HardFault_Handler


    .weak   SVC_Handler
    .type   SVC_Handler, %function
SVC_Handler:
    B       .
    .size   SVC_Handler, . - SVC_Handler


    .weak   PendSV_Handler
    .type   PendSV_Handler, %function
PendSV_Handler:
    B       .
    .size   PendSV_Handler, . - PendSV_Handler


    .weak   SysTick_Handler
    .type   SysTick_Handler, %function
SysTick_Handler:
    B       .
    .size   SysTick_Handler, . - SysTick_Handler


/* IRQ Handlers */

    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    B       .
    .size   Default_Handler, . - Default_Handler

    .macro  IRQ handler
    .weak   \handler
    .set    \handler, Default_Handler
    .endm

    IRQ  POWER_CLOCK_IRQHandler
    IRQ  RADIO_IRQHandler
    IRQ  UART0_IRQHandler
    IRQ  SPI0_TWI0_IRQHandler
    IRQ  SPI1_TWI1_IRQHandler
    IRQ  GPIOTE_IRQHandler
    IRQ  ADC_IRQHandler
    IRQ  TIMER0_IRQHandler
    IRQ  TIMER1_IRQHandler
    IRQ  TIMER2_IRQHandler
    IRQ  RTC0_IRQHandler
    IRQ  TEMP_IRQHandler
    IRQ  RNG_IRQHandler
    IRQ  ECB_IRQHandler
    IRQ  CCM_AAR_IRQHandler
    IRQ  WDT_IRQHandler
    IRQ  RTC1_IRQHandler
    IRQ  QDEC_IRQHandler
    IRQ  WUCOMP_COMP_IRQHandler
    IRQ  SWI0_IRQHandler
    IRQ  SWI1_IRQHandler
    IRQ  SWI2_IRQHandler
    IRQ  SWI3_IRQHandler
    IRQ  SWI4_IRQHandler
    IRQ  SWI5_IRQHandler


  .end

