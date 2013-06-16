/* 
 * Contains the entry point for the project, and defines the ISR vectors
 * Copyright (C) 2013 Richard Meadows
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * A function for each of the interrupt vectors is weakly defined with an alias
 * so it may be optionally overridden elsewhere in the project.
 */
#define alias(f) __attribute__ ((alias(#f)))
#define weak __attribute__ ((weak))

/* Cortex-M3 Core Interrupts */
weak void Reset_Handler(void);
weak void NMI_Handler(void) alias (Default_Handler);
weak void HardFault_Handler(void);
weak void MemManage_Handler(void) alias (Default_Handler);
weak void BusFault_Handler(void) alias (Default_Handler);
weak void UsageFault_Handler(void) alias (Default_Handler);
weak void SVC_Handler(void) alias (Default_Handler);
weak void DebugMon_Handler(void) alias (Default_Handler);
weak void PendSV_Handler(void) alias (Default_Handler);
weak void SysTick_Handler(void) alias (Default_Handler);

/* LPC18xx Chip Interrupts */
weak void DAC_IRQHandler(void) alias (Default_Handler);
weak void DMA_IRQHandler(void) alias (Default_Handler);
weak void FLASHEEPROM_IRQHandler(void) alias (Default_Handler);
weak void ETHERNET_IRQHandler(void) alias (Default_Handler);
weak void SDIO_IRQHandler(void) alias (Default_Handler);
weak void LCD_IRQHandler(void) alias (Default_Handler);
weak void USB0_IRQHandler(void) alias (Default_Handler);
weak void USB1_IRQHandler(void) alias (Default_Handler);
weak void SCT_IRQHandler(void) alias (Default_Handler);
weak void RITIMER_IRQHandler(void) alias (Default_Handler);
weak void TIMER0_IRQHandler(void) alias (Default_Handler);
weak void TIMER1_IRQHandler(void) alias (Default_Handler);
weak void TIMER2_IRQHandler(void) alias (Default_Handler);
weak void TIMER3_IRQHandler(void) alias (Default_Handler);
weak void MCPWM_IRQHandler(void) alias (Default_Handler);
weak void ADC0_IRQHandler(void) alias (Default_Handler);
weak void I2C0_IRQHandler(void) alias (Default_Handler);
weak void I2C1_IRQHandler(void) alias (Default_Handler);
weak void ADC1_IRQHandler(void) alias (Default_Handler);
weak void SSP0_IRQHandler(void) alias (Default_Handler);
weak void SSP1_IRQHandler(void) alias (Default_Handler);
weak void USART0_IRQHandler(void) alias (Default_Handler);
weak void UART1_IRQHandler(void) alias (Default_Handler);
weak void USART2_IRQHandler(void) alias (Default_Handler);
weak void USART3_IRQHandler(void) alias (Default_Handler);
weak void I2S0_IRQHandler(void) alias (Default_Handler);
weak void I2S1_IRQHandler(void) alias (Default_Handler);
weak void PIN_INT0_IRQHandler(void) alias (Default_Handler);
weak void PIN_INT1_IRQHandler(void) alias (Default_Handler);
weak void PIN_INT2_IRQHandler(void) alias (Default_Handler);
weak void PIN_INT3_IRQHandler(void) alias (Default_Handler);
weak void PIN_INT4_IRQHandler(void) alias (Default_Handler);
weak void PIN_INT5_IRQHandler(void) alias (Default_Handler);
weak void PIN_INT6_IRQHandler(void) alias (Default_Handler);
weak void PIN_INT7_IRQHandler(void) alias (Default_Handler);
weak void GINT0_IRQHandler(void) alias (Default_Handler);
weak void GINT1_IRQHandler(void) alias (Default_Handler);
weak void EVENTROUTER_IRQHandler(void) alias (Default_Handler);
weak void C_CAN1_IRQHandler(void) alias (Default_Handler);
weak void ATIMER_IRQHandler(void) alias (Default_Handler);
weak void RTC_IRQHandler(void) alias (Default_Handler);
weak void WWDT_IRQHandler(void) alias (Default_Handler);
weak void C_CAN0_IRQHandler(void) alias (Default_Handler);
weak void QEI_IRQHandler(void) alias (Default_Handler);

/* This is defined in the linker script */
extern void __StackLimit(void);

/*
 * This array of interrupt vectors is decared in a special section so that the
 * linker script can position it at 0x00000000.
 */
__attribute__ ((section(".isr_vector")))
const void *isr_vectors[] = {
  /* Cortex-M3 Core Interrupts */
  &__StackLimit,	// The end of the stack.
  Reset_Handler,	// The Reset handler
  NMI_Handler,		// The NMI handler
  HardFault_Handler,	// The Hard Fault Handler
  MemManage_Handler,	// The MPU Fault Handler
  BusFault_Handler,	// The Bus Fault Handler
  UsageFault_Handler,	// The Usage Fault Handler
  0,			// Reserved
  0,			// Reserved
  0,			// Reserved
  0,			// Reserved
  SVC_Handler,		// SVCall Handler
  DebugMon_Handler,	// Debug Monitor Handler
  0,			// Reserved
  PendSV_Handler,	// The PendSV Handler
  SysTick_Handler,	// The SysTick Handler

  /* LPC18xx Chip Interrupts */
  DAC_IRQHandler,
  0,
  DMA_IRQHandler,
  0,
  FLASHEEPROM_IRQHandler,
  ETHERNET_IRQHandler,
  SDIO_IRQHandler,
  LCD_IRQHandler,
  USB0_IRQHandler,
  USB1_IRQHandler,
  SCT_IRQHandler,
  RITIMER_IRQHandler,
  TIMER0_IRQHandler,
  TIMER1_IRQHandler,
  TIMER2_IRQHandler,
  TIMER3_IRQHandler,
  MCPWM_IRQHandler,
  ADC0_IRQHandler,
  I2C0_IRQHandler,
  I2C1_IRQHandler,
  0,
  ADC1_IRQHandler,
  SSP0_IRQHandler,
  SSP1_IRQHandler,
  USART0_IRQHandler,
  UART1_IRQHandler,
  USART2_IRQHandler,
  USART3_IRQHandler,
  I2S0_IRQHandler,
  I2S1_IRQHandler,
  0,
  0,
  PIN_INT0_IRQHandler,
  PIN_INT1_IRQHandler,
  PIN_INT2_IRQHandler,
  PIN_INT3_IRQHandler,
  PIN_INT4_IRQHandler,
  PIN_INT5_IRQHandler,
  PIN_INT6_IRQHandler,
  PIN_INT7_IRQHandler,
  GINT0_IRQHandler,
  GINT1_IRQHandler,
  EVENTROUTER_IRQHandler,
  C_CAN1_IRQHandler,
  0,
  0,
  ATIMER_IRQHandler,
  RTC_IRQHandler,
  0,
  WWDT_IRQHandler,
  0,
  C_CAN0_IRQHandler,
  QEI_IRQHandler,
};

/* These are defined in the linker script */
extern unsigned int __etext;
extern unsigned int __data_start__;
extern unsigned int __data_end__;
extern unsigned int __bss_start__;
extern unsigned int __bss_end__;

extern int main(void);

/* The entry point to our program */
__attribute__ ((naked))
void Reset_Handler (void) {

  /* Load constants / initial values */
  for (unsigned int *source = &__etext, *destination = &__data_start__;
       destination < &__data_end__; )
    *destination++ = *source++;

  /* Zero out bss data */
  for (unsigned int *destination = &__bss_start__;
       destination < &__bss_end__; )
    *destination++ = 0;

  main();
  
  /* Wait here forever so the chip doesn't go haywire */
  while (1);
}
/* Default handler for undefined interrupts */
void Default_Handler(void) {
  /* You shouldn't have got here! There's been an undefined
     interrupt triggered somewhere.. */

  while(1);
}
/* HardFault Handler */
void HardFault_Handler(void) {
  /* Drat! A HardFault */

  while(1);
}
