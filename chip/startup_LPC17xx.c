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

/* LPC17xx Chip Interrupts */
weak void WDT_IRQHandler(void) alias (Default_Handler);
weak void TIMER0_IRQHandler(void) alias (Default_Handler);
weak void TIMER1_IRQHandler(void) alias (Default_Handler);
weak void TIMER2_IRQHandler(void) alias (Default_Handler);
weak void TIMER3_IRQHandler(void) alias (Default_Handler);
weak void UART0_IRQHandler(void) alias (Default_Handler);
weak void UART1_IRQHandler(void) alias (Default_Handler);
weak void UART2_IRQHandler(void) alias (Default_Handler);
weak void UART3_IRQHandler(void) alias (Default_Handler);
weak void PWM1_IRQHandler(void) alias (Default_Handler);
weak void I2C0_IRQHandler(void) alias (Default_Handler);
weak void I2C1_IRQHandler(void) alias (Default_Handler);
weak void I2C2_IRQHandler(void) alias (Default_Handler);
weak void SPI_IRQHandler(void) alias (Default_Handler);
weak void SSP0_IRQHandler(void) alias (Default_Handler);
weak void SSP1_IRQHandler(void) alias (Default_Handler);
weak void PLL0_IRQHandler(void) alias (Default_Handler);
weak void RTC_IRQHandler(void) alias (Default_Handler);
weak void EINT0_IRQHandler(void) alias (Default_Handler);
weak void EINT1_IRQHandler(void) alias (Default_Handler);
weak void EINT2_IRQHandler(void) alias (Default_Handler);  
weak void EINT3_IRQHandler(void) alias (Default_Handler);
weak void ADC_IRQHandler(void) alias (Default_Handler);
weak void BOD_IRQHandler(void) alias (Default_Handler);
weak void USB_IRQHandler(void) alias (Default_Handler);
weak void CAN_IRQHandler(void) alias (Default_Handler);
weak void DMA_IRQHandler(void) alias (Default_Handler);
weak void I2S_IRQHandler(void) alias (Default_Handler);
weak void ENET_IRQHandler(void) alias (Default_Handler);
weak void RIT_IRQHandler(void) alias (Default_Handler);
weak void MCPWM_IRQHandler(void) alias (Default_Handler);
weak void QEI_IRQHandler(void) alias (Default_Handler);
weak void PLL1_IRQHandler(void) alias (Default_Handler);
weak void USBActivity_IRQHandler(void) alias (Default_Handler);
weak void CANActivity_IRQHandler(void) alias (Default_Handler);

/* This is defined in the linker script */
extern void __StackLimit(void);

/*
 * This array of interrupt vectors is decared in a special section so that the
 * linker script can position it at 0x00000000.
 */
__attribute__ ((section(".isr_vector")))
const void *isr_vectors[] = {
  /* Cortex-M3 Core Interrupts */
  &__StackLimit,	  // The end of the stack.
  Reset_Handler,	  // The Reset handler
  NMI_Handler,		  // The NMI handler
  HardFault_Handler,	  // The Hard Fault Handler
  MemManage_Handler,	  // The MPU Fault Handler
  BusFault_Handler,	  // The Bus Fault Handler
  UsageFault_Handler,     // The Usage Fault Handler
  0,			  // Reserved
  0,			  // Reserved
  0,			  // Reserved
  0,			  // Reserved
  SVC_Handler,		  // SVCall Handler
  DebugMon_Handler,	  // Debug Monitor Handler
  0,			  // Reserved
  PendSV_Handler,	  // The PendSV Handler
  SysTick_Handler,	  // The SysTick Handler

  /* LPC17xx Chip Interrupts */
  WDT_IRQHandler,	  // WDT
  TIMER0_IRQHandler,	  // Timer 0
  TIMER1_IRQHandler,	  // Timer 1
  TIMER2_IRQHandler,	  // Timer 2
  TIMER3_IRQHandler,	  // Timer 3
  UART0_IRQHandler,	  // UART0
  UART1_IRQHandler,	  // UART1
  UART2_IRQHandler,	  // UART2
  UART3_IRQHandler,	  // UART3
  PWM1_IRQHandler,	  // PWM1
  I2C0_IRQHandler,	  // I2C0
  I2C1_IRQHandler,	  // I2C1
  I2C2_IRQHandler,	  // I2C2
  SPI_IRQHandler,	  // SPI
  SSP0_IRQHandler,	  // SSP0
  SSP1_IRQHandler,	  // SSP1
  PLL0_IRQHandler,	  // PLL0 (Main PLL)
  RTC_IRQHandler,	  // RTC
  EINT0_IRQHandler,	  // External Interrupt
  EINT1_IRQHandler,	  // External Interrupt
  EINT2_IRQHandler,	  // External Interrupt
  EINT3_IRQHandler,	  // External Interrupt
  ADC_IRQHandler,	  // ADC
  BOD_IRQHandler,	  // BOD
  USB_IRQHandler,	  // USB
  CAN_IRQHandler,	  // CAN
  DMA_IRQHandler,	  // GPDMA
  I2S_IRQHandler,	  // I2S
  ENET_IRQHandler,	  // Ethernet
  RIT_IRQHandler,	  // Repetitive Interrupt Timer
  MCPWM_IRQHandler,	  // Motor Control PWM
  QEI_IRQHandler,	  // Quadrature Encoder
  PLL1_IRQHandler,	  // PLL1 (USB PLL)
  USBActivity_IRQHandler, // USB Activity Interrupt
  CANActivity_IRQHandler, // CAN Activity Interrupt
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
