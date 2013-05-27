/* 
 * Contains the entry point to the project, and defines the ISR vectors
 * Copyright (C) 2013  Richard Meadows
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

/* Cortex-M0 Core Interrupts */
weak void Reset_Handler(void);
weak void NMI_Handler(void) alias (Default_Handler);
weak void HardFault_Handler(void);
weak void SVCall_Handler(void) alias (Default_Handler);
weak void PendSV_Handler(void) alias (Default_Handler);
weak void SysTick_Handler(void) alias (Default_Handler);

/* LPC11xx Chip Interrupts */
weak void WAKEUP_IRQHandler(void) alias (Default_Handler);
weak void CAN_IRQHandler(void) alias (Default_Handler);
weak void SSP1_IRQHandler(void) alias (Default_Handler);
weak void I2C_IRQHandler(void) alias (Default_Handler);
weak void TIMER16_0_IRQHandler(void) alias (Default_Handler);
weak void TIMER16_1_IRQHandler(void) alias (Default_Handler);
weak void TIMER32_0_IRQHandler(void) alias (Default_Handler);
weak void TIMER32_1_IRQHandler(void) alias (Default_Handler);
weak void SSP0_IRQHandler(void) alias (Default_Handler);
weak void UART_IRQHandler(void) alias (Default_Handler);
weak void ADC_IRQHandler(void) alias (Default_Handler);
weak void WDT_IRQHandler(void) alias (Default_Handler);
weak void BOD_IRQHandler(void) alias (Default_Handler);
weak void PIOINT0_IRQHandler(void) alias (Default_Handler);
weak void PIOINT1_IRQHandler(void) alias (Default_Handler);
weak void PIOINT2_IRQHandler(void) alias (Default_Handler);
weak void PIOINT3_IRQHandler(void) alias (Default_Handler);

/* This is defined in the linker script */
extern void __StackLimit(void);

/*
 * This array of interrupt vectors is declared in a special section so that the
 * linker script can position it at 0x00000000.
 */
__attribute__ ((section(".isr_vector")))
const void *isr_vectors[] = {
  /* Cortex-M0 Core Interrupts */
  &__StackLimit,         // The end of the stack.
  Reset_Handler,         // The Reset Handler
  NMI_Handler,           // The NMI Handler
  HardFault_Handler,     // The Hard Fault Handler
  0,                     // Reserved
  0,                     // Reserved
  0,                     // Reserved
  0,                     // Reserved
  0,                     // Reserved
  0,                     // Reserved
  0,                     // Reserved
  SVCall_Handler,        // SVCall Handler
  0,                     // Reserved
  0,                     // Reserved
  PendSV_Handler,        // The PendSV Handler
  SysTick_Handler,       // The SysTick Handler
  /* LPC11xx Chip Interrupts */
  WAKEUP_IRQHandler,     // PIO0_0 Wakeup
  WAKEUP_IRQHandler,     // PIO0_1 Wakeup
  WAKEUP_IRQHandler,     // PIO0_2 Wakeup
  WAKEUP_IRQHandler,     // PIO0_3 Wakeup
  WAKEUP_IRQHandler,     // PIO0_4 Wakeup
  WAKEUP_IRQHandler,     // PIO0_5 Wakeup
  WAKEUP_IRQHandler,     // PIO0_6 Wakeup
  WAKEUP_IRQHandler,     // PIO0_7 Wakeup
  WAKEUP_IRQHandler,     // PIO0_8 Wakeup
  WAKEUP_IRQHandler,     // PIO0_9 Wakeup
  WAKEUP_IRQHandler,     // PIO0_10 Wakeup
  WAKEUP_IRQHandler,     // PIO0_11 Wakeup
  WAKEUP_IRQHandler,     // PIO1_0 Wakeup
  CAN_IRQHandler,        // C_CAN Interrupt
  SSP1_IRQHandler,       // SPI/SSP1 Interrupt
  I2C_IRQHandler,        // I2C0
  TIMER16_0_IRQHandler,  // CT16B0 (16-bit Timer 0)
  TIMER16_1_IRQHandler,  // CT16B1 (16-bit Timer 1)
  TIMER32_0_IRQHandler,  // CT32B0 (32-bit Timer 0)
  TIMER32_1_IRQHandler,  // CT32B1 (32-bit Timer 1)
  SSP0_IRQHandler,       // SPI/SSP0 Interrupt
  UART_IRQHandler,       // UART0
  0,                     // Reserved
  0,                     // Reserved
  ADC_IRQHandler,        // ADC   (A/D Converter)
  WDT_IRQHandler,        // WDT   (Watchdog Timer)
  BOD_IRQHandler,        // BOD   (Brownout Detect)
  0,                     // Reserved
  PIOINT3_IRQHandler,    // PIO INT3
  PIOINT2_IRQHandler,    // PIO INT2
  PIOINT1_IRQHandler,    // PIO INT1
  PIOINT0_IRQHandler,    // PIO INT0
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
/* HardFault hander */
void HardFault_Handler(void) {
  /* Drat! A HardFault */
  
  while(1);
}
