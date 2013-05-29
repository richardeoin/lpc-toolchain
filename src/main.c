/* 
 * Demo C Application: Toggles an output at 20Hz.
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

#include <LPC11xx.h>

int main (void) {
  SystemInit();

  /* Update the value of SystemCoreClock */
  SystemCoreClockUpdate();

  /* Set an LED output on P0[7]*/
  LPC_GPIO0->DIR |= 1 << 7;

  /* Configure the SysTick for 50ms interrupts */
  SysTick_Config(SystemCoreClock / 20);
}

extern void SysTick_Handler(void) {
  /* Toggle an LED */
  LPC_GPIO0->DATA ^= 1 << 7;
}
