/**************************************************************************//**
 * @file     system_LPC17xx.c
 * @brief    CMSIS Cortex-M3 Device Peripheral Access Layer Source File for
 *           Device LPC17xx
 * @version  V3.10
 * @date     23. November 2012
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2012 ARM LIMITED
   Copyright (c) 2013 Richard Meadows - Added code for LPC17xx

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


#include <stdint.h>
#include "LPC17xx.h"

/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define __XTAL            (12000000UL)    /* Oscillator frequency             */
#define __SYS_OSC_CLK     (    __XTAL)    /* Main oscillator frequency        */
#define __IRC             ( 4000000UL)    /* Internal RC Oscillator           */
#define __RTC             (   32768UL)    /* Real Time Clock Oscillator       */

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __XTAL;    /*!< System Clock Frequency (Core Clock)*/

/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
int PLL0InputClock(void) {
  /* Switch based on the System Clock Select */
  /* See User Manual §4.4 Clock source selection multiplexer */
  switch (LPC_SC->CLKSRCSEL & 0x3) {
    case 0x0: return __IRC;
    case 0x1: return __SYS_OSC_CLK;
    case 0x2: return __RTC;
    default: return 0;
  }
}
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  SystemCoreClock = PLL0InputClock();

  /* If PLL0 is connected and enabled, account for the PLL */
  /* See User Manual §4.5.5 PLL0 Status register */
  if ((LPC_SC->PLL0STAT & 0x03000000) == 0x03000000) {
    uint16_t m = (LPC_SC->PLL0STAT & 0x00007FFF) + 1;
    uint8_t n = ((LPC_SC->PLL0STAT & 0x00FF0000) >> 16) + 1;

    SystemCoreClock *= 2 * m;
    SystemCoreClock /= n;
  }

  /* Divide by the CPU Clock Divider */
  /* See User Manual §4.7.1 CPU Clock Configuration */
  SystemCoreClock /= ((LPC_SC->CCLKCFG & 0xFF) + 1);
}

/* Enables (but doesn't switch to) PLL0 */
void EnablePLL0(uint16_t m, uint8_t n) {
  __disable_irq();

  LPC_SC->PLL0CFG = ((n-1) << 16) | (m-1);
  LPC_SC->PLL0FEED = 0xAA;
  LPC_SC->PLL0FEED = 0x55;

  LPC_SC->PLL0CON = 0x01;
  LPC_SC->PLL0FEED = 0xAA;
  LPC_SC->PLL0FEED = 0x55;

  __enable_irq();
}
/* Switches to PLL0 */
void SwitchToPLL0(void) {
  /* Wait for the PLL to lock */
  while(!(LPC_SC->PLL0STAT & 0x04000000));

  __disable_irq();

  LPC_SC->PLL0CON = 0x03;
  LPC_SC->PLL0FEED = 0xAA;
  LPC_SC->PLL0FEED = 0x55;

  __enable_irq();
}  

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
#ifdef USE_EXTERNAL_XTAL
  /* Enable the Main Oscillator */
  LPC_SC->SCS |= 0x0020;

  /* Wait for it to stabilise */
  while ((LPC_SC->SCS & 0x0040) == 0);
#endif

  /* Divide pllclk by 4 to get the CPU clock */
  LPC_SC->CCLKCFG = 0x03;

#ifdef USE_EXTERNAL_XTAL
  /* Set the Clock Source for PLL0 to the Main Oscillator */
  LPC_SC->CLKSRCSEL = 0x01;
#endif

  /* Set up PLL0 to multiply by 33.333.. */
  EnablePLL0(100, 6);

  /* And switch to PLL0 */
  SwitchToPLL0();

  /*
   * The core is now running at 8.333.. times the XTAL frequency.
   * If the crystal is 12MHz then this frequency is 100MHz.
   * If the IRC is still being used this is 33.333...MHz.
   */
}
