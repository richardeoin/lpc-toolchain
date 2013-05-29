/**************************************************************************//**
 * @file     system_LPC11xx.c
 * @brief    CMSIS Cortex-M0 Device Peripheral Access Layer Source File for
 *           Device LPC11xx
 * @version  V3.10
 * @date     23. November 2012
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2012 ARM LIMITED
   Copyright (c) 2013 Richard Meadows - Added code for LPC11xx

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
#include "LPC11xx.h"


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define __XTAL            (12000000UL)    /* External Crystal frequency       */
#define __SYS_OSC_CLK     (    __XTAL)    /* Main Oscillator frequency        */
#define __IRC             (12000000UL)    /* Internal RC Oscillator           */


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
/* After a call to SystemInit() the Clock will be comming from the crystal  */
uint32_t SystemCoreClock = __XTAL;  /*!< System Clock Frequency (Core Clock)*/


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
int SystemPLLInputClock(void) {              /* Get the frequency of PLL I/P  */
  switch (LPC_SYSCON->SYSPLLCLKSEL & 0x03) {
    /* Set User Manual §3.5.9 System PLL Clock Source Control Register */
    case 0x0: return __IRC; /* IRC Oscillator */
    case 0x1: return __XTAL; /* System Oscillator */
    default: return 0; /* Reserved */
  }
}
int SystemPLLOutputClock(void) {             /* Get the frequency of PLL O/P  */
  /* See User Manual §3.5.3 System PLL Control Register */
  return ((LPC_SYSCON->SYSPLLCTRL & 0x1F) + 1) * SystemPLLInputClock();
}
int WatchdogFClkAna(void) {                  /* Get the W. Osc Analog O/P     */
  /* See User Manual §3.5.6 Watchdog oscillator control register */
  switch ((LPC_SYSCON->WDTOSCCTRL & 0x01E0) >> 5) {
    case 0x1: return 500000;
    case 0x2: return 800000;
    case 0x3: return 1100000;
    case 0x4: return 1400000;
    case 0x5: return 1600000;
    case 0x6: return 1800000;
    case 0x7: return 2000000;
    case 0x8: return 2200000;
    case 0x9: return 2400000;
    case 0xA: return 2600000;
    case 0xB: return 2700000;
    case 0xC: return 2900000;
    case 0xD: return 3100000;
    case 0xE: return 3200000;
    case 0xF: return 3400000;
    default: return 0;
  }
}
int WatchdogOscClock(void) {                 /* Get the frequency of W. Osc   */
  /* See User Manual §3.5.6 Watchdog oscillator control register */
  return WatchdogFClkAna() * (((LPC_SYSCON->WDTOSCCTRL & 0x1F) + 1) * 2);
}
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  /* See LPC11xx User Manual §3.4 Clock Generation for more details */

  /* Switch based on the Main Clock Selection */
  switch (LPC_SYSCON->MAINCLKSEL & 0x03) {
    case 0x0: /* IRC Oscillator */
      SystemCoreClock = __IRC; break;
    case 0x1: /* Input clock to System PLL */
      SystemCoreClock = SystemPLLInputClock(); break;
    case 0x2: /* Watchdog Oscillator */
      SystemCoreClock = WatchdogOscClock(); break;
    case 0x3: /* System PLL Clock Out */
      SystemCoreClock = SystemPLLOutputClock(); break;
  }

  /* Account for the System Clock Divider */
  /* See User Manual §3.5.13 System AHB Clock Register */
  if ((LPC_SYSCON->SYSAHBCLKDIV & 0xFF) > 0) {
    SystemCoreClock /= (LPC_SYSCON->SYSAHBCLKDIV & 0xFF);
  } else {
    SystemCoreClock = 0; /* System Clock Disabled */
  }
}

/* Initialises and switches to the external XTAL */
void InitExternalOscillator(void) {
  /* Power up the System Oscillator */
  LPC_SYSCON->PDRUNCFG &= ~0x0020;

  /* Set the appropriate frequency range */
#if __XTAL < 20000000
  LPC_SYSCON->SYSOSCCTRL = 0x0;
#else
  LPC_SYSCON->SYSOSCCTRL = 0x2;
#endif

  /* Wait about 16µS */
  for (uint32_t i = 0; i < 200; i++) { __NOP(); }

  /* Select the System Oscillator as the PLL Input Source */
  LPC_SYSCON->SYSPLLCLKSEL = 0x1;
  /* And toggle to update */
  LPC_SYSCON->SYSPLLCLKUEN = 0x0;
  LPC_SYSCON->SYSPLLCLKUEN = 0x1;
  while (!(LPC_SYSCON->SYSPLLCLKUEN & 0x1));

  /* Select the PLL Input as the Main Clock source */
  LPC_SYSCON->MAINCLKSEL = 0x1;
  /* And toggle to update */
  LPC_SYSCON->MAINCLKUEN = 0x0;
  LPC_SYSCON->MAINCLKUEN = 0x1;
  while(!(LPC_SYSCON->MAINCLKUEN & 0x1));
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
  /*
   * By default the chip is running from the IRC oscillator. If you
   * want to do something other than this then you should modify this
   * routine.
   */

  /* Enable the clock to the I/O Configuration Block */
  LPC_SYSCON->SYSAHBCLKCTRL |= 0x0100;
}
