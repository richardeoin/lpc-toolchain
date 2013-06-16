/**************************************************************************//**
 * @file     LPC18xx.h
 * @brief    CMSIS Cortex-M3 Core Peripheral Access Layer Header File for
 *           Device LPC18xx
 * @version  V3.10
 * @date     23. November 2012
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2012 ARM LIMITED
   Copyright (c) 2013 Richard Meadows - Added code for LPC18xx

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


#ifndef LPC18xx_H
#define LPC18xx_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup LPC18xx_Definitions LPC18xx Definitions
  This file defines all structures and symbols for LPC18xx:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/


/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup LPC18xx_CMSIS Device CMSIS Definitions
  Configuration of the Cortex-M3 Processor and Core Peripherals
  @{
*/

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers *********************************/

  NonMaskableInt_IRQn           = -14,      /*!<  2 Non Maskable Interrupt      */
  MemoryManagement_IRQn         = -12,      /*!<  4 Memory Management Interrupt */
  BusFault_IRQn                 = -11,      /*!<  5 Bus Fault Interrupt         */
  UsageFault_IRQn               = -10,      /*!<  6 Usage Fault Interrupt       */
  SVCall_IRQn                   = -5,       /*!< 11 SV Call Interrupt           */
  DebugMonitor_IRQn             = -4,       /*!< 12 Debug Monitor Interrupt     */
  PendSV_IRQn                   = -2,       /*!< 14 Pend SV Interrupt           */
  SysTick_IRQn                  = -1,       /*!< 15 System Tick Interrupt       */

/******  Device Specific Interrupt Numbers **************************************/

  DAC_IRQn			= 0,
  Reserved0_IRQn		= 1,
  DMA_IRQn			= 2,
  Reserved1_IRQn		= 3,
  FLASHEEPROM_IRQn		= 4,	/* ORed flash A, flash B, EEPROM        */
  ETHERNET_IRQn			= 5,	/* Ethernet interrupt                   */
  SDIO_IRQn			= 6,	/* SD/MMC interrupt                     */
  LCD_IRQn			= 7,
  USB0_IRQn			= 8,	/* OTG interrupt                        */
  USB1_IRQn			= 9,	/* USB1 AHB_NEED_CLK                    */
  SCT_IRQn			= 10,	/* SCT combined interrupt               */
  RITIMER_IRQn			= 11,
  TIMER0_IRQn			= 12,
  TIMER1_IRQn			= 13,
  TIMER2_IRQn			= 14,
  TIMER3_IRQn			= 15,
  MCPWM_IRQn			= 16,	/* Motor control PWM                    */
  ADC0_IRQn			= 17,
  I2C0_IRQn			= 18,
  I2C1_IRQn			= 19,
  Reserved2_IRQn		= 20,
  ADC1_IRQn			= 21,
  SSP0_IRQn			= 22,
  SSP1_IRQn			= 23,
  USART0_IRQn			= 24,
  UART1_IRQn			= 25,	/* UART and modem interrupt             */
  USART2_IRQn			= 26,
  USART3_IRQn			= 27,	/* USART and IrDA interrupt             */
  I2S0_IRQn			= 28,
  I2S1_IRQn			= 29,
  Reserved3_IRQn		= 30,
  Reserved4_IRQn		= 31,
  PIN_INT0_IRQn			= 32,	/* GPIO pin interrupt 0                 */
  PIN_INT1_IRQn			= 33,	/* GPIO pin interrupt 1                 */
  PIN_INT2_IRQn			= 34,	/* GPIO pin interrupt 2                 */
  PIN_INT3_IRQn			= 35,	/* GPIO pin interrupt 3                 */
  PIN_INT4_IRQn			= 36,	/* GPIO pin interrupt 4                 */
  PIN_INT5_IRQn			= 37,	/* GPIO pin interrupt 5                 */
  PIN_INT6_IRQn			= 38,	/* GPIO pin interrupt 6                 */
  PIN_INT7_IRQn			= 39,	/* GPIO pin interrupt 7                 */
  GINT0_IRQn			= 40,	/* GPIO global interrupt 0              */
  GINT1_IRQn			= 41,	/* GPIO global interrupt 1              */
  EVENTROUTER_IRQn		= 42,	/* Interrupt from the event router      */
  C_CAN1_IRQn			= 43,
  Reserved5_IRQn		= 44,
  Reserved6_IRQn		= 45,
  ATIMER_IRQn			= 46,	/* Alarm timer interrupt                */
  RTC_IRQn			= 47,	/* Combined RTC and event router/monitor*/
  Reserved7_IRQn		= 48,
  WWDT_IRQn			= 49,
  Reserved8_IRQn		= 50,
  C_CAN0_IRQn			= 51,
  QEI_IRQn			= 52,
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __CM3_REV                 0x0201    /*!< Core Revision r2p1             */
#define __NVIC_PRIO_BITS          4         /*!< Number of Bits used for Prio   */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick  */
#define __MPU_PRESENT             1         /*!< MPU present or not             */

/*@}*/ /* end of group LPC18xx_CMSIS */


#include <core_cm3.h>                       /* Cortex-M3 processor and core perips */
#include "system_LPC18xx.h"                 /* LPC18xx System include file         */


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup LPC18xx_Peripherals LPC18xx Peripherals
  LPC18xx Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/*------------- RGU (RGU) ----------------------------*/
/** @addtogroup LPC18xx_RGU LPC18xx RGU (RGU) 
  @{
*/
typedef struct
{
	uint32_t RESERVED0[64];
  __O	uint32_t CTRL0;		// Reset control register 0
  __O	uint32_t CTRL1;		// Reset control register 1
	uint32_t RESERVED1[2];
  __IO	uint32_t STATUS0;		// Reset status register 0
  __IO	uint32_t STATUS1;		// Reset status register 1
  __IO	uint32_t STATUS2;		// Reset status register 2
  __IO	uint32_t STATUS3;		// Reset status register 3
	uint32_t RESERVED2[12];
  __I	uint32_t ACTIVE_STATUS0;	// Reset active status register 0
  __I	uint32_t ACTIVE_STATUS1;	// Reset active status register 1
	uint32_t RESERVED3[170];
  __IO	uint32_t EXT_STAT0;		// Reset external status register 0 for
  __IO	uint32_t EXT_STAT1;		// Reset external status register 1 for
  __IO	uint32_t EXT_STAT2;		// Reset external status register 2 for
  __IO	uint32_t EXT_STAT3;		// Reserved
  __IO	uint32_t EXT_STAT4;		// Reset external status register 4 for
  __IO	uint32_t EXT_STAT5;		// Reset external status register 5 for
  __IO	uint32_t EXT_STAT6;		// Reserved
  __IO	uint32_t EXT_STAT7;		// Reserved
  __IO	uint32_t EXT_STAT8;		// Reset external status register 8 for
  __IO	uint32_t EXT_STAT9;		// Reset external status register 9 for
  __IO	uint32_t EXT_STAT10;		// Reserved
  __IO	uint32_t EXT_STAT11;		// Reserved
  __IO	uint32_t EXT_STAT12;		// Reserved
  __IO	uint32_t EXT_STAT13;		// Reset external status register 13 for
  __IO	uint32_t EXT_STAT14;		// Reserved
  __IO	uint32_t EXT_STAT15;		// Reserved
  __IO	uint32_t EXT_STAT16;		// Reset external status register 16 for
  __IO	uint32_t EXT_STAT17;		// Reset external status register 17 for
  __IO	uint32_t EXT_STAT18;		// Reset external status register 18 for
  __IO	uint32_t EXT_STAT19;		// Reset external status register 19 for
  __IO	uint32_t EXT_STAT20;		// Reset external status register 20 for
  __IO	uint32_t EXT_STAT21;		// Reset external status register 21 for
  __IO	uint32_t EXT_STAT22;		// Reset external status register 22 for
  __IO	uint32_t EXT_STAT23;		// Reserved
  __IO	uint32_t EXT_STAT24;		// Reserved
  __IO	uint32_t EXT_STAT25;		// Reset external status register 25 for
  __IO	uint32_t EXT_STAT26;		// Reserved
  __IO	uint32_t EXT_STAT27;		// Reset external status register 27 for
  __IO	uint32_t EXT_STAT28;		// Reset external status register 28 for
  __IO	uint32_t EXT_STAT29;		// Reset external status register 29 for
  __IO	uint32_t EXT_STAT30;		// Reserved
  __IO	uint32_t EXT_STAT31;		// Reserved
  __IO	uint32_t EXT_STAT32;		// Reset external status register 32 for
  __IO	uint32_t EXT_STAT33;		// Reset external status register 33 for
  __IO	uint32_t EXT_STAT34;		// Reset external status register 34 for
  __IO	uint32_t EXT_STAT35;		// Reset external status register 35 for
  __IO	uint32_t EXT_STAT36;		// Reset external status register 36 for
  __IO	uint32_t EXT_STAT37;		// Reset external status register 37 for
  __IO	uint32_t EXT_STAT38;		// Reset external status register 38 for
  __IO	uint32_t EXT_STAT39;		// Reset external status register 39 for
  __IO	uint32_t EXT_STAT40;		// Reset external status register 40 for
  __IO	uint32_t EXT_STAT41;		// Reset external status register 41 for
  __IO	uint32_t EXT_STAT42;		// Reset external status register 42 for
  __IO	uint32_t EXT_STAT43;		// Reserved
  __IO	uint32_t EXT_STAT44;		// Reset external status register 44 for
  __IO	uint32_t EXT_STAT45;		// Reset external status register 45 for
  __IO	uint32_t EXT_STAT46;		// Reset external status register 46 for
  __IO	uint32_t EXT_STAT47;		// Reset external status register 47 for
  __IO	uint32_t EXT_STAT48;		// Reset external status register 48 for
  __IO	uint32_t EXT_STAT49;		// Reset external status register 49 for
  __IO	uint32_t EXT_STAT50;		// Reset external status register 50 for
  __IO	uint32_t EXT_STAT51;		// Reset external status register 51 for
  __IO	uint32_t EXT_STAT52;		// Reset external status register 52 for
  __IO	uint32_t EXT_STAT53;		// Reset external status register 53 for
  __IO	uint32_t EXT_STAT54;		// Reset external status register 54 for
  __IO	uint32_t EXT_STAT55;		// Reset external status register 55 for
  __IO	uint32_t EXT_STAT56;		// Reserved
  __IO	uint32_t EXT_STAT57;		// Reserved
  __IO	uint32_t EXT_STAT58;		// Reserved
  __IO	uint32_t EXT_STAT59;		// Reserved
  __IO	uint32_t EXT_STAT60;		// Reserved
  __IO	uint32_t EXT_STAT61;		// Reserved
  __IO	uint32_t EXT_STAT62;		// Reserved
  __IO	uint32_t EXT_STAT63;		// Reserved
} LPC_RGU_TypeDef;
/*@}*/ /* end of group LPC18xx_RGU */

/*------------- Event router (EVENTROUTER) ----------------------------*/
/** @addtogroup LPC18xx_EVENTROUTER LPC18xx Event router (EVENTROUTER) 
  @{
*/
typedef struct
{
  __IO	uint32_t HILO;		// Level configuration register
  __IO	uint32_t EDGE;		// Edge configuration
	uint32_t RESERVED0[1012];
  __O	uint32_t CLR_EN;		// Clear event enable register
  __O	uint32_t SET_EN;		// Set event enable register
  __I	uint32_t STATUS;		// Event Status register
  __I	uint32_t ENABLE;		// Event Enable register
  __O	uint32_t CLR_STAT;		// Clear event status register
  __O	uint32_t SET_STAT;		// Set event status register
} LPC_EVENTROUTER_TypeDef;
/*@}*/ /* end of group LPC18xx_EVENTROUTER */

/*------------- Configuration registers (CREG) ----------------------------*/
/** @addtogroup LPC18xx_CREG LPC18xx Configuration registers (CREG) 
  @{
*/
typedef struct
{
	uint32_t RESERVED0[1];
  __IO	uint32_t CREG0;		// Chip configuration register 32 kHz
	uint32_t RESERVED1[62];
  __IO	uint32_t M3MEMMAP;		// ARM Cortex-M3 memory mapping
	uint32_t RESERVED2[5];
  __IO	uint32_t CREG5;		// Chip configuration register 5
  __IO	uint32_t DMAMUX;		// DMA mux control register
  __IO	uint32_t FLASHCFGA;		// Flash accelerator configuration register 0x8000
  __IO	uint32_t FLASHCFGB;		// Flash accelerator configuration register 0x8000
  __IO	uint32_t ETBCFG;		// ETB RAM configuration
  __IO	uint32_t CREG6;		// Chip configuration register 6
	uint32_t RESERVED3[52];
  __I	uint32_t CHIPID;		// Part ID
	uint32_t RESERVED4[191];
  __IO	uint32_t USB0FLADJ;		// USB0 frame length adjust register
	uint32_t RESERVED5[63];
  __IO	uint32_t USB1FLADJ;		// USB1 frame length adjust register
} LPC_CREG_TypeDef;
/*@}*/ /* end of group LPC18xx_CREG */

/*------------- Power Mode Controller (PMC) (PMC) ----------------------------*/
/** @addtogroup LPC18xx_PMC LPC18xx Power Mode Controller (PMC) (PMC) 
  @{
*/
typedef struct
{
  __IO	uint32_t HW_ENA;		// Hardware sleep event
	uint32_t RESERVED0[6];
  __IO	uint32_t MODE;		// Power-down mode
} LPC_PMC_TypeDef;
/*@}*/ /* end of group LPC18xx_PMC */

/*------------- CGU (CGU) ----------------------------*/
/** @addtogroup LPC18xx_CGU LPC18xx CGU (CGU) 
  @{
*/
typedef struct
{
	uint32_t RESERVED0[5];
  __IO	uint32_t FREQ_MON;		// Frequency monitor register
  __IO	uint32_t XTAL_OSC_CTRL;		// Crystal oscillator control
  __I	uint32_t PLL0USB_STAT;		// PLL0USB status register
  __IO	uint32_t PLL0USB_CTRL;		// PLL0USB control register
  __IO	uint32_t PLL0USB_MDIV;		// PLL0USB M-divider register
  __IO	uint32_t PLL0USB_NP_DIV;	// PLL0USB N/P-divider register
  __I	uint32_t PLL0AUDIO_STAT;	// PLL0AUDIO status register
  __IO	uint32_t PLL0AUDIO_CTRL;	// PLL0AUDIO control register
  __IO	uint32_t PLL0AUDIO_MDIV;	// PLL0AUDIO M-divider register
  __IO	uint32_t PLL0AUDIO_NP_DIV;	// PLL0AUDIO N/P-divider
  __IO	uint32_t PLL0AUDIO_FRAC;	// PLL0AUDIO fractional divider
  __I	uint32_t PLL1_STAT;		// PLL1 status register
  __IO	uint32_t PLL1_CTRL;		// PLL1 control register
  __IO	uint32_t IDIVA_CTRL;		// Integer divider A control register 0x0100
  __IO	uint32_t IDIVB_CTRL;		// Integer divider B control register 0x0100
  __IO	uint32_t IDIVC_CTRL;		// Integer divider C control
  __IO	uint32_t IDIVD_CTRL;		// Integer divider D control
  __IO	uint32_t IDIVE_CTRL;		// Integer divider E control register 0x0100
  __I	uint32_t BASE_SAFE_CLK;		// Output stage 0 control register
  __IO	uint32_t BASE_USB0_CLK;		// Output stage 1 control register
	uint32_t RESERVED1[1];
  __IO	uint32_t BASE_USB1_CLK;		// Output stage 3 control register
  __IO	uint32_t BASE_M3_CLK;		// Output stage 4 control register
  __IO	uint32_t BASE_SPIFI_CLK;	// Output stage 5 control register
	uint32_t RESERVED2[1];
  __IO	uint32_t BASE_PHY_RX_CLK;	// Output stage 7 control register
  __IO	uint32_t BASE_PHY_TX_CLK;	// Output stage 8 control register
  __IO	uint32_t BASE_APB1_CLK;		// Output stage 9 control register
  __IO	uint32_t BASE_APB3_CLK;		// Output stage 10 control register 0x0100
  __IO	uint32_t BASE_LCD_CLK;		// Output stage 11 control register 0x0100
	uint32_t RESERVED3[1];
  __IO	uint32_t BASE_SDIO_CLK;		// Output stage 13 control register 0x0100
  __IO	uint32_t BASE_SSP0_CLK;		// Output stage 14 control register 0x0100
  __IO	uint32_t BASE_SSP1_CLK;		// Output stage 15 control register 0x0100
  __IO	uint32_t BASE_UART0_CLK;	// Output stage 16 control register 0x0100
  __IO	uint32_t BASE_UART1_CLK;	// Output stage 17 control register 0x0100
  __IO	uint32_t BASE_UART2_CLK;	// Output stage 18 control register 0x0100
  __IO	uint32_t BASE_UART3_CLK;	// Output stage 19 control register 0x0100
  __IO	uint32_t BASE_OUT_CLK;		// Output stage 20 control register 0x0100
	uint32_t RESERVED4[4];
  __IO	uint32_t BASE_APLL_CLK;		// Output stage 25 control register 0x0100
  __IO	uint32_t BASE_CGU_OUT0_CLK;	// Output stage 26 control register 0x0100
  __IO	uint32_t BASE_CGU_OUT1_CLK;	// Output stage 27 control register 0x0100
} LPC_CGU_TypeDef;
/*@}*/ /* end of group LPC18xx_CGU */

/*------------- CCU1 (CCU1) ----------------------------*/
/** @addtogroup LPC18xx_CCU1 LPC18xx CCU1 (CCU1) 
  @{
*/
typedef struct
{
  __IO	uint32_t PM;				// CCU1 power mode register
  __I	uint32_t BASE_STAT;			// CCU1 base clocks status register
	uint32_t RESERVED0[62];
  __IO	uint32_t CLK_APB3_BUS_CFG;		// CLK_APB3_BUS clock configuration
  __I	uint32_t CLK_APB3_BUS_STAT;		// CLK_APB3_BUS clock status register
  __IO	uint32_t CLK_APB3_I2C1_CFG;		// CLK_APB3_I2C1 configuration register
  __I	uint32_t CLK_APB3_I2C1_STAT;		// CLK_APB3_I2C1 status register
  __IO	uint32_t CLK_APB3_DAC_CFG;		// CLK_APB3_DAC configuration register
  __I	uint32_t CLK_APB3_DAC_STAT;		// CLK_APB3_DAC status register
  __IO	uint32_t CLK_APB3_ADC0_CFG;		// CLK_APB3_ADC0 configuration register
  __I	uint32_t CLK_APB3_ADC0_STAT;		// CLK_APB3_ADC0 status register
  __IO	uint32_t CLK_APB3_ADC1_CFG;		// CLK_APB3_ADC1 configuration register
  __I	uint32_t CLK_APB3_ADC1_STAT;		// CLK_APB3_ADC1 status register
  __IO	uint32_t CLK_APB3_CAN0_CFG;		// CLK_APB3_CAN0 configuration register
  __I	uint32_t CLK_APB3_CAN0_STAT;		// CLK_APB3_CAN0 status register
	uint32_t RESERVED1[52];
  __IO	uint32_t CLK_APB1_BUS_CFG;		// CLK_APB1_BUS configuration register
  __I	uint32_t CLK_APB1_BUS_STAT;		// CLK_APB1_BUS status register
  __IO	uint32_t CLK_APB1_MOTOCONPWM_CFG;	// CLK_APB1_MOTOCON configuration
  __I	uint32_t CLK_APB1_MOTOCONPWM_STAT;	// CLK_APB1_MOTOCON status register
  __IO	uint32_t CLK_APB1_I2C0_CFG;		// CLK_APB1_I2C0 configuration register
  __I	uint32_t CLK_APB1_I2C0_STAT;		// CLK_APB1_I2C0 status register
  __IO	uint32_t CLK_APB1_I2S_CFG;		// CLK_APB1_I2S configuration register
  __I	uint32_t CLK_APB1_I2S_STAT;		// CLK_APB1_I2S status register
  __IO	uint32_t CLK_APB1_CAN1_CFG;		// CLK_APB3_CAN1 configuration register
  __I	uint32_t CLK_APB1_CAN1_STAT;		// CLK_APB3_CAN1 status register
	uint32_t RESERVED2[54];
  __IO	uint32_t CLK_SPIFI_CFG;			// CLK_SPIFI configuration register
  __I	uint32_t CLK_SPIFI_STAT;		// CLK_SPIFI status register
	uint32_t RESERVED3[62];
  __IO	uint32_t CLK_M3_BUS_CFG;		// CLK_M3_BUS configuration register
  __I	uint32_t CLK_M3_BUS_STAT;		// CLK_M3_BUS status register
  __IO	uint32_t CLK_M3_SPIFI_CFG;		// CLK_M3_SPIFI configuration register
  __I	uint32_t CLK_M3_SPIFI_STAT;		// CLK_M3_SPIFI status register
  __IO	uint32_t CLK_M3_GPIO_CFG;		// CLK_M3_GPIO configuration register
  __I	uint32_t CLK_M3_GPIO_STAT;		// CLK_M3_GPIO status register
  __IO	uint32_t CLK_M3_LCD_CFG;		// CLK_M3_LCD configuration register
  __I	uint32_t CLK_M3_LCD_STAT;		// CLK_M3_LCD status register
  __IO	uint32_t CLK_M3_ETHERNET_CFG;		// CLK_M3_ETHERNET configuration
  __I	uint32_t CLK_M3_ETHERNET_STAT;		// CLK_M3_ETHERNET status register
  __IO	uint32_t CLK_M3_USB0_CFG;		// CLK_M3_USB0 configuration register
  __I	uint32_t CLK_M3_USB0_STAT;		// CLK_M3_USB0 status register
  __IO	uint32_t CLK_M3_EMC_CFG;		// CLK_M3_EMC configuration register
  __I	uint32_t CLK_M3_EMC_STAT;		// CLK_M3_EMC status register
  __IO	uint32_t CLK_M3_SDIO_CFG;		// CLK_M3_SDIO configuration register
  __I	uint32_t CLK_M3_SDIO_STAT;		// CLK_M3_SDIO status register
  __IO	uint32_t CLK_M3_DMA_CFG;		// CLK_M3_DMA configuration register
  __I	uint32_t CLK_M3_DMA_STAT;		// CLK_M3_DMA status register
  __IO	uint32_t CLK_M3_M3CORE_CFG;		// CLK_M3_M3CORE configuration register
  __I	uint32_t CLK_M3_M3CORE_STAT;		// CLK_M3_M3CORE status register
	uint32_t RESERVED4[6];
  __IO	uint32_t CLK_M3_SCT_CFG;		// CLK_M3_SCT configuration register
  __I	uint32_t CLK_M3_SCT_STAT;		// CLK_M3_SCT status register
  __IO	uint32_t CLK_M3_USB1_CFG;		// CLK_M3_USB1 configuration register
  __I	uint32_t CLK_M3_USB1_STAT;		// CLK_M3_USB1 status register
  __IO	uint32_t CLK_M3_EMCDIV_CFG;		// CLK_M3_EMCDIV configuration register
  __I	uint32_t CLK_M3_EMCDIV_STAT;		// CLK_M3_EMCDIV status register
  __IO	uint32_t CLK_M3_FLASHA_CFG;		// CLK_M3_FLASHA configuration register
  __I	uint32_t CLK_M3_FLASHA_STAT;		// CLK_M3_FLASHA status register
  __IO	uint32_t CLK_M3_FLASHB_CFG;		// CLK_M3_FLASHB configuration register
  __I	uint32_t CLK_M3_FLASHB_STAT;		// CLK_M3_FLASHB status register
	uint32_t RESERVED5[28];
  __IO	uint32_t CLK_M3_WWDT_CFG;		// CLK_M3_WWDT configuration register
  __I	uint32_t CLK_M3_WWDT_STAT;		// CLK_M3_WWDT status register
  __IO	uint32_t CLK_M3_USART0_CFG;		// CLK_M3_UART0 configuration register
  __I	uint32_t CLK_M3_USART0_STAT;		// CLK_M3_UART0 status register
  __IO	uint32_t CLK_M3_UART1_CFG;		// CLK_M3_UART1 configuration register
  __I	uint32_t CLK_M3_UART1_STAT;		// CLK_M3_UART1 status register
  __IO	uint32_t CLK_M3_SSP0_CFG;		// CLK_M3_SSP0 configuration register
  __I	uint32_t CLK_M3_SSP0_STAT;		// CLK_M3_SSP0 status register
  __IO	uint32_t CLK_M3_TIMER0_CFG;		// CLK_M3_TIMER0 configuration register
  __I	uint32_t CLK_M3_TIMER0_STAT;		// CLK_M3_TIMER0 status register
  __IO	uint32_t CLK_M3_TIMER1_CFG;		// CLK_M3_TIMER1 configuration register
  __I	uint32_t CLK_M3_TIMER1_STAT;		// CLK_M3_TIMER1 status register
  __IO	uint32_t CLK_M3_SCU_CFG;		// CLK_M3_SCU configuration register
  __I	uint32_t CLK_M3_SCU_STAT;		// CLK_M3_SCU status register
  __IO	uint32_t CLK_M3_CREG_CFG;		// CLK_M3_CREG configuration register
  __I	uint32_t CLK_M3_CREG_STAT;		// CLK_M3_CREG status register
	uint32_t RESERVED6[48];
  __IO	uint32_t CLK_M3_RITIMER_CFG;		// CLK_M3_RITIMER configuration register
  __I	uint32_t CLK_M3_RITIMER_STAT;		// CLK_M3_RITIMER status register
  __IO	uint32_t CLK_M3_USART2_CFG;		// CLK_M3_UART2 configuration register
  __I	uint32_t CLK_M3_USART2_STAT;		// CLK_M3_UART2 status register
  __IO	uint32_t CLK_M3_USART3_CFG;		// CLK_M3_UART3 configuration register
  __I	uint32_t CLK_M3_USART3_STAT;		// CLK_M3_UART3 status register
  __IO	uint32_t CLK_M3_TIMER2_CFG;		// CLK_M3_TIMER2 configuration register
  __I	uint32_t CLK_M3_TIMER2_STAT;		// CLK_M3_TIMER2 status register
  __IO	uint32_t CLK_M3_TIMER3_CFG;		// CLK_M3_TIMER3 configuration register
  __I	uint32_t CLK_M3_TIMER3_STAT;		// CLK_M3_TIMER3 status register
  __IO	uint32_t CLK_M3_SSP1_CFG;		// CLK_M3_SSP1 configuration register
  __I	uint32_t CLK_M3_SSP1_STAT;		// CLK_M3_SSP1 status register
  __IO	uint32_t CLK_M3_QEI_CFG;		// CLK_M3_QEI configuration register
  __I	uint32_t CLK_M3_QEI_STAT;		// CLK_M3_QEI status register
	uint32_t RESERVED7[114];
  __IO	uint32_t CLK_USB0_CFG;			// CLK_USB0 configuration register
  __I	uint32_t CLK_USB0_STAT;			// CLK_USB0 status register
	uint32_t RESERVED8[62];
  __IO	uint32_t CLK_USB1_CFG;			// CLK_USB1 configuration register
  __I	uint32_t CLK_USB1_STAT;			// CLK_USB1 status register
	uint32_t RESERVED9[126];
  __IO	uint32_t CLK_VADC_CFG;			// CLK_VADC configuration register
  __I	uint32_t CLK_VADC_STAT;			// CLK_VADC status register
} LPC_CCU1_TypeDef;
/*@}*/ /* end of group LPC18xx_CCU1 */

/*------------- CCU2 (CCU2) ----------------------------*/
/** @addtogroup LPC18xx_CCU2 LPC18xx CCU2 (CCU2) 
  @{
*/
typedef struct
{
  __IO	uint32_t PM;				// CCU2 power mode register
  __I	uint32_t BASE_STAT;			// CCU2 base clocks status register
	uint32_t RESERVED0[62];
  __IO	uint32_t CLK_APLL_CFG;			// CLK_APLL configuration register
  __I	uint32_t CLK_APLL_STAT;			// CLK_APLL status register
	uint32_t RESERVED1[62];
  __IO	uint32_t CLK_APB2_USART3_CFG;		// CLK_APB2_UART3 configuration register
  __I	uint32_t CLK_APB2_USART3_STAT;		// CLK_APB2_UART3 status register
	uint32_t RESERVED2[62];
  __IO	uint32_t CLK_APB2_USART2_CFG;		// CLK_APB2_UART2 configuration register
  __I	uint32_t CLK_APB2_USART2_STAT;		// CLK_APB2_UART2 status register
	uint32_t RESERVED3[62];
  __IO	uint32_t CLK_APB0_UART1_CFG;		// CLK_APB0_UART1 configuration register
  __I	uint32_t CLK_APB0_UART1_STAT;		// CLK_APB0_UART1 status register
	uint32_t RESERVED4[62];
  __IO	uint32_t CLK_APB0_USART0_CFG;		// CLK_APB0_UART0 configuration register
  __I	uint32_t CLK_APB0_USART0_STAT;		// CLK_APB0_UART0 status register
	uint32_t RESERVED5[62];
  __IO	uint32_t CLK_APB2_SSP1_CFG;		// CLK_APB2_SSP1 configuration register
  __I	uint32_t CLK_APB2_SSP1_STAT;		// CLK_APB2_SSP1 status register
	uint32_t RESERVED6[62];
  __IO	uint32_t CLK_APB0_SSP0_CFG;		// CLK_APB0_SSP0 configuration register
  __I	uint32_t CLK_APB0_SSP0_STAT;		// CLK_APB0_SSP0 status register
	uint32_t RESERVED7[62];
  __IO	uint32_t CLK_SDIO_CFG;		// CLK_SDIO configuration register
  __I	uint32_t CLK_SDIO_STAT;		// CLK_SDIO status register
} LPC_CCU2_TypeDef;
/*@}*/ /* end of group LPC18xx_CCU2 */

/*------------- System Control Unit (SCU) (SCU) ----------------------------*/
/** @addtogroup LPC18xx_SCU LPC18xx System Control Unit (SCU) (SCU) 
  @{
*/
typedef struct
{
  __IO	uint32_t SFSP0_0;		// Pin configuration register for pin P0_0
  __IO	uint32_t SFSP0_1;		// Pin configuration register for pin P0_1
	uint32_t RESERVED0[30];
  __IO	uint32_t SFSP1_0;		// Pin configuration register for pin P1_0
  __IO	uint32_t SFSP1_1;		// Pin configuration register for pin P1_1
  __IO	uint32_t SFSP1_2;		// Pin configuration register for pin P1_2
  __IO	uint32_t SFSP1_3;		// Pin configuration register for pin P1_3
  __IO	uint32_t SFSP1_4;		// Pin configuration register for pin P1_4
  __IO	uint32_t SFSP1_5;		// Pin configuration register for pin P1_5
  __IO	uint32_t SFSP1_6;		// Pin configuration register for pin P1_6
  __IO	uint32_t SFSP1_7;		// Pin configuration register for pin P1_7
  __IO	uint32_t SFSP1_8;		// Pin configuration register for pin P1_8
  __IO	uint32_t SFSP1_9;		// Pin configuration register for pin P1_9
  __IO	uint32_t SFSP1_10;		// Pin configuration register for pin P1_10
  __IO	uint32_t SFSP1_11;		// Pin configuration register for pin P1_11
  __IO	uint32_t SFSP1_12;		// Pin configuration register for pin P1_12
  __IO	uint32_t SFSP1_13;		// Pin configuration register for pin P1_13
  __IO	uint32_t SFSP1_14;		// Pin configuration register for pin P1_14
  __IO	uint32_t SFSP1_15;		// Pin configuration register for pin P1_15
  __IO	uint32_t SFSP1_16;		// Pin configuration register for pin P1_16
  __IO	uint32_t SFSP1_17;		// Pin configuration register for pin P1_17
  __IO	uint32_t SFSP1_18;		// Pin configuration register for pin P1_18
  __IO	uint32_t SFSP1_19;		// Pin configuration register for pin P1_19
  __IO	uint32_t SFSP1_20;		// Pin configuration register for pin P1_20
	uint32_t RESERVED1[11];
  __IO	uint32_t SFSP2_0;		// Pin configuration register for pin P2_0
  __IO	uint32_t SFSP2_1;		// Pin configuration register for pin P2_1
  __IO	uint32_t SFSP2_2;		// Pin configuration register for pin P2_2
  __IO	uint32_t SFSP2_3;		// Pin configuration register for pin P2_3
  __IO	uint32_t SFSP2_4;		// Pin configuration register for pin P2_4
  __IO	uint32_t SFSP2_5;		// Pin configuration register for pin P2_5
  __IO	uint32_t SFSP2_6;		// Pin configuration register for pin P2_6
  __IO	uint32_t SFSP2_7;		// Pin configuration register for pin P2_7
  __IO	uint32_t SFSP2_8;		// Pin configuration register for pin P2_8
  __IO	uint32_t SFSP2_9;		// Pin configuration register for pin P2_9
  __IO	uint32_t SFSP2_10;		// Pin configuration register for pin P2_10
  __IO	uint32_t SFSP2_11;		// Pin configuration register for pin P2_11
  __IO	uint32_t SFSP2_12;		// Pin configuration register for pin P2_12
  __IO	uint32_t SFSP2_13;		// Pin configuration register for pin P2_13
	uint32_t RESERVED2[18];
  __IO	uint32_t SFSP3_0;		// Pin configuration register for pin P3_0
  __IO	uint32_t SFSP3_1;		// Pin configuration register for pin P3_1
  __IO	uint32_t SFSP3_2;		// Pin configuration register for pin P3_2
  __IO	uint32_t SFSP3_3;		// Pin configuration register for pin P3_3
  __IO	uint32_t SFSP3_4;		// Pin configuration register for pin P3_4
  __IO	uint32_t SFSP3_5;		// Pin configuration register for pin P3_5
  __IO	uint32_t SFSP3_6;		// Pin configuration register for pin P3_6
  __IO	uint32_t SFSP3_7;		// Pin configuration register for pin P3_7
  __IO	uint32_t SFSP3_8;		// Pin configuration register for pin P3_8
	uint32_t RESERVED3[23];
  __IO	uint32_t SFSP4_0;		// Pin configuration register for pin P4_0
  __IO	uint32_t SFSP4_1;		// Pin configuration register for pin P4_1
  __IO	uint32_t SFSP4_2;		// Pin configuration register for pin P4_2
  __IO	uint32_t SFSP4_3;		// Pin configuration register for pin P4_3
  __IO	uint32_t SFSP4_4;		// Pin configuration register for pin P4_4
  __IO	uint32_t SFSP4_5;		// Pin configuration register for pin P4_5
  __IO	uint32_t SFSP4_6;		// Pin configuration register for pin P4_6
  __IO	uint32_t SFSP4_7;		// Pin configuration register for pin P4_7
  __IO	uint32_t SFSP4_8;		// Pin configuration register for pin P4_8
  __IO	uint32_t SFSP4_9;		// Pin configuration register for pin P4_9
  __IO	uint32_t SFSP4_10;		// Pin configuration register for pin P4_10
	uint32_t RESERVED4[21];
  __IO	uint32_t SFSP5_0;		// Pin configuration register for pin P5_0
  __IO	uint32_t SFSP5_1;		// Pin configuration register for pin P5_1
  __IO	uint32_t SFSP5_2;		// Pin configuration register for pin P5_2
  __IO	uint32_t SFSP5_3;		// Pin configuration register for pin P5_3
  __IO	uint32_t SFSP5_4;		// Pin configuration register for pin P5_4
  __IO	uint32_t SFSP5_5;		// Pin configuration register for pin P5_5
  __IO	uint32_t SFSP5_6;		// Pin configuration register for pin P5_6
  __IO	uint32_t SFSP5_7;		// Pin configuration register for pin P5_7
	uint32_t RESERVED5[24];
  __IO	uint32_t SFSP6_0;		// Pin configuration register for pin P6_0
  __IO	uint32_t SFSP6_1;		// Pin configuration register for pin P6_1
  __IO	uint32_t SFSP6_2;		// Pin configuration register for pin P6_2
  __IO	uint32_t SFSP6_3;		// Pin configuration register for pin P6_3
  __IO	uint32_t SFSP6_4;		// Pin configuration register for pin P6_4
  __IO	uint32_t SFSP6_5;		// Pin configuration register for pin P6_5
  __IO	uint32_t SFSP6_6;		// Pin configuration register for pin P6_6
  __IO	uint32_t SFSP6_7;		// Pin configuration register for pin P6_7
  __IO	uint32_t SFSP6_8;		// Pin configuration register for pin P6_8
  __IO	uint32_t SFSP6_9;		// Pin configuration register for pin P6_9
  __IO	uint32_t SFSP6_10;		// Pin configuration register for pin P6_10
  __IO	uint32_t SFSP6_11;		// Pin configuration register for pin P6_11
  __IO	uint32_t SFSP6_12;		// Pin configuration register for pin P6_12
	uint32_t RESERVED6[19];
  __IO	uint32_t SFSP7_0;		// Pin configuration register for pin P7_0
  __IO	uint32_t SFSP7_1;		// Pin configuration register for pin P7_1
  __IO	uint32_t SFSP7_2;		// Pin configuration register for pin P7_2
  __IO	uint32_t SFSP7_3;		// Pin configuration register for pin P7_3
  __IO	uint32_t SFSP7_4;		// Pin configuration register for pin P7_4
  __IO	uint32_t SFSP7_5;		// Pin configuration register for pin P7_5
  __IO	uint32_t SFSP7_6;		// Pin configuration register for pin P7_6
  __IO	uint32_t SFSP7_7;		// Pin configuration register for pin P7_7
	uint32_t RESERVED7[24];
  __IO	uint32_t SFSP8_0;		// Pin configuration register for pin P8_0
  __IO	uint32_t SFSP8_1;		// Pin configuration register for pin P8_1
  __IO	uint32_t SFSP8_2;		// Pin configuration register for pin P8_2
  __IO	uint32_t SFSP8_3;		// Pin configuration register for pin P8_3
  __IO	uint32_t SFSP8_4;		// Pin configuration register for pin P8_4
  __IO	uint32_t SFSP8_5;		// Pin configuration register for pin P8_5
  __IO	uint32_t SFSP8_6;		// Pin configuration register for pin P8_6
  __IO	uint32_t SFSP8_7;		// Pin configuration register for pin P8_7
  __IO	uint32_t SFSP8_8;		// Pin configuration register for pin P8_8
	uint32_t RESERVED8[23];
  __IO	uint32_t SFSP9_0;		// Pin configuration register for pin P9_0
  __IO	uint32_t SFSP9_1;		// Pin configuration register for pin P9_1
  __IO	uint32_t SFSP9_2;		// Pin configuration register for pin P9_2
  __IO	uint32_t SFSP9_3;		// Pin configuration register for pin P9_3
  __IO	uint32_t SFSP9_4;		// Pin configuration register for pin P9_4
  __IO	uint32_t SFSP9_5;		// Pin configuration register for pin P9_5
  __IO	uint32_t SFSP9_6;		// Pin configuration register for pin P9_6
	uint32_t RESERVED10[24];
  __IO	uint32_t SFSPA_0;		// Pin configuration register for pin PA_0
  __IO	uint32_t SFSPA_1;		// Pin configuration register for pin PA_1
  __IO	uint32_t SFSPA_2;		// Pin configuration register for pin PA_2
  __IO	uint32_t SFSPA_3;		// Pin configuration register for pin PA_3
  __IO	uint32_t SFSPA_4;		// Pin configuration register for pin PA_4
	uint32_t RESERVED11[27];
  __IO	uint32_t SFSPB_0;		// Pin configuration register for pin PB_0
  __IO	uint32_t SFSPB_1;		// Pin configuration register for pin PB_1
  __IO	uint32_t SFSPB_2;		// Pin configuration register for pin PB_2
  __IO	uint32_t SFSPB_3;		// Pin configuration register for pin PB_3
  __IO	uint32_t SFSPB_4;		// Pin configuration register for pin PB_4
  __IO	uint32_t SFSPB_5;		// Pin configuration register for pin PB_5
  __IO	uint32_t SFSPB_6;		// Pin configuration register for pin PB_6
	uint32_t RESERVED12[25];
  __IO	uint32_t SFSPC_0;		// Pin configuration register for pin PC_0
  __IO	uint32_t SFSPC_1;		// Pin configuration register for pin PC_1
  __IO	uint32_t SFSPC_2;		// Pin configuration register for pin PC_2
  __IO	uint32_t SFSPC_3;		// Pin configuration register for pin PC_3
  __IO	uint32_t SFSPC_4;		// Pin configuration register for pin PC_4
  __IO	uint32_t SFSPC_5;		// Pin configuration register for pin PC_5
  __IO	uint32_t SFSPC_6;		// Pin configuration register for pin PC_6
  __IO	uint32_t SFSPC_7;		// Pin configuration register for pin PC_7
  __IO	uint32_t SFSPC_8;		// Pin configuration register for pin PC_8
  __IO	uint32_t SFSPC_9;		// Pin configuration register for pin PC_9
  __IO	uint32_t SFSPC_10;		// Pin configuration register for pin PC_10 0x00
  __IO	uint32_t SFSPC_11;		// Pin configuration register for pin PC_11 0x00
  __IO	uint32_t SFSPC_12;		// Pin configuration register for pin PC_12 0x00
  __IO	uint32_t SFSPC_13;		// Pin configuration register for pin PC_13 0x00
  __IO	uint32_t SFSPC_14;		// Pin configuration register for pin PC_14 0x00
	uint32_t RESERVED13[17];
  __IO	uint32_t SFSPD_0;		// Pin configuration register for pin PD_0
  __IO	uint32_t SFSPD_1;		// Pin configuration register for pin PD_1
  __IO	uint32_t SFSPD_2;		// Pin configuration register for pin PD_2
  __IO	uint32_t SFSPD_3;		// Pin configuration register for pin PD_3
  __IO	uint32_t SFSPD_4;		// Pin configuration register for pin PD_4
  __IO	uint32_t SFSPD_5;		// Pin configuration register for pin PD_5
  __IO	uint32_t SFSPD_6;		// Pin configuration register for pin PD_6
  __IO	uint32_t SFSPD_7;		// Pin configuration register for pin PD_7
  __IO	uint32_t SFSPD_8;		// Pin configuration register for pin PD_8
  __IO	uint32_t SFSPD_9;		// Pin configuration register for pin PD_9
  __IO	uint32_t SFSPD_10;		// Pin configuration register for pin PD_10 0x00
  __IO	uint32_t SFSPD_11;		// Pin configuration register for pin PD_11 0x00
  __IO	uint32_t SFSPD_12;		// Pin configuration register for pin PD_12 0x00
  __IO	uint32_t SFSPD_13;		// Pin configuration register for pin PD_13 0x00
  __IO	uint32_t SFSPD_14;		// Pin configuration register for pin PD_14 0x00
  __IO	uint32_t SFSPD_15;		// Pin configuration register for pin PD_15 0x00
  __IO	uint32_t SFSPD_16;		// Pin configuration register for pin PD_16 0x00
	uint32_t RESERVED14[15];
  __IO	uint32_t SFSPE_0;		// Pin configuration register for pin PE_0
  __IO	uint32_t SFSPE_1;		// Pin configuration register for pin PE_1
  __IO	uint32_t SFSPE_2;		// Pin configuration register for pin PE_2
  __IO	uint32_t SFSPE_3;		// Pin configuration register for pin PE_3
  __IO	uint32_t SFSPE_4;		// Pin configuration register for pin PE_4
  __IO	uint32_t SFSPE_5;		// Pin configuration register for pin PE_5
  __IO	uint32_t SFSPE_6;		// Pin configuration register for pin PE_6
  __IO	uint32_t SFSPE_7;		// Pin configuration register for pin PE_7
  __IO	uint32_t SFSPE_8;		// Pin configuration register for pin PE_8
  __IO	uint32_t SFSPE_9;		// Pin configuration register for pin PE_9
  __IO	uint32_t SFSPE_10;		// Pin configuration register for pin PE_10 0x00
  __IO	uint32_t SFSPE_11;		// Pin configuration register for pin PE_11
  __IO	uint32_t SFSPE_12;		// Pin configuration register for pin PE_12 0x00
  __IO	uint32_t SFSPE_13;		// Pin configuration register for pin PE_13 0x00
  __IO	uint32_t SFSPE_14;		// Pin configuration register for pin PE_14 0x00
  __IO	uint32_t SFSPE_15;		// Pin configuration register for pin PE_15 0x00
	uint32_t RESERVED15[16];
  __IO	uint32_t SFSPF_0;		// Pin configuration register for pin PF_0
  __IO	uint32_t SFSPF_1;		// Pin configuration register for pin PF_1
  __IO	uint32_t SFSPF_2;		// Pin configuration register for pin PF_2
  __IO	uint32_t SFSPF_3;		// Pin configuration register for pin PF_3
  __IO	uint32_t SFSPF_4;		// Pin configuration register for pin PF_4
  __IO	uint32_t SFSPF_5;		// Pin configuration register for pin PF_5
  __IO	uint32_t SFSPF_6;		// Pin configuration register for pin PF_6
  __IO	uint32_t SFSPF_7;		// Pin configuration register for pin PF_7
  __IO	uint32_t SFSPF_8;		// Pin configuration register for pin PF_8
  __IO	uint32_t SFSPF_9;		// Pin configuration register for pin PF_9
  __IO	uint32_t SFSPF_10;		// Pin configuration register for pin PF_10
  __IO	uint32_t SFSPF_11;		// Pin configuration register for pin PF_11
	uint32_t RESERVED16[276];
  __IO	uint32_t SFSCLK0;		// Pin configuration register for pin CLK0
  __IO	uint32_t SFSCLK1;		// Pin configuration register for pin CLK1
  __IO	uint32_t SFSCLK2;		// Pin configuration register for pin CLK2
  __IO	uint32_t SFSCLK3;		// Pin configuration register for pin CLK3
	uint32_t RESERVED17[28];
  __IO	uint32_t SFSUSB;		// Pin configuration register for pins
  __IO	uint32_t SFSI2C0;		// Pin configuration register for I2C0-bus
  __IO	uint32_t ENAIO0;		// ADC0 function select register
  __IO	uint32_t ENAIO1;		// ADC1 function select register
  __IO	uint32_t ENAIO2;		// Analog function select register
	uint32_t RESERVED18[27];
  __IO	uint32_t EMCDELAYCLK;		// EMC clock delay register
	uint32_t RESERVED19[63];
  __IO	uint32_t PINTSEL0;		// Pin interrupt select register for pin
  __IO	uint32_t PINTSEL1;		// Pin interrupt select register for pin
} LPC_SCU_TypeDef;
/*@}*/ /* end of group LPC18xx_SCU */

/*------------- GIMA (GIMA) ----------------------------*/
/** @addtogroup LPC18xx_GIMA LPC18xx GIMA (GIMA) 
  @{
*/
typedef struct
{
  __IO	uint32_t CAP0_0_IN;		// Timer 0 CAP0_0 capture input multiplexer
  __IO	uint32_t CAP0_1_IN;		// Timer 0 CAP0_1 capture input multiplexer
  __IO	uint32_t CAP0_2_IN;		// Timer 0 CAP0_2 capture input multiplexer
  __IO	uint32_t CAP0_3_IN;		// Timer 0 CAP0_3 capture input multiplexer
  __IO	uint32_t CAP1_0_IN;		// Timer 1 CAP1_0 capture input multiplexer
  __IO	uint32_t CAP1_1_IN;		// Timer 1 CAP1_1 capture input multiplexer
  __IO	uint32_t CAP1_2_IN;		// Timer 1 CAP1_2 capture input multiplexer
  __IO	uint32_t CAP1_3_IN;		// Timer 1 CAP1_3 capture input multiplexer
  __IO	uint32_t CAP2_0_IN;		// Timer 2 CAP2_0 capture input multiplexer
  __IO	uint32_t CAP2_1_IN;		// Timer 2 CAP2_1 capture input multiplexer
  __IO	uint32_t CAP2_2_IN;		// Timer 2 CAP2_2 capture input multiplexer
  __IO	uint32_t CAP2_3_IN;		// Timer 2 CAP2_3 capture input multiplexer
  __IO	uint32_t CAP3_0_IN;		// Timer 3 CAP3_0 capture input multiplexer
  __IO	uint32_t CAP3_1_IN;		// Timer 3 CAP3_1 capture input multiplexer
  __IO	uint32_t CAP3_2_IN;		// Timer 3 CAP3_2 capture input multiplexer
  __IO	uint32_t CAP3_3_IN;		// Timer 3 CAP3_3 capture input multiplexer
  __IO	uint32_t CTIN_0_IN;		// SCT CTIN_0 capture input multiplexer
  __IO	uint32_t CTIN_1_IN;		// SCT CTIN_1 capture input multiplexer
  __IO	uint32_t CTIN_2_IN;		// SCT CTIN_2 capture input multiplexer
  __IO	uint32_t CTIN_3_IN;		// SCT CTIN_3 capture input multiplexer
  __IO	uint32_t CTIN_4_IN;		// SCT CTIN_4 capture input multiplexer
  __IO	uint32_t CTIN_5_IN;		// SCT CTIN_5 capture input multiplexer
  __IO	uint32_t CTIN_6_IN;		// SCT CTIN_6 capture input multiplexer
  __IO	uint32_t CTIN_7_IN;		// SCT CTIN_7 capture input multiplexer
  __IO	uint32_t VADC_TRIGGER_IN;	// VADC trigger input multiplexer
  __IO	uint32_t EVENTROUTER_13_IN;	// Event router input 13 multiplexer
  __IO	uint32_t EVENTROUTER_14_IN;	// Event router input 14 multiplexer
  __IO	uint32_t EVENTROUTER_16_IN;	// Event router input 16 multiplexer
  __IO	uint32_t ADCSTART0_IN;		// ADC0 and ADC1 start0 input multiplexer
  __IO	uint32_t ADCSTART1_IN;		// ADC0 and ADC1 start1 input multiplexer
} LPC_GIMA_TypeDef;
/*@}*/ /* end of group LPC18xx_GIMA */

/*------------- GPIO pin interrupts (GPIO_PIN_INT) ----------------------------*/
/** @addtogroup LPC18xx_GPIO_PIN_INT LPC18xx GPIO pin interrupts (GPIO_PIN_INT) 
  @{
*/
typedef struct
{
  __IO	uint32_t ISEL;		// Pin Interrupt Mode register
  __IO	uint32_t IENR;		// Pin interrupt level
  __O	uint32_t SIENR;		// Pin interrupt level
  __O	uint32_t CIENR;		// Pin interrupt level
  __IO	uint32_t IENF;		// Pin interrupt active level
  __O	uint32_t SIENF;		// Pin interrupt active level
  __O	uint32_t CIENF;		// Pin interrupt active level
  __IO	uint32_t RISE;		// Pin interrupt rising edge register
  __IO	uint32_t FALL;		// Pin interrupt falling edge register
  __IO	uint32_t IST;		// Pin interrupt status register
} LPC_GPIO_PIN_INT_TypeDef;
/*@}*/ /* end of group LPC18xx_GPIO_PIN_INT */

/*------------- GPIO GROUP interrupt (GPIO_GROUP_INT) ----------------------------*/
/** @addtogroup LPC18xx_GPIO_GROUP_INT LPC18xx GPIO GROUP interrupt (GPIO_GROUP_INT) 
  @{
*/
typedef struct
{
  __IO	uint32_t CTRL;			// GPIO grouped interrupt control register
	uint32_t RESERVED0[7];
  __IO	uint32_t PORT_POL[8];		// GPIO grouped interrupt polarity register
  __IO	uint32_t PORT_ENA[8];		// GPIO grouped interrupt enable register
} LPC_GPIO_GROUP_INT_TypeDef;
/*@}*/ /* end of group LPC18xx_GPIO_GROUP_INT */

/*------------- GPIO port (GPIO) ----------------------------*/
/** @addtogroup LPC18xx_GPIO LPC18xx GPIO port (GPIO) 
  @{
*/
typedef struct
{
  __IO	uint8_t B[8][32];	// Byte pin registers
	uint32_t RESERVED0[960];
  __IO	uint32_t W[8][32];	// Word pin registers
	uint32_t RESERVED15[768];
  __IO	uint32_t DIR[8];	// Direction registers port
	uint32_t RESERVED16[24];
  __IO	uint32_t MASK[8];	// Mask register port
	uint32_t RESERVED17[24];
  __IO	uint32_t PIN[8];	// Port pin register port
	uint32_t RESERVED18[24];
  __IO	uint32_t MPIN[8];	// Masked port register port
	uint32_t RESERVED19[24];
  __IO	uint32_t SET[8];	// Write
	uint32_t RESERVED20[24];
  __O	uint32_t CLR[8];	// Clear port
	uint32_t RESERVED21[24];
  __O	uint32_t NOT[8];	// Toggle port
} LPC_GPIO_TypeDef;
/*@}*/ /* end of group LPC18xx_GPIO */

/*------------- GPDMA (GPDMA) ----------------------------*/
/** @addtogroup LPC18xx_GPDMA LPC18xx GPDMA (GPDMA) 
  @{
*/
typedef struct
{
  __I	uint32_t INTSTAT;		// DMA Interrupt Status Register
  __I	uint32_t INTTCSTAT;		// DMA Interrupt Terminal Count Request Status
  __O	uint32_t INTTCCLEAR;		// DMA Interrupt Terminal Count Request Clear
  __I	uint32_t INTERRSTAT;		// DMA Interrupt Error Status Register
  __O	uint32_t INTERRCLR;		// DMA Interrupt Error Clear Register
  __I	uint32_t RAWINTTCSTAT;		// DMA Raw Interrupt Terminal Count Status
  __I	uint32_t RAWINTERR;		// DMA Raw Error Interrupt Status Register
  __I	uint32_t ENBLDCHNS;		// DMA Enabled Channel Register
  __IO	uint32_t SOFTBREQ;		// DMA Software Burst Request Register
  __IO	uint32_t SOFTSREQ;		// DMA Software Single Request Register
  __IO	uint32_t SOFTLBREQ;		// DMA Software Last Burst Request Register
  __IO	uint32_t SOFTLSREQ;		// DMA Software Last Single Request Register
  __IO	uint32_t CONFIG;		// DMA Configuration Register
  __IO	uint32_t SYNC;			// DMA Synchronization Register
	uint32_t RESERVED0[50];

  struct {
    __IO uint32_t SRCADDR;		// DMA Channel Source Address Register
    __IO uint32_t DESTADDR;		// DMA Channel Destination Address Register
    __IO uint32_t LLI;			// DMA Channel Linked List Item Register
    __IO uint32_t CONTROL;		// DMA Channel Control Register
    __IO uint32_t CONFIG;		// DMA Channel Configuration
	 uint32_t RESERVED[3];
  } CHANNEL[8];

} LPC_GPDMA_TypeDef;
/*@}*/ /* end of group LPC18xx_GPDMA */

/*------------- SDMMC (SDMMC) ----------------------------*/
/** @addtogroup LPC18xx_SDMMC LPC18xx SDMMC (SDMMC) 
  @{
*/
typedef struct
{
  __IO	uint32_t CTRL;		// Control Register
  __IO	uint32_t PWREN;		// Power Enable Register
  __IO	uint32_t CLKDIV;	// Clock Divider Register
  __IO	uint32_t CLKSRC;	// SD Clock Source Register
  __IO	uint32_t CLKENA;	// Clock Enable Register
  __IO	uint32_t TMOUT;		// Time-out Register
  __IO	uint32_t CTYPE;		// Card Type Register
  __IO	uint32_t BLKSIZ;	// Block Size Register
  __IO	uint32_t BYTCNT;	// Byte Count Register
  __IO	uint32_t INTMASK;	// Interrupt Mask Register
  __IO	uint32_t CMDARG;	// Command Argument Register
  __IO	uint32_t CMD;		// Command Register
  __I	uint32_t RESP0;		// Response Register 0
  __I	uint32_t RESP1;		// Response Register 1
  __I	uint32_t RESP2;		// Response Register 2
  __I	uint32_t RESP3;		// Response Register 3
  __I	uint32_t MINTSTS;	// Masked Interrupt Status Register
  __IO	uint32_t RINTSTS;	// Raw Interrupt Status Register
  __I	uint32_t STATUS;	// Status Register
  __IO	uint32_t FIFOTH;	// FIFO Threshold Watermark Register
  __I	uint32_t CDETECT;	// Card Detect Register
  __I	uint32_t WRTPRT;	// Write Protect Register
	uint32_t RESERVED0[1];
  __I	uint32_t TCBCNT;	// Transferred CIU Card Byte Count Register
  __I	uint32_t TBBCNT;	// Transferred Host to BIU-FIFO Byte Count
  __IO	uint32_t DEBNCE;	// Debounce Count Register
	uint32_t RESERVED1[4];
  __IO	uint32_t RST_N;		// Hardware Reset
	uint32_t RESERVED2[1];
  __IO	uint32_t BMOD;		// Bus Mode Register
  __O	uint32_t PLDMND;	// Poll Demand Register
  __IO	uint32_t DBADDR;	// Descriptor List Base Address Register
  __IO	uint32_t IDSTS;		// Internal DMAC Status Register
  __IO	uint32_t IDINTEN;	// Internal DMAC Interrupt Enable Register
  __I	uint32_t DSCADDR;	// Current Host Descriptor Address Register
  __I	uint32_t BUFADDR;	// Current Buffer Descriptor Address Register
} LPC_SDMMC_TypeDef;
/*@}*/ /* end of group LPC18xx_SDMMC */

/*------------- SPIFI (SPIFI) ----------------------------*/
/** @addtogroup LPC18xx_SPIFI LPC18xx SPIFI (SPIFI) 
  @{
*/
typedef struct
{
  __IO	uint32_t CTRL;		// SPIFI control register
  __IO	uint32_t CMD;		// SPIFI command register
  __IO	uint32_t ADDR;		// SPIFI address register
  __IO	uint32_t IDATA;		// SPIFI intermediate data register
  __IO	uint32_t CLIMIT;	// SPIFI cache limit register
  __IO	uint32_t DATA;		// SPIFI data register
  __IO	uint32_t MCMD;		// SPIFI memory command register
  __IO	uint32_t STAT;		// SPIFI status register
} LPC_SPIFI_TypeDef;
/*@}*/ /* end of group LPC18xx_SPIFI */

/*------------- External memory controller (EMC) ----------------------------*/
/** @addtogroup LPC18xx_EMC LPC18xx External memory controller (EMC) 
  @{
*/
typedef struct
{
  __IO	uint32_t CONTROL;		// Controls operation of the memory
  __I	uint32_t STATUS;		// Provides EMC status information
  __IO	uint32_t CONFIG;		// Configures operation of the memory
	uint32_t RESERVED0[5];
  __IO	uint32_t DYNAMICCONTROL;	// Controls dynamic memory operation
  __IO	uint32_t DYNAMICREFRESH;	// Configures dynamic memory refresh
  __IO	uint32_t DYNAMICREADCONFIG;	// Configures the dynamic memory read
	uint32_t RESERVED1[1];
  __IO	uint32_t DYNAMICRP;		// Selects the precharge command period
  __IO	uint32_t DYNAMICRAS;		// Selects the active to precharge command
  __IO	uint32_t DYNAMICSREX;		// Selects the self-refresh exit time
  __IO	uint32_t DYNAMICAPR;		// Selects the last-data-out to active
  __IO	uint32_t DYNAMICDAL;		// Selects the data-in to active command
  __IO	uint32_t DYNAMICWR;		// Selects the write recovery time
  __IO	uint32_t DYNAMICRC;		// Selects the active to active command
  __IO	uint32_t DYNAMICRFC;		// Selects the auto-refresh period
  __IO	uint32_t DYNAMICXSR;		// Selects the exit self-refresh to active
  __IO	uint32_t DYNAMICRRD;		// Selects the active bank A to active bank B 0xF
  __IO	uint32_t DYNAMICMRD;		// Selects the load mode register to active
	uint32_t RESERVED2[9];
  __IO	uint32_t STATICEXTENDEDWAIT;	// Selects time for long static memory read
	uint32_t RESERVED3[31];
  __IO	uint32_t DYNAMICCONFIG0;	// Selects the configuration information for
  __IO	uint32_t DYNAMICRASCAS0;	// Selects the RAS and CAS latencies for
	uint32_t RESERVED4[6];
  __IO	uint32_t DYNAMICCONFIG1;	// Selects the configuration information for
  __IO	uint32_t DYNAMICRASCAS1;	// Selects the RAS and CAS latencies for
	uint32_t RESERVED5[6];
  __IO	uint32_t DYNAMICCONFIG2;	// Selects the configuration information for
  __IO	uint32_t DYNAMICRASCAS2;	// Selects the RAS and CAS latencies for
	uint32_t RESERVED6[6];
  __IO	uint32_t DYNAMICCONFIG3;	// Selects the configuration information for
  __IO	uint32_t DYNAMICRASCAS3;	// Selects the RAS and CAS latencies for
	uint32_t RESERVED7[38];
  __IO	uint32_t STATICCONFIG0;		// Selects the memory configuration for static 0
  __IO	uint32_t STATICWAITWEN0;	// Selects the delay from chip select 0 to
  __IO	uint32_t STATICWAITOEN0;	// Selects the delay from chip select 0 or
  __IO	uint32_t STATICWAITRD0;		// Selects the delay from chip select 0 to a
  __IO	uint32_t STATICWAITPAGE0;	// Selects the delay for asynchronous page
  __IO	uint32_t STATICWAITWR0;		// Selects the delay from chip select 0 to a
  __IO	uint32_t STATICWAITTURN0;	// Selects the number of bus turnaround
	uint32_t RESERVED8[1];
  __IO	uint32_t STATICCONFIG1;		// Selects the memory configuration for static 0
  __IO	uint32_t STATICWAITWEN1;	// Selects the delay from chip select 1 to
  __IO	uint32_t STATICWAITOEN1;	// Selects the delay from chip select 1 or
  __IO	uint32_t STATICWAITRD1;		// Selects the delay from chip select 1 to a
  __IO	uint32_t STATICWAITPAGE1;	// Selects the delay for asynchronous page
  __IO	uint32_t STATICWAITWR1;		// Selects the delay from chip select 1 to a
  __IO	uint32_t STATICWAITTURN1;	// Selects the number of bus turnaround
	uint32_t RESERVED9[1];
  __IO	uint32_t STATICCONFIG2;		// Selects the memory configuration for static 0
  __IO	uint32_t STATICWAITWEN2;	// Selects the delay from chip select 2 to
  __IO	uint32_t STATICWAITOEN2;	// Selects the delay from chip select 2 or
  __IO	uint32_t STATICWAITRD2;		// Selects the delay from chip select 2 to a
  __IO	uint32_t STATICWAITPAGE2;	// Selects the delay for asynchronous page
  __IO	uint32_t STATICWAITWR2;		// Selects the delay from chip select 2 to a
  __IO	uint32_t STATICWAITTURN2;	// Selects the number of bus turnaround
	uint32_t RESERVED10[1];
  __IO	uint32_t STATICCONFIG3;		// Selects the memory configuration for static 0
  __IO	uint32_t STATICWAITWEN3;	// Selects the delay from chip select 3 to
  __IO	uint32_t STATICWAITOEN3;	// Selects the delay from chip select 3 or
  __IO	uint32_t STATICWAITRD3;		// Selects the delay from chip select 3 to a
  __IO	uint32_t STATICWAITPAGE3;	// Selects the delay for asynchronous page
  __IO	uint32_t STATICWAITWR3;		// Selects the delay from chip select 3 to a
  __IO	uint32_t STATICWAITTURN3;	// Selects the number of bus turnaround
} LPC_EMC_TypeDef;
/*@}*/ /* end of group LPC18xx_EMC */

/*------------- USB0 OTG controller (USB0) ----------------------------*/
/** @addtogroup LPC18xx_USB0 LPC18xx USB0 OTG controller (USB0) 
  @{
*/
typedef struct
{
	uint32_t RESERVED0[64];
  __I	uint32_t CAPLENGTH;		// Capability register length
  __I	uint32_t HCSPARAMS;		// Host controller structural parameters
  __I	uint32_t HCCPARAMS;		// Host controller capability parameters
	uint32_t RESERVED1[5];
  __I	uint32_t DCIVERSION;		// Device interface version number
  __I	uint32_t DCCPARAMS;		// Device controller capability
	uint32_t RESERVED2[6];
  __IO	uint32_t USBCMD;		// USB command
  __IO	uint32_t USBSTS;		// USB status
  __IO	uint32_t USBINTR;		// USB interrupt enable
  __IO	uint32_t FRINDEX;		// USB frame index
	uint32_t RESERVED3[1];

  union {
    __IO uint32_t DEVICEADDR;		// USB Device Address (device mode)
    __IO uint32_t PERIODICLISTBASE;	// Frame list base address (host mode)
  };

  __IO	uint32_t ASYNCLISTADDR;		// Asynchronous list address
  __IO	uint32_t TTCTRL;		// Asynchronous buffer status for
  __IO	uint32_t BURSTSIZE;		// Programmable burst size
  __IO	uint32_t TXFILLTUNING;		// Host transmit pre-buffer packet
	uint32_t RESERVED4[3];
  __IO	uint32_t BINTERVAL;		// Length of virtual frame
  __IO	uint32_t ENDPTNAK;		// Endpoint NAK
  __IO	uint32_t ENDPTNAKEN;		// Endpoint NAK Enable
	uint32_t RESERVED5[1];
  __IO	uint32_t PORTSC1;		// Port 1 status/control
	uint32_t RESERVED6[7];
  __IO	uint32_t OTGSC;			// OTG status and control
  __IO	uint32_t USBMODE;		// USB device mode
  __IO	uint32_t ENDPTSETUPSTAT;	// Endpoint setup status
  __IO	uint32_t ENDPTPRIME;		// Endpoint initialization
  __IO	uint32_t ENDPTFLUSH;		// Endpoint de-initialization
  __I	uint32_t ENDPTSTAT;		// Endpoint status
  __IO	uint32_t ENDPTCOMPLETE;		// Endpoint complete

  union {
    struct {
      __IO uint32_t ENDPTCTRL0;		// Endpoint control 0
      __IO uint32_t ENDPTCTRL1;		// Endpoint control 1
      __IO uint32_t ENDPTCTRL2;		// Endpoint control 2
      __IO uint32_t ENDPTCTRL3;		// Endpoint control 3
      __IO uint32_t ENDPTCTRL4;		// Endpoint control 4
      __IO uint32_t ENDPTCTRL5;		// Endpoint control 5
    };
    __IO uint32_t ENDPTCTRL[6];		// Endpoint control
  };
} LPC_USB0_TypeDef;
/*@}*/ /* end of group LPC18xx_USB0 */

/*------------- USB1 host/device controller (USB1) ----------------------------*/
/** @addtogroup LPC18xx_USB1 LPC18xx USB1 host/device controller (USB1) 
  @{
*/
typedef struct
{
	uint32_t RESERVED0[64];
  __I	uint32_t CAPLENGTH;		// Capability register length
  __I	uint32_t HCSPARAMS;		// Host controller structural parameters
  __I	uint32_t HCCPARAMS;		// Host controller capability parameters
	uint32_t RESERVED1[5];
  __I	uint32_t DCIVERSION;		// Device interface version number
  __I	uint32_t DCCPARAMS;		// Device controller capability parameters 0x0000 0184
	uint32_t RESERVED2[6];
  __IO	uint32_t USBCMD;		// USB command
  __IO	uint32_t USBSTS;		// USB status
  __IO	uint32_t USBINTR;		// USB interrupt enable
  __IO	uint32_t FRINDEX;		// USB frame index
	uint32_t RESERVED3[1];

  union {
    __IO uint32_t DEVICEADDR;		// USB Device Address (device mode)
    __IO uint32_t PERIODICLISTBASE;	// Frame list base address (host mode)
  };

  __IO	uint32_t ASYNCLISTADDR;		// Asynchronous list address
  __IO	uint32_t TTCTRL;		// Asynchronous buffer status for
  __IO	uint32_t BURSTSIZE;		// Programmable burst size
  __IO	uint32_t TXFILLTUNING;		// Host transmit pre-buffer packet tuning
	uint32_t RESERVED4[2];
  __IO	uint32_t ULPIVIEWPORT;		// ULPI viewport
  __IO	uint32_t BINTERVAL;		// Length of virtual frame
  __IO	uint32_t ENDPTNAK;		// Endpoint NAK
  __IO	uint32_t ENDPTNAKEN;		// Endpoint NAK Enable
	uint32_t RESERVED5[1];
  __IO	uint32_t PORTSC1;		// Port 1 status/control
	uint32_t RESERVED6[8];
  __IO	uint32_t USBMODE;		// USB mode
  __IO	uint32_t ENDPTSETUPSTAT;	// Endpoint setup status
  __IO	uint32_t ENDPTPRIME;		// Endpoint initialization
  __IO	uint32_t ENDPTFLUSH;		// Endpoint de-initialization
  __I	uint32_t ENDPTSTAT;		// Endpoint status
  __IO	uint32_t ENDPTCOMPLETE;		// Endpoint complete

  union {
    struct {
      __IO uint32_t ENDPTCTRL0;		// Endpoint control 0
      __IO uint32_t ENDPTCTRL1;		// Endpoint control 1
      __IO uint32_t ENDPTCTRL2;		// Endpoint control 2
      __IO uint32_t ENDPTCTRL3;		// Endpoint control 3
    };
    __IO uint32_t ENDPTCTRL[4];		// Endpoint control
  };
} LPC_USB1_TypeDef;
/*@}*/ /* end of group LPC18xx_USB1 */

/*------------- Ethernet MAC and DMA (EMAC) ----------------------------*/
/** @addtogroup LPC18xx_EMAC LPC18xx Ethernet MAC and DMA (EMAC) 
  @{
*/
typedef struct
{
  __IO	uint32_t MAC_CONFIG;		// MAC configuration register
  __IO	uint32_t MAC_FRAME_FILTER;	// MAC frame filter
  __IO	uint32_t MAC_HASHTABLE_HIGH;	// Hash table high register
  __IO	uint32_t MAC_HASHTABLE_LOW;	// Hash table low register
  __IO	uint32_t MAC_MII_ADDR;		// MII address register
  __IO	uint32_t MAC_MII_DATA;		// MII data register
  __IO	uint32_t MAC_FLOW_CTRL;		// Flow control register
  __IO	uint32_t MAC_VLAN_TAG;		// VLAN tag register
	uint32_t RESERVED0[1];
  __I	uint32_t MAC_DEBUG;		// Debug register
  __IO	uint32_t MAC_RWAKE_FRFLT;	// Remote wake-up frame filter
  __IO	uint32_t MAC_PMT_CTRL_STAT;	// PMT control and status
	uint32_t RESERVED1[2];
  __I	uint32_t MAC_INTR;		// Interrupt status register
  __IO	uint32_t MAC_INTR_MASK;		// Interrupt mask register
  __IO	uint32_t MAC_ADDR0_HIGH;	// MAC address 0 high register
  __IO	uint32_t MAC_ADDR0_LOW;		// MAC address 0 low register
	uint32_t RESERVED2[430];
  __IO	uint32_t MAC_TIMESTP_CTRL;	// Time stamp control register
  __IO	uint32_t SUBSECOND_INCR;	// Sub-second increment
  __I	uint32_t SECONDS;		// System time seconds
  __I	uint32_t NANOSECONDS;		// System time nanoseconds
  __IO	uint32_t SECONDSUPDATE;		// System time seconds
  __IO	uint32_t NANOSECONDSUPDATE;	// System time nanoseconds
  __IO	uint32_t ADDEND;		// Time stamp addend register
  __IO	uint32_t TARGETSECONDS;		// Target time seconds register
  __IO	uint32_t TARGETNANOSECONDS;	// Target time nanoseconds
  __IO	uint32_t HIGHWORD;		// System time higher word
  __I	uint32_t TIMESTAMPSTAT;		// Time stamp status register
	uint32_t RESERVED3[565];
  __IO	uint32_t DMA_BUS_MODE;		// Bus Mode Register
  __IO	uint32_t DMA_TRANS_POLL_DEMAND;	// Transmit poll demand
  __IO	uint32_t DMA_REC_POLL_DEMAND;	// Receive poll demand
  __IO	uint32_t DMA_REC_DES_ADDR;	// Receive descriptor list
  __IO	uint32_t DMA_TRANS_DES_ADDR;	// Transmit descriptor list
  __IO	uint32_t DMA_STAT;		// Status register
  __IO	uint32_t DMA_OP_MODE;		// Operation mode register
  __IO	uint32_t DMA_INT_EN;		// Interrupt enable register
  __I	uint32_t DMA_MFRM_BUFOF;	// Missed frame and buffer
  __IO	uint32_t DMA_REC_INT_WDT;	// Receive interrupt watchdog
	uint32_t RESERVED4[8];
  __I	uint32_t DMA_CURHOST_TRANS_DES;	// Current host transmit
  __I	uint32_t DMA_CURHOST_REC_DES;	// Current host receive
  __I	uint32_t DMA_CURHOST_TRANS_BUF;	// Current host transmit buffer
  __I	uint32_t DMA_CURHOST_REC_BUF;	// Current host receive buffer
} LPC_EMAC_TypeDef;
/*@}*/ /* end of group LPC18xx_EMAC */

/*------------- LCD controller (LCD) ----------------------------*/
/** @addtogroup LPC18xx_LCD LPC18xx LCD controller (LCD) 
  @{
*/
typedef struct
{
  __IO	uint32_t TIMH;			// Horizontal Timing Control register
  __IO	uint32_t TIMV;			// Vertical Timing Control register
  __IO	uint32_t POL;			// Clock and Signal Polarity Control register
  __IO	uint32_t LE;			// Line End Control register
  __IO	uint32_t UPBASE;		// Upper Panel Frame Base Address register
  __IO	uint32_t LPBASE;		// Lower Panel Frame Base Address register
  __IO	uint32_t CTRL;			// LCD Control register
  __IO	uint32_t INTMSK;		// Interrupt Mask register
  __I	uint32_t INTRAW;		// Raw Interrupt Status register
  __I	uint32_t INTSTAT;		// Masked Interrupt Status register
  __O	uint32_t INTCLR;		// Interrupt Clear register
  __I	uint32_t UPCURR;		// Upper Panel Current Address Value register
  __I	uint32_t LPCURR;		// Lower Panel Current Address Value register
	uint32_t RESERVED0[115];
  __IO	uint32_t PAL[256];		// 256x16-bit Color Palette registers
	uint32_t RESERVED1[128];
  __IO	uint32_t CRSR_IMG[256];		// Cursor Image registers
  __IO	uint32_t CRSR_CTRL;		// Cursor Control register
  __IO	uint32_t CRSR_CFG;		// Cursor Configuration register
  __IO	uint32_t CRSR_PAL0;		// Cursor Palette register 0
  __IO	uint32_t CRSR_PAL1;		// Cursor Palette register 1
  __IO	uint32_t CRSR_XY;		// Cursor XY Position register
  __IO	uint32_t CRSR_CLIP;		// Cursor Clip Position register
	uint32_t RESERVED2[2];
  __IO	uint32_t CRSR_INTMSK;		// Cursor Interrupt Mask register
  __O	uint32_t CRSR_INTCLR;		// Cursor Interrupt Clear register
  __I	uint32_t CRSR_INTRAW;		// Cursor Raw Interrupt Status register
  __I	uint32_t CRSR_INTSTAT;		// Cursor Masked Interrupt Status register
} LPC_LCD_TypeDef;
/*@}*/ /* end of group LPC18xx_LCD */

/*------------- State Configurable Timer (SCT) ----------------------------*/
/** @addtogroup LPC18xx_SCT LPC18xx State Configurable Timer (SCT) 
  @{
*/
typedef struct
{
  __IO	uint32_t CONFIG;		// SCT configuration register
  __IO	uint32_t CTRL;			// SCT control register high counter 16-bit
  __IO	uint32_t LIMIT;			// SCT limit register high counter 16-bit
  __IO	uint32_t HALT;			// SCT halt condition register high counter 16-bit
  __IO	uint32_t STOP;			// SCT stop condition register high counter 16-bit
  __IO	uint32_t START;			// SCT start condition register high counter 16-bit
  __IO	uint32_t DITHER;		// SCT dither condition register high counter 16-bit
	uint32_t RESERVED0[9];
  __IO	uint32_t COUNT;			// SCT counter register high counter 16-bit
  __IO	uint32_t STATE;			// SCT state register high counter 16-bit
  __I	uint32_t INPUT;			// SCT input register
  __IO	uint32_t REGMODE;		// SCT match/capture registers mode register high
  __IO	uint32_t OUTPUT;		// SCT output register
  __IO	uint32_t OUTPUTDIRCTRL;		// SCT output counter direction control register
  __IO	uint32_t RES;			// SCT conflict resolution register
  __IO	uint32_t DMAREQ0;		// SCT DMA request 0 register
  __IO	uint32_t DMAREQ1;		// SCT DMA request 1 register
	uint32_t RESERVED1[35];
  __IO	uint32_t EVEN;			// SCT event enable register
  __IO	uint32_t EVFLAG;		// SCT event flag register
  __IO	uint32_t CONEN;			// SCT conflict enable register
  __IO	uint32_t CONFLAG;		// SCT conflict flag register

/* Match - Capture */

  union {
    __IO uint32_t MATCH[16];		// SCT match value registers
    __IO uint32_t CAPTURE[16];		// SCT capture value registers
  };
  __IO	uint32_t FRACMATCH[6];		// SCT fractional match registers
  	uint32_t RESERVED2[10];

  union {
    __IO uint16_t MATCH_L[16];		// SCT match value registers low
    __IO uint16_t CAPTURE_L[16];	// SCT capture value registers low
  };
  __IO	uint16_t FRACMATCH_L[6];	// SCT fractional match registers low
    	uint16_t RESERVED3[10];

  union {
    __IO uint16_t MATCH_H[16];		// SCT match value registers high
    __IO uint16_t CAPTURE_H[16];	// SCT capture value registers high
  };
  __IO	uint16_t FRACMATCH_H[6];	// SCT fractional match registers high
    	uint16_t RESERVED4[10];

/* Match Reload - Capture Control */

  union {
    __IO uint32_t MATCHREL[16];		// SCT match reload registers
    __IO uint32_t CAPCTRL[16];		// SCT capture control registers
  };
  __IO	uint32_t FRACMATCHREL[6];	// SCT fractional match reload registers
  	uint32_t RESERVED5[10];

  union {
    __IO uint16_t MATCHREL_L[16];	// SCT match reload registers low
    __IO uint16_t CAPCTRL_L[16];	// SCT capture control registers low
  };
  __IO	uint16_t FRACMATCHREL_L[6];	// SCT fractional match reload registers low
    	uint16_t RESERVED6[10];

  union {
    __IO uint16_t MATCHREL_H[16];	// SCT match reload registers high
    __IO uint16_t CAPCTRL_H[16];	// SCT capture control registers high
  };
  __IO	uint16_t FRACMATCHREL_H[6];	// SCT fractional match reload registers high
    	uint16_t RESERVED7[10];

/* Events */

  __IO struct {
    uint32_t STATEMSK;			// SCT event state registers
    uint32_t CTRL;			// SCT event control registers
  } EVENT[16];

	uint32_t RESERVED8[96];

/* Outputs */

  __IO struct {
    uint32_t SET;			// SCT output state registers
    uint32_t CLEAR;			// SCT output clear registers
  } OUTCTRL[16];

} LPC_SCT_TypeDef;
/*@}*/ /* end of group LPC18xx_SCT */

/*------------- Timer0/1/2/3 (TIMER) ----------------------------*/
/** @addtogroup LPC18xx_TIMER LPC18xx Timer0/1/2/3 (TIMER) 
  @{
*/
typedef struct
{
  __IO	uint32_t IR;		// Interrupt Register
  __IO	uint32_t TCR;		// Timer Control Register
  __IO	uint32_t TC;		// Timer Counter
  __IO	uint32_t PR;		// Prescale Register
  __IO	uint32_t PC;		// Prescale Counter
  __IO	uint32_t MCR;		// Match Control Register
  union {
    struct {
      __IO uint32_t MR0;	// Match Register 0
      __IO uint32_t MR1;	// Match Register 1
      __IO uint32_t MR2;	// Match Register 2
      __IO uint32_t MR3;	// Match Register 3
    };
    __IO uint32_t MR[4];	// Match Registers
  };

  __IO	uint32_t CCR;		// Capture Control Register
  union {
    struct {
      __I uint32_t CR0;		// Capture Register 0
      __I uint32_t CR1;		// Capture Register 1
      __I uint32_t CR2;		// Capture Register 2
      __I uint32_t CR3;		// Capture Register 3
    };
    __I uint32_t CR[4];		// Capture Registers
  };

  __IO	uint32_t EMR;		// External Match Register
	uint32_t RESERVED0[12];
  __IO	uint32_t CTCR;		// Count Control Register
} LPC_TIMER_TypeDef;
/*@}*/ /* end of group LPC18xx_TIMER */

/*------------- Motor Control Pulse Width Modulator (MCPWM) (MCPWM) -------------------*/
/** @addtogroup LPC18xx_MCPWM LPC18xx Motor Control Pulse Width Modulator (MCPWM) (MCPWM) 
  @{
*/
typedef struct
{
  __I	uint32_t CON;		// PWM Control read address
  __O	uint32_t CON_SET;	// PWM Control set address
  __O	uint32_t CON_CLR;	// PWM Control clear address
  __I	uint32_t CAPCON;	// Capture Control read address
  __O	uint32_t CAPCON_SET;	// Capture Control set address
  __O	uint32_t CAPCON_CLR;	// Event Control clear address
  __IO	uint32_t TC0;		// Timer Counter register
  __IO	uint32_t TC1;		// Timer Counter register
  __IO	uint32_t TC2;		// Timer Counter register
  __IO	uint32_t LIM0;		// Limit register
  __IO	uint32_t LIM1;		// Limit register
  __IO	uint32_t LIM2;		// Limit register
  __IO	uint32_t MAT0;		// Match register
  __IO	uint32_t MAT1;		// Match register
  __IO	uint32_t MAT2;		// Match register
  __IO	uint32_t DT;		// Dead time register
  __IO	uint32_t MCCP;		// Communication Pattern register
  __I	uint32_t CAP0;		// Capture register
  __I	uint32_t CAP1;		// Capture register
  __I	uint32_t CAP2;		// Capture register
  __I	uint32_t INTEN;		// Interrupt Enable read address
  __O	uint32_t INTEN_SET;	// Interrupt Enable set address
  __O	uint32_t INTEN_CLR;	// Interrupt Enable clear address
  __I	uint32_t CNTCON;	// Count Control read address
  __O	uint32_t CNTCON_SET;	// Count Control set address
  __O	uint32_t CNTCON_CLR;	// Count Control clear address
  __I	uint32_t INTF;		// Interrupt flags read address
  __O	uint32_t INTF_SET;	// Interrupt flags set address
  __O	uint32_t INTF_CLR;	// Interrupt flags clear address
  __O	uint32_t CAP_CLR;	// Capture clear address
} LPC_MCPWM_TypeDef;
/*@}*/ /* end of group LPC18xx_MCPWM */

/*------------- QEI (QEI) ----------------------------*/
/** @addtogroup LPC18xx_QEI LPC18xx QEI (QEI) 
  @{
*/
typedef struct
{
  __O	uint32_t CON;		// Control register
  __I	uint32_t STAT;		// Encoder status register
  __IO	uint32_t CONF;		// Configuration register
  __I	uint32_t POS;		// Position register
  __IO	uint32_t MAXPOS;	// Maximum position register
  __IO	uint32_t CMPOS0;	// position compare register 0
  __IO	uint32_t CMPOS1;	// position compare register 1
  __IO	uint32_t CMPOS2;	// position compare register 2
  __I	uint32_t INXCNT;	// Index count register
  __IO	uint32_t INXCMP0;	// Index compare register 0
  __IO	uint32_t LOAD;		// Velocity timer reload register
  __I	uint32_t TIME;		// Velocity timer register
  __I	uint32_t VEL;		// Velocity counter register
  __I	uint32_t CAP;		// Velocity capture register
  __IO	uint32_t VELCOMP;	// Velocity compare register
  __IO	uint32_t FILTERPHA;	// Digital filter register on input
  __IO	uint32_t FILTERPHB;	// Digital filter register on input
  __IO	uint32_t FILTERINX;	// Digital filter register on input
  __IO	uint32_t WINDOW;	// Index acceptance window
  __IO	uint32_t INXCMP1;	// Index compare register 1
  __IO	uint32_t INXCMP2;	// Index compare register 2
	uint32_t RESERVED0[993];
  __O	uint32_t IEC;		// Interrupt enable clear register
  __O	uint32_t IES;		// Interrupt enable set register
} LPC_QEI_TypeDef;
/*@}*/ /* end of group LPC18xx_QEI */

/*------------- Repetitive Interrupt Timer (RIT) (RIT) ----------------------------*/
/** @addtogroup LPC18xx_RIT LPC18xx Repetitive Interrupt Timer (RIT) (RIT) 
  @{
*/
typedef struct
{
  __IO	uint32_t COMPVAL;	// Compare register
  __IO	uint32_t MASK;		// Mask register
  __IO	uint32_t CTRL;		// Control register
  __IO	uint32_t COUNTER;	// 32-bit counter
} LPC_RIT_TypeDef;
/*@}*/ /* end of group LPC18xx_RIT */

/*------------- Alarm timer (ATIMER) ----------------------------*/
/** @addtogroup LPC18xx_ATIMER LPC18xx Alarm timer (ATIMER) 
  @{
*/
typedef struct
{
  __IO	uint32_t DOWNCOUNTER;	// Downcounter register
  __IO	uint32_t PRESET;	// Preset value register
	uint32_t RESERVED0[1012];
  __O	uint32_t CLR_EN;	// Interrupt clear enable register
  __O	uint32_t SET_EN;	// Interrupt set enable register
  __I	uint32_t STATUS;	// Status register
  __I	uint32_t ENABLE;	// Enable register
  __O	uint32_t CLR_STAT;	// Clear register
  __O	uint32_t SET_STAT;	// Set register
} LPC_ATIMER_TypeDef;
/*@}*/ /* end of group LPC18xx_ATIMER */

/*------------- Watchdog timer (WDT) ----------------------------*/
/** @addtogroup LPC18xx_WDT LPC18xx Watchdog timer (WDT) 
  @{
*/
typedef struct
{
  __IO	uint32_t MOD;		// Watchdog mode register
  __IO	uint32_t TC;		// Watchdog timer constant register
  __O	uint32_t FEED;		// Watchdog feed sequence register
  __I	uint32_t TV;		// Watchdog timer value register
	uint32_t RESERVED0[1];
  __IO	uint32_t WARNINT;	// Watchdog warning interrupt register
  __IO	uint32_t WINDOW;	// Watchdog timer window register
} LPC_WDT_TypeDef;
/*@}*/ /* end of group LPC18xx_WDT */

/*------------- RTC (RTC) ----------------------------*/
/** @addtogroup LPC18xx_RTC LPC18xx RTC (RTC) 
  @{
*/
typedef struct
{
  __O	uint32_t ILR;		// Interrupt Location Register
	uint32_t RESERVED0[1];
  __IO	uint32_t CCR;		// Clock Control Register
  __IO	uint32_t CIIR;		// Counter Increment Interrupt Register 0x00
  __IO	uint32_t AMR;		// Alarm Mask Register
  __I	uint32_t CTIME0;	// Consolidated Time Register 0
  __I	uint32_t CTIME1;	// Consolidated Time Register 1
  __I	uint32_t CTIME2;	// Consolidated Time Register 2
  __IO	uint32_t SEC;		// Seconds Register
  __IO	uint32_t MIN;		// Minutes Register
  __IO	uint32_t HRS;		// Hours Register
  __IO	uint32_t DOM;		// Day of Month Register
  __IO	uint32_t DOW;		// Day of Week Register
  __IO	uint32_t DOY;		// Day of Year Register
  __IO	uint32_t MONTH;		// Months Register
  __IO	uint32_t YEAR;		// Years Register
  __IO	uint32_t CALIBRATION;	// Calibration Value Register
	uint32_t RESERVED1[7];
  __IO	uint32_t ASEC;		// Alarm Seconds Register
  __IO	uint32_t AMIN;		// Alarm Minutes Register
  __IO	uint32_t AHRS;		// Alarm Hours Register
  __IO	uint32_t ADOM;		// Alarm Day of Month Register
  __IO	uint32_t ADOW;		// Alarm Day of Week Register
  __IO	uint32_t ADOY;		// Alarm Day of Year Register
  __IO	uint32_t AMON;		// Alarm Month Register
  __IO	uint32_t AYRS;		// Alarm Year Register
} LPC_RTC_TypeDef;
/*@}*/ /* end of group LPC18xx_RTC */

/*------------- REGFILE (REGFILE) ----------------------------*/
/** @addtogroup LPC18xx_REGFILE LPC18xx REGFILE (REGFILE) 
  @{
*/
typedef struct
{
  __IO	uint32_t REGFILE[64];	// General purpose storage register
} LPC_REGFILE_TypeDef;
/*@}*/ /* end of group LPC18xx_REGFILE */

/*------------- event monitor/recorder (EMON) ----------------------------*/
/** @addtogroup LPC18xx_EMON LPC18xx event monitor/recorder (EMON) 
  @{
*/
typedef struct
{
	uint32_t RESERVED0[32];
  __IO	uint32_t STATUS;		// Event Mon/Rec Status register
  __IO	uint32_t CONTROL;		// Event Mon/Rec Control register
  __I	uint32_t COUNT;			// Event Mon/Rec Counters register
	uint32_t RESERVED1[1];
  __I	uint32_t FIRSTSTAMP0;		// Event Mon/Rec First Stamp register for channel 0
  __I	uint32_t FIRSTSTAMP1;		// Event Mon/Re First Stamp register for channel 1
  __I	uint32_t FIRSTSTAMP2;		// Event Mon/Rec First Stamp register for channel 2
	uint32_t RESERVED2[1];
  __I	uint32_t LASTSTAMP0;		// Event Mon/Rec Last Stamp register for channel 0
  __I	uint32_t LASTSTAMP1;		// Event Mon/Rec Last Stamp register for channel 1
  __I	uint32_t LASTSTAMP2;		// Event Mon/Rec Last Stamp register for channel 2
} LPC_EMON_TypeDef;
/*@}*/ /* end of group LPC18xx_EMON */

/*------------- USART0/2/3 (USART) ----------------------------*/
/** @addtogroup LPC18xx_USART LPC18xx USART0/2/3 (USART) 
  @{
*/
typedef struct
{
  union {
    __I  uint32_t RBR;		// Receiver Buffer Register
    __O  uint32_t THR;		// Transmit Holding Register
    __IO uint32_t DLL;		// Divisor Latch LSB
  };
  union {
    __IO uint32_t DLM;		// Divisor Latch MSB
    __IO uint32_t IER;		// Interrupt Enable Register
  };
  union {
    __I uint32_t IIR;		// Interrupt ID Register
    __O uint32_t FCR;		// FIFO Control Register
  };

  __IO	uint32_t LCR;		// Line Control Register
	uint32_t RESERVED0[1];
  __I	uint32_t LSR;		// Line Status Register
	uint32_t RESERVED1[1];
  __IO	uint32_t SCR;		// Scratch Pad Register
  __IO	uint32_t ACR;		// Auto-baud Control Register
  __IO	uint32_t ICR;		// IrDA control register
  __IO	uint32_t FDR;		// Fractional Divider Register
  __IO	uint32_t OSR;		// Oversampling Register
	uint32_t RESERVED2[4];
  __IO	uint32_t HDEN;		// Half-duplex enable Register
	uint32_t RESERVED3[1];
  __IO	uint32_t SCICTRL;	// Smart card interface control register
  __IO	uint32_t RS485CTRL;	// RS-485/EIA-485 Control
  __IO	uint32_t RS485ADRMATCH;	// RS-485/EIA-485 address match
  __IO	uint32_t RS485DLY;	// RS-485/EIA-485 direction control delay
  __IO	uint32_t SYNCCTRL;	// Synchronous mode control register
  __IO	uint32_t TER;		// Transmit Enable Register
} LPC_USART_TypeDef;
/*@}*/ /* end of group LPC18xx_USART */

/*------------- UART1 (UART) ----------------------------*/
/** @addtogroup LPC18xx_UART LPC18xx UART1 (UART) 
  @{
*/
typedef struct
{
  union {
    __I  uint32_t RBR;		// Receiver Buffer Register
    __O  uint32_t THR;		// Transmit Holding Register
    __IO uint32_t DLL;		// Divisor Latch LSB
  };
  union {
    __IO uint32_t DLM;		// Divisor Latch MSB
    __IO uint32_t IER;		// Interrupt Enable Register
  };
  union {
    __I uint32_t IIR;		// Interrupt ID Register
    __O uint32_t FCR;		// FIFO Control Register
  };
  __IO	uint32_t LCR;		// Line Control Register
  __IO	uint32_t MCR;		// Modem Control Register
  __I	uint32_t LSR;		// Line Status Register
  __I	uint32_t MSR;		// Modem Status Register
  __IO	uint32_t SCR;		// Scratch Pad Register
  __IO	uint32_t ACR;		// Auto-baud Control Register
	uint32_t RESERVED0[1];
  __IO	uint32_t FDR;		// Fractional Divider Register
	uint32_t RESERVED1[8];
  __IO	uint32_t RS485CTRL;	// RS-485/EIA-485 Control
  __IO	uint32_t RS485ADRMATCH;	// RS-485/EIA-485 address match
  __IO	uint32_t RS485DLY;	// RS-485/EIA-485 direction control delay
	uint32_t RESERVED2[1];
  __IO	uint32_t TER;		// Transmit Enable Register
} LPC_UART_TypeDef;
/*@}*/ /* end of group LPC18xx_UART */

/*------------- SSP (SSP) ----------------------------*/
/** @addtogroup LPC18xx_SSP LPC18xx SSP (SSP) 
  @{
*/
typedef struct
{
  __IO	uint32_t CR0;		// Control Register 0
  __IO	uint32_t CR1;		// Control Register 1
  __IO	uint32_t DR;		// Data Register
  __I	uint32_t SR;		// Status Register
  __IO	uint32_t CPSR;		// Clock Prescale Register
  __IO	uint32_t IMSC;		// Interrupt Mask Set and Clear Register
  __I	uint32_t RIS;		// Raw Interrupt Status Register
  __I	uint32_t MIS;		// Masked Interrupt Status Register
  __O	uint32_t ICR;		// SSPICR Interrupt Clear Register
  __IO	uint32_t DMACR;		// SSP0 DMA control register
} LPC_SSP_TypeDef;
/*@}*/ /* end of group LPC18xx_SSP */

/*------------- I2S (I2S) ----------------------------*/
/** @addtogroup LPC18xx_I2S LPC18xx I2S (I2S) 
  @{
*/
typedef struct
{
  __IO	uint32_t DAO;		// I2S Digital Audio Output Register
  __IO	uint32_t DAI;		// I2S Digital Audio Input Register
  __O	uint32_t TXFIFO;	// I2S Transmit FIFO
  __I	uint32_t RXFIFO;	// I2S Receive FIFO
  __I	uint32_t STATE;		// I2S Status Feedback Register
  __IO	uint32_t DMA1;		// I2S DMA Configuration Register 1
  __IO	uint32_t DMA2;		// I2S DMA Configuration Register 2
  __IO	uint32_t IRQ;		// I2S Interrupt Request Control Register
  __IO	uint32_t TXRATE;	// I2S Transmit MCLK divider
  __IO	uint32_t RXRATE;	// I2S Receive MCLK divider
  __IO	uint32_t TXBITRATE;	// I2S Transmit bit rate divider
  __IO	uint32_t RXBITRATE;	// I2S Receive bit rate divider
  __IO	uint32_t TXMODE;	// I2S Transmit mode control
  __IO	uint32_t RXMODE;	// I2S Receive mode control
} LPC_I2S_TypeDef;
/*@}*/ /* end of group LPC18xx_I2S */

/*------------- I2C (I2C) ----------------------------*/
/** @addtogroup LPC18xx_I2C LPC18xx I2C (I2C) 
  @{
*/
typedef struct
{
  __IO	uint32_t CONSET;	// I2C Control Set Register
  __I	uint32_t STAT;		// I2C Status Register
  __IO	uint32_t DAT;		// I2C Data Register
  __IO	uint32_t ADR0;		// I2C Slave Address Register 0
  __IO	uint32_t SCLH;		// SCH Duty Cycle Register High Half Word
  __IO	uint32_t SCLL;		// SCL Duty Cycle Register Low Half Word
  __O	uint32_t CONCLR;	// I2C Control Clear Register
  __IO	uint32_t MMCTRL;	// Monitor mode control register
  __IO	uint32_t ADR1;		// I2C Slave Address Register 1
  __IO	uint32_t ADR2;		// I2C Slave Address Register 2
  __IO	uint32_t ADR3;		// I2C Slave Address Register 3
  __I	uint32_t DATA_BUFFER;	// Data buffer register
  __IO	uint32_t MASK0;		// I2C Slave address mask register 0
  __IO	uint32_t MASK1;		// I2C Slave address mask register 1
  __IO	uint32_t MASK2;		// I2C Slave address mask register 2
  __IO	uint32_t MASK3;		// I2C Slave address mask register 3
} LPC_I2C_TypeDef;
/*@}*/ /* end of group LPC18xx_I2C */

/*------------- C_CAN (CAN) ----------------------------*/
/** @addtogroup LPC18xx_CAN0 LPC18xx C_CAN (CAN) 
  @{
*/
typedef struct
{
  __IO	uint32_t CNTL;		// CAN control register
  __IO	uint32_t STAT;		// Status register
  __I	uint32_t EC;		// Error counter register
	uint32_t RESERVED0[1];
  __I	uint32_t INT;		// Interrupt register
  __IO	uint32_t TEST;		// Test register
  __IO	uint32_t BRPE;		// Baud rate prescaler extension register
	uint32_t RESERVED1[1];
  __IO	uint32_t IF1_CMDREQ;	// Message interface 1 command request
  __IO	uint32_t IF1_CMDMSK_R;	// Message interface 1 command mask
  __IO	uint32_t IF1_MSK1;	// Message interface 1 mask 1
  __IO	uint32_t IF1_MSK2;	// Message interface 1 mask 2
  __IO	uint32_t IF1_ARB1;	// Message interface 1 arbitration 1
  __IO	uint32_t IF1_ARB2;	// Message interface 1 arbitration 2
  __IO	uint32_t IF1_MCTRL;	// Message interface 1 message control
  __IO	uint32_t IF1_DA1;	// Message interface 1 data A1
  __IO	uint32_t IF1_DA2;	// Message interface 1 data A2
  __IO	uint32_t IF1_DB1;	// Message interface 1 data B1
  __IO	uint32_t IF1_DB2;	// Message interface 1 data B2
	uint32_t RESERVED2[13];
  __IO	uint32_t IF2_CMDREQ;	// Message interface 2 command request
  __IO	uint32_t IF2_CMDMSK_R;	// Message interface 2 command mask
  __IO	uint32_t IF2_MSK1;	// Message interface 2 mask 1
  __IO	uint32_t IF2_MSK2;	// Message interface 2 mask 2
  __IO	uint32_t IF2_ARB1;	// Message interface 2 arbitration 1
  __IO	uint32_t IF2_ARB2;	// Message interface 2 arbitration 2
  __IO	uint32_t IF2_MCTRL;	// Message interface 2 message control
  __IO	uint32_t IF2_DA1;	// Message interface 2 data A1
  __IO	uint32_t IF2_DA2;	// Message interface 2 data A2
  __IO	uint32_t IF2_DB1;	// Message interface 2 data B1
  __IO	uint32_t IF2_DB2;	// Message interface 2 data B2
	uint32_t RESERVED3[21];
  __I	uint32_t TXREQ1;	// Transmission request 1
  __I	uint32_t TXREQ2;	// Transmission request 2
	uint32_t RESERVED4[6];
  __I	uint32_t ND1;		// New data 1
  __I	uint32_t ND2;		// New data 2
	uint32_t RESERVED5[6];
  __I	uint32_t IR1;		// Interrupt pending 1
  __I	uint32_t IR2;		// Interrupt pending 2
	uint32_t RESERVED6[6];
  __I	uint32_t MSGV1;		// Message valid 1
  __I	uint32_t MSGV2;		// Message valid 2
	uint32_t RESERVED7[6];
  __IO	uint32_t CLKDIV;		// CAN clock divider register
} LPC_CAN_TypeDef;
/*@}*/ /* end of group LPC18xx_CAN */

/*------------- ADC (ADC) ----------------------------*/
/** @addtogroup LPC18xx_ADC LPC18xx ADC (ADC) 
  @{
*/
typedef struct
{
  __IO	uint32_t CR;		// A/D Control Register
	uint32_t RESERVED0[2];
  __IO	uint32_t INTEN;		// A/D Interrupt Enable Register
  __I	uint32_t DR[8];		// A/D Channel Data Register
  __I	uint32_t STAT;		// A/D Status Register
} LPC_ADC_TypeDef;
/*@}*/ /* end of group LPC18xx_ADC */

/*------------- DAC (DAC) ----------------------------*/
/** @addtogroup LPC18xx_DAC LPC18xx DAC (DAC) 
  @{
*/
typedef struct
{
  __IO	uint32_t CR;		// DAC register
  __IO	uint32_t CTRL;		// DAC control register
  __IO	uint32_t CNTVAL;	// DAC counter value register
} LPC_DAC_TypeDef;
/*@}*/ /* end of group LPC18xx_DAC */

/*------------- FMC controller for flash bank A/B (FMC) ----------------------------*/
/** @addtogroup LPC18xx_FMC LPC18xx FMC controller for flash bank A/B (FMC) 
  @{
*/
typedef struct
{
	uint32_t RESERVED0[8];
  __IO	uint32_t START;		// Signature start address register
  __IO	uint32_t STOP;		// Signature stop-address register
	uint32_t RESERVED1[1];
  __I	uint32_t W0;		// 128-bit signature Word 0
  __I	uint32_t W1;		// 128-bit signature Word 1
  __I	uint32_t W2;		// 128-bit signature Word 2
  __I	uint32_t W3;		// 128-bit signature Word 3
	uint32_t RESERVED2[1001];
  __I	uint32_t TAT;		// Signature generation status register
	uint32_t RESERVED3[1];
  __O	uint32_t TATCLR;	// Signature generation status clear register
} LPC_FMC_TypeDef;
/*@}*/ /* end of group LPC18xx_FMC */

/*------------- EEPROM (EEPROM) ----------------------------*/
/** @addtogroup LPC18xx_EEPROM LPC18xx EEPROM (EEPROM) 
  @{
*/
typedef struct
{
  __IO	uint32_t CMD;		// EEPROM command register
	uint32_t RESERVED0[1];
  __IO	uint32_t RWSTATE;	// EEPROM read wait state
  __IO	uint32_t AUTOPROG;	// EEPROM auto programming
  __IO	uint32_t WSTATE;	// EEPROM wait state register
  __IO	uint32_t CLKDIV;	// EEPROM clock divider register 0x0000
  __IO	uint32_t PWRDWN;	// EEPROM power-down register
	uint32_t RESERVED1[1007];
  __O	uint32_t INTENCLR;	// EEPROM interrupt enable clear 0
  __O	uint32_t INTENSET;	// EEPROM interrupt enable set
  __I	uint32_t INTSTAT;	// EEPROM interrupt status
  __I	uint32_t INTEN;		// EEPROM interrupt enable
  __O	uint32_t INTSTATCLR;	// EEPROM interrupt status clear
  __O	uint32_t INTSTATSET;	// EEPROM interrupt status set
} LPC_EEPROM_TypeDef;
/*@}*/ /* end of group LPC18xx_EEPROM */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/*@}*/ /* end of group LPC18xx_Peripherals */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* ToDo: add here your device peripherals base addresses
         following is an example for timer                                    */
/** @addtogroup LPC18xx_MemoryMap LPC18xx Memory Mapping
  @{
*/

#define LPC_FLASH_BASE		(0x1A0000000UL)
#define LPC_SRAM_BASE		(0x100000000UL)

/*@}*/ /* end of group LPC18xx_MemoryMap */


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
/* ToDo: add here your device peripherals pointer definitions
         following is an example for timer                                    */

/** @addtogroup LPC18xx_PeripheralDecl LPC18xx Peripheral Declaration
  @{
*/

#define LPC_RGU			((LPC_RGU_TypeDef*	) 0x40053000) )
#define LPC_NVIC		((LPC_NVIC_TypeDef*	) 0xE000E000) )
#define LPC_EVENTROUTER		((LPC_EVENTROUTER_TypeDef*	) 0x40044000) )
#define LPC_CREG		((LPC_CREG_TypeDef*	) 0x40043000) )
#define LPC_PMC			((LPC_PMC_TypeDef*	) 0x40042000) )
#define LPC_CGU			((LPC_CGU_TypeDef*	) 0x40050000) )
#define LPC_CCU1		((LPC_CCU1_TypeDef*	) 0x40051000) )
#define LPC_CCU2		((LPC_CCU2_TypeDef*	) 0x40052000) )
#define LPC_SCU			((LPC_SCU_TypeDef*	) 0x40086000) )
#define LPC_GIMA		((LPC_GIMA_TypeDef*	) 0x400C7000) )
#define LPC_GPIO_PIN_INT	((LPC_GPIO_PIN_INT_TypeDef*	) 0x40087000) )
#define LPC_GPIO_GROUP_INT0	((LPC_GPIO_GROUP_INT_TypeDef*	) 0x40088000) )
#define LPC_GPIO_GROUP_INT1	((LPC_GPIO_GROUP_INT_TypeDef*	) 0x40089000) )
#define LPC_GPIO		((LPC_GPIO_TypeDef*	) 0x400F4000) )
#define LPC_GPDMA		((LPC_GPDMA_TypeDef*	) 0x40002000) )
#define LPC_SDMMC		((LPC_SDMMC_TypeDef*	) 0x40004000) )
#define LPC_SPIFI		((LPC_SPIFI_TypeDef*	) 0x40003000) )
#define LPC_EMC			((LPC_EMC_TypeDef*	) 0x40005000) )
#define LPC_USB0		((LPC_USB0_TypeDef*	) 0x40006000) )
#define LPC_USB1		((LPC_USB1_TypeDef*	) 0x40007000) )
#define LPC_EMAC		((LPC_EMAC_TypeDef*	) 0x40010000) )
#define LPC_LCD			((LPC_LCD_TypeDef*	) 0x40008000) )
#define LPC_SCT			((LPC_SCT_TypeDef*	) 0x40000000) )
#define LPC_TIMER0		((LPC_TIMER_TypeDef*	) 0x40084000) )
#define LPC_TIMER1		((LPC_TIMER_TypeDef*	) 0x40085000) )
#define LPC_TIMER2		((LPC_TIMER_TypeDef*	) 0x400C3000) )
#define LPC_TIMER3		((LPC_TIMER_TypeDef*	) 0x400C4000) )
#define LPC_MCPWM		((LPC_MCPWM_TypeDef*	) 0x400A0000) )
#define LPC_QEI			((LPC_QEI_TypeDef*	) 0x400C6000) )
#define LPC_RIT			((LPC_RIT_TypeDef*	) 0x400C0000) )
#define LPC_ATIMER		((LPC_ATIMER_TypeDef*	) 0x40040000) )
#define LPC_WDT			((LPC_WDT_TypeDef*	) 0x40080000) )
#define LPC_RTC			((LPC_RTC_TypeDef*	) 0x40046000) )
#define LPC_REGFILE		((LPC_REGFILE_TypeDef*	) 0x40041000) )
#define LPC_EMON		((LPC_EMON_TypeDef*	) 0x40046000) )
#define LPC_USART0		((LPC_USART_TypeDef*	) 0x40081000) )
#define LPC_UART1		((LPC_UART_TypeDef*	) 0x40082000) )
#define LPC_USART2		((LPC_USART_TypeDef*	) 0x400C1000) )
#define LPC_USART3		((LPC_USART_TypeDef*	) 0x400C2000) )
#define LPC_SSP0		((LPC_SSP_TypeDef*	) 0x40083000) )
#define LPC_SSP1		((LPC_SSP_TypeDef*	) 0x400C5000) )
#define LPC_I2S0		((LPC_I2S_TypeDef*	) 0x400A2000) )
#define LPC_I2S1		((LPC_I2S_TypeDef*	) 0x400A3000) )
#define LPC_I2C0		((LPC_I2C_TypeDef*	) 0x400A1000) )
#define LPC_I2C1		((LPC_I2C_TypeDef*	) 0x400E0000) )
#define LPC_CAN0		((LPC_CAN_TypeDef*	) 0x400E2000) )
#define LPC_CAN1		((LPC_CAN_TypeDef*	) 0x400A4000) )
#define LPC_ADC0		((LPC_ADC_TypeDef*	) 0x400E3000) )
#define LPC_ADC1		((LPC_ADC_TypeDef*	) 0x400E4000) )
#define LPC_DAC			((LPC_DAC_TypeDef*	) 0x400E1000) )
#define LPC_FMCA		((LPC_FMC_TypeDef*	) 0x4000C000) )
#define LPC_FMCB		((LPC_FMC_TypeDef*	) 0x4000D000) )
#define LPC_EEPROM		((LPC_EEPROM_TypeDef*	) 0x4000E000) )

/*@}*/ /* end of group LPC18xx_PeripheralDecl */

/*@}*/ /* end of group LPC18xx_Definitions */

#ifdef __cplusplus
}
#endif

#endif  /* LPC18xx_H */
