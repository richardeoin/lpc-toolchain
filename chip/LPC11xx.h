/**************************************************************************//**
 * @file     LPC11xx.h
 * @brief    CMSIS Cortex-M0 Core Peripheral Access Layer Header File for
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


#ifndef LPC11xx_H
#define LPC11xx_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup LPC11xx_Definitions LPC11xx Definitions
  This file defines all structures and symbols for LPC11xx:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/


/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup LPC11xx_CMSIS Device CMSIS Definitions
  Configuration of the Cortex-M0 Processor and Core Peripherals
  @{
*/

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers ***************************************************/

  NonMaskableInt_IRQn           = -14,      /*!<  2 Non Maskable Interrupt            */
  HardFault_IRQn                = -13,      /*!<  3 Hard Fault Interrupt              */
  SVCall_IRQn                   = -5,       /*!< 11 SV Call Interrupt                 */
  PendSV_IRQn                   = -2,       /*!< 14 Pend SV Interrupt                 */
  SysTick_IRQn                  = -1,       /*!< 15 System Tick Interrupt             */

/******  Device Specific Interrupt Numbers ********************************************************/

  WAKEUP0_IRQn                  = 0,        /*!< Wakeup                               */
  WAKEUP1_IRQn                  = 1,        /*!< Wakeup                               */
  WAKEUP2_IRQn                  = 2,        /*!< Wakeup                               */
  WAKEUP3_IRQn                  = 3,        /*!< Wakeup                               */
  WAKEUP4_IRQn                  = 4,        /*!< Wakeup                               */
  WAKEUP5_IRQn                  = 5,        /*!< Wakeup                               */
  WAKEUP6_IRQn                  = 6,        /*!< Wakeup                               */
  WAKEUP7_IRQn                  = 7,        /*!< Wakeup                               */
  WAKEUP8_IRQn                  = 8,        /*!< Wakeup                               */
  WAKEUP9_IRQn                  = 9,        /*!< Wakeup                               */
  WAKEUP10_IRQn                 = 10,       /*!< Wakeup                               */
  WAKEUP11_IRQn                 = 11,       /*!< Wakeup                               */
  WAKEUP12_IRQn                 = 12,       /*!< Wakeup                               */
  CAN_IRQn                      = 13,       /*!< CAN                                  */
  SSP1_IRQn                     = 14,       /*!< SSP1                                 */
  I2C_IRQn                      = 15,       /*!< I2C                                  */
  TIMER_16_0_IRQn               = 16,       /*!< 16-bit 0                             */
  TIMER_16_1_IRQn               = 17,       /*!< 16-bit 1                             */
  TIMER_32_0_IRQn               = 18,       /*!< 32-bit 0                             */
  TIMER_32_1_IRQn               = 19,       /*!< 32-bit 1                             */
  SSP0_IRQn                     = 20,       /*!< SSP0                                 */
  UART_IRQn                     = 21,       /*!< UART                                 */
  ADC_IRQn                      = 24,       /*!< A/D Converter                        */
  WDT_IRQn                      = 25,       /*!< Watchdog                             */
  BOD_IRQn                      = 26,       /*!< Brown Out                            */
  EINT3_IRQn                    = 28,       /*!< External                             */
  EINT2_IRQn                    = 29,       /*!< External                             */
  EINT1_IRQn                    = 30,       /*!< External                             */
  EINT0_IRQn                    = 31,       /*!< External                             */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __CM0_REV                 0x0201    /*!< Core Revision r2p1                   */
#define __NVIC_PRIO_BITS          2         /*!< Number of Bits for Priority Levels   */
#define __Vendor_SysTickConfig    0         /*!< Set if different SysTick Config      */
#define __MPU_PRESENT             0         /*!< MPU present or not                   */

/*@}*/ /* end of group LPC11xx_CMSIS */

#include <core_cm0.h>                      /* Cortex-M0 processor and core peripherals */
#include "system_LPC11xx.h"                /* LPC11xx System include file              */

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup LPC11xx_Peripherals LPC11xx Peripherals
  LPC11xx Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/*------------- system control block (SYSCON) ----------------------------*/
/** @addtogroup LPC11xx_SYSCON LPC11xx system control block (SYSCON) 
  @{
*/
typedef struct
{
  __IO	uint32_t SYSMEMREMAP;		// System memory remap
  __IO	uint32_t PRESETCTRL;		// Peripheral reset control
  __IO	uint32_t SYSPLLCTRL;		// System PLL control
  __I	uint32_t SYSPLLSTAT;		// System PLL status
	uint32_t RESERVED0[4];
  __IO	uint32_t SYSOSCCTRL;		// System oscillator control
  __IO	uint32_t WDTOSCCTRL;		// Watchdog oscillator control
  __IO	uint32_t IRCCTRL;		// IRC control
	uint32_t RESERVED1[1];
  __I	uint32_t SYSRSTSTAT;		// System reset status register
	uint32_t RESERVED2[3];
  __IO	uint32_t SYSPLLCLKSEL;		// System PLL clock source select
  __IO	uint32_t SYSPLLCLKUEN;		// System PLL clock source update enable
	uint32_t RESERVED3[10];
  __IO	uint32_t MAINCLKSEL;		// Main clock source select
  __IO	uint32_t MAINCLKUEN;		// Main clock source update enable
  __IO	uint32_t SYSAHBCLKDIV;		// System AHB clock divider
	uint32_t RESERVED4[1];
  __IO	uint32_t SYSAHBCLKCTRL;		// System AHB clock control
	uint32_t RESERVED5[4];
  __IO	uint32_t SSP0CLKDIV;		// SPI0 clock divider
  __IO	uint32_t UARTCLKDIV;		// UART clock divder
  __IO	uint32_t SSP1CLKDIV;		// SPI1 clock divder
	uint32_t RESERVED6[12];
  __IO	uint32_t WDTCLKSEL;		// WDT clock source select
  __IO	uint32_t WDTCLKUEN;		// WDT clock source update enable
  __IO	uint32_t WDTCLKDIV;		// WDT clock divider
	uint32_t RESERVED7[1];
  __IO	uint32_t CLKOUTCLKSEL;		// CLKOUT clock source select
  __IO	uint32_t CLKOUTUEN;		// CLKOUT clock source update enable
  __IO	uint32_t CLKOUTCLKDIV;		// CLKOUT clock divider
	uint32_t RESERVED8[5];
  __I	uint32_t PIOPORCAP0;		// POR captured PIO status 0
  __I	uint32_t PIOPORCAP1;		// POR captured PIO status 1
	uint32_t RESERVED9[18];
  __IO	uint32_t BODCTRL;		// BOD control
  __IO	uint32_t SYSTCKCAL;		// System tick counter calibration
	uint32_t RESERVED10[42];
  __IO	uint32_t STARTAPRP0;		// Start logic edge control register 0
  __IO	uint32_t STARTERP0;		// Start logic signal enable register 0
  __O	uint32_t STARTRSRP0CLR;		// Start logic reset register 0
  __I	uint32_t STARTSRP0;		// Start logic status register 0
	uint32_t RESERVED11[8];
  __IO	uint32_t PDSLEEPCFG;		// Power-down states in Deep-sleep mode
  __IO	uint32_t PDAWAKECFG;		// Power-down states after wake-up from
  __IO	uint32_t PDRUNCFG;		// Power-down configuration register
	uint32_t RESERVED12[110];
  __I	uint32_t DEVICE_ID;		// Device ID
} LPC_SYSCON_TypeDef;
/*@}*/ /* end of group LPC11xx_SYSCON */

/*------------- PMU (PMU) ----------------------------*/
/** @addtogroup LPC11xx_PMU LPC11xx PMU (PMU) 
  @{
*/
typedef struct
{
  __IO	uint32_t PCON;		// Power control register
  __IO	uint32_t GPREG0;		// General purpose register 0
  __IO	uint32_t GPREG1;		// General purpose register 1
  __IO	uint32_t GPREG2;		// General purpose register 2
  __IO	uint32_t GPREG3;		// General purpose register 3
  __IO	uint32_t GPREG4;		// General purpose register 4
} LPC_PMU_TypeDef;
/*@}*/ /* end of group LPC11xx_PMU */

/*------------- I/O configuration (IOCON) ----------------------------*/
/** @addtogroup LPC11xx_IOCON LPC11xx I/O configuration (IOCON) 
  @{
*/
typedef struct
{
  __IO	uint32_t PIO2_6;		// I/O configuration for pin PIO2_6
	uint32_t RESERVED0[1];
  __IO	uint32_t PIO2_0;		// I/O configuration for pin
  __IO	uint32_t RESET_PIO0_0;		// I/O configuration for pin RESET/PIO0_0 0xD0
  __IO	uint32_t PIO0_1;		// I/O configuration for pin
  __IO	uint32_t PIO1_8;		// I/O configuration for pin
	uint32_t RESERVED1[1];
  __IO	uint32_t PIO0_2;		// I/O configuration for pin
  __IO	uint32_t PIO2_7;		// I/O configuration for pin PIO2_7
  __IO	uint32_t PIO2_8;		// I/O configuration for pin PIO2_8
  __IO	uint32_t PIO2_1;		// I/O configuration for pin
  __IO	uint32_t PIO0_3;		// I/O configuration for pin PIO0_3
  __IO	uint32_t PIO0_4;		// I/O configuration for pin PIO0_4/SCL
  __IO	uint32_t PIO0_5;		// I/O configuration for pin PIO0_5/SDA
  __IO	uint32_t PIO1_9;		// I/O configuration for pin
  __IO	uint32_t PIO3_4;		// I/O configuration for pin PIO3_4
  __IO	uint32_t PIO2_4;		// I/O configuration for pin PIO2_4
  __IO	uint32_t PIO2_5;		// I/O configuration for pin PIO2_5
  __IO	uint32_t PIO3_5;		// I/O configuration for pin PIO3_5
  __IO	uint32_t PIO0_6;		// I/O configuration for pin PIO0_6/SCK0
  __IO	uint32_t PIO0_7;		// I/O configuration for pin PIO0_7/CTS
  __IO	uint32_t PIO2_9;		// I/O configuration for pin PIO2_9
  __IO	uint32_t PIO2_10;		// I/O configuration for pin PIO2_10
  __IO	uint32_t PIO2_2;		// I/O configuration for pin
  __IO	uint32_t PIO0_8;		// I/O configuration for pin
  __IO	uint32_t PIO0_9;		// I/O configuration for pin
  __IO	uint32_t SWCLK_PIO0_10;		// I/O configuration for pin
  __IO	uint32_t PIO1_10;		// I/O configuration for pin
  __IO	uint32_t PIO2_11;		// I/O configuration for pin PIO2_11/SCK0 0xD0
  __IO	uint32_t R_PIO0_11;		// I/O configuration for pin
  __IO	uint32_t R_PIO1_0;		// I/O configuration for pin
  __IO	uint32_t R_PIO1_1;		// I/O configuration for pin
  __IO	uint32_t R_PIO1_2;		// I/O configuration for pin
  __IO	uint32_t PIO3_0;		// I/O configuration for pin PIO3_0/DTR
  __IO	uint32_t PIO3_1;		// I/O configuration for pin PIO3_1/DSR
  __IO	uint32_t PIO2_3;		// I/O configuration for pin
  __IO	uint32_t SWDIO_PIO1_3;		// I/O configuration for pin
  __IO	uint32_t PIO1_4;		// I/O configuration for pin
  __IO	uint32_t PIO1_11;		// I/O configuration for pin PIO1_11/AD7
  __IO	uint32_t PIO3_2;		// I/O configuration for pin PIO3_2/DCD
  __IO	uint32_t PIO1_5;		// I/O configuration for pin
  __IO	uint32_t PIO1_6;		// I/O configuration for pin
  __IO	uint32_t PIO1_7;		// I/O configuration for pin
  __IO	uint32_t PIO3_3;		// I/O configuration for pin PIO3_3/RI
  __IO	uint32_t SCK_LOC;		// SCK pin location select register
  __IO	uint32_t DSR_LOC;		// DSR pin location select register
  __IO	uint32_t DCD_LOC;		// DCD pin location select register
  __IO	uint32_t RI_LOC;		// RI pin location register
} LPC_IOCON_TypeDef;
/*@}*/ /* end of group LPC11xx_IOCON */

/*------------- GPIO (GPIO) ----------------------------*/
/** @addtogroup LPC11xx_GPIO LPC11xx GPIO (GPIO) 
  @{
*/
typedef struct
{
  union {
    __IO uint32_t MASKED_ACCESS[4096]; // Port n data address masking register
    struct {
      uint32_t RESERVED0[4094];
      __IO uint32_t DATA; // Port n data register for pins PIOn_0 to
    };
  };
	uint32_t RESERVED1[4096];
  __IO	uint32_t DIR;		// Data direction register for port n
  __IO	uint32_t IS;		// Interrupt sense register for port n
  __IO	uint32_t IBE;		// Interrupt both edges register for port n
  __IO	uint32_t IEV;		// Interrupt event register for port n
  __IO	uint32_t IE;		// Interrupt mask register for port n
  __I	uint32_t RIS;		// Raw interrupt status register for port n
  __I	uint32_t MIS;		// Masked interrupt status register for port n
  __O	uint32_t IC;		// Interrupt clear register for port n
} LPC_GPIO_TypeDef;
/*@}*/ /* end of group LPC11xx_GPIO */

/*------------- UART (UART) ----------------------------*/
/** @addtogroup LPC11xx_UART LPC11xx UART (UART) 
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
    __O	uint32_t FCR;		// FIFO Control Register
  };
  __IO	uint32_t LCR;		// Line Control Register
  __IO	uint32_t MCR;		// Modem control register
  __I	uint32_t LSR;		// Line Status Register
  __I	uint32_t MSR;		// Modem status register
  __IO	uint32_t SCR;		// Scratch Pad Register
  __IO	uint32_t ACR;		// Auto-baud Control Register
	uint32_t RESERVED0[1];
  __IO	uint32_t FDR;		// Fractional Divider Register
	uint32_t RESERVED1[1];
  __IO	uint32_t TER;		// Transmit Enable Register
	uint32_t RESERVED2[6];
  __IO	uint32_t RS485CTRL;		// RS-485/EIA-485 Control
  __IO	uint32_t RS485ADR;		// RS-485/EIA-485 address match
} LPC_UART_TypeDef;
/*@}*/ /* end of group LPC11xx_UART */

/*------------- SPI0 (SPI0) ----------------------------*/
/** @addtogroup LPC11xx_SPI0 LPC11xx SPI0 (SPI0) 
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
} LPC_SPI0_TypeDef;
/*@}*/ /* end of group LPC11xx_SPI0 */

/*------------- SPI1 (SPI1) ----------------------------*/
/** @addtogroup LPC11xx_SPI1 LPC11xx SPI1 (SPI1) 
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
} LPC_SPI1_TypeDef;
/*@}*/ /* end of group LPC11xx_SPI1 */

/*------------- I2C (I2C) ----------------------------*/
/** @addtogroup LPC11xx_I2C LPC11xx I2C (I2C) 
  @{
*/
typedef struct
{
  __IO	uint32_t CONSET;		// I2C Control Set Register
  __I	uint32_t STAT;		// I2C Status Register
  __IO	uint32_t DAT;		// I2C Data Register
  __IO	uint32_t ADR0;		// I2C Slave Address Register 0
  __IO	uint32_t SCLH;		// SCH Duty Cycle Register High Half Word
  __IO	uint32_t SCLL;		// SCL Duty Cycle Register Low Half Word
  __O	uint32_t CONCLR;		// I2C Control Clear Register
  __IO	uint32_t MMCTRL;		// Monitor mode control register
  __IO  uint32_t ADR1;          // I2C Slave Address Register 1
  __IO  uint32_t ADR2;          // I2C Slave Address Register 2
  __IO  uint32_t ADR3;          // I2C Slave Address Register 3
  __I   uint32_t DATA_BUFFER;   // Data buffer register
  __IO  uint32_t MASK0;         // I2C Slave address mask register 0
  __IO  uint32_t MASK1;         // I2C Slave address mask register 1
  __IO  uint32_t MASK2;         // I2C Slave address mask register 2
  __IO  uint32_t MASK3;         // I2C Slave address mask register 3
} LPC_I2C_TypeDef;
/*@}*/ /* end of group LPC11xx_I2C */

/*------------- CCAN (CCAN) ----------------------------*/
/** @addtogroup LPC11xx_CCAN LPC11xx CCAN (CCAN) 
  @{
*/
typedef struct
{
  __IO	uint32_t CNTL;		// CAN control
  __IO	uint32_t STAT;		// Status register
  __I	uint32_t EC;		// Error counter
  __IO	uint32_t BT;		// Bit timing register
  __I	uint32_t INT;		// Interrupt register
  __IO	uint32_t TEST;		// Test register
  __IO	uint32_t BRPE;		// Baud rate prescaler extension register
	uint32_t RESERVED0[1];
  __IO	uint32_t IF1_CMDREQ;		// Message interface 1 command request
  __IO	uint32_t IF1_CMDMSK;		// Message interface 1 command mask
  __IO	uint32_t IF1_MSK1;		// Message interface 1 mask 1
  __IO	uint32_t IF1_MSK2;		// Message interface 1 mask 2
  __IO	uint32_t IF1_ARB1;		// Message interface 1 arbitration 1
  __IO	uint32_t IF1_ARB2;		// Message interface 1 arbitration 2
  __IO	uint32_t IF1_MCTRL;		// Message interface 1 message control
  __IO	uint32_t IF1_DA1;		// Message interface 1 data A1
  __IO	uint32_t IF1_DA2;		// Message interface 1 data A2
  __IO	uint32_t IF1_DB1;		// Message interface 1 data B1
  __IO	uint32_t IF1_DB2;		// Message interface 1 data B2
	uint32_t RESERVED1[13];
  __IO	uint32_t IF2_CMDREQ;		// Message interface 2 command request
  __IO	uint32_t IF2_CMDMSK;		// Message interface 2 command mask
  __IO	uint32_t IF2_MSK1;		// Message interface 2 mask 1
  __IO	uint32_t IF2_MSK2;		// Message interface 2 mask 2
  __IO	uint32_t IF2_ARB1;		// Message interface 2 arbitration 1
  __IO	uint32_t IF2_ARB2;		// Message interface 2 arbitration 2
  __IO	uint32_t IF2_MCTRL;		// Message interface 2 message control
  __IO	uint32_t IF2_DA1;		// Message interface 2 data A1
  __IO	uint32_t IF2_DA2;		// Message interface 2 data A2
  __IO	uint32_t IF2_DB1;		// Message interface 2 data B1
  __IO	uint32_t IF2_DB2;		// Message interface 2 data B2
	uint32_t RESERVED2[21];
  __I	uint32_t TXREQ1;		// Transmission request 1
  __I	uint32_t TXREQ2;		// Transmission request 2
	uint32_t RESERVED3[6];
  __I	uint32_t ND1;		// New data 1
  __I	uint32_t ND2;		// New data 2
	uint32_t RESERVED4[6];
  __I	uint32_t IR1;		// Interrupt pending 1
  __I	uint32_t IR2;		// Interrupt pending 2
	uint32_t RESERVED5[6];
  __I	uint32_t MSGV1;		// Message valid 1
  __I	uint32_t MSGV2;		// Message valid 2
	uint32_t RESERVED6[6];
  __IO	uint32_t CLKDIV;		// Can clock divider register
} LPC_CCAN_TypeDef;
/*@}*/ /* end of group LPC11xx_CCAN */

/*------------- 16-bit counter/timer 0 CT16B0 (CT16B0) ----------------------------*/
/** @addtogroup LPC11xx_CT16B0 LPC11xx 16-bit counter/timer 0 CT16B0 (CT16B0) 
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
  __IO	uint32_t MR0;		// Match Register 0
  __IO	uint32_t MR1;		// Match Register 1
  __IO	uint32_t MR2;		// Match Register 2
  __IO	uint32_t MR3;		// Match Register 3
  __IO	uint32_t CCR;		// Capture Control Register
  __I	uint32_t CR0;		// Capture Register 0
	uint32_t RESERVED0[3];
  __IO	uint32_t EMR;		// External Match Register
	uint32_t RESERVED1[12];
  __IO	uint32_t CTCR;		// Count Control Register
  __IO	uint32_t PWMC;		// PWM Control Register
} LPC_CT16B0_TypeDef;
/*@}*/ /* end of group LPC11xx_CT16B0 */

/*------------- 16-bit counter/timer 1 CT16B1 (CT16B1) ----------------------------*/
/** @addtogroup LPC11xx_CT16B1 LPC11xx 16-bit counter/timer 1 CT16B1 (CT16B1) 
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
  __IO	uint32_t MR0;		// Match Register 0
  __IO	uint32_t MR1;		// Match Register 1
  __IO	uint32_t MR2;		// Match Register 2
  __IO	uint32_t MR3;		// Match Register 3
  __IO	uint32_t CCR;		// Capture Control Register
  __I	uint32_t CR0;		// Capture Register 0
	uint32_t RESERVED0[3];
  __IO	uint32_t EMR;		// External Match Register
	uint32_t RESERVED1[12];
  __IO	uint32_t CTCR;		// Count Control Register
  __IO	uint32_t PWMC;		// PWM Control Register
} LPC_CT16B1_TypeDef;
/*@}*/ /* end of group LPC11xx_CT16B1 */

/*------------- 32-bit counter/timer 0 CT32B0 (CT32B0) ----------------------------*/
/** @addtogroup LPC11xx_CT32B0 LPC11xx 32-bit counter/timer 0 CT32B0 (CT32B0) 
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
  __IO	uint32_t MR0;		// Match Register 0
  __IO	uint32_t MR1;		// Match Register 1
  __IO	uint32_t MR2;		// Match Register 2
  __IO	uint32_t MR3;		// Match Register 3
  __IO	uint32_t CCR;		// Capture Control Register
  __I	uint32_t CR0;		// Capture Register 0
	uint32_t RESERVED0[3];
  __IO	uint32_t EMR;		// External Match Register
	uint32_t RESERVED1[12];
  __IO	uint32_t CTCR;		// Count Control Register
  __IO	uint32_t PWMC;		// PWM Control Register
} LPC_CT32B0_TypeDef;
/*@}*/ /* end of group LPC11xx_CT32B0 */

/*------------- 32-bit counter/timer 1 CT32B1 (CT32B1) ----------------------------*/
/** @addtogroup LPC11xx_CT32B1 LPC11xx 32-bit counter/timer 1 CT32B1 (CT32B1) 
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
  __IO	uint32_t MR0;		// Match Register 0
  __IO	uint32_t MR1;		// Match Register 1
  __IO	uint32_t MR2;		// Match Register 2
  __IO	uint32_t MR3;		// Match Register 3
  __IO	uint32_t CCR;		// Capture Control Register
  __I	uint32_t CR0;		// Capture Register 0
	uint32_t RESERVED0[3];
  __IO	uint32_t EMR;		// External Match Register
	uint32_t RESERVED1[12];
  __IO	uint32_t CTCR;		// Count Control Register
  __IO	uint32_t PWMC;		// PWM Control Register
} LPC_CT32B1_TypeDef;
/*@}*/ /* end of group LPC11xx_CT32B1 */

/*------------- Watchdog timer (WDT) ----------------------------*/
/** @addtogroup LPC11xx_WDT LPC11xx Watchdog timer (WDT) 
  @{
*/
typedef struct
{
  __IO	uint32_t MOD;		// Watchdog mode register
  __IO	uint32_t TC;		// Watchdog timer constant register
  __O	uint32_t FEED;		// Watchdog feed sequence register
  __I	uint32_t TV;		// Watchdog timer value register
	uint32_t RESERVED0[1];
  __IO	uint32_t WARNINT;		// Watchdog Warning Interrupt compare value
  __IO	uint32_t WINDOW;		// Watchdog Window compare value
} LPC_WDT_TypeDef;
/*@}*/ /* end of group LPC11xx_WDT */

/*------------- SysTick timer (SYSTICK) ----------------------------*/
/** @addtogroup LPC11xx_SYSTICK LPC11xx SysTick timer (SYSTICK) 
  @{
*/
typedef struct
{
	uint32_t RESERVED0[4];
  __IO	uint32_t CSR;		// System Timer Control and status register
  __IO	uint32_t RVR;		// System Timer Reload value register
  __IO	uint32_t CVR;		// System Timer Current value register
  __IO	uint32_t CALIB;		// System Timer Calibration value register
} LPC_SYSTICK_TypeDef;
/*@}*/ /* end of group LPC11xx_SYSTICK */

/*------------- ADC (ADC) ----------------------------*/
/** @addtogroup LPC11xx_ADC LPC11xx ADC (ADC) 
  @{
*/
typedef struct
{
  __IO	uint32_t CR;		// A/D Control Register
  __IO	uint32_t GDR;		// A/D Global Data Register
	uint32_t RESERVED0[1];
  __IO	uint32_t INTEN;		// A/D Interrupt Enable Register
  __IO	uint32_t DR[8];		// A/D Channel Data Registers
  __I	uint32_t STAT;		// A/D Status Register
} LPC_ADC_TypeDef;
/*@}*/ /* end of group LPC11xx_ADC */

/*------------- FMC (FMC) ----------------------------*/
/** @addtogroup LPC11xx_FMC LPC11xx FMC (FMC) 
  @{
*/
typedef struct
{
	uint32_t RESERVED0[8];
  __IO	uint32_t START;		// Signature start address register
  __IO	uint32_t STOP;		// Signature stop-address register
	uint32_t RESERVED1[1];
  __I	uint32_t W0;		// Word 0
  __I	uint32_t W1;		// Word 1
  __I	uint32_t W2;		// Word 2
  __I	uint32_t W3;		// Word 3
	uint32_t RESERVED2[1001];
  __I	uint32_t TAT;		// Signature generation status register
	uint32_t RESERVED3[1];
  __O	uint32_t TATCLR;		// Signature generation status clear
} LPC_FMC_TypeDef;
/*@}*/ /* end of group LPC11xx_FMC */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/*@}*/ /* end of group LPC11xx_Peripherals */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup LPC11xx_MemoryMap LPC11xx Memory Mapping
  @{
*/

/* Flash and RAM Base Addresses */
#define LPC_FLASH_BASE      (0x00000000UL)            /*!< FLASH Base Address */
#define LPC_RAM_BASE        (0x10000000UL)              /*!< RAM Base Address */

/*@}*/ /* end of group LPC11xx_MemoryMap */


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
/** @addtogroup LPC11xx_PeripheralDecl LPC11xx Peripheral Declaration
  @{
*/

#define LPC_SYSCON	((LPC_SYSCON_TypeDef*	) 0x40048000)
#define LPC_PMU		((LPC_PMU_TypeDef*	) 0x40038000)
#define LPC_IOCON	((LPC_IOCON_TypeDef*	) 0x40044000)
#define LPC_GPIO0	((LPC_GPIO_TypeDef*	) 0x50000000)
#define LPC_GPIO1	((LPC_GPIO_TypeDef*	) 0x50010000)
#define LPC_GPIO2	((LPC_GPIO_TypeDef*	) 0x50020000)
#define LPC_GPIO3	((LPC_GPIO_TypeDef*	) 0x50030000)
#define LPC_UART	((LPC_UART_TypeDef*	) 0x40008000)
#define LPC_SPI0	((LPC_SPI0_TypeDef*	) 0x40040000)
#define LPC_SPI1	((LPC_SPI1_TypeDef*	) 0x40058000)
#define LPC_I2C		((LPC_I2C_TypeDef*	) 0x40000000)
#define LPC_CCAN	((LPC_CCAN_TypeDef*	) 0x40050000)
#define LPC_CT16B0	((LPC_CT16B0_TypeDef*	) 0x4000C000)
#define LPC_CT16B1	((LPC_CT16B1_TypeDef*	) 0x40010000)
#define LPC_CT32B0	((LPC_CT32B0_TypeDef*	) 0x40014000)
#define LPC_CT32B1	((LPC_CT32B1_TypeDef*	) 0x40018000)
#define LPC_WDT		((LPC_WDT_TypeDef*	) 0x40004000)
#define LPC_SYSTICK	((LPC_SYSTCIK_TypeDef*	) 0xE000E000)
#define LPC_ADC		((LPC_ADC_TypeDef*	) 0x4001C000)
#define LPC_FMC		((LPC_FMC_TypeDef*	) 0x4003C000)

/*@}*/ /* end of group LPC11xx_PeripheralDecl */

/*@}*/ /* end of group LPC11xx_Definitions */

#ifdef __cplusplus
}
#endif

#endif  /* LPC11xx_H */
