/**************************************************************************//**
 * @file     LPC11Uxx.h
 * @brief    CMSIS Cortex-M0 Core Peripheral Access Layer Header File for
 *           Device LPC11Uxx
 * @version  V3.10
 * @date     23. November 2012
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2012 ARM LIMITED
   Copyright (c) 2013 Richard Meadows - Added code for LPC11Uxx

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


#ifndef LPC11Uxx_H
#define LPC11Uxx_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup LPC11Uxx_Definitions LPC11Uxx Definitions
  This file defines all structures and symbols for LPC11Uxx:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/


/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup LPC11Uxx_CMSIS Device CMSIS Definitions
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

  PIN_INT0_IRQn                 = 0,        /*!< GPIO pin                             */
  PIN_INT1_IRQn                 = 1,        /*!< GPIO pin                             */
  PIN_INT2_IRQn                 = 2,        /*!< GPIO pin                             */
  PIN_INT3_IRQn                 = 3,        /*!< GPIO pin                             */
  PIN_INT4_IRQn                 = 4,        /*!< GPIO pin                             */
  PIN_INT5_IRQn                 = 5,        /*!< GPIO pin                             */
  PIN_INT6_IRQn                 = 6,        /*!< GPIO pin                             */
  PIN_INT7_IRQn                 = 7,        /*!< GPIO pin                             */
  GINT0_IRQn                    = 8,        /*!< GPIO GROUP0                          */
  GINT1_IRQn                    = 9,        /*!< GPIO GROUP1                          */
  SSP1_IRQn                     = 14,       /*!< SSP1                                 */
  I2C_IRQn                      = 15,       /*!< I2C                                  */
  TIMER_16_0_IRQn               = 16,       /*!< 16-bit 0                             */
  TIMER_16_1_IRQn               = 17,       /*!< 16-bit 1                             */
  TIMER_32_0_IRQn               = 18,       /*!< 32-bit 0                             */
  TIMER_32_1_IRQn               = 19,       /*!< 32-bit 1                             */
  SSP0_IRQn                     = 20,       /*!< SSP0                                 */
  UART_IRQn                     = 21,       /*!< UART                                 */
  USB_IRQ_IRQn                  = 22,       /*!< USB                                  */
  USB_FIQ_IRQn                  = 23,       /*!< USB                                  */
  ADC_IRQn                      = 24,       /*!< A/D Converter                        */
  WDT_IRQn                      = 25,       /*!< Watchdog                             */
  BOD_IRQn                      = 26,       /*!< Brown Out                            */
  FLASH_IRQn                    = 27,       /*!< Flash/EEPROM interface               */
  USB_WAKEUP_IRQn               = 30,       /*!< USB                                  */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __CM0_REV                 0x0201    /*!< Core Revision r2p1                   */
#define __NVIC_PRIO_BITS          3         /*!< Number of Bits for Priority Levels   */
#define __Vendor_SysTickConfig    0         /*!< Set if different SysTick Config      */
#define __MPU_PRESENT             0         /*!< MPU present or not                   */

/*@}*/ /* end of group LPC11Uxx_CMSIS */

#include <core_cm0.h>                      /* Cortex-M0 processor and core peripherals */
#include "system_LPC11Uxx.h"                /* LPC11Uxx System include file              */

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup LPC11Uxx_Peripherals LPC11Uxx Peripherals
  LPC11Uxx Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/*------------- system control block (SYSCON) ----------------------------*/
/** @addtogroup LPC11Uxx_SYSCON LPC11Uxx system control block (SYSCON) 
  @{
*/
typedef struct
{
  __IO	uint32_t SYSMEMREMAP;		// System memory remap
  __IO	uint32_t PRESETCTRL;		// Peripheral reset control
  __IO	uint32_t SYSPLLCTRL;		// System PLL control
  __I	uint32_t SYSPLLSTAT;		// System PLL status
  __IO	uint32_t USBPLLCTRL;		// USB PLL control
  __I	uint32_t USBPLLSTAT;		// USB PLL status
	uint32_t RESERVED0[2];
  __IO	uint32_t SYSOSCCTRL;		// System oscillator control
  __IO	uint32_t WDTOSCCTRL;		// Watchdog oscillator control
	uint32_t RESERVED1[2];
  __IO	uint32_t SYSRSTSTAT;		// System reset status register
	uint32_t RESERVED2[3];
  __IO	uint32_t SYSPLLCLKSEL;		// System PLL clock source select
  __IO	uint32_t SYSPLLCLKUEN;		// System PLL clock source update
  __IO	uint32_t USBPLLCLKSEL;		// USB PLL clock source select
  __IO	uint32_t USBPLLCLKUEN;		// USB PLL clock source update enable 0
	uint32_t RESERVED3[8];
  __IO	uint32_t MAINCLKSEL;		// Main clock source select
  __IO	uint32_t MAINCLKUEN;		// Main clock source update enable
  __IO	uint32_t SYSAHBCLKDIV;		// System clock divider
	uint32_t RESERVED4[1];
  __IO	uint32_t SYSAHBCLKCTRL;		// System clock control
	uint32_t RESERVED5[4];
  __IO	uint32_t SSP0CLKDIV;		// SSP0 clock divider
  __IO	uint32_t UARTCLKDIV;		// UART clock divider
  __IO	uint32_t SSP1CLKDIV;		// SSP1 clock divider
	uint32_t RESERVED6[8];
  __IO	uint32_t USBCLKSEL;		// USB clock source select
  __IO	uint32_t USBCLKUEN;		// USB clock source update enable
  __IO	uint32_t USBCLKDIV;		// USB clock source divider
	uint32_t RESERVED7[5];
  __IO	uint32_t CLKOUTSEL;		// CLKOUT clock source select
  __IO	uint32_t CLKOUTUEN;		// CLKOUT clock source update enable 0
  __IO	uint32_t CLKOUTDIV;		// CLKOUT clock divider
	uint32_t RESERVED8[5];
  __I	uint32_t PIOPORCAP0;		// POR captured PIO status 0
  __I	uint32_t PIOPORCAP1;		// POR captured PIO status 1
	uint32_t RESERVED9[18];
  __IO	uint32_t BODCTRL;		// Brown-Out Detect
  __IO	uint32_t SYSTCKCAL;		// System tick counter calibration
	uint32_t RESERVED10[6];
  __IO	uint32_t IRQLATENCY;		// IQR delay
  __IO	uint32_t NMISRC;		// NMI Source Control
  __IO	uint32_t PINTSEL0;		// GPIO Pin Interrupt Select register 0
  __IO	uint32_t PINTSEL1;		// GPIO Pin Interrupt Select register 1
  __IO	uint32_t PINTSEL2;		// GPIO Pin Interrupt Select register 2
  __IO	uint32_t PINTSEL3;		// GPIO Pin Interrupt Select register 3
  __IO	uint32_t PINTSEL4;		// GPIO Pin Interrupt Select register 4
  __IO	uint32_t PINTSEL5;		// GPIO Pin Interrupt Select register 5
  __IO	uint32_t PINTSEL6;		// GPIO Pin Interrupt Select register 6
  __IO	uint32_t PINTSEL7;		// GPIO Pin Interrupt Select register 7
  __IO	uint32_t USBCLKCTRL;		// USB clock control
  __I	uint32_t USBCLKST;		// USB clock status
	uint32_t RESERVED11[25];
  __IO	uint32_t STARTERP0;		// Start logic 0 interrupt wake-up enable 0
	uint32_t RESERVED12[3];
  __IO	uint32_t STARTERP1;		// Start logic 1 interrupt wake-up enable 0
	uint32_t RESERVED13[6];
  __IO	uint32_t PDSLEEPCFG;		// Power-down states in deep-sleep
  __IO	uint32_t PDAWAKECFG;		// Power-down states for wake-up from 0xEDF0
  __IO	uint32_t PDRUNCFG;		// Power configuration register
	uint32_t RESERVED14[110];
  __I	uint32_t DEVICE_ID;		// Device ID
} LPC_SYSCON_TypeDef;
/*@}*/ /* end of group LPC11Uxx_SYSCON */

/*------------- PMU (PMU) ----------------------------*/
/** @addtogroup LPC11Uxx_PMU LPC11Uxx PMU (PMU) 
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
/*@}*/ /* end of group LPC11Uxx_PMU */

/*------------- USB (USB) ----------------------------*/
/** @addtogroup LPC11Uxx_USB LPC11Uxx USB (USB) 
  @{
*/
typedef struct
{
  __IO	uint32_t DEVCMDSTAT;		// USB Device Command/Status
  __IO	uint32_t INFO;		// USB Info register
  __IO	uint32_t EPLISTSTART;		// USB EP Command/Status List
  __IO	uint32_t DATABUFSTART;		// USB Data buffer start address
  __IO	uint32_t LPM;		// USB Link Power Management
  __IO	uint32_t EPSKIP;		// USB Endpoint skip
  __IO	uint32_t EPINUSE;		// USB Endpoint Buffer in use
  __IO	uint32_t EPBUFCFG;		// USB Endpoint Buffer
  __IO	uint32_t INTSTAT;		// USB interrupt status register
  __IO	uint32_t INTEN;		// USB interrupt enable register
  __IO	uint32_t INTSETSTAT;		// USB set interrupt status register  
  __IO	uint32_t INTROUTING;		// USB interrupt routing register 
	uint32_t RESERVED14[110];
  __IO	uint32_t EPTOGGLE;		// USB Endpoint toggle register 
} LPC_USB_TypeDef;
/*@}*/ /* end of group LPC11Uxx_USB */

/*------------- I/O configuration (IOCON) ----------------------------*/
/** @addtogroup LPC11Uxx_IOCON LPC11Uxx I/O configuration (IOCON) 
  @{
*/
typedef struct
{
  __IO	uint32_t RESET_PIO0_0;		// I/O configuration for pin RESET/PIO0_0 0x0000 0090
  __IO	uint32_t PIO0_1;		// I/O configuration for pin
  __IO	uint32_t PIO0_2;		// I/O configuration for pin
  __IO	uint32_t PIO0_3;		// I/O configuration for pin
  __IO	uint32_t PIO0_4;		// I/O configuration for pin PIO0_4/SCL
  __IO	uint32_t PIO0_5;		// I/O configuration for pin PIO0_5/SDA
  __IO	uint32_t PIO0_6;		// I/O configuration for pin
  __IO	uint32_t PIO0_7;		// I/O configuration for pin PIO0_7/CTS
  __IO	uint32_t PIO0_8;		// I/O configuration for pin
  __IO	uint32_t PIO0_9;		// I/O configuration for pin
  __IO	uint32_t SWCLK_PIO0_10;		// I/O configuration for pin
  __IO	uint32_t TDI_PIO0_11;		// I/O configuration for pin
  __IO	uint32_t TMS_PIO0_12;		// I/O configuration for pin
  __IO	uint32_t TDO_PIO0_13;		// I/O configuration for pin
  __IO	uint32_t TRST_PIO0_14;		// I/O configuration for pin
  __IO	uint32_t SWDIO_PIO0_15;		// I/O configuration for pin
  __IO	uint32_t PIO0_16;		// I/O configuration for pin
  __IO	uint32_t PIO0_17;		// I/O configuration for pin
  __IO	uint32_t PIO0_18;		// I/O configuration for pin
  __IO	uint32_t PIO0_19;		// I/O configuration for pin
  __IO	uint32_t PIO0_20;		// I/O configuration for pin
  __IO	uint32_t PIO0_21;		// I/O configuration for pin
  __IO	uint32_t PIO0_22;		// I/O configuration for pin
  __IO	uint32_t PIO0_23;		// I/O configuration for pin PIO0_23/AD7
  __IO	uint32_t PIO1_0;		// I/O configuration for pin
  __IO	uint32_t PIO1_1;		// I/O configuration for pin
  __IO	uint32_t PIO1_2;		// I/O configuration for pin
  __IO	uint32_t PIO1_3;		// I/O configuration for pin
  __IO	uint32_t PIO1_4;		// I/O configuration for pin
  __IO	uint32_t PIO1_5;		// I/O configuration for pin
  __IO	uint32_t PIO1_6;		// I/O configuration for pin PIO1_6
  __IO	uint32_t PIO1_7;		// I/O configuration for pin PIO1_7
  __IO	uint32_t PIO1_8;		// I/O configuration for pin PIO1_8
  __IO	uint32_t PIO1_9;		// I/O configuration for pin PIO1_9
  __IO	uint32_t PIO1_10;		// I/O configuration for pin PIO1_10
  __IO	uint32_t PIO1_11;		// I/O configuration for pin PIO1_11
  __IO	uint32_t PIO1_12;		// I/O configuration for pin PIO1_12
  __IO	uint32_t PIO1_13;		// I/O configuration for pin
  __IO	uint32_t PIO1_14;		// I/O configuration for pin
  __IO	uint32_t PIO1_15;		// I/O configuration for pin PIO1_15/DCD/
  __IO	uint32_t PIO1_16;		// I/O configuration for pin
  __IO	uint32_t PIO1_17;		// I/O configuration for
  __IO	uint32_t PIO1_18;		// I/O configuration for
  __IO	uint32_t PIO1_19;		// I/O configuration for pin
  __IO	uint32_t PIO1_20;		// I/O configuration for pin
  __IO	uint32_t PIO1_21;		// I/O configuration for pin
  __IO	uint32_t PIO1_22;		// I/O configuration for pin
  __IO	uint32_t PIO1_23;		// I/O configuration for pin
  __IO	uint32_t PIO1_24;		// I/O configuration for pin PIO1_24/
  __IO	uint32_t PIO1_25;		// I/O configuration for pin
  __IO	uint32_t PIO1_26;		// I/O configuration for pin
  __IO	uint32_t PIO1_27;		// I/O configuration for pin
  __IO	uint32_t PIO1_28;		// I/O configuration for pin
  __IO	uint32_t PIO1_29;		// I/O configuration for pin PIO1_29/SCK0/ 0x0000 0090
	uint32_t RESERVED0[1];
  __IO	uint32_t PIO1_31;		// I/O configuration for pin PIO1_31
} LPC_IOCON_TypeDef;
/*@}*/ /* end of group LPC11Uxx_IOCON */

/*------------- GPIO pin interrupts (GPIO_PIN_INT) ----------------------------*/
/** @addtogroup LPC11Uxx_GPIO_PIN_INT LPC11Uxx GPIO pin interrupts (GPIO_PIN_INT) 
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
/*@}*/ /* end of group LPC11Uxx_GPIO_PIN_INT */

/*------------- GPIO GROUP interrupt (GPIO_GROUP_INT) ----------------------------*/
/** @addtogroup LPC11Uxx_GPIO_GROUP_INT LPC11Uxx GPIO GROUP interrupt (GPIO_GROUP_INT) 
  @{
*/
typedef struct
{
  __IO	uint32_t CTRL;		// GPIO grouped interrupt control
	uint32_t RESERVED0[7];
  __IO	uint32_t POL0;		// GPIO grouped interrupt port 0 polarity 0xFFFF
  __IO	uint32_t POL1;		// GPIO grouped interrupt port 1 polarity 0xFFFF
	uint32_t RESERVED1[6];
  __IO	uint32_t ENA0;		// GPIO grouped interrupt port 0 enable 0
  __IO	uint32_t ENA1;		// GPIO grouped interrupt port 1 enable 0
} LPC_GPIO_GROUP_INT_TypeDef;
/*@}*/ /* end of group LPC11Uxx_GPIO_GROUP_INT */

/*------------- GPIO port (GPIO) ----------------------------*/
/** @addtogroup LPC11Uxx_GPIO LPC11Uxx GPIO port (GPIO) 
  @{
*/
typedef struct
{
  union {
    struct {
      __IO uint8_t B0[32];		// Byte pin registers port 0
      __IO uint8_t B1[32];		// Byte pin registers port 1
    };
    __IO uint8_t B[64];
  };
	uint32_t RESERVED1[1008];
  union {
    struct {
      __IO uint32_t W0[32];		// Word pin registers port 0
      __IO uint32_t W1[32];		// Word pin registers port 1
    };
    __IO uint32_t W[64];
  };
	uint32_t RESERVED3[960];
  __IO	uint32_t DIR0;		// Direction registers port 0
  __IO	uint32_t DIR1;		// Direction registers port 1
	uint32_t RESERVED4[30];
  __IO	uint32_t MASK0;		// Mask register port 0
  __IO	uint32_t MASK1;		// Mask register port 1
	uint32_t RESERVED5[30];
  __IO	uint32_t PIN0;		// Port pin register port 0
  __IO	uint32_t PIN1;		// Port pin register port 1
	uint32_t RESERVED6[30];
  __IO	uint32_t MPIN0;		// Masked port register port 0
  __IO	uint32_t MPIN1;		// Masked port register port 1
	uint32_t RESERVED7[30];
  __IO	uint32_t SET0;		// Write
  __IO	uint32_t SET1;		// Write
	uint32_t RESERVED8[30];
  __O	uint32_t CLR0;		// Clear port 0
  __O	uint32_t CLR1;		// Clear port 1
	uint32_t RESERVED9[30];
  __O	uint32_t NOT0;		// Toggle port 0
  __O	uint32_t NOT1;		// Toggle port 1
} LPC_GPIO_TypeDef;
/*@}*/ /* end of group LPC11Uxx_GPIO */

/*------------- UART (UART) ----------------------------*/
/** @addtogroup LPC11Uxx_UART LPC11Uxx UART (UART) 
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
  __IO	uint32_t MCR;		// Modem Control Register
  __I	uint32_t LSR;		// Line Status Register
  __I	uint32_t MSR;		// Modem Status Register
  __IO	uint32_t SCR;		// Scratch Pad Register
  __IO	uint32_t ACR;		// Auto-baud Control Register
  __IO	uint32_t ICR;		// IrDA Control Register
  __IO	uint32_t FDR;		// Fractional Divider Register
  __IO	uint32_t OSR;		// Oversampling Register
  __IO	uint32_t TER;		// Transmit Enable Register
	uint32_t RESERVED0[3];
  __IO	uint32_t HDEN;		// Half duplex enable register
	uint32_t RESERVED1[1];
  __IO	uint32_t SCICTRL;		// Smart Card Interface Control register
  __IO	uint32_t RS485CTRL;		// RS-485/EIA-485 Control
  __IO	uint32_t RS485ADRMATCH;		// RS-485/EIA-485 address match
  __IO	uint32_t RS485DLY;		// RS-485/EIA-485 direction control delay
  __IO	uint32_t SYNCCTRL;		// Synchronous mode control register
} LPC_UART_TypeDef;
/*@}*/ /* end of group LPC11Uxx_UART */

/*------------- SPI0 (SPI0) ----------------------------*/
/** @addtogroup LPC11Uxx_SPI0 LPC11Uxx SPI0 (SPI0) 
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
/*@}*/ /* end of group LPC11Uxx_SPI0 */

/*------------- SPI1 (SPI1) ----------------------------*/
/** @addtogroup LPC11Uxx_SPI1 LPC11Uxx SPI1 (SPI1) 
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
/*@}*/ /* end of group LPC11Uxx_SPI1 */

/*------------- I2C (I2C) ----------------------------*/
/** @addtogroup LPC11Uxx_I2C LPC11Uxx I2C (I2C) 
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
/*@}*/ /* end of group LPC11Uxx_I2C */

/*------------- 16-bit counter/timer 0 CT16B0 (CT16B0) ----------------------------*/
/** @addtogroup LPC11Uxx_CT16B0 LPC11Uxx 16-bit counter/timer 0 CT16B0 (CT16B0) 
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
    __IO uint32_t MR[4];
  };
  __IO	uint32_t CCR;		// Capture Control Register
  __I	uint32_t CR0;		// Capture Register 0
        uint32_t RESERVED0[1];
  __I	uint32_t CR1;		// Capture Register 1
	uint32_t RESERVED1[1];
  __IO	uint32_t EMR;		// External Match Register
	uint32_t RESERVED2[12];
  __IO	uint32_t CTCR;		// Count Control Register
  __IO	uint32_t PWMC;		// PWM Control Register
} LPC_CT16B0_TypeDef;
/*@}*/ /* end of group LPC11Uxx_CT16B0 */

/*------------- 16-bit counter/timer 1 CT16B1 (CT16B1) ----------------------------*/
/** @addtogroup LPC11Uxx_CT16B1 LPC11Uxx 16-bit counter/timer 1 CT16B1 (CT16B1) 
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
    __IO uint32_t MR[4];
  };
  __IO	uint32_t CCR;		// Capture Control Register
  __I	uint32_t CR0;		// Capture Register 0
  __I	uint32_t CR1;		// Capture Register 1
	uint32_t RESERVED0[2];
  __IO	uint32_t EMR;		// External Match Register
	uint32_t RESERVED1[12];
  __IO	uint32_t CTCR;		// Count Control Register
  __IO	uint32_t PWMC;		// PWM Control Register
} LPC_CT16B1_TypeDef;
/*@}*/ /* end of group LPC11Uxx_CT16B1 */

/*------------- 32-bit counter/timer 0 CT32B0 (CT32B0) ----------------------------*/
/** @addtogroup LPC11Uxx_CT32B0 LPC11Uxx 32-bit counter/timer 0 CT32B0 (CT32B0) 
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
    __IO uint32_t MR[4];
  };
  __IO	uint32_t CCR;		// Capture Control Register
  __I	uint32_t CR0;		// Capture Register 0
        uint32_t RESERVED0[1];
  __I	uint32_t CR1;		// Capture Register 1
	uint32_t RESERVED1[1];
  __IO	uint32_t EMR;		// External Match Register
	uint32_t RESERVED2[12];
  __IO	uint32_t CTCR;		// Count Control Register
  __IO	uint32_t PWMC;		// PWM Control Register
} LPC_CT32B0_TypeDef;
/*@}*/ /* end of group LPC11Uxx_CT32B0 */

/*------------- 32-bit counter/timer 1 CT32B1 (CT32B1) ----------------------------*/
/** @addtogroup LPC11Uxx_CT32B1 LPC11Uxx 32-bit counter/timer 1 CT32B1 (CT32B1) 
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
    __IO uint32_t MR[4];
  };
  __IO	uint32_t CCR;		// Capture Control Register
  __I	uint32_t CR0;		// Capture Register 0
  __I	uint32_t CR1;		// Capture Register 1
	uint32_t RESERVED0[2];
  __IO	uint32_t EMR;		// External Match Register
	uint32_t RESERVED1[12];
  __IO	uint32_t CTCR;		// Count Control Register
  __IO	uint32_t PWMC;		// PWM Control Register
} LPC_CT32B1_TypeDef;
/*@}*/ /* end of group LPC11Uxx_CT32B1 */

/*------------- Watchdog timer (WDT) ----------------------------*/
/** @addtogroup LPC11Uxx_WDT LPC11Uxx Watchdog timer (WDT) 
  @{
*/
typedef struct
{
  __IO	uint32_t MOD;		// Watchdog mode register
  __IO	uint32_t TC;		// Watchdog timer constant register
  __O	uint32_t FEED;		// Watchdog feed sequence register
  __I	uint32_t TV;		// Watchdog timer value register
  __IO	uint32_t CLKSEL;		// Watchdog clock select register
  __IO	uint32_t WARNINT;		// Watchdog Warning Interrupt compare value
  __IO	uint32_t WINDOW;		// Watchdog Window compare value
} LPC_WDT_TypeDef;
/*@}*/ /* end of group LPC11Uxx_WDT */

/*------------- SysTick timer (SYSTICK) ----------------------------*/
/** @addtogroup LPC11Uxx_SYSTICK LPC11Uxx SysTick timer (SYSTICK) 
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
/*@}*/ /* end of group LPC11Uxx_SYSTICK */

/*------------- ADC (ADC) ----------------------------*/
/** @addtogroup LPC11Uxx_ADC LPC11Uxx ADC (ADC) 
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
/*@}*/ /* end of group LPC11Uxx_ADC */

/*------------- FMC (FMC) ----------------------------*/
/** @addtogroup LPC11Uxx_FMC LPC11Uxx FMC (FMC) 
  @{
*/
typedef struct
{
	uint32_t RESERVED0[4];
  __IO	uint32_t FLASHCFG;		// Flash memory access time
	uint32_t RESERVED1[3];
  __IO	uint32_t FMSSTART;		// Signature start address register
  __IO	uint32_t FMSSTOP;		// Signature stop-address register
	uint32_t RESERVED2[1];
  __I	uint32_t FMSW0;		// Word 0
  __I	uint32_t FMSW1;		// Word 1
  __I	uint32_t FMSW2;		// Word 2
  __I	uint32_t FMSW3;		// Word 3
	uint32_t RESERVED3[24];
  __IO	uint32_t EEMSSTART;		// EEPROM BIST start address register
  __IO	uint32_t EEMSSTOP;		// EEPROM BIST stop address register
  __I	uint32_t EEMSSIG;		// EEPROM 24-bit BIST signature register 0x0
	uint32_t RESERVED4[974];
  __I	uint32_t FMSTAT;		// Signature generation status register
	uint32_t RESERVED5[1];
  __O	uint32_t FMSTATCLR;		// Signature generation status clear
} LPC_FMC_TypeDef;
/*@}*/ /* end of group LPC11Uxx_FMC */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/*@}*/ /* end of group LPC11Uxx_Peripherals */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup LPC11Uxx_MemoryMap LPC11Uxx Memory Mapping
  @{
*/

/* Flash and RAM Base Addresses */
#define LPC_FLASH_BASE      (0x00000000UL)            /*!< FLASH Base Address */
#define LPC_RAM_BASE        (0x10000000UL)              /*!< RAM Base Address */

/*@}*/ /* end of group LPC11Uxx_MemoryMap */


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
/** @addtogroup LPC11Uxx_PeripheralDecl LPC11Uxx Peripheral Declaration
  @{
*/

#define LPC_SYSCON		((LPC_SYSCON_TypeDef*	) 0x40048000)
#define LPC_PMU			((LPC_PMU_TypeDef*	) 0x40038000)
#define LPC_IOCON		((LPC_IOCON_TypeDef*	) 0x40044000)
#define LPC_GPIO		((LPC_GPIO_TypeDef*	) 0x50000000)
#define LPC_GPIO_PIN_INT	((LPC_GPIO_PIN_INT_TypeDef*	) 0x4004C000)
#define LPC_GPIO_GROUP_INT0	((LPC_GPIO_GROUP_INT_TypeDef*	) 0x4005C000)
#define LPC_GPIO_GROUP_INT1	((LPC_GPIO_GROUP_INT_TypeDef*	) 0x40060000)
#define LPC_USB			((LPC_USB_TypeDef*	) 0x40080000)
#define LPC_UART		((LPC_UART_TypeDef*	) 0x40008000)
#define LPC_SPI0		((LPC_SPI0_TypeDef*	) 0x40040000)
#define LPC_SPI1		((LPC_SPI1_TypeDef*	) 0x40058000)
#define LPC_I2C			((LPC_I2C_TypeDef*	) 0x40000000)
#define LPC_CT16B0		((LPC_CT16B0_TypeDef*	) 0x4000C000)
#define LPC_CT16B1		((LPC_CT16B1_TypeDef*	) 0x40010000)
#define LPC_CT32B0		((LPC_CT32B0_TypeDef*	) 0x40014000)
#define LPC_CT32B1		((LPC_CT32B1_TypeDef*	) 0x40018000)
#define LPC_WDT			((LPC_WDT_TypeDef*	) 0x40004000)
#define LPC_SYSTICK		((LPC_SYSTCIK_TypeDef*	) 0xE000E000)
#define LPC_ADC			((LPC_ADC_TypeDef*	) 0x4001C000)
#define LPC_FMC			((LPC_FMC_TypeDef*	) 0x4003C000)

/*@}*/ /* end of group LPC11Uxx_PeripheralDecl */

/*@}*/ /* end of group LPC11Uxx_Definitions */

#ifdef __cplusplus
}
#endif

#endif  /* LPC11Uxx_H */
