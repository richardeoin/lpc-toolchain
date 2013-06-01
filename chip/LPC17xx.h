/**************************************************************************//**
 * @file     LPC17xx.h
 * @brief    CMSIS Cortex-M3 Core Peripheral Access Layer Header File for
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


#ifndef LPC17xx_H
#define LPC17xx_H

#ifdef __cplusplus
 extern "C" {
#endif


/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup LPC17xx_CMSIS Device CMSIS Definitions
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
/******  Cortex-M3 Processor Exceptions Numbers *******************************************/

  NonMaskableInt_IRQn           = -14,      /*!<  2 Non Maskable Interrupt      */
  MemoryManagement_IRQn         = -12,      /*!<  4 Memory Management Interrupt */
  BusFault_IRQn                 = -11,      /*!<  5 Bus Fault Interrupt         */
  UsageFault_IRQn               = -10,      /*!<  6 Usage Fault Interrupt       */
  SVCall_IRQn                   = -5,       /*!< 11 SV Call Interrupt           */
  DebugMonitor_IRQn             = -4,       /*!< 12 Debug Monitor Interrupt     */
  PendSV_IRQn                   = -2,       /*!< 14 Pend SV Interrupt           */
  SysTick_IRQn                  = -1,       /*!< 15 System Tick Interrupt       */

/******  Device Specific Interrupt Numbers ************************************************/
  WDT_IRQn                      = 0,        /*!< Watchdog Timer Interrupt       */
  TIMER0_IRQn                   = 1,        /*!< Timer0 Interrupt               */
  TIMER1_IRQn                   = 2,        /*!< Timer1 Interrupt               */
  TIMER2_IRQn                   = 3,        /*!< Timer2 Interrupt               */
  TIMER3_IRQn                   = 4,        /*!< Timer3 Interrupt               */
  UART0_IRQn                    = 5,        /*!< UART0 Interrupt                */
  UART1_IRQn                    = 6,        /*!< UART1 Interrupt                */
  UART2_IRQn                    = 7,        /*!< UART2 Interrupt                */
  UART3_IRQn                    = 8,        /*!< UART3 Interrupt                */
  PWM1_IRQn                     = 9,        /*!< PWM1 Interrupt                 */
  I2C0_IRQn                     = 10,       /*!< I2C0 Interrupt                 */
  I2C1_IRQn                     = 11,       /*!< I2C1 Interrupt                 */
  I2C2_IRQn                     = 12,       /*!< I2C2 Interrupt                 */
  SPI_IRQn                      = 13,       /*!< SPI Interrupt                  */
  SSP0_IRQn                     = 14,       /*!< SSP0 Interrupt                 */
  SSP1_IRQn                     = 15,       /*!< SSP1 Interrupt                 */
  PLL0_IRQn                     = 16,       /*!< PLL0 Lock (Main PLL) Interrupt */
  RTC_IRQn                      = 17,       /*!< Real Time Clock Interrupt      */
  EINT0_IRQn                    = 18,       /*!< External Interrupt 0 Interrupt */
  EINT1_IRQn                    = 19,       /*!< External Interrupt 1 Interrupt */
  EINT2_IRQn                    = 20,       /*!< External Interrupt 2 Interrupt */
  EINT3_IRQn                    = 21,       /*!< External Interrupt 3 Interrupt */
  ADC_IRQn                      = 22,       /*!< A/D Converter Interrupt        */
  BOD_IRQn                      = 23,       /*!< Brown-Out Detect Interrupt     */
  USB_IRQn                      = 24,       /*!< USB Interrupt                  */
  CAN_IRQn                      = 25,       /*!< CAN Interrupt                  */
  DMA_IRQn                      = 26,       /*!< General Purpose DMA Interrupt  */
  I2S_IRQn                      = 27,       /*!< I2S Interrupt                  */
  ENET_IRQn                     = 28,       /*!< Ethernet Interrupt             */
  RIT_IRQn                      = 29,       /*!< Repetitive Interrupt Timer     */
  MCPWM_IRQn                    = 30,       /*!< Motor Control PWM Interrupt    */
  QEI_IRQn                      = 31,       /*!< Quadrature Encoder Interface   */
  PLL1_IRQn                     = 32,       /*!< PLL1 Lock (USB PLL) Interrupt  */
  USBActivity_IRQn              = 33,       /* USB Activity interrupt           */
  CANActivity_IRQn              = 34,       /* CAN Activity interrupt           */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

#define __CM3_REV                 0x0201    /*!< Core Revision r2p1                      */
#define __NVIC_PRIO_BITS          5         /*!< Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config    */
#define __MPU_PRESENT             1         /*!< MPU present or not                      */

/*@}*/ /* end of group LPC17xx_CMSIS */

#include <core_cm3.h>                       /* Cortex-M3 processor and core peripherals */
#include "system_LPC17xx.h"                 /* LPC17xx System include file              */

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup LPC17xx_Peripherals LPC17xx Peripherals
  LPC17xx Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/*-------------   Summary of system control (SC) ----------------------------*/
/** @addtogroup LPC11xx_SC LPC11xx   Summary of system control (SC) 
  @{
*/
typedef struct
{
  __IO	uint32_t FLASHCFG;		// Flash Accelerator Configuration Register.
	uint32_t RESERVED0[31];

  __IO	uint32_t PLL0CON;		// PLL0 Control Register
  __IO	uint32_t PLL0CFG;		// PLL0 Configuration Register
  __I	uint32_t PLL0STAT;		// PLL0 Status Register
  __O	uint32_t PLL0FEED;		// PLL0 Feed Register
	uint32_t RESERVED1[4];

  __IO	uint32_t PLL1CON;		// PLL1 Control Register
  __IO	uint32_t PLL1CFG;		// PLL1 Configuration Register
  __I	uint32_t PLL1STAT;		// PLL1 Status Register
  __O	uint32_t PLL1FEED;		// PLL1 Feed Register
	uint32_t RESERVED2[4];

  __IO	uint32_t PCON;		// Power Control Register
  __IO	uint32_t PCONP;		// Power Control for Peripherals Register
	uint32_t RESERVED3[15];
  __IO	uint32_t CCLKCFG;		// CPU Clock Configuration Register
  __IO	uint32_t USBCLKCFG;		// USB Clock Configuration Register
  __IO	uint32_t CLKSRCSEL;		// Clock Source Select Register

  __IO	uint32_t CANSLEEPCLR;		// Allows clearing the current CAN channel sleep state
  __IO	uint32_t CANWAKEFLAGS;		// Allows reading the wake-up state of the CAN channels
	uint32_t RESERVED4[12];

  __IO	uint32_t EXTINT;		// External Interrupt Flag Register
	uint32_t RESERVED5[1];
  __IO	uint32_t EXTMODE;		// External Interrupt Mode register
  __IO	uint32_t EXTPOLAR;		// External Interrupt Polarity register
	uint32_t RESERVED6[12];

  __IO	uint32_t RSID;		// Reset Source Identification Register
	uint32_t RESERVED7[7];

  __IO	uint32_t SCS;		// System Control and Status
	uint32_t RESERVED8[1];

  __IO	uint32_t PCLKSEL0;		// Peripheral Clock Selection register 0.
  __IO	uint32_t PCLKSEL1;		// Peripheral Clock Selection register 1.
	uint32_t RESERVED9[4];

  __IO	uint32_t USBIntSt;		// USB Interrupt status

  __IO	uint32_t DMAREQSEL;		// Selects between UART and timer DMA requests on

  __IO	uint32_t CLKOUTCFG;		// Clock Output Configuration Register
} LPC_SC_TypeDef;
/*@}*/ /* end of group LPC11xx_SC */

/*------------- NVIC (NVIC) ----------------------------*/
/** @addtogroup LPC11xx_NVIC LPC11xx NVIC (NVIC) 
  @{
*/
typedef struct
{
  __IO	uint32_t ISER0;		// Interrupt Set-Enable Registers
  __IO	uint32_t ISER1;		// Interrupt Set-Enable Registers
	uint32_t RESERVED0[30];
  __IO	uint32_t ICER0;		// Interrupt Clear-Enable Registers
  __IO	uint32_t ICER1;		// Interrupt Clear-Enable Registers
	uint32_t RESERVED1[30];
  __IO	uint32_t ISPR0;		// Interrupt Set-Pending Registers
  __IO	uint32_t ISPR1;		// Interrupt Set-Pending Registers
	uint32_t RESERVED2[30];
  __IO	uint32_t ICPR0;		// Interrupt Clear-Pending Registers
  __IO	uint32_t ICPR1;		// Interrupt Clear-Pending Registers
	uint32_t RESERVED3[30];
  __I	uint32_t IABR0;		// Interrupt Active Bit Registers
  __I	uint32_t IABR1;		// Interrupt Active Bit Registers
	uint32_t RESERVED4[62];
  __IO	uint32_t IPR0;		// Interrupt Priority Registers
  __IO	uint32_t IPR1;		// Interrupt Priority Registers
  __IO	uint32_t IPR2;		// Interrupt Priority Registers
  __IO	uint32_t IPR3;		// Interrupt Priority Registers
  __IO	uint32_t IPR4;		// Interrupt Priority Registers
  __IO	uint32_t IPR5;		// Interrupt Priority Registers
  __IO	uint32_t IPR6;		// Interrupt Priority Registers
  __IO	uint32_t IPR7;		// Interrupt Priority Registers
  __IO	uint32_t IPR8;		// Interrupt Priority Registers
	uint32_t RESERVED5[695];
  __O	uint32_t STIR;		// Software Trigger Interrupt Register
} LPC_NVIC_TypeDef;
/*@}*/ /* end of group LPC11xx_NVIC */

/*-------------     Pin Connect Block (PINCON) ----------------------------*/
/** @addtogroup LPC11xx_PINCON LPC11xx     Pin Connect Block (PINCON) 
  @{
*/
typedef struct
{
  __IO	uint32_t PINSEL0;		// Pin function select register 0.
  __IO	uint32_t PINSEL1;		// Pin function select register 1.
  __IO	uint32_t PINSEL2;		// Pin function select register 2.
  __IO	uint32_t PINSEL3;		// Pin function select register 3.
  __IO	uint32_t PINSEL4;		// Pin function select register 4
  __IO	uint32_t PINSEL5;		// Pin function select register 5
  __IO	uint32_t PINSEL6;		// Pin function select register 6
  __IO	uint32_t PINSEL7;		// Pin function select register 7
  __IO	uint32_t PINSEL8;		// Pin function select register 8
  __IO	uint32_t PINSEL9;		// Pin function select register 9
  __IO	uint32_t PINSEL10;		// Pin function select register 10
	uint32_t RESERVED1[5];
  __IO	uint32_t PINMODE0;		// Pin mode select register 0
  __IO	uint32_t PINMODE1;		// Pin mode select register 1
  __IO	uint32_t PINMODE2;		// Pin mode select register 2
  __IO	uint32_t PINMODE3;		// Pin mode select register 3.
  __IO	uint32_t PINMODE4;		// Pin mode select register 4
  __IO	uint32_t PINMODE5;		// Pin mode select register 5
  __IO	uint32_t PINMODE6;		// Pin mode select register 6
  __IO	uint32_t PINMODE7;		// Pin mode select register 7
  __IO	uint32_t PINMODE8;		// Pin mode select register 8
  __IO	uint32_t PINMODE9;		// Pin mode select register 9
  __IO	uint32_t PINMODE_OD0;		// Open drain mode control register 0
  __IO	uint32_t PINMODE_OD1;		// Open drain mode control register 1
  __IO	uint32_t PINMODE_OD2;		// Open drain mode control register 2
  __IO	uint32_t PINMODE_OD3;		// Open drain mode control register 3
  __IO	uint32_t PINMODE_OD4;		// Open drain mode control register 4
  __IO  uint32_t I2CPADCFG;             // I2C Pin Configuration
} LPC_PINCON_TypeDef;
/*@}*/ /* end of group LPC11xx_PINCON */

/*------------- GPIO (GPIO) ----------------------------*/
/** @addtogroup LPC11xx_GPIO LPC11xx GPIO (GPIO) 
  @{
*/
typedef struct
{
  __IO	uint32_t FIODIR;		// Fast GPIO Port Direction control register
	uint32_t RESERVED0[3];
  __IO	uint32_t FIOMASK;		// Fast Mask register for port
  __IO	uint32_t FIOPIN;		// Fast Port Pin value register using FIOMASK
  __IO	uint32_t FIOSET;		// Fast Port Output Set register using FIOMASK
  __O	uint32_t FIOCLR;		// Fast Port Output Clear register using FIOMASK
} LPC_GPIO_TypeDef;
/*@}*/ /* end of group LPC11xx_GPIO */

/*------------- GPIO (GPIOINT) ----------------------------*/
/** @addtogroup LPC11xx_GPIOINT LPC11xx GPIO (GPIOINT) 
  @{
*/
typedef struct
{
  __I	uint32_t IntStatus;		// GPIO overall Interrupt Status.
  __I	uint32_t IO0IntStatR;		// GPIO Interrupt Status for Rising edge.
  __I	uint32_t IO0IntStatF;		// GPIO Interrupt Status for Falling edge.
  __O	uint32_t IO0IntClr;		// GPIO Interrupt Clear.
  __IO	uint32_t IO0IntEnR;		// GPIO Interrupt Enable for Rising edge.
  __IO	uint32_t IO0IntEnF;		// GPIO Interrupt Enable for Falling edge.
        uint32_t RESERVED0[3];
  __I	uint32_t IO2IntStatR;		// GPIO Interrupt Status for Rising edge.
  __I	uint32_t IO2IntStatF;		// GPIO Interrupt Status for Falling edge.
  __O	uint32_t IO2IntClr;		// GPIO Interrupt Clear.
  __IO	uint32_t IO2IntEnR;		// GPIO Interrupt Enable for Rising edge.
  __IO	uint32_t IO2IntEnF;		// GPIO Interrupt Enable for Falling edge.
} LPC_GPIOINT_TypeDef;
/*@}*/ /* end of group LPC11xx_GPIOINT */

/*------------- Ethernet (EMAC) ----------------------------*/
/** @addtogroup LPC11xx_EMAC LPC11xx Ethernet (EMAC) 
  @{
*/
typedef struct
{
  __IO	uint32_t MAC1;		// MAC configuration register 1.
  __IO	uint32_t MAC2;		// MAC configuration register 2.
  __IO	uint32_t IPGT;		// Back-to-Back Inter-Packet-Gap register.
  __IO	uint32_t IPGR;		// Non Back-to-Back Inter-Packet-Gap register.
  __IO	uint32_t CLRT;		// Collision window / Retry register.
  __IO	uint32_t MAXF;		// Maximum Frame register.
  __IO	uint32_t SUPP;		// PHY Support register.
  __IO	uint32_t TEST;		// Test register.
  __IO	uint32_t MCFG;		// MII Mgmt Configuration register.
  __IO	uint32_t MCMD;		// MII Mgmt Command register.
  __IO	uint32_t MADR;		// MII Mgmt Address register.
  __O	uint32_t MWTD;		// MII Mgmt Write Data register.
  __I	uint32_t MRDD;		// MII Mgmt Read Data register.
  __I	uint32_t MIND;		// MII Mgmt Indicators register.
	uint32_t RESERVED0[2];
  __IO	uint32_t SA0;		// Station Address 0 register.
  __IO	uint32_t SA1;		// Station Address 1 register.
  __IO	uint32_t SA2;		// Station Address 2 register.
	uint32_t RESERVED1[45];
  __IO	uint32_t Command;		// Command register.
  __I	uint32_t Status;		// Status register.
  __IO	uint32_t RxDescriptor;		// Receive descriptor base address register.
  __IO	uint32_t RxStatus;		// Receive status base address register.
  __IO	uint32_t RxDescriptorNumber;		// Receive number of descriptors register.
  __I	uint32_t RxProduceIndex;		// Receive produce index register.
  __IO	uint32_t RxConsumeIndex;		// Receive consume index register.
  __IO	uint32_t TxDescriptor;		// Transmit descriptor base address register.
  __IO	uint32_t TxStatus;		// Transmit status base address register.
  __IO	uint32_t TxDescriptorNumber;		// Transmit number of descriptors register.
  __IO	uint32_t TxProduceIndex;		// Transmit produce index register.
  __I	uint32_t TxConsumeIndex;		// Transmit consume index register.
	uint32_t RESERVED2[10];
  __I	uint32_t TSV0;		// Transmit status vector 0 register.
  __I	uint32_t TSV1;		// Transmit status vector 1 register.
  __I	uint32_t RSV;		// Receive status vector register.
	uint32_t RESERVED3[3];
  __IO	uint32_t FlowControlCounter;		// Flow control counter register.
  __I	uint32_t FlowControlStatus;		// Flow control status register.
        uint32_t RESERVED4[34];
  __IO  uint32_t RxFilterCtrl;		// Receive filter control register
  __IO  uint32_t RxFilterWoLStatus;		// Receive filter WoL status register
  __IO  uint32_t RxFilterWoLClear;		// Receive filter WoL clear register
        uint32_t RESERVED5[1];
  __IO  uint32_t HashFilterL;		// Hash filter table LSBs register
  __IO  uint32_t HashFilterH;		// Hash filter table MSBs register
        uint32_t RESERVED6[882];
  __IO  uint32_t IntStatus;		// Interrupt status register
  __IO  uint32_t IntEnable;		// Interrupt enable register
  __IO  uint32_t IntClear;		// Interrupt clear register
  __IO  uint32_t IntSet;		// Interrupt set register
        uint32_t RESERVED7[1];
  __IO  uint32_t PowerDown;		// Power-down register
} LPC_EMAC_TypeDef;
/*@}*/ /* end of group LPC11xx_EMAC */

/*------------- USB device (USB) ----------------------------*/
/** @addtogroup LPC11xx_USB LPC11xx USB device (USB) 
  @{
*/
typedef struct
{
  __I	uint32_t HcRevision;		// BCD representation of the version of the HCI
  __IO	uint32_t HcControl;		// Defines the operating modes of the HC
  __IO	uint32_t HcCommandStatus;		// This register is used to receive the
  __IO	uint32_t HcInterruptStatus;		// Indicates the status on various events that
  __IO	uint32_t HcInterruptEnable;		// Controls the bits in the HcInterruptStatus
  __IO	uint32_t HcInterruptDisable;		// The bits in this register are used to
  __IO	uint32_t HcHCCA;		// Contains the physical address of the host controller
  __I	uint32_t HcPeriodCurrentED;		// Contains the physical address of the current
  __IO	uint32_t HcControlHeadED;		// Contains the physical address of the first
  __IO	uint32_t HcControlCurrentED;		// R/W Contains the physical address of the 
  __IO	uint32_t HcBulkHeadED;		// Contains the physical address of the first endpoint
  __IO	uint32_t HcBulkCurrentED;		// Contains the physical address of the current
  __I	uint32_t HcDoneHead;		// Contains the physical address of the last transfer
  __IO	uint32_t HcFmInterval;		// Defines the bit time interval in a frame and the
  __I	uint32_t HcFmRemaining;		// A 14-bit counter showing the bit time remaining in
  __I	uint32_t HcFmNumber;		// Contains a 16-bit counter and provides the timing
  __IO	uint32_t HcPeriodicStart;		// Contains a programmable 14-bit value which
  __IO	uint32_t HcLSThreshold;		// Contains 11-bit value which is used by the HC to
  __IO	uint32_t HcRhDescriptorA;		// First of the two registers which describes t
  __IO	uint32_t HcRhDescriptorB;		// Second of the two registers which describes
  __IO	uint32_t HcRhStatus;		// This register is divided into two parts. The lower
  __IO	uint32_t HcRhPortStatus1;		// Controls and reports the port events on a
  __IO	uint32_t HcRhPortStatus2;		// Controls and reports the port events on a
        uint32_t RESERVED0[40];
  __I	uint32_t Module_ID;		// IP number, where yy (00) is unique version number 

  __I	uint32_t OTGIntSt;		// OTG Interrupt Status
  __IO	uint32_t OTGIntEn;		// OTG Interrupt Enable
  __O	uint32_t OTGIntSet;		// OTG Interrupt Set
  __O	uint32_t OTGIntClr;		// OTG Interrupt Clear
  __IO	uint32_t OTGStCtrl;		// OTG Status and Control
  __IO	uint32_t OTGTmr;		// OTG Timer
        uint32_t RESERVED1[58];
  __I	uint32_t USBDevIntSt;		// USB Device Interrupt Status
  __IO	uint32_t USBDevIntEn;		// USB Device Interrupt Enable
  __O	uint32_t USBDevIntClr;		// USB Device Interrupt Clear
  __O	uint32_t USBDevIntSet;		// USB Device Interrupt Set
  __O	uint32_t USBCmdCode;		// USB Command Code
  __I	uint32_t USBCmdData;		// USB Command Data
  __I	uint32_t USBRxData;		// USB Receive Data
  __O	uint32_t USBTxData;		// USB Transmit Data
  __I	uint32_t USBRxPLen;		// USB Receive Packet Length
  __O	uint32_t USBTxPLen;		// USB Transmit Packet Length
  __IO	uint32_t USBCtrl;		// USB Control
  __O	uint32_t USBDevIntPri;		// USB Device Interrupt Priority
  __I	uint32_t USBEpIntSt;		// USB Endpoint Interrupt Status
  __IO	uint32_t USBEpIntEn;		// USB Endpoint Interrupt Enable
  __O	uint32_t USBEpIntClr;		// USB Endpoint Interrupt Clear
  __O	uint32_t USBEpIntSet;		// USB Endpoint Interrupt Set
  __O	uint32_t USBEpIntPri;		// USB Endpoint Priority
  __IO	uint32_t USBReEp;		// USB Realize Endpoint
  __O	uint32_t USBEpIn;		// USB Endpoint Index
  __IO	uint32_t USBMaxPSize;		// USB MaxPacketSize

  __I	uint32_t USBDMARSt;		// USB DMA Request Status
  __O	uint32_t USBDMARClr;		// USB DMA Request Clear
  __O	uint32_t USBDMARSet;		// USB DMA Request Set
	uint32_t USBRESERVED0[9];
  __IO	uint32_t USBUDCAH;		// USB UDCA Head
  __I	uint32_t USBEpDMASt;		// USB Endpoint DMA Status
  __O	uint32_t USBEpDMAEn;		// USB Endpoint DMA Enable
  __O	uint32_t USBEpDMADis;		// USB Endpoint DMA Disable
  __I	uint32_t USBDMAIntSt;		// USB DMA Interrupt Status
  __IO	uint32_t USBDMAIntEn;		// USB DMA Interrupt Enable
	uint32_t USBRESERVED1[2];
  __I	uint32_t USBEoTIntSt;		// USB End of Transfer Interrupt Status
  __O	uint32_t USBEoTIntClr;		// USB End of Transfer Interrupt Clear
  __O	uint32_t USBEoTIntSet;		// USB End of Transfer Interrupt Set
  __I	uint32_t USBNDDRIntSt;		// USB New DD Request Interrupt Status
  __O	uint32_t USBNDDRIntClr;		// USB New DD Request Interrupt Clear
  __O	uint32_t USBNDDRIntSet;		// USB New DD Request Interrupt Set
  __I	uint32_t USBSysErrIntSt;		// USB System Error Interrupt Status
  __O	uint32_t USBSysErrIntClr;		// USB System Error Interrupt Clear
  __O	uint32_t USBSysErrIntSet;		// USB System Error Interrupt Set
        uint32_t RESERVED2[15];

  union {
  __I	uint32_t I2C_RX;		// I2C Receive
  __O	uint32_t I2C_TX;		// I2C Transmit
  };
  __I	uint32_t I2C_STS;		// I2C Status
  __IO	uint32_t I2C_CTL;		// I2C Control
  __IO	uint32_t I2C_CLKHI;		// I2C Clock High
  __O	uint32_t I2C_CLKLO;		// I2C Clock Low
        uint32_t RESERVED3[824];
  
  union {
  __IO	uint32_t USBClkCtrl;		// USB Clock Control
  __IO	uint32_t OTGClkCtrl;		// USB Clock Control
  };
  union {
  __I	uint32_t USBClkSt;		// USB Clock Status
  __I	uint32_t OTGClkSt;		// USB Clock Status
  };
} LPC_USB_TypeDef;
/*@}*/ /* end of group LPC11xx_USB */

/*------------- UART (UART) ----------------------------*/
/** @addtogroup LPC11xx_UART LPC11xx UART (UART) 
  @{
*/
typedef struct
{
  union {
    __I	uint32_t RBR;		// Receiver Buffer Register. Contains the next received
    __O	uint32_t THR;		// Transmit Holding Register. The next character to be
    __IO uint32_t DLL;		// Divisor Latch LSB. Least significant byte of the baud
  };
  union {
    __IO uint32_t DLM;		// Divisor Latch LSB. Least significant byte of the baud
    __IO uint32_t IER;		// Interrupt Enable Register. Contains individual interrupt
  };
  union {
    __I uint32_t IIR;		// Interrupt ID Register. Identifies which interrupt(s) are
    __O	uint32_t FCR;		// FIFO Control Register. Controls UART FIFO usage and 
  };
  __IO	uint32_t LCR;		// Line Control Register. Contains controls for frame
	uint32_t RESERVED0[1];
  __I	uint32_t LSR;		// Line Status Register. Contains flags for transmit and
	uint32_t RESERVED1[1];
  __IO	uint32_t SCR;		// Scratch Pad Register. 8-bit temporary storage for
  __IO	uint32_t ACR;		// Auto-baud Control Register. Contains controls for the
  __IO	uint32_t ICR;		// IrDA Control Register. Enables and configures the
  __IO	uint32_t FDR;		// Fractional Divider Register. Generates a clock input for
	uint32_t RESERVED2[1];
  __IO	uint32_t TER;		// Transmit Enable Register. Turns off UART transmitter
} LPC_UART_TypeDef;
/*@}*/ /* end of group LPC11xx_UART */

/*------------- CAN1 and CAN2 controller (CAN) ----------------------------*/
/** @addtogroup LPC11xx_CAN LPC11xx CAN1 and CAN2 controller (CAN) 
  @{
*/
typedef struct
{
  __IO	uint32_t MOD;		// Controls the operating mode of the CAN Controller.
  __O	uint32_t CMR;		// Command bits that affect the state of the CAN Controller
  __I	uint32_t GSR;		// Global Controller Status and Error Counters
  __I	uint32_t ICR;		// Interrupt status, Arbitration Lost Capture, Error Code
  __IO	uint32_t IER;		// Interrupt Enable
  __IO	uint32_t BTR;		// Bus Timing
  __IO	uint32_t EWL;		// Error Warning Limit
  __I	uint32_t SR;		// Status Register
  __IO	uint32_t RFS;		// Receive frame status
  __IO	uint32_t RID;		// Received Identifier
  __IO	uint32_t RDA;		// Received data bytes 1-4
  __IO	uint32_t RDB;		// Received data bytes 5-8
  __IO	uint32_t TFI1;		// Transmit frame info (Tx Buffer 1)
  __IO	uint32_t TID1;		// Transmit Identifier (Tx Buffer 1)
  __IO	uint32_t TDA1;		// Transmit data bytes 1-4 (Tx Buffer 1)
  __IO	uint32_t TDB1;		// Transmit data bytes 5-8 (Tx Buffer 1)
  __IO	uint32_t TFI2;		// Transmit frame info (Tx Buffer 2)
  __IO	uint32_t TID2;		// Transmit Identifier (Tx Buffer 2)
  __IO	uint32_t TDA2;		// Transmit data bytes 1-4 (Tx Buffer 2)
  __IO	uint32_t TDB2;		// Transmit data bytes 5-8 (Tx Buffer 2)
  __IO	uint32_t TFI3;		// Transmit frame info (Tx Buffer 3)
  __IO	uint32_t TID3;		// Transmit Identifier (Tx Buffer 3)
  __IO	uint32_t TDA3;		// Transmit data bytes 1-4 (Tx Buffer 3)
  __IO	uint32_t TDB3;		// Transmit data bytes 5-8 (Tx Buffer 3)

} LPC_CAN_TypeDef;
/*@}*/ /* end of group LPC11xx_CAN */

/*------------- SPI (SPI) ----------------------------*/
/** @addtogroup LPC11xx_SPI LPC11xx SPI (SPI) 
  @{
*/
typedef struct
{
  __IO	uint32_t SPCR;		// SPI Control Register. This register controls the
  __I	uint32_t SPSR;		// SPI Status Register. This register shows the
  __IO	uint32_t SPDR;		// SPI Data Register. This bi-directional register
  __IO	uint32_t SPCCR;		// SPI Clock Counter Register. This register
	uint32_t RESERVED0[3];
  __IO	uint32_t SPINT;		// SPI Interrupt Flag. This register contains the
} LPC_SPI_TypeDef;
/*@}*/ /* end of group LPC11xx_SPI */

/*------------- SSP (SSP) ----------------------------*/
/** @addtogroup LPC11xx_SSP LPC11xx SSP (SSP) 
  @{
*/
typedef struct
{
  __IO	uint32_t CR0;		// Control Register 0. Selects the
  __IO	uint32_t CR1;		// Control Register 1. Selects
  __IO	uint32_t DR;		// Data Register. Writes fill the
  __I	uint32_t SR;		// Status Register
  __IO	uint32_t CPSR;		// Clock Prescale Register
  __IO	uint32_t IMSC;		// Interrupt Mask Set and Clear
	uint32_t RESERVED1[1];
  __IO	uint32_t MIS;		// Masked Interrupt Status Register
  __IO	uint32_t ICR;		// SSPICR Interrupt Clear Register
  __IO	uint32_t DMACR;		// DMA Control Register 
} LPC_SSP_TypeDef;
/*@}*/ /* end of group LPC11xx_SSP */

/*------------- I2C (I2C) ----------------------------*/
/** @addtogroup LPC11xx_I2C LPC11xx I2C (I2C) 
  @{
*/
typedef struct
{
  __IO	uint32_t I2CONSET;		// I2C Control Set Register. When a one is written to
  __I	uint32_t I2STAT;		// I2C Status Register. During I2C operation, this
  __IO	uint32_t I2DAT;		// I2C Data Register. During master or slave transmit
  __IO	uint32_t I2ADR0;		// I2C Slave Address Register 0. Contains the 7-bit
  __IO	uint32_t I2SCLH;		// SCH Duty Cycle Register High Half Word.
  __IO	uint32_t I2SCLL;		// SCL Duty Cycle Register Low Half Word.
  __O	uint32_t I2CONCLR;		// I2C Control Clear Register. When a one is written
  __IO	uint32_t MMCTRL;		// Monitor mode control register.
  __IO	uint32_t I2ADR1;		// I2C Slave Address Register 1. Contains the 7-bit
  __IO	uint32_t I2ADR2;		// I2C Slave Address Register 2. Contains the 7-bit
  __IO	uint32_t I2ADR3;		// I2C Slave Address Register 3. Contains the 7-bit
  __I	uint32_t I2DATA_BUFFER;		// Data buffer register. The contents of the 8 MSBs of
  __IO	uint32_t I2MASK0;		// I2C Slave address mask register 0. This mask
  __IO	uint32_t I2MASK1;		// I2C Slave address mask register 1. This mask
  __IO	uint32_t I2MASK2;		// I2C Slave address mask register 2. This mask
  __IO	uint32_t I2MASK3;		// I2C Slave address mask register 3. This mask
} LPC_I2C_TypeDef;
/*@}*/ /* end of group LPC11xx_I2C */

/*------------- I2S (I2S) ----------------------------*/
/** @addtogroup LPC11xx_I2S LPC11xx I2S (I2S) 
  @{
*/
typedef struct
{
  __IO	uint32_t DAO;		// Digital Audio Output Register. Contains control bits for
  __IO	uint32_t DAI;		// Digital Audio Input Register. Contains control bits for
  __O	uint32_t TXFIFO;		// Transmit FIFO. Access register for the 8 x 32-bit
  __I	uint32_t RXFIFO;		// Receive FIFO. Access register for the 8 x 32-bit
  __I	uint32_t STATE;		// Status Feedback Register. Contains status information about
  __IO	uint32_t DMA1;		// DMA Configuration Register 1. Contains control information
  __IO	uint32_t DMA2;		// DMA Configuration Register 2. Contains control information
  __IO	uint32_t IRQ;		// Interrupt Request Control Register. Contains bits that
  __IO	uint32_t TXRATE;		// Transmit MCLK divider. This register determines
  __IO	uint32_t RXRATE;		// Receive MCLK divider. This register determines
  __IO	uint32_t TXBITRATE;		// Transmit bit rate divider. This register determines
  __IO	uint32_t RXBITRATE;		// Receive bit rate divider. This register determines
  __IO	uint32_t TXMODE;		// Transmit mode control.
  __IO	uint32_t RXMODE;		// Receive mode control.
} LPC_I2S_TypeDef;
/*@}*/ /* end of group LPC11xx_I2S */

/*------------- TIMER/COUNTER0-3 (TIM) ----------------------------*/
/** @addtogroup LPC11xx_TIM LPC11xx TIMER/COUNTER0-3 (TIM) 
  @{
*/
typedef struct
{
  __IO	uint32_t IR;		// Interrupt Register. The IR can be written to clear
  __IO	uint32_t TCR;		// Timer Control Register. The TCR is used to control the Timer
  __IO	uint32_t TC;		// Timer Counter. The 32-bit TC is incremented every PR+1
  __IO	uint32_t PR;		// Prescale Register. When the Prescale Counter (below)
  __IO	uint32_t PC;		// Prescale Counter. The 32-bit PC is a counter which is
  __IO	uint32_t MCR;		// Match Control Register. The MCR is used to control if an
  __IO	uint32_t MR0;		// Match Register 0. MR0 can be enabled through the MCR to
  __IO	uint32_t MR1;		// Match Register 1. See MR0 description.
  __IO	uint32_t MR2;		// Match Register 2. See MR0 description.
  __IO	uint32_t MR3;		// Match Register 3. See MR0 description.
  __IO	uint32_t CCR;		// Capture Control Register. The CCR controls which edges of
  __I	uint32_t CR0;		// Capture Register 0. CR0 is loaded with the value of TC when
  __I	uint32_t CR1;		// Capture Register 1. See CR0 description.
	uint32_t RESERVED0[2];
  __IO	uint32_t EMR;		// External Match Register. The EMR controls the external
	uint32_t RESERVED1[12];
  __IO	uint32_t CTCR;		// Count Control Register. The CTCR selects between Timer and
} LPC_TIM_TypeDef;
/*@}*/ /* end of group LPC11xx_TIM */

/*------------- Repetitive Interrupt Timer (RIT) ----------------------------*/
/** @addtogroup LPC11xx_RIT LPC11xx Repetitive Interrupt Timer (RIT) 
  @{
*/
typedef struct
{
  __IO	uint32_t RICOMPVAL;		// Compare register
  __IO	uint32_t RIMASK;		// Mask register. This register holds the 32-bit mask
  __IO	uint32_t RICTRL;		// Control register.
  __IO	uint32_t RICOUNTER;		// 32-bit counter
} LPC_RIT_TypeDef;
/*@}*/ /* end of group LPC11xx_RIT */

/*------------- System Tick Timer (SYSTICK) ----------------------------*/
/** @addtogroup LPC11xx_SYSTICK LPC11xx System Tick Timer (SYSTICK) 
  @{
*/
typedef struct
{
  __IO	uint32_t CTRL;		// System Timer Control and status register
  __IO	uint32_t RELOAD;		// System Timer Reload value register
  __IO	uint32_t CURR;		// System Timer Current value register
  __IO	uint32_t CALIB;		// System Timer Calibration value register
} LPC_SYSTICK_TypeDef;
/*@}*/ /* end of group LPC11xx_SYSTICK */

/*------------- PWM1 (PWM) ----------------------------*/
/** @addtogroup LPC11xx_PWM LPC11xx PWM1 (PWM) 
  @{
*/
typedef struct
{
  __IO	uint32_t IR;		// Interrupt Register. The IR can be written to clear interrupt
  __IO	uint32_t TCR;		// Timer Control Register. The TCR is used to control the
  __IO	uint32_t TC;		// Timer Counter. The 32-bit TC is incremented every PR+1 cycle
  __IO	uint32_t PR;		// Prescale Register. The TC is incremented every PR+1 cycles 
  __IO	uint32_t PC;		// Prescale Counter. The 32-bit PC is a counter which is
  __IO	uint32_t MCR;		// Match Control Register. The MCR is used to control if
  __IO	uint32_t MR0;		// Match Register 0. MR0 can be enabled in the MCR to reset
  __IO	uint32_t MR1;		// Match Register 1. MR1 can be enabled in the MCR to reset
  __IO	uint32_t MR2;		// Match Register 2. MR2 can be enabled in the MCR to reset
  __IO	uint32_t MR3;		// Match Register 3. MR3 can be enabled in the MCR to reset
  __IO	uint32_t CCR;		// Capture Control Register. The CCR controls which edges
  __I	uint32_t CR0;		// Capture Register 0. CR0 is loaded with the value of the TC
  __I	uint32_t CR1;		// Capture Register 1. See CR0 description.
  __I	uint32_t CR2;		// Capture Register 2. See CR0 description.
  __I	uint32_t CR3;		// Capture Register 3. See CR0 description.
	uint32_t RESERVED0[1];
  __IO	uint32_t MR4;		// Match Register 4. MR4 can be enabled in the MCR to reset
  __IO	uint32_t MR5;		// Match Register 5. MR5 can be enabled in the MCR to reset
  __IO	uint32_t MR6;		// Match Register 6. MR6 can be enabled in the MCR to reset
  __IO	uint32_t PCR;		// PWM Control Register. Enables PWM outputs and selects PWM
  __IO	uint32_t LER;		// Load Enable Register. Enables use of new PWM match values.
	uint32_t RESERVED1[7];
  __IO	uint32_t CTCR;		// Count Control Register. The CTCR selects between Timer and
} LPC_PWM_TypeDef;
/*@}*/ /* end of group LPC11xx_PWM */

/*------------- Motor Control Pulse Width Modulator (MCPWM) (MCPWM) ----------------------------*/
/** @addtogroup LPC11xx_MCPWM LPC11xx Motor Control Pulse Width Modulator (MCPWM) (MCPWM) 
  @{
*/
typedef struct
{
  __I	uint32_t MCCON;		// PWM Control read address
  __O	uint32_t MCCON_SET;		// PWM Control set address
  __O	uint32_t MCCON_CLR;		// PWM Control clear address
  __I	uint32_t MCCAPCON;		// Capture Control read address
  __O	uint32_t MCCAPCON_SET;		// Capture Control set address
  __O	uint32_t MCCAPCON_CLR;		// Event Control clear address
  __IO	uint32_t MCTC0;		// Timer Counter register, channel 0
  __IO	uint32_t MCTC1;		// Timer Counter register, channel 1
  __IO	uint32_t MCTC2;		// Timer Counter register, channel 2
  __IO	uint32_t MCLIM0;		// Limit register, channel 0
  __IO	uint32_t MCLIM1;		// Limit register, channel 1
  __IO	uint32_t MCLIM2;		// Limit register, channel 2
  __IO	uint32_t MCMAT0;		// Match register, channel 0
  __IO	uint32_t MCMAT1;		// Match register, channel 1
  __IO	uint32_t MCMAT2;		// Match register, channel 2
  __IO	uint32_t MCDT;		// Dead time register
  __IO	uint32_t MCCP;		// Commutation Pattern register
  __I	uint32_t MCCAP0;		// Capture register, channel 0
  __I	uint32_t MCCAP1;		// Capture register, channel 1
  __I	uint32_t MCCAP2;		// Capture register, channel 2
  __I	uint32_t MCINTEN;		// Interrupt Enable read address
  __O	uint32_t MCINTEN_SET;		// Interrupt Enable set address
  __O	uint32_t MCINTEN_CLR;		// Interrupt Enable clear address
  __I	uint32_t MCCNTCON;		// Count Control read address
  __O	uint32_t MCCNTCON_SET;		// Count Control set address
  __O	uint32_t MCCNTCON_CLR;		// Count Control clear address
  __I	uint32_t MCINTF;		// Interrupt flags read address
  __O	uint32_t MCINTF_SET;		// Interrupt flags set address
  __O	uint32_t MCINTF_CLR;		// Interrupt flags clear address
  __O	uint32_t MCCAP_CLR;		// Capture clear address
} LPC_MCPWM_TypeDef;
/*@}*/ /* end of group LPC11xx_MCPWM */

/*------------- QEI (QEI) ----------------------------*/
/** @addtogroup LPC11xx_QEI LPC11xx QEI (QEI) 
  @{
*/
typedef struct
{
  __O	uint32_t QEICON;		// Control register
  __I	uint32_t QEISTAT;		// Encoder status register
  __IO	uint32_t QEICONF;		// Configuration register
  __I	uint32_t QEIPOS;		// Position register
  __IO	uint32_t QEIMAXPOS;		// Maximum position register
  __IO	uint32_t CMPOS0;		// position compare register 0
  __IO	uint32_t CMPOS1;		// position compare register 1
  __IO	uint32_t CMPOS2;		// position compare register 2
  __I	uint32_t INXCNT;		// Index count register
  __IO	uint32_t INXCMP;		// Index compare register
  __IO	uint32_t QEILOAD;		// Velocity timer reload register
  __I	uint32_t QEITIME;		// Velocity timer register
  __I	uint32_t QEIVEL;		// Velocity counter register
  __I	uint32_t QEICAP;		// Velocity capture register
  __IO	uint32_t VELCOMP;		// Velocity compare register
  __IO	uint32_t FILTER;		// Digital filter register
	uint32_t RESERVED0[998];
  __O	uint32_t QEIIEC;		// Interrupt enable clear register
  __O	uint32_t QEIIES;		// Interrupt enable set register
  __I	uint32_t QEIINTSTAT;		// Interrupt status register
  __I	uint32_t QEIIE;		// Interrupt enable register
  __O	uint32_t QEICLR;		// Interrupt status clear register
  __O	uint32_t QEISET;		// Interrupt status set register
} LPC_QEI_TypeDef;
/*@}*/ /* end of group LPC11xx_QEI */

/*------------- Real-Time Clock (RTC) ----------------------------*/
/** @addtogroup LPC11xx_RTC LPC11xx Real-Time Clock (RTC) 
  @{
*/
typedef struct
{
  __IO	uint32_t ILR;		// Interrupt Location Register
	uint32_t RESERVED0[1];
  __IO	uint32_t CCR;		// Clock Control Register
  __IO	uint32_t CIIR;		// Counter Increment Interrupt Register
  __IO	uint32_t AMR;		// Alarm Mask Register
  __I	uint32_t CTIME0;		// Consolidated Time Register 0
  __I	uint32_t CTIME1;		// Consolidated Time Register 1
  __I	uint32_t CTIME2;		// Consolidated Time Register 2
  __IO	uint32_t SEC;		// Seconds Counter
  __IO	uint32_t MIN;		// Minutes Register
  __IO	uint32_t HOUR;		// Hours Register
  __IO	uint32_t DOM;		// Day of Month Register
  __IO	uint32_t DOW;		// Day of Week Register
  __IO	uint32_t DOY;		// Day of Year Register
  __IO	uint32_t MONTH;		// Months Register
  __IO	uint32_t YEAR;		// Years Register
  __IO	uint32_t CALIBRATION;		// Calibration Value Register
  __IO	uint32_t GPREG0;		// General Purpose Register 0
  __IO	uint32_t GPREG1;		// General Purpose Register 1
  __IO	uint32_t GPREG2;		// General Purpose Register 2
  __IO	uint32_t GPREG3;		// General Purpose Register 3
  __IO	uint32_t GPREG4;		// General Purpose Register 4
  __IO	uint32_t RTC_AUXEN;		// RTC Auxiliary Enable register
  __IO	uint32_t RTC_AUX;		// RTC Auxiliary control register
  __IO	uint32_t ALSEC;		// Alarm value for Seconds
  __IO	uint32_t ALMIN;		// Alarm value for Minutes
  __IO	uint32_t ALHOUR;		// Alarm value for Hours
  __IO	uint32_t ALDOM;		// Alarm value for Day of Month
  __IO	uint32_t ALDOW;		// Alarm value for Day of Week
  __IO	uint32_t ALDOY;		// Alarm value for Day of Year
  __IO	uint32_t ALMON;		// Alarm value for Months
  __IO	uint32_t ALYEAR;		// Alarm value for Year
} LPC_RTC_TypeDef;
/*@}*/ /* end of group LPC11xx_RTC */

/*------------- Watchdog (WDT) ----------------------------*/
/** @addtogroup LPC11xx_WDT LPC11xx Watchdog (WDT) 
  @{
*/
typedef struct
{
  __IO	uint32_t WDMOD;		// Watchdog mode register. This register contains the basic
  __IO	uint32_t WDTC;		// Watchdog timer constant register. This register determines
  __O	uint32_t WDFEED;		// Watchdog feed sequence register. Writing 0xAA follow
  __I	uint32_t WDTV;		// Watchdog timer value register. This register reads out the 
  __IO	uint32_t WDCLKSEL;		// Watchdog clock source selection register.
} LPC_WDT_TypeDef;
/*@}*/ /* end of group LPC11xx_WDT */

/*------------- ADC (ADC) ----------------------------*/
/** @addtogroup LPC11xx_ADC LPC11xx ADC (ADC) 
  @{
*/
typedef struct
{
  __IO	uint32_t ADCR;		// A/D Control Register. The ADCR register must be written
  __IO	uint32_t ADGDR;		// A/D Global Data Register. This register contains the ADC’s
	uint32_t RESERVED0;
  __IO	uint32_t ADINTEN;		// A/D Interrupt Enable Register. This register
  __I	uint32_t ADDR0;		// A/D Channel 0 Data Register. This register contains the
  __I	uint32_t ADDR1;		// A/D Channel 1 Data Register. This register contains the
  __I	uint32_t ADDR2;		// A/D Channel 2 Data Register. This register contains the
  __I	uint32_t ADDR3;		// A/D Channel 3 Data Register. This register contains the
  __I	uint32_t ADDR4;		// A/D Channel 4 Data Register. This register contains the
  __I	uint32_t ADDR5;		// A/D Channel 5 Data Register. This register contains the
  __I	uint32_t ADDR6;		// A/D Channel 6 Data Register. This register contains the
  __I	uint32_t ADDR7;		// A/D Channel 7 Data Register. This register contains the 
  __I	uint32_t ADSTAT;		// A/D Status Register. This register contains DONE and
  __IO	uint32_t ADTRM;		// ADC trim register.
} LPC_ADC_TypeDef;
/*@}*/ /* end of group LPC11xx_ADC */

/*------------- DAC (DAC) ----------------------------*/
/** @addtogroup LPC11xx_DAC LPC11xx DAC (DAC) 
  @{
*/
typedef struct
{
  __IO	uint32_t DACR;		// D/A Converter Register. This register contains the digital
  __IO	uint32_t DACCTRL;		// DAC Control register. This register controls DMA and timer
  __IO	uint32_t DACCNTVAL;		// DAC Counter Value register. This register contains
} LPC_DAC_TypeDef;
/*@}*/ /* end of group LPC11xx_DAC */

/*------------- GPDMA (GPDMA) ----------------------------*/
/** @addtogroup LPC11xx_GPDMA LPC11xx GPDMA (GPDMA) 
  @{
*/
typedef struct
{
  __I	uint32_t DMACIntStat;		// DMA Interrupt Status Register
  __I	uint32_t DMACIntTCStat;		// DMA Interrupt Terminal Count Request Status Register
  __O	uint32_t DMACIntTCClear;		// DMA Interrupt Terminal Count Request Clear Register
  __I	uint32_t DMACIntErrStat;		// DMA Interrupt Error Status Register
  __O	uint32_t DMACIntErrClr;		// DMA Interrupt Error Clear Register
  __I	uint32_t DMACRawIntTCStat;		// DMA Raw Interrupt Terminal Count Status Register
  __I	uint32_t DMACRawIntErrStat;		// DMA Raw Error Interrupt Status Register
  __I	uint32_t DMACEnbldChns;		// DMA Enabled Channel Register
  __IO	uint32_t DMACSoftBReq;		// DMA Software Burst Request Register
  __IO	uint32_t DMACSoftSReq;		// DMA Software Single Request Register
  __IO	uint32_t DMACSoftLBReq;		// DMA Software Last Burst Request Register
  __IO	uint32_t DMACSoftLSReq;		// DMA Software Last Single Request Register
  __IO	uint32_t DMACConfig;		// DMA Configuration Register
  __IO	uint32_t DMACSync;		// DMA Synchronization Register
	uint32_t DMARESERVED0[50];
  __IO	uint32_t DMACC0SrcAddr;		// DMA Channel 0 Source Address Register
  __IO	uint32_t DMACC0DestAddr;		// DMA Channel 0 Destination Address Register
  __IO	uint32_t DMACC0LLI;		// DMA Channel 0 Linked List Item Register
  __IO	uint32_t DMACC0Config;		// DMA Channel 0 Configuration Register
	uint32_t DMARESERVED1[4];
  __IO	uint32_t DMACC1SrcAddr;		// DMA Channel 1 Source Address Register
  __IO	uint32_t DMACC1DestAddr;		// DMA Channel 1 Destination Address Register
  __IO	uint32_t DMACC1LLI;		// DMA Channel 1 Linked List Item Register
  __IO	uint32_t DMACC1Config;		// DMA Channel 1 Configuration Register
	uint32_t DMARESERVED2[4];
  __IO	uint32_t DMACC2SrcAddr;		// DMA Channel 2 Source Address Register
  __IO	uint32_t DMACC2DestAddr;		// DMA Channel 2 Destination Address Register
  __IO	uint32_t DMACC2LLI;		// DMA Channel 2 Linked List Item Register
  __IO	uint32_t DMACC2Control;		// DMA Channel 2 Control Register
	uint32_t DMARESERVED3[4];
  __IO	uint32_t DMACC3SrcAddr;		// DMA Channel 3 Source Address Register
  __IO	uint32_t DMACC3DestAddr;		// DMA Channel 3 Destination Address Register
  __IO	uint32_t DMACC3LLI;		// DMA Channel 3 Linked List Item Register
  __IO	uint32_t DMACC3Config;		// DMA Channel 3 Configuration Register
	uint32_t DMARESERVED4[4];
  __IO	uint32_t DMACC4SrcAddr;		// DMA Channel 4 Source Address Register
  __IO	uint32_t DMACC4DestAddr;		// DMA Channel 4 Destination Address Register
  __IO	uint32_t DMACC4LLI;		// DMA Channel 4 Linked List Item Register
  __IO	uint32_t DMACC4Config;		// DMA Channel 4 Configuration Register
	uint32_t DMARESERVED5[4];
  __IO	uint32_t DMACC5SrcAddr;		// DMA Channel 5 Source Address Register
  __IO	uint32_t DMACC5DestAddr;		// DMA Channel 5 Destination Address Register
  __IO	uint32_t DMACC5LLI;		// DMA Channel 5 Linked List Item Register
  __IO	uint32_t DMACC5Config;		// DMA Channel 5 Configuration Register
	uint32_t DMARESERVED6[4];
  __IO	uint32_t DMACC6SrcAddr;		// DMA Channel 6 Source Address Register
  __IO	uint32_t DMACC6DestAddr;		// DMA Channel 6 Destination Address Register
  __IO	uint32_t DMACC6LLI;		// DMA Channel 6 Linked List Item Register
  __IO	uint32_t DMACC6Config;		// DMA Channel 6 Configuration Register
	uint32_t DMARESERVED7[4];
  __IO	uint32_t DMACC7SrcAddr;		// DMA Channel 7 Source Address Register
  __IO	uint32_t DMACC7DestAddr;		// DMA Channel 7 Destination Address Register
  __IO	uint32_t DMACC7LLI;		// DMA Channel 7 Linked List Item Register
  __IO	uint32_t DMACC7Config;		// DMA Channel 7 Configuration Register
} LPC_GPDMA_TypeDef;
/*@}*/ /* end of group LPC11xx_GPDMA */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/*@}*/ /* end of group LPC17xx_Peripherals */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/

/** @addtogroup LPC17xx_MemoryMap LPC17xx Memory Mapping
  @{
*/

/* Peripheral and SRAM base address */
#define LPC_FLASH_BASE       (0x00000000UL)     /*!< FLASH Base Address       */
#define LPC_SRAM_BASE        (0x10000000UL)     /*!< SRAM Base Address        */

/*@}*/ /* end of group LPC17xx_MemoryMap */


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/

/** @addtogroup LPC17xx_PeripheralDecl LPC17xx Peripheral Declaration
  @{
*/

#define LPC_SC			((LPC_SC_TypeDef*	) 0x400fc000 )
#define LPC_NVIC		((LPC_NVIC_TypeDef*	) 0xe000e100 )
#define LPC_PINCON		((LPC_PINCON_TypeDef*	) 0x4002c000 )

#define LPC_GPIO0		((LPC_GPIO_TypeDef*	) 0x2009c000 )
#define LPC_GPIO1		((LPC_GPIO_TypeDef*	) 0x2009c020 )
#define LPC_GPIO2		((LPC_GPIO_TypeDef*	) 0x2009c040 )
#define LPC_GPIO3		((LPC_GPIO_TypeDef*	) 0x2009c060 )
#define LPC_GPIO4		((LPC_GPIO_TypeDef*	) 0x2009c080 )
#define LPC_GPIOINT		((LPC_GPIOINT_TypeDef*	) 0x40028080 )

#define LPC_EMAC		((LPC_EMAC_TypeDef*	) 0x50000000 )

#define LPC_UART0		((LPC_UART_TypeDef*	) 0x4000c000 )
#define LPC_UART1		((LPC_UART_TypeDef*	) 0x40010000 )
#define LPC_UART2		((LPC_UART_TypeDef*	) 0x40098000 )
#define LPC_UART3		((LPC_UART_TypeDef*	) 0x4009c000 )

#define LPC_CAN1		((LPC_CAN_TypeDef*	) 0x40044000 )
#define LPC_CAN2		((LPC_CAN_TypeDef*	) 0x40048000 )

#define LPC_SPI			((LPC_SPI_TypeDef*	) 0x40020000 )
#define LPC_SSP1		((LPC_SSP_TypeDef*	) 0x40088000 )
#define LPC_SSP2		((LPC_SSP_TypeDef*	) 0x40030000 )

#define LPC_I2C0		((LPC_I2C_TypeDef*	) 0x4001c000 )
#define LPC_I2C1		((LPC_I2C_TypeDef*	) 0x4005c000 )
#define LPC_I2C2		((LPC_I2C_TypeDef*	) 0x400a0000 )
#define LPC_I2S			((LPC_I2S_TypeDef*	) 0x400a8000 )

#define LPC_TIM0		((LPC_TIM_TypeDef*	) 0x40004000 )
#define LPC_TIM1		((LPC_TIM_TypeDef*	) 0x40008000 )
#define LPC_TIM2		((LPC_TIM_TypeDef*	) 0x40090000 )
#define LPC_TIM3		((LPC_TIM_TypeDef*	) 0x40094000 )

#define LPC_RIT			((LPC_RIT_TypeDef*	) 0x400b0000 )
#define LPC_SYSTICK		((LPC_SYSTICK_TypeDef*	) 0xe000e010 )
#define LPC_PWM			((LPC_PWM_TypeDef*	) 0x40018000 )
#define LPC_MCPWM		((LPC_MCPWM_TypeDef*	) 0x400b8000 )
#define LPC_QEI			((LPC_QEI_TypeDef*	) 0x400bc000 )
#define LPC_RTC			((LPC_RTC_TypeDef*	) 0x40024000 )
#define LPC_WDT			((LPC_WDT_TypeDef*	) 0x40000000 )
#define LPC_ADC			((LPC_ADC_TypeDef*	) 0x40034000 )
#define LPC_DAC			((LPC_DAC_TypeDef*	) 0x4008c000 )
#define LPC_GPDMA		((LPC_GPDMA_TypeDef*	) 0x50004000 )

#define LPC_USB			((LPC_USB_TypeDef*	) 0x5000c000 )

/*@}*/ /* end of group LPC17xx_PeripheralDecl */

/*@}*/ /* end of group LPC17xx_Definitions */

#ifdef __cplusplus
}
#endif

#endif  /* LPC17xx_H */
