/**
  ******************************************************************************
  * @file    stm32l476xx.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-June-2015
  * @brief   CMSIS STM32L476xx Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral�s registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup stm32l476xx
  * @{
  */

#ifndef __STM32L476xx_H
#define __STM32L476xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
   */
#define __CM4_REV                 0x0001  /*!< Cortex-M4 revision r0p1                       */
#define __MPU_PRESENT             1       /*!< STM32L4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4       /*!< STM32L4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1       /*!< FPU present                                   */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32L4XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  HardFault_IRQn              = -13,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_PVM_IRQn                = 1,      /*!< PVD/PVM1/PVM2/PVM3/PVM4 through EXTI Line detection Interrupts    */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                                   */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                                   */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                                   */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                                   */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                                   */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                                   */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                                   */
  ADC1_2_IRQn                 = 18,     /*!< ADC1, ADC2 SAR global Interrupts                                  */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break interrupt and TIM15 global interrupt                   */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM16 global interrupt                  */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM17 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  DFSDM3_IRQn                 = 42,     /*!< SD Filter 3 global Interrupt                                      */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                              */
  TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                                             */
  TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupt                            */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  ADC3_IRQn                   = 47,     /*!< ADC3 global  Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDMMC1_IRQn                 = 49,     /*!< SDMMC1 global Interrupt                                           */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                                   */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                                   */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                                   */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                                   */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                                   */
  DFSDM0_IRQn                 = 61,     /*!< SD Filter 0 global Interrupt                                      */
  DFSDM1_IRQn                 = 62,     /*!< SD Filter 1 global Interrupt                                      */
  DFSDM2_IRQn                 = 63,     /*!< SD Filter 2 global Interrupt                                      */
  COMP_IRQn                   = 64,     /*!< COMP1 and COMP2 Interrupts                                        */
  LPTIM1_IRQn                 = 65,     /*!< LP TIM1 interrupt                                                 */
  LPTIM2_IRQn                 = 66,     /*!< LP TIM2 interrupt                                                 */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Channel6_IRQn          = 68,     /*!< DMA2 Channel 6 global interrupt                                   */
  DMA2_Channel7_IRQn          = 69,     /*!< DMA2 Channel 7 global interrupt                                   */
  LPUART1_IRQn                = 70,     /*!< LP UART1 interrupt                                                */
  QUADSPI_IRQn                = 71,     /*!< Quad SPI global interrupt                                         */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  SAI1_IRQn                   = 74,     /*!< Serial Audio Interface 1 global interrupt                         */
  SAI2_IRQn                   = 75,     /*!< Serial Audio Interface 2 global interrupt                         */
  SWPMI1_IRQn                 = 76,     /*!< Serial Wire Interface 1 global interrupt                          */
  TSC_IRQn                    = 77,     /*!< Touch Sense Controller global interrupt                           */
  LCD_IRQn                    = 78,     /*!< LCD global interrupt                                              */
  RNG_IRQn                    = 80,     /*!< RNG global interrupt                                              */
  FPU_IRQn                    = 81      /*!< FPU global interrupt                                              */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm4.h"             /* Cortex-M4 processor and core peripherals */
#include "system_stm32l4xx.h"
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */

typedef struct
{
  __IO uint32_t ISR;              /*!< ADC Interrupt and Status Register,                 Address offset: 0x00 */
  __IO uint32_t IER;              /*!< ADC Interrupt Enable Register,                     Address offset: 0x04 */
  __IO uint32_t CR;               /*!< ADC control register,                              Address offset: 0x08 */
  __IO uint32_t CFGR;             /*!< ADC Configuration register,                        Address offset: 0x0C */
  __IO uint32_t CFGR2;            /*!< ADC Configuration register 2,                      Address offset: 0x10 */
  __IO uint32_t SMPR1;            /*!< ADC sample time register 1,                        Address offset: 0x14 */
  __IO uint32_t SMPR2;            /*!< ADC sample time register 2,                        Address offset: 0x18 */
  uint32_t      RESERVED1;        /*!< Reserved, 0x01C                                                         */
  __IO uint32_t TR1;              /*!< ADC watchdog threshold register 1,                 Address offset: 0x20 */
  __IO uint32_t TR2;              /*!< ADC watchdog threshold register 2,                 Address offset: 0x24 */
  __IO uint32_t TR3;              /*!< ADC watchdog threshold register 3,                 Address offset: 0x28 */
  uint32_t      RESERVED2;        /*!< Reserved, 0x02C                                                         */
  __IO uint32_t SQR1;             /*!< ADC regular sequence register 1,                   Address offset: 0x30 */
  __IO uint32_t SQR2;             /*!< ADC regular sequence register 2,                   Address offset: 0x34 */
  __IO uint32_t SQR3;             /*!< ADC regular sequence register 3,                   Address offset: 0x38 */
  __IO uint32_t SQR4;             /*!< ADC regular sequence register 4,                   Address offset: 0x3C */
  __IO uint32_t DR;               /*!< ADC regular data register,                         Address offset: 0x40 */
  uint32_t      RESERVED3;        /*!< Reserved, 0x044                                                         */
  uint32_t      RESERVED4;        /*!< Reserved, 0x048                                                         */
  __IO uint32_t JSQR;             /*!< ADC injected sequence register,                    Address offset: 0x4C */
  uint32_t      RESERVED5[4];     /*!< Reserved, 0x050 - 0x05C                                                 */
  __IO uint32_t OFR1;             /*!< ADC offset register 1,                             Address offset: 0x60 */
  __IO uint32_t OFR2;             /*!< ADC offset register 2,                             Address offset: 0x64 */
  __IO uint32_t OFR3;             /*!< ADC offset register 3,                             Address offset: 0x68 */
  __IO uint32_t OFR4;             /*!< ADC offset register 4,                             Address offset: 0x6C */
  uint32_t      RESERVED6[4];     /*!< Reserved, 0x070 - 0x07C                                                 */
  __IO uint32_t JDR1;             /*!< ADC injected data register 1,                      Address offset: 0x80 */
  __IO uint32_t JDR2;             /*!< ADC injected data register 2,                      Address offset: 0x84 */
  __IO uint32_t JDR3;             /*!< ADC injected data register 3,                      Address offset: 0x88 */
  __IO uint32_t JDR4;             /*!< ADC injected data register 4,                      Address offset: 0x8C */
  uint32_t      RESERVED7[4];     /*!< Reserved, 0x090 - 0x09C                                                 */
  __IO uint32_t AWD2CR;           /*!< ADC  Analog Watchdog 2 Configuration Register,     Address offset: 0xA0 */
  __IO uint32_t AWD3CR;           /*!< ADC  Analog Watchdog 3 Configuration Register,     Address offset: 0xA4 */
  uint32_t      RESERVED8;        /*!< Reserved, 0x0A8                                                         */
  uint32_t      RESERVED9;        /*!< Reserved, 0x0AC                                                         */
  __IO uint32_t DIFSEL;           /*!< ADC  Differential Mode Selection Register,         Address offset: 0xB0 */
  __IO uint32_t CALFACT;          /*!< ADC  Calibration Factors,                          Address offset: 0xB4 */

} ADC_TypeDef;

typedef struct
{
  __IO uint32_t CSR;              /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300   */
  uint32_t      RESERVED;         /*!< Reserved, ADC1 base address + 0x304                                                      */
  __IO uint32_t CCR;              /*!< ADC common control register,                 Address offset: ADC1 base address + 0x308   */
  __IO uint32_t CDR;              /*!< ADC common regular data register for dual    Address offset: ADC1 base address + 0x30C   */
} ADC_Common_TypeDef;


/**
  * @brief Controller Area Network TxMailBox
  */

typedef struct
{
  __IO uint32_t TIR;  /*!< CAN TX mailbox identifier register */
  __IO uint32_t TDTR; /*!< CAN mailbox data length control and time stamp register */
  __IO uint32_t TDLR; /*!< CAN mailbox data low register */
  __IO uint32_t TDHR; /*!< CAN mailbox data high register */
} CAN_TxMailBox_TypeDef;

/**
  * @brief Controller Area Network FIFOMailBox
  */

typedef struct
{
  __IO uint32_t RIR;  /*!< CAN receive FIFO mailbox identifier register */
  __IO uint32_t RDTR; /*!< CAN receive FIFO mailbox data length control and time stamp register */
  __IO uint32_t RDLR; /*!< CAN receive FIFO mailbox data low register */
  __IO uint32_t RDHR; /*!< CAN receive FIFO mailbox data high register */
} CAN_FIFOMailBox_TypeDef;

/**
  * @brief Controller Area Network FilterRegister
  */

typedef struct
{
  __IO uint32_t FR1; /*!< CAN Filter bank register 1 */
  __IO uint32_t FR2; /*!< CAN Filter bank register 1 */
} CAN_FilterRegister_TypeDef;

/**
  * @brief Controller Area Network
  */

typedef struct
{
  __IO uint32_t              MCR;                 /*!< CAN master control register,         Address offset: 0x00          */
  __IO uint32_t              MSR;                 /*!< CAN master status register,          Address offset: 0x04          */
  __IO uint32_t              TSR;                 /*!< CAN transmit status register,        Address offset: 0x08          */
  __IO uint32_t              RF0R;                /*!< CAN receive FIFO 0 register,         Address offset: 0x0C          */
  __IO uint32_t              RF1R;                /*!< CAN receive FIFO 1 register,         Address offset: 0x10          */
  __IO uint32_t              IER;                 /*!< CAN interrupt enable register,       Address offset: 0x14          */
  __IO uint32_t              ESR;                 /*!< CAN error status register,           Address offset: 0x18          */
  __IO uint32_t              BTR;                 /*!< CAN bit timing register,             Address offset: 0x1C          */
  uint32_t                   RESERVED0[88];       /*!< Reserved, 0x020 - 0x17F                                            */
  CAN_TxMailBox_TypeDef      sTxMailBox[3];       /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];     /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
  uint32_t                   RESERVED1[12];       /*!< Reserved, 0x1D0 - 0x1FF                                            */
  __IO uint32_t              FMR;                 /*!< CAN filter master register,          Address offset: 0x200         */
  __IO uint32_t              FM1R;                /*!< CAN filter mode register,            Address offset: 0x204         */
  uint32_t                   RESERVED2;           /*!< Reserved, 0x208                                                    */
  __IO uint32_t              FS1R;                /*!< CAN filter scale register,           Address offset: 0x20C         */
  uint32_t                   RESERVED3;           /*!< Reserved, 0x210                                                    */
  __IO uint32_t              FFA1R;               /*!< CAN filter FIFO assignment register, Address offset: 0x214         */
  uint32_t                   RESERVED4;           /*!< Reserved, 0x218                                                    */
  __IO uint32_t              FA1R;                /*!< CAN filter activation register,      Address offset: 0x21C         */
  uint32_t                   RESERVED5[8];        /*!< Reserved, 0x220-0x23F                                              */
  CAN_FilterRegister_TypeDef sFilterRegister[28]; /*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
} CAN_TypeDef;


/**
  * @brief Comparator
  */

typedef struct
{
  __IO uint32_t CSR;         /*!< COMP comparator control and status register, Address offset: 0x00 */
} COMP_TypeDef;


/**
  * @brief CRC calculation unit
  */

typedef struct
{
  __IO uint32_t DR;          /*!< CRC Data register,                           Address offset: 0x00 */
  __IO uint8_t  IDR;         /*!< CRC Independent data register,               Address offset: 0x04 */
  uint8_t       RESERVED0;   /*!< Reserved,                                                    0x05 */
  uint16_t      RESERVED1;   /*!< Reserved,                                                    0x06 */
  __IO uint32_t CR;          /*!< CRC Control register,                        Address offset: 0x08 */
  uint32_t      RESERVED2;   /*!< Reserved,                                                    0x0C */
  __IO uint32_t INIT;        /*!< Initial CRC value register,                  Address offset: 0x10 */
  __IO uint32_t POL;         /*!< CRC polynomial register,                     Address offset: 0x14 */
} CRC_TypeDef;

/**
  * @brief Digital to Analog Converter
  */

typedef struct
{
  __IO uint32_t CR;          /*!< DAC control register,                                    Address offset: 0x00 */
  __IO uint32_t SWTRIGR;     /*!< DAC software trigger register,                           Address offset: 0x04 */
  __IO uint32_t DHR12R1;     /*!< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08 */
  __IO uint32_t DHR12L1;     /*!< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C */
  __IO uint32_t DHR8R1;      /*!< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10 */
  __IO uint32_t DHR12R2;     /*!< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14 */
  __IO uint32_t DHR12L2;     /*!< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18 */
  __IO uint32_t DHR8R2;      /*!< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C */
  __IO uint32_t DHR12RD;     /*!< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20 */
  __IO uint32_t DHR12LD;     /*!< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24 */
  __IO uint32_t DHR8RD;      /*!< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28 */
  __IO uint32_t DOR1;        /*!< DAC channel1 data output register,                       Address offset: 0x2C */
  __IO uint32_t DOR2;        /*!< DAC channel2 data output register,                       Address offset: 0x30 */
  __IO uint32_t SR;          /*!< DAC status register,                                     Address offset: 0x34 */
  __IO uint32_t CCR;         /*!< DAC calibration control register,                        Address offset: 0x38 */
  __IO uint32_t MCR;         /*!< DAC mode control register,                               Address offset: 0x3C */
  __IO uint32_t SHSR1;       /*!< DAC Sample and Hold sample time register 1,              Address offset: 0x40 */
  __IO uint32_t SHSR2;       /*!< DAC Sample and Hold sample time register 2,              Address offset: 0x44 */
  __IO uint32_t SHHR;        /*!< DAC Sample and Hold hold time register,                  Address offset: 0x48 */
  __IO uint32_t SHRR;        /*!< DAC Sample and Hold refresh time register,               Address offset: 0x4C */
} DAC_TypeDef;

/**
  * @brief DFSDM module registers
  */
typedef struct
{
  __IO uint32_t CR1;         /*!< DFSDM control register1,                          Address offset: 0x100 */
  __IO uint32_t CR2;         /*!< DFSDM control register2,                          Address offset: 0x104 */
  __IO uint32_t ISR;         /*!< DFSDM interrupt and status register,              Address offset: 0x108 */
  __IO uint32_t ICR;         /*!< DFSDM interrupt flag clear register,              Address offset: 0x10C */
  __IO uint32_t JCHGR;       /*!< DFSDM injected channel group selection register,  Address offset: 0x110 */
  __IO uint32_t FCR;         /*!< DFSDM filter control register,                    Address offset: 0x114 */
  __IO uint32_t JDATAR;      /*!< DFSDM data register for injected group,           Address offset: 0x118 */
  __IO uint32_t RDATAR;      /*!< DFSDM data register for regular group,            Address offset: 0x11C */
  __IO uint32_t AWHTR;       /*!< DFSDM analog watchdog high threshold register,    Address offset: 0x120 */
  __IO uint32_t AWLTR;       /*!< DFSDM analog watchdog low threshold register,     Address offset: 0x124 */
  __IO uint32_t AWSR;        /*!< DFSDM analog watchdog status register             Address offset: 0x128 */
  __IO uint32_t AWCFR;       /*!< DFSDM analog watchdog clear flag register         Address offset: 0x12C */
  __IO uint32_t EXMAX;       /*!< DFSDM extreme detector maximum register,          Address offset: 0x130 */
  __IO uint32_t EXMIN;       /*!< DFSDM extreme detector minimum register           Address offset: 0x134 */
  __IO uint32_t CNVTIMR;     /*!< DFSDM conversion timer,                           Address offset: 0x138 */
} DFSDM_Filter_TypeDef;

/**
  * @brief DFSDM channel configuration registers
  */
typedef struct
{
  __IO uint32_t CHCFGR1;     /*!< DFSDM channel configuration register1,            Address offset: 0x00 */
  __IO uint32_t CHCFGR2;     /*!< DFSDM channel configuration register2,            Address offset: 0x04 */
  __IO uint32_t AWSCDR;      /*!< DFSDM channel analog watchdog and
                                  short circuit detector register,                  Address offset: 0x08 */
  __IO uint32_t CHWDATAR;    /*!< DFSDM channel watchdog filter data register,      Address offset: 0x0C */
  __IO uint32_t CHDATINR;    /*!< DFSDM channel data input register,                Address offset: 0x10 */
} DFSDM_Channel_TypeDef;

/**
  * @brief Debug MCU
  */

typedef struct
{
  __IO uint32_t IDCODE;      /*!< MCU device ID code,                 Address offset: 0x00 */
  __IO uint32_t CR;          /*!< Debug MCU configuration register,   Address offset: 0x04 */
  __IO uint32_t APB1FZR1;    /*!< Debug MCU APB1 freeze register 1,   Address offset: 0x08 */
  __IO uint32_t APB1FZR2;    /*!< Debug MCU APB1 freeze register 2,   Address offset: 0x0C */
  __IO uint32_t APB2FZ;      /*!< Debug MCU APB2 freeze register,     Address offset: 0x10 */
} DBGMCU_TypeDef;


/**
  * @brief DMA Controller
  */

typedef struct
{
  __IO uint32_t CCR;         /*!< DMA channel x configuration register        */
  __IO uint32_t CNDTR;       /*!< DMA channel x number of data register       */
  __IO uint32_t CPAR;        /*!< DMA channel x peripheral address register   */
  __IO uint32_t CMAR;        /*!< DMA channel x memory address register       */
} DMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t ISR;         /*!< DMA interrupt status register,                 Address offset: 0x00 */
  __IO uint32_t IFCR;        /*!< DMA interrupt flag clear register,             Address offset: 0x04 */
} DMA_TypeDef;

typedef struct
{
  __IO uint32_t CSELR;       /*!< DMA option register,                           Address offset: 0x00 */
} DMA_request_TypeDef;


/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  __IO uint32_t IMR1;        /*!< EXTI Interrupt mask register 1,             Address offset: 0x00 */
  __IO uint32_t EMR1;        /*!< EXTI Event mask register 1,                 Address offset: 0x04 */
  __IO uint32_t RTSR1;       /*!< EXTI Rising trigger selection register 1,   Address offset: 0x08 */
  __IO uint32_t FTSR1;       /*!< EXTI Falling trigger selection register 1,  Address offset: 0x0C */
  __IO uint32_t SWIER1;      /*!< EXTI Software interrupt event register 1,   Address offset: 0x10 */
  __IO uint32_t PR1;         /*!< EXTI Pending register 1,                    Address offset: 0x14 */
  uint32_t      RESERVED1;   /*!< Reserved, 0x18                                                   */
  uint32_t      RESERVED2;   /*!< Reserved, 0x1C                                                   */
  __IO uint32_t IMR2;        /*!< EXTI Interrupt mask register 2,             Address offset: 0x20 */
  __IO uint32_t EMR2;        /*!< EXTI Event mask register 2,                 Address offset: 0x24 */
  __IO uint32_t RTSR2;       /*!< EXTI Rising trigger selection register 2,   Address offset: 0x28 */
  __IO uint32_t FTSR2;       /*!< EXTI Falling trigger selection register 2,  Address offset: 0x2C */
  __IO uint32_t SWIER2;      /*!< EXTI Software interrupt event register 2,   Address offset: 0x30 */
  __IO uint32_t PR2;         /*!< EXTI Pending register 2,                    Address offset: 0x34 */
} EXTI_TypeDef;


/**
  * @brief Firewall
  */

typedef struct
{
  __IO uint32_t CSSA;        /*!< Code Segment Start Address register,              Address offset: 0x00 */
  __IO uint32_t CSL;         /*!< Code Segment Length register,                      Address offset: 0x04 */
  __IO uint32_t NVDSSA;      /*!< NON volatile data Segment Start Address register,  Address offset: 0x08 */
  __IO uint32_t NVDSL;       /*!< NON volatile data Segment Length register,         Address offset: 0x0C */
  __IO uint32_t VDSSA ;      /*!< Volatile data Segment Start Address register,      Address offset: 0x10 */
  __IO uint32_t VDSL ;       /*!< Volatile data Segment Length register,             Address offset: 0x14 */
  uint32_t      RESERVED1;   /*!< Reserved1,                                         Address offset: 0x18 */
  uint32_t      RESERVED2;   /*!< Reserved2,                                         Address offset: 0x1C */
  __IO uint32_t CR ;         /*!< Configuration  register,                           Address offset: 0x20 */
} FIREWALL_TypeDef;


/**
  * @brief FLASH Registers
  */

typedef struct
{
  __IO uint32_t ACR;              /*!< FLASH access control register,            Address offset: 0x00 */
  __IO uint32_t PDKEYR;           /*!< FLASH power down key register,            Address offset: 0x04 */
  __IO uint32_t KEYR;             /*!< FLASH key register,                       Address offset: 0x08 */
  __IO uint32_t OPTKEYR;          /*!< FLASH option key register,                Address offset: 0x0C */
  __IO uint32_t SR;               /*!< FLASH status register,                    Address offset: 0x10 */
  __IO uint32_t CR;               /*!< FLASH control register,                   Address offset: 0x14 */
  __IO uint32_t ECCR;             /*!< FLASH ECC register,                       Address offset: 0x18 */
  __IO uint32_t RESERVED1;        /*!< Reserved1,                                Address offset: 0x1C */
  __IO uint32_t OPTR;             /*!< FLASH option register,                    Address offset: 0x20 */
  __IO uint32_t PCROP1SR;         /*!< FLASH bank1 PCROP start address register, Address offset: 0x24 */
  __IO uint32_t PCROP1ER;         /*!< FLASH bank1 PCROP end address register,   Address offset: 0x28 */
  __IO uint32_t WRP1AR;           /*!< FLASH bank1 WRP area A address register,  Address offset: 0x2C */
  __IO uint32_t WRP1BR;           /*!< FLASH bank1 WRP area B address register,  Address offset: 0x30 */
       uint32_t RESERVED2[4];     /*!< Reserved2,                                Address offset: 0x34 */
  __IO uint32_t PCROP2SR;         /*!< FLASH bank2 PCROP start address register, Address offset: 0x44 */
  __IO uint32_t PCROP2ER;         /*!< FLASH bank2 PCROP end address register,   Address offset: 0x48 */
  __IO uint32_t WRP2AR;           /*!< FLASH bank2 WRP area A address register,  Address offset: 0x4C */
  __IO uint32_t WRP2BR;           /*!< FLASH bank2 WRP area B address register,  Address offset: 0x50 */
} FLASH_TypeDef;


/**
  * @brief Flexible Memory Controller
  */

typedef struct
{
  __IO uint32_t BTCR[8];     /*!< NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C */
} FMC_Bank1_TypeDef;

/**
  * @brief Flexible Memory Controller Bank1E
  */

typedef struct
{
  __IO uint32_t BWTR[7];     /*!< NOR/PSRAM write timing registers, Address offset: 0x104-0x11C */
} FMC_Bank1E_TypeDef;

/**
  * @brief Flexible Memory Controller Bank3
  */

typedef struct
{
  __IO uint32_t PCR;        /*!< NAND Flash control register,                       Address offset: 0x80 */
  __IO uint32_t SR;         /*!< NAND Flash FIFO status and interrupt register,     Address offset: 0x84 */
  __IO uint32_t PMEM;       /*!< NAND Flash Common memory space timing register,    Address offset: 0x88 */
  __IO uint32_t PATT;       /*!< NAND Flash Attribute memory space timing register, Address offset: 0x8C */
  uint32_t      RESERVED0;  /*!< Reserved, 0x90                                                            */
  __IO uint32_t ECCR;       /*!< NAND Flash ECC result registers,                   Address offset: 0x94 */
} FMC_Bank3_TypeDef;

/**
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
  __IO uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];      /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  __IO uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */
  __IO uint32_t ASCR;        /*!< GPIO analog switch control register,   Address offset: 0x2C     */

} GPIO_TypeDef;


/**
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< I2C Control register 1,            Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< I2C Control register 2,            Address offset: 0x04 */
  __IO uint32_t OAR1;        /*!< I2C Own address 1 register,        Address offset: 0x08 */
  __IO uint32_t OAR2;        /*!< I2C Own address 2 register,        Address offset: 0x0C */
  __IO uint32_t TIMINGR;     /*!< I2C Timing register,               Address offset: 0x10 */
  __IO uint32_t TIMEOUTR;    /*!< I2C Timeout register,              Address offset: 0x14 */
  __IO uint32_t ISR;         /*!< I2C Interrupt and status register, Address offset: 0x18 */
  __IO uint32_t ICR;         /*!< I2C Interrupt clear register,      Address offset: 0x1C */
  __IO uint32_t PECR;        /*!< I2C PEC register,                  Address offset: 0x20 */
  __IO uint32_t RXDR;        /*!< I2C Receive data register,         Address offset: 0x24 */
  __IO uint32_t TXDR;        /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;

/**
  * @brief Independent WATCHDOG
  */

typedef struct
{
  __IO uint32_t KR;          /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;          /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;         /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;          /*!< IWDG Status register,    Address offset: 0x0C */
  __IO uint32_t WINR;        /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_TypeDef;

/**
  * @brief LCD
  */

typedef struct
{
  __IO uint32_t CR;          /*!< LCD control register,              Address offset: 0x00 */
  __IO uint32_t FCR;         /*!< LCD frame control register,        Address offset: 0x04 */
  __IO uint32_t SR;          /*!< LCD status register,               Address offset: 0x08 */
  __IO uint32_t CLR;         /*!< LCD clear register,                Address offset: 0x0C */
  uint32_t RESERVED;         /*!< Reserved,                          Address offset: 0x10 */
  __IO uint32_t RAM[16];     /*!< LCD display memory,           Address offset: 0x14-0x50 */
} LCD_TypeDef;

/**
  * @brief LPTIMER
  */
typedef struct
{
  __IO uint32_t ISR;         /*!< LPTIM Interrupt and Status register,                Address offset: 0x00 */
  __IO uint32_t ICR;         /*!< LPTIM Interrupt Clear register,                     Address offset: 0x04 */
  __IO uint32_t IER;         /*!< LPTIM Interrupt Enable register,                    Address offset: 0x08 */
  __IO uint32_t CFGR;        /*!< LPTIM Configuration register,                       Address offset: 0x0C */
  __IO uint32_t CR;          /*!< LPTIM Control register,                             Address offset: 0x10 */
  __IO uint32_t CMP;         /*!< LPTIM Compare register,                             Address offset: 0x14 */
  __IO uint32_t ARR;         /*!< LPTIM Autoreload register,                          Address offset: 0x18 */
  __IO uint32_t CNT;         /*!< LPTIM Counter register,                             Address offset: 0x1C */
  __IO uint32_t OR;          /*!< LPTIM Option register,                              Address offset: 0x20 */
} LPTIM_TypeDef;


/**
  * @brief Operational Amplifier (OPAMP)
  */

typedef struct
{
  __IO uint32_t CSR;         /*!< OPAMP control/status register,                     Address offset: 0x00 */
  __IO uint32_t OTR;         /*!< OPAMP offset trimming register for normal mode,    Address offset: 0x04 */
  __IO uint32_t LPOTR;       /*!< OPAMP offset trimming register for low power mode, Address offset: 0x08 */
} OPAMP_TypeDef;


/**
  * @brief Power Control
  */

typedef struct
{
  __IO uint32_t CR1;   /*!< PWR power control register 1,        Address offset: 0x00 */
  __IO uint32_t CR2;   /*!< PWR power control register 2,        Address offset: 0x04 */
  __IO uint32_t CR3;   /*!< PWR power control register 3,        Address offset: 0x08 */
  __IO uint32_t CR4;   /*!< PWR power control register 4,        Address offset: 0x0C */
  __IO uint32_t SR1;   /*!< PWR power status register 1,         Address offset: 0x10 */
  __IO uint32_t SR2;   /*!< PWR power status register 2,         Address offset: 0x14 */
  __IO uint32_t SCR;   /*!< PWR power status reset register,     Address offset: 0x18 */
  uint32_t RESERVED;   /*!< Reserved,                            Address offset: 0x1C */
  __IO uint32_t PUCRA; /*!< Pull_up control register of portA,   Address offset: 0x20 */
  __IO uint32_t PDCRA; /*!< Pull_Down control register of portA, Address offset: 0x24 */
  __IO uint32_t PUCRB; /*!< Pull_up control register of portB,   Address offset: 0x28 */
  __IO uint32_t PDCRB; /*!< Pull_Down control register of portB, Address offset: 0x2C */
  __IO uint32_t PUCRC; /*!< Pull_up control register of portC,   Address offset: 0x30 */
  __IO uint32_t PDCRC; /*!< Pull_Down control register of portC, Address offset: 0x34 */
  __IO uint32_t PUCRD; /*!< Pull_up control register of portD,   Address offset: 0x38 */
  __IO uint32_t PDCRD; /*!< Pull_Down control register of portD, Address offset: 0x3C */
  __IO uint32_t PUCRE; /*!< Pull_up control register of portE,   Address offset: 0x40 */
  __IO uint32_t PDCRE; /*!< Pull_Down control register of portE, Address offset: 0x44 */
  __IO uint32_t PUCRF; /*!< Pull_up control register of portF,   Address offset: 0x48 */
  __IO uint32_t PDCRF; /*!< Pull_Down control register of portF, Address offset: 0x4C */
  __IO uint32_t PUCRG; /*!< Pull_up control register of portG,   Address offset: 0x50 */
  __IO uint32_t PDCRG; /*!< Pull_Down control register of portG, Address offset: 0x54 */
  __IO uint32_t PUCRH; /*!< Pull_up control register of portH,   Address offset: 0x58 */
  __IO uint32_t PDCRH; /*!< Pull_Down control register of portH, Address offset: 0x5C */
} PWR_TypeDef;


/**
  * @brief QUAD Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR;          /*!< QUADSPI Control register,                           Address offset: 0x00 */
  __IO uint32_t DCR;         /*!< QUADSPI Device Configuration register,              Address offset: 0x04 */
  __IO uint32_t SR;          /*!< QUADSPI Status register,                            Address offset: 0x08 */
  __IO uint32_t FCR;         /*!< QUADSPI Flag Clear register,                        Address offset: 0x0C */
  __IO uint32_t DLR;         /*!< QUADSPI Data Length register,                       Address offset: 0x10 */
  __IO uint32_t CCR;         /*!< QUADSPI Communication Configuration register,       Address offset: 0x14 */
  __IO uint32_t AR;          /*!< QUADSPI Address register,                           Address offset: 0x18 */
  __IO uint32_t ABR;         /*!< QUADSPI Alternate Bytes register,                   Address offset: 0x1C */
  __IO uint32_t DR;          /*!< QUADSPI Data register,                              Address offset: 0x20 */
  __IO uint32_t PSMKR;       /*!< QUADSPI Polling Status Mask register,               Address offset: 0x24 */
  __IO uint32_t PSMAR;       /*!< QUADSPI Polling Status Match register,              Address offset: 0x28 */
  __IO uint32_t PIR;         /*!< QUADSPI Polling Interval register,                  Address offset: 0x2C */
  __IO uint32_t LPTR;        /*!< QUADSPI Low Power Timeout register,                 Address offset: 0x30 */
} QUADSPI_TypeDef;


/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;          /*!< RCC clock control register,                                              Address offset: 0x00 */
  __IO uint32_t ICSCR;       /*!< RCC Internal Clock Sources Calibration Register,                         Address offset: 0x04 */
  __IO uint32_t CFGR;        /*!< RCC clock configuration register,                                        Address offset: 0x08 */
  __IO uint32_t PLLCFGR;     /*!< RCC System PLL configuration register,                                   Address offset: 0x0C */
  __IO uint32_t PLLSAI1CFGR; /*!< RCC PLL SAI1 Configuration Register,                                     Address offset: 0x10 */
  __IO uint32_t PLLSAI2CFGR; /*!< RCC PLL SAI2 Configuration Register,                                     Address offset: 0x14 */
  __IO uint32_t CIER;        /*!< RCC Clock Interrupt Enable Register,                                     Address offset: 0x18 */
  __IO uint32_t CIFR;        /*!< RCC Clock Interrupt Flag Register,                                       Address offset: 0x1C */
  __IO uint32_t CICR;        /*!< RCC Clock Interrupt Clear Register,                                      Address offset: 0x20 */
  uint32_t      RESERVED0;   /*!< Reserved,                                                                Address offset: 0x24 */
  __IO uint32_t AHB1RSTR;    /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x28 */
  __IO uint32_t AHB2RSTR;    /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x2C */
  __IO uint32_t AHB3RSTR;    /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x30 */
  uint32_t      RESERVED1;   /*!< Reserved,                                                                Address offset: 0x34 */
  __IO uint32_t APB1RSTR1;   /*!< RCC LowSpeed APB1 macrocells resets Low Word,                            Address offset: 0x38 */
  __IO uint32_t APB1RSTR2;   /*!< RCC LowSpeed APB1 macrocells resets High Word,                           Address offset: 0x3C */
  __IO uint32_t APB2RSTR;    /*!< RCC High Speed APB macrocells resets,                                    Address offset: 0x40 */
  uint32_t      RESERVED2;   /*!< Reserved,                                                                Address offset: 0x44 */
  __IO uint32_t AHB1ENR;     /*!< RCC AHB1 peripheral clock enable register,                               Address offset: 0x48 */
  __IO uint32_t AHB2ENR;     /*!< RCC AHB2 peripheral clock enable register,                               Address offset: 0x4C */
  __IO uint32_t AHB3ENR;     /*!< RCC AHB3 peripheral clock enable register,                               Address offset: 0x50 */
  uint32_t      RESERVED3;   /*!< Reserved,                                                                Address offset: 0x54 */
  __IO uint32_t APB1ENR1;    /*!< RCC LowSpeed APB1 macrocells clock enables Low Word,                     Address offset: 0x58 */
  __IO uint32_t APB1ENR2;    /*!< RCC LowSpeed APB1 macrocells clock enables High Word,                    Address offset: 0x5C */
  __IO uint32_t APB2ENR;     /*!< RCC High Speed APB macrocells clock enabled,                             Address offset: 0x60 */
  uint32_t      RESERVED4;   /*!< Reserved,                                                                Address offset: 0x64 */
  __IO uint32_t AHB1SMENR;   /*!< RCC AHB1 macrocells clocks enables in sleep mode,                        Address offset: 0x60 */
  __IO uint32_t AHB2SMENR;   /*!< RCC AHB2 macrocells clock enables in sleep mode,                         Address offset: 0x64 */
  __IO uint32_t AHB3SMENR;   /*!< RCC AHB3 macrocells clock enables in sleep mode,                         Address offset: 0x70 */
  uint32_t      RESERVED5;   /*!< Reserved,                                                                Address offset: 0x74 */
  __IO uint32_t APB1SMENR1;  /*!< RCC LowSpeed APB1 macrocells clock enables in sleep mode Low Word,       Address offset: 0x78 */
  __IO uint32_t APB1SMENR2;  /*!< RCC LowSpeed APB1 macrocells clock enables in sleep mode High Word,      Address offset: 0x7C */
  __IO uint32_t APB2SMENR;   /*!< RCC High Speed APB macrocells clock enabled in sleep mode,               Address offset: 0x80 */
  uint32_t      RESERVED6;   /*!< Reserved,                                                                Address offset: 0x84 */
  __IO uint32_t CCIPR;       /*!< RCC IPs Clocks Configuration Register,                                   Address offset: 0x88 */
  __IO uint32_t RESERVED7;   /*!< Reserved,                                                                Address offset: 0x8C */
  __IO uint32_t BDCR;        /*!< RCC Vswitch Backup Domain Control Register,                              Address offset: 0x90 */
  __IO uint32_t CSR;         /*!< RCC clock control & status register,                                     Address offset: 0x94 */
} RCC_TypeDef;

/**
  * @brief Real-Time Clock
  */

typedef struct
{
  __IO uint32_t TR;          /*!< RTC time register,                                         Address offset: 0x00 */
  __IO uint32_t DR;          /*!< RTC date register,                                         Address offset: 0x04 */
  __IO uint32_t CR;          /*!< RTC control register,                                      Address offset: 0x08 */
  __IO uint32_t ISR;         /*!< RTC initialization and status register,                    Address offset: 0x0C */
  __IO uint32_t PRER;        /*!< RTC prescaler register,                                    Address offset: 0x10 */
  __IO uint32_t WUTR;        /*!< RTC wakeup timer register,                                 Address offset: 0x14 */
       uint32_t reserved;    /*!< Reserved  */
  __IO uint32_t ALRMAR;      /*!< RTC alarm A register,                                      Address offset: 0x1C */
  __IO uint32_t ALRMBR;      /*!< RTC alarm B register,                                      Address offset: 0x20 */
  __IO uint32_t WPR;         /*!< RTC write protection register,                             Address offset: 0x24 */
  __IO uint32_t SSR;         /*!< RTC sub second register,                                   Address offset: 0x28 */
  __IO uint32_t SHIFTR;      /*!< RTC shift control register,                                Address offset: 0x2C */
  __IO uint32_t TSTR;        /*!< RTC time stamp time register,                              Address offset: 0x30 */
  __IO uint32_t TSDR;        /*!< RTC time stamp date register,                              Address offset: 0x34 */
  __IO uint32_t TSSSR;       /*!< RTC time-stamp sub second register,                        Address offset: 0x38 */
  __IO uint32_t CALR;        /*!< RTC calibration register,                                  Address offset: 0x3C */
  __IO uint32_t TAMPCR;      /*!< RTC tamper configuration register,                         Address offset: 0x40 */
  __IO uint32_t ALRMASSR;    /*!< RTC alarm A sub second register,                           Address offset: 0x44 */
  __IO uint32_t ALRMBSSR;    /*!< RTC alarm B sub second register,                           Address offset: 0x48 */
  __IO uint32_t OR;          /*!< RTC option register,                                       Address offset: 0x4C */
  __IO uint32_t BKP0R;       /*!< RTC backup register 0,                                     Address offset: 0x50 */
  __IO uint32_t BKP1R;       /*!< RTC backup register 1,                                     Address offset: 0x54 */
  __IO uint32_t BKP2R;       /*!< RTC backup register 2,                                     Address offset: 0x58 */
  __IO uint32_t BKP3R;       /*!< RTC backup register 3,                                     Address offset: 0x5C */
  __IO uint32_t BKP4R;       /*!< RTC backup register 4,                                     Address offset: 0x60 */
  __IO uint32_t BKP5R;       /*!< RTC backup register 5,                                     Address offset: 0x64 */
  __IO uint32_t BKP6R;       /*!< RTC backup register 6,                                     Address offset: 0x68 */
  __IO uint32_t BKP7R;       /*!< RTC backup register 7,                                     Address offset: 0x6C */
  __IO uint32_t BKP8R;       /*!< RTC backup register 8,                                     Address offset: 0x70 */
  __IO uint32_t BKP9R;       /*!< RTC backup register 9,                                     Address offset: 0x74 */
  __IO uint32_t BKP10R;      /*!< RTC backup register 10,                                    Address offset: 0x78 */
  __IO uint32_t BKP11R;      /*!< RTC backup register 11,                                    Address offset: 0x7C */
  __IO uint32_t BKP12R;      /*!< RTC backup register 12,                                    Address offset: 0x80 */
  __IO uint32_t BKP13R;      /*!< RTC backup register 13,                                    Address offset: 0x84 */
  __IO uint32_t BKP14R;      /*!< RTC backup register 14,                                    Address offset: 0x88 */
  __IO uint32_t BKP15R;      /*!< RTC backup register 15,                                    Address offset: 0x8C */
  __IO uint32_t BKP16R;      /*!< RTC backup register 16,                                    Address offset: 0x90 */
  __IO uint32_t BKP17R;      /*!< RTC backup register 17,                                    Address offset: 0x94 */
  __IO uint32_t BKP18R;      /*!< RTC backup register 18,                                    Address offset: 0x98 */
  __IO uint32_t BKP19R;      /*!< RTC backup register 19,                                    Address offset: 0x9C */
  __IO uint32_t BKP20R;      /*!< RTC backup register 20,                                    Address offset: 0xA0 */
  __IO uint32_t BKP21R;      /*!< RTC backup register 21,                                    Address offset: 0xA4 */
  __IO uint32_t BKP22R;      /*!< RTC backup register 22,                                    Address offset: 0xA8 */
  __IO uint32_t BKP23R;      /*!< RTC backup register 23,                                    Address offset: 0xAC */
  __IO uint32_t BKP24R;      /*!< RTC backup register 24,                                    Address offset: 0xB0 */
  __IO uint32_t BKP25R;      /*!< RTC backup register 25,                                    Address offset: 0xB4 */
  __IO uint32_t BKP26R;      /*!< RTC backup register 26,                                    Address offset: 0xB8 */
  __IO uint32_t BKP27R;      /*!< RTC backup register 27,                                    Address offset: 0xBC */
  __IO uint32_t BKP28R;      /*!< RTC backup register 28,                                    Address offset: 0xC0 */
  __IO uint32_t BKP29R;      /*!< RTC backup register 29,                                    Address offset: 0xC4 */
  __IO uint32_t BKP30R;      /*!< RTC backup register 30,                                    Address offset: 0xC8 */
  __IO uint32_t BKP31R;      /*!< RTC backup register 31,                                    Address offset: 0xCC */
} RTC_TypeDef;


/**
  * @brief Serial Audio Interface
  */

typedef struct
{
  __IO uint32_t GCR;         /*!< SAI global configuration register,        Address offset: 0x00 */
} SAI_TypeDef;

typedef struct
{
  __IO uint32_t CR1;         /*!< SAI block x configuration register 1,     Address offset: 0x04 */
  __IO uint32_t CR2;         /*!< SAI block x configuration register 2,     Address offset: 0x08 */
  __IO uint32_t FRCR;        /*!< SAI block x frame configuration register, Address offset: 0x0C */
  __IO uint32_t SLOTR;       /*!< SAI block x slot register,                Address offset: 0x10 */
  __IO uint32_t IMR;         /*!< SAI block x interrupt mask register,      Address offset: 0x14 */
  __IO uint32_t SR;          /*!< SAI block x status register,              Address offset: 0x18 */
  __IO uint32_t CLRFR;       /*!< SAI block x clear flag register,          Address offset: 0x1C */
  __IO uint32_t DR;          /*!< SAI block x data register,                Address offset: 0x20 */
} SAI_Block_TypeDef;


/**
  * @brief Secure digital input/output Interface
  */

typedef struct
{
  __IO uint32_t POWER;          /*!< SDMMC power control register,    Address offset: 0x00 */
  __IO uint32_t CLKCR;          /*!< SDMMC clock control register,    Address offset: 0x04 */
  __IO uint32_t ARG;            /*!< SDMMC argument register,         Address offset: 0x08 */
  __IO uint32_t CMD;            /*!< SDMMC command register,          Address offset: 0x0C */
  __I uint32_t  RESPCMD;        /*!< SDMMC command response register, Address offset: 0x10 */
  __I uint32_t  RESP1;          /*!< SDMMC response 1 register,       Address offset: 0x14 */
  __I uint32_t  RESP2;          /*!< SDMMC response 2 register,       Address offset: 0x18 */
  __I uint32_t  RESP3;          /*!< SDMMC response 3 register,       Address offset: 0x1C */
  __I uint32_t  RESP4;          /*!< SDMMC response 4 register,       Address offset: 0x20 */
  __IO uint32_t DTIMER;         /*!< SDMMC data timer register,       Address offset: 0x24 */
  __IO uint32_t DLEN;           /*!< SDMMC data length register,      Address offset: 0x28 */
  __IO uint32_t DCTRL;          /*!< SDMMC data control register,     Address offset: 0x2C */
  __I uint32_t  DCOUNT;         /*!< SDMMC data counter register,     Address offset: 0x30 */
  __I uint32_t  STA;            /*!< SDMMC status register,           Address offset: 0x34 */
  __IO uint32_t ICR;            /*!< SDMMC interrupt clear register,  Address offset: 0x38 */
  __IO uint32_t MASK;           /*!< SDMMC mask register,             Address offset: 0x3C */
  uint32_t      RESERVED0[2];   /*!< Reserved, 0x40-0x44                                  */
  __I uint32_t  FIFOCNT;        /*!< SDMMC FIFO counter register,     Address offset: 0x48 */
  uint32_t      RESERVED1[13];  /*!< Reserved, 0x4C-0x7C                                  */
  __IO uint32_t FIFO;           /*!< SDMMC data FIFO register,        Address offset: 0x80 */
} SDMMC_TypeDef;


/**
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< SPI Control register 1,                              Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< SPI Control register 2,                              Address offset: 0x04 */
  __IO uint32_t SR;          /*!< SPI Status register,                                 Address offset: 0x08 */
  __IO uint32_t DR;          /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;       /*!< SPI CRC polynomial register,                         Address offset: 0x10 */
  __IO uint32_t RXCRCR;      /*!< SPI Rx CRC register,                                 Address offset: 0x14 */
  __IO uint32_t TXCRCR;      /*!< SPI Tx CRC register,                                 Address offset: 0x18 */
  uint32_t  RESERVED1;       /*!< Reserved,                                            Address offset: 0x1C */
  uint32_t  RESERVED2;       /*!< Reserved,                                            Address offset: 0x20 */
} SPI_TypeDef;


/**
  * @brief Single Wire Protocol Master Interface SPWMI
  */

typedef struct
{
  __IO uint32_t CR;          /*!< SWPMI Configuration/Control register,     Address offset: 0x00 */
  __IO uint32_t BRR;         /*!< SWPMI bitrate register,                   Address offset: 0x04 */
    uint32_t  RESERVED1;     /*!< Reserved, 0x08                                                 */
  __IO uint32_t ISR;         /*!< SWPMI Interrupt and Status register,      Address offset: 0x0C */
  __IO uint32_t ICR;         /*!< SWPMI Interrupt Flag Clear register,      Address offset: 0x10 */
  __IO uint32_t IER;         /*!< SWPMI Interrupt Enable register,          Address offset: 0x14 */
  __IO uint32_t RFL;         /*!< SWPMI Receive Frame Length register,      Address offset: 0x18 */
  __IO uint32_t TDR;         /*!< SWPMI Transmit data register,             Address offset: 0x1C */
  __IO uint32_t RDR;         /*!< SWPMI Receive data register,              Address offset: 0x20 */
  __IO uint32_t OR;          /*!< SWPMI Option register,                    Address offset: 0x24 */
} SWPMI_TypeDef;


/**
  * @brief System configuration controller
  */

typedef struct
{
  __IO uint32_t MEMRMP;      /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  __IO uint32_t CFGR1;       /*!< SYSCFG configuration register 1,                   Address offset: 0x04      */
  __IO uint32_t EXTICR[4];   /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  __IO uint32_t SCSR;        /*!< SYSCFG SRAM2 control and status register,          Address offset: 0x18      */
  __IO uint32_t CFGR2;       /*!< SYSCFG configuration register 2,                   Address offset: 0x1C      */
  __IO uint32_t SWPR;        /*!< SYSCFG SRAM2 write protection register,            Address offset: 0x20      */
  __IO uint32_t SKR;         /*!< SYSCFG SRAM2 key register,                         Address offset: 0x24      */
} SYSCFG_TypeDef;


/**
  * @brief TIM
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,                   Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,                   Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,          Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                      Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,            Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register,      Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                     Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler,                            Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,                 Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,          Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,           Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,           Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,           Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,           Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,         Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIM DMA control register,                 Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
  __IO uint32_t OR1;         /*!< TIM option register 1,                    Address offset: 0x50 */
  __IO uint32_t CCMR3;       /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
  __IO uint32_t CCR5;        /*!< TIM capture/compare register5,            Address offset: 0x58 */
  __IO uint32_t CCR6;        /*!< TIM capture/compare register6,            Address offset: 0x5C */
  __IO uint32_t OR2;         /*!< TIM option register 2,                    Address offset: 0x60 */
  __IO uint32_t OR3;         /*!< TIM option register 3,                    Address offset: 0x64 */
} TIM_TypeDef;


/**
  * @brief Touch Sensing Controller (TSC)
  */

typedef struct
{
  __IO uint32_t CR;            /*!< TSC control register,                                     Address offset: 0x00 */
  __IO uint32_t IER;           /*!< TSC interrupt enable register,                            Address offset: 0x04 */
  __IO uint32_t ICR;           /*!< TSC interrupt clear register,                             Address offset: 0x08 */
  __IO uint32_t ISR;           /*!< TSC interrupt status register,                            Address offset: 0x0C */
  __IO uint32_t IOHCR;         /*!< TSC I/O hysteresis control register,                      Address offset: 0x10 */
  uint32_t      RESERVED1;     /*!< Reserved,                                                 Address offset: 0x14 */
  __IO uint32_t IOASCR;        /*!< TSC I/O analog switch control register,                   Address offset: 0x18 */
  uint32_t      RESERVED2;     /*!< Reserved,                                                 Address offset: 0x1C */
  __IO uint32_t IOSCR;         /*!< TSC I/O sampling control register,                        Address offset: 0x20 */
  uint32_t      RESERVED3;     /*!< Reserved,                                                 Address offset: 0x24 */
  __IO uint32_t IOCCR;         /*!< TSC I/O channel control register,                         Address offset: 0x28 */
  uint32_t      RESERVED4;     /*!< Reserved,                                                 Address offset: 0x2C */
  __IO uint32_t IOGCSR;        /*!< TSC I/O group control status register,                    Address offset: 0x30 */
  __IO uint32_t IOGXCR[8];     /*!< TSC I/O group x counter register,                         Address offset: 0x34-50 */
} TSC_TypeDef;


/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< USART Control register 1,                 Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< USART Control register 2,                 Address offset: 0x04 */
  __IO uint32_t CR3;         /*!< USART Control register 3,                 Address offset: 0x08 */
  __IO uint32_t BRR;         /*!< USART Baud rate register,                 Address offset: 0x0C */
  __IO uint16_t GTPR;        /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
  uint16_t  RESERVED2;       /*!< Reserved, 0x12                                                 */
  __IO uint32_t RTOR;        /*!< USART Receiver Time Out register,         Address offset: 0x14 */
  __IO uint16_t RQR;         /*!< USART Request register,                   Address offset: 0x18 */
  uint16_t  RESERVED3;       /*!< Reserved, 0x1A                                                 */
  __IO uint32_t ISR;         /*!< USART Interrupt and status register,      Address offset: 0x1C */
  __IO uint32_t ICR;         /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
  __IO uint16_t RDR;         /*!< USART Receive Data register,              Address offset: 0x24 */
  uint16_t  RESERVED4;       /*!< Reserved, 0x26                                                 */
  __IO uint16_t TDR;         /*!< USART Transmit Data register,             Address offset: 0x28 */
  uint16_t  RESERVED5;       /*!< Reserved, 0x2A                                                 */
} USART_TypeDef;


/**
  * @brief VREFBUF
  */

typedef struct
{
  __IO uint32_t CSR;         /*!< VREFBUF control and status register,         Address offset: 0x00 */
  __IO uint32_t CCR;         /*!< VREFBUF calibration and control register,    Address offset: 0x04 */
} VREFBUF_TypeDef;

/**
  * @brief Window WATCHDOG
  */

typedef struct
{
  __IO uint32_t CR;          /*!< WWDG Control register,       Address offset: 0x00 */
  __IO uint32_t CFR;         /*!< WWDG Configuration register, Address offset: 0x04 */
  __IO uint32_t SR;          /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;



/**
  * @brief RNG
  */

typedef struct
{
  __IO uint32_t CR;  /*!< RNG control register, Address offset: 0x00 */
  __IO uint32_t SR;  /*!< RNG status register,  Address offset: 0x04 */
  __IO uint32_t DR;  /*!< RNG data register,    Address offset: 0x08 */
} RNG_TypeDef;

/** 
  * @brief USB_OTG_Core_register
  */
typedef struct
{
  __IO uint32_t GOTGCTL;              /*!<  USB_OTG Control and Status Register          000h*/
  __IO uint32_t GOTGINT;              /*!<  USB_OTG Interrupt Register                   004h*/
  __IO uint32_t GAHBCFG;              /*!<  Core AHB Configuration Register              008h*/
  __IO uint32_t GUSBCFG;              /*!<  Core USB Configuration Register              00Ch*/
  __IO uint32_t GRSTCTL;              /*!<  Core Reset Register                          010h*/
  __IO uint32_t GINTSTS;              /*!<  Core Interrupt Register                      014h*/
  __IO uint32_t GINTMSK;              /*!<  Core Interrupt Mask Register                 018h*/
  __IO uint32_t GRXSTSR;              /*!<  Receive Sts Q Read Register                  01Ch*/
  __IO uint32_t GRXSTSP;              /*!<  Receive Sts Q Read & POP Register            020h*/
  __IO uint32_t GRXFSIZ;              /* Receive FIFO Size Register                      024h*/
  __IO uint32_t DIEPTXF0_HNPTXFSIZ;   /*!<  EP0 / Non Periodic Tx FIFO Size Register     028h*/
  __IO uint32_t HNPTXSTS;             /*!<  Non Periodic Tx FIFO/Queue Sts reg           02Ch*/
  uint32_t Reserved30[2];             /* Reserved                                        030h*/
  __IO uint32_t GCCFG;                /* General Purpose IO Register                     038h*/
  __IO uint32_t CID;                  /* User ID Register                                03Ch*/
  uint32_t  Reserved5[3];             /* Reserved                                        040h-048h*/
  __IO uint32_t GHWCFG3;              /* User HW config3                                 04Ch*/
  uint32_t  Reserved6;                /* Reserved                                        050h*/ 
  __IO uint32_t GLPMCFG;              /* LPM Register                                    054h*/
  __IO uint32_t GPWRDN;               /* Power Down Register                             058h*/
  __IO uint32_t GDFIFOCFG;            /* DFIFO Software Config Register                  05Ch*/
   __IO uint32_t GADPCTL;             /* ADP Timer, Control and Status Register          60Ch*/
    uint32_t  Reserved43[39];         /* Reserved                                        058h-0FFh*/
  __IO uint32_t HPTXFSIZ;             /* Host Periodic Tx FIFO Size Reg                  100h*/
  __IO uint32_t DIEPTXF[0x0F];        /* dev Periodic Transmit FIFO */
} USB_OTG_GlobalTypeDef;

/** 
  * @brief USB_OTG_device_Registers
  */
typedef struct 
{
  __IO uint32_t DCFG;        /* dev Configuration Register   800h*/
  __IO uint32_t DCTL;        /* dev Control Register         804h*/
  __IO uint32_t DSTS;        /* dev Status Register (RO)     808h*/
  uint32_t Reserved0C;       /* Reserved                     80Ch*/
  __IO uint32_t DIEPMSK;     /* dev IN Endpoint Mask         810h*/
  __IO uint32_t DOEPMSK;     /* dev OUT Endpoint Mask        814h*/
  __IO uint32_t DAINT;       /* dev All Endpoints Itr Reg    818h*/
  __IO uint32_t DAINTMSK;    /* dev All Endpoints Itr Mask   81Ch*/
  uint32_t  Reserved20;      /* Reserved                     820h*/
  uint32_t Reserved9;        /* Reserved                     824h*/
  __IO uint32_t DVBUSDIS;    /* dev VBUS discharge Register  828h*/
  __IO uint32_t DVBUSPULSE;  /* dev VBUS Pulse Register      82Ch*/
  __IO uint32_t DTHRCTL;     /* dev thr                      830h*/
  __IO uint32_t DIEPEMPMSK;  /* dev empty msk             834h*/
  __IO uint32_t DEACHINT;    /* dedicated EP interrupt       838h*/
  __IO uint32_t DEACHMSK;    /* dedicated EP msk             83Ch*/  
  uint32_t Reserved40;       /* dedicated EP mask           840h*/
  __IO uint32_t DINEP1MSK;   /* dedicated EP mask           844h*/
  uint32_t  Reserved44[15];  /* Reserved                 844-87Ch*/
  __IO uint32_t DOUTEP1MSK;  /* dedicated EP msk            884h*/   
} USB_OTG_DeviceTypeDef;

/** 
  * @brief USB_OTG_IN_Endpoint-Specific_Register
  */
typedef struct 
{
  __IO uint32_t DIEPCTL;     /* dev IN Endpoint Control Reg 900h + (ep_num * 20h) + 00h*/
  uint32_t Reserved04;       /* Reserved                       900h + (ep_num * 20h) + 04h*/
  __IO uint32_t DIEPINT;     /* dev IN Endpoint Itr Reg     900h + (ep_num * 20h) + 08h*/
  uint32_t Reserved0C;       /* Reserved                       900h + (ep_num * 20h) + 0Ch*/
  __IO uint32_t DIEPTSIZ;    /* IN Endpoint Txfer Size   900h + (ep_num * 20h) + 10h*/
  __IO uint32_t DIEPDMA;     /* IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h*/
  __IO uint32_t DTXFSTS;     /*IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h*/
  uint32_t Reserved18;       /* Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch*/
} USB_OTG_INEndpointTypeDef;

/** 
  * @brief USB_OTG_OUT_Endpoint-Specific_Registers
  */
typedef struct 
{
  __IO uint32_t DOEPCTL;     /* dev OUT Endpoint Control Reg  B00h + (ep_num * 20h) + 00h*/
  uint32_t Reserved04;       /* Reserved                      B00h + (ep_num * 20h) + 04h*/
  __IO uint32_t DOEPINT;     /* dev OUT Endpoint Itr Reg      B00h + (ep_num * 20h) + 08h*/
  uint32_t Reserved0C;       /* Reserved                      B00h + (ep_num * 20h) + 0Ch*/
  __IO uint32_t DOEPTSIZ;    /* dev OUT Endpoint Txfer Size   B00h + (ep_num * 20h) + 10h*/
  __IO uint32_t DOEPDMA;     /* dev OUT Endpoint DMA Address  B00h + (ep_num * 20h) + 14h*/
  uint32_t Reserved18[2];    /* Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch*/
} USB_OTG_OUTEndpointTypeDef;

/** 
  * @brief USB_OTG_Host_Mode_Register_Structures
  */
typedef struct 
{
  __IO uint32_t HCFG;        /* Host Configuration Register    400h*/
  __IO uint32_t HFIR;        /* Host Frame Interval Register   404h*/
  __IO uint32_t HFNUM;       /* Host Frame Nbr/Frame Remaining 408h*/
  uint32_t Reserved40C;      /* Reserved                       40Ch*/
  __IO uint32_t HPTXSTS;     /* Host Periodic Tx FIFO/ Queue Status 410h*/
  __IO uint32_t HAINT;       /* Host All Channels Interrupt Register 414h*/
  __IO uint32_t HAINTMSK;    /* Host All Channels Interrupt Mask 418h*/
} USB_OTG_HostTypeDef;

/** 
  * @brief USB_OTG_Host_Channel_Specific_Registers
  */
typedef struct
{
  __IO uint32_t HCCHAR;
  __IO uint32_t HCSPLT;
  __IO uint32_t HCINT;
  __IO uint32_t HCINTMSK;
  __IO uint32_t HCTSIZ;
  __IO uint32_t HCDMA;
  uint32_t Reserved[2];
} USB_OTG_HostChannelTypeDef;

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH(up to 1 MB) base address */
#define SRAM1_BASE            ((uint32_t)0x20000000) /*!< SRAM1(96 KB) base address*/
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address */
#define FMC_BASE              ((uint32_t)0x60000000) /*!< FMC base address */
#define SRAM2_BASE            ((uint32_t)0x10000000) /*!< SRAM2(32 KB) base address*/
#define FMC_R_BASE            ((uint32_t)0xA0000000) /*!< FMC  control registers base address */
#define QSPI_R_BASE           ((uint32_t)0xA0001000) /*!< QUADSPI control registers base address */
#define SRAM1_BB_BASE         ((uint32_t)0x22000000) /*!< SRAM1(96 KB) base address in the bit-band region */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000) /*!< Peripheral base address in the bit-band region */
#define SRAM2_BB_BASE         ((uint32_t)0x12000000) /*!< SRAM2(32 KB) base address in the bit-band region */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE

#define SRAM1_SIZE_MAX        ((uint32_t)0x00018000) /*!< maximum SRAM1 size (up to 96 KBytes) */
#define SRAM2_SIZE            ((uint32_t)0x00008000) /*!< SRAM2 size (32 KBytes) */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000)

#define FMC_BANK1             FMC_BASE
#define FMC_BANK1_1           FMC_BANK1
#define FMC_BANK1_2           (FMC_BANK1 + 0x04000000)
#define FMC_BANK1_3           (FMC_BANK1 + 0x08000000)
#define FMC_BANK1_4           (FMC_BANK1 + 0x0C000000)
#define FMC_BANK3             (FMC_BASE  + 0x20000000)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400)
#define LCD_BASE              (APB1PERIPH_BASE + 0x2400)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400)
#define LPTIM1_BASE           (APB1PERIPH_BASE + 0x7C00)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400)
#define DAC1_BASE             (APB1PERIPH_BASE + 0x7400)
#define OPAMP_BASE            (APB1PERIPH_BASE + 0x7800)
#define OPAMP1_BASE           (APB1PERIPH_BASE + 0x7800)
#define OPAMP2_BASE           (APB1PERIPH_BASE + 0x7810)
#define LPUART1_BASE          (APB1PERIPH_BASE + 0x8000)
#define SWPMI1_BASE           (APB1PERIPH_BASE + 0x8800)
#define LPTIM2_BASE           (APB1PERIPH_BASE + 0x9400)


/*!< APB2 peripherals */
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x0000)
#define VREFBUF_BASE          (APB2PERIPH_BASE + 0x0030)
#define COMP1_BASE            (APB2PERIPH_BASE + 0x0200)
#define COMP2_BASE            (APB2PERIPH_BASE + 0x0204)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define FIREWALL_BASE         (APB2PERIPH_BASE + 0x1C00)
#define SDMMC1_BASE           (APB2PERIPH_BASE + 0x2800)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x3400)
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800)
#define TIM15_BASE            (APB2PERIPH_BASE + 0x4000)
#define TIM16_BASE            (APB2PERIPH_BASE + 0x4400)
#define TIM17_BASE            (APB2PERIPH_BASE + 0x4800)
#define SAI1_BASE             (APB2PERIPH_BASE + 0x5400)
#define SAI1_Block_A_BASE     (SAI1_BASE + 0x004)
#define SAI1_Block_B_BASE     (SAI1_BASE + 0x024)
#define SAI2_BASE             (APB2PERIPH_BASE + 0x5800)
#define SAI2_Block_A_BASE     (SAI2_BASE + 0x004)
#define SAI2_Block_B_BASE     (SAI2_BASE + 0x024)
#define DFSDM_BASE            (APB2PERIPH_BASE + 0x6000)
#define DFSDM_Channel0_BASE   (DFSDM_BASE + 0x00)
#define DFSDM_Channel1_BASE   (DFSDM_BASE + 0x20)
#define DFSDM_Channel2_BASE   (DFSDM_BASE + 0x40)
#define DFSDM_Channel3_BASE   (DFSDM_BASE + 0x60)
#define DFSDM_Channel4_BASE   (DFSDM_BASE + 0x80)
#define DFSDM_Channel5_BASE   (DFSDM_BASE + 0xA0)
#define DFSDM_Channel6_BASE   (DFSDM_BASE + 0xC0)
#define DFSDM_Channel7_BASE   (DFSDM_BASE + 0xE0)
#define DFSDM_Filter0_BASE    (DFSDM_BASE + 0x100)
#define DFSDM_Filter1_BASE    (DFSDM_BASE + 0x180)
#define DFSDM_Filter2_BASE    (DFSDM_BASE + 0x200)
#define DFSDM_Filter3_BASE    (DFSDM_BASE + 0x280)

/*!< AHB1 peripherals */
#define DMA1_BASE             (AHB1PERIPH_BASE)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x0400)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x1000)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x2000)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000)
#define TSC_BASE              (AHB1PERIPH_BASE + 0x4000)


#define DMA1_Channel1_BASE    (DMA1_BASE + 0x0008)
#define DMA1_Channel2_BASE    (DMA1_BASE + 0x001C)
#define DMA1_Channel3_BASE    (DMA1_BASE + 0x0030)
#define DMA1_Channel4_BASE    (DMA1_BASE + 0x0044)
#define DMA1_Channel5_BASE    (DMA1_BASE + 0x0058)
#define DMA1_Channel6_BASE    (DMA1_BASE + 0x006C)
#define DMA1_Channel7_BASE    (DMA1_BASE + 0x0080)
#define DMA1_CSELR_BASE       (DMA1_BASE + 0x00A8)


#define DMA2_Channel1_BASE    (DMA2_BASE + 0x0008)
#define DMA2_Channel2_BASE    (DMA2_BASE + 0x001C)
#define DMA2_Channel3_BASE    (DMA2_BASE + 0x0030)
#define DMA2_Channel4_BASE    (DMA2_BASE + 0x0044)
#define DMA2_Channel5_BASE    (DMA2_BASE + 0x0058)
#define DMA2_Channel6_BASE    (DMA2_BASE + 0x006C)
#define DMA2_Channel7_BASE    (DMA2_BASE + 0x0080)
#define DMA2_CSELR_BASE       (DMA2_BASE + 0x00A8)


/*!< AHB2 peripherals */
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB2PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x1400)
#define GPIOG_BASE            (AHB2PERIPH_BASE + 0x1800)
#define GPIOH_BASE            (AHB2PERIPH_BASE + 0x1C00)

#define USBOTG_BASE           (AHB2PERIPH_BASE + 0x08000000)

#define ADC1_BASE             (AHB2PERIPH_BASE + 0x08040000)
#define ADC2_BASE             (AHB2PERIPH_BASE + 0x08040100)
#define ADC3_BASE             (AHB2PERIPH_BASE + 0x08040200)
#define ADC123_COMMON_BASE    (AHB2PERIPH_BASE + 0x08040300)

#define RNG_BASE              (AHB2PERIPH_BASE + 0x08060800)

/*!< FMC Banks registers base  address */
#define FMC_Bank1_R_BASE      (FMC_R_BASE + 0x0000)
#define FMC_Bank1E_R_BASE     (FMC_R_BASE + 0x0104)
#define FMC_Bank2_R_BASE      (FMC_R_BASE + 0x0060)
#define FMC_Bank3_R_BASE      (FMC_R_BASE + 0x0080)
#define FMC_Bank4_R_BASE      (FMC_R_BASE + 0x00A0)

/* Debug MCU registers base address */
#define DBGMCU_BASE           ((uint32_t )0xE0042000)

/*!< USB registers base address */
#define USB_OTG_FS_PERIPH_BASE               ((uint32_t )0x50000000)

#define USB_OTG_GLOBAL_BASE                  ((uint32_t )0x000)
#define USB_OTG_DEVICE_BASE                  ((uint32_t )0x800)
#define USB_OTG_IN_ENDPOINT_BASE             ((uint32_t )0x900)
#define USB_OTG_OUT_ENDPOINT_BASE            ((uint32_t )0xB00)
#define USB_OTG_EP_REG_SIZE                  ((uint32_t )0x20)
#define USB_OTG_HOST_BASE                    ((uint32_t )0x400)
#define USB_OTG_HOST_PORT_BASE               ((uint32_t )0x440)
#define USB_OTG_HOST_CHANNEL_BASE            ((uint32_t )0x500)
#define USB_OTG_HOST_CHANNEL_SIZE            ((uint32_t )0x20)
#define USB_OTG_PCGCCTL_BASE                 ((uint32_t )0xE00)
#define USB_OTG_FIFO_BASE                    ((uint32_t )0x1000)
#define USB_OTG_FIFO_SIZE                    ((uint32_t )0x1000)

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define LCD                 ((LCD_TypeDef *) LCD_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define UART4               ((USART_TypeDef *) UART4_BASE)
#define UART5               ((USART_TypeDef *) UART5_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define CAN                 ((CAN_TypeDef *) CAN1_BASE)
#define CAN1                ((CAN_TypeDef *) CAN1_BASE)
#define LPTIM1              ((LPTIM_TypeDef *) LPTIM1_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define DAC                 ((DAC_TypeDef *) DAC1_BASE)
#define DAC1                ((DAC_TypeDef *) DAC1_BASE)
#define OPAMP               ((OPAMP_TypeDef *) OPAMP_BASE)
#define OPAMP1              ((OPAMP_TypeDef *) OPAMP1_BASE)
#define OPAMP2              ((OPAMP_TypeDef *) OPAMP2_BASE)
#define LPUART1             ((USART_TypeDef *) LPUART1_BASE)
#define SWPMI1              ((SWPMI_TypeDef *) SWPMI1_BASE)
#define LPTIM2              ((LPTIM_TypeDef *) LPTIM2_BASE)

#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define VREFBUF             ((VREFBUF_TypeDef *) VREFBUF_BASE)
#define COMP1               ((COMP_TypeDef *) COMP1_BASE)
#define COMP2               ((COMP_TypeDef *) COMP2_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define FIREWALL            ((FIREWALL_TypeDef *) FIREWALL_BASE)
#define SDMMC1              ((SDMMC_TypeDef *) SDMMC1_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define TIM8                ((TIM_TypeDef *) TIM8_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define TIM15               ((TIM_TypeDef *) TIM15_BASE)
#define TIM16               ((TIM_TypeDef *) TIM16_BASE)
#define TIM17               ((TIM_TypeDef *) TIM17_BASE)
#define SAI1                ((SAI_TypeDef *) SAI1_BASE)
#define SAI1_Block_A        ((SAI_Block_TypeDef *)SAI1_Block_A_BASE)
#define SAI1_Block_B        ((SAI_Block_TypeDef *)SAI1_Block_B_BASE)
#define SAI2                ((SAI_TypeDef *) SAI2_BASE)
#define SAI2_Block_A        ((SAI_Block_TypeDef *)SAI2_Block_A_BASE)
#define SAI2_Block_B        ((SAI_Block_TypeDef *)SAI2_Block_B_BASE)
#define DFSDM_Channel0      ((DFSDM_Channel_TypeDef *) DFSDM_Channel0_BASE)
#define DFSDM_Channel1      ((DFSDM_Channel_TypeDef *) DFSDM_Channel1_BASE)
#define DFSDM_Channel2      ((DFSDM_Channel_TypeDef *) DFSDM_Channel2_BASE)
#define DFSDM_Channel3      ((DFSDM_Channel_TypeDef *) DFSDM_Channel3_BASE)
#define DFSDM_Channel4      ((DFSDM_Channel_TypeDef *) DFSDM_Channel4_BASE)
#define DFSDM_Channel5      ((DFSDM_Channel_TypeDef *) DFSDM_Channel5_BASE)
#define DFSDM_Channel6      ((DFSDM_Channel_TypeDef *) DFSDM_Channel6_BASE)
#define DFSDM_Channel7      ((DFSDM_Channel_TypeDef *) DFSDM_Channel7_BASE)
#define DFSDM_Filter0       ((DFSDM_Filter_TypeDef *) DFSDM_Filter0_BASE)
#define DFSDM_Filter1       ((DFSDM_Filter_TypeDef *) DFSDM_Filter1_BASE)
#define DFSDM_Filter2       ((DFSDM_Filter_TypeDef *) DFSDM_Filter2_BASE)
#define DFSDM_Filter3       ((DFSDM_Filter_TypeDef *) DFSDM_Filter3_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define TSC                 ((TSC_TypeDef *) TSC_BASE)

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define ADC3                ((ADC_TypeDef *) ADC3_BASE)
#define ADC123_COMMON       ((ADC_Common_TypeDef *) ADC123_COMMON_BASE)
#define RNG                 ((RNG_TypeDef *) RNG_BASE)


#define DMA1_Channel1       ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE)
#define DMA1_CSELR          ((DMA_request_TypeDef *) DMA1_CSELR_BASE)


#define DMA2_Channel1       ((DMA_Channel_TypeDef *) DMA2_Channel1_BASE)
#define DMA2_Channel2       ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE)
#define DMA2_Channel3       ((DMA_Channel_TypeDef *) DMA2_Channel3_BASE)
#define DMA2_Channel4       ((DMA_Channel_TypeDef *) DMA2_Channel4_BASE)
#define DMA2_Channel5       ((DMA_Channel_TypeDef *) DMA2_Channel5_BASE)
#define DMA2_Channel6       ((DMA_Channel_TypeDef *) DMA2_Channel6_BASE)
#define DMA2_Channel7       ((DMA_Channel_TypeDef *) DMA2_Channel7_BASE)
#define DMA2_CSELR          ((DMA_request_TypeDef *) DMA2_CSELR_BASE)


#define FMC_Bank1_R         ((FMC_Bank1_TypeDef *) FMC_Bank1_R_BASE)
#define FMC_Bank1E_R        ((FMC_Bank1E_TypeDef *) FMC_Bank1E_R_BASE)
#define FMC_Bank3_R         ((FMC_Bank3_TypeDef *) FMC_Bank3_R_BASE)

#define QUADSPI             ((QUADSPI_TypeDef *) QSPI_R_BASE)

#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)

#define USB_OTG_FS          ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)
/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

/** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for ADC_ISR register  ********************/
#define ADC_ISR_ADRDY         ((uint32_t)0x00000001) /*!< ADC Ready (ADRDY) flag  */
#define ADC_ISR_EOSMP         ((uint32_t)0x00000002) /*!< ADC End of Sampling flag */
#define ADC_ISR_EOC           ((uint32_t)0x00000004) /*!< ADC End of Regular Conversion flag */
#define ADC_ISR_EOS           ((uint32_t)0x00000008) /*!< ADC End of Regular sequence of Conversions flag */
#define ADC_ISR_OVR           ((uint32_t)0x00000010) /*!< ADC overrun flag */
#define ADC_ISR_JEOC          ((uint32_t)0x00000020) /*!< ADC End of Injected Conversion flag */
#define ADC_ISR_JEOS          ((uint32_t)0x00000040) /*!< ADC End of Injected sequence of Conversions flag */
#define ADC_ISR_AWD1          ((uint32_t)0x00000080) /*!< ADC Analog watchdog 1 flag */
#define ADC_ISR_AWD2          ((uint32_t)0x00000100) /*!< ADC Analog watchdog 2 flag */
#define ADC_ISR_AWD3          ((uint32_t)0x00000200) /*!< ADC Analog watchdog 3 flag */
#define ADC_ISR_JQOVF         ((uint32_t)0x00000400) /*!< ADC Injected Context Queue Overflow flag */

/********************  Bit definition for ADC_IER register  ********************/
#define ADC_IER_ADRDY         ((uint32_t)0x00000001) /*!< ADC Ready (ADRDY) interrupt source */
#define ADC_IER_EOSMP         ((uint32_t)0x00000002) /*!< ADC End of Sampling interrupt source */
#define ADC_IER_EOC           ((uint32_t)0x00000004) /*!< ADC End of Regular Conversion interrupt source */
#define ADC_IER_EOS           ((uint32_t)0x00000008) /*!< ADC End of Regular sequence of Conversions interrupt source */
#define ADC_IER_OVR           ((uint32_t)0x00000010) /*!< ADC overrun interrupt source */
#define ADC_IER_JEOC          ((uint32_t)0x00000020) /*!< ADC End of Injected Conversion interrupt source */
#define ADC_IER_JEOS          ((uint32_t)0x00000040) /*!< ADC End of Injected sequence of Conversions interrupt source */
#define ADC_IER_AWD1          ((uint32_t)0x00000080) /*!< ADC Analog watchdog 1 interrupt source */
#define ADC_IER_AWD2          ((uint32_t)0x00000100) /*!< ADC Analog watchdog 2 interrupt source */
#define ADC_IER_AWD3          ((uint32_t)0x00000200) /*!< ADC Analog watchdog 3 interrupt source */
#define ADC_IER_JQOVF         ((uint32_t)0x00000400) /*!< ADC Injected Context Queue Overflow interrupt source */

/********************  Bit definition for ADC_CR register  ********************/
#define ADC_CR_ADEN           ((uint32_t)0x00000001) /*!< ADC Enable control */
#define ADC_CR_ADDIS          ((uint32_t)0x00000002) /*!< ADC Disable command */
#define ADC_CR_ADSTART        ((uint32_t)0x00000004) /*!< ADC Start of Regular conversion */
#define ADC_CR_JADSTART       ((uint32_t)0x00000008) /*!< ADC Start of injected conversion */
#define ADC_CR_ADSTP          ((uint32_t)0x00000010) /*!< ADC Stop of Regular conversion */
#define ADC_CR_JADSTP         ((uint32_t)0x00000020) /*!< ADC Stop of injected conversion */
#define ADC_CR_ADVREGEN       ((uint32_t)0x10000000) /*!< ADC Voltage regulator Enable */
#define ADC_CR_DEEPPWD        ((uint32_t)0x20000000) /*!< ADC Deep power down Enable */
#define ADC_CR_ADCALDIF       ((uint32_t)0x40000000) /*!< ADC Differential Mode for calibration */
#define ADC_CR_ADCAL          ((uint32_t)0x80000000) /*!< ADC Calibration */

/********************  Bit definition for ADC_CFGR register  ********************/
#define ADC_CFGR_DMAEN        ((uint32_t)0x00000001) /*!< ADC DMA Enable */
#define ADC_CFGR_DMACFG       ((uint32_t)0x00000002) /*!< ADC DMA configuration */
                             
#define ADC_CFGR_RES          ((uint32_t)0x00000018) /*!< ADC Data resolution */
#define ADC_CFGR_RES_0        ((uint32_t)0x00000008) /*!< ADC RES bit 0 */
#define ADC_CFGR_RES_1        ((uint32_t)0x00000010) /*!< ADC RES bit 1 */
                             
#define ADC_CFGR_ALIGN        ((uint32_t)0x00000020) /*!< ADC Data Alignement */
                             
#define ADC_CFGR_EXTSEL      ((uint32_t)0x000003C0) /*!< ADC External trigger selection for regular group */
#define ADC_CFGR_EXTSEL_0    ((uint32_t)0x00000040) /*!< ADC EXTSEL bit 0 */
#define ADC_CFGR_EXTSEL_1    ((uint32_t)0x00000080) /*!< ADC EXTSEL bit 1 */
#define ADC_CFGR_EXTSEL_2    ((uint32_t)0x00000100) /*!< ADC EXTSEL bit 2 */
#define ADC_CFGR_EXTSEL_3    ((uint32_t)0x00000200) /*!< ADC EXTSEL bit 3 */
                             
#define ADC_CFGR_EXTEN        ((uint32_t)0x00000C00) /*!< ADC External trigger enable and polarity selection for regular channels */
#define ADC_CFGR_EXTEN_0      ((uint32_t)0x00000400) /*!< ADC EXTEN bit 0 */
#define ADC_CFGR_EXTEN_1      ((uint32_t)0x00000800) /*!< ADC EXTEN bit 1 */
                             
#define ADC_CFGR_OVRMOD       ((uint32_t)0x00001000) /*!< ADC overrun mode */
#define ADC_CFGR_CONT         ((uint32_t)0x00002000) /*!< ADC Single/continuous conversion mode for regular conversion */
#define ADC_CFGR_AUTDLY       ((uint32_t)0x00004000) /*!< ADC Delayed conversion mode */

#define ADC_CFGR_DISCEN       ((uint32_t)0x00010000) /*!< ADC Discontinuous mode for regular channels */
                             
#define ADC_CFGR_DISCNUM      ((uint32_t)0x000E0000) /*!< ADC Discontinuous mode channel count */
#define ADC_CFGR_DISCNUM_0    ((uint32_t)0x00020000) /*!< ADC DISCNUM bit 0 */
#define ADC_CFGR_DISCNUM_1    ((uint32_t)0x00040000) /*!< ADC DISCNUM bit 1 */
#define ADC_CFGR_DISCNUM_2    ((uint32_t)0x00080000) /*!< ADC DISCNUM bit 2 */
                              
#define ADC_CFGR_JDISCEN      ((uint32_t)0x00100000) /*!< ADC Discontinuous mode on injected channels */
#define ADC_CFGR_JQM          ((uint32_t)0x00200000) /*!< ADC JSQR Queue mode */
#define ADC_CFGR_AWD1SGL      ((uint32_t)0x00400000) /*!< Enable the watchdog 1 on a single channel or on all channels */
#define ADC_CFGR_AWD1EN       ((uint32_t)0x00800000) /*!< ADC Analog watchdog 1 enable on regular Channels */
#define ADC_CFGR_JAWD1EN      ((uint32_t)0x01000000) /*!< ADC Analog watchdog 1 enable on injected Channels */
#define ADC_CFGR_JAUTO        ((uint32_t)0x02000000) /*!< ADC Automatic injected group conversion */
                              
#define ADC_CFGR_AWD1CH       ((uint32_t)0x7C000000) /*!< ADC Analog watchdog 1 Channel selection */
#define ADC_CFGR_AWD1CH_0     ((uint32_t)0x04000000) /*!< ADC AWD1CH bit 0 */
#define ADC_CFGR_AWD1CH_1     ((uint32_t)0x08000000) /*!< ADC AWD1CH bit 1  */
#define ADC_CFGR_AWD1CH_2     ((uint32_t)0x10000000) /*!< ADC AWD1CH bit 2  */
#define ADC_CFGR_AWD1CH_3     ((uint32_t)0x20000000) /*!< ADC AWD1CH bit 3  */
#define ADC_CFGR_AWD1CH_4     ((uint32_t)0x40000000) /*!< ADC AWD1CH bit 4  */
                              
#define ADC_CFGR_JQDIS        ((uint32_t)0x80000000) /*!< ADC Injected queue disable */

/********************  Bit definition for ADC_CFGR2 register  ********************/
#define ADC_CFGR2_ROVSE       ((uint32_t)0x00000001) /*!< ADC Regular group oversampler enable */
#define ADC_CFGR2_JOVSE       ((uint32_t)0x00000002) /*!< ADC Injected group oversampler enable */
                           
#define ADC_CFGR2_OVSR        ((uint32_t)0x0000001C) /*!< ADC Regular group oversampler enable */
#define ADC_CFGR2_OVSR_0      ((uint32_t)0x00000004) /*!< ADC OVSR bit 0 */
#define ADC_CFGR2_OVSR_1      ((uint32_t)0x00000008) /*!< ADC OVSR bit 1 */
#define ADC_CFGR2_OVSR_2      ((uint32_t)0x00000010) /*!< ADC OVSR bit 2 */
                           
#define ADC_CFGR2_OVSS        ((uint32_t)0x000001E0) /*!< ADC Regular Oversampling shift */
#define ADC_CFGR2_OVSS_0      ((uint32_t)0x00000020) /*!< ADC OVSS bit 0 */
#define ADC_CFGR2_OVSS_1      ((uint32_t)0x00000040) /*!< ADC OVSS bit 1 */
#define ADC_CFGR2_OVSS_2      ((uint32_t)0x00000080) /*!< ADC OVSS bit 2 */
#define ADC_CFGR2_OVSS_3      ((uint32_t)0x00000100) /*!< ADC OVSS bit 3 */
                           
#define ADC_CFGR2_TROVS       ((uint32_t)0x00000200) /*!< ADC Triggered regular Oversampling */
#define ADC_CFGR2_ROVSM       ((uint32_t)0x00000400) /*!< ADC Regular oversampling mode */
                           
/********************  Bit definition for ADC_SMPR1 register  ********************/
#define ADC_SMPR1_SMP0        ((uint32_t)0x00000007) /*!< ADC Channel 0 Sampling time selection  */
#define ADC_SMPR1_SMP0_0      ((uint32_t)0x00000001) /*!< ADC SMP0 bit 0 */
#define ADC_SMPR1_SMP0_1      ((uint32_t)0x00000002) /*!< ADC SMP0 bit 1 */
#define ADC_SMPR1_SMP0_2      ((uint32_t)0x00000004) /*!< ADC SMP0 bit 2 */
                             
#define ADC_SMPR1_SMP1        ((uint32_t)0x00000038) /*!< ADC Channel 1 Sampling time selection  */
#define ADC_SMPR1_SMP1_0      ((uint32_t)0x00000008) /*!< ADC SMP1 bit 0 */
#define ADC_SMPR1_SMP1_1      ((uint32_t)0x00000010) /*!< ADC SMP1 bit 1 */
#define ADC_SMPR1_SMP1_2      ((uint32_t)0x00000020) /*!< ADC SMP1 bit 2 */
                             
#define ADC_SMPR1_SMP2        ((uint32_t)0x000001C0) /*!< ADC Channel 2 Sampling time selection  */
#define ADC_SMPR1_SMP2_0      ((uint32_t)0x00000040) /*!< ADC SMP2 bit 0 */
#define ADC_SMPR1_SMP2_1      ((uint32_t)0x00000080) /*!< ADC SMP2 bit 1 */
#define ADC_SMPR1_SMP2_2      ((uint32_t)0x00000100) /*!< ADC SMP2 bit 2 */
                             
#define ADC_SMPR1_SMP3        ((uint32_t)0x00000E00) /*!< ADC Channel 3 Sampling time selection  */
#define ADC_SMPR1_SMP3_0      ((uint32_t)0x00000200) /*!< ADC SMP3 bit 0 */
#define ADC_SMPR1_SMP3_1      ((uint32_t)0x00000400) /*!< ADC SMP3 bit 1 */
#define ADC_SMPR1_SMP3_2      ((uint32_t)0x00000800) /*!< ADC SMP3 bit 2 */
                             
#define ADC_SMPR1_SMP4        ((uint32_t)0x00007000) /*!< ADC Channel 4 Sampling time selection  */
#define ADC_SMPR1_SMP4_0      ((uint32_t)0x00001000) /*!< ADC SMP4 bit 0 */
#define ADC_SMPR1_SMP4_1      ((uint32_t)0x00002000) /*!< ADC SMP4 bit 1 */
#define ADC_SMPR1_SMP4_2      ((uint32_t)0x00004000) /*!< ADC SMP4 bit 2 */
                             
#define ADC_SMPR1_SMP5        ((uint32_t)0x00038000) /*!< ADC Channel 5 Sampling time selection  */
#define ADC_SMPR1_SMP5_0      ((uint32_t)0x00008000) /*!< ADC SMP5 bit 0 */
#define ADC_SMPR1_SMP5_1      ((uint32_t)0x00010000) /*!< ADC SMP5 bit 1 */
#define ADC_SMPR1_SMP5_2      ((uint32_t)0x00020000) /*!< ADC SMP5 bit 2 */
                             
#define ADC_SMPR1_SMP6        ((uint32_t)0x001C0000) /*!< ADC Channel 6 Sampling time selection  */
#define ADC_SMPR1_SMP6_0      ((uint32_t)0x00040000) /*!< ADC SMP6 bit 0 */
#define ADC_SMPR1_SMP6_1      ((uint32_t)0x00080000) /*!< ADC SMP6 bit 1 */
#define ADC_SMPR1_SMP6_2      ((uint32_t)0x00100000) /*!< ADC SMP6 bit 2 */
                             
#define ADC_SMPR1_SMP7        ((uint32_t)0x00E00000) /*!< ADC Channel 7 Sampling time selection  */
#define ADC_SMPR1_SMP7_0      ((uint32_t)0x00200000) /*!< ADC SMP7 bit 0 */
#define ADC_SMPR1_SMP7_1      ((uint32_t)0x00400000) /*!< ADC SMP7 bit 1 */
#define ADC_SMPR1_SMP7_2      ((uint32_t)0x00800000) /*!< ADC SMP7 bit 2 */
                             
#define ADC_SMPR1_SMP8        ((uint32_t)0x07000000) /*!< ADC Channel 8 Sampling time selection  */
#define ADC_SMPR1_SMP8_0      ((uint32_t)0x01000000) /*!< ADC SMP8 bit 0 */
#define ADC_SMPR1_SMP8_1      ((uint32_t)0x02000000) /*!< ADC SMP8 bit 1 */
#define ADC_SMPR1_SMP8_2      ((uint32_t)0x04000000) /*!< ADC SMP8 bit 2 */
                             
#define ADC_SMPR1_SMP9        ((uint32_t)0x38000000) /*!< ADC Channel 9 Sampling time selection  */
#define ADC_SMPR1_SMP9_0      ((uint32_t)0x08000000) /*!< ADC SMP9 bit 0 */
#define ADC_SMPR1_SMP9_1      ((uint32_t)0x10000000) /*!< ADC SMP9 bit 1 */
#define ADC_SMPR1_SMP9_2      ((uint32_t)0x20000000) /*!< ADC SMP9 bit 2 */

/********************  Bit definition for ADC_SMPR2 register  ********************/
#define ADC_SMPR2_SMP10     ((uint32_t)0x00000007) /*!< ADC Channel 10 Sampling time selection  */
#define ADC_SMPR2_SMP10_0   ((uint32_t)0x00000001) /*!< ADC SMP10 bit 0 */
#define ADC_SMPR2_SMP10_1   ((uint32_t)0x00000002) /*!< ADC SMP10 bit 1 */
#define ADC_SMPR2_SMP10_2   ((uint32_t)0x00000004) /*!< ADC SMP10 bit 2 */

#define ADC_SMPR2_SMP11     ((uint32_t)0x00000038) /*!< ADC Channel 11 Sampling time selection  */
#define ADC_SMPR2_SMP11_0   ((uint32_t)0x00000008) /*!< ADC SMP11 bit 0 */
#define ADC_SMPR2_SMP11_1   ((uint32_t)0x00000010) /*!< ADC SMP11 bit 1 */
#define ADC_SMPR2_SMP11_2   ((uint32_t)0x00000020) /*!< ADC SMP11 bit 2 */

#define ADC_SMPR2_SMP12     ((uint32_t)0x000001C0) /*!< ADC Channel 12 Sampling time selection  */
#define ADC_SMPR2_SMP12_0   ((uint32_t)0x00000040) /*!< ADC SMP12 bit 0 */
#define ADC_SMPR2_SMP12_1   ((uint32_t)0x00000080) /*!< ADC SMP12 bit 1 */
#define ADC_SMPR2_SMP12_2   ((uint32_t)0x00000100) /*!< ADC SMP12 bit 2 */

#define ADC_SMPR2_SMP13     ((uint32_t)0x00000E00) /*!< ADC Channel 13 Sampling time selection  */
#define ADC_SMPR2_SMP13_0   ((uint32_t)0x00000200) /*!< ADC SMP13 bit 0 */
#define ADC_SMPR2_SMP13_1   ((uint32_t)0x00000400) /*!< ADC SMP13 bit 1 */
#define ADC_SMPR2_SMP13_2   ((uint32_t)0x00000800) /*!< ADC SMP13 bit 2 */

#define ADC_SMPR2_SMP14     ((uint32_t)0x00007000) /*!< ADC Channel 14 Sampling time selection  */
#define ADC_SMPR2_SMP14_0   ((uint32_t)0x00001000) /*!< ADC SMP14 bit 0 */
#define ADC_SMPR2_SMP14_1   ((uint32_t)0x00002000) /*!< ADC SMP14 bit 1 */
#define ADC_SMPR2_SMP14_2   ((uint32_t)0x00004000) /*!< ADC SMP14 bit 2 */

#define ADC_SMPR2_SMP15     ((uint32_t)0x00038000) /*!< ADC Channel 15 Sampling time selection  */
#define ADC_SMPR2_SMP15_0   ((uint32_t)0x00008000) /*!< ADC SMP15 bit 0 */
#define ADC_SMPR2_SMP15_1   ((uint32_t)0x00010000) /*!< ADC SMP15 bit 1 */
#define ADC_SMPR2_SMP15_2   ((uint32_t)0x00020000) /*!< ADC SMP15 bit 2 */

#define ADC_SMPR2_SMP16     ((uint32_t)0x001C0000) /*!< ADC Channel 16 Sampling time selection  */
#define ADC_SMPR2_SMP16_0   ((uint32_t)0x00040000) /*!< ADC SMP16 bit 0 */
#define ADC_SMPR2_SMP16_1   ((uint32_t)0x00080000) /*!< ADC SMP16 bit 1 */
#define ADC_SMPR2_SMP16_2   ((uint32_t)0x00100000) /*!< ADC SMP16 bit 2 */

#define ADC_SMPR2_SMP17     ((uint32_t)0x00E00000) /*!< ADC Channel 17 Sampling time selection  */
#define ADC_SMPR2_SMP17_0   ((uint32_t)0x00200000) /*!< ADC SMP17 bit 0 */
#define ADC_SMPR2_SMP17_1   ((uint32_t)0x00400000) /*!< ADC SMP17 bit 1 */
#define ADC_SMPR2_SMP17_2   ((uint32_t)0x00800000) /*!< ADC SMP17 bit 2 */

#define ADC_SMPR2_SMP18     ((uint32_t)0x07000000) /*!< ADC Channel 18 Sampling time selection  */
#define ADC_SMPR2_SMP18_0   ((uint32_t)0x01000000) /*!< ADC SMP18 bit 0 */
#define ADC_SMPR2_SMP18_1   ((uint32_t)0x02000000) /*!< ADC SMP18 bit 1 */
#define ADC_SMPR2_SMP18_2   ((uint32_t)0x04000000) /*!< ADC SMP18 bit 2 */

/********************  Bit definition for ADC_TR1 register  ********************/
#define ADC_TR1_LT1         ((uint32_t)0x00000FFF) /*!< ADC Analog watchdog 1 lower threshold */
#define ADC_TR1_LT1_0       ((uint32_t)0x00000001) /*!< ADC LT1 bit 0 */
#define ADC_TR1_LT1_1       ((uint32_t)0x00000002) /*!< ADC LT1 bit 1 */
#define ADC_TR1_LT1_2       ((uint32_t)0x00000004) /*!< ADC LT1 bit 2 */
#define ADC_TR1_LT1_3       ((uint32_t)0x00000008) /*!< ADC LT1 bit 3 */
#define ADC_TR1_LT1_4       ((uint32_t)0x00000010) /*!< ADC LT1 bit 4 */
#define ADC_TR1_LT1_5       ((uint32_t)0x00000020) /*!< ADC LT1 bit 5 */
#define ADC_TR1_LT1_6       ((uint32_t)0x00000040) /*!< ADC LT1 bit 6 */
#define ADC_TR1_LT1_7       ((uint32_t)0x00000080) /*!< ADC LT1 bit 7 */
#define ADC_TR1_LT1_8       ((uint32_t)0x00000100) /*!< ADC LT1 bit 8 */
#define ADC_TR1_LT1_9       ((uint32_t)0x00000200) /*!< ADC LT1 bit 9 */
#define ADC_TR1_LT1_10      ((uint32_t)0x00000400) /*!< ADC LT1 bit 10 */
#define ADC_TR1_LT1_11      ((uint32_t)0x00000800) /*!< ADC LT1 bit 11 */

#define ADC_TR1_HT1         ((uint32_t)0x0FFF0000) /*!< ADC Analog watchdog 1 higher threshold */
#define ADC_TR1_HT1_0       ((uint32_t)0x00010000) /*!< ADC HT1 bit 0 */
#define ADC_TR1_HT1_1       ((uint32_t)0x00020000) /*!< ADC HT1 bit 1 */
#define ADC_TR1_HT1_2       ((uint32_t)0x00040000) /*!< ADC HT1 bit 2 */
#define ADC_TR1_HT1_3       ((uint32_t)0x00080000) /*!< ADC HT1 bit 3 */
#define ADC_TR1_HT1_4       ((uint32_t)0x00100000) /*!< ADC HT1 bit 4 */
#define ADC_TR1_HT1_5       ((uint32_t)0x00200000) /*!< ADC HT1 bit 5 */
#define ADC_TR1_HT1_6       ((uint32_t)0x00400000) /*!< ADC HT1 bit 6 */
#define ADC_TR1_HT1_7       ((uint32_t)0x00800000) /*!< ADC HT1 bit 7 */
#define ADC_TR1_HT1_8       ((uint32_t)0x01000000) /*!< ADC HT1 bit 8 */
#define ADC_TR1_HT1_9       ((uint32_t)0x02000000) /*!< ADC HT1 bit 9 */
#define ADC_TR1_HT1_10      ((uint32_t)0x04000000) /*!< ADC HT1 bit 10 */
#define ADC_TR1_HT1_11      ((uint32_t)0x08000000) /*!< ADC HT1 bit 11 */

/********************  Bit definition for ADC_TR2 register  ********************/
#define ADC_TR2_LT2         ((uint32_t)0x000000FF) /*!< ADC Analog watchdog 2 lower threshold */
#define ADC_TR2_LT2_0       ((uint32_t)0x00000001) /*!< ADC LT2 bit 0 */
#define ADC_TR2_LT2_1       ((uint32_t)0x00000002) /*!< ADC LT2 bit 1 */
#define ADC_TR2_LT2_2       ((uint32_t)0x00000004) /*!< ADC LT2 bit 2 */
#define ADC_TR2_LT2_3       ((uint32_t)0x00000008) /*!< ADC LT2 bit 3 */
#define ADC_TR2_LT2_4       ((uint32_t)0x00000010) /*!< ADC LT2 bit 4 */
#define ADC_TR2_LT2_5       ((uint32_t)0x00000020) /*!< ADC LT2 bit 5 */
#define ADC_TR2_LT2_6       ((uint32_t)0x00000040) /*!< ADC LT2 bit 6 */
#define ADC_TR2_LT2_7       ((uint32_t)0x00000080) /*!< ADC LT2 bit 7 */

#define ADC_TR2_HT2         ((uint32_t)0x00FF0000) /*!< ADC Analog watchdog 2 higher threshold */
#define ADC_TR2_HT2_0       ((uint32_t)0x00010000) /*!< ADC HT2 bit 0 */
#define ADC_TR2_HT2_1       ((uint32_t)0x00020000) /*!< ADC HT2 bit 1 */
#define ADC_TR2_HT2_2       ((uint32_t)0x00040000) /*!< ADC HT2 bit 2 */
#define ADC_TR2_HT2_3       ((uint32_t)0x00080000) /*!< ADC HT2 bit 3 */
#define ADC_TR2_HT2_4       ((uint32_t)0x00100000) /*!< ADC HT2 bit 4 */
#define ADC_TR2_HT2_5       ((uint32_t)0x00200000) /*!< ADC HT2 bit 5 */
#define ADC_TR2_HT2_6       ((uint32_t)0x00400000) /*!< ADC HT2 bit 6 */
#define ADC_TR2_HT2_7       ((uint32_t)0x00800000) /*!< ADC HT2 bit 7 */

/********************  Bit definition for ADC_TR3 register  ********************/
#define ADC_TR3_LT3         ((uint32_t)0x000000FF) /*!< ADC Analog watchdog 3 lower threshold */
#define ADC_TR3_LT3_0       ((uint32_t)0x00000001) /*!< ADC LT3 bit 0 */
#define ADC_TR3_LT3_1       ((uint32_t)0x00000002) /*!< ADC LT3 bit 1 */
#define ADC_TR3_LT3_2       ((uint32_t)0x00000004) /*!< ADC LT3 bit 2 */
#define ADC_TR3_LT3_3       ((uint32_t)0x00000008) /*!< ADC LT3 bit 3 */
#define ADC_TR3_LT3_4       ((uint32_t)0x00000010) /*!< ADC LT3 bit 4 */
#define ADC_TR3_LT3_5       ((uint32_t)0x00000020) /*!< ADC LT3 bit 5 */
#define ADC_TR3_LT3_6       ((uint32_t)0x00000040) /*!< ADC LT3 bit 6 */
#define ADC_TR3_LT3_7       ((uint32_t)0x00000080) /*!< ADC LT3 bit 7 */

#define ADC_TR3_HT3         ((uint32_t)0x00FF0000) /*!< ADC Analog watchdog 3 higher threshold */
#define ADC_TR3_HT3_0       ((uint32_t)0x00010000) /*!< ADC HT3 bit 0 */
#define ADC_TR3_HT3_1       ((uint32_t)0x00020000) /*!< ADC HT3 bit 1 */
#define ADC_TR3_HT3_2       ((uint32_t)0x00040000) /*!< ADC HT3 bit 2 */
#define ADC_TR3_HT3_3       ((uint32_t)0x00080000) /*!< ADC HT3 bit 3 */
#define ADC_TR3_HT3_4       ((uint32_t)0x00100000) /*!< ADC HT3 bit 4 */
#define ADC_TR3_HT3_5       ((uint32_t)0x00200000) /*!< ADC HT3 bit 5 */
#define ADC_TR3_HT3_6       ((uint32_t)0x00400000) /*!< ADC HT3 bit 6 */
#define ADC_TR3_HT3_7       ((uint32_t)0x00800000) /*!< ADC HT3 bit 7 */

/********************  Bit definition for ADC_SQR1 register  ********************/
#define ADC_SQR1_L          ((uint32_t)0x0000000F) /*!< ADC regular channel sequence lenght */
#define ADC_SQR1_L_0        ((uint32_t)0x00000001) /*!< ADC L bit 0 */
#define ADC_SQR1_L_1        ((uint32_t)0x00000002) /*!< ADC L bit 1 */
#define ADC_SQR1_L_2        ((uint32_t)0x00000004) /*!< ADC L bit 2 */
#define ADC_SQR1_L_3        ((uint32_t)0x00000008) /*!< ADC L bit 3 */

#define ADC_SQR1_SQ1        ((uint32_t)0x000007C0) /*!< ADC 1st conversion in regular sequence */
#define ADC_SQR1_SQ1_0      ((uint32_t)0x00000040) /*!< ADC SQ1 bit 0 */
#define ADC_SQR1_SQ1_1      ((uint32_t)0x00000080) /*!< ADC SQ1 bit 1 */
#define ADC_SQR1_SQ1_2      ((uint32_t)0x00000100) /*!< ADC SQ1 bit 2 */
#define ADC_SQR1_SQ1_3      ((uint32_t)0x00000200) /*!< ADC SQ1 bit 3 */
#define ADC_SQR1_SQ1_4      ((uint32_t)0x00000400) /*!< ADC SQ1 bit 4 */

#define ADC_SQR1_SQ2        ((uint32_t)0x0001F000) /*!< ADC 2nd conversion in regular sequence */
#define ADC_SQR1_SQ2_0      ((uint32_t)0x00001000) /*!< ADC SQ2 bit 0 */
#define ADC_SQR1_SQ2_1      ((uint32_t)0x00002000) /*!< ADC SQ2 bit 1 */
#define ADC_SQR1_SQ2_2      ((uint32_t)0x00004000) /*!< ADC SQ2 bit 2 */
#define ADC_SQR1_SQ2_3      ((uint32_t)0x00008000) /*!< ADC SQ2 bit 3 */
#define ADC_SQR1_SQ2_4      ((uint32_t)0x00010000) /*!< ADC SQ2 bit 4 */

#define ADC_SQR1_SQ3        ((uint32_t)0x007C0000) /*!< ADC 3rd conversion in regular sequence */
#define ADC_SQR1_SQ3_0      ((uint32_t)0x00040000) /*!< ADC SQ3 bit 0 */
#define ADC_SQR1_SQ3_1      ((uint32_t)0x00080000) /*!< ADC SQ3 bit 1 */
#define ADC_SQR1_SQ3_2      ((uint32_t)0x00100000) /*!< ADC SQ3 bit 2 */
#define ADC_SQR1_SQ3_3      ((uint32_t)0x00200000) /*!< ADC SQ3 bit 3 */
#define ADC_SQR1_SQ3_4      ((uint32_t)0x00400000) /*!< ADC SQ3 bit 4 */

#define ADC_SQR1_SQ4        ((uint32_t)0x1F000000) /*!< ADC 4th conversion in regular sequence */
#define ADC_SQR1_SQ4_0      ((uint32_t)0x01000000) /*!< ADC SQ4 bit 0 */
#define ADC_SQR1_SQ4_1      ((uint32_t)0x02000000) /*!< ADC SQ4 bit 1 */
#define ADC_SQR1_SQ4_2      ((uint32_t)0x04000000) /*!< ADC SQ4 bit 2 */
#define ADC_SQR1_SQ4_3      ((uint32_t)0x08000000) /*!< ADC SQ4 bit 3 */
#define ADC_SQR1_SQ4_4      ((uint32_t)0x10000000) /*!< ADC SQ4 bit 4 */

/********************  Bit definition for ADC_SQR2 register  ********************/
#define ADC_SQR2_SQ5        ((uint32_t)0x0000001F) /*!< ADC 5th conversion in regular sequence */
#define ADC_SQR2_SQ5_0      ((uint32_t)0x00000001) /*!< ADC SQ5 bit 0 */
#define ADC_SQR2_SQ5_1      ((uint32_t)0x00000002) /*!< ADC SQ5 bit 1 */
#define ADC_SQR2_SQ5_2      ((uint32_t)0x00000004) /*!< ADC SQ5 bit 2 */
#define ADC_SQR2_SQ5_3      ((uint32_t)0x00000008) /*!< ADC SQ5 bit 3 */
#define ADC_SQR2_SQ5_4      ((uint32_t)0x00000010) /*!< ADC SQ5 bit 4 */

#define ADC_SQR2_SQ6        ((uint32_t)0x000007C0) /*!< ADC 6th conversion in regular sequence */
#define ADC_SQR2_SQ6_0      ((uint32_t)0x00000040) /*!< ADC SQ6 bit 0 */
#define ADC_SQR2_SQ6_1      ((uint32_t)0x00000080) /*!< ADC SQ6 bit 1 */
#define ADC_SQR2_SQ6_2      ((uint32_t)0x00000100) /*!< ADC SQ6 bit 2 */
#define ADC_SQR2_SQ6_3      ((uint32_t)0x00000200) /*!< ADC SQ6 bit 3 */
#define ADC_SQR2_SQ6_4      ((uint32_t)0x00000400) /*!< ADC SQ6 bit 4 */

#define ADC_SQR2_SQ7        ((uint32_t)0x0001F000) /*!< ADC 7th conversion in regular sequence */
#define ADC_SQR2_SQ7_0      ((uint32_t)0x00001000) /*!< ADC SQ7 bit 0 */
#define ADC_SQR2_SQ7_1      ((uint32_t)0x00002000) /*!< ADC SQ7 bit 1 */
#define ADC_SQR2_SQ7_2      ((uint32_t)0x00004000) /*!< ADC SQ7 bit 2 */
#define ADC_SQR2_SQ7_3      ((uint32_t)0x00008000) /*!< ADC SQ7 bit 3 */
#define ADC_SQR2_SQ7_4      ((uint32_t)0x00010000) /*!< ADC SQ7 bit 4 */

#define ADC_SQR2_SQ8        ((uint32_t)0x007C0000) /*!< ADC 8th conversion in regular sequence */
#define ADC_SQR2_SQ8_0      ((uint32_t)0x00040000) /*!< ADC SQ8 bit 0 */
#define ADC_SQR2_SQ8_1      ((uint32_t)0x00080000) /*!< ADC SQ8 bit 1 */
#define ADC_SQR2_SQ8_2      ((uint32_t)0x00100000) /*!< ADC SQ8 bit 2 */
#define ADC_SQR2_SQ8_3      ((uint32_t)0x00200000) /*!< ADC SQ8 bit 3 */
#define ADC_SQR2_SQ8_4      ((uint32_t)0x00400000) /*!< ADC SQ8 bit 4 */

#define ADC_SQR2_SQ9        ((uint32_t)0x1F000000) /*!< ADC 9th conversion in regular sequence */
#define ADC_SQR2_SQ9_0      ((uint32_t)0x01000000) /*!< ADC SQ9 bit 0 */
#define ADC_SQR2_SQ9_1      ((uint32_t)0x02000000) /*!< ADC SQ9 bit 1 */
#define ADC_SQR2_SQ9_2      ((uint32_t)0x04000000) /*!< ADC SQ9 bit 2 */
#define ADC_SQR2_SQ9_3      ((uint32_t)0x08000000) /*!< ADC SQ9 bit 3 */
#define ADC_SQR2_SQ9_4      ((uint32_t)0x10000000) /*!< ADC SQ9 bit 4 */

/********************  Bit definition for ADC_SQR3 register  ********************/
#define ADC_SQR3_SQ10       ((uint32_t)0x0000001F) /*!< ADC 10th conversion in regular sequence */
#define ADC_SQR3_SQ10_0     ((uint32_t)0x00000001) /*!< ADC SQ10 bit 0 */
#define ADC_SQR3_SQ10_1     ((uint32_t)0x00000002) /*!< ADC SQ10 bit 1 */
#define ADC_SQR3_SQ10_2     ((uint32_t)0x00000004) /*!< ADC SQ10 bit 2 */
#define ADC_SQR3_SQ10_3     ((uint32_t)0x00000008) /*!< ADC SQ10 bit 3 */
#define ADC_SQR3_SQ10_4     ((uint32_t)0x00000010) /*!< ADC SQ10 bit 4 */

#define ADC_SQR3_SQ11       ((uint32_t)0x000007C0) /*!< ADC 11th conversion in regular sequence */
#define ADC_SQR3_SQ11_0     ((uint32_t)0x00000040) /*!< ADC SQ11 bit 0 */
#define ADC_SQR3_SQ11_1     ((uint32_t)0x00000080) /*!< ADC SQ11 bit 1 */
#define ADC_SQR3_SQ11_2     ((uint32_t)0x00000100) /*!< ADC SQ11 bit 2 */
#define ADC_SQR3_SQ11_3     ((uint32_t)0x00000200) /*!< ADC SQ11 bit 3 */
#define ADC_SQR3_SQ11_4     ((uint32_t)0x00000400) /*!< ADC SQ11 bit 4 */

#define ADC_SQR3_SQ12       ((uint32_t)0x0001F000) /*!< ADC 12th conversion in regular sequence */
#define ADC_SQR3_SQ12_0     ((uint32_t)0x00001000) /*!< ADC SQ12 bit 0 */
#define ADC_SQR3_SQ12_1     ((uint32_t)0x00002000) /*!< ADC SQ12 bit 1 */
#define ADC_SQR3_SQ12_2     ((uint32_t)0x00004000) /*!< ADC SQ12 bit 2 */
#define ADC_SQR3_SQ12_3     ((uint32_t)0x00008000) /*!< ADC SQ12 bit 3 */
#define ADC_SQR3_SQ12_4     ((uint32_t)0x00010000) /*!< ADC SQ12 bit 4 */

#define ADC_SQR3_SQ13       ((uint32_t)0x007C0000) /*!< ADC 13th conversion in regular sequence */
#define ADC_SQR3_SQ13_0     ((uint32_t)0x00040000) /*!< ADC SQ13 bit 0 */
#define ADC_SQR3_SQ13_1     ((uint32_t)0x00080000) /*!< ADC SQ13 bit 1 */
#define ADC_SQR3_SQ13_2     ((uint32_t)0x00100000) /*!< ADC SQ13 bit 2 */
#define ADC_SQR3_SQ13_3     ((uint32_t)0x00200000) /*!< ADC SQ13 bit 3 */
#define ADC_SQR3_SQ13_4     ((uint32_t)0x00400000) /*!< ADC SQ13 bit 4 */

#define ADC_SQR3_SQ14       ((uint32_t)0x1F000000) /*!< ADC 14th conversion in regular sequence */
#define ADC_SQR3_SQ14_0     ((uint32_t)0x01000000) /*!< ADC SQ14 bit 0 */
#define ADC_SQR3_SQ14_1     ((uint32_t)0x02000000) /*!< ADC SQ14 bit 1 */
#define ADC_SQR3_SQ14_2     ((uint32_t)0x04000000) /*!< ADC SQ14 bit 2 */
#define ADC_SQR3_SQ14_3     ((uint32_t)0x08000000) /*!< ADC SQ14 bit 3 */
#define ADC_SQR3_SQ14_4     ((uint32_t)0x10000000) /*!< ADC SQ14 bit 4 */

/********************  Bit definition for ADC_SQR4 register  ********************/
#define ADC_SQR4_SQ15       ((uint32_t)0x0000001F) /*!< ADC 15th conversion in regular sequence */
#define ADC_SQR4_SQ15_0     ((uint32_t)0x00000001) /*!< ADC SQ15 bit 0 */
#define ADC_SQR4_SQ15_1     ((uint32_t)0x00000002) /*!< ADC SQ15 bit 1 */
#define ADC_SQR4_SQ15_2     ((uint32_t)0x00000004) /*!< ADC SQ15 bit 2 */
#define ADC_SQR4_SQ15_3     ((uint32_t)0x00000008) /*!< ADC SQ15 bit 3 */
#define ADC_SQR4_SQ15_4     ((uint32_t)0x00000010) /*!< ADC SQ105 bit 4 */

#define ADC_SQR4_SQ16       ((uint32_t)0x000007C0) /*!< ADC 16th conversion in regular sequence */
#define ADC_SQR4_SQ16_0     ((uint32_t)0x00000040) /*!< ADC SQ16 bit 0 */
#define ADC_SQR4_SQ16_1     ((uint32_t)0x00000080) /*!< ADC SQ16 bit 1 */
#define ADC_SQR4_SQ16_2     ((uint32_t)0x00000100) /*!< ADC SQ16 bit 2 */
#define ADC_SQR4_SQ16_3     ((uint32_t)0x00000200) /*!< ADC SQ16 bit 3 */
#define ADC_SQR4_SQ16_4     ((uint32_t)0x00000400) /*!< ADC SQ16 bit 4 */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_RDATA        ((uint32_t)0x0000FFFF) /*!< ADC regular Data converted */
#define ADC_DR_RDATA_0      ((uint32_t)0x00000001) /*!< ADC RDATA bit 0 */
#define ADC_DR_RDATA_1      ((uint32_t)0x00000002) /*!< ADC RDATA bit 1 */
#define ADC_DR_RDATA_2      ((uint32_t)0x00000004) /*!< ADC RDATA bit 2 */
#define ADC_DR_RDATA_3      ((uint32_t)0x00000008) /*!< ADC RDATA bit 3 */
#define ADC_DR_RDATA_4      ((uint32_t)0x00000010) /*!< ADC RDATA bit 4 */
#define ADC_DR_RDATA_5      ((uint32_t)0x00000020) /*!< ADC RDATA bit 5 */
#define ADC_DR_RDATA_6      ((uint32_t)0x00000040) /*!< ADC RDATA bit 6 */
#define ADC_DR_RDATA_7      ((uint32_t)0x00000080) /*!< ADC RDATA bit 7 */
#define ADC_DR_RDATA_8      ((uint32_t)0x00000100) /*!< ADC RDATA bit 8 */
#define ADC_DR_RDATA_9      ((uint32_t)0x00000200) /*!< ADC RDATA bit 9 */
#define ADC_DR_RDATA_10     ((uint32_t)0x00000400) /*!< ADC RDATA bit 10 */
#define ADC_DR_RDATA_11     ((uint32_t)0x00000800) /*!< ADC RDATA bit 11 */
#define ADC_DR_RDATA_12     ((uint32_t)0x00001000) /*!< ADC RDATA bit 12 */
#define ADC_DR_RDATA_13     ((uint32_t)0x00002000) /*!< ADC RDATA bit 13 */
#define ADC_DR_RDATA_14     ((uint32_t)0x00004000) /*!< ADC RDATA bit 14 */
#define ADC_DR_RDATA_15     ((uint32_t)0x00008000) /*!< ADC RDATA bit 15 */

/********************  Bit definition for ADC_JSQR register  ********************/
#define ADC_JSQR_JL         ((uint32_t)0x00000003) /*!< ADC injected channel sequence length */
#define ADC_JSQR_JL_0       ((uint32_t)0x00000001) /*!< ADC JL bit 0 */
#define ADC_JSQR_JL_1       ((uint32_t)0x00000002) /*!< ADC JL bit 1 */

#define ADC_JSQR_JEXTSEL    ((uint32_t)0x0000003C) /*!< ADC external trigger selection for injected group */
#define ADC_JSQR_JEXTSEL_0  ((uint32_t)0x00000004) /*!< ADC JEXTSEL bit 0 */
#define ADC_JSQR_JEXTSEL_1  ((uint32_t)0x00000008) /*!< ADC JEXTSEL bit 1 */
#define ADC_JSQR_JEXTSEL_2  ((uint32_t)0x00000010) /*!< ADC JEXTSEL bit 2 */
#define ADC_JSQR_JEXTSEL_3  ((uint32_t)0x00000020) /*!< ADC JEXTSEL bit 3 */

#define ADC_JSQR_JEXTEN     ((uint32_t)0x000000C0) /*!< ADC external trigger enable and polarity selection for injected channels */
#define ADC_JSQR_JEXTEN_0   ((uint32_t)0x00000040) /*!< ADC JEXTEN bit 0 */
#define ADC_JSQR_JEXTEN_1   ((uint32_t)0x00000080) /*!< ADC JEXTEN bit 1 */

#define ADC_JSQR_JSQ1       ((uint32_t)0x00001F00) /*!< ADC 1st conversion in injected sequence */
#define ADC_JSQR_JSQ1_0     ((uint32_t)0x00000100) /*!< ADC JSQ1 bit 0 */
#define ADC_JSQR_JSQ1_1     ((uint32_t)0x00000200) /*!< ADC JSQ1 bit 1 */
#define ADC_JSQR_JSQ1_2     ((uint32_t)0x00000400) /*!< ADC JSQ1 bit 2 */
#define ADC_JSQR_JSQ1_3     ((uint32_t)0x00000800) /*!< ADC JSQ1 bit 3 */
#define ADC_JSQR_JSQ1_4     ((uint32_t)0x00001000) /*!< ADC JSQ1 bit 4 */

#define ADC_JSQR_JSQ2       ((uint32_t)0x0007C000) /*!< ADC 2nd conversion in injected sequence */
#define ADC_JSQR_JSQ2_0     ((uint32_t)0x00004000) /*!< ADC JSQ2 bit 0 */
#define ADC_JSQR_JSQ2_1     ((uint32_t)0x00008000) /*!< ADC JSQ2 bit 1 */
#define ADC_JSQR_JSQ2_2     ((uint32_t)0x00010000) /*!< ADC JSQ2 bit 2 */
#define ADC_JSQR_JSQ2_3     ((uint32_t)0x00020000) /*!< ADC JSQ2 bit 3 */
#define ADC_JSQR_JSQ2_4     ((uint32_t)0x00040000) /*!< ADC JSQ2 bit 4 */

#define ADC_JSQR_JSQ3       ((uint32_t)0x01F00000) /*!< ADC 3rd conversion in injected sequence */
#define ADC_JSQR_JSQ3_0     ((uint32_t)0x00100000) /*!< ADC JSQ3 bit 0 */
#define ADC_JSQR_JSQ3_1     ((uint32_t)0x00200000) /*!< ADC JSQ3 bit 1 */
#define ADC_JSQR_JSQ3_2     ((uint32_t)0x00400000) /*!< ADC JSQ3 bit 2 */
#define ADC_JSQR_JSQ3_3     ((uint32_t)0x00800000) /*!< ADC JSQ3 bit 3 */
#define ADC_JSQR_JSQ3_4     ((uint32_t)0x01000000) /*!< ADC JSQ3 bit 4 */

#define ADC_JSQR_JSQ4       ((uint32_t)0x7C000000) /*!< ADC 4th conversion in injected sequence */
#define ADC_JSQR_JSQ4_0     ((uint32_t)0x04000000) /*!< ADC JSQ4 bit 0 */
#define ADC_JSQR_JSQ4_1     ((uint32_t)0x08000000) /*!< ADC JSQ4 bit 1 */
#define ADC_JSQR_JSQ4_2     ((uint32_t)0x10000000) /*!< ADC JSQ4 bit 2 */
#define ADC_JSQR_JSQ4_3     ((uint32_t)0x20000000) /*!< ADC JSQ4 bit 3 */
#define ADC_JSQR_JSQ4_4     ((uint32_t)0x40000000) /*!< ADC JSQ4 bit 4 */


/********************  Bit definition for ADC_OFR1 register  ********************/
#define ADC_OFR1_OFFSET1    ((uint32_t)0x00000FFF) /*!< ADC data offset 1 for channel programmed into bits OFFSET1_CH[4:0] */
#define ADC_OFR1_OFFSET1_0  ((uint32_t)0x00000001) /*!< ADC OFFSET1 bit 0 */
#define ADC_OFR1_OFFSET1_1  ((uint32_t)0x00000002) /*!< ADC OFFSET1 bit 1 */
#define ADC_OFR1_OFFSET1_2  ((uint32_t)0x00000004) /*!< ADC OFFSET1 bit 2 */
#define ADC_OFR1_OFFSET1_3  ((uint32_t)0x00000008) /*!< ADC OFFSET1 bit 3 */
#define ADC_OFR1_OFFSET1_4  ((uint32_t)0x00000010) /*!< ADC OFFSET1 bit 4 */
#define ADC_OFR1_OFFSET1_5  ((uint32_t)0x00000020) /*!< ADC OFFSET1 bit 5 */
#define ADC_OFR1_OFFSET1_6  ((uint32_t)0x00000040) /*!< ADC OFFSET1 bit 6 */
#define ADC_OFR1_OFFSET1_7  ((uint32_t)0x00000080) /*!< ADC OFFSET1 bit 7 */
#define ADC_OFR1_OFFSET1_8  ((uint32_t)0x00000100) /*!< ADC OFFSET1 bit 8 */
#define ADC_OFR1_OFFSET1_9  ((uint32_t)0x00000200) /*!< ADC OFFSET1 bit 9 */
#define ADC_OFR1_OFFSET1_10 ((uint32_t)0x00000400) /*!< ADC OFFSET1 bit 10 */
#define ADC_OFR1_OFFSET1_11 ((uint32_t)0x00000800) /*!< ADC OFFSET1 bit 11 */

#define ADC_OFR1_OFFSET1_CH    ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 1 */
#define ADC_OFR1_OFFSET1_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET1_CH bit 0 */
#define ADC_OFR1_OFFSET1_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET1_CH bit 1 */
#define ADC_OFR1_OFFSET1_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET1_CH bit 2 */
#define ADC_OFR1_OFFSET1_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET1_CH bit 3 */
#define ADC_OFR1_OFFSET1_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET1_CH bit 4 */

#define ADC_OFR1_OFFSET1_EN    ((uint32_t)0x80000000) /*!< ADC offset 1 enable */

/********************  Bit definition for ADC_OFR2 register  ********************/
#define ADC_OFR2_OFFSET2    ((uint32_t)0x00000FFF) /*!< ADC data offset 2 for channel programmed into bits OFFSET2_CH[4:0] */
#define ADC_OFR2_OFFSET2_0  ((uint32_t)0x00000001) /*!< ADC OFFSET2 bit 0 */
#define ADC_OFR2_OFFSET2_1  ((uint32_t)0x00000002) /*!< ADC OFFSET2 bit 1 */
#define ADC_OFR2_OFFSET2_2  ((uint32_t)0x00000004) /*!< ADC OFFSET2 bit 2 */
#define ADC_OFR2_OFFSET2_3  ((uint32_t)0x00000008) /*!< ADC OFFSET2 bit 3 */
#define ADC_OFR2_OFFSET2_4  ((uint32_t)0x00000010) /*!< ADC OFFSET2 bit 4 */
#define ADC_OFR2_OFFSET2_5  ((uint32_t)0x00000020) /*!< ADC OFFSET2 bit 5 */
#define ADC_OFR2_OFFSET2_6  ((uint32_t)0x00000040) /*!< ADC OFFSET2 bit 6 */
#define ADC_OFR2_OFFSET2_7  ((uint32_t)0x00000080) /*!< ADC OFFSET2 bit 7 */
#define ADC_OFR2_OFFSET2_8  ((uint32_t)0x00000100) /*!< ADC OFFSET2 bit 8 */
#define ADC_OFR2_OFFSET2_9  ((uint32_t)0x00000200) /*!< ADC OFFSET2 bit 9 */
#define ADC_OFR2_OFFSET2_10 ((uint32_t)0x00000400) /*!< ADC OFFSET2 bit 10 */
#define ADC_OFR2_OFFSET2_11 ((uint32_t)0x00000800) /*!< ADC OFFSET2 bit 11 */

#define ADC_OFR2_OFFSET2_CH    ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 2 */
#define ADC_OFR2_OFFSET2_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET2_CH bit 0 */
#define ADC_OFR2_OFFSET2_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET2_CH bit 1 */
#define ADC_OFR2_OFFSET2_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET2_CH bit 2 */
#define ADC_OFR2_OFFSET2_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET2_CH bit 3 */
#define ADC_OFR2_OFFSET2_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET2_CH bit 4 */

#define ADC_OFR2_OFFSET2_EN    ((uint32_t)0x80000000) /*!< ADC offset 2 enable */

/********************  Bit definition for ADC_OFR3 register  ********************/
#define ADC_OFR3_OFFSET3    ((uint32_t)0x00000FFF) /*!< ADC data offset 3 for channel programmed into bits OFFSET3_CH[4:0] */
#define ADC_OFR3_OFFSET3_0  ((uint32_t)0x00000001) /*!< ADC OFFSET3 bit 0 */
#define ADC_OFR3_OFFSET3_1  ((uint32_t)0x00000002) /*!< ADC OFFSET3 bit 1 */
#define ADC_OFR3_OFFSET3_2  ((uint32_t)0x00000004) /*!< ADC OFFSET3 bit 2 */
#define ADC_OFR3_OFFSET3_3  ((uint32_t)0x00000008) /*!< ADC OFFSET3 bit 3 */
#define ADC_OFR3_OFFSET3_4  ((uint32_t)0x00000010) /*!< ADC OFFSET3 bit 4 */
#define ADC_OFR3_OFFSET3_5  ((uint32_t)0x00000020) /*!< ADC OFFSET3 bit 5 */
#define ADC_OFR3_OFFSET3_6  ((uint32_t)0x00000040) /*!< ADC OFFSET3 bit 6 */
#define ADC_OFR3_OFFSET3_7  ((uint32_t)0x00000080) /*!< ADC OFFSET3 bit 7 */
#define ADC_OFR3_OFFSET3_8  ((uint32_t)0x00000100) /*!< ADC OFFSET3 bit 8 */
#define ADC_OFR3_OFFSET3_9  ((uint32_t)0x00000200) /*!< ADC OFFSET3 bit 9 */
#define ADC_OFR3_OFFSET3_10 ((uint32_t)0x00000400) /*!< ADC OFFSET3 bit 10 */
#define ADC_OFR3_OFFSET3_11 ((uint32_t)0x00000800) /*!< ADC OFFSET3 bit 11 */

#define ADC_OFR3_OFFSET3_CH    ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 3 */
#define ADC_OFR3_OFFSET3_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET3_CH bit 0 */
#define ADC_OFR3_OFFSET3_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET3_CH bit 1 */
#define ADC_OFR3_OFFSET3_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET3_CH bit 2 */
#define ADC_OFR3_OFFSET3_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET3_CH bit 3 */
#define ADC_OFR3_OFFSET3_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET3_CH bit 4 */

#define ADC_OFR3_OFFSET3_EN    ((uint32_t)0x80000000) /*!< ADC offset 3 enable */

/********************  Bit definition for ADC_OFR4 register  ********************/
#define ADC_OFR4_OFFSET4    ((uint32_t)0x00000FFF) /*!< ADC data offset 4 for channel programmed into bits OFFSET4_CH[4:0] */
#define ADC_OFR4_OFFSET4_0  ((uint32_t)0x00000001) /*!< ADC OFFSET4 bit 0 */
#define ADC_OFR4_OFFSET4_1  ((uint32_t)0x00000002) /*!< ADC OFFSET4 bit 1 */
#define ADC_OFR4_OFFSET4_2  ((uint32_t)0x00000004) /*!< ADC OFFSET4 bit 2 */
#define ADC_OFR4_OFFSET4_3  ((uint32_t)0x00000008) /*!< ADC OFFSET4 bit 3 */
#define ADC_OFR4_OFFSET4_4  ((uint32_t)0x00000010) /*!< ADC OFFSET4 bit 4 */
#define ADC_OFR4_OFFSET4_5  ((uint32_t)0x00000020) /*!< ADC OFFSET4 bit 5 */
#define ADC_OFR4_OFFSET4_6  ((uint32_t)0x00000040) /*!< ADC OFFSET4 bit 6 */
#define ADC_OFR4_OFFSET4_7  ((uint32_t)0x00000080) /*!< ADC OFFSET4 bit 7 */
#define ADC_OFR4_OFFSET4_8  ((uint32_t)0x00000100) /*!< ADC OFFSET4 bit 8 */
#define ADC_OFR4_OFFSET4_9  ((uint32_t)0x00000200) /*!< ADC OFFSET4 bit 9 */
#define ADC_OFR4_OFFSET4_10 ((uint32_t)0x00000400) /*!< ADC OFFSET4 bit 10 */
#define ADC_OFR4_OFFSET4_11 ((uint32_t)0x00000800) /*!< ADC OFFSET4 bit 11 */

#define ADC_OFR4_OFFSET4_CH    ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 4 */
#define ADC_OFR4_OFFSET4_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET4_CH bit 0 */
#define ADC_OFR4_OFFSET4_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET4_CH bit 1 */
#define ADC_OFR4_OFFSET4_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET4_CH bit 2 */
#define ADC_OFR4_OFFSET4_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET4_CH bit 3 */
#define ADC_OFR4_OFFSET4_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET4_CH bit 4 */

#define ADC_OFR4_OFFSET4_EN    ((uint32_t)0x80000000) /*!< ADC offset 4 enable */

/********************  Bit definition for ADC_JDR1 register  ********************/
#define ADC_JDR1_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR1_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR1_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR1_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR1_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR1_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR1_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR1_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR1_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR1_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR1_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR1_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR1_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR1_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR1_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR1_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR1_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR2 register  ********************/
#define ADC_JDR2_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR2_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR2_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR2_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR2_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR2_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR2_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR2_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR2_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR2_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR2_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR2_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR2_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR2_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR2_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR2_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR2_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR3 register  ********************/
#define ADC_JDR3_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR3_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR3_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR3_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR3_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR3_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR3_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR3_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR3_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR3_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR3_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR3_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR3_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR3_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR3_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR3_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR3_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR4 register  ********************/
#define ADC_JDR4_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR4_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR4_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR4_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR4_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR4_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR4_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR4_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR4_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR4_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR4_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR4_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR4_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR4_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR4_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR4_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR4_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_AWD2CR register  ********************/
#define ADC_AWD2CR_AWD2CH    ((uint32_t)0x0007FFFF) /*!< ADC Analog watchdog 2 channel selection */
#define ADC_AWD2CR_AWD2CH_0  ((uint32_t)0x00000001) /*!< ADC AWD2CH bit 0 */
#define ADC_AWD2CR_AWD2CH_1  ((uint32_t)0x00000002) /*!< ADC AWD2CH bit 1 */
#define ADC_AWD2CR_AWD2CH_2  ((uint32_t)0x00000004) /*!< ADC AWD2CH bit 2 */
#define ADC_AWD2CR_AWD2CH_3  ((uint32_t)0x00000008) /*!< ADC AWD2CH bit 3 */
#define ADC_AWD2CR_AWD2CH_4  ((uint32_t)0x00000010) /*!< ADC AWD2CH bit 4 */
#define ADC_AWD2CR_AWD2CH_5  ((uint32_t)0x00000020) /*!< ADC AWD2CH bit 5 */
#define ADC_AWD2CR_AWD2CH_6  ((uint32_t)0x00000040) /*!< ADC AWD2CH bit 6 */
#define ADC_AWD2CR_AWD2CH_7  ((uint32_t)0x00000080) /*!< ADC AWD2CH bit 7 */
#define ADC_AWD2CR_AWD2CH_8  ((uint32_t)0x00000100) /*!< ADC AWD2CH bit 8 */
#define ADC_AWD2CR_AWD2CH_9  ((uint32_t)0x00000200) /*!< ADC AWD2CH bit 9 */
#define ADC_AWD2CR_AWD2CH_10 ((uint32_t)0x00000400) /*!< ADC AWD2CH bit 10 */
#define ADC_AWD2CR_AWD2CH_11 ((uint32_t)0x00000800) /*!< ADC AWD2CH bit 11 */
#define ADC_AWD2CR_AWD2CH_12 ((uint32_t)0x00001000) /*!< ADC AWD2CH bit 12 */
#define ADC_AWD2CR_AWD2CH_13 ((uint32_t)0x00002000) /*!< ADC AWD2CH bit 13 */
#define ADC_AWD2CR_AWD2CH_14 ((uint32_t)0x00004000) /*!< ADC AWD2CH bit 14 */
#define ADC_AWD2CR_AWD2CH_15 ((uint32_t)0x00008000) /*!< ADC AWD2CH bit 15 */
#define ADC_AWD2CR_AWD2CH_16 ((uint32_t)0x00010000) /*!< ADC AWD2CH bit 16 */
#define ADC_AWD2CR_AWD2CH_17 ((uint32_t)0x00020000) /*!< ADC AWD2CH bit 17 */
#define ADC_AWD2CR_AWD2CH_18 ((uint32_t)0x00040000) /*!< ADC AWD2CH bit 18 */

/********************  Bit definition for ADC_AWD3CR register  ********************/
#define ADC_AWD3CR_AWD3CH    ((uint32_t)0x0007FFFF) /*!< ADC Analog watchdog 3 channel selection */
#define ADC_AWD3CR_AWD3CH_0  ((uint32_t)0x00000001) /*!< ADC AWD3CH bit 0 */
#define ADC_AWD3CR_AWD3CH_1  ((uint32_t)0x00000002) /*!< ADC AWD3CH bit 1 */
#define ADC_AWD3CR_AWD3CH_2  ((uint32_t)0x00000004) /*!< ADC AWD3CH bit 2 */
#define ADC_AWD3CR_AWD3CH_3  ((uint32_t)0x00000008) /*!< ADC AWD3CH bit 3 */
#define ADC_AWD3CR_AWD3CH_4  ((uint32_t)0x00000010) /*!< ADC AWD3CH bit 4 */
#define ADC_AWD3CR_AWD3CH_5  ((uint32_t)0x00000020) /*!< ADC AWD3CH bit 5 */
#define ADC_AWD3CR_AWD3CH_6  ((uint32_t)0x00000040) /*!< ADC AWD3CH bit 6 */
#define ADC_AWD3CR_AWD3CH_7  ((uint32_t)0x00000080) /*!< ADC AWD3CH bit 7 */
#define ADC_AWD3CR_AWD3CH_8  ((uint32_t)0x00000100) /*!< ADC AWD3CH bit 8 */
#define ADC_AWD3CR_AWD3CH_9  ((uint32_t)0x00000200) /*!< ADC AWD3CH bit 9 */
#define ADC_AWD3CR_AWD3CH_10 ((uint32_t)0x00000400) /*!< ADC AWD3CH bit 10 */
#define ADC_AWD3CR_AWD3CH_11 ((uint32_t)0x00000800) /*!< ADC AWD3CH bit 11 */
#define ADC_AWD3CR_AWD3CH_12 ((uint32_t)0x00001000) /*!< ADC AWD3CH bit 12 */
#define ADC_AWD3CR_AWD3CH_13 ((uint32_t)0x00002000) /*!< ADC AWD3CH bit 13 */
#define ADC_AWD3CR_AWD3CH_14 ((uint32_t)0x00004000) /*!< ADC AWD3CH bit 14 */
#define ADC_AWD3CR_AWD3CH_15 ((uint32_t)0x00008000) /*!< ADC AWD3CH bit 15 */
#define ADC_AWD3CR_AWD3CH_16 ((uint32_t)0x00010000) /*!< ADC AWD3CH bit 16 */
#define ADC_AWD3CR_AWD3CH_17 ((uint32_t)0x00020000) /*!< ADC AWD3CH bit 17 */
#define ADC_AWD3CR_AWD3CH_18 ((uint32_t)0x00040000) /*!< ADC AWD3CH bit 18 */

/********************  Bit definition for ADC_DIFSEL register  ********************/
#define ADC_DIFSEL_DIFSEL    ((uint32_t)0x0007FFFF) /*!< ADC differential modes for channels 1 to 18 */
#define ADC_DIFSEL_DIFSEL_0  ((uint32_t)0x00000001) /*!< ADC DIFSEL bit 0 */
#define ADC_DIFSEL_DIFSEL_1  ((uint32_t)0x00000002) /*!< ADC DIFSEL bit 1 */
#define ADC_DIFSEL_DIFSEL_2  ((uint32_t)0x00000004) /*!< ADC DIFSEL bit 2 */
#define ADC_DIFSEL_DIFSEL_3  ((uint32_t)0x00000008) /*!< ADC DIFSEL bit 3 */
#define ADC_DIFSEL_DIFSEL_4  ((uint32_t)0x00000010) /*!< ADC DIFSEL bit 4 */
#define ADC_DIFSEL_DIFSEL_5  ((uint32_t)0x00000020) /*!< ADC DIFSEL bit 5 */
#define ADC_DIFSEL_DIFSEL_6  ((uint32_t)0x00000040) /*!< ADC DIFSEL bit 6 */
#define ADC_DIFSEL_DIFSEL_7  ((uint32_t)0x00000080) /*!< ADC DIFSEL bit 7 */
#define ADC_DIFSEL_DIFSEL_8  ((uint32_t)0x00000100) /*!< ADC DIFSEL bit 8 */
#define ADC_DIFSEL_DIFSEL_9  ((uint32_t)0x00000200) /*!< ADC DIFSEL bit 9 */
#define ADC_DIFSEL_DIFSEL_10 ((uint32_t)0x00000400) /*!< ADC DIFSEL bit 10 */
#define ADC_DIFSEL_DIFSEL_11 ((uint32_t)0x00000800) /*!< ADC DIFSEL bit 11 */
#define ADC_DIFSEL_DIFSEL_12 ((uint32_t)0x00001000) /*!< ADC DIFSEL bit 12 */
#define ADC_DIFSEL_DIFSEL_13 ((uint32_t)0x00002000) /*!< ADC DIFSEL bit 13 */
#define ADC_DIFSEL_DIFSEL_14 ((uint32_t)0x00004000) /*!< ADC DIFSEL bit 14 */
#define ADC_DIFSEL_DIFSEL_15 ((uint32_t)0x00008000) /*!< ADC DIFSEL bit 15 */
#define ADC_DIFSEL_DIFSEL_16 ((uint32_t)0x00010000) /*!< ADC DIFSEL bit 16 */
#define ADC_DIFSEL_DIFSEL_17 ((uint32_t)0x00020000) /*!< ADC DIFSEL bit 17 */
#define ADC_DIFSEL_DIFSEL_18 ((uint32_t)0x00040000) /*!< ADC DIFSEL bit 18 */

/********************  Bit definition for ADC_CALFACT register  ********************/
#define ADC_CALFACT_CALFACT_S   ((uint32_t)0x0000007F) /*!< ADC calibration factors in single-ended mode */
#define ADC_CALFACT_CALFACT_S_0 ((uint32_t)0x00000001) /*!< ADC CALFACT_S bit 0 */
#define ADC_CALFACT_CALFACT_S_1 ((uint32_t)0x00000002) /*!< ADC CALFACT_S bit 1 */
#define ADC_CALFACT_CALFACT_S_2 ((uint32_t)0x00000004) /*!< ADC CALFACT_S bit 2 */
#define ADC_CALFACT_CALFACT_S_3 ((uint32_t)0x00000008) /*!< ADC CALFACT_S bit 3 */
#define ADC_CALFACT_CALFACT_S_4 ((uint32_t)0x00000010) /*!< ADC CALFACT_S bit 4 */
#define ADC_CALFACT_CALFACT_S_5 ((uint32_t)0x00000020) /*!< ADC CALFACT_S bit 5 */
#define ADC_CALFACT_CALFACT_S_6 ((uint32_t)0x00000040) /*!< ADC CALFACT_S bit 6 */

#define ADC_CALFACT_CALFACT_D   ((uint32_t)0x007F0000) /*!< ADC calibration factors in differential mode */
#define ADC_CALFACT_CALFACT_D_0 ((uint32_t)0x00010000) /*!< ADC CALFACT_D bit 0 */
#define ADC_CALFACT_CALFACT_D_1 ((uint32_t)0x00020000) /*!< ADC CALFACT_D bit 1 */
#define ADC_CALFACT_CALFACT_D_2 ((uint32_t)0x00040000) /*!< ADC CALFACT_D bit 2 */
#define ADC_CALFACT_CALFACT_D_3 ((uint32_t)0x00080000) /*!< ADC CALFACT_D bit 3 */
#define ADC_CALFACT_CALFACT_D_4 ((uint32_t)0x00100000) /*!< ADC CALFACT_D bit 4 */
#define ADC_CALFACT_CALFACT_D_5 ((uint32_t)0x00200000) /*!< ADC CALFACT_D bit 5 */
#define ADC_CALFACT_CALFACT_D_6 ((uint32_t)0x00400000) /*!< ADC CALFACT_D bit 6 */

/*************************  ADC Common registers  *****************************/
/********************  Bit definition for ADC_CSR register  ********************/
#define ADC_CSR_ADRDY_MST         ((uint32_t)0x00000001) /*!< Master ADC ready */
#define ADC_CSR_EOSMP_MST         ((uint32_t)0x00000002) /*!< End of sampling phase flag of the master ADC */
#define ADC_CSR_EOC_MST           ((uint32_t)0x00000004) /*!< End of regular conversion of the master ADC */
#define ADC_CSR_EOS_MST           ((uint32_t)0x00000008) /*!< End of regular sequence flag of the master ADC */
#define ADC_CSR_OVR_MST           ((uint32_t)0x00000010) /*!< Overrun flag of the master ADC */
#define ADC_CSR_JEOC_MST          ((uint32_t)0x00000020) /*!< End of injected conversion of the master ADC */
#define ADC_CSR_JEOS_MST          ((uint32_t)0x00000040) /*!< End of injected sequence flag of the master ADC */
#define ADC_CSR_AWD1_MST          ((uint32_t)0x00000080) /*!< Analog watchdog 1 flag of the master ADC */
#define ADC_CSR_AWD2_MST          ((uint32_t)0x00000100) /*!< Analog watchdog 2 flag of the master ADC */
#define ADC_CSR_AWD3_MST          ((uint32_t)0x00000200) /*!< Analog watchdog 3 flag of the master ADC */
#define ADC_CSR_JQOVF_MST         ((uint32_t)0x00000400) /*!< Injected context queue overflow flag of the master ADC */

#define ADC_CSR_ADRDY_SLV         ((uint32_t)0x00010000) /*!< Slave ADC ready */
#define ADC_CSR_EOSMP_SLV         ((uint32_t)0x00020000) /*!< End of sampling phase flag of the slave ADC */
#define ADC_CSR_EOC_SLV           ((uint32_t)0x00040000) /*!< End of regular conversion of the slave ADC */
#define ADC_CSR_EOS_SLV           ((uint32_t)0x00080000) /*!< End of regular sequence flag of the slave ADC */
#define ADC_CSR_OVR_SLV           ((uint32_t)0x00100000) /*!< Overrun flag of the slave ADC */
#define ADC_CSR_JEOC_SLV          ((uint32_t)0x00200000) /*!< End of injected conversion of the slave ADC */
#define ADC_CSR_JEOS_SLV          ((uint32_t)0x00400000) /*!< End of injected sequence flag of the slave ADC */
#define ADC_CSR_AWD1_SLV          ((uint32_t)0x00800000) /*!< Analog watchdog 1 flag of the slave ADC */
#define ADC_CSR_AWD2_SLV          ((uint32_t)0x01000000) /*!< Analog watchdog 2 flag of the slave ADC */
#define ADC_CSR_AWD3_SLV          ((uint32_t)0x02000000) /*!< Analog watchdog 3 flag of the slave ADC */
#define ADC_CSR_JQOVF_SLV         ((uint32_t)0x04000000) /*!< Injected context queue overflow flag of the slave ADC */

/********************  Bit definition for ADC_CCR register  ********************/
#define ADC_CCR_DUAL             ((uint32_t)0x0000001F) /*!< Dual ADC mode selection */
#define ADC_CCR_DUAL_0           ((uint32_t)0x00000001) /*!< Dual bit 0 */
#define ADC_CCR_DUAL_1           ((uint32_t)0x00000002) /*!< Dual bit 1 */
#define ADC_CCR_DUAL_2           ((uint32_t)0x00000004) /*!< Dual bit 2 */
#define ADC_CCR_DUAL_3           ((uint32_t)0x00000008) /*!< Dual bit 3 */
#define ADC_CCR_DUAL_4           ((uint32_t)0x00000010) /*!< Dual bit 4 */

#define ADC_CCR_DELAY             ((uint32_t)0x00000F00) /*!< Delay between 2 sampling phases */
#define ADC_CCR_DELAY_0           ((uint32_t)0x00000100) /*!< DELAY bit 0 */
#define ADC_CCR_DELAY_1           ((uint32_t)0x00000200) /*!< DELAY bit 1 */
#define ADC_CCR_DELAY_2           ((uint32_t)0x00000400) /*!< DELAY bit 2 */
#define ADC_CCR_DELAY_3           ((uint32_t)0x00000800) /*!< DELAY bit 3 */

#define ADC_CCR_DMACFG            ((uint32_t)0x00002000) /*!< DMA configuration for multi-ADC mode */

#define ADC_CCR_MDMA              ((uint32_t)0x0000C000) /*!< DMA mode for multi-ADC mode */
#define ADC_CCR_MDMA_0            ((uint32_t)0x00004000) /*!< MDMA bit 0 */
#define ADC_CCR_MDMA_1            ((uint32_t)0x00008000) /*!< MDMA bit 1 */

#define ADC_CCR_CKMODE            ((uint32_t)0x00030000) /*!< ADC clock mode */
#define ADC_CCR_CKMODE_0          ((uint32_t)0x00010000) /*!< CKMODE bit 0 */
#define ADC_CCR_CKMODE_1          ((uint32_t)0x00020000) /*!< CKMODE bit 1 */

#define ADC_CCR_PRESC             ((uint32_t)0x003C0000) /*!< ADC prescaler */
#define ADC_CCR_PRESC_0           ((uint32_t)0x00040000) /*!< ADC prescaler bit 0 */
#define ADC_CCR_PRESC_1           ((uint32_t)0x00080000) /*!< ADC prescaler bit 1 */
#define ADC_CCR_PRESC_2           ((uint32_t)0x00100000) /*!< ADC prescaler bit 2 */
#define ADC_CCR_PRESC_3           ((uint32_t)0x00200000) /*!< ADC prescaler bit 3 */

#define ADC_CCR_VREFEN            ((uint32_t)0x00400000) /*!< VREFINT enable */
#define ADC_CCR_TSEN              ((uint32_t)0x00800000) /*!< Temperature sensor enable */
#define ADC_CCR_VBATEN            ((uint32_t)0x01000000) /*!< VBAT enable */

/********************  Bit definition for ADC_CDR register  ********************/
#define ADC_CDR_RDATA_MST         ((uint32_t)0x0000FFFF) /*!< Regular Data of the master ADC */
#define ADC_CDR_RDATA_MST_0       ((uint32_t)0x00000001) /*!< RDATA_MST bit 0 */
#define ADC_CDR_RDATA_MST_1       ((uint32_t)0x00000002) /*!< RDATA_MST bit 1 */
#define ADC_CDR_RDATA_MST_2       ((uint32_t)0x00000004) /*!< RDATA_MST bit 2 */
#define ADC_CDR_RDATA_MST_3       ((uint32_t)0x00000008) /*!< RDATA_MST bit 3 */
#define ADC_CDR_RDATA_MST_4       ((uint32_t)0x00000010) /*!< RDATA_MST bit 4 */
#define ADC_CDR_RDATA_MST_5       ((uint32_t)0x00000020) /*!< RDATA_MST bit 5 */
#define ADC_CDR_RDATA_MST_6       ((uint32_t)0x00000040) /*!< RDATA_MST bit 6 */
#define ADC_CDR_RDATA_MST_7       ((uint32_t)0x00000080) /*!< RDATA_MST bit 7 */
#define ADC_CDR_RDATA_MST_8       ((uint32_t)0x00000100) /*!< RDATA_MST bit 8 */
#define ADC_CDR_RDATA_MST_9       ((uint32_t)0x00000200) /*!< RDATA_MST bit 9 */
#define ADC_CDR_RDATA_MST_10      ((uint32_t)0x00000400) /*!< RDATA_MST bit 10 */
#define ADC_CDR_RDATA_MST_11      ((uint32_t)0x00000800) /*!< RDATA_MST bit 11 */
#define ADC_CDR_RDATA_MST_12      ((uint32_t)0x00001000) /*!< RDATA_MST bit 12 */
#define ADC_CDR_RDATA_MST_13      ((uint32_t)0x00002000) /*!< RDATA_MST bit 13 */
#define ADC_CDR_RDATA_MST_14      ((uint32_t)0x00004000) /*!< RDATA_MST bit 14 */
#define ADC_CDR_RDATA_MST_15      ((uint32_t)0x00008000) /*!< RDATA_MST bit 15 */

#define ADC_CDR_RDATA_SLV         ((uint32_t)0xFFFF0000) /*!< Regular Data of the master ADC */
#define ADC_CDR_RDATA_SLV_0       ((uint32_t)0x00010000) /*!< RDATA_SLV bit 0 */
#define ADC_CDR_RDATA_SLV_1       ((uint32_t)0x00020000) /*!< RDATA_SLV bit 1 */
#define ADC_CDR_RDATA_SLV_2       ((uint32_t)0x00040000) /*!< RDATA_SLV bit 2 */
#define ADC_CDR_RDATA_SLV_3       ((uint32_t)0x00080000) /*!< RDATA_SLV bit 3 */
#define ADC_CDR_RDATA_SLV_4       ((uint32_t)0x00100000) /*!< RDATA_SLV bit 4 */
#define ADC_CDR_RDATA_SLV_5       ((uint32_t)0x00200000) /*!< RDATA_SLV bit 5 */
#define ADC_CDR_RDATA_SLV_6       ((uint32_t)0x00400000) /*!< RDATA_SLV bit 6 */
#define ADC_CDR_RDATA_SLV_7       ((uint32_t)0x00800000) /*!< RDATA_SLV bit 7 */
#define ADC_CDR_RDATA_SLV_8       ((uint32_t)0x01000000) /*!< RDATA_SLV bit 8 */
#define ADC_CDR_RDATA_SLV_9       ((uint32_t)0x02000000) /*!< RDATA_SLV bit 9 */
#define ADC_CDR_RDATA_SLV_10      ((uint32_t)0x04000000) /*!< RDATA_SLV bit 10 */
#define ADC_CDR_RDATA_SLV_11      ((uint32_t)0x08000000) /*!< RDATA_SLV bit 11 */
#define ADC_CDR_RDATA_SLV_12      ((uint32_t)0x10000000) /*!< RDATA_SLV bit 12 */
#define ADC_CDR_RDATA_SLV_13      ((uint32_t)0x20000000) /*!< RDATA_SLV bit 13 */
#define ADC_CDR_RDATA_SLV_14      ((uint32_t)0x40000000) /*!< RDATA_SLV bit 14 */
#define ADC_CDR_RDATA_SLV_15      ((uint32_t)0x80000000) /*!< RDATA_SLV bit 15 */

/******************************************************************************/
/*                                                                            */
/*                         Controller Area Network                            */
/*                                                                            */
/******************************************************************************/
/*!<CAN control and status registers */
/*******************  Bit definition for CAN_MCR register  ********************/
#define  CAN_MCR_INRQ                        ((uint16_t)0x0001)            /*!<Initialization Request */
#define  CAN_MCR_SLEEP                       ((uint16_t)0x0002)            /*!<Sleep Mode Request */
#define  CAN_MCR_TXFP                        ((uint16_t)0x0004)            /*!<Transmit FIFO Priority */
#define  CAN_MCR_RFLM                        ((uint16_t)0x0008)            /*!<Receive FIFO Locked Mode */
#define  CAN_MCR_NART                        ((uint16_t)0x0010)            /*!<No Automatic Retransmission */
#define  CAN_MCR_AWUM                        ((uint16_t)0x0020)            /*!<Automatic Wakeup Mode */
#define  CAN_MCR_ABOM                        ((uint16_t)0x0040)            /*!<Automatic Bus-Off Management */
#define  CAN_MCR_TTCM                        ((uint16_t)0x0080)            /*!<Time Triggered Communication Mode */
#define  CAN_MCR_RESET                       ((uint16_t)0x8000)            /*!<bxCAN software master reset */

/*******************  Bit definition for CAN_MSR register  ********************/
#define  CAN_MSR_INAK                        ((uint16_t)0x0001)            /*!<Initialization Acknowledge */
#define  CAN_MSR_SLAK                        ((uint16_t)0x0002)            /*!<Sleep Acknowledge */
#define  CAN_MSR_ERRI                        ((uint16_t)0x0004)            /*!<Error Interrupt */
#define  CAN_MSR_WKUI                        ((uint16_t)0x0008)            /*!<Wakeup Interrupt */
#define  CAN_MSR_SLAKI                       ((uint16_t)0x0010)            /*!<Sleep Acknowledge Interrupt */
#define  CAN_MSR_TXM                         ((uint16_t)0x0100)            /*!<Transmit Mode */
#define  CAN_MSR_RXM                         ((uint16_t)0x0200)            /*!<Receive Mode */
#define  CAN_MSR_SAMP                        ((uint16_t)0x0400)            /*!<Last Sample Point */
#define  CAN_MSR_RX                          ((uint16_t)0x0800)            /*!<CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
#define  CAN_TSR_RQCP0                       ((uint32_t)0x00000001)        /*!<Request Completed Mailbox0 */
#define  CAN_TSR_TXOK0                       ((uint32_t)0x00000002)        /*!<Transmission OK of Mailbox0 */
#define  CAN_TSR_ALST0                       ((uint32_t)0x00000004)        /*!<Arbitration Lost for Mailbox0 */
#define  CAN_TSR_TERR0                       ((uint32_t)0x00000008)        /*!<Transmission Error of Mailbox0 */
#define  CAN_TSR_ABRQ0                       ((uint32_t)0x00000080)        /*!<Abort Request for Mailbox0 */
#define  CAN_TSR_RQCP1                       ((uint32_t)0x00000100)        /*!<Request Completed Mailbox1 */
#define  CAN_TSR_TXOK1                       ((uint32_t)0x00000200)        /*!<Transmission OK of Mailbox1 */
#define  CAN_TSR_ALST1                       ((uint32_t)0x00000400)        /*!<Arbitration Lost for Mailbox1 */
#define  CAN_TSR_TERR1                       ((uint32_t)0x00000800)        /*!<Transmission Error of Mailbox1 */
#define  CAN_TSR_ABRQ1                       ((uint32_t)0x00008000)        /*!<Abort Request for Mailbox 1 */
#define  CAN_TSR_RQCP2                       ((uint32_t)0x00010000)        /*!<Request Completed Mailbox2 */
#define  CAN_TSR_TXOK2                       ((uint32_t)0x00020000)        /*!<Transmission OK of Mailbox 2 */
#define  CAN_TSR_ALST2                       ((uint32_t)0x00040000)        /*!<Arbitration Lost for mailbox 2 */
#define  CAN_TSR_TERR2                       ((uint32_t)0x00080000)        /*!<Transmission Error of Mailbox 2 */
#define  CAN_TSR_ABRQ2                       ((uint32_t)0x00800000)        /*!<Abort Request for Mailbox 2 */
#define  CAN_TSR_CODE                        ((uint32_t)0x03000000)        /*!<Mailbox Code */

#define  CAN_TSR_TME                         ((uint32_t)0x1C000000)        /*!<TME[2:0] bits */
#define  CAN_TSR_TME0                        ((uint32_t)0x04000000)        /*!<Transmit Mailbox 0 Empty */
#define  CAN_TSR_TME1                        ((uint32_t)0x08000000)        /*!<Transmit Mailbox 1 Empty */
#define  CAN_TSR_TME2                        ((uint32_t)0x10000000)        /*!<Transmit Mailbox 2 Empty */

#define  CAN_TSR_LOW                         ((uint32_t)0xE0000000)        /*!<LOW[2:0] bits */
#define  CAN_TSR_LOW0                        ((uint32_t)0x20000000)        /*!<Lowest Priority Flag for Mailbox 0 */
#define  CAN_TSR_LOW1                        ((uint32_t)0x40000000)        /*!<Lowest Priority Flag for Mailbox 1 */
#define  CAN_TSR_LOW2                        ((uint32_t)0x80000000)        /*!<Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0R register  *******************/
#define  CAN_RF0R_FMP0                       ((uint8_t)0x03)               /*!<FIFO 0 Message Pending */
#define  CAN_RF0R_FULL0                      ((uint8_t)0x08)               /*!<FIFO 0 Full */
#define  CAN_RF0R_FOVR0                      ((uint8_t)0x10)               /*!<FIFO 0 Overrun */
#define  CAN_RF0R_RFOM0                      ((uint8_t)0x20)               /*!<Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define  CAN_RF1R_FMP1                       ((uint8_t)0x03)               /*!<FIFO 1 Message Pending */
#define  CAN_RF1R_FULL1                      ((uint8_t)0x08)               /*!<FIFO 1 Full */
#define  CAN_RF1R_FOVR1                      ((uint8_t)0x10)               /*!<FIFO 1 Overrun */
#define  CAN_RF1R_RFOM1                      ((uint8_t)0x20)               /*!<Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
#define  CAN_IER_TMEIE                       ((uint32_t)0x00000001)        /*!<Transmit Mailbox Empty Interrupt Enable */
#define  CAN_IER_FMPIE0                      ((uint32_t)0x00000002)        /*!<FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE0                       ((uint32_t)0x00000004)        /*!<FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE0                      ((uint32_t)0x00000008)        /*!<FIFO Overrun Interrupt Enable */
#define  CAN_IER_FMPIE1                      ((uint32_t)0x00000010)        /*!<FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE1                       ((uint32_t)0x00000020)        /*!<FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE1                      ((uint32_t)0x00000040)        /*!<FIFO Overrun Interrupt Enable */
#define  CAN_IER_EWGIE                       ((uint32_t)0x00000100)        /*!<Error Warning Interrupt Enable */
#define  CAN_IER_EPVIE                       ((uint32_t)0x00000200)        /*!<Error Passive Interrupt Enable */
#define  CAN_IER_BOFIE                       ((uint32_t)0x00000400)        /*!<Bus-Off Interrupt Enable */
#define  CAN_IER_LECIE                       ((uint32_t)0x00000800)        /*!<Last Error Code Interrupt Enable */
#define  CAN_IER_ERRIE                       ((uint32_t)0x00008000)        /*!<Error Interrupt Enable */
#define  CAN_IER_WKUIE                       ((uint32_t)0x00010000)        /*!<Wakeup Interrupt Enable */
#define  CAN_IER_SLKIE                       ((uint32_t)0x00020000)        /*!<Sleep Interrupt Enable */

/********************  Bit definition for CAN_ESR register  *******************/
#define  CAN_ESR_EWGF                        ((uint32_t)0x00000001)        /*!<Error Warning Flag */
#define  CAN_ESR_EPVF                        ((uint32_t)0x00000002)        /*!<Error Passive Flag */
#define  CAN_ESR_BOFF                        ((uint32_t)0x00000004)        /*!<Bus-Off Flag */

#define  CAN_ESR_LEC                         ((uint32_t)0x00000070)        /*!<LEC[2:0] bits (Last Error Code) */
#define  CAN_ESR_LEC_0                       ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  CAN_ESR_LEC_1                       ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  CAN_ESR_LEC_2                       ((uint32_t)0x00000040)        /*!<Bit 2 */

#define  CAN_ESR_TEC                         ((uint32_t)0x00FF0000)        /*!<Least significant byte of the 9-bit Transmit Error Counter */
#define  CAN_ESR_REC                         ((uint32_t)0xFF000000)        /*!<Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
#define  CAN_BTR_BRP                         ((uint32_t)0x000003FF)        /*!<Baud Rate Prescaler */
#define  CAN_BTR_TS1_0                       ((uint32_t)0x00010000)        /*!<Time Segment 1 (Bit 0) */
#define  CAN_BTR_TS1_1                       ((uint32_t)0x00020000)        /*!<Time Segment 1 (Bit 1) */
#define  CAN_BTR_TS1_2                       ((uint32_t)0x00040000)        /*!<Time Segment 1 (Bit 2) */
#define  CAN_BTR_TS1_3                       ((uint32_t)0x00080000)        /*!<Time Segment 1 (Bit 3) */
#define  CAN_BTR_TS1                         ((uint32_t)0x000F0000)        /*!<Time Segment 1 */
#define  CAN_BTR_TS2_0                       ((uint32_t)0x00100000)        /*!<Time Segment 2 (Bit 0) */
#define  CAN_BTR_TS2_1                       ((uint32_t)0x00200000)        /*!<Time Segment 2 (Bit 1) */
#define  CAN_BTR_TS2_2                       ((uint32_t)0x00400000)        /*!<Time Segment 2 (Bit 2) */
#define  CAN_BTR_TS2                         ((uint32_t)0x00700000)        /*!<Time Segment 2 */
#define  CAN_BTR_SJW_0                       ((uint32_t)0x01000000)        /*!<Resynchronization Jump Width (Bit 0) */
#define  CAN_BTR_SJW_1                       ((uint32_t)0x02000000)        /*!<Resynchronization Jump Width (Bit 1) */
#define  CAN_BTR_SJW                         ((uint32_t)0x03000000)        /*!<Resynchronization Jump Width */
#define  CAN_BTR_LBKM                        ((uint32_t)0x40000000)        /*!<Loop Back Mode (Debug) */
#define  CAN_BTR_SILM                        ((uint32_t)0x80000000)        /*!<Silent Mode */

/*!<Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define  CAN_TI0R_TXRQ                       ((uint32_t)0x00000001)        /*!<Transmit Mailbox Request */
#define  CAN_TI0R_RTR                        ((uint32_t)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_TI0R_IDE                        ((uint32_t)0x00000004)        /*!<Identifier Extension */
#define  CAN_TI0R_EXID                       ((uint32_t)0x001FFFF8)        /*!<Extended Identifier */
#define  CAN_TI0R_STID                       ((uint32_t)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
#define  CAN_TDT0R_DLC                       ((uint32_t)0x0000000F)        /*!<Data Length Code */
#define  CAN_TDT0R_TGT                       ((uint32_t)0x00000100)        /*!<Transmit Global Time */
#define  CAN_TDT0R_TIME                      ((uint32_t)0xFFFF0000)        /*!<Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
#define  CAN_TDL0R_DATA0                     ((uint32_t)0x000000FF)        /*!<Data byte 0 */
#define  CAN_TDL0R_DATA1                     ((uint32_t)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_TDL0R_DATA2                     ((uint32_t)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_TDL0R_DATA3                     ((uint32_t)0xFF000000)        /*!<Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
#define  CAN_TDH0R_DATA4                     ((uint32_t)0x000000FF)        /*!<Data byte 4 */
#define  CAN_TDH0R_DATA5                     ((uint32_t)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_TDH0R_DATA6                     ((uint32_t)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_TDH0R_DATA7                     ((uint32_t)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
#define  CAN_TI1R_TXRQ                       ((uint32_t)0x00000001)        /*!<Transmit Mailbox Request */
#define  CAN_TI1R_RTR                        ((uint32_t)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_TI1R_IDE                        ((uint32_t)0x00000004)        /*!<Identifier Extension */
#define  CAN_TI1R_EXID                       ((uint32_t)0x001FFFF8)        /*!<Extended Identifier */
#define  CAN_TI1R_STID                       ((uint32_t)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
#define  CAN_TDT1R_DLC                       ((uint32_t)0x0000000F)        /*!<Data Length Code */
#define  CAN_TDT1R_TGT                       ((uint32_t)0x00000100)        /*!<Transmit Global Time */
#define  CAN_TDT1R_TIME                      ((uint32_t)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
#define  CAN_TDL1R_DATA0                     ((uint32_t)0x000000FF)        /*!<Data byte 0 */
#define  CAN_TDL1R_DATA1                     ((uint32_t)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_TDL1R_DATA2                     ((uint32_t)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_TDL1R_DATA3                     ((uint32_t)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
#define  CAN_TDH1R_DATA4                     ((uint32_t)0x000000FF)        /*!<Data byte 4 */
#define  CAN_TDH1R_DATA5                     ((uint32_t)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_TDH1R_DATA6                     ((uint32_t)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_TDH1R_DATA7                     ((uint32_t)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
#define  CAN_TI2R_TXRQ                       ((uint32_t)0x00000001)        /*!<Transmit Mailbox Request */
#define  CAN_TI2R_RTR                        ((uint32_t)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_TI2R_IDE                        ((uint32_t)0x00000004)        /*!<Identifier Extension */
#define  CAN_TI2R_EXID                       ((uint32_t)0x001FFFF8)        /*!<Extended identifier */
#define  CAN_TI2R_STID                       ((uint32_t)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/
#define  CAN_TDT2R_DLC                       ((uint32_t)0x0000000F)        /*!<Data Length Code */
#define  CAN_TDT2R_TGT                       ((uint32_t)0x00000100)        /*!<Transmit Global Time */
#define  CAN_TDT2R_TIME                      ((uint32_t)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
#define  CAN_TDL2R_DATA0                     ((uint32_t)0x000000FF)        /*!<Data byte 0 */
#define  CAN_TDL2R_DATA1                     ((uint32_t)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_TDL2R_DATA2                     ((uint32_t)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_TDL2R_DATA3                     ((uint32_t)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
#define  CAN_TDH2R_DATA4                     ((uint32_t)0x000000FF)        /*!<Data byte 4 */
#define  CAN_TDH2R_DATA5                     ((uint32_t)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_TDH2R_DATA6                     ((uint32_t)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_TDH2R_DATA7                     ((uint32_t)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
#define  CAN_RI0R_RTR                        ((uint32_t)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_RI0R_IDE                        ((uint32_t)0x00000004)        /*!<Identifier Extension */
#define  CAN_RI0R_EXID                       ((uint32_t)0x001FFFF8)        /*!<Extended Identifier */
#define  CAN_RI0R_STID                       ((uint32_t)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
#define  CAN_RDT0R_DLC                       ((uint32_t)0x0000000F)        /*!<Data Length Code */
#define  CAN_RDT0R_FMI                       ((uint32_t)0x0000FF00)        /*!<Filter Match Index */
#define  CAN_RDT0R_TIME                      ((uint32_t)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
#define  CAN_RDL0R_DATA0                     ((uint32_t)0x000000FF)        /*!<Data byte 0 */
#define  CAN_RDL0R_DATA1                     ((uint32_t)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_RDL0R_DATA2                     ((uint32_t)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_RDL0R_DATA3                     ((uint32_t)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
#define  CAN_RDH0R_DATA4                     ((uint32_t)0x000000FF)        /*!<Data byte 4 */
#define  CAN_RDH0R_DATA5                     ((uint32_t)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_RDH0R_DATA6                     ((uint32_t)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_RDH0R_DATA7                     ((uint32_t)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
#define  CAN_RI1R_RTR                        ((uint32_t)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_RI1R_IDE                        ((uint32_t)0x00000004)        /*!<Identifier Extension */
#define  CAN_RI1R_EXID                       ((uint32_t)0x001FFFF8)        /*!<Extended identifier */
#define  CAN_RI1R_STID                       ((uint32_t)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
#define  CAN_RDT1R_DLC                       ((uint32_t)0x0000000F)        /*!<Data Length Code */
#define  CAN_RDT1R_FMI                       ((uint32_t)0x0000FF00)        /*!<Filter Match Index */
#define  CAN_RDT1R_TIME                      ((uint32_t)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
#define  CAN_RDL1R_DATA0                     ((uint32_t)0x000000FF)        /*!<Data byte 0 */
#define  CAN_RDL1R_DATA1                     ((uint32_t)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_RDL1R_DATA2                     ((uint32_t)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_RDL1R_DATA3                     ((uint32_t)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
#define  CAN_RDH1R_DATA4                     ((uint32_t)0x000000FF)        /*!<Data byte 4 */
#define  CAN_RDH1R_DATA5                     ((uint32_t)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_RDH1R_DATA6                     ((uint32_t)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_RDH1R_DATA7                     ((uint32_t)0xFF000000)        /*!<Data byte 7 */

/*!<CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define  CAN_FMR_FINIT                       ((uint8_t)0x01)               /*!<Filter Init Mode */

/*******************  Bit definition for CAN_FM1R register  *******************/
#define  CAN_FM1R_FBM                        ((uint16_t)0x3FFF)            /*!<Filter Mode */
#define  CAN_FM1R_FBM0                       ((uint16_t)0x0001)            /*!<Filter Init Mode bit 0 */
#define  CAN_FM1R_FBM1                       ((uint16_t)0x0002)            /*!<Filter Init Mode bit 1 */
#define  CAN_FM1R_FBM2                       ((uint16_t)0x0004)            /*!<Filter Init Mode bit 2 */
#define  CAN_FM1R_FBM3                       ((uint16_t)0x0008)            /*!<Filter Init Mode bit 3 */
#define  CAN_FM1R_FBM4                       ((uint16_t)0x0010)            /*!<Filter Init Mode bit 4 */
#define  CAN_FM1R_FBM5                       ((uint16_t)0x0020)            /*!<Filter Init Mode bit 5 */
#define  CAN_FM1R_FBM6                       ((uint16_t)0x0040)            /*!<Filter Init Mode bit 6 */
#define  CAN_FM1R_FBM7                       ((uint16_t)0x0080)            /*!<Filter Init Mode bit 7 */
#define  CAN_FM1R_FBM8                       ((uint16_t)0x0100)            /*!<Filter Init Mode bit 8 */
#define  CAN_FM1R_FBM9                       ((uint16_t)0x0200)            /*!<Filter Init Mode bit 9 */
#define  CAN_FM1R_FBM10                      ((uint16_t)0x0400)            /*!<Filter Init Mode bit 10 */
#define  CAN_FM1R_FBM11                      ((uint16_t)0x0800)            /*!<Filter Init Mode bit 11 */
#define  CAN_FM1R_FBM12                      ((uint16_t)0x1000)            /*!<Filter Init Mode bit 12 */
#define  CAN_FM1R_FBM13                      ((uint16_t)0x2000)            /*!<Filter Init Mode bit 13 */

/*******************  Bit definition for CAN_FS1R register  *******************/
#define  CAN_FS1R_FSC                        ((uint16_t)0x3FFF)            /*!<Filter Scale Configuration */
#define  CAN_FS1R_FSC0                       ((uint16_t)0x0001)            /*!<Filter Scale Configuration bit 0 */
#define  CAN_FS1R_FSC1                       ((uint16_t)0x0002)            /*!<Filter Scale Configuration bit 1 */
#define  CAN_FS1R_FSC2                       ((uint16_t)0x0004)            /*!<Filter Scale Configuration bit 2 */
#define  CAN_FS1R_FSC3                       ((uint16_t)0x0008)            /*!<Filter Scale Configuration bit 3 */
#define  CAN_FS1R_FSC4                       ((uint16_t)0x0010)            /*!<Filter Scale Configuration bit 4 */
#define  CAN_FS1R_FSC5                       ((uint16_t)0x0020)            /*!<Filter Scale Configuration bit 5 */
#define  CAN_FS1R_FSC6                       ((uint16_t)0x0040)            /*!<Filter Scale Configuration bit 6 */
#define  CAN_FS1R_FSC7                       ((uint16_t)0x0080)            /*!<Filter Scale Configuration bit 7 */
#define  CAN_FS1R_FSC8                       ((uint16_t)0x0100)            /*!<Filter Scale Configuration bit 8 */
#define  CAN_FS1R_FSC9                       ((uint16_t)0x0200)            /*!<Filter Scale Configuration bit 9 */
#define  CAN_FS1R_FSC10                      ((uint16_t)0x0400)            /*!<Filter Scale Configuration bit 10 */
#define  CAN_FS1R_FSC11                      ((uint16_t)0x0800)            /*!<Filter Scale Configuration bit 11 */
#define  CAN_FS1R_FSC12                      ((uint16_t)0x1000)            /*!<Filter Scale Configuration bit 12 */
#define  CAN_FS1R_FSC13                      ((uint16_t)0x2000)            /*!<Filter Scale Configuration bit 13 */

/******************  Bit definition for CAN_FFA1R register  *******************/
#define  CAN_FFA1R_FFA                       ((uint16_t)0x3FFF)            /*!<Filter FIFO Assignment */
#define  CAN_FFA1R_FFA0                      ((uint16_t)0x0001)            /*!<Filter FIFO Assignment for Filter 0 */
#define  CAN_FFA1R_FFA1                      ((uint16_t)0x0002)            /*!<Filter FIFO Assignment for Filter 1 */
#define  CAN_FFA1R_FFA2                      ((uint16_t)0x0004)            /*!<Filter FIFO Assignment for Filter 2 */
#define  CAN_FFA1R_FFA3                      ((uint16_t)0x0008)            /*!<Filter FIFO Assignment for Filter 3 */
#define  CAN_FFA1R_FFA4                      ((uint16_t)0x0010)            /*!<Filter FIFO Assignment for Filter 4 */
#define  CAN_FFA1R_FFA5                      ((uint16_t)0x0020)            /*!<Filter FIFO Assignment for Filter 5 */
#define  CAN_FFA1R_FFA6                      ((uint16_t)0x0040)            /*!<Filter FIFO Assignment for Filter 6 */
#define  CAN_FFA1R_FFA7                      ((uint16_t)0x0080)            /*!<Filter FIFO Assignment for Filter 7 */
#define  CAN_FFA1R_FFA8                      ((uint16_t)0x0100)            /*!<Filter FIFO Assignment for Filter 8 */
#define  CAN_FFA1R_FFA9                      ((uint16_t)0x0200)            /*!<Filter FIFO Assignment for Filter 9 */
#define  CAN_FFA1R_FFA10                     ((uint16_t)0x0400)            /*!<Filter FIFO Assignment for Filter 10 */
#define  CAN_FFA1R_FFA11                     ((uint16_t)0x0800)            /*!<Filter FIFO Assignment for Filter 11 */
#define  CAN_FFA1R_FFA12                     ((uint16_t)0x1000)            /*!<Filter FIFO Assignment for Filter 12 */
#define  CAN_FFA1R_FFA13                     ((uint16_t)0x2000)            /*!<Filter FIFO Assignment for Filter 13 */

/*******************  Bit definition for CAN_FA1R register  *******************/
#define  CAN_FA1R_FACT                       ((uint16_t)0x3FFF)            /*!<Filter Active */
#define  CAN_FA1R_FACT0                      ((uint16_t)0x0001)            /*!<Filter 0 Active */
#define  CAN_FA1R_FACT1                      ((uint16_t)0x0002)            /*!<Filter 1 Active */
#define  CAN_FA1R_FACT2                      ((uint16_t)0x0004)            /*!<Filter 2 Active */
#define  CAN_FA1R_FACT3                      ((uint16_t)0x0008)            /*!<Filter 3 Active */
#define  CAN_FA1R_FACT4                      ((uint16_t)0x0010)            /*!<Filter 4 Active */
#define  CAN_FA1R_FACT5                      ((uint16_t)0x0020)            /*!<Filter 5 Active */
#define  CAN_FA1R_FACT6                      ((uint16_t)0x0040)            /*!<Filter 6 Active */
#define  CAN_FA1R_FACT7                      ((uint16_t)0x0080)            /*!<Filter 7 Active */
#define  CAN_FA1R_FACT8                      ((uint16_t)0x0100)            /*!<Filter 8 Active */
#define  CAN_FA1R_FACT9                      ((uint16_t)0x0200)            /*!<Filter 9 Active */
#define  CAN_FA1R_FACT10                     ((uint16_t)0x0400)            /*!<Filter 10 Active */
#define  CAN_FA1R_FACT11                     ((uint16_t)0x0800)            /*!<Filter 11 Active */
#define  CAN_FA1R_FACT12                     ((uint16_t)0x1000)            /*!<Filter 12 Active */
#define  CAN_FA1R_FACT13                     ((uint16_t)0x2000)            /*!<Filter 13 Active */

/*******************  Bit definition for CAN_F0R1 register  *******************/
#define  CAN_F0R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F0R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F0R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F0R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F0R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F0R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F0R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F0R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F0R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F0R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F0R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F0R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F0R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F0R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F0R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F0R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F0R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F0R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F0R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F0R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F0R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F0R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F0R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F0R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F0R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F0R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F0R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F0R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F0R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F0R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F0R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F0R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R1 register  *******************/
#define  CAN_F1R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F1R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F1R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F1R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F1R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F1R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F1R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F1R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F1R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F1R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F1R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F1R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F1R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F1R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F1R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F1R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F1R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F1R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F1R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F1R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F1R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F1R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F1R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F1R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F1R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F1R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F1R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F1R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F1R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F1R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F1R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F1R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R1 register  *******************/
#define  CAN_F2R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F2R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F2R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F2R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F2R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F2R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F2R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F2R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F2R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F2R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F2R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F2R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F2R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F2R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F2R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F2R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F2R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F2R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F2R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F2R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F2R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F2R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F2R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F2R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F2R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F2R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F2R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F2R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F2R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F2R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F2R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F2R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R1 register  *******************/
#define  CAN_F3R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F3R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F3R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F3R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F3R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F3R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F3R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F3R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F3R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F3R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F3R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F3R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F3R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F3R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F3R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F3R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F3R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F3R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F3R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F3R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F3R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F3R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F3R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F3R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F3R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F3R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F3R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F3R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F3R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F3R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F3R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F3R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R1 register  *******************/
#define  CAN_F4R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F4R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F4R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F4R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F4R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F4R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F4R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F4R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F4R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F4R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F4R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F4R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F4R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F4R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F4R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F4R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F4R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F4R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F4R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F4R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F4R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F4R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F4R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F4R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F4R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F4R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F4R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F4R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F4R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F4R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F4R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F4R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R1 register  *******************/
#define  CAN_F5R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F5R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F5R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F5R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F5R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F5R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F5R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F5R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F5R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F5R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F5R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F5R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F5R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F5R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F5R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F5R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F5R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F5R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F5R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F5R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F5R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F5R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F5R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F5R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F5R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F5R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F5R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F5R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F5R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F5R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F5R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F5R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R1 register  *******************/
#define  CAN_F6R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F6R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F6R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F6R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F6R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F6R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F6R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F6R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F6R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F6R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F6R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F6R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F6R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F6R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F6R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F6R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F6R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F6R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F6R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F6R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F6R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F6R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F6R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F6R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F6R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F6R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F6R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F6R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F6R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F6R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F6R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F6R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R1 register  *******************/
#define  CAN_F7R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F7R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F7R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F7R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F7R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F7R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F7R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F7R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F7R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F7R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F7R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F7R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F7R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F7R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F7R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F7R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F7R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F7R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F7R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F7R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F7R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F7R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F7R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F7R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F7R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F7R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F7R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F7R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F7R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F7R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F7R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F7R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R1 register  *******************/
#define  CAN_F8R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F8R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F8R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F8R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F8R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F8R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F8R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F8R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F8R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F8R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F8R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F8R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F8R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F8R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F8R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F8R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F8R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F8R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F8R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F8R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F8R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F8R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F8R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F8R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F8R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F8R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F8R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F8R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F8R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F8R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F8R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F8R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R1 register  *******************/
#define  CAN_F9R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F9R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F9R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F9R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F9R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F9R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F9R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F9R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F9R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F9R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F9R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F9R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F9R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F9R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F9R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F9R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F9R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F9R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F9R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F9R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F9R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F9R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F9R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F9R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F9R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F9R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F9R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F9R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F9R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F9R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F9R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F9R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R1 register  ******************/
#define  CAN_F10R1_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F10R1_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F10R1_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F10R1_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F10R1_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F10R1_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F10R1_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F10R1_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F10R1_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F10R1_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F10R1_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F10R1_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F10R1_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F10R1_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F10R1_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F10R1_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F10R1_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F10R1_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F10R1_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F10R1_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F10R1_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F10R1_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F10R1_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F10R1_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F10R1_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F10R1_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F10R1_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F10R1_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F10R1_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F10R1_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F10R1_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F10R1_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R1 register  ******************/
#define  CAN_F11R1_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F11R1_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F11R1_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F11R1_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F11R1_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F11R1_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F11R1_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F11R1_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F11R1_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F11R1_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F11R1_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F11R1_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F11R1_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F11R1_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F11R1_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F11R1_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F11R1_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F11R1_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F11R1_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F11R1_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F11R1_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F11R1_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F11R1_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F11R1_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F11R1_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F11R1_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F11R1_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F11R1_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F11R1_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F11R1_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F11R1_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F11R1_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R1 register  ******************/
#define  CAN_F12R1_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F12R1_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F12R1_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F12R1_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F12R1_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F12R1_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F12R1_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F12R1_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F12R1_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F12R1_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F12R1_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F12R1_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F12R1_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F12R1_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F12R1_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F12R1_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F12R1_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F12R1_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F12R1_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F12R1_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F12R1_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F12R1_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F12R1_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F12R1_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F12R1_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F12R1_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F12R1_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F12R1_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F12R1_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F12R1_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F12R1_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F12R1_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R1 register  ******************/
#define  CAN_F13R1_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F13R1_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F13R1_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F13R1_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F13R1_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F13R1_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F13R1_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F13R1_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F13R1_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F13R1_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F13R1_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F13R1_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F13R1_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F13R1_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F13R1_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F13R1_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F13R1_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F13R1_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F13R1_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F13R1_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F13R1_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F13R1_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F13R1_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F13R1_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F13R1_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F13R1_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F13R1_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F13R1_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F13R1_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F13R1_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F13R1_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F13R1_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F0R2 register  *******************/
#define  CAN_F0R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F0R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F0R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F0R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F0R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F0R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F0R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F0R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F0R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F0R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F0R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F0R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F0R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F0R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F0R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F0R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F0R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F0R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F0R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F0R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F0R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F0R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F0R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F0R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F0R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F0R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F0R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F0R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F0R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F0R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F0R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F0R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R2 register  *******************/
#define  CAN_F1R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F1R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F1R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F1R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F1R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F1R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F1R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F1R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F1R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F1R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F1R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F1R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F1R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F1R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F1R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F1R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F1R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F1R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F1R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F1R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F1R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F1R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F1R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F1R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F1R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F1R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F1R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F1R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F1R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F1R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F1R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F1R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R2 register  *******************/
#define  CAN_F2R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F2R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F2R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F2R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F2R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F2R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F2R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F2R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F2R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F2R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F2R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F2R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F2R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F2R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F2R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F2R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F2R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F2R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F2R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F2R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F2R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F2R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F2R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F2R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F2R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F2R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F2R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F2R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F2R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F2R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F2R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F2R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R2 register  *******************/
#define  CAN_F3R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F3R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F3R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F3R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F3R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F3R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F3R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F3R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F3R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F3R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F3R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F3R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F3R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F3R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F3R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F3R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F3R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F3R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F3R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F3R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F3R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F3R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F3R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F3R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F3R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F3R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F3R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F3R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F3R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F3R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F3R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F3R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R2 register  *******************/
#define  CAN_F4R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F4R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F4R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F4R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F4R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F4R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F4R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F4R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F4R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F4R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F4R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F4R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F4R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F4R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F4R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F4R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F4R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F4R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F4R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F4R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F4R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F4R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F4R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F4R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F4R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F4R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F4R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F4R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F4R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F4R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F4R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F4R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R2 register  *******************/
#define  CAN_F5R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F5R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F5R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F5R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F5R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F5R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F5R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F5R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F5R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F5R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F5R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F5R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F5R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F5R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F5R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F5R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F5R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F5R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F5R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F5R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F5R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F5R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F5R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F5R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F5R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F5R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F5R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F5R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F5R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F5R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F5R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F5R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R2 register  *******************/
#define  CAN_F6R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F6R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F6R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F6R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F6R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F6R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F6R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F6R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F6R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F6R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F6R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F6R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F6R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F6R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F6R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F6R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F6R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F6R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F6R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F6R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F6R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F6R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F6R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F6R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F6R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F6R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F6R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F6R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F6R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F6R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F6R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F6R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R2 register  *******************/
#define  CAN_F7R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F7R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F7R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F7R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F7R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F7R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F7R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F7R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F7R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F7R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F7R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F7R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F7R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F7R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F7R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F7R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F7R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F7R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F7R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F7R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F7R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F7R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F7R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F7R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F7R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F7R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F7R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F7R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F7R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F7R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F7R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F7R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R2 register  *******************/
#define  CAN_F8R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F8R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F8R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F8R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F8R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F8R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F8R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F8R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F8R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F8R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F8R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F8R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F8R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F8R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F8R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F8R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F8R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F8R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F8R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F8R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F8R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F8R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F8R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F8R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F8R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F8R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F8R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F8R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F8R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F8R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F8R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F8R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R2 register  *******************/
#define  CAN_F9R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F9R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F9R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F9R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F9R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F9R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F9R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F9R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F9R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F9R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F9R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F9R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F9R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F9R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F9R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F9R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F9R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F9R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F9R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F9R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F9R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F9R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F9R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F9R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F9R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F9R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F9R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F9R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F9R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F9R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F9R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F9R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R2 register  ******************/
#define  CAN_F10R2_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F10R2_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F10R2_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F10R2_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F10R2_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F10R2_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F10R2_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F10R2_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F10R2_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F10R2_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F10R2_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F10R2_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F10R2_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F10R2_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F10R2_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F10R2_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F10R2_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F10R2_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F10R2_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F10R2_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F10R2_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F10R2_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F10R2_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F10R2_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F10R2_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F10R2_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F10R2_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F10R2_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F10R2_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F10R2_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F10R2_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F10R2_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R2 register  ******************/
#define  CAN_F11R2_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F11R2_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F11R2_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F11R2_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F11R2_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F11R2_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F11R2_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F11R2_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F11R2_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F11R2_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F11R2_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F11R2_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F11R2_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F11R2_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F11R2_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F11R2_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F11R2_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F11R2_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F11R2_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F11R2_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F11R2_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F11R2_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F11R2_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F11R2_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F11R2_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F11R2_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F11R2_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F11R2_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F11R2_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F11R2_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F11R2_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F11R2_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R2 register  ******************/
#define  CAN_F12R2_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F12R2_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F12R2_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F12R2_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F12R2_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F12R2_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F12R2_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F12R2_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F12R2_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F12R2_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F12R2_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F12R2_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F12R2_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F12R2_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F12R2_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F12R2_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F12R2_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F12R2_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F12R2_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F12R2_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F12R2_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F12R2_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F12R2_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F12R2_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F12R2_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F12R2_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F12R2_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F12R2_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F12R2_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F12R2_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F12R2_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F12R2_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R2 register  ******************/
#define  CAN_F13R2_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F13R2_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F13R2_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F13R2_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F13R2_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F13R2_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F13R2_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F13R2_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F13R2_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F13R2_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F13R2_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F13R2_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F13R2_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F13R2_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F13R2_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F13R2_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F13R2_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F13R2_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F13R2_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F13R2_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F13R2_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F13R2_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F13R2_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F13R2_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F13R2_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F13R2_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F13R2_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F13R2_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F13R2_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F13R2_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F13R2_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F13R2_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           ((uint32_t)0xFFFFFFFF) /*!< Data register bits */

/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         ((uint8_t)0xFF)        /*!< General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        ((uint32_t)0x00000001) /*!< RESET the CRC computation unit bit */
#define  CRC_CR_POLYSIZE                     ((uint32_t)0x00000018) /*!< Polynomial size bits */
#define  CRC_CR_POLYSIZE_0                   ((uint32_t)0x00000008) /*!< Polynomial size bit 0 */
#define  CRC_CR_POLYSIZE_1                   ((uint32_t)0x00000010) /*!< Polynomial size bit 1 */
#define  CRC_CR_REV_IN                       ((uint32_t)0x00000060) /*!< REV_IN Reverse Input Data bits */
#define  CRC_CR_REV_IN_0                     ((uint32_t)0x00000020) /*!< Bit 0 */
#define  CRC_CR_REV_IN_1                     ((uint32_t)0x00000040) /*!< Bit 1 */
#define  CRC_CR_REV_OUT                      ((uint32_t)0x00000080) /*!< REV_OUT Reverse Output Data bits */

/*******************  Bit definition for CRC_INIT register  *******************/
#define  CRC_INIT_INIT                       ((uint32_t)0xFFFFFFFF) /*!< Initial CRC value bits */

/*******************  Bit definition for CRC_POL register  ********************/
#define  CRC_POL_POL                         ((uint32_t)0xFFFFFFFF) /*!< Coefficients of the polynomial */

/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DAC_CR register  ********************/
#define  DAC_CR_EN1                          ((uint32_t)0x00000001)        /*!<DAC channel1 enable */
#define  DAC_CR_TEN1                         ((uint32_t)0x00000004)        /*!<DAC channel1 Trigger enable */

#define  DAC_CR_TSEL1                        ((uint32_t)0x00000038)        /*!<TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_CR_TSEL1_0                      ((uint32_t)0x00000008)        /*!<Bit 0 */
#define  DAC_CR_TSEL1_1                      ((uint32_t)0x00000010)        /*!<Bit 1 */
#define  DAC_CR_TSEL1_2                      ((uint32_t)0x00000020)        /*!<Bit 2 */

#define  DAC_CR_WAVE1                        ((uint32_t)0x000000C0)        /*!<WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE1_0                      ((uint32_t)0x00000040)        /*!<Bit 0 */
#define  DAC_CR_WAVE1_1                      ((uint32_t)0x00000080)        /*!<Bit 1 */

#define  DAC_CR_MAMP1                        ((uint32_t)0x00000F00)        /*!<MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_CR_MAMP1_0                      ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  DAC_CR_MAMP1_1                      ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  DAC_CR_MAMP1_2                      ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  DAC_CR_MAMP1_3                      ((uint32_t)0x00000800)        /*!<Bit 3 */

#define  DAC_CR_DMAEN1                       ((uint32_t)0x00001000)        /*!<DAC channel1 DMA enable */
#define  DAC_CR_DMAUDRIE1                    ((uint32_t)0x00002000)        /*!<DAC channel 1 DMA underrun interrupt enable  >*/
#define  DAC_CR_CEN1                         ((uint32_t)0x00004000)        /*!<DAC channel 1 calibration enable >*/

#define  DAC_CR_EN2                          ((uint32_t)0x00010000)        /*!<DAC channel2 enable */
#define  DAC_CR_TEN2                         ((uint32_t)0x00040000)        /*!<DAC channel2 Trigger enable */

#define  DAC_CR_TSEL2                        ((uint32_t)0x00380000)        /*!<TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  DAC_CR_TSEL2_0                      ((uint32_t)0x00080000)        /*!<Bit 0 */
#define  DAC_CR_TSEL2_1                      ((uint32_t)0x00100000)        /*!<Bit 1 */
#define  DAC_CR_TSEL2_2                      ((uint32_t)0x00200000)        /*!<Bit 2 */

#define  DAC_CR_WAVE2                        ((uint32_t)0x00C00000)        /*!<WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE2_0                      ((uint32_t)0x00400000)        /*!<Bit 0 */
#define  DAC_CR_WAVE2_1                      ((uint32_t)0x00800000)        /*!<Bit 1 */

#define  DAC_CR_MAMP2                        ((uint32_t)0x0F000000)        /*!<MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define  DAC_CR_MAMP2_0                      ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  DAC_CR_MAMP2_1                      ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  DAC_CR_MAMP2_2                      ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  DAC_CR_MAMP2_3                      ((uint32_t)0x08000000)        /*!<Bit 3 */

#define  DAC_CR_DMAEN2                       ((uint32_t)0x10000000)        /*!<DAC channel2 DMA enabled */
#define  DAC_CR_DMAUDRIE2                    ((uint32_t)0x20000000)        /*!<DAC channel2 DMA underrun interrupt enable  >*/
#define  DAC_CR_CEN2                         ((uint32_t)0x40000000)        /*!<DAC channel2 calibration enable >*/

/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define  DAC_SWTRIGR_SWTRIG1                 ((uint32_t)0x00000001)        /*!<DAC channel1 software trigger */
#define  DAC_SWTRIGR_SWTRIG2                 ((uint32_t)0x00000002)        /*!<DAC channel2 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define  DAC_DHR12R1_DACC1DHR                ((uint32_t)0x00000FFF)        /*!<DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define  DAC_DHR12L1_DACC1DHR                ((uint32_t)0x0000FFF0)        /*!<DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define  DAC_DHR8R1_DACC1DHR                 ((uint32_t)0x000000FF)        /*!<DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12R2 register  ******************/
#define  DAC_DHR12R2_DACC2DHR                ((uint32_t)0x00000FFF)        /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L2 register  ******************/
#define  DAC_DHR12L2_DACC2DHR                ((uint32_t)0x0000FFF0)        /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R2 register  ******************/
#define  DAC_DHR8R2_DACC2DHR                 ((uint32_t)0x000000FF)        /*!<DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define  DAC_DHR12RD_DACC1DHR                ((uint32_t)0x00000FFF)        /*!<DAC channel1 12-bit Right aligned data */
#define  DAC_DHR12RD_DACC2DHR                ((uint32_t)0x0FFF0000)        /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define  DAC_DHR12LD_DACC1DHR                ((uint32_t)0x0000FFF0)        /*!<DAC channel1 12-bit Left aligned data */
#define  DAC_DHR12LD_DACC2DHR                ((uint32_t)0xFFF00000)        /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define  DAC_DHR8RD_DACC1DHR                 ((uint32_t)0x000000FF)        /*!<DAC channel1 8-bit Right aligned data */
#define  DAC_DHR8RD_DACC2DHR                 ((uint32_t)0x0000FF00)        /*!<DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define  DAC_DOR1_DACC1DOR                   ((uint32_t)0x00000FFF)        /*!<DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
#define  DAC_DOR2_DACC2DOR                   ((uint32_t)0x00000FFF)        /*!<DAC channel2 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define  DAC_SR_DMAUDR1                      ((uint32_t)0x00002000)        /*!<DAC channel1 DMA underrun flag */
#define  DAC_SR_CAL_FLAG1                    ((uint32_t)0x00004000)        /*!<DAC channel1 calibration offset status */
#define  DAC_SR_BWST1                        ((uint32_t)0x20008000)        /*!<DAC channel1 busy writing sample time flag */

#define  DAC_SR_DMAUDR2                      ((uint32_t)0x20000000)        /*!<DAC channel2 DMA underrun flag */
#define  DAC_SR_CAL_FLAG2                    ((uint32_t)0x40000000)        /*!<DAC channel2 calibration offset status */
#define  DAC_SR_BWST2                        ((uint32_t)0x80000000)        /*!<DAC channel2 busy writing sample time flag */

/*******************  Bit definition for DAC_CCR register  ********************/
#define  DAC_CCR_OTRIM1                      ((uint32_t)0x0000001F)        /*!<DAC channel1 offset trimming value */
#define  DAC_CCR_OTRIM2                      ((uint32_t)0x001F0000)        /*!<DAC channel2 offset trimming value */

/*******************  Bit definition for DAC_MCR register  *******************/
#define  DAC_MCR_MODE1                        ((uint32_t)0x00000007)        /*!<MODE1[2:0] (DAC channel1 mode) */
#define  DAC_MCR_MODE1_0                      ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  DAC_MCR_MODE1_1                      ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  DAC_MCR_MODE1_2                      ((uint32_t)0x00000004)        /*!<Bit 2 */

#define  DAC_MCR_MODE2                        ((uint32_t)0x00070000)        /*!<MODE2[2:0] (DAC channel2 mode) */
#define  DAC_MCR_MODE2_0                      ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  DAC_MCR_MODE2_1                      ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  DAC_MCR_MODE2_2                      ((uint32_t)0x00040000)        /*!<Bit 2 */

/******************  Bit definition for DAC_SHSR1 register  ******************/
#define  DAC_SHSR1_TSAMPLE1                   ((uint32_t)0x000003FF)        /*!<DAC channel1 sample time */

/******************  Bit definition for DAC_SHSR2 register  ******************/
#define  DAC_SHSR2_TSAMPLE2                   ((uint32_t)0x000003FF)        /*!<DAC channel2 sample time */

/******************  Bit definition for DAC_SHHR register  ******************/
#define  DAC_SHHR_THOLD1                      ((uint32_t)0x000003FF)        /*!<DAC channel1 hold time */
#define  DAC_SHHR_THOLD2                      ((uint32_t)0x03FF0000)        /*!<DAC channel2 hold time */

/******************  Bit definition for DAC_SHRR register  ******************/
#define  DAC_SHRR_TREFRESH1                   ((uint32_t)0x000000FF)        /*!<DAC channel1 refresh time */
#define  DAC_SHRR_TREFRESH2                   ((uint32_t)0x00FF0000)        /*!<DAC channel2 refresh time */


/******************************************************************************/
/*                                                                            */
/*                 Digital Filter for Sigma Delta Modulators                  */
/*                                                                            */
/******************************************************************************/

/****************   DFSDM channel configuration registers  ********************/

/***************  Bit definition for DFSDM_CHCFGR1 register  ******************/
#define  DFSDM_CHCFGR1_DFSDMEN                ((uint32_t)0x80000000)            /*!< Global enable for DFSDM interface */
#define  DFSDM_CHCFGR1_CKOUTSRC               ((uint32_t)0x40000000)            /*!< Output serial clock source selection */
#define  DFSDM_CHCFGR1_CKOUTDIV               ((uint32_t)0x00FF0000)            /*!< CKOUTDIV[7:0] output serial clock divider */
#define  DFSDM_CHCFGR1_DATPACK                ((uint32_t)0x0000C000)            /*!< DATPACK[1:0] Data packing mode */
#define  DFSDM_CHCFGR1_DATPACK_1              ((uint32_t)0x00008000)            /*!< Data packing mode, Bit 1 */
#define  DFSDM_CHCFGR1_DATPACK_0              ((uint32_t)0x00004000)            /*!< Data packing mode, Bit 0 */
#define  DFSDM_CHCFGR1_DATMPX                 ((uint32_t)0x00003000)            /*!< DATMPX[1:0] Input data multiplexer for channel y */
#define  DFSDM_CHCFGR1_DATMPX_1               ((uint32_t)0x00002000)            /*!< Input data multiplexer for channel y, Bit 1 */
#define  DFSDM_CHCFGR1_DATMPX_0               ((uint32_t)0x00001000)            /*!< Input data multiplexer for channel y, Bit 0 */
#define  DFSDM_CHCFGR1_CHINSEL                ((uint32_t)0x00000100)            /*!< Serial inputs selection for channel y */
#define  DFSDM_CHCFGR1_CHEN                   ((uint32_t)0x00000080)            /*!< Channel y enable */
#define  DFSDM_CHCFGR1_CKABEN                 ((uint32_t)0x00000040)            /*!< Clock absence detector enable on channel y */
#define  DFSDM_CHCFGR1_SCDEN                  ((uint32_t)0x00000020)            /*!< Short circuit detector enable on channel y */
#define  DFSDM_CHCFGR1_SPICKSEL               ((uint32_t)0x0000000C)            /*!< SPICKSEL[1:0] SPI clock select for channel y */
#define  DFSDM_CHCFGR1_SPICKSEL_1             ((uint32_t)0x00000008)            /*!< SPI clock select for channel y, Bit 1 */
#define  DFSDM_CHCFGR1_SPICKSEL_0             ((uint32_t)0x00000004)            /*!< SPI clock select for channel y, Bit 0 */
#define  DFSDM_CHCFGR1_SITP                   ((uint32_t)0x00000003)            /*!< SITP[1:0] Serial interface type for channel y */
#define  DFSDM_CHCFGR1_SITP_1                 ((uint32_t)0x00000002)            /*!< Serial interface type for channel y, Bit 1 */
#define  DFSDM_CHCFGR1_SITP_0                 ((uint32_t)0x00000001)            /*!< Serial interface type for channel y, Bit 0 */

/***************  Bit definition for DFSDM_CHCFGR2 register  ******************/
#define  DFSDM_CHCFGR2_OFFSET                 ((uint32_t)0xFFFFFF00)            /*!< OFFSET[23:0] 24-bit calibration offset for channel y */
#define  DFSDM_CHCFGR2_DTRBS                  ((uint32_t)0x000000F8)            /*!< DTRBS[4:0] Data right bit-shift for channel y */

/******************  Bit definition for DFSDM_AWSCDR register *****************/
#define  DFSDM_AWSCDR_AWFORD                  ((uint32_t)0x00C00000)            /*!< AWFORD[1:0] Analog watchdog Sinc filter order on channel y */
#define  DFSDM_AWSCDR_AWFORD_1                ((uint32_t)0x00800000)            /*!< Analog watchdog Sinc filter order on channel y, Bit 1 */
#define  DFSDM_AWSCDR_AWFORD_0                ((uint32_t)0x00400000)            /*!< Analog watchdog Sinc filter order on channel y, Bit 0 */
#define  DFSDM_AWSCDR_AWFOSR                  ((uint32_t)0x001F0000)            /*!< AWFOSR[4:0] Analog watchdog filter oversampling ratio on channel y */
#define  DFSDM_AWSCDR_BKSCD                   ((uint32_t)0x0000F000)            /*!< BKSCD[3:0] Break signal assignment for short circuit detector on channel y */
#define  DFSDM_AWSCDR_SCDT                    ((uint32_t)0x000000FF)            /*!< SCDT[7:0] Short circuit detector threshold for channel y */

/****************  Bit definition for DFSDM_CHWDATR register *******************/
#define  DFSDM_AWSCDR_WDATA                   ((uint32_t)0x0000FFFF)            /*!< WDATA[15:0] Input channel y watchdog data */

/****************  Bit definition for DFSDM_CHDATINR register *****************/
#define  DFSDM_AWSCDR_INDAT0                   ((uint32_t)0x0000FFFF)            /*!< INDAT0[31:16] Input data for channel y or channel (y+1) */
#define  DFSDM_AWSCDR_INDAT1                   ((uint32_t)0xFFFF0000)            /*!< INDAT0[15:0] Input data for channel y */

/************************   DFSDM module registers  ****************************/

/********************  Bit definition for DFSDM_CR1 register *******************/
#define  DFSDM_CR1_AWFSEL                     ((uint32_t)0x40000000)            /*!< Analog watchdog fast mode select */
#define  DFSDM_CR1_FAST                       ((uint32_t)0x20000000)            /*!< Fast conversion mode selection */
#define  DFSDM_CR1_RCH                        ((uint32_t)0x07000000)            /*!< RCH[2:0] Regular channel selection */
#define  DFSDM_CR1_RDMAEN                     ((uint32_t)0x00200000)            /*!< DMA channel enabled to read data for the regular conversion */
#define  DFSDM_CR1_RSYNC                      ((uint32_t)0x00080000)            /*!< Launch regular conversion synchronously with DFSDMx */
#define  DFSDM_CR1_RCONT                      ((uint32_t)0x00040000)            /*!< Continuous mode selection for regular conversions */
#define  DFSDM_CR1_RSWSTART                   ((uint32_t)0x00020000)            /*!< Software start of a conversion on the regular channel */
#define  DFSDM_CR1_JEXTEN                     ((uint32_t)0x00006000)            /*!< JEXTEN[1:0] Trigger enable and trigger edge selection for injected conversions */
#define  DFSDM_CR1_JEXTEN_1                   ((uint32_t)0x00004000)            /*!< Trigger enable and trigger edge selection for injected conversions, Bit 1 */
#define  DFSDM_CR1_JEXTEN_0                   ((uint32_t)0x00002000)            /*!< Trigger enable and trigger edge selection for injected conversions, Bit 0 */
#define  DFSDM_CR1_JEXTSEL                    ((uint32_t)0x00000700)            /*!< JEXTSEL[2:0]Trigger signal selection for launching injected conversions */
#define  DFSDM_CR1_JEXTSEL_2                  ((uint32_t)0x00000400)            /*!< Trigger signal selection for launching injected conversions, Bit 2 */
#define  DFSDM_CR1_JEXTSEL_1                  ((uint32_t)0x00000200)            /*!< Trigger signal selection for launching injected conversions, Bit 1 */
#define  DFSDM_CR1_JEXTSEL_0                  ((uint32_t)0x00000100)            /*!< Trigger signal selection for launching injected conversions, Bit 0 */
#define  DFSDM_CR1_JDMAEN                     ((uint32_t)0x00000020)            /*!< DMA channel enabled to read data for the injected channel group */
#define  DFSDM_CR1_JSCAN                      ((uint32_t)0x00000010)            /*!< Scanning conversion in continuous mode selection for injected conversions */
#define  DFSDM_CR1_JSYNC                      ((uint32_t)0x00000008)            /*!< Launch an injected conversion synchronously with DFSDMx JSWSTART trigger  */
#define  DFSDM_CR1_JSWSTART                   ((uint32_t)0x00000002)            /*!< Start the conversion of the injected group of channels */
#define  DFSDM_CR1_DFEN                       ((uint32_t)0x00000001)            /*!< DFSDM enable */

/********************  Bit definition for DFSDM_CR2 register *******************/
#define  DFSDM_CR2_AWDCH                      ((uint32_t)0x00FF0000)            /*!< AWDCH[7:0] Analog watchdog channel selection */
#define  DFSDM_CR2_EXCH                       ((uint32_t)0x0000FF00)            /*!< EXCH[7:0] Extreme detector channel selection */
#define  DFSDM_CR2_CKABIE                     ((uint32_t)0x00000040)            /*!< Clock absence interrupt enable */
#define  DFSDM_CR2_SCDIE                      ((uint32_t)0x00000020)            /*!< Short circuit detector interrupt enable */
#define  DFSDM_CR2_AWDIE                      ((uint32_t)0x00000010)            /*!< Analog watchdog interrupt enable */
#define  DFSDM_CR2_ROVRIE                     ((uint32_t)0x00000008)            /*!< Regular data overrun interrupt enable */
#define  DFSDM_CR2_JOVRIE                     ((uint32_t)0x00000004)            /*!< Injected data overrun interrupt enable */
#define  DFSDM_CR2_REOCIE                     ((uint32_t)0x00000002)            /*!< Regular end of conversion interrupt enable */
#define  DFSDM_CR2_JEOCIE                     ((uint32_t)0x00000001)            /*!< Injected end of conversion interrupt enable */

/********************  Bit definition for DFSDM_ISR register *******************/
#define  DFSDM_ISR_SCDF                       ((uint32_t)0xFF000000)            /*!< SCDF[7:0] Short circuit detector flag */
#define  DFSDM_ISR_CKABF                      ((uint32_t)0x00FF0000)            /*!< CKABF[7:0] Clock absence flag */
#define  DFSDM_ISR_RCIP                       ((uint32_t)0x00004000)            /*!< Regular conversion in progress status */
#define  DFSDM_ISR_JCIP                       ((uint32_t)0x00002000)            /*!< Injected conversion in progress status */
#define  DFSDM_ISR_AWDF                       ((uint32_t)0x00000010)            /*!< Analog watchdog */
#define  DFSDM_ISR_ROVRF                      ((uint32_t)0x00000008)            /*!< Regular conversion overrun flag */
#define  DFSDM_ISR_JOVRF                      ((uint32_t)0x00000004)            /*!< Injected conversion overrun flag */
#define  DFSDM_ISR_REOCF                      ((uint32_t)0x00000002)            /*!< End of regular conversion flag */
#define  DFSDM_ISR_JEOCF                      ((uint32_t)0x00000001)            /*!< End of injected conversion flag */

/********************  Bit definition for DFSDM_ICR register *******************/
#define  DFSDM_ICR_CLRSCSDF                   ((uint32_t)0xFF000000)            /*!< CLRSCSDF[7:0] Clear the short circuit detector flag */
#define  DFSDM_ICR_CLRCKABF                   ((uint32_t)0x00FF0000)            /*!< CLRCKABF[7:0] Clear the clock absence flag */
#define  DFSDM_ICR_CLRROVRF                   ((uint32_t)0x00000008)            /*!< Clear the regular conversion overrun flag */
#define  DFSDM_ICR_CLRJOVRF                   ((uint32_t)0x00000004)            /*!< Clear the injected conversion overrun flag */

/*******************  Bit definition for DFSDM_JCHGR register ******************/
#define  DFSDM_JCHGR_JCHG                     ((uint32_t)0x000000FF)            /*!< JCHG[7:0] Injected channel group selection */

/********************  Bit definition for DFSDM_FCR register *******************/
#define  DFSDM_FCR_FORD                       ((uint32_t)0xE0000000)            /*!< FORD[2:0] Sinc filter order */
#define  DFSDM_FCR_FORD_2                     ((uint32_t)0x80000000)            /*!< Sinc filter order, Bit 2 */
#define  DFSDM_FCR_FORD_1                     ((uint32_t)0x40000000)            /*!< Sinc filter order, Bit 1 */
#define  DFSDM_FCR_FORD_0                     ((uint32_t)0x20000000)            /*!< Sinc filter order, Bit 0 */
#define  DFSDM_FCR_FOSR                       ((uint32_t)0x03FF0000)            /*!< FOSR[9:0] Sinc filter oversampling ratio (decimation rate) */
#define  DFSDM_FCR_IOSR                       ((uint32_t)0x000000FF)            /*!< IOSR[7:0] Integrator oversampling ratio (averaging length) */

/******************  Bit definition for DFSDM_JDATAR register *****************/
#define  DFSDM_JDATAR_JDATA                   ((uint32_t)0xFFFFFF00)            /*!< JDATA[23:0] Injected group conversion data */
#define  DFSDM_JDATAR_JDATACH                 ((uint32_t)0x00000007)            /*!< JDATACH[2:0] Injected channel most recently converted */

/******************  Bit definition for DFSDM_RDATAR register *****************/
#define  DFSDM_RDATAR_RDATA                   ((uint32_t)0xFFFFFF00)            /*!< RDATA[23:0] Regular channel conversion data */
#define  DFSDM_RDATAR_RPEND                   ((uint32_t)0x00000010)            /*!< RPEND Regular channel pending data */
#define  DFSDM_RDATAR_RDATACH                 ((uint32_t)0x00000007)            /*!< RDATACH[2:0] Regular channel most recently converted */

/******************  Bit definition for DFSDM_AWHTR register ******************/
#define  DFSDM_AWHTR_AWHT                    ((uint32_t)0xFFFFFF00)             /*!< AWHT[23:0] Analog watchdog high threshold */
#define  DFSDM_AWHTR_BKAWH                   ((uint32_t)0x0000000F)             /*!< BKAWH[3:0] Break signal assignment to analog watchdog high threshold event */

/******************  Bit definition for DFSDM_AWLTR register ******************/
#define  DFSDM_AWLTR_AWLT                    ((uint32_t)0xFFFFFF00)             /*!< AWHT[23:0] Analog watchdog low threshold */
#define  DFSDM_AWLTR_BKAWL                   ((uint32_t)0x0000000F)             /*!< BKAWL[3:0] Break signal assignment to analog watchdog low threshold event */

/******************  Bit definition for DFSDM_AWSR register ******************/
#define  DFSDM_AWSR_AWHTF                    ((uint32_t)0x0000FF00)             /*!< AWHTF[15:8] Analog watchdog high threshold error on given channels */
#define  DFSDM_AWSR_AWLTF                    ((uint32_t)0x000000FF)             /*!< AWLTF[7:0] Analog watchdog low threshold error on given channels */

/******************  Bit definition for DFSDM_AWCFR) register *****************/
#define  DFSDM_AWCFR_CLRAWHTF                ((uint32_t)0x0000FF00)             /*!< CLRAWHTF[15:8] Clear the Analog watchdog high threshold flag */
#define  DFSDM_AWCFR_CLRAWLTF                ((uint32_t)0x000000FF)             /*!< CLRAWLTF[7:0] Clear the Analog watchdog low threshold flag */

/******************  Bit definition for DFSDM_EXMAX register ******************/
#define  DFSDM_EXMAX_EXMAX                   ((uint32_t)0xFFFFFF00)             /*!< EXMAX[23:0] Extreme detector maximum value */
#define  DFSDM_EXMAX_EXMAXCH                 ((uint32_t)0x00000007)             /*!< EXMAXCH[2:0] Extreme detector maximum data channel */

/******************  Bit definition for DFSDM_EXMIN register ******************/
#define  DFSDM_EXMIN_EXMIN                   ((uint32_t)0xFFFFFF00)             /*!< EXMIN[23:0] Extreme detector minimum value */
#define  DFSDM_EXMIN_EXMINCH                 ((uint32_t)0x00000007)             /*!< EXMINCH[2:0] Extreme detector minimum data channel */

/******************  Bit definition for DFSDM_EXMIN register ******************/
#define  DFSDM_CNVTIMR_CNVCNT                ((uint32_t)0xFFFFFFF0)             /*!< CNVCNT[27:0]: 28-bit timer counting conversion time */

/******************************************************************************/
/*                                                                            */
/*                           DMA Controller (DMA)                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for DMA_ISR register  ********************/
#define  DMA_ISR_GIF1                        ((uint32_t)0x00000001)        /*!< Channel 1 Global interrupt flag */
#define  DMA_ISR_TCIF1                       ((uint32_t)0x00000002)        /*!< Channel 1 Transfer Complete flag */
#define  DMA_ISR_HTIF1                       ((uint32_t)0x00000004)        /*!< Channel 1 Half Transfer flag */
#define  DMA_ISR_TEIF1                       ((uint32_t)0x00000008)        /*!< Channel 1 Transfer Error flag */
#define  DMA_ISR_GIF2                        ((uint32_t)0x00000010)        /*!< Channel 2 Global interrupt flag */
#define  DMA_ISR_TCIF2                       ((uint32_t)0x00000020)        /*!< Channel 2 Transfer Complete flag */
#define  DMA_ISR_HTIF2                       ((uint32_t)0x00000040)        /*!< Channel 2 Half Transfer flag */
#define  DMA_ISR_TEIF2                       ((uint32_t)0x00000080)        /*!< Channel 2 Transfer Error flag */
#define  DMA_ISR_GIF3                        ((uint32_t)0x00000100)        /*!< Channel 3 Global interrupt flag */
#define  DMA_ISR_TCIF3                       ((uint32_t)0x00000200)        /*!< Channel 3 Transfer Complete flag */
#define  DMA_ISR_HTIF3                       ((uint32_t)0x00000400)        /*!< Channel 3 Half Transfer flag */
#define  DMA_ISR_TEIF3                       ((uint32_t)0x00000800)        /*!< Channel 3 Transfer Error flag */
#define  DMA_ISR_GIF4                        ((uint32_t)0x00001000)        /*!< Channel 4 Global interrupt flag */
#define  DMA_ISR_TCIF4                       ((uint32_t)0x00002000)        /*!< Channel 4 Transfer Complete flag */
#define  DMA_ISR_HTIF4                       ((uint32_t)0x00004000)        /*!< Channel 4 Half Transfer flag */
#define  DMA_ISR_TEIF4                       ((uint32_t)0x00008000)        /*!< Channel 4 Transfer Error flag */
#define  DMA_ISR_GIF5                        ((uint32_t)0x00010000)        /*!< Channel 5 Global interrupt flag */
#define  DMA_ISR_TCIF5                       ((uint32_t)0x00020000)        /*!< Channel 5 Transfer Complete flag */
#define  DMA_ISR_HTIF5                       ((uint32_t)0x00040000)        /*!< Channel 5 Half Transfer flag */
#define  DMA_ISR_TEIF5                       ((uint32_t)0x00080000)        /*!< Channel 5 Transfer Error flag */
#define  DMA_ISR_GIF6                        ((uint32_t)0x00100000)        /*!< Channel 6 Global interrupt flag */
#define  DMA_ISR_TCIF6                       ((uint32_t)0x00200000)        /*!< Channel 6 Transfer Complete flag */
#define  DMA_ISR_HTIF6                       ((uint32_t)0x00400000)        /*!< Channel 6 Half Transfer flag */
#define  DMA_ISR_TEIF6                       ((uint32_t)0x00800000)        /*!< Channel 6 Transfer Error flag */
#define  DMA_ISR_GIF7                        ((uint32_t)0x01000000)        /*!< Channel 7 Global interrupt flag */
#define  DMA_ISR_TCIF7                       ((uint32_t)0x02000000)        /*!< Channel 7 Transfer Complete flag */
#define  DMA_ISR_HTIF7                       ((uint32_t)0x04000000)        /*!< Channel 7 Half Transfer flag */
#define  DMA_ISR_TEIF7                       ((uint32_t)0x08000000)        /*!< Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define  DMA_IFCR_CGIF1                      ((uint32_t)0x00000001)        /*!< Channel 1 Global interrupt clearr */
#define  DMA_IFCR_CTCIF1                     ((uint32_t)0x00000002)        /*!< Channel 1 Transfer Complete clear */
#define  DMA_IFCR_CHTIF1                     ((uint32_t)0x00000004)        /*!< Channel 1 Half Transfer clear */
#define  DMA_IFCR_CTEIF1                     ((uint32_t)0x00000008)        /*!< Channel 1 Transfer Error clear */
#define  DMA_IFCR_CGIF2                      ((uint32_t)0x00000010)        /*!< Channel 2 Global interrupt clear */
#define  DMA_IFCR_CTCIF2                     ((uint32_t)0x00000020)        /*!< Channel 2 Transfer Complete clear */
#define  DMA_IFCR_CHTIF2                     ((uint32_t)0x00000040)        /*!< Channel 2 Half Transfer clear */
#define  DMA_IFCR_CTEIF2                     ((uint32_t)0x00000080)        /*!< Channel 2 Transfer Error clear */
#define  DMA_IFCR_CGIF3                      ((uint32_t)0x00000100)        /*!< Channel 3 Global interrupt clear */
#define  DMA_IFCR_CTCIF3                     ((uint32_t)0x00000200)        /*!< Channel 3 Transfer Complete clear */
#define  DMA_IFCR_CHTIF3                     ((uint32_t)0x00000400)        /*!< Channel 3 Half Transfer clear */
#define  DMA_IFCR_CTEIF3                     ((uint32_t)0x00000800)        /*!< Channel 3 Transfer Error clear */
#define  DMA_IFCR_CGIF4                      ((uint32_t)0x00001000)        /*!< Channel 4 Global interrupt clear */
#define  DMA_IFCR_CTCIF4                     ((uint32_t)0x00002000)        /*!< Channel 4 Transfer Complete clear */
#define  DMA_IFCR_CHTIF4                     ((uint32_t)0x00004000)        /*!< Channel 4 Half Transfer clear */
#define  DMA_IFCR_CTEIF4                     ((uint32_t)0x00008000)        /*!< Channel 4 Transfer Error clear */
#define  DMA_IFCR_CGIF5                      ((uint32_t)0x00010000)        /*!< Channel 5 Global interrupt clear */
#define  DMA_IFCR_CTCIF5                     ((uint32_t)0x00020000)        /*!< Channel 5 Transfer Complete clear */
#define  DMA_IFCR_CHTIF5                     ((uint32_t)0x00040000)        /*!< Channel 5 Half Transfer clear */
#define  DMA_IFCR_CTEIF5                     ((uint32_t)0x00080000)        /*!< Channel 5 Transfer Error clear */
#define  DMA_IFCR_CGIF6                      ((uint32_t)0x00100000)        /*!< Channel 6 Global interrupt clear */
#define  DMA_IFCR_CTCIF6                     ((uint32_t)0x00200000)        /*!< Channel 6 Transfer Complete clear */
#define  DMA_IFCR_CHTIF6                     ((uint32_t)0x00400000)        /*!< Channel 6 Half Transfer clear */
#define  DMA_IFCR_CTEIF6                     ((uint32_t)0x00800000)        /*!< Channel 6 Transfer Error clear */
#define  DMA_IFCR_CGIF7                      ((uint32_t)0x01000000)        /*!< Channel 7 Global interrupt clear */
#define  DMA_IFCR_CTCIF7                     ((uint32_t)0x02000000)        /*!< Channel 7 Transfer Complete clear */
#define  DMA_IFCR_CHTIF7                     ((uint32_t)0x04000000)        /*!< Channel 7 Half Transfer clear */
#define  DMA_IFCR_CTEIF7                     ((uint32_t)0x08000000)        /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCR register  ********************/
#define  DMA_CCR_EN                          ((uint32_t)0x00000001)        /*!< Channel enable                      */
#define  DMA_CCR_TCIE                        ((uint32_t)0x00000002)        /*!< Transfer complete interrupt enable  */
#define  DMA_CCR_HTIE                        ((uint32_t)0x00000004)        /*!< Half Transfer interrupt enable      */
#define  DMA_CCR_TEIE                        ((uint32_t)0x00000008)        /*!< Transfer error interrupt enable     */
#define  DMA_CCR_DIR                         ((uint32_t)0x00000010)        /*!< Data transfer direction             */
#define  DMA_CCR_CIRC                        ((uint32_t)0x00000020)        /*!< Circular mode                       */
#define  DMA_CCR_PINC                        ((uint32_t)0x00000040)        /*!< Peripheral increment mode           */
#define  DMA_CCR_MINC                        ((uint32_t)0x00000080)        /*!< Memory increment mode               */

#define  DMA_CCR_PSIZE                       ((uint32_t)0x00000300)        /*!< PSIZE[1:0] bits (Peripheral size)   */
#define  DMA_CCR_PSIZE_0                     ((uint32_t)0x00000100)        /*!< Bit 0                               */
#define  DMA_CCR_PSIZE_1                     ((uint32_t)0x00000200)        /*!< Bit 1                               */

#define  DMA_CCR_MSIZE                       ((uint32_t)0x00000C00)        /*!< MSIZE[1:0] bits (Memory size)       */
#define  DMA_CCR_MSIZE_0                     ((uint32_t)0x00000400)        /*!< Bit 0                               */
#define  DMA_CCR_MSIZE_1                     ((uint32_t)0x00000800)        /*!< Bit 1                               */

#define  DMA_CCR_PL                          ((uint32_t)0x00003000)        /*!< PL[1:0] bits(Channel Priority level)*/
#define  DMA_CCR_PL_0                        ((uint32_t)0x00001000)        /*!< Bit 0                               */
#define  DMA_CCR_PL_1                        ((uint32_t)0x00002000)        /*!< Bit 1                               */

#define  DMA_CCR_MEM2MEM                     ((uint32_t)0x00004000)        /*!< Memory to memory mode               */

/******************  Bit definition for DMA_CNDTR register  *******************/
#define  DMA_CNDTR_NDT                       ((uint32_t)0x0000FFFF)        /*!< Number of data to Transfer          */

/******************  Bit definition for DMA_CPAR register  ********************/
#define  DMA_CPAR_PA                         ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address                  */

/******************  Bit definition for DMA_CMAR register  ********************/
#define  DMA_CMAR_MA                         ((uint32_t)0xFFFFFFFF)        /*!< Memory Address                      */


/*******************  Bit definition for DMA_CSELR register  *******************/
#define  DMA_CSELR_C1S                          ((uint32_t)0x0000000F)          /*!< Channel 1 Selection */
#define  DMA_CSELR_C2S                          ((uint32_t)0x000000F0)          /*!< Channel 2 Selection */
#define  DMA_CSELR_C3S                          ((uint32_t)0x00000F00)          /*!< Channel 3 Selection */
#define  DMA_CSELR_C4S                          ((uint32_t)0x0000F000)          /*!< Channel 4 Selection */
#define  DMA_CSELR_C5S                          ((uint32_t)0x000F0000)          /*!< Channel 5 Selection */
#define  DMA_CSELR_C6S                          ((uint32_t)0x00F00000)          /*!< Channel 6 Selection */
#define  DMA_CSELR_C7S                          ((uint32_t)0x0F000000)          /*!< Channel 7 Selection */


/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR1 register  ******************/
#define  EXTI_IMR1_IM0                       ((uint32_t)0x00000001)        /*!< Interrupt Mask on line 0 */
#define  EXTI_IMR1_IM1                       ((uint32_t)0x00000002)        /*!< Interrupt Mask on line 1 */
#define  EXTI_IMR1_IM2                       ((uint32_t)0x00000004)        /*!< Interrupt Mask on line 2 */
#define  EXTI_IMR1_IM3                       ((uint32_t)0x00000008)        /*!< Interrupt Mask on line 3 */
#define  EXTI_IMR1_IM4                       ((uint32_t)0x00000010)        /*!< Interrupt Mask on line 4 */
#define  EXTI_IMR1_IM5                       ((uint32_t)0x00000020)        /*!< Interrupt Mask on line 5 */
#define  EXTI_IMR1_IM6                       ((uint32_t)0x00000040)        /*!< Interrupt Mask on line 6 */
#define  EXTI_IMR1_IM7                       ((uint32_t)0x00000080)        /*!< Interrupt Mask on line 7 */
#define  EXTI_IMR1_IM8                       ((uint32_t)0x00000100)        /*!< Interrupt Mask on line 8 */
#define  EXTI_IMR1_IM9                       ((uint32_t)0x00000200)        /*!< Interrupt Mask on line 9 */
#define  EXTI_IMR1_IM10                      ((uint32_t)0x00000400)        /*!< Interrupt Mask on line 10 */
#define  EXTI_IMR1_IM11                      ((uint32_t)0x00000800)        /*!< Interrupt Mask on line 11 */
#define  EXTI_IMR1_IM12                      ((uint32_t)0x00001000)        /*!< Interrupt Mask on line 12 */
#define  EXTI_IMR1_IM13                      ((uint32_t)0x00002000)        /*!< Interrupt Mask on line 13 */
#define  EXTI_IMR1_IM14                      ((uint32_t)0x00004000)        /*!< Interrupt Mask on line 14 */
#define  EXTI_IMR1_IM15                      ((uint32_t)0x00008000)        /*!< Interrupt Mask on line 15 */
#define  EXTI_IMR1_IM16                      ((uint32_t)0x00010000)        /*!< Interrupt Mask on line 16 */
#define  EXTI_IMR1_IM17                      ((uint32_t)0x00020000)        /*!< Interrupt Mask on line 17 */
#define  EXTI_IMR1_IM18                      ((uint32_t)0x00040000)        /*!< Interrupt Mask on line 18 */
#define  EXTI_IMR1_IM19                      ((uint32_t)0x00080000)        /*!< Interrupt Mask on line 19 */
#define  EXTI_IMR1_IM20                      ((uint32_t)0x00100000)        /*!< Interrupt Mask on line 20 */
#define  EXTI_IMR1_IM21                      ((uint32_t)0x00200000)        /*!< Interrupt Mask on line 21 */
#define  EXTI_IMR1_IM22                      ((uint32_t)0x00400000)        /*!< Interrupt Mask on line 22 */
#define  EXTI_IMR1_IM23                      ((uint32_t)0x00800000)        /*!< Interrupt Mask on line 23 */
#define  EXTI_IMR1_IM24                      ((uint32_t)0x01000000)        /*!< Interrupt Mask on line 24 */
#define  EXTI_IMR1_IM25                      ((uint32_t)0x02000000)        /*!< Interrupt Mask on line 25 */
#define  EXTI_IMR1_IM26                      ((uint32_t)0x04000000)        /*!< Interrupt Mask on line 26 */
#define  EXTI_IMR1_IM27                      ((uint32_t)0x08000000)        /*!< Interrupt Mask on line 27 */
#define  EXTI_IMR1_IM28                      ((uint32_t)0x10000000)        /*!< Interrupt Mask on line 28 */
#define  EXTI_IMR1_IM29                      ((uint32_t)0x20000000)        /*!< Interrupt Mask on line 29 */
#define  EXTI_IMR1_IM30                      ((uint32_t)0x40000000)        /*!< Interrupt Mask on line 30 */
#define  EXTI_IMR1_IM31                      ((uint32_t)0x80000000)        /*!< Interrupt Mask on line 31 */

/*******************  Bit definition for EXTI_EMR1 register  ******************/
#define  EXTI_EMR1_EM0                       ((uint32_t)0x00000001)        /*!< Event Mask on line 0 */
#define  EXTI_EMR1_EM1                       ((uint32_t)0x00000002)        /*!< Event Mask on line 1 */
#define  EXTI_EMR1_EM2                       ((uint32_t)0x00000004)        /*!< Event Mask on line 2 */
#define  EXTI_EMR1_EM3                       ((uint32_t)0x00000008)        /*!< Event Mask on line 3 */
#define  EXTI_EMR1_EM4                       ((uint32_t)0x00000010)        /*!< Event Mask on line 4 */
#define  EXTI_EMR1_EM5                       ((uint32_t)0x00000020)        /*!< Event Mask on line 5 */
#define  EXTI_EMR1_EM6                       ((uint32_t)0x00000040)        /*!< Event Mask on line 6 */
#define  EXTI_EMR1_EM7                       ((uint32_t)0x00000080)        /*!< Event Mask on line 7 */
#define  EXTI_EMR1_EM8                       ((uint32_t)0x00000100)        /*!< Event Mask on line 8 */
#define  EXTI_EMR1_EM9                       ((uint32_t)0x00000200)        /*!< Event Mask on line 9 */
#define  EXTI_EMR1_EM10                      ((uint32_t)0x00000400)        /*!< Event Mask on line 10 */
#define  EXTI_EMR1_EM11                      ((uint32_t)0x00000800)        /*!< Event Mask on line 11 */
#define  EXTI_EMR1_EM12                      ((uint32_t)0x00001000)        /*!< Event Mask on line 12 */
#define  EXTI_EMR1_EM13                      ((uint32_t)0x00002000)        /*!< Event Mask on line 13 */
#define  EXTI_EMR1_EM14                      ((uint32_t)0x00004000)        /*!< Event Mask on line 14 */
#define  EXTI_EMR1_EM15                      ((uint32_t)0x00008000)        /*!< Event Mask on line 15 */
#define  EXTI_EMR1_EM16                      ((uint32_t)0x00010000)        /*!< Event Mask on line 16 */
#define  EXTI_EMR1_EM17                      ((uint32_t)0x00020000)        /*!< Event Mask on line 17 */
#define  EXTI_EMR1_EM18                      ((uint32_t)0x00040000)        /*!< Event Mask on line 18 */
#define  EXTI_EMR1_EM19                      ((uint32_t)0x00080000)        /*!< Event Mask on line 19 */
#define  EXTI_EMR1_EM20                      ((uint32_t)0x00100000)        /*!< Event Mask on line 20 */
#define  EXTI_EMR1_EM21                      ((uint32_t)0x00200000)        /*!< Event Mask on line 21 */
#define  EXTI_EMR1_EM22                      ((uint32_t)0x00400000)        /*!< Event Mask on line 22 */
#define  EXTI_EMR1_EM23                      ((uint32_t)0x00800000)        /*!< Event Mask on line 23 */
#define  EXTI_EMR1_EM24                      ((uint32_t)0x01000000)        /*!< Event Mask on line 24 */
#define  EXTI_EMR1_EM25                      ((uint32_t)0x02000000)        /*!< Event Mask on line 25 */
#define  EXTI_EMR1_EM26                      ((uint32_t)0x04000000)        /*!< Event Mask on line 26 */
#define  EXTI_EMR1_EM27                      ((uint32_t)0x08000000)        /*!< Event Mask on line 27 */
#define  EXTI_EMR1_EM28                      ((uint32_t)0x10000000)        /*!< Event Mask on line 28 */
#define  EXTI_EMR1_EM29                      ((uint32_t)0x20000000)        /*!< Event Mask on line 29 */
#define  EXTI_EMR1_EM30                      ((uint32_t)0x40000000)        /*!< Event Mask on line 30 */
#define  EXTI_EMR1_EM31                      ((uint32_t)0x80000000)        /*!< Event Mask on line 31 */

/******************  Bit definition for EXTI_RTSR1 register  ******************/
#define  EXTI_RTSR1_RT0                      ((uint32_t)0x00000001)        /*!< Rising trigger event configuration bit of line 0 */
#define  EXTI_RTSR1_RT1                      ((uint32_t)0x00000002)        /*!< Rising trigger event configuration bit of line 1 */
#define  EXTI_RTSR1_RT2                      ((uint32_t)0x00000004)        /*!< Rising trigger event configuration bit of line 2 */
#define  EXTI_RTSR1_RT3                      ((uint32_t)0x00000008)        /*!< Rising trigger event configuration bit of line 3 */
#define  EXTI_RTSR1_RT4                      ((uint32_t)0x00000010)        /*!< Rising trigger event configuration bit of line 4 */
#define  EXTI_RTSR1_RT5                      ((uint32_t)0x00000020)        /*!< Rising trigger event configuration bit of line 5 */
#define  EXTI_RTSR1_RT6                      ((uint32_t)0x00000040)        /*!< Rising trigger event configuration bit of line 6 */
#define  EXTI_RTSR1_RT7                      ((uint32_t)0x00000080)        /*!< Rising trigger event configuration bit of line 7 */
#define  EXTI_RTSR1_RT8                      ((uint32_t)0x00000100)        /*!< Rising trigger event configuration bit of line 8 */
#define  EXTI_RTSR1_RT9                      ((uint32_t)0x00000200)        /*!< Rising trigger event configuration bit of line 9 */
#define  EXTI_RTSR1_RT10                     ((uint32_t)0x00000400)        /*!< Rising trigger event configuration bit of line 10 */
#define  EXTI_RTSR1_RT11                     ((uint32_t)0x00000800)        /*!< Rising trigger event configuration bit of line 11 */
#define  EXTI_RTSR1_RT12                     ((uint32_t)0x00001000)        /*!< Rising trigger event configuration bit of line 12 */
#define  EXTI_RTSR1_RT13                     ((uint32_t)0x00002000)        /*!< Rising trigger event configuration bit of line 13 */
#define  EXTI_RTSR1_RT14                     ((uint32_t)0x00004000)        /*!< Rising trigger event configuration bit of line 14 */
#define  EXTI_RTSR1_RT15                     ((uint32_t)0x00008000)        /*!< Rising trigger event configuration bit of line 15 */
#define  EXTI_RTSR1_RT16                     ((uint32_t)0x00010000)        /*!< Rising trigger event configuration bit of line 16 */
#define  EXTI_RTSR1_RT18                     ((uint32_t)0x00040000)        /*!< Rising trigger event configuration bit of line 18 */
#define  EXTI_RTSR1_RT19                     ((uint32_t)0x00080000)        /*!< Rising trigger event configuration bit of line 19 */
#define  EXTI_RTSR1_RT20                     ((uint32_t)0x00100000)        /*!< Rising trigger event configuration bit of line 20 */
#define  EXTI_RTSR1_RT21                     ((uint32_t)0x00200000)        /*!< Rising trigger event configuration bit of line 21 */
#define  EXTI_RTSR1_RT22                     ((uint32_t)0x00400000)        /*!< Rising trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_FTSR1 register  ******************/
#define  EXTI_FTSR1_FT0                      ((uint32_t)0x00000001)        /*!< Falling trigger event configuration bit of line 0 */
#define  EXTI_FTSR1_FT1                      ((uint32_t)0x00000002)        /*!< Falling trigger event configuration bit of line 1 */
#define  EXTI_FTSR1_FT2                      ((uint32_t)0x00000004)        /*!< Falling trigger event configuration bit of line 2 */
#define  EXTI_FTSR1_FT3                      ((uint32_t)0x00000008)        /*!< Falling trigger event configuration bit of line 3 */
#define  EXTI_FTSR1_FT4                      ((uint32_t)0x00000010)        /*!< Falling trigger event configuration bit of line 4 */
#define  EXTI_FTSR1_FT5                      ((uint32_t)0x00000020)        /*!< Falling trigger event configuration bit of line 5 */
#define  EXTI_FTSR1_FT6                      ((uint32_t)0x00000040)        /*!< Falling trigger event configuration bit of line 6 */
#define  EXTI_FTSR1_FT7                      ((uint32_t)0x00000080)        /*!< Falling trigger event configuration bit of line 7 */
#define  EXTI_FTSR1_FT8                      ((uint32_t)0x00000100)        /*!< Falling trigger event configuration bit of line 8 */
#define  EXTI_FTSR1_FT9                      ((uint32_t)0x00000200)        /*!< Falling trigger event configuration bit of line 9 */
#define  EXTI_FTSR1_FT10                     ((uint32_t)0x00000400)        /*!< Falling trigger event configuration bit of line 10 */
#define  EXTI_FTSR1_FT11                     ((uint32_t)0x00000800)        /*!< Falling trigger event configuration bit of line 11 */
#define  EXTI_FTSR1_FT12                     ((uint32_t)0x00001000)        /*!< Falling trigger event configuration bit of line 12 */
#define  EXTI_FTSR1_FT13                     ((uint32_t)0x00002000)        /*!< Falling trigger event configuration bit of line 13 */
#define  EXTI_FTSR1_FT14                     ((uint32_t)0x00004000)        /*!< Falling trigger event configuration bit of line 14 */
#define  EXTI_FTSR1_FT15                     ((uint32_t)0x00008000)        /*!< Falling trigger event configuration bit of line 15 */
#define  EXTI_FTSR1_FT16                     ((uint32_t)0x00010000)        /*!< Falling trigger event configuration bit of line 16 */
#define  EXTI_FTSR1_FT18                     ((uint32_t)0x00040000)        /*!< Falling trigger event configuration bit of line 18 */
#define  EXTI_FTSR1_FT19                     ((uint32_t)0x00080000)        /*!< Falling trigger event configuration bit of line 19 */
#define  EXTI_FTSR1_FT20                     ((uint32_t)0x00100000)        /*!< Falling trigger event configuration bit of line 20 */
#define  EXTI_FTSR1_FT21                     ((uint32_t)0x00200000)        /*!< Falling trigger event configuration bit of line 21 */
#define  EXTI_FTSR1_FT22                     ((uint32_t)0x00400000)        /*!< Falling trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_SWIER1 register  *****************/
#define  EXTI_SWIER1_SWI0                    ((uint32_t)0x00000001)        /*!< Software Interrupt on line 0 */
#define  EXTI_SWIER1_SWI1                    ((uint32_t)0x00000002)        /*!< Software Interrupt on line 1 */
#define  EXTI_SWIER1_SWI2                    ((uint32_t)0x00000004)        /*!< Software Interrupt on line 2 */
#define  EXTI_SWIER1_SWI3                    ((uint32_t)0x00000008)        /*!< Software Interrupt on line 3 */
#define  EXTI_SWIER1_SWI4                    ((uint32_t)0x00000010)        /*!< Software Interrupt on line 4 */
#define  EXTI_SWIER1_SWI5                    ((uint32_t)0x00000020)        /*!< Software Interrupt on line 5 */
#define  EXTI_SWIER1_SWI6                    ((uint32_t)0x00000040)        /*!< Software Interrupt on line 6 */
#define  EXTI_SWIER1_SWI7                    ((uint32_t)0x00000080)        /*!< Software Interrupt on line 7 */
#define  EXTI_SWIER1_SWI8                    ((uint32_t)0x00000100)        /*!< Software Interrupt on line 8 */
#define  EXTI_SWIER1_SWI9                    ((uint32_t)0x00000200)        /*!< Software Interrupt on line 9 */
#define  EXTI_SWIER1_SWI10                   ((uint32_t)0x00000400)        /*!< Software Interrupt on line 10 */
#define  EXTI_SWIER1_SWI11                   ((uint32_t)0x00000800)        /*!< Software Interrupt on line 11 */
#define  EXTI_SWIER1_SWI12                   ((uint32_t)0x00001000)        /*!< Software Interrupt on line 12 */
#define  EXTI_SWIER1_SWI13                   ((uint32_t)0x00002000)        /*!< Software Interrupt on line 13 */
#define  EXTI_SWIER1_SWI14                   ((uint32_t)0x00004000)        /*!< Software Interrupt on line 14 */
#define  EXTI_SWIER1_SWI15                   ((uint32_t)0x00008000)        /*!< Software Interrupt on line 15 */
#define  EXTI_SWIER1_SWI16                   ((uint32_t)0x00010000)        /*!< Software Interrupt on line 16 */
#define  EXTI_SWIER1_SWI18                   ((uint32_t)0x00040000)        /*!< Software Interrupt on line 18 */
#define  EXTI_SWIER1_SWI19                   ((uint32_t)0x00080000)        /*!< Software Interrupt on line 19 */
#define  EXTI_SWIER1_SWI20                   ((uint32_t)0x00100000)        /*!< Software Interrupt on line 20 */
#define  EXTI_SWIER1_SWI21                   ((uint32_t)0x00200000)        /*!< Software Interrupt on line 21 */
#define  EXTI_SWIER1_SWI22                   ((uint32_t)0x00400000)        /*!< Software Interrupt on line 22 */

/*******************  Bit definition for EXTI_PR1 register  *******************/
#define  EXTI_PR1_PIF0                       ((uint32_t)0x00000001)        /*!< Pending bit for line 0 */
#define  EXTI_PR1_PIF1                       ((uint32_t)0x00000002)        /*!< Pending bit for line 1 */
#define  EXTI_PR1_PIF2                       ((uint32_t)0x00000004)        /*!< Pending bit for line 2 */
#define  EXTI_PR1_PIF3                       ((uint32_t)0x00000008)        /*!< Pending bit for line 3 */
#define  EXTI_PR1_PIF4                       ((uint32_t)0x00000010)        /*!< Pending bit for line 4 */
#define  EXTI_PR1_PIF5                       ((uint32_t)0x00000020)        /*!< Pending bit for line 5 */
#define  EXTI_PR1_PIF6                       ((uint32_t)0x00000040)        /*!< Pending bit for line 6 */
#define  EXTI_PR1_PIF7                       ((uint32_t)0x00000080)        /*!< Pending bit for line 7 */
#define  EXTI_PR1_PIF8                       ((uint32_t)0x00000100)        /*!< Pending bit for line 8 */
#define  EXTI_PR1_PIF9                       ((uint32_t)0x00000200)        /*!< Pending bit for line 9 */
#define  EXTI_PR1_PIF10                      ((uint32_t)0x00000400)        /*!< Pending bit for line 10 */
#define  EXTI_PR1_PIF11                      ((uint32_t)0x00000800)        /*!< Pending bit for line 11 */
#define  EXTI_PR1_PIF12                      ((uint32_t)0x00001000)        /*!< Pending bit for line 12 */
#define  EXTI_PR1_PIF13                      ((uint32_t)0x00002000)        /*!< Pending bit for line 13 */
#define  EXTI_PR1_PIF14                      ((uint32_t)0x00004000)        /*!< Pending bit for line 14 */
#define  EXTI_PR1_PIF15                      ((uint32_t)0x00008000)        /*!< Pending bit for line 15 */
#define  EXTI_PR1_PIF16                      ((uint32_t)0x00010000)        /*!< Pending bit for line 16 */
#define  EXTI_PR1_PIF18                      ((uint32_t)0x00040000)        /*!< Pending bit for line 18 */
#define  EXTI_PR1_PIF19                      ((uint32_t)0x00080000)        /*!< Pending bit for line 19 */
#define  EXTI_PR1_PIF20                      ((uint32_t)0x00100000)        /*!< Pending bit for line 20 */
#define  EXTI_PR1_PIF21                      ((uint32_t)0x00200000)        /*!< Pending bit for line 21 */
#define  EXTI_PR1_PIF22                      ((uint32_t)0x00400000)        /*!< Pending bit for line 22 */

/*******************  Bit definition for EXTI_IMR2 register  ******************/
#define  EXTI_IMR2_IM32                      ((uint32_t)0x00000001)        /*!< Interrupt Mask on line 32 */
#define  EXTI_IMR2_IM33                      ((uint32_t)0x00000002)        /*!< Interrupt Mask on line 33 */
#define  EXTI_IMR2_IM34                      ((uint32_t)0x00000004)        /*!< Interrupt Mask on line 34 */
#define  EXTI_IMR2_IM35                      ((uint32_t)0x00000008)        /*!< Interrupt Mask on line 35 */
#define  EXTI_IMR2_IM36                      ((uint32_t)0x00000010)        /*!< Interrupt Mask on line 36 */
#define  EXTI_IMR2_IM37                      ((uint32_t)0x00000020)        /*!< Interrupt Mask on line 37 */
#define  EXTI_IMR2_IM38                      ((uint32_t)0x00000040)        /*!< Interrupt Mask on line 38 */
#define  EXTI_IMR2_IM39                      ((uint32_t)0x00000080)        /*!< Interrupt Mask on line 39 */

/*******************  Bit definition for EXTI_EMR2 register  ******************/
#define  EXTI_EMR2_EM32                      ((uint32_t)0x00000001)        /*!< Event Mask on line 32 */
#define  EXTI_EMR2_EM33                      ((uint32_t)0x00000002)        /*!< Event Mask on line 33 */
#define  EXTI_EMR2_EM34                      ((uint32_t)0x00000004)        /*!< Event Mask on line 34 */
#define  EXTI_EMR2_EM35                      ((uint32_t)0x00000008)        /*!< Event Mask on line 35 */
#define  EXTI_EMR2_EM36                      ((uint32_t)0x00000010)        /*!< Event Mask on line 36 */
#define  EXTI_EMR2_EM37                      ((uint32_t)0x00000020)        /*!< Event Mask on line 37 */
#define  EXTI_EMR2_EM38                      ((uint32_t)0x00000040)        /*!< Event Mask on line 38 */
#define  EXTI_EMR2_EM39                      ((uint32_t)0x00000080)        /*!< Event Mask on line 39 */

/******************  Bit definition for EXTI_RTSR2 register  ******************/
#define  EXTI_RTSR2_RT35                     ((uint32_t)0x00000008)        /*!< Rising trigger event configuration bit of line 35 */
#define  EXTI_RTSR2_RT36                     ((uint32_t)0x00000010)        /*!< Rising trigger event configuration bit of line 36 */
#define  EXTI_RTSR2_RT37                     ((uint32_t)0x00000020)        /*!< Rising trigger event configuration bit of line 37 */
#define  EXTI_RTSR2_RT38                     ((uint32_t)0x00000040)        /*!< Rising trigger event configuration bit of line 38 */

/******************  Bit definition for EXTI_FTSR2 register  ******************/
#define  EXTI_FTSR2_FT35                     ((uint32_t)0x00000008)        /*!< Falling trigger event configuration bit of line 35 */
#define  EXTI_FTSR2_FT36                     ((uint32_t)0x00000010)        /*!< Falling trigger event configuration bit of line 36 */
#define  EXTI_FTSR2_FT37                     ((uint32_t)0x00000020)        /*!< Falling trigger event configuration bit of line 37 */
#define  EXTI_FTSR2_FT38                     ((uint32_t)0x00000040)        /*!< Falling trigger event configuration bit of line 38 */

/******************  Bit definition for EXTI_SWIER2 register  *****************/
#define  EXTI_SWIER2_SWI35                   ((uint32_t)0x00000008)        /*!< Software Interrupt on line 35 */
#define  EXTI_SWIER2_SWI36                   ((uint32_t)0x00000010)        /*!< Software Interrupt on line 36 */
#define  EXTI_SWIER2_SWI37                   ((uint32_t)0x00000020)        /*!< Software Interrupt on line 37 */
#define  EXTI_SWIER2_SWI38                   ((uint32_t)0x00000040)        /*!< Software Interrupt on line 38 */

/*******************  Bit definition for EXTI_PR2 register  *******************/
#define  EXTI_PR2_PIF35                       ((uint32_t)0x00000008)        /*!< Pending bit for line 35 */
#define  EXTI_PR2_PIF36                       ((uint32_t)0x00000010)        /*!< Pending bit for line 36 */
#define  EXTI_PR2_PIF37                       ((uint32_t)0x00000020)        /*!< Pending bit for line 37 */
#define  EXTI_PR2_PIF38                       ((uint32_t)0x00000040)        /*!< Pending bit for line 38 */


/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY                    ((uint32_t)0x00000007)
#define FLASH_ACR_LATENCY_0WS                ((uint32_t)0x00000000)
#define FLASH_ACR_LATENCY_1WS                ((uint32_t)0x00000001)
#define FLASH_ACR_LATENCY_2WS                ((uint32_t)0x00000002)
#define FLASH_ACR_LATENCY_3WS                ((uint32_t)0x00000003)
#define FLASH_ACR_LATENCY_4WS                ((uint32_t)0x00000004)
#define FLASH_ACR_PRFTEN                     ((uint32_t)0x00000100)
#define FLASH_ACR_ICEN                       ((uint32_t)0x00000200)
#define FLASH_ACR_DCEN                       ((uint32_t)0x00000400)
#define FLASH_ACR_ICRST                      ((uint32_t)0x00000800)
#define FLASH_ACR_DCRST                      ((uint32_t)0x00001000)
#define FLASH_ACR_RUN_PD                     ((uint32_t)0x00002000)   /*!< Flash power down mode during run */
#define FLASH_ACR_SLEEP_PD                   ((uint32_t)0x00004000)   /*!< Flash power down mode during sleep */

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP                         ((uint32_t)0x00000001)
#define FLASH_SR_OPERR                       ((uint32_t)0x00000002)
#define FLASH_SR_PROGERR                     ((uint32_t)0x00000008)
#define FLASH_SR_WRPERR                      ((uint32_t)0x00000010)
#define FLASH_SR_PGAERR                      ((uint32_t)0x00000020)
#define FLASH_SR_SIZERR                      ((uint32_t)0x00000040)
#define FLASH_SR_PGSERR                      ((uint32_t)0x00000080)
#define FLASH_SR_MISERR                      ((uint32_t)0x00000100)
#define FLASH_SR_FASTERR                     ((uint32_t)0x00000200)
#define FLASH_SR_RDERR                       ((uint32_t)0x00004000)
#define FLASH_SR_OPTVERR                     ((uint32_t)0x00008000)
#define FLASH_SR_BSY                         ((uint32_t)0x00010000)

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG                          ((uint32_t)0x00000001)
#define FLASH_CR_PER                         ((uint32_t)0x00000002)
#define FLASH_CR_MER1                        ((uint32_t)0x00000004)
#define FLASH_CR_PNB                         ((uint32_t)0x000007F8)
/*#define FLASH_CR_PNB_0                       ((uint32_t)0x00000008)
#define FLASH_CR_PNB_1                       ((uint32_t)0x00000010)
#define FLASH_CR_PNB_2                       ((uint32_t)0x00000020)
#define FLASH_CR_PNB_3                       ((uint32_t)0x00000040)
#define FLASH_CR_PNB_4                       ((uint32_t)0x00000080)
#define FLASH_CR_PNB_5                       ((uint32_t)0x00000100)
#define FLASH_CR_PNB_6                       ((uint32_t)0x00000200)
#define FLASH_CR_PNB_7                       ((uint32_t)0x00000400)*/
#define FLASH_CR_BKER                        ((uint32_t)0x00000800)
#define FLASH_CR_MER2                        ((uint32_t)0x00008000)
#define FLASH_CR_STRT                        ((uint32_t)0x00010000)
#define FLASH_CR_OPTSTRT                     ((uint32_t)0x00020000)
#define FLASH_CR_FSTPG                       ((uint32_t)0x00040000)
#define FLASH_CR_EOPIE                       ((uint32_t)0x01000000)
#define FLASH_CR_ERRIE                       ((uint32_t)0x02000000)
#define FLASH_CR_RDERRIE                     ((uint32_t)0x04000000)
#define FLASH_CR_OBL_LAUNCH                  ((uint32_t)0x08000000)
#define FLASH_CR_OPTLOCK                     ((uint32_t)0x40000000)
#define FLASH_CR_LOCK                        ((uint32_t)0x80000000)

/*******************  Bits definition for FLASH_ECCR register  ***************/
#define FLASH_ECCR_ADDR_ECC                 ((uint32_t)0x0007FFFF)
/*#define FLASH_ECCR_ADDR_ECC_0               ((uint32_t)0x00000001)
#define FLASH_ECCR_ADDR_ECC_1               ((uint32_t)0x00000002)
#define FLASH_ECCR_ADDR_ECC_2               ((uint32_t)0x00000004)
#define FLASH_ECCR_ADDR_ECC_3               ((uint32_t)0x00000008)
#define FLASH_ECCR_ADDR_ECC_4               ((uint32_t)0x00000010)
#define FLASH_ECCR_ADDR_ECC_5               ((uint32_t)0x00000020)
#define FLASH_ECCR_ADDR_ECC_6               ((uint32_t)0x00000040)
#define FLASH_ECCR_ADDR_ECC_7               ((uint32_t)0x00000080)
#define FLASH_ECCR_ADDR_ECC_8               ((uint32_t)0x00000100)
#define FLASH_ECCR_ADDR_ECC_9               ((uint32_t)0x00000200)
#define FLASH_ECCR_ADDR_ECC_10              ((uint32_t)0x00000400)
#define FLASH_ECCR_ADDR_ECC_11              ((uint32_t)0x00000800)
#define FLASH_ECCR_ADDR_ECC_12              ((uint32_t)0x00001000)
#define FLASH_ECCR_ADDR_ECC_13              ((uint32_t)0x00002000)
#define FLASH_ECCR_ADDR_ECC_14              ((uint32_t)0x00004000)
#define FLASH_ECCR_ADDR_ECC_15              ((uint32_t)0x00008000)
#define FLASH_ECCR_ADDR_ECC_16              ((uint32_t)0x00010000)
#define FLASH_ECCR_ADDR_ECC_17              ((uint32_t)0x00020000)
#define FLASH_ECCR_ADDR_ECC_18              ((uint32_t)0x00040000)*/
#define FLASH_ECCR_BK_ECC                   ((uint32_t)0x00080000)
#define FLASH_ECCR_SYSF_ECC                 ((uint32_t)0x00100000)
#define FLASH_ECCR_ECCIE                    ((uint32_t)0x01000000)
#define FLASH_ECCR_ECCC                     ((uint32_t)0x40000000)
#define FLASH_ECCR_ECCD                     ((uint32_t)0x80000000)

/*******************  Bits definition for FLASH_OPTR register  ***************/
#define FLASH_OPTR_RDP                      ((uint32_t)0x000000FF)
/*#define FLASH_OPTR_RDP_0                    ((uint32_t)0x00000001)
#define FLASH_OPTR_RDP_1                    ((uint32_t)0x00000002)
#define FLASH_OPTR_RDP_2                    ((uint32_t)0x00000004)
#define FLASH_OPTR_RDP_3                    ((uint32_t)0x00000008)
#define FLASH_OPTR_RDP_4                    ((uint32_t)0x00000010)
#define FLASH_OPTR_RDP_5                    ((uint32_t)0x00000020)
#define FLASH_OPTR_RDP_6                    ((uint32_t)0x00000040)
#define FLASH_OPTR_RDP_7                    ((uint32_t)0x00000080)*/
#define FLASH_OPTR_BOR_LEV                  ((uint32_t)0x00000700)
#define FLASH_OPTR_BOR_LEV_0                ((uint32_t)0x00000000)
#define FLASH_OPTR_BOR_LEV_1                ((uint32_t)0x00000100)
#define FLASH_OPTR_BOR_LEV_2                ((uint32_t)0x00000200)
#define FLASH_OPTR_BOR_LEV_3                ((uint32_t)0x00000300)
#define FLASH_OPTR_BOR_LEV_4                ((uint32_t)0x00000400)
#define FLASH_OPTR_nRST_STOP                ((uint32_t)0x00001000)
#define FLASH_OPTR_nRST_STDBY               ((uint32_t)0x00002000)
#define FLASH_OPTR_IWDG_SW                  ((uint32_t)0x00010000)
#define FLASH_OPTR_IWDG_STOP                ((uint32_t)0x00020000)
#define FLASH_OPTR_IWDG_STDBY               ((uint32_t)0x00040000)
#define FLASH_OPTR_WWDG_SW                  ((uint32_t)0x00080000)
#define FLASH_OPTR_BFB2                     ((uint32_t)0x00100000)
#define FLASH_OPTR_DUALBANK                 ((uint32_t)0x00200000)
#define FLASH_OPTR_nBOOT1                   ((uint32_t)0x00800000)
#define FLASH_OPTR_SRAM2_PE                 ((uint32_t)0x01000000)
#define FLASH_OPTR_SRAM2_RST                ((uint32_t)0x02000000)

/******************  Bits definition for FLASH_PCROP1SR register  **********/
#define FLASH_PCROP1SR_PCROP1_STRT          ((uint32_t)0x0000FFFF)

/******************  Bits definition for FLASH_PCROP1ER register  ***********/
#define FLASH_PCROP1ER_PCROP1_END           ((uint32_t)0x0000FFFF)
#define FLASH_PCROP1ER_PCROP_RDP            ((uint32_t)0x80000000)

/******************  Bits definition for FLASH_WRP1AR register  ***************/
#define FLASH_WRP1AR_WRP1A_STRT             ((uint32_t)0x000000FF)
#define FLASH_WRP1AR_WRP1A_END              ((uint32_t)0x00FF0000)

/******************  Bits definition for FLASH_WRPB1R register  ***************/
#define FLASH_WRP1BR_WRP1B_STRT             ((uint32_t)0x000000FF)
#define FLASH_WRP1BR_WRP1B_END              ((uint32_t)0x00FF0000)

/******************  Bits definition for FLASH_PCROP2SR register  **********/
#define FLASH_PCROP2SR_PCROP2_STRT          ((uint32_t)0x0000FFFF)

/******************  Bits definition for FLASH_PCROP2ER register  ***********/
 #define FLASH_PCROP2ER_PCROP2_END          ((uint32_t)0x0000FFFF)

/******************  Bits definition for FLASH_WRP2AR register  ***************/
#define FLASH_WRP2AR_WRP2A_STRT             ((uint32_t)0x000000FF)
#define FLASH_WRP2AR_WRP2A_END              ((uint32_t)0x00FF0000)

/******************  Bits definition for FLASH_WRP2BR register  ***************/
#define FLASH_WRP2BR_WRP2B_STRT             ((uint32_t)0x000000FF)
#define FLASH_WRP2BR_WRP2B_END              ((uint32_t)0x00FF0000)


/******************************************************************************/
/*                                                                            */
/*                          Flexible Memory Controller                        */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for FMC_BCR1 register  *******************/
#define  FMC_BCR1_CCLKEN                    ((uint32_t)0x00100000)        /*!<Continous clock enable     */
#define  FMC_BCR1_WFDIS                     ((uint32_t)0x00200000)        /*!<Write FIFO Disable         */

/******************  Bit definition for FMC_BCRx registers (x=1..4)  *********/
#define  FMC_BCRx_MBKEN                     ((uint32_t)0x00000001)        /*!<Memory bank enable bit                 */
#define  FMC_BCRx_MUXEN                     ((uint32_t)0x00000002)        /*!<Address/data multiplexing enable bit   */

#define  FMC_BCRx_MTYP                      ((uint32_t)0x0000000C)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FMC_BCRx_MTYP_0                    ((uint32_t)0x00000004)        /*!<Bit 0 */
#define  FMC_BCRx_MTYP_1                    ((uint32_t)0x00000008)        /*!<Bit 1 */

#define  FMC_BCRx_MWID                      ((uint32_t)0x00000030)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FMC_BCRx_MWID_0                    ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FMC_BCRx_MWID_1                    ((uint32_t)0x00000020)        /*!<Bit 1 */

#define  FMC_BCRx_FACCEN                    ((uint32_t)0x00000040)        /*!<Flash access enable        */
#define  FMC_BCRx_BURSTEN                   ((uint32_t)0x00000100)        /*!<Burst enable bit           */
#define  FMC_BCRx_WAITPOL                   ((uint32_t)0x00000200)        /*!<Wait signal polarity bit   */
#define  FMC_BCRx_WAITCFG                   ((uint32_t)0x00000800)        /*!<Wait timing configuration  */
#define  FMC_BCRx_WREN                      ((uint32_t)0x00001000)        /*!<Write enable bit           */
#define  FMC_BCRx_WAITEN                    ((uint32_t)0x00002000)        /*!<Wait enable bit            */
#define  FMC_BCRx_EXTMOD                    ((uint32_t)0x00004000)        /*!<Extended mode enable       */
#define  FMC_BCRx_ASYNCWAIT                 ((uint32_t)0x00008000)        /*!<Asynchronous wait          */

#define  FMC_BCRx_CPSIZE                    ((uint32_t)0x00070000)        /*!<CRAM page size             */
#define  FMC_BCRx_CPSIZE_0                  ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FMC_BCRx_CPSIZE_1                  ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FMC_BCRx_CPSIZE_2                  ((uint32_t)0x00040000)        /*!<Bit 1 */

#define  FMC_BCRx_CBURSTRW                  ((uint32_t)0x00080000)        /*!<Write burst enable         */

/******************  Bit definition for FMC_BTRx registers (x=1..4)  *********/
#define  FMC_BTRx_ADDSET                    ((uint32_t)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FMC_BTRx_ADDSET_0                  ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FMC_BTRx_ADDSET_1                  ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FMC_BTRx_ADDSET_2                  ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FMC_BTRx_ADDSET_3                  ((uint32_t)0x00000008)        /*!<Bit 3 */

#define  FMC_BTRx_ADDHLD                    ((uint32_t)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration)  */
#define  FMC_BTRx_ADDHLD_0                  ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FMC_BTRx_ADDHLD_1                  ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  FMC_BTRx_ADDHLD_2                  ((uint32_t)0x00000040)        /*!<Bit 2 */
#define  FMC_BTRx_ADDHLD_3                  ((uint32_t)0x00000080)        /*!<Bit 3 */

#define  FMC_BTRx_DATAST                    ((uint32_t)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FMC_BTRx_DATAST_0                  ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FMC_BTRx_DATAST_1                  ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FMC_BTRx_DATAST_2                  ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FMC_BTRx_DATAST_3                  ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FMC_BTRx_DATAST_4                  ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FMC_BTRx_DATAST_5                  ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FMC_BTRx_DATAST_6                  ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FMC_BTRx_DATAST_7                  ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FMC_BTRx_BUSTURN                   ((uint32_t)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FMC_BTRx_BUSTURN_0                 ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FMC_BTRx_BUSTURN_1                 ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FMC_BTRx_BUSTURN_2                 ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FMC_BTRx_BUSTURN_3                 ((uint32_t)0x00080000)        /*!<Bit 3 */

#define  FMC_BTRx_CLKDIV                    ((uint32_t)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FMC_BTRx_CLKDIV_0                  ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  FMC_BTRx_CLKDIV_1                  ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  FMC_BTRx_CLKDIV_2                  ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  FMC_BTRx_CLKDIV_3                  ((uint32_t)0x00800000)        /*!<Bit 3 */

#define  FMC_BTRx_DATLAT                    ((uint32_t)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FMC_BTRx_DATLAT_0                  ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FMC_BTRx_DATLAT_1                  ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FMC_BTRx_DATLAT_2                  ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FMC_BTRx_DATLAT_3                  ((uint32_t)0x08000000)        /*!<Bit 3 */

#define  FMC_BTRx_ACCMOD                    ((uint32_t)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FMC_BTRx_ACCMOD_0                  ((uint32_t)0x10000000)        /*!<Bit 0 */
#define  FMC_BTRx_ACCMOD_1                  ((uint32_t)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FMC_BWTRx registers (x=1..4)  *********/
#define  FMC_BWTRx_ADDSET                   ((uint32_t)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FMC_BWTRx_ADDSET_0                 ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FMC_BWTRx_ADDSET_1                 ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FMC_BWTRx_ADDSET_2                 ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FMC_BWTRx_ADDSET_3                 ((uint32_t)0x00000008)        /*!<Bit 3 */

#define  FMC_BWTRx_ADDHLD                   ((uint32_t)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FMC_BWTRx_ADDHLD_0                 ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FMC_BWTRx_ADDHLD_1                 ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  FMC_BWTRx_ADDHLD_2                 ((uint32_t)0x00000040)        /*!<Bit 2 */
#define  FMC_BWTRx_ADDHLD_3                 ((uint32_t)0x00000080)        /*!<Bit 3 */

#define  FMC_BWTRx_DATAST                   ((uint32_t)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FMC_BWTRx_DATAST_0                 ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FMC_BWTRx_DATAST_1                 ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FMC_BWTRx_DATAST_2                 ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FMC_BWTRx_DATAST_3                 ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FMC_BWTRx_DATAST_4                 ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FMC_BWTRx_DATAST_5                 ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FMC_BWTRx_DATAST_6                 ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FMC_BWTRx_DATAST_7                 ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FMC_BWTRx_ACCMOD                   ((uint32_t)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FMC_BWTRx_ACCMOD_0                 ((uint32_t)0x10000000)        /*!<Bit 0 */
#define  FMC_BWTRx_ACCMOD_1                 ((uint32_t)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FMC_PCR register  ********************/
#define  FMC_PCR_PWAITEN                    ((uint32_t)0x00000002)        /*!<Wait feature enable bit                   */
#define  FMC_PCR_PBKEN                      ((uint32_t)0x00000004)        /*!<NAND Flash memory bank enable bit */
#define  FMC_PCR_PTYP                       ((uint32_t)0x00000008)        /*!<Memory type                               */

#define  FMC_PCR_PWID                       ((uint32_t)0x00000030)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FMC_PCR_PWID_0                     ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FMC_PCR_PWID_1                     ((uint32_t)0x00000020)        /*!<Bit 1 */

#define  FMC_PCR_ECCEN                      ((uint32_t)0x00000040)        /*!<ECC computation logic enable bit          */

#define  FMC_PCR_TCLR                       ((uint32_t)0x00001E00)        /*!<TCLR[3:0] bits (CLE to RE delay)          */
#define  FMC_PCR_TCLR_0                     ((uint32_t)0x00000200)        /*!<Bit 0 */
#define  FMC_PCR_TCLR_1                     ((uint32_t)0x00000400)        /*!<Bit 1 */
#define  FMC_PCR_TCLR_2                     ((uint32_t)0x00000800)        /*!<Bit 2 */
#define  FMC_PCR_TCLR_3                     ((uint32_t)0x00001000)        /*!<Bit 3 */

#define  FMC_PCR_TAR                        ((uint32_t)0x0001E000)        /*!<TAR[3:0] bits (ALE to RE delay)           */
#define  FMC_PCR_TAR_0                      ((uint32_t)0x00002000)        /*!<Bit 0 */
#define  FMC_PCR_TAR_1                      ((uint32_t)0x00004000)        /*!<Bit 1 */
#define  FMC_PCR_TAR_2                      ((uint32_t)0x00008000)        /*!<Bit 2 */
#define  FMC_PCR_TAR_3                      ((uint32_t)0x00010000)        /*!<Bit 3 */

#define  FMC_PCR_ECCPS                      ((uint32_t)0x000E0000)        /*!<ECCPS[1:0] bits (ECC page size)           */
#define  FMC_PCR_ECCPS_0                    ((uint32_t)0x00020000)        /*!<Bit 0 */
#define  FMC_PCR_ECCPS_1                    ((uint32_t)0x00040000)        /*!<Bit 1 */
#define  FMC_PCR_ECCPS_2                    ((uint32_t)0x00080000)        /*!<Bit 2 */

/*******************  Bit definition for FMC_SR register  ********************/
#define  FMC_SR_IRS                         ((uint32_t)0x00000001)        /*!<Interrupt Rising Edge status                */
#define  FMC_SR_ILS                         ((uint32_t)0x00000002)        /*!<Interrupt Level status                      */
#define  FMC_SR_IFS                         ((uint32_t)0x00000004)        /*!<Interrupt Falling Edge status               */
#define  FMC_SR_IREN                        ((uint32_t)0x00000008)        /*!<Interrupt Rising Edge detection Enable bit  */
#define  FMC_SR_ILEN                        ((uint32_t)0x00000010)        /*!<Interrupt Level detection Enable bit        */
#define  FMC_SR_IFEN                        ((uint32_t)0x00000020)        /*!<Interrupt Falling Edge detection Enable bit */
#define  FMC_SR_FEMPT                       ((uint32_t)0x00000040)        /*!<FIFO empty                                  */

/******************  Bit definition for FMC_PMEM register  ******************/
#define  FMC_PMEM_MEMSET                    ((uint32_t)0x000000FF)        /*!<MEMSET[7:0] bits (Common memory setup time) */
#define  FMC_PMEM_MEMSET_0                  ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FMC_PMEM_MEMSET_1                  ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FMC_PMEM_MEMSET_2                  ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FMC_PMEM_MEMSET_3                  ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  FMC_PMEM_MEMSET_4                  ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  FMC_PMEM_MEMSET_5                  ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  FMC_PMEM_MEMSET_6                  ((uint32_t)0x00000040)        /*!<Bit 6 */
#define  FMC_PMEM_MEMSET_7                  ((uint32_t)0x00000080)        /*!<Bit 7 */

#define  FMC_PMEM_MEMWAIT                   ((uint32_t)0x0000FF00)        /*!<MEMWAIT[7:0] bits (Common memory wait time) */
#define  FMC_PMEM_MEMWAIT_0                 ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FMC_PMEM_MEMWAIT_1                 ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FMC_PMEM_MEMWAIT_2                 ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FMC_PMEM_MEMWAIT_3                 ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FMC_PMEM_MEMWAIT_4                 ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FMC_PMEM_MEMWAIT_5                 ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FMC_PMEM_MEMWAIT_6                 ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FMC_PMEM_MEMWAIT_7                 ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FMC_PMEM_MEMHOLD                   ((uint32_t)0x00FF0000)        /*!<MEMHOLD[7:0] bits (Common memory hold time) */
#define  FMC_PMEM_MEMHOLD_0                 ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FMC_PMEM_MEMHOLD_1                 ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FMC_PMEM_MEMHOLD_2                 ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FMC_PMEM_MEMHOLD_3                 ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  FMC_PMEM_MEMHOLD_4                 ((uint32_t)0x00100000)        /*!<Bit 4 */
#define  FMC_PMEM_MEMHOLD_5                 ((uint32_t)0x00200000)        /*!<Bit 5 */
#define  FMC_PMEM_MEMHOLD_6                 ((uint32_t)0x00400000)        /*!<Bit 6 */
#define  FMC_PMEM_MEMHOLD_7                 ((uint32_t)0x00800000)        /*!<Bit 7 */

#define  FMC_PMEM_MEMHIZ                    ((uint32_t)0xFF000000)        /*!<MEMHIZ[7:0] bits (Common memory databus HiZ time) */
#define  FMC_PMEM_MEMHIZ_0                  ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FMC_PMEM_MEMHIZ_1                  ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FMC_PMEM_MEMHIZ_2                  ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FMC_PMEM_MEMHIZ_3                  ((uint32_t)0x08000000)        /*!<Bit 3 */
#define  FMC_PMEM_MEMHIZ_4                  ((uint32_t)0x10000000)        /*!<Bit 4 */
#define  FMC_PMEM_MEMHIZ_5                  ((uint32_t)0x20000000)        /*!<Bit 5 */
#define  FMC_PMEM_MEMHIZ_6                  ((uint32_t)0x40000000)        /*!<Bit 6 */
#define  FMC_PMEM_MEMHIZ_7                  ((uint32_t)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FMC_PATT register  *******************/
#define  FMC_PATT_ATTSET                    ((uint32_t)0x000000FF)        /*!<ATTSET[7:0] bits (Attribute memory setup time) */
#define  FMC_PATT_ATTSET_0                  ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FMC_PATT_ATTSET_1                  ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FMC_PATT_ATTSET_2                  ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FMC_PATT_ATTSET_3                  ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  FMC_PATT_ATTSET_4                  ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  FMC_PATT_ATTSET_5                  ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  FMC_PATT_ATTSET_6                  ((uint32_t)0x00000040)        /*!<Bit 6 */
#define  FMC_PATT_ATTSET_7                  ((uint32_t)0x00000080)        /*!<Bit 7 */

#define  FMC_PATT_ATTWAIT                   ((uint32_t)0x0000FF00)        /*!<ATTWAIT[7:0] bits (Attribute memory wait time) */
#define  FMC_PATT_ATTWAIT_0                 ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FMC_PATT_ATTWAIT_1                 ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FMC_PATT_ATTWAIT_2                 ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FMC_PATT_ATTWAIT_3                 ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FMC_PATT_ATTWAIT_4                 ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FMC_PATT_ATTWAIT_5                 ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FMC_PATT_ATTWAIT_6                 ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FMC_PATT_ATTWAIT_7                 ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FMC_PATT_ATTHOLD                   ((uint32_t)0x00FF0000)        /*!<ATTHOLD[7:0] bits (Attribute memory hold time) */
#define  FMC_PATT_ATTHOLD_0                 ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FMC_PATT_ATTHOLD_1                 ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FMC_PATT_ATTHOLD_2                 ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FMC_PATT_ATTHOLD_3                 ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  FMC_PATT_ATTHOLD_4                 ((uint32_t)0x00100000)        /*!<Bit 4 */
#define  FMC_PATT_ATTHOLD_5                 ((uint32_t)0x00200000)        /*!<Bit 5 */
#define  FMC_PATT_ATTHOLD_6                 ((uint32_t)0x00400000)        /*!<Bit 6 */
#define  FMC_PATT_ATTHOLD_7                 ((uint32_t)0x00800000)        /*!<Bit 7 */

#define  FMC_PATT_ATTHIZ                    ((uint32_t)0xFF000000)        /*!<ATTHIZ[7:0] bits (Attribute memory databus HiZ time) */
#define  FMC_PATT_ATTHIZ_0                  ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FMC_PATT_ATTHIZ_1                  ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FMC_PATT_ATTHIZ_2                  ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FMC_PATT_ATTHIZ_3                  ((uint32_t)0x08000000)        /*!<Bit 3 */
#define  FMC_PATT_ATTHIZ_4                  ((uint32_t)0x10000000)        /*!<Bit 4 */
#define  FMC_PATT_ATTHIZ_5                  ((uint32_t)0x20000000)        /*!<Bit 5 */
#define  FMC_PATT_ATTHIZ_6                  ((uint32_t)0x40000000)        /*!<Bit 6 */
#define  FMC_PATT_ATTHIZ_7                  ((uint32_t)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FMC_ECCR register  *******************/
#define  FMC_ECCR_ECC                       ((uint32_t)0xFFFFFFFF)        /*!<ECC result */

/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0                    ((uint32_t)0x00000003)
#define GPIO_MODER_MODER0_0                  ((uint32_t)0x00000001)
#define GPIO_MODER_MODER0_1                  ((uint32_t)0x00000002)
#define GPIO_MODER_MODER1                    ((uint32_t)0x0000000C)
#define GPIO_MODER_MODER1_0                  ((uint32_t)0x00000004)
#define GPIO_MODER_MODER1_1                  ((uint32_t)0x00000008)
#define GPIO_MODER_MODER2                    ((uint32_t)0x00000030)
#define GPIO_MODER_MODER2_0                  ((uint32_t)0x00000010)
#define GPIO_MODER_MODER2_1                  ((uint32_t)0x00000020)
#define GPIO_MODER_MODER3                    ((uint32_t)0x000000C0)
#define GPIO_MODER_MODER3_0                  ((uint32_t)0x00000040)
#define GPIO_MODER_MODER3_1                  ((uint32_t)0x00000080)
#define GPIO_MODER_MODER4                    ((uint32_t)0x00000300)
#define GPIO_MODER_MODER4_0                  ((uint32_t)0x00000100)
#define GPIO_MODER_MODER4_1                  ((uint32_t)0x00000200)
#define GPIO_MODER_MODER5                    ((uint32_t)0x00000C00)
#define GPIO_MODER_MODER5_0                  ((uint32_t)0x00000400)
#define GPIO_MODER_MODER5_1                  ((uint32_t)0x00000800)
#define GPIO_MODER_MODER6                    ((uint32_t)0x00003000)
#define GPIO_MODER_MODER6_0                  ((uint32_t)0x00001000)
#define GPIO_MODER_MODER6_1                  ((uint32_t)0x00002000)
#define GPIO_MODER_MODER7                    ((uint32_t)0x0000C000)
#define GPIO_MODER_MODER7_0                  ((uint32_t)0x00004000)
#define GPIO_MODER_MODER7_1                  ((uint32_t)0x00008000)
#define GPIO_MODER_MODER8                    ((uint32_t)0x00030000)
#define GPIO_MODER_MODER8_0                  ((uint32_t)0x00010000)
#define GPIO_MODER_MODER8_1                  ((uint32_t)0x00020000)
#define GPIO_MODER_MODER9                    ((uint32_t)0x000C0000)
#define GPIO_MODER_MODER9_0                  ((uint32_t)0x00040000)
#define GPIO_MODER_MODER9_1                  ((uint32_t)0x00080000)
#define GPIO_MODER_MODER10                   ((uint32_t)0x00300000)
#define GPIO_MODER_MODER10_0                 ((uint32_t)0x00100000)
#define GPIO_MODER_MODER10_1                 ((uint32_t)0x00200000)
#define GPIO_MODER_MODER11                   ((uint32_t)0x00C00000)
#define GPIO_MODER_MODER11_0                 ((uint32_t)0x00400000)
#define GPIO_MODER_MODER11_1                 ((uint32_t)0x00800000)
#define GPIO_MODER_MODER12                   ((uint32_t)0x03000000)
#define GPIO_MODER_MODER12_0                 ((uint32_t)0x01000000)
#define GPIO_MODER_MODER12_1                 ((uint32_t)0x02000000)
#define GPIO_MODER_MODER13                   ((uint32_t)0x0C000000)
#define GPIO_MODER_MODER13_0                 ((uint32_t)0x04000000)
#define GPIO_MODER_MODER13_1                 ((uint32_t)0x08000000)
#define GPIO_MODER_MODER14                   ((uint32_t)0x30000000)
#define GPIO_MODER_MODER14_0                 ((uint32_t)0x10000000)
#define GPIO_MODER_MODER14_1                 ((uint32_t)0x20000000)
#define GPIO_MODER_MODER15                   ((uint32_t)0xC0000000)
#define GPIO_MODER_MODER15_0                 ((uint32_t)0x40000000)
#define GPIO_MODER_MODER15_1                 ((uint32_t)0x80000000)

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT_0                     ((uint32_t)0x00000001)
#define GPIO_OTYPER_OT_1                     ((uint32_t)0x00000002)
#define GPIO_OTYPER_OT_2                     ((uint32_t)0x00000004)
#define GPIO_OTYPER_OT_3                     ((uint32_t)0x00000008)
#define GPIO_OTYPER_OT_4                     ((uint32_t)0x00000010)
#define GPIO_OTYPER_OT_5                     ((uint32_t)0x00000020)
#define GPIO_OTYPER_OT_6                     ((uint32_t)0x00000040)
#define GPIO_OTYPER_OT_7                     ((uint32_t)0x00000080)
#define GPIO_OTYPER_OT_8                     ((uint32_t)0x00000100)
#define GPIO_OTYPER_OT_9                     ((uint32_t)0x00000200)
#define GPIO_OTYPER_OT_10                    ((uint32_t)0x00000400)
#define GPIO_OTYPER_OT_11                    ((uint32_t)0x00000800)
#define GPIO_OTYPER_OT_12                    ((uint32_t)0x00001000)
#define GPIO_OTYPER_OT_13                    ((uint32_t)0x00002000)
#define GPIO_OTYPER_OT_14                    ((uint32_t)0x00004000)
#define GPIO_OTYPER_OT_15                    ((uint32_t)0x00008000)

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDER_OSPEEDR0               ((uint32_t)0x00000003)
#define GPIO_OSPEEDER_OSPEEDR0_0             ((uint32_t)0x00000001)
#define GPIO_OSPEEDER_OSPEEDR0_1             ((uint32_t)0x00000002)
#define GPIO_OSPEEDER_OSPEEDR1               ((uint32_t)0x0000000C)
#define GPIO_OSPEEDER_OSPEEDR1_0             ((uint32_t)0x00000004)
#define GPIO_OSPEEDER_OSPEEDR1_1             ((uint32_t)0x00000008)
#define GPIO_OSPEEDER_OSPEEDR2               ((uint32_t)0x00000030)
#define GPIO_OSPEEDER_OSPEEDR2_0             ((uint32_t)0x00000010)
#define GPIO_OSPEEDER_OSPEEDR2_1             ((uint32_t)0x00000020)
#define GPIO_OSPEEDER_OSPEEDR3               ((uint32_t)0x000000C0)
#define GPIO_OSPEEDER_OSPEEDR3_0             ((uint32_t)0x00000040)
#define GPIO_OSPEEDER_OSPEEDR3_1             ((uint32_t)0x00000080)
#define GPIO_OSPEEDER_OSPEEDR4               ((uint32_t)0x00000300)
#define GPIO_OSPEEDER_OSPEEDR4_0             ((uint32_t)0x00000100)
#define GPIO_OSPEEDER_OSPEEDR4_1             ((uint32_t)0x00000200)
#define GPIO_OSPEEDER_OSPEEDR5               ((uint32_t)0x00000C00)
#define GPIO_OSPEEDER_OSPEEDR5_0             ((uint32_t)0x00000400)
#define GPIO_OSPEEDER_OSPEEDR5_1             ((uint32_t)0x00000800)
#define GPIO_OSPEEDER_OSPEEDR6               ((uint32_t)0x00003000)
#define GPIO_OSPEEDER_OSPEEDR6_0             ((uint32_t)0x00001000)
#define GPIO_OSPEEDER_OSPEEDR6_1             ((uint32_t)0x00002000)
#define GPIO_OSPEEDER_OSPEEDR7               ((uint32_t)0x0000C000)
#define GPIO_OSPEEDER_OSPEEDR7_0             ((uint32_t)0x00004000)
#define GPIO_OSPEEDER_OSPEEDR7_1             ((uint32_t)0x00008000)
#define GPIO_OSPEEDER_OSPEEDR8               ((uint32_t)0x00030000)
#define GPIO_OSPEEDER_OSPEEDR8_0             ((uint32_t)0x00010000)
#define GPIO_OSPEEDER_OSPEEDR8_1             ((uint32_t)0x00020000)
#define GPIO_OSPEEDER_OSPEEDR9               ((uint32_t)0x000C0000)
#define GPIO_OSPEEDER_OSPEEDR9_0             ((uint32_t)0x00040000)
#define GPIO_OSPEEDER_OSPEEDR9_1             ((uint32_t)0x00080000)
#define GPIO_OSPEEDER_OSPEEDR10              ((uint32_t)0x00300000)
#define GPIO_OSPEEDER_OSPEEDR10_0            ((uint32_t)0x00100000)
#define GPIO_OSPEEDER_OSPEEDR10_1            ((uint32_t)0x00200000)
#define GPIO_OSPEEDER_OSPEEDR11              ((uint32_t)0x00C00000)
#define GPIO_OSPEEDER_OSPEEDR11_0            ((uint32_t)0x00400000)
#define GPIO_OSPEEDER_OSPEEDR11_1            ((uint32_t)0x00800000)
#define GPIO_OSPEEDER_OSPEEDR12              ((uint32_t)0x03000000)
#define GPIO_OSPEEDER_OSPEEDR12_0            ((uint32_t)0x01000000)
#define GPIO_OSPEEDER_OSPEEDR12_1            ((uint32_t)0x02000000)
#define GPIO_OSPEEDER_OSPEEDR13              ((uint32_t)0x0C000000)
#define GPIO_OSPEEDER_OSPEEDR13_0            ((uint32_t)0x04000000)
#define GPIO_OSPEEDER_OSPEEDR13_1            ((uint32_t)0x08000000)
#define GPIO_OSPEEDER_OSPEEDR14              ((uint32_t)0x30000000)
#define GPIO_OSPEEDER_OSPEEDR14_0            ((uint32_t)0x10000000)
#define GPIO_OSPEEDER_OSPEEDR14_1            ((uint32_t)0x20000000)
#define GPIO_OSPEEDER_OSPEEDR15              ((uint32_t)0xC0000000)
#define GPIO_OSPEEDER_OSPEEDR15_0            ((uint32_t)0x40000000)
#define GPIO_OSPEEDER_OSPEEDR15_1            ((uint32_t)0x80000000)

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPDR0                    ((uint32_t)0x00000003)
#define GPIO_PUPDR_PUPDR0_0                  ((uint32_t)0x00000001)
#define GPIO_PUPDR_PUPDR0_1                  ((uint32_t)0x00000002)
#define GPIO_PUPDR_PUPDR1                    ((uint32_t)0x0000000C)
#define GPIO_PUPDR_PUPDR1_0                  ((uint32_t)0x00000004)
#define GPIO_PUPDR_PUPDR1_1                  ((uint32_t)0x00000008)
#define GPIO_PUPDR_PUPDR2                    ((uint32_t)0x00000030)
#define GPIO_PUPDR_PUPDR2_0                  ((uint32_t)0x00000010)
#define GPIO_PUPDR_PUPDR2_1                  ((uint32_t)0x00000020)
#define GPIO_PUPDR_PUPDR3                    ((uint32_t)0x000000C0)
#define GPIO_PUPDR_PUPDR3_0                  ((uint32_t)0x00000040)
#define GPIO_PUPDR_PUPDR3_1                  ((uint32_t)0x00000080)
#define GPIO_PUPDR_PUPDR4                    ((uint32_t)0x00000300)
#define GPIO_PUPDR_PUPDR4_0                  ((uint32_t)0x00000100)
#define GPIO_PUPDR_PUPDR4_1                  ((uint32_t)0x00000200)
#define GPIO_PUPDR_PUPDR5                    ((uint32_t)0x00000C00)
#define GPIO_PUPDR_PUPDR5_0                  ((uint32_t)0x00000400)
#define GPIO_PUPDR_PUPDR5_1                  ((uint32_t)0x00000800)
#define GPIO_PUPDR_PUPDR6                    ((uint32_t)0x00003000)
#define GPIO_PUPDR_PUPDR6_0                  ((uint32_t)0x00001000)
#define GPIO_PUPDR_PUPDR6_1                  ((uint32_t)0x00002000)
#define GPIO_PUPDR_PUPDR7                    ((uint32_t)0x0000C000)
#define GPIO_PUPDR_PUPDR7_0                  ((uint32_t)0x00004000)
#define GPIO_PUPDR_PUPDR7_1                  ((uint32_t)0x00008000)
#define GPIO_PUPDR_PUPDR8                    ((uint32_t)0x00030000)
#define GPIO_PUPDR_PUPDR8_0                  ((uint32_t)0x00010000)
#define GPIO_PUPDR_PUPDR8_1                  ((uint32_t)0x00020000)
#define GPIO_PUPDR_PUPDR9                    ((uint32_t)0x000C0000)
#define GPIO_PUPDR_PUPDR9_0                  ((uint32_t)0x00040000)
#define GPIO_PUPDR_PUPDR9_1                  ((uint32_t)0x00080000)
#define GPIO_PUPDR_PUPDR10                   ((uint32_t)0x00300000)
#define GPIO_PUPDR_PUPDR10_0                 ((uint32_t)0x00100000)
#define GPIO_PUPDR_PUPDR10_1                 ((uint32_t)0x00200000)
#define GPIO_PUPDR_PUPDR11                   ((uint32_t)0x00C00000)
#define GPIO_PUPDR_PUPDR11_0                 ((uint32_t)0x00400000)
#define GPIO_PUPDR_PUPDR11_1                 ((uint32_t)0x00800000)
#define GPIO_PUPDR_PUPDR12                   ((uint32_t)0x03000000)
#define GPIO_PUPDR_PUPDR12_0                 ((uint32_t)0x01000000)
#define GPIO_PUPDR_PUPDR12_1                 ((uint32_t)0x02000000)
#define GPIO_PUPDR_PUPDR13                   ((uint32_t)0x0C000000)
#define GPIO_PUPDR_PUPDR13_0                 ((uint32_t)0x04000000)
#define GPIO_PUPDR_PUPDR13_1                 ((uint32_t)0x08000000)
#define GPIO_PUPDR_PUPDR14                   ((uint32_t)0x30000000)
#define GPIO_PUPDR_PUPDR14_0                 ((uint32_t)0x10000000)
#define GPIO_PUPDR_PUPDR14_1                 ((uint32_t)0x20000000)
#define GPIO_PUPDR_PUPDR15                   ((uint32_t)0xC0000000)
#define GPIO_PUPDR_PUPDR15_0                 ((uint32_t)0x40000000)
#define GPIO_PUPDR_PUPDR15_1                 ((uint32_t)0x80000000)

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR_0                       ((uint32_t)0x00000001)
#define GPIO_IDR_IDR_1                       ((uint32_t)0x00000002)
#define GPIO_IDR_IDR_2                       ((uint32_t)0x00000004)
#define GPIO_IDR_IDR_3                       ((uint32_t)0x00000008)
#define GPIO_IDR_IDR_4                       ((uint32_t)0x00000010)
#define GPIO_IDR_IDR_5                       ((uint32_t)0x00000020)
#define GPIO_IDR_IDR_6                       ((uint32_t)0x00000040)
#define GPIO_IDR_IDR_7                       ((uint32_t)0x00000080)
#define GPIO_IDR_IDR_8                       ((uint32_t)0x00000100)
#define GPIO_IDR_IDR_9                       ((uint32_t)0x00000200)
#define GPIO_IDR_IDR_10                      ((uint32_t)0x00000400)
#define GPIO_IDR_IDR_11                      ((uint32_t)0x00000800)
#define GPIO_IDR_IDR_12                      ((uint32_t)0x00001000)
#define GPIO_IDR_IDR_13                      ((uint32_t)0x00002000)
#define GPIO_IDR_IDR_14                      ((uint32_t)0x00004000)
#define GPIO_IDR_IDR_15                      ((uint32_t)0x00008000)

/* Old GPIO_IDR register bits definition, maintained for legacy purpose */
#define GPIO_OTYPER_IDR_0                    GPIO_IDR_IDR_0
#define GPIO_OTYPER_IDR_1                    GPIO_IDR_IDR_1
#define GPIO_OTYPER_IDR_2                    GPIO_IDR_IDR_2
#define GPIO_OTYPER_IDR_3                    GPIO_IDR_IDR_3
#define GPIO_OTYPER_IDR_4                    GPIO_IDR_IDR_4
#define GPIO_OTYPER_IDR_5                    GPIO_IDR_IDR_5
#define GPIO_OTYPER_IDR_6                    GPIO_IDR_IDR_6
#define GPIO_OTYPER_IDR_7                    GPIO_IDR_IDR_7
#define GPIO_OTYPER_IDR_8                    GPIO_IDR_IDR_8
#define GPIO_OTYPER_IDR_9                    GPIO_IDR_IDR_9
#define GPIO_OTYPER_IDR_10                   GPIO_IDR_IDR_10
#define GPIO_OTYPER_IDR_11                   GPIO_IDR_IDR_11
#define GPIO_OTYPER_IDR_12                   GPIO_IDR_IDR_12
#define GPIO_OTYPER_IDR_13                   GPIO_IDR_IDR_13
#define GPIO_OTYPER_IDR_14                   GPIO_IDR_IDR_14
#define GPIO_OTYPER_IDR_15                   GPIO_IDR_IDR_15

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR_0                       ((uint32_t)0x00000001)
#define GPIO_ODR_ODR_1                       ((uint32_t)0x00000002)
#define GPIO_ODR_ODR_2                       ((uint32_t)0x00000004)
#define GPIO_ODR_ODR_3                       ((uint32_t)0x00000008)
#define GPIO_ODR_ODR_4                       ((uint32_t)0x00000010)
#define GPIO_ODR_ODR_5                       ((uint32_t)0x00000020)
#define GPIO_ODR_ODR_6                       ((uint32_t)0x00000040)
#define GPIO_ODR_ODR_7                       ((uint32_t)0x00000080)
#define GPIO_ODR_ODR_8                       ((uint32_t)0x00000100)
#define GPIO_ODR_ODR_9                       ((uint32_t)0x00000200)
#define GPIO_ODR_ODR_10                      ((uint32_t)0x00000400)
#define GPIO_ODR_ODR_11                      ((uint32_t)0x00000800)
#define GPIO_ODR_ODR_12                      ((uint32_t)0x00001000)
#define GPIO_ODR_ODR_13                      ((uint32_t)0x00002000)
#define GPIO_ODR_ODR_14                      ((uint32_t)0x00004000)
#define GPIO_ODR_ODR_15                      ((uint32_t)0x00008000)

/* Old GPIO_ODR register bits definition, maintained for legacy purpose */
#define GPIO_OTYPER_ODR_0                    GPIO_ODR_ODR_0
#define GPIO_OTYPER_ODR_1                    GPIO_ODR_ODR_1
#define GPIO_OTYPER_ODR_2                    GPIO_ODR_ODR_2
#define GPIO_OTYPER_ODR_3                    GPIO_ODR_ODR_3
#define GPIO_OTYPER_ODR_4                    GPIO_ODR_ODR_4
#define GPIO_OTYPER_ODR_5                    GPIO_ODR_ODR_5
#define GPIO_OTYPER_ODR_6                    GPIO_ODR_ODR_6
#define GPIO_OTYPER_ODR_7                    GPIO_ODR_ODR_7
#define GPIO_OTYPER_ODR_8                    GPIO_ODR_ODR_8
#define GPIO_OTYPER_ODR_9                    GPIO_ODR_ODR_9
#define GPIO_OTYPER_ODR_10                   GPIO_ODR_ODR_10
#define GPIO_OTYPER_ODR_11                   GPIO_ODR_ODR_11
#define GPIO_OTYPER_ODR_12                   GPIO_ODR_ODR_12
#define GPIO_OTYPER_ODR_13                   GPIO_ODR_ODR_13
#define GPIO_OTYPER_ODR_14                   GPIO_ODR_ODR_14
#define GPIO_OTYPER_ODR_15                   GPIO_ODR_ODR_15

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS_0                       ((uint32_t)0x00000001)
#define GPIO_BSRR_BS_1                       ((uint32_t)0x00000002)
#define GPIO_BSRR_BS_2                       ((uint32_t)0x00000004)
#define GPIO_BSRR_BS_3                       ((uint32_t)0x00000008)
#define GPIO_BSRR_BS_4                       ((uint32_t)0x00000010)
#define GPIO_BSRR_BS_5                       ((uint32_t)0x00000020)
#define GPIO_BSRR_BS_6                       ((uint32_t)0x00000040)
#define GPIO_BSRR_BS_7                       ((uint32_t)0x00000080)
#define GPIO_BSRR_BS_8                       ((uint32_t)0x00000100)
#define GPIO_BSRR_BS_9                       ((uint32_t)0x00000200)
#define GPIO_BSRR_BS_10                      ((uint32_t)0x00000400)
#define GPIO_BSRR_BS_11                      ((uint32_t)0x00000800)
#define GPIO_BSRR_BS_12                      ((uint32_t)0x00001000)
#define GPIO_BSRR_BS_13                      ((uint32_t)0x00002000)
#define GPIO_BSRR_BS_14                      ((uint32_t)0x00004000)
#define GPIO_BSRR_BS_15                      ((uint32_t)0x00008000)
#define GPIO_BSRR_BR_0                       ((uint32_t)0x00010000)
#define GPIO_BSRR_BR_1                       ((uint32_t)0x00020000)
#define GPIO_BSRR_BR_2                       ((uint32_t)0x00040000)
#define GPIO_BSRR_BR_3                       ((uint32_t)0x00080000)
#define GPIO_BSRR_BR_4                       ((uint32_t)0x00100000)
#define GPIO_BSRR_BR_5                       ((uint32_t)0x00200000)
#define GPIO_BSRR_BR_6                       ((uint32_t)0x00400000)
#define GPIO_BSRR_BR_7                       ((uint32_t)0x00800000)
#define GPIO_BSRR_BR_8                       ((uint32_t)0x01000000)
#define GPIO_BSRR_BR_9                       ((uint32_t)0x02000000)
#define GPIO_BSRR_BR_10                      ((uint32_t)0x04000000)
#define GPIO_BSRR_BR_11                      ((uint32_t)0x08000000)
#define GPIO_BSRR_BR_12                      ((uint32_t)0x10000000)
#define GPIO_BSRR_BR_13                      ((uint32_t)0x20000000)
#define GPIO_BSRR_BR_14                      ((uint32_t)0x40000000)
#define GPIO_BSRR_BR_15                      ((uint32_t)0x80000000)

/******************  Bits definition for GPIO_BRR register  ******************/
#define GPIO_BRR_BR_0                       ((uint32_t)0x00000001)
#define GPIO_BRR_BR_1                       ((uint32_t)0x00000002)
#define GPIO_BRR_BR_2                       ((uint32_t)0x00000004)
#define GPIO_BRR_BR_3                       ((uint32_t)0x00000008)
#define GPIO_BRR_BR_4                       ((uint32_t)0x00000010)
#define GPIO_BRR_BR_5                       ((uint32_t)0x00000020)
#define GPIO_BRR_BR_6                       ((uint32_t)0x00000040)
#define GPIO_BRR_BR_7                       ((uint32_t)0x00000080)
#define GPIO_BRR_BR_8                       ((uint32_t)0x00000100)
#define GPIO_BRR_BR_9                       ((uint32_t)0x00000200)
#define GPIO_BRR_BR_10                      ((uint32_t)0x00000400)
#define GPIO_BRR_BR_11                      ((uint32_t)0x00000800)
#define GPIO_BRR_BR_12                      ((uint32_t)0x00001000)
#define GPIO_BRR_BR_13                      ((uint32_t)0x00002000)
#define GPIO_BRR_BR_14                      ((uint32_t)0x00004000)
#define GPIO_BRR_BR_15                      ((uint32_t)0x00008000)

/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0                       ((uint32_t)0x00000001)
#define GPIO_LCKR_LCK1                       ((uint32_t)0x00000002)
#define GPIO_LCKR_LCK2                       ((uint32_t)0x00000004)
#define GPIO_LCKR_LCK3                       ((uint32_t)0x00000008)
#define GPIO_LCKR_LCK4                       ((uint32_t)0x00000010)
#define GPIO_LCKR_LCK5                       ((uint32_t)0x00000020)
#define GPIO_LCKR_LCK6                       ((uint32_t)0x00000040)
#define GPIO_LCKR_LCK7                       ((uint32_t)0x00000080)
#define GPIO_LCKR_LCK8                       ((uint32_t)0x00000100)
#define GPIO_LCKR_LCK9                       ((uint32_t)0x00000200)
#define GPIO_LCKR_LCK10                      ((uint32_t)0x00000400)
#define GPIO_LCKR_LCK11                      ((uint32_t)0x00000800)
#define GPIO_LCKR_LCK12                      ((uint32_t)0x00001000)
#define GPIO_LCKR_LCK13                      ((uint32_t)0x00002000)
#define GPIO_LCKR_LCK14                      ((uint32_t)0x00004000)
#define GPIO_LCKR_LCK15                      ((uint32_t)0x00008000)
#define GPIO_LCKR_LCKK                       ((uint32_t)0x00010000)

/****************** Bit definition for GPIO_AFRL register  ********************/
#define GPIO_AFRL_AFRL0                      ((uint32_t)0x0000000F)
#define GPIO_AFRL_AFRL1                      ((uint32_t)0x000000F0)
#define GPIO_AFRL_AFRL2                      ((uint32_t)0x00000F00)
#define GPIO_AFRL_AFRL3                      ((uint32_t)0x0000F000)
#define GPIO_AFRL_AFRL4                      ((uint32_t)0x000F0000)
#define GPIO_AFRL_AFRL5                      ((uint32_t)0x00F00000)
#define GPIO_AFRL_AFRL6                      ((uint32_t)0x0F000000)
#define GPIO_AFRL_AFRL7                      ((uint32_t)0xF0000000)

/****************** Bit definition for GPIO_AFRH register  ********************/
#define GPIO_AFRH_AFRH0                      ((uint32_t)0x0000000F)
#define GPIO_AFRH_AFRH1                      ((uint32_t)0x000000F0)
#define GPIO_AFRH_AFRH2                      ((uint32_t)0x00000F00)
#define GPIO_AFRH_AFRH3                      ((uint32_t)0x0000F000)
#define GPIO_AFRH_AFRH4                      ((uint32_t)0x000F0000)
#define GPIO_AFRH_AFRH5                      ((uint32_t)0x00F00000)
#define GPIO_AFRH_AFRH6                      ((uint32_t)0x0F000000)
#define GPIO_AFRH_AFRH7                      ((uint32_t)0xF0000000)

/******************  Bits definition for GPIO_ASCR register  *******************/
#define GPIO_ASCR_EN_0                       ((uint32_t)0x00000001)
#define GPIO_ASCR_EN_1                       ((uint32_t)0x00000002)
#define GPIO_ASCR_EN_2                       ((uint32_t)0x00000004)
#define GPIO_ASCR_EN_3                       ((uint32_t)0x00000008)
#define GPIO_ASCR_EN_4                       ((uint32_t)0x00000010)
#define GPIO_ASCR_EN_5                       ((uint32_t)0x00000020)
#define GPIO_ASCR_EN_6                       ((uint32_t)0x00000040)
#define GPIO_ASCR_EN_7                       ((uint32_t)0x00000080)
#define GPIO_ASCR_EN_8                       ((uint32_t)0x00000100)
#define GPIO_ASCR_EN_9                       ((uint32_t)0x00000200)
#define GPIO_ASCR_EN_10                      ((uint32_t)0x00000400)
#define GPIO_ASCR_EN_11                      ((uint32_t)0x00000800)
#define GPIO_ASCR_EN_12                      ((uint32_t)0x00001000)
#define GPIO_ASCR_EN_13                      ((uint32_t)0x00002000)
#define GPIO_ASCR_EN_14                      ((uint32_t)0x00004000)
#define GPIO_ASCR_EN_15                      ((uint32_t)0x00008000)

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface (I2C)              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  *******************/
#define  I2C_CR1_PE                          ((uint32_t)0x00000001)        /*!< Peripheral enable                   */
#define  I2C_CR1_TXIE                        ((uint32_t)0x00000002)        /*!< TX interrupt enable                 */
#define  I2C_CR1_RXIE                        ((uint32_t)0x00000004)        /*!< RX interrupt enable                 */
#define  I2C_CR1_ADDRIE                      ((uint32_t)0x00000008)        /*!< Address match interrupt enable      */
#define  I2C_CR1_NACKIE                      ((uint32_t)0x00000010)        /*!< NACK received interrupt enable      */
#define  I2C_CR1_STOPIE                      ((uint32_t)0x00000020)        /*!< STOP detection interrupt enable     */
#define  I2C_CR1_TCIE                        ((uint32_t)0x00000040)        /*!< Transfer complete interrupt enable  */
#define  I2C_CR1_ERRIE                       ((uint32_t)0x00000080)        /*!< Errors interrupt enable             */
#define  I2C_CR1_DFN                         ((uint32_t)0x00000F00)        /*!< Digital noise filter                */
#define  I2C_CR1_ANFOFF                      ((uint32_t)0x00001000)        /*!< Analog noise filter OFF             */
#define  I2C_CR1_SWRST                       ((uint32_t)0x00002000)        /*!< Software reset                      */
#define  I2C_CR1_TXDMAEN                     ((uint32_t)0x00004000)        /*!< DMA transmission requests enable    */
#define  I2C_CR1_RXDMAEN                     ((uint32_t)0x00008000)        /*!< DMA reception requests enable       */
#define  I2C_CR1_SBC                         ((uint32_t)0x00010000)        /*!< Slave byte control                  */
#define  I2C_CR1_NOSTRETCH                   ((uint32_t)0x00020000)        /*!< Clock stretching disable            */
#define  I2C_CR1_WUPEN                       ((uint32_t)0x00040000)        /*!< Wakeup from STOP enable             */
#define  I2C_CR1_GCEN                        ((uint32_t)0x00080000)        /*!< General call enable                 */
#define  I2C_CR1_SMBHEN                      ((uint32_t)0x00100000)        /*!< SMBus host address enable           */
#define  I2C_CR1_SMBDEN                      ((uint32_t)0x00200000)        /*!< SMBus device default address enable */
#define  I2C_CR1_ALERTEN                     ((uint32_t)0x00400000)        /*!< SMBus alert enable                  */
#define  I2C_CR1_PECEN                       ((uint32_t)0x00800000)        /*!< PEC enable                          */

/******************  Bit definition for I2C_CR2 register  ********************/
#define  I2C_CR2_SADD                        ((uint32_t)0x000003FF)        /*!< Slave address (master mode)                             */
#define  I2C_CR2_RD_WRN                      ((uint32_t)0x00000400)        /*!< Transfer direction (master mode)                        */
#define  I2C_CR2_ADD10                       ((uint32_t)0x00000800)        /*!< 10-bit addressing mode (master mode)                    */
#define  I2C_CR2_HEAD10R                     ((uint32_t)0x00001000)        /*!< 10-bit address header only read direction (master mode) */
#define  I2C_CR2_START                       ((uint32_t)0x00002000)        /*!< START generation                                        */
#define  I2C_CR2_STOP                        ((uint32_t)0x00004000)        /*!< STOP generation (master mode)                           */
#define  I2C_CR2_NACK                        ((uint32_t)0x00008000)        /*!< NACK generation (slave mode)                            */
#define  I2C_CR2_NBYTES                      ((uint32_t)0x00FF0000)        /*!< Number of bytes                                         */
#define  I2C_CR2_RELOAD                      ((uint32_t)0x01000000)        /*!< NBYTES reload mode                                      */
#define  I2C_CR2_AUTOEND                     ((uint32_t)0x02000000)        /*!< Automatic end mode (master mode)                        */
#define  I2C_CR2_PECBYTE                     ((uint32_t)0x04000000)        /*!< Packet error checking byte                              */

/*******************  Bit definition for I2C_OAR1 register  ******************/
#define  I2C_OAR1_OA1                        ((uint32_t)0x000003FF)        /*!< Interface own address 1   */
#define  I2C_OAR1_OA1MODE                    ((uint32_t)0x00000400)        /*!< Own address 1 10-bit mode */
#define  I2C_OAR1_OA1EN                      ((uint32_t)0x00008000)        /*!< Own address 1 enable      */

/*******************  Bit definition for I2C_OAR2 register  ******************/
#define  I2C_OAR2_OA2                        ((uint32_t)0x000000FE)        /*!< Interface own address 2                        */
#define  I2C_OAR2_OA2MSK                     ((uint32_t)0x00000700)        /*!< Own address 2 masks                            */
#define  I2C_OAR2_OA2NOMASK                  ((uint32_t)0x00000000)        /*!< No mask                                        */
#define  I2C_OAR2_OA2MASK01                  ((uint32_t)0x00000100)        /*!< OA2[1] is masked, Only OA2[7:2] are compared   */
#define  I2C_OAR2_OA2MASK02                  ((uint32_t)0x00000200)        /*!< OA2[2:1] is masked, Only OA2[7:3] are compared */
#define  I2C_OAR2_OA2MASK03                  ((uint32_t)0x00000300)        /*!< OA2[3:1] is masked, Only OA2[7:4] are compared */
#define  I2C_OAR2_OA2MASK04                  ((uint32_t)0x00000400)        /*!< OA2[4:1] is masked, Only OA2[7:5] are compared */
#define  I2C_OAR2_OA2MASK05                  ((uint32_t)0x00000500)        /*!< OA2[5:1] is masked, Only OA2[7:6] are compared */
#define  I2C_OAR2_OA2MASK06                  ((uint32_t)0x00000600)        /*!< OA2[6:1] is masked, Only OA2[7] are compared   */
#define  I2C_OAR2_OA2MASK07                  ((uint32_t)0x00000700)        /*!< OA2[7:1] is masked, No comparison is done      */
#define  I2C_OAR2_OA2EN                      ((uint32_t)0x00008000)        /*!< Own address 2 enable                           */

/*******************  Bit definition for I2C_TIMINGR register *******************/
#define  I2C_TIMINGR_SCLL                    ((uint32_t)0x000000FF)        /*!< SCL low period (master mode)  */
#define  I2C_TIMINGR_SCLH                    ((uint32_t)0x0000FF00)        /*!< SCL high period (master mode) */
#define  I2C_TIMINGR_SDADEL                  ((uint32_t)0x000F0000)        /*!< Data hold time                */
#define  I2C_TIMINGR_SCLDEL                  ((uint32_t)0x00F00000)        /*!< Data setup time               */
#define  I2C_TIMINGR_PRESC                   ((uint32_t)0xF0000000)        /*!< Timings prescaler             */

/******************* Bit definition for I2C_TIMEOUTR register *******************/
#define  I2C_TIMEOUTR_TIMEOUTA               ((uint32_t)0x00000FFF)        /*!< Bus timeout A                 */
#define  I2C_TIMEOUTR_TIDLE                  ((uint32_t)0x00001000)        /*!< Idle clock timeout detection  */
#define  I2C_TIMEOUTR_TIMOUTEN               ((uint32_t)0x00008000)        /*!< Clock timeout enable          */
#define  I2C_TIMEOUTR_TIMEOUTB               ((uint32_t)0x0FFF0000)        /*!< Bus timeout B                 */
#define  I2C_TIMEOUTR_TEXTEN                 ((uint32_t)0x80000000)        /*!< Extended clock timeout enable */

/******************  Bit definition for I2C_ISR register  *********************/
#define  I2C_ISR_TXE                         ((uint32_t)0x00000001)        /*!< Transmit data register empty    */
#define  I2C_ISR_TXIS                        ((uint32_t)0x00000002)        /*!< Transmit interrupt status       */
#define  I2C_ISR_RXNE                        ((uint32_t)0x00000004)        /*!< Receive data register not empty */
#define  I2C_ISR_ADDR                        ((uint32_t)0x00000008)        /*!< Address matched (slave mode)    */
#define  I2C_ISR_NACKF                       ((uint32_t)0x00000010)        /*!< NACK received flag              */
#define  I2C_ISR_STOPF                       ((uint32_t)0x00000020)        /*!< STOP detection flag             */
#define  I2C_ISR_TC                          ((uint32_t)0x00000040)        /*!< Transfer complete (master mode) */
#define  I2C_ISR_TCR                         ((uint32_t)0x00000080)        /*!< Transfer complete reload        */
#define  I2C_ISR_BERR                        ((uint32_t)0x00000100)        /*!< Bus error                       */
#define  I2C_ISR_ARLO                        ((uint32_t)0x00000200)        /*!< Arbitration lost                */
#define  I2C_ISR_OVR                         ((uint32_t)0x00000400)        /*!< Overrun/Underrun                */
#define  I2C_ISR_PECERR                      ((uint32_t)0x00000800)        /*!< PEC error in reception          */
#define  I2C_ISR_TIMEOUT                     ((uint32_t)0x00001000)        /*!< Timeout or Tlow detection flag  */
#define  I2C_ISR_ALERT                       ((uint32_t)0x00002000)        /*!< SMBus alert                     */
#define  I2C_ISR_BUSY                        ((uint32_t)0x00008000)        /*!< Bus busy                        */
#define  I2C_ISR_DIR                         ((uint32_t)0x00010000)        /*!< Transfer direction (slave mode) */
#define  I2C_ISR_ADDCODE                     ((uint32_t)0x00FE0000)        /*!< Address match code (slave mode) */

/******************  Bit definition for I2C_ICR register  *********************/
#define  I2C_ICR_ADDRCF                      ((uint32_t)0x00000008)        /*!< Address matched clear flag  */
#define  I2C_ICR_NACKCF                      ((uint32_t)0x00000010)        /*!< NACK clear flag             */
#define  I2C_ICR_STOPCF                      ((uint32_t)0x00000020)        /*!< STOP detection clear flag   */
#define  I2C_ICR_BERRCF                      ((uint32_t)0x00000100)        /*!< Bus error clear flag        */
#define  I2C_ICR_ARLOCF                      ((uint32_t)0x00000200)        /*!< Arbitration lost clear flag */
#define  I2C_ICR_OVRCF                       ((uint32_t)0x00000400)        /*!< Overrun/Underrun clear flag */
#define  I2C_ICR_PECCF                       ((uint32_t)0x00000800)        /*!< PAC error clear flag        */
#define  I2C_ICR_TIMOUTCF                    ((uint32_t)0x00001000)        /*!< Timeout clear flag          */
#define  I2C_ICR_ALERTCF                     ((uint32_t)0x00002000)        /*!< Alert clear flag            */

/******************  Bit definition for I2C_PECR register  *********************/
#define  I2C_PECR_PEC                        ((uint32_t)0x000000FF)        /*!< PEC register */

/******************  Bit definition for I2C_RXDR register  *********************/
#define  I2C_RXDR_RXDATA                     ((uint32_t)0x000000FF)        /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *********************/
#define  I2C_TXDR_TXDATA                     ((uint32_t)0x000000FF)        /*!< 8-bit transmit data */

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_KR_KEY                         ((uint32_t)0x0000FFFF)        /*!<Key value (write only, read 0000h)  */

/*******************  Bit definition for IWDG_PR register  ********************/
#define  IWDG_PR_PR                          ((uint32_t)0x00000007)        /*!<PR[2:0] (Prescaler divider)         */
#define  IWDG_PR_PR_0                        ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  IWDG_PR_PR_1                        ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  IWDG_PR_PR_2                        ((uint32_t)0x00000004)        /*!<Bit 2 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define  IWDG_RLR_RL                         ((uint32_t)0x00000FFF)        /*!<Watchdog counter reload value        */

/*******************  Bit definition for IWDG_SR register  ********************/
#define  IWDG_SR_PVU                         ((uint32_t)0x00000001)        /*!< Watchdog prescaler value update */
#define  IWDG_SR_RVU                         ((uint32_t)0x00000002)        /*!< Watchdog counter reload value update */
#define  IWDG_SR_WVU                         ((uint32_t)0x00000004)        /*!< Watchdog counter window value update */

/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_WINR_WIN                       ((uint32_t)0x00000FFF)        /*!< Watchdog counter window value */

/******************************************************************************/
/*                                                                            */
/*                                     Firewall                               */
/*                                                                            */
/******************************************************************************/

/*******Bit definition for CSSA;CSL;NVDSSA;NVDSL;VDSSA;VDSL;LSSA;LSL register */
#define  FW_CSSA_ADD                   ((uint32_t)0x00FFFF00)        /*!< Code Segment Start Address */
#define  FW_CSL_LENG                   ((uint32_t)0x003FFF00)        /*!< Code Segment Length        */
#define  FW_NVDSSA_ADD                 ((uint32_t)0x00FFFF00)        /*!< Non Volatile Dat Segment Start Address */
#define  FW_NVDSL_LENG                 ((uint32_t)0x003FFF00)        /*!< Non Volatile Data Segment Length */
#define  FW_VDSSA_ADD                  ((uint32_t)0x0001FFC0)        /*!< Volatile Data Segment Start Address */
#define  FW_VDSL_LENG                  ((uint32_t)0x0001FFC0)        /*!< Volatile Data Segment Length */
#define  FW_LSSA_ADD                   ((uint32_t)0x0007FF80)        /*!< Library Segment Start Address*/
#define  FW_LSL_LENG                   ((uint32_t)0x0007FF80)        /*!< Library Segment Length*/

/**************************Bit definition for CR register *********************/
#define  FW_CR_FPA                     ((uint32_t)0x00000001)         /*!< Firewall Pre Arm*/
#define  FW_CR_VDS                     ((uint32_t)0x00000002)         /*!< Volatile Data Sharing*/
#define  FW_CR_VDE                     ((uint32_t)0x00000004)         /*!< Volatile Data Execution*/

/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for PWR_CR1 register  ********************/

#define  PWR_CR1_LPR                         ((uint32_t)0x00004000)     /*!< Regulator low-power mode */
#define  PWR_CR1_VOS                         ((uint32_t)0x00000600)     /*!< VOS[1:0] bits (Regulator voltage scaling output selection) */
#define  PWR_CR1_VOS_0                       ((uint32_t)0x00000200)     /*!< Bit 0 */
#define  PWR_CR1_VOS_1                       ((uint32_t)0x00000400)     /*!< Bit 1 */
#define  PWR_CR1_DBP                         ((uint32_t)0x00000100)     /*!< Disable Back-up domain Protection */
#define  PWR_CR1_LPMS                        ((uint32_t)0x00000007)     /*!< Low-power mode selection field */
#define  PWR_CR1_LPMS_STOP1MR                ((uint32_t)0x00000000)     /*!< Stop 1 mode with Main Regulator */
#define  PWR_CR1_LPMS_STOP1LPR               ((uint32_t)0x00000001)     /*!< Stop 1 mode with Low-Power Regulator */
#define  PWR_CR1_LPMS_STOP2                  ((uint32_t)0x00000002)     /*!< Stop 2 mode */
#define  PWR_CR1_LPMS_STANDBY                ((uint32_t)0x00000003)     /*!< Stand-by mode */
#define  PWR_CR1_LPMS_SHUTDOWN               ((uint32_t)0x00000004)     /*!< Shut-down mode */


/********************  Bit definition for PWR_CR2 register  ********************/
#define  PWR_CR2_USV                         ((uint32_t)0x00000400)     /*!< VDD USB Supply Valid */
#define  PWR_CR2_IOSV                        ((uint32_t)0x00000200)     /*!< VDD IO2 independent I/Os Supply Valid */
/*!< PVME  Peripheral Voltage Monitor Enable */
#define  PWR_CR2_PVME                        ((uint32_t)0x000000F0)     /*!< PVM bits field */
#define  PWR_CR2_PVME4                       ((uint32_t)0x00000080)     /*!< PVM 4 Enable */
#define  PWR_CR2_PVME3                       ((uint32_t)0x00000040)     /*!< PVM 3 Enable */
#define  PWR_CR2_PVME2                       ((uint32_t)0x00000020)     /*!< PVM 2 Enable */
#define  PWR_CR2_PVME1                       ((uint32_t)0x00000010)     /*!< PVM 1 Enable */
/*!< PVD level configuration */
#define  PWR_CR2_PLS                         ((uint32_t)0x0000000E)     /*!< PVD level selection */
#define  PWR_CR2_PLS_LEV0                    ((uint32_t)0x00000000)     /*!< PVD level 0 */
#define  PWR_CR2_PLS_LEV1                    ((uint32_t)0x00000002)     /*!< PVD level 1 */
#define  PWR_CR2_PLS_LEV2                    ((uint32_t)0x00000004)     /*!< PVD level 2 */
#define  PWR_CR2_PLS_LEV3                    ((uint32_t)0x00000006)     /*!< PVD level 3 */
#define  PWR_CR2_PLS_LEV4                    ((uint32_t)0x00000008)     /*!< PVD level 4 */
#define  PWR_CR2_PLS_LEV5                    ((uint32_t)0x0000000A)     /*!< PVD level 5 */
#define  PWR_CR2_PLS_LEV6                    ((uint32_t)0x0000000C)     /*!< PVD level 6 */
#define  PWR_CR2_PLS_LEV7                    ((uint32_t)0x0000000E)     /*!< PVD level 7 */
#define  PWR_CR2_PVDE                        ((uint32_t)0x00000001)     /*!< Power Voltage Detector Enable */

/********************  Bit definition for PWR_CR3 register  ********************/
#define  PWR_CR3_EIWF                        ((uint32_t)0x00008000)      /*!< Enable Internal Wake-up line */
#define  PWR_CR3_APC                         ((uint32_t)0x00000400)      /*!< Apply pull-up and pull-down configuration */
#define  PWR_CR3_RRS                         ((uint32_t)0x00000100)      /*!< SRAM2 Retention in Stand-by mode */
#define  PWR_CR3_EWUP5                       ((uint32_t)0x00000010)      /*!< Enable Wake-Up Pin 5 */
#define  PWR_CR3_EWUP4                       ((uint32_t)0x00000008)      /*!< Enable Wake-Up Pin 4 */
#define  PWR_CR3_EWUP3                       ((uint32_t)0x00000004)      /*!< Enable Wake-Up Pin 3 */
#define  PWR_CR3_EWUP2                       ((uint32_t)0x00000002)      /*!< Enable Wake-Up Pin 2 */
#define  PWR_CR3_EWUP1                       ((uint32_t)0x00000001)      /*!< Enable Wake-Up Pin 1 */
#define  PWR_CR3_EWUP                        ((uint32_t)0x0000001F)      /*!< Enable Wake-Up Pins  */

/********************  Bit definition for PWR_CR4 register  ********************/
#define  PWR_CR4_VBRS                        ((uint32_t)0x00000200)      /*!< VBAT Battery charging Resistor Selection */
#define  PWR_CR4_VBE                         ((uint32_t)0x00000100)      /*!< VBAT Battery charging Enable  */
#define  PWR_CR4_WP5                         ((uint32_t)0x00000010)      /*!< Wake-Up Pin 5 polarity */
#define  PWR_CR4_WP4                         ((uint32_t)0x00000008)      /*!< Wake-Up Pin 4 polarity */
#define  PWR_CR4_WP3                         ((uint32_t)0x00000004)      /*!< Wake-Up Pin 3 polarity */
#define  PWR_CR4_WP2                         ((uint32_t)0x00000002)      /*!< Wake-Up Pin 2 polarity */
#define  PWR_CR4_WP1                         ((uint32_t)0x00000001)      /*!< Wake-Up Pin 1 polarity */

/********************  Bit definition for PWR_SR1 register  ********************/
#define   PWR_SR1_WUFI                       ((uint32_t)0x00008000)     /*!< Wake-Up Flag Internal */
#define   PWR_SR1_SBF                        ((uint32_t)0x00000100)     /*!< Stand-By Flag */
#define   PWR_SR1_WUF                        ((uint32_t)0x0000001F)     /*!< Wake-up Flags */
#define   PWR_SR1_WUF5                       ((uint32_t)0x00000010)     /*!< Wake-up Flag 5 */
#define   PWR_SR1_WUF4                       ((uint32_t)0x00000008)     /*!< Wake-up Flag 4 */
#define   PWR_SR1_WUF3                       ((uint32_t)0x00000004)     /*!< Wake-up Flag 3 */
#define   PWR_SR1_WUF2                       ((uint32_t)0x00000002)     /*!< Wake-up Flag 2 */
#define   PWR_SR1_WUF1                       ((uint32_t)0x00000001)     /*!< Wake-up Flag 1 */

/********************  Bit definition for PWR_SR2 register  ********************/
#define   PWR_SR2_PVMO4                      ((uint32_t)0x00008000)     /*!< Peripheral Voltage Monitoring Output 4 */
#define   PWR_SR2_PVMO3                      ((uint32_t)0x00004000)     /*!< Peripheral Voltage Monitoring Output 3 */
#define   PWR_SR2_PVMO2                      ((uint32_t)0x00002000)     /*!< Peripheral Voltage Monitoring Output 2 */
#define   PWR_SR2_PVMO1                      ((uint32_t)0x00001000)     /*!< Peripheral Voltage Monitoring Output 1 */
#define   PWR_SR2_PVDO                       ((uint32_t)0x00000800)     /*!< Power Voltage Detector Output */
#define   PWR_SR2_VOSF                       ((uint32_t)0x00000400)     /*!< Voltage Scaling Flag */
#define   PWR_SR2_REGLPF                     ((uint32_t)0x00000200)     /*!< Low-power Regulator Flag */
#define   PWR_SR2_REGLPS                     ((uint32_t)0x00000100)     /*!< Low-power Regulator Started */

/********************  Bit definition for PWR_SCR register  ********************/
#define   PWR_SCR_CSBF                       ((uint32_t)0x00000100)      /*!< Clear Stand-By Flag */
#define   PWR_SCR_CWUF                       ((uint32_t)0x0000001F)      /*!< Clear Wake-up Flags  */
#define   PWR_SCR_CWUF5                      ((uint32_t)0x00000010)      /*!< Clear Wake-up Flag 5 */
#define   PWR_SCR_CWUF4                      ((uint32_t)0x00000008)      /*!< Clear Wake-up Flag 4 */
#define   PWR_SCR_CWUF3                      ((uint32_t)0x00000004)      /*!< Clear Wake-up Flag 3 */
#define   PWR_SCR_CWUF2                      ((uint32_t)0x00000002)      /*!< Clear Wake-up Flag 2 */
#define   PWR_SCR_CWUF1                      ((uint32_t)0x00000001)      /*!< Clear Wake-up Flag 1 */

/********************  Bit definition for PWR_PUCRA register  ********************/
#define   PWR_PUCRA_PA15                     ((uint32_t)0x00008000)      /*!< Port PA15 Pull-Up set */
#define   PWR_PUCRA_PA13                     ((uint32_t)0x00002000)      /*!< Port PA13 Pull-Up set */
#define   PWR_PUCRA_PA12                     ((uint32_t)0x00001000)      /*!< Port PA12 Pull-Up set */
#define   PWR_PUCRA_PA11                     ((uint32_t)0x00000800)      /*!< Port PA11 Pull-Up set */
#define   PWR_PUCRA_PA10                     ((uint32_t)0x00000400)      /*!< Port PA10 Pull-Up set */
#define   PWR_PUCRA_PA9                      ((uint32_t)0x00000200)      /*!< Port PA9 Pull-Up set  */
#define   PWR_PUCRA_PA8                      ((uint32_t)0x00000100)      /*!< Port PA8 Pull-Up set  */
#define   PWR_PUCRA_PA7                      ((uint32_t)0x00000080)      /*!< Port PA7 Pull-Up set  */
#define   PWR_PUCRA_PA6                      ((uint32_t)0x00000040)      /*!< Port PA6 Pull-Up set  */
#define   PWR_PUCRA_PA5                      ((uint32_t)0x00000020)      /*!< Port PA5 Pull-Up set  */
#define   PWR_PUCRA_PA4                      ((uint32_t)0x00000010)      /*!< Port PA4 Pull-Up set  */
#define   PWR_PUCRA_PA3                      ((uint32_t)0x00000008)      /*!< Port PA3 Pull-Up set  */
#define   PWR_PUCRA_PA2                      ((uint32_t)0x00000004)      /*!< Port PA2 Pull-Up set  */
#define   PWR_PUCRA_PA1                      ((uint32_t)0x00000002)      /*!< Port PA1 Pull-Up set  */
#define   PWR_PUCRA_PA0                      ((uint32_t)0x00000001)      /*!< Port PA0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRA register  ********************/
#define   PWR_PDCRA_PA14                     ((uint32_t)0x00004000)      /*!< Port PA14 Pull-Down set */
#define   PWR_PDCRA_PA12                     ((uint32_t)0x00001000)      /*!< Port PA12 Pull-Down set */
#define   PWR_PDCRA_PA11                     ((uint32_t)0x00000800)      /*!< Port PA11 Pull-Down set */
#define   PWR_PDCRA_PA10                     ((uint32_t)0x00000400)      /*!< Port PA10 Pull-Down set */
#define   PWR_PDCRA_PA9                      ((uint32_t)0x00000200)      /*!< Port PA9 Pull-Down set  */
#define   PWR_PDCRA_PA8                      ((uint32_t)0x00000100)      /*!< Port PA8 Pull-Down set  */
#define   PWR_PDCRA_PA7                      ((uint32_t)0x00000080)      /*!< Port PA7 Pull-Down set  */
#define   PWR_PDCRA_PA6                      ((uint32_t)0x00000040)      /*!< Port PA6 Pull-Down set  */
#define   PWR_PDCRA_PA5                      ((uint32_t)0x00000020)      /*!< Port PA5 Pull-Down set  */
#define   PWR_PDCRA_PA4                      ((uint32_t)0x00000010)      /*!< Port PA4 Pull-Down set  */
#define   PWR_PDCRA_PA3                      ((uint32_t)0x00000008)      /*!< Port PA3 Pull-Down set  */
#define   PWR_PDCRA_PA2                      ((uint32_t)0x00000004)      /*!< Port PA2 Pull-Down set  */
#define   PWR_PDCRA_PA1                      ((uint32_t)0x00000002)      /*!< Port PA1 Pull-Down set  */
#define   PWR_PDCRA_PA0                      ((uint32_t)0x00000001)      /*!< Port PA0 Pull-Down set  */

/********************  Bit definition for PWR_PUCRB register  ********************/
#define   PWR_PUCRB_PB15                     ((uint32_t)0x00008000)      /*!< Port PB15 Pull-Up set */
#define   PWR_PUCRB_PB14                     ((uint32_t)0x00004000)      /*!< Port PB14 Pull-Up set */
#define   PWR_PUCRB_PB13                     ((uint32_t)0x00002000)      /*!< Port PB13 Pull-Up set */
#define   PWR_PUCRB_PB12                     ((uint32_t)0x00001000)      /*!< Port PB12 Pull-Up set */
#define   PWR_PUCRB_PB11                     ((uint32_t)0x00000800)      /*!< Port PB11 Pull-Up set */
#define   PWR_PUCRB_PB10                     ((uint32_t)0x00000400)      /*!< Port PB10 Pull-Up set */
#define   PWR_PUCRB_PB9                      ((uint32_t)0x00000200)      /*!< Port PB9 Pull-Up set  */
#define   PWR_PUCRB_PB8                      ((uint32_t)0x00000100)      /*!< Port PB8 Pull-Up set  */
#define   PWR_PUCRB_PB7                      ((uint32_t)0x00000080)      /*!< Port PB7 Pull-Up set  */
#define   PWR_PUCRB_PB6                      ((uint32_t)0x00000040)      /*!< Port PB6 Pull-Up set  */
#define   PWR_PUCRB_PB5                      ((uint32_t)0x00000020)      /*!< Port PB5 Pull-Up set  */
#define   PWR_PUCRB_PB4                      ((uint32_t)0x00000010)      /*!< Port PB4 Pull-Up set  */
#define   PWR_PUCRB_PB3                      ((uint32_t)0x00000008)      /*!< Port PB3 Pull-Up set  */
#define   PWR_PUCRB_PB2                      ((uint32_t)0x00000004)      /*!< Port PB2 Pull-Up set  */
#define   PWR_PUCRB_PB1                      ((uint32_t)0x00000002)      /*!< Port PB1 Pull-Up set  */
#define   PWR_PUCRB_PB0                      ((uint32_t)0x00000001)      /*!< Port PB0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRB register  ********************/
#define   PWR_PDCRB_PB15                     ((uint32_t)0x00008000)      /*!< Port PB15 Pull-Down set */
#define   PWR_PDCRB_PB14                     ((uint32_t)0x00004000)      /*!< Port PB14 Pull-Down set */
#define   PWR_PDCRB_PB13                     ((uint32_t)0x00002000)      /*!< Port PB13 Pull-Down set */
#define   PWR_PDCRB_PB12                     ((uint32_t)0x00001000)      /*!< Port PB12 Pull-Down set */
#define   PWR_PDCRB_PB11                     ((uint32_t)0x00000800)      /*!< Port PB11 Pull-Down set */
#define   PWR_PDCRB_PB10                     ((uint32_t)0x00000400)      /*!< Port PB10 Pull-Down set */
#define   PWR_PDCRB_PB9                      ((uint32_t)0x00000200)      /*!< Port PB9 Pull-Down set  */
#define   PWR_PDCRB_PB8                      ((uint32_t)0x00000100)      /*!< Port PB8 Pull-Down set  */
#define   PWR_PDCRB_PB7                      ((uint32_t)0x00000080)      /*!< Port PB7 Pull-Down set  */
#define   PWR_PDCRB_PB6                      ((uint32_t)0x00000040)      /*!< Port PB6 Pull-Down set  */
#define   PWR_PDCRB_PB5                      ((uint32_t)0x00000020)      /*!< Port PB5 Pull-Down set  */
#define   PWR_PDCRB_PB3                      ((uint32_t)0x00000008)      /*!< Port PB3 Pull-Down set  */
#define   PWR_PDCRB_PB2                      ((uint32_t)0x00000004)      /*!< Port PB2 Pull-Down set  */
#define   PWR_PDCRB_PB1                      ((uint32_t)0x00000002)      /*!< Port PB1 Pull-Down set  */
#define   PWR_PDCRB_PB0                      ((uint32_t)0x00000001)      /*!< Port PB0 Pull-Down set  */

/********************  Bit definition for PWR_PUCRC register  ********************/
#define   PWR_PUCRC_PC15                     ((uint32_t)0x00008000)      /*!< Port PC15 Pull-Up set */
#define   PWR_PUCRC_PC14                     ((uint32_t)0x00004000)      /*!< Port PC14 Pull-Up set */
#define   PWR_PUCRC_PC13                     ((uint32_t)0x00002000)      /*!< Port PC13 Pull-Up set */
#define   PWR_PUCRC_PC12                     ((uint32_t)0x00001000)      /*!< Port PC12 Pull-Up set */
#define   PWR_PUCRC_PC11                     ((uint32_t)0x00000800)      /*!< Port PC11 Pull-Up set */
#define   PWR_PUCRC_PC10                     ((uint32_t)0x00000400)      /*!< Port PC10 Pull-Up set */
#define   PWR_PUCRC_PC9                      ((uint32_t)0x00000200)      /*!< Port PC9 Pull-Up set  */
#define   PWR_PUCRC_PC8                      ((uint32_t)0x00000100)      /*!< Port PC8 Pull-Up set  */
#define   PWR_PUCRC_PC7                      ((uint32_t)0x00000080)      /*!< Port PC7 Pull-Up set  */
#define   PWR_PUCRC_PC6                      ((uint32_t)0x00000040)      /*!< Port PC6 Pull-Up set  */
#define   PWR_PUCRC_PC5                      ((uint32_t)0x00000020)      /*!< Port PC5 Pull-Up set  */
#define   PWR_PUCRC_PC4                      ((uint32_t)0x00000010)      /*!< Port PC4 Pull-Up set  */
#define   PWR_PUCRC_PC3                      ((uint32_t)0x00000008)      /*!< Port PC3 Pull-Up set  */
#define   PWR_PUCRC_PC2                      ((uint32_t)0x00000004)      /*!< Port PC2 Pull-Up set  */
#define   PWR_PUCRC_PC1                      ((uint32_t)0x00000002)      /*!< Port PC1 Pull-Up set  */
#define   PWR_PUCRC_PC0                      ((uint32_t)0x00000001)      /*!< Port PC0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRC register  ********************/
#define   PWR_PDCRC_PC15                     ((uint32_t)0x00008000)      /*!< Port PC15 Pull-Down set */
#define   PWR_PDCRC_PC14                     ((uint32_t)0x00004000)      /*!< Port PC14 Pull-Down set */
#define   PWR_PDCRC_PC13                     ((uint32_t)0x00002000)      /*!< Port PC13 Pull-Down set */
#define   PWR_PDCRC_PC12                     ((uint32_t)0x00001000)      /*!< Port PC12 Pull-Down set */
#define   PWR_PDCRC_PC11                     ((uint32_t)0x00000800)      /*!< Port PC11 Pull-Down set */
#define   PWR_PDCRC_PC10                     ((uint32_t)0x00000400)      /*!< Port PC10 Pull-Down set */
#define   PWR_PDCRC_PC9                      ((uint32_t)0x00000200)      /*!< Port PC9 Pull-Down set  */
#define   PWR_PDCRC_PC8                      ((uint32_t)0x00000100)      /*!< Port PC8 Pull-Down set  */
#define   PWR_PDCRC_PC7                      ((uint32_t)0x00000080)      /*!< Port PC7 Pull-Down set  */
#define   PWR_PDCRC_PC6                      ((uint32_t)0x00000040)      /*!< Port PC6 Pull-Down set  */
#define   PWR_PDCRC_PC5                      ((uint32_t)0x00000020)      /*!< Port PC5 Pull-Down set  */
#define   PWR_PDCRC_PC4                      ((uint32_t)0x00000010)      /*!< Port PC4 Pull-Down set  */
#define   PWR_PDCRC_PC3                      ((uint32_t)0x00000008)      /*!< Port PC3 Pull-Down set  */
#define   PWR_PDCRC_PC2                      ((uint32_t)0x00000004)      /*!< Port PC2 Pull-Down set  */
#define   PWR_PDCRC_PC1                      ((uint32_t)0x00000002)      /*!< Port PC1 Pull-Down set  */
#define   PWR_PDCRC_PC0                      ((uint32_t)0x00000001)      /*!< Port PC0 Pull-Down set  */

/********************  Bit definition for PWR_PUCRD register  ********************/
#define   PWR_PUCRD_PD15                     ((uint32_t)0x00008000)      /*!< Port PD15 Pull-Up set */
#define   PWR_PUCRD_PD14                     ((uint32_t)0x00004000)      /*!< Port PD14 Pull-Up set */
#define   PWR_PUCRD_PD13                     ((uint32_t)0x00002000)      /*!< Port PD13 Pull-Up set */
#define   PWR_PUCRD_PD12                     ((uint32_t)0x00001000)      /*!< Port PD12 Pull-Up set */
#define   PWR_PUCRD_PD11                     ((uint32_t)0x00000800)      /*!< Port PD11 Pull-Up set */
#define   PWR_PUCRD_PD10                     ((uint32_t)0x00000400)      /*!< Port PD10 Pull-Up set */
#define   PWR_PUCRD_PD9                      ((uint32_t)0x00000200)      /*!< Port PD9 Pull-Up set  */
#define   PWR_PUCRD_PD8                      ((uint32_t)0x00000100)      /*!< Port PD8 Pull-Up set  */
#define   PWR_PUCRD_PD7                      ((uint32_t)0x00000080)      /*!< Port PD7 Pull-Up set  */
#define   PWR_PUCRD_PD6                      ((uint32_t)0x00000040)      /*!< Port PD6 Pull-Up set  */
#define   PWR_PUCRD_PD5                      ((uint32_t)0x00000020)      /*!< Port PD5 Pull-Up set  */
#define   PWR_PUCRD_PD4                      ((uint32_t)0x00000010)      /*!< Port PD4 Pull-Up set  */
#define   PWR_PUCRD_PD3                      ((uint32_t)0x00000008)      /*!< Port PD3 Pull-Up set  */
#define   PWR_PUCRD_PD2                      ((uint32_t)0x00000004)      /*!< Port PD2 Pull-Up set  */
#define   PWR_PUCRD_PD1                      ((uint32_t)0x00000002)      /*!< Port PD1 Pull-Up set  */
#define   PWR_PUCRD_PD0                      ((uint32_t)0x00000001)      /*!< Port PD0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRD register  ********************/
#define   PWR_PDCRD_PD15                     ((uint32_t)0x00008000)      /*!< Port PD15 Pull-Down set */
#define   PWR_PDCRD_PD14                     ((uint32_t)0x00004000)      /*!< Port PD14 Pull-Down set */
#define   PWR_PDCRD_PD13                     ((uint32_t)0x00002000)      /*!< Port PD13 Pull-Down set */
#define   PWR_PDCRD_PD12                     ((uint32_t)0x00001000)      /*!< Port PD12 Pull-Down set */
#define   PWR_PDCRD_PD11                     ((uint32_t)0x00000800)      /*!< Port PD11 Pull-Down set */
#define   PWR_PDCRD_PD10                     ((uint32_t)0x00000400)      /*!< Port PD10 Pull-Down set */
#define   PWR_PDCRD_PD9                      ((uint32_t)0x00000200)      /*!< Port PD9 Pull-Down set  */
#define   PWR_PDCRD_PD8                      ((uint32_t)0x00000100)      /*!< Port PD8 Pull-Down set  */
#define   PWR_PDCRD_PD7                      ((uint32_t)0x00000080)      /*!< Port PD7 Pull-Down set  */
#define   PWR_PDCRD_PD6                      ((uint32_t)0x00000040)      /*!< Port PD6 Pull-Down set  */
#define   PWR_PDCRD_PD5                      ((uint32_t)0x00000020)      /*!< Port PD5 Pull-Down set  */
#define   PWR_PDCRD_PD4                      ((uint32_t)0x00000010)      /*!< Port PD4 Pull-Down set  */
#define   PWR_PDCRD_PD3                      ((uint32_t)0x00000008)      /*!< Port PD3 Pull-Down set  */
#define   PWR_PDCRD_PD2                      ((uint32_t)0x00000004)      /*!< Port PD2 Pull-Down set  */
#define   PWR_PDCRD_PD1                      ((uint32_t)0x00000002)      /*!< Port PD1 Pull-Down set  */
#define   PWR_PDCRD_PD0                      ((uint32_t)0x00000001)      /*!< Port PD0 Pull-Down set  */

/********************  Bit definition for PWR_PUCRE register  ********************/
#define   PWR_PUCRE_PE15                     ((uint32_t)0x00008000)      /*!< Port PE15 Pull-Up set */
#define   PWR_PUCRE_PE14                     ((uint32_t)0x00004000)      /*!< Port PE14 Pull-Up set */
#define   PWR_PUCRE_PE13                     ((uint32_t)0x00002000)      /*!< Port PE13 Pull-Up set */
#define   PWR_PUCRE_PE12                     ((uint32_t)0x00001000)      /*!< Port PE12 Pull-Up set */
#define   PWR_PUCRE_PE11                     ((uint32_t)0x00000800)      /*!< Port PE11 Pull-Up set */
#define   PWR_PUCRE_PE10                     ((uint32_t)0x00000400)      /*!< Port PE10 Pull-Up set */
#define   PWR_PUCRE_PE9                      ((uint32_t)0x00000200)      /*!< Port PE9 Pull-Up set  */
#define   PWR_PUCRE_PE8                      ((uint32_t)0x00000100)      /*!< Port PE8 Pull-Up set  */
#define   PWR_PUCRE_PE7                      ((uint32_t)0x00000080)      /*!< Port PE7 Pull-Up set  */
#define   PWR_PUCRE_PE6                      ((uint32_t)0x00000040)      /*!< Port PE6 Pull-Up set  */
#define   PWR_PUCRE_PE5                      ((uint32_t)0x00000020)      /*!< Port PE5 Pull-Up set  */
#define   PWR_PUCRE_PE4                      ((uint32_t)0x00000010)      /*!< Port PE4 Pull-Up set  */
#define   PWR_PUCRE_PE3                      ((uint32_t)0x00000008)      /*!< Port PE3 Pull-Up set  */
#define   PWR_PUCRE_PE2                      ((uint32_t)0x00000004)      /*!< Port PE2 Pull-Up set  */
#define   PWR_PUCRE_PE1                      ((uint32_t)0x00000002)      /*!< Port PE1 Pull-Up set  */
#define   PWR_PUCRE_PE0                      ((uint32_t)0x00000001)      /*!< Port PE0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRE register  ********************/
#define   PWR_PDCRE_PE15                     ((uint32_t)0x00008000)      /*!< Port PE15 Pull-Down set */
#define   PWR_PDCRE_PE14                     ((uint32_t)0x00004000)      /*!< Port PE14 Pull-Down set */
#define   PWR_PDCRE_PE13                     ((uint32_t)0x00002000)      /*!< Port PE13 Pull-Down set */
#define   PWR_PDCRE_PE12                     ((uint32_t)0x00001000)      /*!< Port PE12 Pull-Down set */
#define   PWR_PDCRE_PE11                     ((uint32_t)0x00000800)      /*!< Port PE11 Pull-Down set */
#define   PWR_PDCRE_PE10                     ((uint32_t)0x00000400)      /*!< Port PE10 Pull-Down set */
#define   PWR_PDCRE_PE9                      ((uint32_t)0x00000200)      /*!< Port PE9 Pull-Down set  */
#define   PWR_PDCRE_PE8                      ((uint32_t)0x00000100)      /*!< Port PE8 Pull-Down set  */
#define   PWR_PDCRE_PE7                      ((uint32_t)0x00000080)      /*!< Port PE7 Pull-Down set  */
#define   PWR_PDCRE_PE6                      ((uint32_t)0x00000040)      /*!< Port PE6 Pull-Down set  */
#define   PWR_PDCRE_PE5                      ((uint32_t)0x00000020)      /*!< Port PE5 Pull-Down set  */
#define   PWR_PDCRE_PE4                      ((uint32_t)0x00000010)      /*!< Port PE4 Pull-Down set  */
#define   PWR_PDCRE_PE3                      ((uint32_t)0x00000008)      /*!< Port PE3 Pull-Down set  */
#define   PWR_PDCRE_PE2                      ((uint32_t)0x00000004)      /*!< Port PE2 Pull-Down set  */
#define   PWR_PDCRE_PE1                      ((uint32_t)0x00000002)      /*!< Port PE1 Pull-Down set  */
#define   PWR_PDCRE_PE0                      ((uint32_t)0x00000001)      /*!< Port PE0 Pull-Down set  */

/********************  Bit definition for PWR_PUCRF register  ********************/
#define   PWR_PUCRF_PF15                     ((uint32_t)0x00008000)      /*!< Port PF15 Pull-Up set */
#define   PWR_PUCRF_PF14                     ((uint32_t)0x00004000)      /*!< Port PF14 Pull-Up set */
#define   PWR_PUCRF_PF13                     ((uint32_t)0x00002000)      /*!< Port PF13 Pull-Up set */
#define   PWR_PUCRF_PF12                     ((uint32_t)0x00001000)      /*!< Port PF12 Pull-Up set */
#define   PWR_PUCRF_PF11                     ((uint32_t)0x00000800)      /*!< Port PF11 Pull-Up set */
#define   PWR_PUCRF_PF10                     ((uint32_t)0x00000400)      /*!< Port PF10 Pull-Up set */
#define   PWR_PUCRF_PF9                      ((uint32_t)0x00000200)      /*!< Port PF9 Pull-Up set  */
#define   PWR_PUCRF_PF8                      ((uint32_t)0x00000100)      /*!< Port PF8 Pull-Up set  */
#define   PWR_PUCRF_PF7                      ((uint32_t)0x00000080)      /*!< Port PF7 Pull-Up set  */
#define   PWR_PUCRF_PF6                      ((uint32_t)0x00000040)      /*!< Port PF6 Pull-Up set  */
#define   PWR_PUCRF_PF5                      ((uint32_t)0x00000020)      /*!< Port PF5 Pull-Up set  */
#define   PWR_PUCRF_PF4                      ((uint32_t)0x00000010)      /*!< Port PF4 Pull-Up set  */
#define   PWR_PUCRF_PF3                      ((uint32_t)0x00000008)      /*!< Port PF3 Pull-Up set  */
#define   PWR_PUCRF_PF2                      ((uint32_t)0x00000004)      /*!< Port PF2 Pull-Up set  */
#define   PWR_PUCRF_PF1                      ((uint32_t)0x00000002)      /*!< Port PF1 Pull-Up set  */
#define   PWR_PUCRF_PF0                      ((uint32_t)0x00000001)      /*!< Port PF0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRF register  ********************/
#define   PWR_PDCRF_PF15                     ((uint32_t)0x00008000)      /*!< Port PF15 Pull-Down set */
#define   PWR_PDCRF_PF14                     ((uint32_t)0x00004000)      /*!< Port PF14 Pull-Down set */
#define   PWR_PDCRF_PF13                     ((uint32_t)0x00002000)      /*!< Port PF13 Pull-Down set */
#define   PWR_PDCRF_PF12                     ((uint32_t)0x00001000)      /*!< Port PF12 Pull-Down set */
#define   PWR_PDCRF_PF11                     ((uint32_t)0x00000800)      /*!< Port PF11 Pull-Down set */
#define   PWR_PDCRF_PF10                     ((uint32_t)0x00000400)      /*!< Port PF10 Pull-Down set */
#define   PWR_PDCRF_PF9                      ((uint32_t)0x00000200)      /*!< Port PF9 Pull-Down set  */
#define   PWR_PDCRF_PF8                      ((uint32_t)0x00000100)      /*!< Port PF8 Pull-Down set  */
#define   PWR_PDCRF_PF7                      ((uint32_t)0x00000080)      /*!< Port PF7 Pull-Down set  */
#define   PWR_PDCRF_PF6                      ((uint32_t)0x00000040)      /*!< Port PF6 Pull-Down set  */
#define   PWR_PDCRF_PF5                      ((uint32_t)0x00000020)      /*!< Port PF5 Pull-Down set  */
#define   PWR_PDCRF_PF4                      ((uint32_t)0x00000010)      /*!< Port PF4 Pull-Down set  */
#define   PWR_PDCRF_PF3                      ((uint32_t)0x00000008)      /*!< Port PF3 Pull-Down set  */
#define   PWR_PDCRF_PF2                      ((uint32_t)0x00000004)      /*!< Port PF2 Pull-Down set  */
#define   PWR_PDCRF_PF1                      ((uint32_t)0x00000002)      /*!< Port PF1 Pull-Down set  */
#define   PWR_PDCRF_PF0                      ((uint32_t)0x00000001)      /*!< Port PF0 Pull-Down set  */

/********************  Bit definition for PWR_PUCRG register  ********************/
#define   PWR_PUCRG_PG15                     ((uint32_t)0x00008000)      /*!< Port PG15 Pull-Up set */
#define   PWR_PUCRG_PG14                     ((uint32_t)0x00004000)      /*!< Port PG14 Pull-Up set */
#define   PWR_PUCRG_PG13                     ((uint32_t)0x00002000)      /*!< Port PG13 Pull-Up set */
#define   PWR_PUCRG_PG12                     ((uint32_t)0x00001000)      /*!< Port PG12 Pull-Up set */
#define   PWR_PUCRG_PG11                     ((uint32_t)0x00000800)      /*!< Port PG11 Pull-Up set */
#define   PWR_PUCRG_PG10                     ((uint32_t)0x00000400)      /*!< Port PG10 Pull-Up set */
#define   PWR_PUCRG_PG9                      ((uint32_t)0x00000200)      /*!< Port PG9 Pull-Up set  */
#define   PWR_PUCRG_PG8                      ((uint32_t)0x00000100)      /*!< Port PG8 Pull-Up set  */
#define   PWR_PUCRG_PG7                      ((uint32_t)0x00000080)      /*!< Port PG7 Pull-Up set  */
#define   PWR_PUCRG_PG6                      ((uint32_t)0x00000040)      /*!< Port PG6 Pull-Up set  */
#define   PWR_PUCRG_PG5                      ((uint32_t)0x00000020)      /*!< Port PG5 Pull-Up set  */
#define   PWR_PUCRG_PG4                      ((uint32_t)0x00000010)      /*!< Port PG4 Pull-Up set  */
#define   PWR_PUCRG_PG3                      ((uint32_t)0x00000008)      /*!< Port PG3 Pull-Up set  */
#define   PWR_PUCRG_PG2                      ((uint32_t)0x00000004)      /*!< Port PG2 Pull-Up set  */
#define   PWR_PUCRG_PG1                      ((uint32_t)0x00000002)      /*!< Port PG1 Pull-Up set  */
#define   PWR_PUCRG_PG0                      ((uint32_t)0x00000001)      /*!< Port PG0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRG register  ********************/
#define   PWR_PDCRG_PG15                     ((uint32_t)0x00008000)      /*!< Port PG15 Pull-Down set */
#define   PWR_PDCRG_PG14                     ((uint32_t)0x00004000)      /*!< Port PG14 Pull-Down set */
#define   PWR_PDCRG_PG13                     ((uint32_t)0x00002000)      /*!< Port PG13 Pull-Down set */
#define   PWR_PDCRG_PG12                     ((uint32_t)0x00001000)      /*!< Port PG12 Pull-Down set */
#define   PWR_PDCRG_PG11                     ((uint32_t)0x00000800)      /*!< Port PG11 Pull-Down set */
#define   PWR_PDCRG_PG10                     ((uint32_t)0x00000400)      /*!< Port PG10 Pull-Down set */
#define   PWR_PDCRG_PG9                      ((uint32_t)0x00000200)      /*!< Port PG9 Pull-Down set  */
#define   PWR_PDCRG_PG8                      ((uint32_t)0x00000100)      /*!< Port PG8 Pull-Down set  */
#define   PWR_PDCRG_PG7                      ((uint32_t)0x00000080)      /*!< Port PG7 Pull-Down set  */
#define   PWR_PDCRG_PG6                      ((uint32_t)0x00000040)      /*!< Port PG6 Pull-Down set  */
#define   PWR_PDCRG_PG5                      ((uint32_t)0x00000020)      /*!< Port PG5 Pull-Down set  */
#define   PWR_PDCRG_PG4                      ((uint32_t)0x00000010)      /*!< Port PG4 Pull-Down set  */
#define   PWR_PDCRG_PG3                      ((uint32_t)0x00000008)      /*!< Port PG3 Pull-Down set  */
#define   PWR_PDCRG_PG2                      ((uint32_t)0x00000004)      /*!< Port PG2 Pull-Down set  */
#define   PWR_PDCRG_PG1                      ((uint32_t)0x00000002)      /*!< Port PG1 Pull-Down set  */
#define   PWR_PDCRG_PG0                      ((uint32_t)0x00000001)      /*!< Port PG0 Pull-Down set  */

/********************  Bit definition for PWR_PUCRH register  ********************/
#define   PWR_PUCRH_PH1                      ((uint32_t)0x00000002)      /*!< Port PH1 Pull-Up set  */
#define   PWR_PUCRH_PH0                      ((uint32_t)0x00000001)      /*!< Port PH0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRH register  ********************/
#define   PWR_PDCRH_PH1                      ((uint32_t)0x00000002)      /*!< Port PH1 Pull-Down set  */
#define   PWR_PDCRH_PH0                      ((uint32_t)0x00000001)      /*!< Port PH0 Pull-Down set  */


/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_MSION                        ((uint32_t)0x00000001)      /*!< Internal Multi Speed clock enable */
#define  RCC_CR_MSIRDY                       ((uint32_t)0x00000002)      /*!< Internal Multi Speed clock ready flag */
#define  RCC_CR_MSIPLLEN                     ((uint32_t)0x00000004)      /*!< Internal Multi Speed PLL enable */
#define  RCC_CR_MSIRGSEL                     ((uint32_t)0x00000008)      /*!< Internal Multi Speed range selection */

/*!< MSIRANGE configuration : 12 frequency ranges available */
#define  RCC_CR_MSIRANGE                     ((uint32_t)0x000000F0)       /*!< Internal Multi Speed clock Range */
#define  RCC_CR_MSIRANGE_0                   ((uint32_t)0x00000000)       /*!< Internal Multi Speed clock Range 100 KHz */
#define  RCC_CR_MSIRANGE_1                   ((uint32_t)0x00000010)       /*!< Internal Multi Speed clock Range 200 KHz */
#define  RCC_CR_MSIRANGE_2                   ((uint32_t)0x00000020)       /*!< Internal Multi Speed clock Range 400 KHz */
#define  RCC_CR_MSIRANGE_3                   ((uint32_t)0x00000030)       /*!< Internal Multi Speed clock Range 800 KHz */
#define  RCC_CR_MSIRANGE_4                   ((uint32_t)0x00000040)       /*!< Internal Multi Speed clock Range 1 MHz */
#define  RCC_CR_MSIRANGE_5                   ((uint32_t)0x00000050)       /*!< Internal Multi Speed clock Range 2 MHz */
#define  RCC_CR_MSIRANGE_6                   ((uint32_t)0x00000060)       /*!< Internal Multi Speed clock Range 4  MHz */
#define  RCC_CR_MSIRANGE_7                   ((uint32_t)0x00000070)       /*!< Internal Multi Speed clock Range 8 KHz */
#define  RCC_CR_MSIRANGE_8                   ((uint32_t)0x00000080)       /*!< Internal Multi Speed clock Range 16 MHz */
#define  RCC_CR_MSIRANGE_9                   ((uint32_t)0x00000090)       /*!< Internal Multi Speed clock Range 24 MHz */
#define  RCC_CR_MSIRANGE_10                  ((uint32_t)0x000000A0)       /*!< Internal Multi Speed clock Range 32 MHz */
#define  RCC_CR_MSIRANGE_11                  ((uint32_t)0x000000B0)       /*!< Internal Multi Speed clock Range 48  MHz */

#define  RCC_CR_HSION                        ((uint32_t)0x00000100)       /*!< Internal High Speed clock enable */
#define  RCC_CR_HSIKERON                     ((uint32_t)0x00000200)       /*!< Internal High Speed clock enable for some IPs Kernel */
#define  RCC_CR_HSIRDY                       ((uint32_t)0x00000400)       /*!< Internal High Speed clock ready flag */
#define  RCC_CR_HSIASFS                      ((uint32_t)0x00000800)       /*!< HSI Automatic Start from Stop */

#define  RCC_CR_HSEON                        ((uint32_t)0x00010000)       /*!< External High Speed clock enable */
#define  RCC_CR_HSERDY                       ((uint32_t)0x00020000)       /*!< External High Speed clock ready */
#define  RCC_CR_HSEBYP                       ((uint32_t)0x00040000)       /*!< External High Speed clock Bypass */
#define  RCC_CR_CSSON                        ((uint32_t)0x00080000)       /*!< HSE Clock Security System enable */

#define  RCC_CR_PLLON                        ((uint32_t)0x01000000)       /*!< System PLL clock enable */
#define  RCC_CR_PLLRDY                       ((uint32_t)0x02000000)       /*!< System PLL clock ready */
#define  RCC_CR_PLLSAI1ON                    ((uint32_t)0x04000000)       /*!< SAI1 PLL enable */
#define  RCC_CR_PLLSAI1RDY                   ((uint32_t)0x08000000)       /*!< SAI1 PLL ready */
#define  RCC_CR_PLLSAI2ON                    ((uint32_t)0x10000000)       /*!< SAI2 PLL enable */
#define  RCC_CR_PLLSAI2RDY                   ((uint32_t)0x20000000)       /*!< SAI2 PLL ready */

/********************  Bit definition for RCC_ICSCR register  ***************/
/*!< MSICAL configuration */
#define  RCC_ICSCR_MSICAL                    ((uint32_t)0x000000FF)       /*!< MSICAL[7:0] bits */
#define  RCC_ICSCR_MSICAL_0                  ((uint32_t)0x00000001)       /*!<Bit 0 */
#define  RCC_ICSCR_MSICAL_1                  ((uint32_t)0x00000002)       /*!<Bit 1 */
#define  RCC_ICSCR_MSICAL_2                  ((uint32_t)0x00000004)       /*!<Bit 2 */
#define  RCC_ICSCR_MSICAL_3                  ((uint32_t)0x00000008)       /*!<Bit 3 */
#define  RCC_ICSCR_MSICAL_4                  ((uint32_t)0x00000010)       /*!<Bit 4 */
#define  RCC_ICSCR_MSICAL_5                  ((uint32_t)0x00000020)       /*!<Bit 5 */
#define  RCC_ICSCR_MSICAL_6                  ((uint32_t)0x00000040)       /*!<Bit 6 */
#define  RCC_ICSCR_MSICAL_7                  ((uint32_t)0x00000080)       /*!<Bit 7 */

/*!< MSITRIM configuration */
#define  RCC_ICSCR_MSITRIM                   ((uint32_t)0x0000FF00)       /*!< MSITRIM[7:0] bits */
#define  RCC_ICSCR_MSITRIM_0                 ((uint32_t)0x00000100)       /*!<Bit 0 */
#define  RCC_ICSCR_MSITRIM_1                 ((uint32_t)0x00000200)       /*!<Bit 1 */
#define  RCC_ICSCR_MSITRIM_2                 ((uint32_t)0x00000400)       /*!<Bit 2 */
#define  RCC_ICSCR_MSITRIM_3                 ((uint32_t)0x00000800)       /*!<Bit 3 */
#define  RCC_ICSCR_MSITRIM_4                 ((uint32_t)0x00001000)       /*!<Bit 4 */
#define  RCC_ICSCR_MSITRIM_5                 ((uint32_t)0x00002000)       /*!<Bit 5 */
#define  RCC_ICSCR_MSITRIM_6                 ((uint32_t)0x00004000)       /*!<Bit 6 */
#define  RCC_ICSCR_MSITRIM_7                 ((uint32_t)0x00008000)       /*!<Bit 7 */

/*!< HSICAL configuration */
#define  RCC_ICSCR_HSICAL                    ((uint32_t)0x00FF0000)       /*!< HSICAL[7:0] bits */
#define  RCC_ICSCR_HSICAL_0                  ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  RCC_ICSCR_HSICAL_1                  ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  RCC_ICSCR_HSICAL_2                  ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  RCC_ICSCR_HSICAL_3                  ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  RCC_ICSCR_HSICAL_4                  ((uint32_t)0x00100000)        /*!<Bit 4 */
#define  RCC_ICSCR_HSICAL_5                  ((uint32_t)0x00200000)        /*!<Bit 5 */
#define  RCC_ICSCR_HSICAL_6                  ((uint32_t)0x00400000)        /*!<Bit 6 */
#define  RCC_ICSCR_HSICAL_7                  ((uint32_t)0x00800000)        /*!<Bit 7 */

/*!< HSITRIM configuration */
#define  RCC_ICSCR_HSITRIM                   ((uint32_t)0x1F000000)       /*!< HSITRIM[7:0] bits */
#define  RCC_ICSCR_HSITRIM_0                 ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  RCC_ICSCR_HSITRIM_1                 ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  RCC_ICSCR_HSITRIM_2                 ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  RCC_ICSCR_HSITRIM_3                 ((uint32_t)0x08000000)        /*!<Bit 3 */
#define  RCC_ICSCR_HSITRIM_4                 ((uint32_t)0x10000000)        /*!<Bit 4 */

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define  RCC_PLLCFGR_PLLSRC                  ((uint32_t)0x00000003)

#define  RCC_PLLCFGR_PLLSRC_MSI              ((uint32_t)0x00000001)      /*!< MSI source clock selected */
#define  RCC_PLLCFGR_PLLSRC_HSI              ((uint32_t)0x00000002)      /*!< HSI source clock selected */
#define  RCC_PLLCFGR_PLLSRC_HSE              ((uint32_t)0x00000003)      /*!< HSE source clock selected */

#define  RCC_PLLCFGR_PLLM                    ((uint32_t)0x00000070)
#define  RCC_PLLCFGR_PLLM_0                  ((uint32_t)0x00000010)
#define  RCC_PLLCFGR_PLLM_1                  ((uint32_t)0x00000020)
#define  RCC_PLLCFGR_PLLM_2                  ((uint32_t)0x00000040)

#define  RCC_PLLCFGR_PLLN                    ((uint32_t)0x00007F00)
#define  RCC_PLLCFGR_PLLN_0                  ((uint32_t)0x00000100)
#define  RCC_PLLCFGR_PLLN_1                  ((uint32_t)0x00000200)
#define  RCC_PLLCFGR_PLLN_2                  ((uint32_t)0x00000400)
#define  RCC_PLLCFGR_PLLN_3                  ((uint32_t)0x00000800)
#define  RCC_PLLCFGR_PLLN_4                  ((uint32_t)0x00001000)
#define  RCC_PLLCFGR_PLLN_5                  ((uint32_t)0x00002000)
#define  RCC_PLLCFGR_PLLN_6                  ((uint32_t)0x00004000)

#define  RCC_PLLCFGR_PLLPEN                  ((uint32_t)0x00010000)
#define  RCC_PLLCFGR_PLLP                    ((uint32_t)0x00020000)
#define  RCC_PLLCFGR_PLLQEN                  ((uint32_t)0x00100000)

#define  RCC_PLLCFGR_PLLQ                    ((uint32_t)0x00600000)
#define  RCC_PLLCFGR_PLLQ_0                  ((uint32_t)0x00200000)
#define  RCC_PLLCFGR_PLLQ_1                  ((uint32_t)0x00400000)

#define  RCC_PLLCFGR_PLLREN                  ((uint32_t)0x01000000)
#define  RCC_PLLCFGR_PLLR                    ((uint32_t)0x06000000)
#define  RCC_PLLCFGR_PLLR_0                  ((uint32_t)0x02000000)
#define  RCC_PLLCFGR_PLLR_1                  ((uint32_t)0x04000000)

/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003)      /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001)      /*!<Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002)      /*!<Bit 1 */

#define  RCC_CFGR_SW_MSI                     ((uint32_t)0x00000000)      /*!< MSI selection as system clock */
#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000001)      /*!< HSI selection as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000002)      /*!< HSE selection as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000003)      /*!< PLL selection as system clock */

#define  RCC_CFGR_SWS_MSI                    ((uint32_t)0x00000000)      /*!< MSI used as system clock */
#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000004)      /*!< HSI used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000008)      /*!< HSE used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x0000000C)      /*!< PLL used as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000C)      /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004)      /*!<Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008)      /*!<Bit 1 */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0)      /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((uint32_t)0x00000010)      /*!<Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((uint32_t)0x00000020)      /*!<Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((uint32_t)0x00000040)      /*!<Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((uint32_t)0x00000080)      /*!<Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((uint32_t)0x00000000)      /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((uint32_t)0x00000080)      /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((uint32_t)0x00000090)      /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((uint32_t)0x000000A0)      /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((uint32_t)0x000000B0)      /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((uint32_t)0x000000C0)      /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((uint32_t)0x000000D0)      /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((uint32_t)0x000000E0)      /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((uint32_t)0x000000F0)      /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00000700)      /*!< PRE1[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000100)      /*!<Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000200)      /*!<Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00000400)      /*!<Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000)      /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00000400)      /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00000500)      /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00000600)      /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00000700)      /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x00003800)      /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00000800)      /*!<Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00001000)      /*!<Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00002000)      /*!<Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000)      /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00002000)      /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x00002800)      /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x00003000)      /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x00003800)      /*!< HCLK divided by 16 */

#define  RCC_CFGR_STOPWUCK                   ((uint32_t)0x00008000)      /*!< Wake Up from stop and CSS backup clock selection */

/*!< MCOSEL configuration */
#define  RCC_CFGR_MCOSEL                     ((uint32_t)0x07000000)      /*!< MCOSEL [2:0] bits (Clock output selection) */
#define  RCC_CFGR_MCOSEL_0                   ((uint32_t)0x01000000)      /*!<Bit 0 */
#define  RCC_CFGR_MCOSEL_1                   ((uint32_t)0x02000000)      /*!<Bit 1 */
#define  RCC_CFGR_MCOSEL_2                   ((uint32_t)0x04000000)      /*!<Bit 2 */

#define  RCC_CFGR_MCO_PRE                    ((uint32_t)0x70000000)      /*!< MCO prescaler */
#define  RCC_CFGR_MCO_PRE_1                  ((uint32_t)0x00000000)      /*!< MCO is divided by 1 */
#define  RCC_CFGR_MCO_PRE_2                  ((uint32_t)0x10000000)      /*!< MCO is divided by 2 */
#define  RCC_CFGR_MCO_PRE_4                  ((uint32_t)0x20000000)      /*!< MCO is divided by 4 */
#define  RCC_CFGR_MCO_PRE_8                  ((uint32_t)0x30000000)      /*!< MCO is divided by 8 */
#define  RCC_CFGR_MCO_PRE_16                 ((uint32_t)0x40000000)      /*!< MCO is divided by 16 */

/********************  Bit definition for RCC_CIER register  ******************/
#define  RCC_CIER_LSIRDYIE                   ((uint32_t)0x00000001)
#define  RCC_CIER_LSERDYIE                   ((uint32_t)0x00000002)
#define  RCC_CIER_MSIRDYIE                   ((uint32_t)0x00000004)
#define  RCC_CIER_HSIRDYIE                   ((uint32_t)0x00000008)
#define  RCC_CIER_HSERDYIE                   ((uint32_t)0x00000010)
#define  RCC_CIER_PLLRDYIE                   ((uint32_t)0x00000020)
#define  RCC_CIER_PLLSAI1RDYIE               ((uint32_t)0x00000040)
#define  RCC_CIER_PLLSAI2RDYIE               ((uint32_t)0x00000080)
#define  RCC_CIER_LSECSSIE                   ((uint32_t)0x00000200)

/********************  Bit definition for RCC_CIFR register  ******************/
#define  RCC_CIFR_LSIRDYF                    ((uint32_t)0x00000001)
#define  RCC_CIFR_LSERDYF                    ((uint32_t)0x00000002)
#define  RCC_CIFR_MSIRDYF                    ((uint32_t)0x00000004)
#define  RCC_CIFR_HSIRDYF                    ((uint32_t)0x00000008)
#define  RCC_CIFR_HSERDYF                    ((uint32_t)0x00000010)
#define  RCC_CIFR_PLLRDYF                    ((uint32_t)0x00000020)
#define  RCC_CIFR_PLLSAI1RDYF                ((uint32_t)0x00000040)
#define  RCC_CIFR_PLLSAI2RDYF                ((uint32_t)0x00000080)
#define  RCC_CIFR_CSSF                       ((uint32_t)0x00000100)
#define  RCC_CIFR_LSECSSF                    ((uint32_t)0x00000200)

/********************  Bit definition for RCC_CICR register  ******************/
#define  RCC_CICR_LSIRDYC                    ((uint32_t)0x00000001)
#define  RCC_CICR_LSERDYC                    ((uint32_t)0x00000002)
#define  RCC_CICR_MSIRDYC                    ((uint32_t)0x00000004)
#define  RCC_CICR_HSIRDYC                    ((uint32_t)0x00000008)
#define  RCC_CICR_HSERDYC                    ((uint32_t)0x00000010)
#define  RCC_CICR_PLLRDYC                    ((uint32_t)0x00000020)
#define  RCC_CICR_PLLSAI1RDYC                ((uint32_t)0x00000040)
#define  RCC_CICR_PLLSAI2RDYC                ((uint32_t)0x00000080)
#define  RCC_CICR_CSSC                       ((uint32_t)0x00000100)
#define  RCC_CICR_LSECSSC                    ((uint32_t)0x00000200)

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define  RCC_AHB1RSTR_DMA1RST                ((uint32_t)0x00000001)
#define  RCC_AHB1RSTR_DMA2RST                ((uint32_t)0x00000002)
#define  RCC_AHB1RSTR_FLASHRST               ((uint32_t)0x00000100)
#define  RCC_AHB1RSTR_CRCRST                 ((uint32_t)0x00001000)
#define  RCC_AHB1RSTR_TSCRST                 ((uint32_t)0x00010000)

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define  RCC_AHB2RSTR_GPIOARST               ((uint32_t)0x00000001)
#define  RCC_AHB2RSTR_GPIOBRST               ((uint32_t)0x00000002)
#define  RCC_AHB2RSTR_GPIOCRST               ((uint32_t)0x00000004)
#define  RCC_AHB2RSTR_GPIODRST               ((uint32_t)0x00000008)
#define  RCC_AHB2RSTR_GPIOERST               ((uint32_t)0x00000010)
#define  RCC_AHB2RSTR_GPIOFRST               ((uint32_t)0x00000020)
#define  RCC_AHB2RSTR_GPIOGRST               ((uint32_t)0x00000040)
#define  RCC_AHB2RSTR_GPIOHRST               ((uint32_t)0x00000080)
#define  RCC_AHB2RSTR_OTGFSRST               ((uint32_t)0x00001000)
#define  RCC_AHB2RSTR_ADCRST                 ((uint32_t)0x00002000)
#define  RCC_AHB2RSTR_RNGRST                 ((uint32_t)0x00040000)

/********************  Bit definition for RCC_AHB3RSTR register  **************/
#define  RCC_AHB3RSTR_FMCRST                 ((uint32_t)0x00000001)
#define  RCC_AHB3RSTR_QSPIRST                ((uint32_t)0x00000100)

/********************  Bit definition for RCC_APB1RSTR1 register  **************/
#define  RCC_APB1RSTR1_TIM2RST               ((uint32_t)0x00000001)
#define  RCC_APB1RSTR1_TIM3RST               ((uint32_t)0x00000002)
#define  RCC_APB1RSTR1_TIM4RST               ((uint32_t)0x00000004)
#define  RCC_APB1RSTR1_TIM5RST               ((uint32_t)0x00000008)
#define  RCC_APB1RSTR1_TIM6RST               ((uint32_t)0x00000010)
#define  RCC_APB1RSTR1_TIM7RST               ((uint32_t)0x00000020)
#define  RCC_APB1RSTR1_LCDRST                ((uint32_t)0x00000200)
#define  RCC_APB1RSTR1_SPI2RST               ((uint32_t)0x00004000)
#define  RCC_APB1RSTR1_SPI3RST               ((uint32_t)0x00008000)
#define  RCC_APB1RSTR1_USART2RST             ((uint32_t)0x00020000)
#define  RCC_APB1RSTR1_USART3RST             ((uint32_t)0x00040000)
#define  RCC_APB1RSTR1_UART4RST              ((uint32_t)0x00080000)
#define  RCC_APB1RSTR1_UART5RST              ((uint32_t)0x00100000)
#define  RCC_APB1RSTR1_I2C1RST               ((uint32_t)0x00200000)
#define  RCC_APB1RSTR1_I2C2RST               ((uint32_t)0x00400000)
#define  RCC_APB1RSTR1_I2C3RST               ((uint32_t)0x00800000)
#define  RCC_APB1RSTR1_CAN1RST               ((uint32_t)0x02000000)
#define  RCC_APB1RSTR1_PWRRST                ((uint32_t)0x10000000)
#define  RCC_APB1RSTR1_DAC1RST               ((uint32_t)0x20000000)
#define  RCC_APB1RSTR1_OPAMPRST              ((uint32_t)0x40000000)
#define  RCC_APB1RSTR1_LPTIM1RST             ((uint32_t)0x80000000)

/********************  Bit definition for RCC_APB1RSTR2 register  **************/
#define  RCC_APB1RSTR2_LPUART1RST            ((uint32_t)0x00000001)
#define  RCC_APB1RSTR2_SWPMI1RST             ((uint32_t)0x00000004)
#define  RCC_APB1RSTR2_LPTIM2RST             ((uint32_t)0x00000020)

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define  RCC_APB2RSTR_SYSCFGRST              ((uint32_t)0x00000001)
#define  RCC_APB2RSTR_SDMMC1RST              ((uint32_t)0x00000400)
#define  RCC_APB2RSTR_TIM1RST                ((uint32_t)0x00000800)
#define  RCC_APB2RSTR_SPI1RST                ((uint32_t)0x00001000)
#define  RCC_APB2RSTR_TIM8RST                ((uint32_t)0x00002000)
#define  RCC_APB2RSTR_USART1RST              ((uint32_t)0x00004000)
#define  RCC_APB2RSTR_TIM15RST               ((uint32_t)0x00010000)
#define  RCC_APB2RSTR_TIM16RST               ((uint32_t)0x00020000)
#define  RCC_APB2RSTR_TIM17RST               ((uint32_t)0x00040000)
#define  RCC_APB2RSTR_SAI1RST                ((uint32_t)0x00200000)
#define  RCC_APB2RSTR_SAI2RST                ((uint32_t)0x00400000)
#define  RCC_APB2RSTR_DFSDMRST               ((uint32_t)0x01000000)

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define  RCC_AHB1ENR_DMA1EN                  ((uint32_t)0x00000001)
#define  RCC_AHB1ENR_DMA2EN                  ((uint32_t)0x00000002)
#define  RCC_AHB1ENR_FLASHEN                 ((uint32_t)0x00000100)
#define  RCC_AHB1ENR_CRCEN                   ((uint32_t)0x00001000)
#define  RCC_AHB1ENR_TSCEN                   ((uint32_t)0x00010000)

/********************  Bit definition for RCC_AHB2ENR register  ***************/
#define  RCC_AHB2ENR_GPIOAEN                 ((uint32_t)0x00000001)
#define  RCC_AHB2ENR_GPIOBEN                 ((uint32_t)0x00000002)
#define  RCC_AHB2ENR_GPIOCEN                 ((uint32_t)0x00000004)
#define  RCC_AHB2ENR_GPIODEN                 ((uint32_t)0x00000008)
#define  RCC_AHB2ENR_GPIOEEN                 ((uint32_t)0x00000010)
#define  RCC_AHB2ENR_GPIOFEN                 ((uint32_t)0x00000020)
#define  RCC_AHB2ENR_GPIOGEN                 ((uint32_t)0x00000040)
#define  RCC_AHB2ENR_GPIOHEN                 ((uint32_t)0x00000080)
#define  RCC_AHB2ENR_OTGFSEN                 ((uint32_t)0x00001000)
#define  RCC_AHB2ENR_ADCEN                   ((uint32_t)0x00002000)
#define  RCC_AHB2ENR_RNGEN                   ((uint32_t)0x00040000)

/********************  Bit definition for RCC_AHB3ENR register  ***************/
#define  RCC_AHB3ENR_FMCEN                   ((uint32_t)0x00000001)
#define  RCC_AHB3ENR_QSPIEN                  ((uint32_t)0x00000100)

/********************  Bit definition for RCC_APB1ENR1 register  ***************/
#define  RCC_APB1ENR1_TIM2EN                 ((uint32_t)0x00000001)
#define  RCC_APB1ENR1_TIM3EN                 ((uint32_t)0x00000002)
#define  RCC_APB1ENR1_TIM4EN                 ((uint32_t)0x00000004)
#define  RCC_APB1ENR1_TIM5EN                 ((uint32_t)0x00000008)
#define  RCC_APB1ENR1_TIM6EN                 ((uint32_t)0x00000010)
#define  RCC_APB1ENR1_TIM7EN                 ((uint32_t)0x00000020)
#define  RCC_APB1ENR1_LCDEN                  ((uint32_t)0x00000200)
#define  RCC_APB1ENR1_WWDGEN                 ((uint32_t)0x00000800)
#define  RCC_APB1ENR1_SPI2EN                 ((uint32_t)0x00004000)
#define  RCC_APB1ENR1_SPI3EN                 ((uint32_t)0x00008000)
#define  RCC_APB1ENR1_USART2EN               ((uint32_t)0x00020000)
#define  RCC_APB1ENR1_USART3EN               ((uint32_t)0x00040000)
#define  RCC_APB1ENR1_UART4EN                ((uint32_t)0x00080000)
#define  RCC_APB1ENR1_UART5EN                ((uint32_t)0x00100000)
#define  RCC_APB1ENR1_I2C1EN                 ((uint32_t)0x00200000)
#define  RCC_APB1ENR1_I2C2EN                 ((uint32_t)0x00400000)
#define  RCC_APB1ENR1_I2C3EN                 ((uint32_t)0x00800000)
#define  RCC_APB1ENR1_CAN1EN                 ((uint32_t)0x02000000)
#define  RCC_APB1ENR1_PWREN                  ((uint32_t)0x10000000)
#define  RCC_APB1ENR1_DAC1EN                 ((uint32_t)0x20000000)
#define  RCC_APB1ENR1_OPAMPEN                ((uint32_t)0x40000000)
#define  RCC_APB1ENR1_LPTIM1EN               ((uint32_t)0x80000000)

/********************  Bit definition for RCC_APB1RSTR2 register  **************/
#define  RCC_APB1ENR2_LPUART1EN              ((uint32_t)0x00000001)
#define  RCC_APB1ENR2_SWPMI1EN               ((uint32_t)0x00000004)
#define  RCC_APB1ENR2_LPTIM2EN               ((uint32_t)0x00000020)

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define  RCC_APB2ENR_SYSCFGEN                ((uint32_t)0x00000001)
#define  RCC_APB2ENR_FWEN                    ((uint32_t)0x00000080)
#define  RCC_APB2ENR_SDMMC1EN                ((uint32_t)0x00000400)
#define  RCC_APB2ENR_TIM1EN                  ((uint32_t)0x00000800)
#define  RCC_APB2ENR_SPI1EN                  ((uint32_t)0x00001000)
#define  RCC_APB2ENR_TIM8EN                  ((uint32_t)0x00002000)
#define  RCC_APB2ENR_USART1EN                ((uint32_t)0x00004000)
#define  RCC_APB2ENR_TIM15EN                 ((uint32_t)0x00010000)
#define  RCC_APB2ENR_TIM16EN                 ((uint32_t)0x00020000)
#define  RCC_APB2ENR_TIM17EN                 ((uint32_t)0x00040000)
#define  RCC_APB2ENR_SAI1EN                  ((uint32_t)0x00200000)
#define  RCC_APB2ENR_SAI2EN                  ((uint32_t)0x00400000)
#define  RCC_APB2ENR_DFSDMEN                 ((uint32_t)0x01000000)

/********************  Bit definition for RCC_AHB1SMENR register  ***************/
#define  RCC_AHB1SMENR_DMA1SMEN              ((uint32_t)0x00000001)
#define  RCC_AHB1SMENR_DMA2SMEN              ((uint32_t)0x00000002)
#define  RCC_AHB1SMENR_FLASHSMEN             ((uint32_t)0x00000100)
#define  RCC_AHB1SMENR_SRAM1SMEN             ((uint32_t)0x00000200)
#define  RCC_AHB1SMENR_CRCSMEN               ((uint32_t)0x00001000)
#define  RCC_AHB1SMENR_TSCSMEN               ((uint32_t)0x00010000)

/********************  Bit definition for RCC_AHB2SMENR register  *************/
#define  RCC_AHB2SMENR_GPIOASMEN             ((uint32_t)0x00000001)
#define  RCC_AHB2SMENR_GPIOBSMEN             ((uint32_t)0x00000002)
#define  RCC_AHB2SMENR_GPIOCSMEN             ((uint32_t)0x00000004)
#define  RCC_AHB2SMENR_GPIODSMEN             ((uint32_t)0x00000008)
#define  RCC_AHB2SMENR_GPIOESMEN             ((uint32_t)0x00000010)
#define  RCC_AHB2SMENR_GPIOFSMEN             ((uint32_t)0x00000020)
#define  RCC_AHB2SMENR_GPIOGSMEN             ((uint32_t)0x00000040)
#define  RCC_AHB2SMENR_GPIOHSMEN             ((uint32_t)0x00000080)
#define  RCC_AHB2SMENR_SRAM2SMEN             ((uint32_t)0x00000200)
#define  RCC_AHB2SMENR_OTGFSSMEN             ((uint32_t)0x00001000)
#define  RCC_AHB2SMENR_ADCSMEN               ((uint32_t)0x00002000)
#define  RCC_AHB2SMENR_RNGSMEN               ((uint32_t)0x00040000)

/********************  Bit definition for RCC_AHB3SMENR register  *************/
#define  RCC_AHB3SMENR_FMCSMEN               ((uint32_t)0x00000001)
#define  RCC_AHB3SMENR_QSPISMEN              ((uint32_t)0x00000100)

/********************  Bit definition for RCC_APB1SMENR1 register  *************/
#define  RCC_APB1SMENR1_TIM2SMEN             ((uint32_t)0x00000001)
#define  RCC_APB1SMENR1_TIM3SMEN             ((uint32_t)0x00000002)
#define  RCC_APB1SMENR1_TIM4SMEN             ((uint32_t)0x00000004)
#define  RCC_APB1SMENR1_TIM5SMEN             ((uint32_t)0x00000008)
#define  RCC_APB1SMENR1_TIM6SMEN             ((uint32_t)0x00000010)
#define  RCC_APB1SMENR1_TIM7SMEN             ((uint32_t)0x00000020)
#define  RCC_APB1SMENR1_LCDSMEN              ((uint32_t)0x00000200)
#define  RCC_APB1SMENR1_WWDGSMEN             ((uint32_t)0x00000800)
#define  RCC_APB1SMENR1_SPI2SMEN             ((uint32_t)0x00004000)
#define  RCC_APB1SMENR1_SPI3SMEN             ((uint32_t)0x00008000)
#define  RCC_APB1SMENR1_USART2SMEN           ((uint32_t)0x00020000)
#define  RCC_APB1SMENR1_USART3SMEN           ((uint32_t)0x00040000)
#define  RCC_APB1SMENR1_UART4SMEN            ((uint32_t)0x00080000)
#define  RCC_APB1SMENR1_UART5SMEN            ((uint32_t)0x00100000)
#define  RCC_APB1SMENR1_I2C1SMEN             ((uint32_t)0x00200000)
#define  RCC_APB1SMENR1_I2C2SMEN             ((uint32_t)0x00400000)
#define  RCC_APB1SMENR1_I2C3SMEN             ((uint32_t)0x00800000)
#define  RCC_APB1SMENR1_CAN1SMEN             ((uint32_t)0x02000000)
#define  RCC_APB1SMENR1_PWRSMEN              ((uint32_t)0x10000000)
#define  RCC_APB1SMENR1_DAC1SMEN             ((uint32_t)0x20000000)
#define  RCC_APB1SMENR1_OPAMPSMEN            ((uint32_t)0x40000000)
#define  RCC_APB1SMENR1_LPTIM1SMEN           ((uint32_t)0x80000000)

/********************  Bit definition for RCC_APB1SMENR2 register  *************/
#define  RCC_APB1SMENR2_LPUART1SMEN          ((uint32_t)0x00000001)
#define  RCC_APB1SMENR2_SWPMI1SMEN           ((uint32_t)0x00000004)
#define  RCC_APB1SMENR2_LPTIM2SMEN           ((uint32_t)0x00000020)

/********************  Bit definition for RCC_APB2SMENR register  *************/
#define  RCC_APB2SMENR_SYSCFGSMEN            ((uint32_t)0x00000001)
#define  RCC_APB2SMENR_SDMMC1SMEN            ((uint32_t)0x00000400)
#define  RCC_APB2SMENR_TIM1SMEN              ((uint32_t)0x00000800)
#define  RCC_APB2SMENR_SPI1SMEN              ((uint32_t)0x00001000)
#define  RCC_APB2SMENR_TIM8SMEN              ((uint32_t)0x00002000)
#define  RCC_APB2SMENR_USART1SMEN            ((uint32_t)0x00004000)
#define  RCC_APB2SMENR_TIM15SMEN             ((uint32_t)0x00010000)
#define  RCC_APB2SMENR_TIM16SMEN             ((uint32_t)0x00020000)
#define  RCC_APB2SMENR_TIM17SMEN             ((uint32_t)0x00040000)
#define  RCC_APB2SMENR_SAI1SMEN              ((uint32_t)0x00200000)
#define  RCC_APB2SMENR_SAI2SMEN              ((uint32_t)0x00400000)
#define  RCC_APB2SMENR_DFSDMSMEN             ((uint32_t)0x01000000)

/********************  Bit definition for RCC_CCIPR register  ******************/
#define  RCC_CCIPR_USART1SEL                 ((uint32_t)0x00000003)
#define  RCC_CCIPR_USART1SEL_0               ((uint32_t)0x00000001)
#define  RCC_CCIPR_USART1SEL_1               ((uint32_t)0x00000002)

#define  RCC_CCIPR_USART2SEL                 ((uint32_t)0x0000000C)
#define  RCC_CCIPR_USART2SEL_0               ((uint32_t)0x00000004)
#define  RCC_CCIPR_USART2SEL_1               ((uint32_t)0x00000008)

#define  RCC_CCIPR_USART3SEL                 ((uint32_t)0x00000030)
#define  RCC_CCIPR_USART3SEL_0               ((uint32_t)0x00000010)
#define  RCC_CCIPR_USART3SEL_1               ((uint32_t)0x00000020)

#define  RCC_CCIPR_UART4SEL                  ((uint32_t)0x000000C0)
#define  RCC_CCIPR_UART4SEL_0                ((uint32_t)0x00000040)
#define  RCC_CCIPR_UART4SEL_1                ((uint32_t)0x00000080)

#define  RCC_CCIPR_UART5SEL                  ((uint32_t)0x00000300)
#define  RCC_CCIPR_UART5SEL_0                ((uint32_t)0x00000100)
#define  RCC_CCIPR_UART5SEL_1                ((uint32_t)0x00000200)

#define  RCC_CCIPR_LPUART1SEL                ((uint32_t)0x00000C00)
#define  RCC_CCIPR_LPUART1SEL_0              ((uint32_t)0x00000400)
#define  RCC_CCIPR_LPUART1SEL_1              ((uint32_t)0x00000800)

#define  RCC_CCIPR_I2C1SEL                   ((uint32_t)0x00003000)
#define  RCC_CCIPR_I2C1SEL_0                 ((uint32_t)0x00001000)
#define  RCC_CCIPR_I2C1SEL_1                 ((uint32_t)0x00002000)

#define  RCC_CCIPR_I2C2SEL                   ((uint32_t)0x0000C000)
#define  RCC_CCIPR_I2C2SEL_0                 ((uint32_t)0x00004000)
#define  RCC_CCIPR_I2C2SEL_1                 ((uint32_t)0x00008000)

#define  RCC_CCIPR_I2C3SEL                   ((uint32_t)0x00030000)
#define  RCC_CCIPR_I2C3SEL_0                 ((uint32_t)0x00010000)
#define  RCC_CCIPR_I2C3SEL_1                 ((uint32_t)0x00020000)

#define  RCC_CCIPR_LPTIM1SEL                 ((uint32_t)0x000C0000)
#define  RCC_CCIPR_LPTIM1SEL_0               ((uint32_t)0x00040000)
#define  RCC_CCIPR_LPTIM1SEL_1               ((uint32_t)0x00080000)

#define  RCC_CCIPR_LPTIM2SEL                 ((uint32_t)0x00300000)
#define  RCC_CCIPR_LPTIM2SEL_0               ((uint32_t)0x00100000)
#define  RCC_CCIPR_LPTIM2SEL_1               ((uint32_t)0x00200000)

#define  RCC_CCIPR_SAI1SEL                   ((uint32_t)0x00C00000)
#define  RCC_CCIPR_SAI1SEL_0                 ((uint32_t)0x00400000)
#define  RCC_CCIPR_SAI1SEL_1                 ((uint32_t)0x00800000)

#define  RCC_CCIPR_SAI2SEL                   ((uint32_t)0x03000000)
#define  RCC_CCIPR_SAI2SEL_0                 ((uint32_t)0x01000000)
#define  RCC_CCIPR_SAI2SEL_1                 ((uint32_t)0x02000000)

#define  RCC_CCIPR_CLK48SEL                  ((uint32_t)0x0C000000)
#define  RCC_CCIPR_CLK48SEL_0                ((uint32_t)0x04000000)
#define  RCC_CCIPR_CLK48SEL_1                ((uint32_t)0x08000000)

#define  RCC_CCIPR_ADCSEL                    ((uint32_t)0x30000000)
#define  RCC_CCIPR_ADCSEL_0                  ((uint32_t)0x10000000)
#define  RCC_CCIPR_ADCSEL_1                  ((uint32_t)0x20000000)

#define  RCC_CCIPR_SWPMI1SEL                 ((uint32_t)0x40000000)
#define  RCC_CCIPR_DFSDMSEL                  ((uint32_t)0x80000000)

/********************  Bit definition for RCC_BDCR register  ******************/
#define  RCC_BDCR_LSEON                      ((uint32_t)0x00000001)
#define  RCC_BDCR_LSERDY                     ((uint32_t)0x00000002)
#define  RCC_BDCR_LSEBYP                     ((uint32_t)0x00000004)

#define  RCC_BDCR_LSEDRV                     ((uint32_t)0x00000018)
#define  RCC_BDCR_LSEDRV_0                   ((uint32_t)0x00000008)
#define  RCC_BDCR_LSEDRV_1                   ((uint32_t)0x00000010)

#define  RCC_BDCR_LSECSSON                   ((uint32_t)0x00000020)
#define  RCC_BDCR_LSECSSD                    ((uint32_t)0x00000040)

#define  RCC_BDCR_RTCSEL                     ((uint32_t)0x00000300)
#define  RCC_BDCR_RTCSEL_0                   ((uint32_t)0x00000100)
#define  RCC_BDCR_RTCSEL_1                   ((uint32_t)0x00000200)

#define  RCC_BDCR_RTCEN                      ((uint32_t)0x00008000)
#define  RCC_BDCR_BDRST                      ((uint32_t)0x00010000)
#define  RCC_BDCR_LSCOEN                     ((uint32_t)0x01000000)
#define  RCC_BDCR_LSCOSEL                    ((uint32_t)0x02000000)

/********************  Bit definition for RCC_CSR register  *******************/
#define  RCC_CSR_LSION                       ((uint32_t)0x00000001)
#define  RCC_CSR_LSIRDY                      ((uint32_t)0x00000002)

#define  RCC_CSR_MSISRANGE                   ((uint32_t)0x00000F00)
#define  RCC_CSR_MSISRANGE_1                 ((uint32_t)0x00000400)      /*!< MSI frequency 1MHZ */
#define  RCC_CSR_MSISRANGE_2                 ((uint32_t)0x00000500)      /*!< MSI frequency 2MHZ */
#define  RCC_CSR_MSISRANGE_4                 ((uint32_t)0x00000600)      /*!< The default frequency 4MHZ */
#define  RCC_CSR_MSISRANGE_8                 ((uint32_t)0x00000700)      /*!< MSI frequency 8MHZ */

#define  RCC_CSR_RMVF                        ((uint32_t)0x00800000)
#define  RCC_CSR_FWRSTF                      ((uint32_t)0x01000000)
#define  RCC_CSR_OBLRSTF                     ((uint32_t)0x02000000)
#define  RCC_CSR_PINRSTF                     ((uint32_t)0x04000000)
#define  RCC_CSR_BORRSTF                     ((uint32_t)0x08000000)
#define  RCC_CSR_SFTRSTF                     ((uint32_t)0x10000000)
#define  RCC_CSR_IWDGRSTF                    ((uint32_t)0x20000000)
#define  RCC_CSR_WWDGRSTF                    ((uint32_t)0x40000000)
#define  RCC_CSR_LPWRRSTF                    ((uint32_t)0x80000000)

/********************  Bit definition for RCC_PLLSAI1CFGR register  ************/
#define  RCC_PLLSAI1CFGR_PLLSAI1N            ((uint32_t)0x00007F00)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_0          ((uint32_t)0x00000100)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_1          ((uint32_t)0x00000200)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_2          ((uint32_t)0x00000400)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_3          ((uint32_t)0x00000800)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_4          ((uint32_t)0x00001000)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_5          ((uint32_t)0x00002000)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_6          ((uint32_t)0x00004000)

#define  RCC_PLLSAI1CFGR_PLLSAI1PEN          ((uint32_t)0x00010000)
#define  RCC_PLLSAI1CFGR_PLLSAI1P            ((uint32_t)0x00020000)
#define  RCC_PLLSAI1CFGR_PLLSAI1QEN          ((uint32_t)0x00100000)
#define  RCC_PLLSAI1CFGR_PLLSAI1Q            ((uint32_t)0x00600000)
#define  RCC_PLLSAI1CFGR_PLLSAI1REN          ((uint32_t)0x01000000)
#define  RCC_PLLSAI1CFGR_PLLSAI1R            ((uint32_t)0x06000000)

/********************  Bit definition for RCC_PLLSAI2CFGR register  ************/
#define  RCC_PLLSAI2CFGR_PLLSAI2N            ((uint32_t)0x00007F00)
#define  RCC_PLLSAI2CFGR_PLLSAI2N_0          ((uint32_t)0x00000100)
#define  RCC_PLLSAI2CFGR_PLLSAI2N_1          ((uint32_t)0x00000200)
#define  RCC_PLLSAI2CFGR_PLLSAI2N_2          ((uint32_t)0x00000400)
#define  RCC_PLLSAI2CFGR_PLLSAI2N_3          ((uint32_t)0x00000800)
#define  RCC_PLLSAI2CFGR_PLLSAI2N_4          ((uint32_t)0x00001000)
#define  RCC_PLLSAI2CFGR_PLLSAI2N_5          ((uint32_t)0x00002000)
#define  RCC_PLLSAI2CFGR_PLLSAI2N_6          ((uint32_t)0x00004000)

#define  RCC_PLLSAI2CFGR_PLLSAI2PEN          ((uint32_t)0x00010000)
#define  RCC_PLLSAI2CFGR_PLLSAI2P            ((uint32_t)0x00020000)
#define  RCC_PLLSAI2CFGR_PLLSAI2REN          ((uint32_t)0x01000000)
#define  RCC_PLLSAI2CFGR_PLLSAI2R            ((uint32_t)0x06000000)



/******************************************************************************/
/*                                                                            */
/*                                    RNG                                     */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RNG_CR register  *******************/
#define RNG_CR_RNGEN                         ((uint32_t)0x00000004)
#define RNG_CR_IE                            ((uint32_t)0x00000008)

/********************  Bits definition for RNG_SR register  *******************/
#define RNG_SR_DRDY                          ((uint32_t)0x00000001)
#define RNG_SR_CECS                          ((uint32_t)0x00000002)
#define RNG_SR_SECS                          ((uint32_t)0x00000004)
#define RNG_SR_CEIS                          ((uint32_t)0x00000020)
#define RNG_SR_SEIS                          ((uint32_t)0x00000040)

/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTC)                            */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RTC_TR register  *******************/
#define RTC_TR_PM                            ((uint32_t)0x00400000)
#define RTC_TR_HT                            ((uint32_t)0x00300000)
#define RTC_TR_HT_0                          ((uint32_t)0x00100000)
#define RTC_TR_HT_1                          ((uint32_t)0x00200000)
#define RTC_TR_HU                            ((uint32_t)0x000F0000)
#define RTC_TR_HU_0                          ((uint32_t)0x00010000)
#define RTC_TR_HU_1                          ((uint32_t)0x00020000)
#define RTC_TR_HU_2                          ((uint32_t)0x00040000)
#define RTC_TR_HU_3                          ((uint32_t)0x00080000)
#define RTC_TR_MNT                           ((uint32_t)0x00007000)
#define RTC_TR_MNT_0                         ((uint32_t)0x00001000)
#define RTC_TR_MNT_1                         ((uint32_t)0x00002000)
#define RTC_TR_MNT_2                         ((uint32_t)0x00004000)
#define RTC_TR_MNU                           ((uint32_t)0x00000F00)
#define RTC_TR_MNU_0                         ((uint32_t)0x00000100)
#define RTC_TR_MNU_1                         ((uint32_t)0x00000200)
#define RTC_TR_MNU_2                         ((uint32_t)0x00000400)
#define RTC_TR_MNU_3                         ((uint32_t)0x00000800)
#define RTC_TR_ST                            ((uint32_t)0x00000070)
#define RTC_TR_ST_0                          ((uint32_t)0x00000010)
#define RTC_TR_ST_1                          ((uint32_t)0x00000020)
#define RTC_TR_ST_2                          ((uint32_t)0x00000040)
#define RTC_TR_SU                            ((uint32_t)0x0000000F)
#define RTC_TR_SU_0                          ((uint32_t)0x00000001)
#define RTC_TR_SU_1                          ((uint32_t)0x00000002)
#define RTC_TR_SU_2                          ((uint32_t)0x00000004)
#define RTC_TR_SU_3                          ((uint32_t)0x00000008)

/********************  Bits definition for RTC_DR register  *******************/
#define RTC_DR_YT                            ((uint32_t)0x00F00000)
#define RTC_DR_YT_0                          ((uint32_t)0x00100000)
#define RTC_DR_YT_1                          ((uint32_t)0x00200000)
#define RTC_DR_YT_2                          ((uint32_t)0x00400000)
#define RTC_DR_YT_3                          ((uint32_t)0x00800000)
#define RTC_DR_YU                            ((uint32_t)0x000F0000)
#define RTC_DR_YU_0                          ((uint32_t)0x00010000)
#define RTC_DR_YU_1                          ((uint32_t)0x00020000)
#define RTC_DR_YU_2                          ((uint32_t)0x00040000)
#define RTC_DR_YU_3                          ((uint32_t)0x00080000)
#define RTC_DR_WDU                           ((uint32_t)0x0000E000)
#define RTC_DR_WDU_0                         ((uint32_t)0x00002000)
#define RTC_DR_WDU_1                         ((uint32_t)0x00004000)
#define RTC_DR_WDU_2                         ((uint32_t)0x00008000)
#define RTC_DR_MT                            ((uint32_t)0x00001000)
#define RTC_DR_MU                            ((uint32_t)0x00000F00)
#define RTC_DR_MU_0                          ((uint32_t)0x00000100)
#define RTC_DR_MU_1                          ((uint32_t)0x00000200)
#define RTC_DR_MU_2                          ((uint32_t)0x00000400)
#define RTC_DR_MU_3                          ((uint32_t)0x00000800)
#define RTC_DR_DT                            ((uint32_t)0x00000030)
#define RTC_DR_DT_0                          ((uint32_t)0x00000010)
#define RTC_DR_DT_1                          ((uint32_t)0x00000020)
#define RTC_DR_DU                            ((uint32_t)0x0000000F)
#define RTC_DR_DU_0                          ((uint32_t)0x00000001)
#define RTC_DR_DU_1                          ((uint32_t)0x00000002)
#define RTC_DR_DU_2                          ((uint32_t)0x00000004)
#define RTC_DR_DU_3                          ((uint32_t)0x00000008)

/********************  Bits definition for RTC_CR register  *******************/
#define RTC_CR_ITSE                          ((uint32_t)0x01000000)
#define RTC_CR_COE                           ((uint32_t)0x00800000)
#define RTC_CR_OSEL                          ((uint32_t)0x00600000)
#define RTC_CR_OSEL_0                        ((uint32_t)0x00200000)
#define RTC_CR_OSEL_1                        ((uint32_t)0x00400000)
#define RTC_CR_POL                           ((uint32_t)0x00100000)
#define RTC_CR_COSEL                         ((uint32_t)0x00080000)
#define RTC_CR_BCK                           ((uint32_t)0x00040000)
#define RTC_CR_SUB1H                         ((uint32_t)0x00020000)
#define RTC_CR_ADD1H                         ((uint32_t)0x00010000)
#define RTC_CR_TSIE                          ((uint32_t)0x00008000)
#define RTC_CR_WUTIE                         ((uint32_t)0x00004000)
#define RTC_CR_ALRBIE                        ((uint32_t)0x00002000)
#define RTC_CR_ALRAIE                        ((uint32_t)0x00001000)
#define RTC_CR_TSE                           ((uint32_t)0x00000800)
#define RTC_CR_WUTE                          ((uint32_t)0x00000400)
#define RTC_CR_ALRBE                         ((uint32_t)0x00000200)
#define RTC_CR_ALRAE                         ((uint32_t)0x00000100)
#define RTC_CR_FMT                           ((uint32_t)0x00000040)
#define RTC_CR_BYPSHAD                       ((uint32_t)0x00000020)
#define RTC_CR_REFCKON                       ((uint32_t)0x00000010)
#define RTC_CR_TSEDGE                        ((uint32_t)0x00000008)
#define RTC_CR_WUCKSEL                       ((uint32_t)0x00000007)
#define RTC_CR_WUCKSEL_0                     ((uint32_t)0x00000001)
#define RTC_CR_WUCKSEL_1                     ((uint32_t)0x00000002)
#define RTC_CR_WUCKSEL_2                     ((uint32_t)0x00000004)

/********************  Bits definition for RTC_ISR register  ******************/
#define RTC_ISR_ITSF                         ((uint32_t)0x00020000)
#define RTC_ISR_RECALPF                      ((uint32_t)0x00010000)
#define RTC_ISR_TAMP3F                       ((uint32_t)0x00008000)
#define RTC_ISR_TAMP2F                       ((uint32_t)0x00004000)
#define RTC_ISR_TAMP1F                       ((uint32_t)0x00002000)
#define RTC_ISR_TSOVF                        ((uint32_t)0x00001000)
#define RTC_ISR_TSF                          ((uint32_t)0x00000800)
#define RTC_ISR_WUTF                         ((uint32_t)0x00000400)
#define RTC_ISR_ALRBF                        ((uint32_t)0x00000200)
#define RTC_ISR_ALRAF                        ((uint32_t)0x00000100)
#define RTC_ISR_INIT                         ((uint32_t)0x00000080)
#define RTC_ISR_INITF                        ((uint32_t)0x00000040)
#define RTC_ISR_RSF                          ((uint32_t)0x00000020)
#define RTC_ISR_INITS                        ((uint32_t)0x00000010)
#define RTC_ISR_SHPF                         ((uint32_t)0x00000008)
#define RTC_ISR_WUTWF                        ((uint32_t)0x00000004)
#define RTC_ISR_ALRBWF                       ((uint32_t)0x00000002)
#define RTC_ISR_ALRAWF                       ((uint32_t)0x00000001)

/********************  Bits definition for RTC_PRER register  *****************/
#define RTC_PRER_PREDIV_A                    ((uint32_t)0x007F0000)
#define RTC_PRER_PREDIV_S                    ((uint32_t)0x00007FFF)

/********************  Bits definition for RTC_WUTR register  *****************/
#define RTC_WUTR_WUT                         ((uint32_t)0x0000FFFF)

/********************  Bits definition for RTC_ALRMAR register  ***************/
#define RTC_ALRMAR_MSK4                      ((uint32_t)0x80000000)
#define RTC_ALRMAR_WDSEL                     ((uint32_t)0x40000000)
#define RTC_ALRMAR_DT                        ((uint32_t)0x30000000)
#define RTC_ALRMAR_DT_0                      ((uint32_t)0x10000000)
#define RTC_ALRMAR_DT_1                      ((uint32_t)0x20000000)
#define RTC_ALRMAR_DU                        ((uint32_t)0x0F000000)
#define RTC_ALRMAR_DU_0                      ((uint32_t)0x01000000)
#define RTC_ALRMAR_DU_1                      ((uint32_t)0x02000000)
#define RTC_ALRMAR_DU_2                      ((uint32_t)0x04000000)
#define RTC_ALRMAR_DU_3                      ((uint32_t)0x08000000)
#define RTC_ALRMAR_MSK3                      ((uint32_t)0x00800000)
#define RTC_ALRMAR_PM                        ((uint32_t)0x00400000)
#define RTC_ALRMAR_HT                        ((uint32_t)0x00300000)
#define RTC_ALRMAR_HT_0                      ((uint32_t)0x00100000)
#define RTC_ALRMAR_HT_1                      ((uint32_t)0x00200000)
#define RTC_ALRMAR_HU                        ((uint32_t)0x000F0000)
#define RTC_ALRMAR_HU_0                      ((uint32_t)0x00010000)
#define RTC_ALRMAR_HU_1                      ((uint32_t)0x00020000)
#define RTC_ALRMAR_HU_2                      ((uint32_t)0x00040000)
#define RTC_ALRMAR_HU_3                      ((uint32_t)0x00080000)
#define RTC_ALRMAR_MSK2                      ((uint32_t)0x00008000)
#define RTC_ALRMAR_MNT                       ((uint32_t)0x00007000)
#define RTC_ALRMAR_MNT_0                     ((uint32_t)0x00001000)
#define RTC_ALRMAR_MNT_1                     ((uint32_t)0x00002000)
#define RTC_ALRMAR_MNT_2                     ((uint32_t)0x00004000)
#define RTC_ALRMAR_MNU                       ((uint32_t)0x00000F00)
#define RTC_ALRMAR_MNU_0                     ((uint32_t)0x00000100)
#define RTC_ALRMAR_MNU_1                     ((uint32_t)0x00000200)
#define RTC_ALRMAR_MNU_2                     ((uint32_t)0x00000400)
#define RTC_ALRMAR_MNU_3                     ((uint32_t)0x00000800)
#define RTC_ALRMAR_MSK1                      ((uint32_t)0x00000080)
#define RTC_ALRMAR_ST                        ((uint32_t)0x00000070)
#define RTC_ALRMAR_ST_0                      ((uint32_t)0x00000010)
#define RTC_ALRMAR_ST_1                      ((uint32_t)0x00000020)
#define RTC_ALRMAR_ST_2                      ((uint32_t)0x00000040)
#define RTC_ALRMAR_SU                        ((uint32_t)0x0000000F)
#define RTC_ALRMAR_SU_0                      ((uint32_t)0x00000001)
#define RTC_ALRMAR_SU_1                      ((uint32_t)0x00000002)
#define RTC_ALRMAR_SU_2                      ((uint32_t)0x00000004)
#define RTC_ALRMAR_SU_3                      ((uint32_t)0x00000008)

/********************  Bits definition for RTC_ALRMBR register  ***************/
#define RTC_ALRMBR_MSK4                      ((uint32_t)0x80000000)
#define RTC_ALRMBR_WDSEL                     ((uint32_t)0x40000000)
#define RTC_ALRMBR_DT                        ((uint32_t)0x30000000)
#define RTC_ALRMBR_DT_0                      ((uint32_t)0x10000000)
#define RTC_ALRMBR_DT_1                      ((uint32_t)0x20000000)
#define RTC_ALRMBR_DU                        ((uint32_t)0x0F000000)
#define RTC_ALRMBR_DU_0                      ((uint32_t)0x01000000)
#define RTC_ALRMBR_DU_1                      ((uint32_t)0x02000000)
#define RTC_ALRMBR_DU_2                      ((uint32_t)0x04000000)
#define RTC_ALRMBR_DU_3                      ((uint32_t)0x08000000)
#define RTC_ALRMBR_MSK3                      ((uint32_t)0x00800000)
#define RTC_ALRMBR_PM                        ((uint32_t)0x00400000)
#define RTC_ALRMBR_HT                        ((uint32_t)0x00300000)
#define RTC_ALRMBR_HT_0                      ((uint32_t)0x00100000)
#define RTC_ALRMBR_HT_1                      ((uint32_t)0x00200000)
#define RTC_ALRMBR_HU                        ((uint32_t)0x000F0000)
#define RTC_ALRMBR_HU_0                      ((uint32_t)0x00010000)
#define RTC_ALRMBR_HU_1                      ((uint32_t)0x00020000)
#define RTC_ALRMBR_HU_2                      ((uint32_t)0x00040000)
#define RTC_ALRMBR_HU_3                      ((uint32_t)0x00080000)
#define RTC_ALRMBR_MSK2                      ((uint32_t)0x00008000)
#define RTC_ALRMBR_MNT                       ((uint32_t)0x00007000)
#define RTC_ALRMBR_MNT_0                     ((uint32_t)0x00001000)
#define RTC_ALRMBR_MNT_1                     ((uint32_t)0x00002000)
#define RTC_ALRMBR_MNT_2                     ((uint32_t)0x00004000)
#define RTC_ALRMBR_MNU                       ((uint32_t)0x00000F00)
#define RTC_ALRMBR_MNU_0                     ((uint32_t)0x00000100)
#define RTC_ALRMBR_MNU_1                     ((uint32_t)0x00000200)
#define RTC_ALRMBR_MNU_2                     ((uint32_t)0x00000400)
#define RTC_ALRMBR_MNU_3                     ((uint32_t)0x00000800)
#define RTC_ALRMBR_MSK1                      ((uint32_t)0x00000080)
#define RTC_ALRMBR_ST                        ((uint32_t)0x00000070)
#define RTC_ALRMBR_ST_0                      ((uint32_t)0x00000010)
#define RTC_ALRMBR_ST_1                      ((uint32_t)0x00000020)
#define RTC_ALRMBR_ST_2                      ((uint32_t)0x00000040)
#define RTC_ALRMBR_SU                        ((uint32_t)0x0000000F)
#define RTC_ALRMBR_SU_0                      ((uint32_t)0x00000001)
#define RTC_ALRMBR_SU_1                      ((uint32_t)0x00000002)
#define RTC_ALRMBR_SU_2                      ((uint32_t)0x00000004)
#define RTC_ALRMBR_SU_3                      ((uint32_t)0x00000008)

/********************  Bits definition for RTC_WPR register  ******************/
#define RTC_WPR_KEY                          ((uint32_t)0x000000FF)

/********************  Bits definition for RTC_SSR register  ******************/
#define RTC_SSR_SS                           ((uint32_t)0x0000FFFF)

/********************  Bits definition for RTC_SHIFTR register  ***************/
#define RTC_SHIFTR_SUBFS                     ((uint32_t)0x00007FFF)
#define RTC_SHIFTR_ADD1S                     ((uint32_t)0x80000000)

/********************  Bits definition for RTC_TSTR register  *****************/
#define RTC_TSTR_PM                          ((uint32_t)0x00400000)
#define RTC_TSTR_HT                          ((uint32_t)0x00300000)
#define RTC_TSTR_HT_0                        ((uint32_t)0x00100000)
#define RTC_TSTR_HT_1                        ((uint32_t)0x00200000)
#define RTC_TSTR_HU                          ((uint32_t)0x000F0000)
#define RTC_TSTR_HU_0                        ((uint32_t)0x00010000)
#define RTC_TSTR_HU_1                        ((uint32_t)0x00020000)
#define RTC_TSTR_HU_2                        ((uint32_t)0x00040000)
#define RTC_TSTR_HU_3                        ((uint32_t)0x00080000)
#define RTC_TSTR_MNT                         ((uint32_t)0x00007000)
#define RTC_TSTR_MNT_0                       ((uint32_t)0x00001000)
#define RTC_TSTR_MNT_1                       ((uint32_t)0x00002000)
#define RTC_TSTR_MNT_2                       ((uint32_t)0x00004000)
#define RTC_TSTR_MNU                         ((uint32_t)0x00000F00)
#define RTC_TSTR_MNU_0                       ((uint32_t)0x00000100)
#define RTC_TSTR_MNU_1                       ((uint32_t)0x00000200)
#define RTC_TSTR_MNU_2                       ((uint32_t)0x00000400)
#define RTC_TSTR_MNU_3                       ((uint32_t)0x00000800)
#define RTC_TSTR_ST                          ((uint32_t)0x00000070)
#define RTC_TSTR_ST_0                        ((uint32_t)0x00000010)
#define RTC_TSTR_ST_1                        ((uint32_t)0x00000020)
#define RTC_TSTR_ST_2                        ((uint32_t)0x00000040)
#define RTC_TSTR_SU                          ((uint32_t)0x0000000F)
#define RTC_TSTR_SU_0                        ((uint32_t)0x00000001)
#define RTC_TSTR_SU_1                        ((uint32_t)0x00000002)
#define RTC_TSTR_SU_2                        ((uint32_t)0x00000004)
#define RTC_TSTR_SU_3                        ((uint32_t)0x00000008)

/********************  Bits definition for RTC_TSDR register  *****************/
#define RTC_TSDR_WDU                         ((uint32_t)0x0000E000)
#define RTC_TSDR_WDU_0                       ((uint32_t)0x00002000)
#define RTC_TSDR_WDU_1                       ((uint32_t)0x00004000)
#define RTC_TSDR_WDU_2                       ((uint32_t)0x00008000)
#define RTC_TSDR_MT                          ((uint32_t)0x00001000)
#define RTC_TSDR_MU                          ((uint32_t)0x00000F00)
#define RTC_TSDR_MU_0                        ((uint32_t)0x00000100)
#define RTC_TSDR_MU_1                        ((uint32_t)0x00000200)
#define RTC_TSDR_MU_2                        ((uint32_t)0x00000400)
#define RTC_TSDR_MU_3                        ((uint32_t)0x00000800)
#define RTC_TSDR_DT                          ((uint32_t)0x00000030)
#define RTC_TSDR_DT_0                        ((uint32_t)0x00000010)
#define RTC_TSDR_DT_1                        ((uint32_t)0x00000020)
#define RTC_TSDR_DU                          ((uint32_t)0x0000000F)
#define RTC_TSDR_DU_0                        ((uint32_t)0x00000001)
#define RTC_TSDR_DU_1                        ((uint32_t)0x00000002)
#define RTC_TSDR_DU_2                        ((uint32_t)0x00000004)
#define RTC_TSDR_DU_3                        ((uint32_t)0x00000008)

/********************  Bits definition for RTC_TSSSR register  ****************/
#define RTC_TSSSR_SS                         ((uint32_t)0x0000FFFF)

/********************  Bits definition for RTC_CAL register  *****************/
#define RTC_CALR_CALP                        ((uint32_t)0x00008000)
#define RTC_CALR_CALW8                       ((uint32_t)0x00004000)
#define RTC_CALR_CALW16                      ((uint32_t)0x00002000)
#define RTC_CALR_CALM                        ((uint32_t)0x000001FF)
#define RTC_CALR_CALM_0                      ((uint32_t)0x00000001)
#define RTC_CALR_CALM_1                      ((uint32_t)0x00000002)
#define RTC_CALR_CALM_2                      ((uint32_t)0x00000004)
#define RTC_CALR_CALM_3                      ((uint32_t)0x00000008)
#define RTC_CALR_CALM_4                      ((uint32_t)0x00000010)
#define RTC_CALR_CALM_5                      ((uint32_t)0x00000020)
#define RTC_CALR_CALM_6                      ((uint32_t)0x00000040)
#define RTC_CALR_CALM_7                      ((uint32_t)0x00000080)
#define RTC_CALR_CALM_8                      ((uint32_t)0x00000100)

/********************  Bits definition for RTC_TAMPCR register  ***************/
#define RTC_TAMPCR_TAMP3MF                   ((uint32_t)0x01000000)
#define RTC_TAMPCR_TAMP3NOERASE              ((uint32_t)0x00800000)
#define RTC_TAMPCR_TAMP3IE                   ((uint32_t)0x00400000)
#define RTC_TAMPCR_TAMP2MF                   ((uint32_t)0x00200000)
#define RTC_TAMPCR_TAMP2NOERASE              ((uint32_t)0x00100000)
#define RTC_TAMPCR_TAMP2IE                   ((uint32_t)0x00080000)
#define RTC_TAMPCR_TAMP1MF                   ((uint32_t)0x00040000)
#define RTC_TAMPCR_TAMP1NOERASE              ((uint32_t)0x00020000)
#define RTC_TAMPCR_TAMP1IE                   ((uint32_t)0x00010000)
#define RTC_TAMPCR_TAMPPUDIS                 ((uint32_t)0x00008000)
#define RTC_TAMPCR_TAMPPRCH                  ((uint32_t)0x00006000)
#define RTC_TAMPCR_TAMPPRCH_0                ((uint32_t)0x00002000)
#define RTC_TAMPCR_TAMPPRCH_1                ((uint32_t)0x00004000)
#define RTC_TAMPCR_TAMPFLT                   ((uint32_t)0x00001800)
#define RTC_TAMPCR_TAMPFLT_0                 ((uint32_t)0x00000800)
#define RTC_TAMPCR_TAMPFLT_1                 ((uint32_t)0x00001000)
#define RTC_TAMPCR_TAMPFREQ                  ((uint32_t)0x00000700)
#define RTC_TAMPCR_TAMPFREQ_0                ((uint32_t)0x00000100)
#define RTC_TAMPCR_TAMPFREQ_1                ((uint32_t)0x00000200)
#define RTC_TAMPCR_TAMPFREQ_2                ((uint32_t)0x00000400)
#define RTC_TAMPCR_TAMPTS                    ((uint32_t)0x00000080)
#define RTC_TAMPCR_TAMP3TRG                  ((uint32_t)0x00000040)
#define RTC_TAMPCR_TAMP3E                    ((uint32_t)0x00000020)
#define RTC_TAMPCR_TAMP2TRG                  ((uint32_t)0x00000010)
#define RTC_TAMPCR_TAMP2E                    ((uint32_t)0x00000008)
#define RTC_TAMPCR_TAMPIE                    ((uint32_t)0x00000004)
#define RTC_TAMPCR_TAMP1TRG                  ((uint32_t)0x00000002)
#define RTC_TAMPCR_TAMP1E                    ((uint32_t)0x00000001)

/********************  Bits definition for RTC_ALRMASSR register  *************/
#define RTC_ALRMASSR_MASKSS                  ((uint32_t)0x0F000000)
#define RTC_ALRMASSR_MASKSS_0                ((uint32_t)0x01000000)
#define RTC_ALRMASSR_MASKSS_1                ((uint32_t)0x02000000)
#define RTC_ALRMASSR_MASKSS_2                ((uint32_t)0x04000000)
#define RTC_ALRMASSR_MASKSS_3                ((uint32_t)0x08000000)
#define RTC_ALRMASSR_SS                      ((uint32_t)0x00007FFF)

/********************  Bits definition for RTC_ALRMBSSR register  *************/
#define RTC_ALRMBSSR_MASKSS                  ((uint32_t)0x0F000000)
#define RTC_ALRMBSSR_MASKSS_0                ((uint32_t)0x01000000)
#define RTC_ALRMBSSR_MASKSS_1                ((uint32_t)0x02000000)
#define RTC_ALRMBSSR_MASKSS_2                ((uint32_t)0x04000000)
#define RTC_ALRMBSSR_MASKSS_3                ((uint32_t)0x08000000)
#define RTC_ALRMBSSR_SS                      ((uint32_t)0x00007FFF)

/********************  Bits definition for RTC_0R register  *******************/
#define RTC_OR_OUT_RMP                      ((uint32_t)0x00000002)
#define RTC_OR_ALARMOUTTYPE                 ((uint32_t)0x00000001)


/********************  Bits definition for RTC_BKP0R register  ****************/
#define RTC_BKP0R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP1R register  ****************/
#define RTC_BKP1R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP2R register  ****************/
#define RTC_BKP2R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP3R register  ****************/
#define RTC_BKP3R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP4R register  ****************/
#define RTC_BKP4R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP5R register  ****************/
#define RTC_BKP5R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP6R register  ****************/
#define RTC_BKP6R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP7R register  ****************/
#define RTC_BKP7R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP8R register  ****************/
#define RTC_BKP8R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP9R register  ****************/
#define RTC_BKP9R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP10R register  ***************/
#define RTC_BKP10R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP11R register  ***************/
#define RTC_BKP11R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP12R register  ***************/
#define RTC_BKP12R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP13R register  ***************/
#define RTC_BKP13R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP14R register  ***************/
#define RTC_BKP14R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP15R register  ***************/
#define RTC_BKP15R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP16R register  ***************/
#define RTC_BKP16R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP17R register  ***************/
#define RTC_BKP17R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP18R register  ***************/
#define RTC_BKP18R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP19R register  ***************/
#define RTC_BKP19R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP20R register  ***************/
#define RTC_BKP20R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP21R register  ***************/
#define RTC_BKP21R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP22R register  ***************/
#define RTC_BKP22R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP23R register  ***************/
#define RTC_BKP23R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP24R register  ***************/
#define RTC_BKP24R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP25R register  ***************/
#define RTC_BKP25R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP26R register  ***************/
#define RTC_BKP26R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP27R register  ***************/
#define RTC_BKP27R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP28R register  ***************/
#define RTC_BKP28R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP29R register  ***************/
#define RTC_BKP29R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP30R register  ***************/
#define RTC_BKP30R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP31R register  ***************/
#define RTC_BKP31R                           ((uint32_t)0xFFFFFFFF)

/******************** Number of backup registers ******************************/
#define RTC_BKP_NUMBER                       ((uint32_t)0x00000020)

/******************************************************************************/
/*                                                                            */
/*                          Serial Audio Interface                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for SAI_GCR register  *******************/
#define  SAI_GCR_SYNCIN                  ((uint32_t)0x00000003)        /*!<SYNCIN[1:0] bits (Synchronization Inputs)   */
#define  SAI_GCR_SYNCIN_0                ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  SAI_GCR_SYNCIN_1                ((uint32_t)0x00000002)        /*!<Bit 1 */

#define  SAI_GCR_SYNCOUT                 ((uint32_t)0x00000030)        /*!<SYNCOUT[1:0] bits (Synchronization Outputs) */
#define  SAI_GCR_SYNCOUT_0               ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  SAI_GCR_SYNCOUT_1               ((uint32_t)0x00000020)        /*!<Bit 1 */

/*******************  Bit definition for SAI_xCR1 register  *******************/
#define  SAI_xCR1_MODE                    ((uint32_t)0x00000003)        /*!<MODE[1:0] bits (Audio Block Mode)           */
#define  SAI_xCR1_MODE_0                  ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  SAI_xCR1_MODE_1                  ((uint32_t)0x00000002)        /*!<Bit 1 */

#define  SAI_xCR1_PRTCFG                  ((uint32_t)0x0000000C)        /*!<PRTCFG[1:0] bits (Protocol Configuration)   */
#define  SAI_xCR1_PRTCFG_0                ((uint32_t)0x00000004)        /*!<Bit 0 */
#define  SAI_xCR1_PRTCFG_1                ((uint32_t)0x00000008)        /*!<Bit 1 */

#define  SAI_xCR1_DS                      ((uint32_t)0x000000E0)        /*!<DS[1:0] bits (Data Size) */
#define  SAI_xCR1_DS_0                    ((uint32_t)0x00000020)        /*!<Bit 0 */
#define  SAI_xCR1_DS_1                    ((uint32_t)0x00000040)        /*!<Bit 1 */
#define  SAI_xCR1_DS_2                    ((uint32_t)0x00000080)        /*!<Bit 2 */

#define  SAI_xCR1_LSBFIRST                ((uint32_t)0x00000100)        /*!<LSB First Configuration  */
#define  SAI_xCR1_CKSTR                   ((uint32_t)0x00000200)        /*!<ClocK STRobing edge      */

#define  SAI_xCR1_SYNCEN                  ((uint32_t)0x00000C00)        /*!<SYNCEN[1:0](SYNChronization ENable) */
#define  SAI_xCR1_SYNCEN_0                ((uint32_t)0x00000400)        /*!<Bit 0 */
#define  SAI_xCR1_SYNCEN_1                ((uint32_t)0x00000800)        /*!<Bit 1 */

#define  SAI_xCR1_MONO                    ((uint32_t)0x00001000)        /*!<Mono mode                  */
#define  SAI_xCR1_OUTDRIV                 ((uint32_t)0x00002000)        /*!<Output Drive               */
#define  SAI_xCR1_SAIEN                   ((uint32_t)0x00010000)        /*!<Audio Block enable         */
#define  SAI_xCR1_DMAEN                   ((uint32_t)0x00020000)        /*!<DMA enable                 */
#define  SAI_xCR1_NODIV                   ((uint32_t)0x00080000)        /*!<No Divider Configuration   */

#define  SAI_xCR1_MCKDIV                  ((uint32_t)0x00F00000)        /*!<MCKDIV[3:0] (Master ClocK Divider)  */
#define  SAI_xCR1_MCKDIV_0                ((uint32_t)0x00100000)        /*!<Bit 0  */
#define  SAI_xCR1_MCKDIV_1                ((uint32_t)0x00200000)        /*!<Bit 1  */
#define  SAI_xCR1_MCKDIV_2                ((uint32_t)0x00400000)        /*!<Bit 2  */
#define  SAI_xCR1_MCKDIV_3                ((uint32_t)0x00800000)        /*!<Bit 3  */

/*******************  Bit definition for SAI_xCR2 register  *******************/
#define  SAI_xCR2_FTH                     ((uint32_t)0x00000007)        /*!<FTH[2:0](Fifo THreshold)  */
#define  SAI_xCR2_FTH_0                   ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  SAI_xCR2_FTH_1                   ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  SAI_xCR2_FTH_2                   ((uint32_t)0x00000004)        /*!<Bit 2 */

#define  SAI_xCR2_FFLUSH                  ((uint32_t)0x00000008)        /*!<Fifo FLUSH                       */
#define  SAI_xCR2_TRIS                    ((uint32_t)0x00000010)        /*!<TRIState Management on data line */
#define  SAI_xCR2_MUTE                    ((uint32_t)0x00000020)        /*!<Mute mode                        */
#define  SAI_xCR2_MUTEVAL                 ((uint32_t)0x00000040)        /*!<Muate value                      */


#define  SAI_xCR2_MUTECNT                 ((uint32_t)0x00001F80)        /*!<MUTECNT[5:0] (MUTE counter) */
#define  SAI_xCR2_MUTECNT_0               ((uint32_t)0x00000080)        /*!<Bit 0 */
#define  SAI_xCR2_MUTECNT_1               ((uint32_t)0x00000100)        /*!<Bit 1 */
#define  SAI_xCR2_MUTECNT_2               ((uint32_t)0x00000200)        /*!<Bit 2 */
#define  SAI_xCR2_MUTECNT_3               ((uint32_t)0x00000400)        /*!<Bit 3 */
#define  SAI_xCR2_MUTECNT_4               ((uint32_t)0x00000800)        /*!<Bit 4 */
#define  SAI_xCR2_MUTECNT_5               ((uint32_t)0x00001000)        /*!<Bit 5 */

#define  SAI_xCR2_CPL                     ((uint32_t)0x00002000)        /*!<CPL mode                    */
#define  SAI_xCR2_COMP                    ((uint32_t)0x0000C000)        /*!<COMP[1:0] (Companding mode) */
#define  SAI_xCR2_COMP_0                  ((uint32_t)0x00004000)        /*!<Bit 0 */
#define  SAI_xCR2_COMP_1                  ((uint32_t)0x00008000)        /*!<Bit 1 */


/******************  Bit definition for SAI_xFRCR register  *******************/
#define  SAI_xFRCR_FRL                    ((uint32_t)0x000000FF)        /*!<FRL[7:0](Frame length)  */
#define  SAI_xFRCR_FRL_0                  ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  SAI_xFRCR_FRL_1                  ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  SAI_xFRCR_FRL_2                  ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  SAI_xFRCR_FRL_3                  ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  SAI_xFRCR_FRL_4                  ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  SAI_xFRCR_FRL_5                  ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  SAI_xFRCR_FRL_6                  ((uint32_t)0x00000040)        /*!<Bit 6 */
#define  SAI_xFRCR_FRL_7                  ((uint32_t)0x00000080)        /*!<Bit 7 */

#define  SAI_xFRCR_FSALL                  ((uint32_t)0x00007F00)        /*!<FRL[6:0] (Frame synchronization active level length)  */
#define  SAI_xFRCR_FSALL_0                ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  SAI_xFRCR_FSALL_1                ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  SAI_xFRCR_FSALL_2                ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  SAI_xFRCR_FSALL_3                ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  SAI_xFRCR_FSALL_4                ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  SAI_xFRCR_FSALL_5                ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  SAI_xFRCR_FSALL_6                ((uint32_t)0x00004000)        /*!<Bit 6 */

#define  SAI_xFRCR_FSDEF                  ((uint32_t)0x00010000)        /*!< Frame Synchronization Definition */
#define  SAI_xFRCR_FSPO                   ((uint32_t)0x00020000)        /*!<Frame Synchronization POLarity    */
#define  SAI_xFRCR_FSOFF                  ((uint32_t)0x00040000)        /*!<Frame Synchronization OFFset      */

/******************  Bit definition for SAI_xSLOTR register  *******************/
#define  SAI_xSLOTR_FBOFF                 ((uint32_t)0x0000001F)        /*!<FRL[4:0](First Bit Offset)  */
#define  SAI_xSLOTR_FBOFF_0               ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  SAI_xSLOTR_FBOFF_1               ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  SAI_xSLOTR_FBOFF_2               ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  SAI_xSLOTR_FBOFF_3               ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  SAI_xSLOTR_FBOFF_4               ((uint32_t)0x00000010)        /*!<Bit 4 */

#define  SAI_xSLOTR_SLOTSZ                ((uint32_t)0x000000C0)        /*!<SLOTSZ[1:0] (Slot size)  */
#define  SAI_xSLOTR_SLOTSZ_0              ((uint32_t)0x00000040)        /*!<Bit 0 */
#define  SAI_xSLOTR_SLOTSZ_1              ((uint32_t)0x00000080)        /*!<Bit 1 */

#define  SAI_xSLOTR_NBSLOT                ((uint32_t)0x00000F00)        /*!<NBSLOT[3:0] (Number of Slot in audio Frame)  */
#define  SAI_xSLOTR_NBSLOT_0              ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  SAI_xSLOTR_NBSLOT_1              ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  SAI_xSLOTR_NBSLOT_2              ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  SAI_xSLOTR_NBSLOT_3              ((uint32_t)0x00000800)        /*!<Bit 3 */

#define  SAI_xSLOTR_SLOTEN                ((uint32_t)0xFFFF0000)        /*!<SLOTEN[15:0] (Slot Enable)  */

/*******************  Bit definition for SAI_xIMR register  *******************/
#define  SAI_xIMR_OVRUDRIE                ((uint32_t)0x00000001)        /*!<Overrun underrun interrupt enable                              */
#define  SAI_xIMR_MUTEDETIE               ((uint32_t)0x00000002)        /*!<Mute detection interrupt enable                                */
#define  SAI_xIMR_WCKCFGIE                ((uint32_t)0x00000004)        /*!<Wrong Clock Configuration interrupt enable                     */
#define  SAI_xIMR_FREQIE                  ((uint32_t)0x00000008)        /*!<FIFO request interrupt enable                                  */
#define  SAI_xIMR_CNRDYIE                 ((uint32_t)0x00000010)        /*!<Codec not ready interrupt enable                               */
#define  SAI_xIMR_AFSDETIE                ((uint32_t)0x00000020)        /*!<Anticipated frame synchronization detection interrupt enable   */
#define  SAI_xIMR_LFSDETIE                ((uint32_t)0x00000040)        /*!<Late frame synchronization detection interrupt enable          */

/********************  Bit definition for SAI_xSR register  *******************/
#define  SAI_xSR_OVRUDR                   ((uint32_t)0x00000001)         /*!<Overrun underrun                               */
#define  SAI_xSR_MUTEDET                  ((uint32_t)0x00000002)         /*!<Mute detection                                 */
#define  SAI_xSR_WCKCFG                   ((uint32_t)0x00000004)         /*!<Wrong Clock Configuration                      */
#define  SAI_xSR_FREQ                     ((uint32_t)0x00000008)         /*!<FIFO request                                   */
#define  SAI_xSR_CNRDY                    ((uint32_t)0x00000010)         /*!<Codec not ready                                */
#define  SAI_xSR_AFSDET                   ((uint32_t)0x00000020)         /*!<Anticipated frame synchronization detection    */
#define  SAI_xSR_LFSDET                   ((uint32_t)0x00000040)         /*!<Late frame synchronization detection           */

#define  SAI_xSR_FLVL                     ((uint32_t)0x00070000)         /*!<FLVL[2:0] (FIFO Level Threshold)               */
#define  SAI_xSR_FLVL_0                   ((uint32_t)0x00010000)         /*!<Bit 0 */
#define  SAI_xSR_FLVL_1                   ((uint32_t)0x00020000)         /*!<Bit 1 */
#define  SAI_xSR_FLVL_2                   ((uint32_t)0x00030000)         /*!<Bit 2 */

/******************  Bit definition for SAI_xCLRFR register  ******************/
#define  SAI_xCLRFR_COVRUDR               ((uint32_t)0x00000001)        /*!<Clear Overrun underrun                               */
#define  SAI_xCLRFR_CMUTEDET              ((uint32_t)0x00000002)        /*!<Clear Mute detection                                 */
#define  SAI_xCLRFR_CWCKCFG               ((uint32_t)0x00000004)        /*!<Clear Wrong Clock Configuration                      */
#define  SAI_xCLRFR_CFREQ                 ((uint32_t)0x00000008)        /*!<Clear FIFO request                                   */
#define  SAI_xCLRFR_CCNRDY                ((uint32_t)0x00000010)        /*!<Clear Codec not ready                                */
#define  SAI_xCLRFR_CAFSDET               ((uint32_t)0x00000020)        /*!<Clear Anticipated frame synchronization detection    */
#define  SAI_xCLRFR_CLFSDET               ((uint32_t)0x00000040)        /*!<Clear Late frame synchronization detection           */

/******************  Bit definition for SAI_xDR register  ******************/
#define  SAI_xDR_DATA                     ((uint32_t)0xFFFFFFFF)

/******************************************************************************/
/*                                                                            */
/*                          LCD Controller (LCD)                              */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for LCD_CR register  *********************/
#define LCD_CR_LCDEN               ((uint32_t)0x00000001)     /*!< LCD Enable Bit */
#define LCD_CR_VSEL                ((uint32_t)0x00000002)     /*!< Voltage source selector Bit */

#define LCD_CR_DUTY                ((uint32_t)0x0000001C)     /*!< DUTY[2:0] bits (Duty selector) */
#define LCD_CR_DUTY_0              ((uint32_t)0x00000004)     /*!< Duty selector Bit 0 */
#define LCD_CR_DUTY_1              ((uint32_t)0x00000008)     /*!< Duty selector Bit 1 */
#define LCD_CR_DUTY_2              ((uint32_t)0x00000010)     /*!< Duty selector Bit 2 */

#define LCD_CR_BIAS                ((uint32_t)0x00000060)     /*!< BIAS[1:0] bits (Bias selector) */
#define LCD_CR_BIAS_0              ((uint32_t)0x00000020)     /*!< Bias selector Bit 0 */
#define LCD_CR_BIAS_1              ((uint32_t)0x00000040)     /*!< Bias selector Bit 1 */

#define LCD_CR_MUX_SEG             ((uint32_t)0x00000080)     /*!< Mux Segment Enable Bit */
#define LCD_CR_BUFEN               ((uint32_t)0x00000100)     /*!< Voltage output buffer enable */

/*******************  Bit definition for LCD_FCR register  ********************/
#define LCD_FCR_HD                 ((uint32_t)0x00000001)     /*!< High Drive Enable Bit */
#define LCD_FCR_SOFIE              ((uint32_t)0x00000002)     /*!< Start of Frame Interrupt Enable Bit */
#define LCD_FCR_UDDIE              ((uint32_t)0x00000008)     /*!< Update Display Done Interrupt Enable Bit */

#define LCD_FCR_PON                ((uint32_t)0x00000070)     /*!< PON[2:0] bits (Pulse ON Duration) */
#define LCD_FCR_PON_0              ((uint32_t)0x00000010)     /*!< Bit 0 */
#define LCD_FCR_PON_1              ((uint32_t)0x00000020)     /*!< Bit 1 */
#define LCD_FCR_PON_2              ((uint32_t)0x00000040)     /*!< Bit 2 */

#define LCD_FCR_DEAD               ((uint32_t)0x00000380)     /*!< DEAD[2:0] bits (DEAD Time) */
#define LCD_FCR_DEAD_0             ((uint32_t)0x00000080)     /*!< Bit 0 */
#define LCD_FCR_DEAD_1             ((uint32_t)0x00000100)     /*!< Bit 1 */
#define LCD_FCR_DEAD_2             ((uint32_t)0x00000200)     /*!< Bit 2 */

#define LCD_FCR_CC                 ((uint32_t)0x00001C00)     /*!< CC[2:0] bits (Contrast Control) */
#define LCD_FCR_CC_0               ((uint32_t)0x00000400)     /*!< Bit 0 */
#define LCD_FCR_CC_1               ((uint32_t)0x00000800)     /*!< Bit 1 */
#define LCD_FCR_CC_2               ((uint32_t)0x00001000)     /*!< Bit 2 */

#define LCD_FCR_BLINKF             ((uint32_t)0x0000E000)     /*!< BLINKF[2:0] bits (Blink Frequency) */
#define LCD_FCR_BLINKF_0           ((uint32_t)0x00002000)     /*!< Bit 0 */
#define LCD_FCR_BLINKF_1           ((uint32_t)0x00004000)     /*!< Bit 1 */
#define LCD_FCR_BLINKF_2           ((uint32_t)0x00008000)     /*!< Bit 2 */

#define LCD_FCR_BLINK              ((uint32_t)0x00030000)     /*!< BLINK[1:0] bits (Blink Enable) */
#define LCD_FCR_BLINK_0            ((uint32_t)0x00010000)     /*!< Bit 0 */
#define LCD_FCR_BLINK_1            ((uint32_t)0x00020000)     /*!< Bit 1 */

#define LCD_FCR_DIV                ((uint32_t)0x003C0000)     /*!< DIV[3:0] bits (Divider) */
#define LCD_FCR_PS                 ((uint32_t)0x03C00000)     /*!< PS[3:0] bits (Prescaler) */

/*******************  Bit definition for LCD_SR register  *********************/
#define LCD_SR_ENS                 ((uint32_t)0x00000001)     /*!< LCD Enabled Bit */
#define LCD_SR_SOF                 ((uint32_t)0x00000002)     /*!< Start Of Frame Flag Bit */
#define LCD_SR_UDR                 ((uint32_t)0x00000004)     /*!< Update Display Request Bit */
#define LCD_SR_UDD                 ((uint32_t)0x00000008)     /*!< Update Display Done Flag Bit */
#define LCD_SR_RDY                 ((uint32_t)0x00000010)     /*!< Ready Flag Bit */
#define LCD_SR_FCRSR               ((uint32_t)0x00000020)     /*!< LCD FCR Register Synchronization Flag Bit */

/*******************  Bit definition for LCD_CLR register  ********************/
#define LCD_CLR_SOFC               ((uint32_t)0x00000002)     /*!< Start Of Frame Flag Clear Bit */
#define LCD_CLR_UDDC               ((uint32_t)0x00000008)     /*!< Update Display Done Flag Clear Bit */

/*******************  Bit definition for LCD_RAM register  ********************/
#define LCD_RAM_SEGMENT_DATA       ((uint32_t)0xFFFFFFFF)     /*!< Segment Data Bits */

/******************************************************************************/
/*                                                                            */
/*                           SDMMC Interface                                  */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SDMMC_POWER register  ******************/
#define  SDMMC_POWER_PWRCTRL                  ((uint8_t)0x03)               /*!<PWRCTRL[1:0] bits (Power supply control bits) */
#define  SDMMC_POWER_PWRCTRL_0                ((uint8_t)0x01)               /*!<Bit 0 */
#define  SDMMC_POWER_PWRCTRL_1                ((uint8_t)0x02)               /*!<Bit 1 */

/******************  Bit definition for SDMMC_CLKCR register  ******************/
#define  SDMMC_CLKCR_CLKDIV                   ((uint16_t)0x00FF)            /*!<Clock divide factor             */
#define  SDMMC_CLKCR_CLKEN                    ((uint16_t)0x0100)            /*!<Clock enable bit                */
#define  SDMMC_CLKCR_PWRSAV                   ((uint16_t)0x0200)            /*!<Power saving configuration bit  */
#define  SDMMC_CLKCR_BYPASS                   ((uint16_t)0x0400)            /*!<Clock divider bypass enable bit */

#define  SDMMC_CLKCR_WIDBUS                   ((uint16_t)0x1800)            /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define  SDMMC_CLKCR_WIDBUS_0                 ((uint16_t)0x0800)            /*!<Bit 0 */
#define  SDMMC_CLKCR_WIDBUS_1                 ((uint16_t)0x1000)            /*!<Bit 1 */

#define  SDMMC_CLKCR_NEGEDGE                  ((uint16_t)0x2000)            /*!<SDMMC_CK dephasing selection bit */
#define  SDMMC_CLKCR_HWFC_EN                  ((uint16_t)0x4000)            /*!<HW Flow Control enable          */

/*******************  Bit definition for SDMMC_ARG register  *******************/
#define  SDMMC_ARG_CMDARG                     ((uint32_t)0xFFFFFFFF)            /*!<Command argument */

/*******************  Bit definition for SDMMC_CMD register  *******************/
#define  SDMMC_CMD_CMDINDEX                   ((uint16_t)0x003F)            /*!<Command Index                               */

#define  SDMMC_CMD_WAITRESP                   ((uint16_t)0x00C0)            /*!<WAITRESP[1:0] bits (Wait for response bits) */
#define  SDMMC_CMD_WAITRESP_0                 ((uint16_t)0x0040)            /*!< Bit 0 */
#define  SDMMC_CMD_WAITRESP_1                 ((uint16_t)0x0080)            /*!< Bit 1 */

#define  SDMMC_CMD_WAITINT                    ((uint16_t)0x0100)            /*!<CPSM Waits for Interrupt Request                               */
#define  SDMMC_CMD_WAITPEND                   ((uint16_t)0x0200)            /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define  SDMMC_CMD_CPSMEN                     ((uint16_t)0x0400)            /*!<Command path state machine (CPSM) Enable bit                   */
#define  SDMMC_CMD_SDIOSUSPEND                ((uint16_t)0x0800)            /*!<SD I/O suspend command                                         */

/*****************  Bit definition for SDMMC_RESPCMD register  *****************/
#define  SDMMC_RESPCMD_RESPCMD                ((uint8_t)0x3F)               /*!<Response command index */

/******************  Bit definition for SDMMC_RESP0 register  ******************/
#define  SDMMC_RESP0_CARDSTATUS0              ((uint32_t)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDMMC_RESP1 register  ******************/
#define  SDMMC_RESP1_CARDSTATUS1              ((uint32_t)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDMMC_RESP2 register  ******************/
#define  SDMMC_RESP2_CARDSTATUS2              ((uint32_t)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDMMC_RESP3 register  ******************/
#define  SDMMC_RESP3_CARDSTATUS3              ((uint32_t)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDMMC_RESP4 register  ******************/
#define  SDMMC_RESP4_CARDSTATUS4              ((uint32_t)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDMMC_DTIMER register  *****************/
#define  SDMMC_DTIMER_DATATIME                ((uint32_t)0xFFFFFFFF)        /*!<Data timeout period. */

/******************  Bit definition for SDMMC_DLEN register  *******************/
#define  SDMMC_DLEN_DATALENGTH                ((uint32_t)0x01FFFFFF)        /*!<Data length value    */

/******************  Bit definition for SDMMC_DCTRL register  ******************/
#define  SDMMC_DCTRL_DTEN                     ((uint16_t)0x0001)            /*!<Data transfer enabled bit         */
#define  SDMMC_DCTRL_DTDIR                    ((uint16_t)0x0002)            /*!<Data transfer direction selection */
#define  SDMMC_DCTRL_DTMODE                   ((uint16_t)0x0004)            /*!<Data transfer mode selection      */
#define  SDMMC_DCTRL_DMAEN                    ((uint16_t)0x0008)            /*!<DMA enabled bit                   */

#define  SDMMC_DCTRL_DBLOCKSIZE               ((uint16_t)0x00F0)            /*!<DBLOCKSIZE[3:0] bits (Data block size) */
#define  SDMMC_DCTRL_DBLOCKSIZE_0             ((uint16_t)0x0010)            /*!<Bit 0 */
#define  SDMMC_DCTRL_DBLOCKSIZE_1             ((uint16_t)0x0020)            /*!<Bit 1 */
#define  SDMMC_DCTRL_DBLOCKSIZE_2             ((uint16_t)0x0040)            /*!<Bit 2 */
#define  SDMMC_DCTRL_DBLOCKSIZE_3             ((uint16_t)0x0080)            /*!<Bit 3 */

#define  SDMMC_DCTRL_RWSTART                  ((uint16_t)0x0100)            /*!<Read wait start         */
#define  SDMMC_DCTRL_RWSTOP                   ((uint16_t)0x0200)            /*!<Read wait stop          */
#define  SDMMC_DCTRL_RWMOD                    ((uint16_t)0x0400)            /*!<Read wait mode          */
#define  SDMMC_DCTRL_SDIOEN                   ((uint16_t)0x0800)            /*!<SD I/O enable functions */

/******************  Bit definition for SDMMC_DCOUNT register  *****************/
#define  SDMMC_DCOUNT_DATACOUNT               ((uint32_t)0x01FFFFFF)        /*!<Data count value */

/******************  Bit definition for SDMMC_STA register  ********************/
#define  SDMMC_STA_CCRCFAIL                   ((uint32_t)0x00000001)        /*!<Command response received (CRC check failed)  */
#define  SDMMC_STA_DCRCFAIL                   ((uint32_t)0x00000002)        /*!<Data block sent/received (CRC check failed)   */
#define  SDMMC_STA_CTIMEOUT                   ((uint32_t)0x00000004)        /*!<Command response timeout                      */
#define  SDMMC_STA_DTIMEOUT                   ((uint32_t)0x00000008)        /*!<Data timeout                                  */
#define  SDMMC_STA_TXUNDERR                   ((uint32_t)0x00000010)        /*!<Transmit FIFO underrun error                  */
#define  SDMMC_STA_RXOVERR                    ((uint32_t)0x00000020)        /*!<Received FIFO overrun error                   */
#define  SDMMC_STA_CMDREND                    ((uint32_t)0x00000040)        /*!<Command response received (CRC check passed)  */
#define  SDMMC_STA_CMDSENT                    ((uint32_t)0x00000080)        /*!<Command sent (no response required)           */
#define  SDMMC_STA_DATAEND                    ((uint32_t)0x00000100)        /*!<Data end (data counter, SDIDCOUNT, is zero)   */
#define  SDMMC_STA_STBITERR                   ((uint32_t)0x00000200)        /*!<Start bit not detected on all data signals in wide bus mode */
#define  SDMMC_STA_DBCKEND                    ((uint32_t)0x00000400)        /*!<Data block sent/received (CRC check passed)   */
#define  SDMMC_STA_CMDACT                     ((uint32_t)0x00000800)        /*!<Command transfer in progress                  */
#define  SDMMC_STA_TXACT                      ((uint32_t)0x00001000)        /*!<Data transmit in progress                     */
#define  SDMMC_STA_RXACT                      ((uint32_t)0x00002000)        /*!<Data receive in progress                      */
#define  SDMMC_STA_TXFIFOHE                   ((uint32_t)0x00004000)        /*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define  SDMMC_STA_RXFIFOHF                   ((uint32_t)0x00008000)        /*!<Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define  SDMMC_STA_TXFIFOF                    ((uint32_t)0x00010000)        /*!<Transmit FIFO full                            */
#define  SDMMC_STA_RXFIFOF                    ((uint32_t)0x00020000)        /*!<Receive FIFO full                             */
#define  SDMMC_STA_TXFIFOE                    ((uint32_t)0x00040000)        /*!<Transmit FIFO empty                           */
#define  SDMMC_STA_RXFIFOE                    ((uint32_t)0x00080000)        /*!<Receive FIFO empty                            */
#define  SDMMC_STA_TXDAVL                     ((uint32_t)0x00100000)        /*!<Data available in transmit FIFO               */
#define  SDMMC_STA_RXDAVL                     ((uint32_t)0x00200000)        /*!<Data available in receive FIFO                */
#define  SDMMC_STA_SDIOIT                     ((uint32_t)0x00400000)        /*!<SDIO interrupt received                       */

/*******************  Bit definition for SDMMC_ICR register  *******************/
#define  SDMMC_ICR_CCRCFAILC                  ((uint32_t)0x00000001)        /*!<CCRCFAIL flag clear bit */
#define  SDMMC_ICR_DCRCFAILC                  ((uint32_t)0x00000002)        /*!<DCRCFAIL flag clear bit */
#define  SDMMC_ICR_CTIMEOUTC                  ((uint32_t)0x00000004)        /*!<CTIMEOUT flag clear bit */
#define  SDMMC_ICR_DTIMEOUTC                  ((uint32_t)0x00000008)        /*!<DTIMEOUT flag clear bit */
#define  SDMMC_ICR_TXUNDERRC                  ((uint32_t)0x00000010)        /*!<TXUNDERR flag clear bit */
#define  SDMMC_ICR_RXOVERRC                   ((uint32_t)0x00000020)        /*!<RXOVERR flag clear bit  */
#define  SDMMC_ICR_CMDRENDC                   ((uint32_t)0x00000040)        /*!<CMDREND flag clear bit  */
#define  SDMMC_ICR_CMDSENTC                   ((uint32_t)0x00000080)        /*!<CMDSENT flag clear bit  */
#define  SDMMC_ICR_DATAENDC                   ((uint32_t)0x00000100)        /*!<DATAEND flag clear bit  */
#define  SDMMC_ICR_STBITERRC                  ((uint32_t)0x00000200)        /*!<STBITERR flag clear bit */
#define  SDMMC_ICR_DBCKENDC                   ((uint32_t)0x00000400)        /*!<DBCKEND flag clear bit  */
#define  SDMMC_ICR_SDIOITC                    ((uint32_t)0x00400000)        /*!<SDIOIT flag clear bit   */

/******************  Bit definition for SDMMC_MASK register  *******************/
#define  SDMMC_MASK_CCRCFAILIE                ((uint32_t)0x00000001)        /*!<Command CRC Fail Interrupt Enable          */
#define  SDMMC_MASK_DCRCFAILIE                ((uint32_t)0x00000002)        /*!<Data CRC Fail Interrupt Enable             */
#define  SDMMC_MASK_CTIMEOUTIE                ((uint32_t)0x00000004)        /*!<Command TimeOut Interrupt Enable           */
#define  SDMMC_MASK_DTIMEOUTIE                ((uint32_t)0x00000008)        /*!<Data TimeOut Interrupt Enable              */
#define  SDMMC_MASK_TXUNDERRIE                ((uint32_t)0x00000010)        /*!<Tx FIFO UnderRun Error Interrupt Enable    */
#define  SDMMC_MASK_RXOVERRIE                 ((uint32_t)0x00000020)        /*!<Rx FIFO OverRun Error Interrupt Enable     */
#define  SDMMC_MASK_CMDRENDIE                 ((uint32_t)0x00000040)        /*!<Command Response Received Interrupt Enable */
#define  SDMMC_MASK_CMDSENTIE                 ((uint32_t)0x00000080)        /*!<Command Sent Interrupt Enable              */
#define  SDMMC_MASK_DATAENDIE                 ((uint32_t)0x00000100)        /*!<Data End Interrupt Enable                  */
#define  SDMMC_MASK_DBCKENDIE                 ((uint32_t)0x00000400)        /*!<Data Block End Interrupt Enable            */
#define  SDMMC_MASK_CMDACTIE                  ((uint32_t)0x00000800)        /*!<CCommand Acting Interrupt Enable           */
#define  SDMMC_MASK_TXACTIE                   ((uint32_t)0x00001000)        /*!<Data Transmit Acting Interrupt Enable      */
#define  SDMMC_MASK_RXACTIE                   ((uint32_t)0x00002000)        /*!<Data receive acting interrupt enabled      */
#define  SDMMC_MASK_TXFIFOHEIE                ((uint32_t)0x00004000)        /*!<Tx FIFO Half Empty interrupt Enable        */
#define  SDMMC_MASK_RXFIFOHFIE                ((uint32_t)0x00008000)        /*!<Rx FIFO Half Full interrupt Enable         */
#define  SDMMC_MASK_TXFIFOFIE                 ((uint32_t)0x00010000)        /*!<Tx FIFO Full interrupt Enable              */
#define  SDMMC_MASK_RXFIFOFIE                 ((uint32_t)0x00020000)        /*!<Rx FIFO Full interrupt Enable              */
#define  SDMMC_MASK_TXFIFOEIE                 ((uint32_t)0x00040000)        /*!<Tx FIFO Empty interrupt Enable             */
#define  SDMMC_MASK_RXFIFOEIE                 ((uint32_t)0x00080000)        /*!<Rx FIFO Empty interrupt Enable             */
#define  SDMMC_MASK_TXDAVLIE                  ((uint32_t)0x00100000)        /*!<Data available in Tx FIFO interrupt Enable */
#define  SDMMC_MASK_RXDAVLIE                  ((uint32_t)0x00200000)        /*!<Data available in Rx FIFO interrupt Enable */
#define  SDMMC_MASK_SDIOITIE                  ((uint32_t)0x00400000)        /*!<SDIO Mode Interrupt Received interrupt Enable */

/*****************  Bit definition for SDMMC_FIFOCNT register  *****************/
#define  SDMMC_FIFOCNT_FIFOCOUNT              ((uint32_t)0x00FFFFFF)        /*!<Remaining number of words to be written to or read from the FIFO */

/******************  Bit definition for SDMMC_FIFO register  *******************/
#define  SDMMC_FIFO_FIFODATA                  ((uint32_t)0xFFFFFFFF)        /*!<Receive and transmit FIFO data */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface (SPI)                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for SPI_CR1 register  ********************/
#define  SPI_CR1_CPHA                        ((uint32_t)0x00000001)            /*!<Clock Phase      */
#define  SPI_CR1_CPOL                        ((uint32_t)0x00000002)            /*!<Clock Polarity   */
#define  SPI_CR1_MSTR                        ((uint32_t)0x00000004)            /*!<Master Selection */

#define  SPI_CR1_BR                          ((uint32_t)0x00000038)            /*!<BR[2:0] bits (Baud Rate Control) */
#define  SPI_CR1_BR_0                        ((uint32_t)0x00000008)            /*!<Bit 0 */
#define  SPI_CR1_BR_1                        ((uint32_t)0x00000010)            /*!<Bit 1 */
#define  SPI_CR1_BR_2                        ((uint32_t)0x00000020)            /*!<Bit 2 */

#define  SPI_CR1_SPE                         ((uint32_t)0x00000040)            /*!<SPI Enable                          */
#define  SPI_CR1_LSBFIRST                    ((uint32_t)0x00000080)            /*!<Frame Format                        */
#define  SPI_CR1_SSI                         ((uint32_t)0x00000100)            /*!<Internal slave select               */
#define  SPI_CR1_SSM                         ((uint32_t)0x00000200)            /*!<Software slave management           */
#define  SPI_CR1_RXONLY                      ((uint32_t)0x00000400)            /*!<Receive only                        */
#define  SPI_CR1_CRCL                        ((uint32_t)0x00000800)            /*!< CRC Length */
#define  SPI_CR1_CRCNEXT                     ((uint32_t)0x00001000)            /*!<Transmit CRC next                   */
#define  SPI_CR1_CRCEN                       ((uint32_t)0x00002000)            /*!<Hardware CRC calculation enable     */
#define  SPI_CR1_BIDIOE                      ((uint32_t)0x00004000)            /*!<Output enable in bidirectional mode */
#define  SPI_CR1_BIDIMODE                    ((uint32_t)0x00008000)            /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  SPI_CR2_RXDMAEN                     ((uint32_t)0x00000001)            /*!< Rx Buffer DMA Enable */
#define  SPI_CR2_TXDMAEN                     ((uint32_t)0x00000002)            /*!< Tx Buffer DMA Enable */
#define  SPI_CR2_SSOE                        ((uint32_t)0x00000004)            /*!< SS Output Enable */
#define  SPI_CR2_NSSP                        ((uint32_t)0x00000008)            /*!< NSS pulse management Enable */
#define  SPI_CR2_FRF                         ((uint32_t)0x00000010)            /*!< Frame Format Enable */
#define  SPI_CR2_ERRIE                       ((uint32_t)0x00000020)            /*!< Error Interrupt Enable */
#define  SPI_CR2_RXNEIE                      ((uint32_t)0x00000040)            /*!< RX buffer Not Empty Interrupt Enable */
#define  SPI_CR2_TXEIE                       ((uint32_t)0x00000080)            /*!< Tx buffer Empty Interrupt Enable */
#define  SPI_CR2_DS                          ((uint32_t)0x00000F00)            /*!< DS[3:0] Data Size */
#define  SPI_CR2_DS_0                        ((uint32_t)0x00000100)            /*!< Bit 0 */
#define  SPI_CR2_DS_1                        ((uint32_t)0x00000200)            /*!< Bit 1 */
#define  SPI_CR2_DS_2                        ((uint32_t)0x00000400)            /*!< Bit 2 */
#define  SPI_CR2_DS_3                        ((uint32_t)0x00000800)            /*!< Bit 3 */
#define  SPI_CR2_FRXTH                       ((uint32_t)0x00001000)            /*!< FIFO reception Threshold */
#define  SPI_CR2_LDMARX                      ((uint32_t)0x00002000)            /*!< Last DMA transfer for reception */
#define  SPI_CR2_LDMATX                      ((uint32_t)0x00004000)            /*!< Last DMA transfer for transmission */

/********************  Bit definition for SPI_SR register  ********************/
#define  SPI_SR_RXNE                         ((uint32_t)0x00000001)            /*!< Receive buffer Not Empty */
#define  SPI_SR_TXE                          ((uint32_t)0x00000002)            /*!< Transmit buffer Empty */
#define  SPI_SR_CHSIDE                       ((uint32_t)0x00000004)            /*!< Channel side */
#define  SPI_SR_UDR                          ((uint32_t)0x00000008)            /*!< Underrun flag */
#define  SPI_SR_CRCERR                       ((uint32_t)0x00000010)            /*!< CRC Error flag */
#define  SPI_SR_MODF                         ((uint32_t)0x00000020)            /*!< Mode fault */
#define  SPI_SR_OVR                          ((uint32_t)0x00000040)            /*!< Overrun flag */
#define  SPI_SR_BSY                          ((uint32_t)0x00000080)            /*!< Busy flag */
#define  SPI_SR_FRE                          ((uint32_t)0x00000100)            /*!< TI frame format error */
#define  SPI_SR_FRLVL                        ((uint32_t)0x00000600)            /*!< FIFO Reception Level */
#define  SPI_SR_FRLVL_0                      ((uint32_t)0x00000200)            /*!< Bit 0 */
#define  SPI_SR_FRLVL_1                      ((uint32_t)0x00000400)            /*!< Bit 1 */
#define  SPI_SR_FTLVL                        ((uint32_t)0x00001800)            /*!< FIFO Transmission Level */
#define  SPI_SR_FTLVL_0                      ((uint32_t)0x00000800)            /*!< Bit 0 */
#define  SPI_SR_FTLVL_1                      ((uint32_t)0x00001000)            /*!< Bit 1 */

/********************  Bit definition for SPI_DR register  ********************/
#define  SPI_DR_DR                           ((uint32_t)0x0000FFFF)            /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define  SPI_CRCPR_CRCPOLY                   ((uint32_t)0x0000FFFF)            /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define  SPI_RXCRCR_RXCRC                    ((uint32_t)0x0000FFFF)            /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define  SPI_TXCRCR_TXCRC                    ((uint32_t)0x0000FFFF)            /*!<Tx CRC Register         */

/******************************************************************************/
/*                                                                            */
/*                                    QUADSPI                                 */
/*                                                                            */
/******************************************************************************/
/*****************  Bit definition for QUADSPI_CR register  *******************/
#define  QUADSPI_CR_EN                           ((uint32_t)0x00000001)            /*!< Enable */
#define  QUADSPI_CR_ABORT                        ((uint32_t)0x00000002)            /*!< Abort request */
#define  QUADSPI_CR_DMAEN                        ((uint32_t)0x00000004)            /*!< DMA Enable */
#define  QUADSPI_CR_TCEN                         ((uint32_t)0x00000008)            /*!< Timeout Counter Enable */
#define  QUADSPI_CR_SSHIFT                       ((uint32_t)0x00000010)            /*!< Sample Shift */
#define  QUADSPI_CR_FTHRES                       ((uint32_t)0x00000F00)            /*!< FTHRES[3:0] FIFO Level */
#define  QUADSPI_CR_FTHRES_0                     ((uint32_t)0x00000100)            /*!< Bit 0 */
#define  QUADSPI_CR_FTHRES_1                     ((uint32_t)0x00000200)            /*!< Bit 1 */
#define  QUADSPI_CR_FTHRES_2                     ((uint32_t)0x00000400)            /*!< Bit 2 */
#define  QUADSPI_CR_FTHRES_3                     ((uint32_t)0x00000800)            /*!< Bit 3 */
#define  QUADSPI_CR_TEIE                         ((uint32_t)0x00010000)            /*!< Transfer Error Interrupt Enable */
#define  QUADSPI_CR_TCIE                         ((uint32_t)0x00020000)            /*!< Transfer Complete Interrupt Enable */
#define  QUADSPI_CR_FTIE                         ((uint32_t)0x00040000)            /*!< FIFO Threshold Interrupt Enable */
#define  QUADSPI_CR_SMIE                         ((uint32_t)0x00080000)            /*!< Status Match Interrupt Enable */
#define  QUADSPI_CR_TOIE                         ((uint32_t)0x00100000)            /*!< TimeOut Interrupt Enable */
#define  QUADSPI_CR_APMS                         ((uint32_t)0x00400000)            /*!< Automatic Polling Mode Stop */
#define  QUADSPI_CR_PMM                          ((uint32_t)0x00800000)            /*!< Polling Match Mode */
#define  QUADSPI_CR_PRESCALER                    ((uint32_t)0xFF000000)            /*!< PRESCALER[7:0] Clock prescaler */
#define  QUADSPI_CR_PRESCALER_0                  ((uint32_t)0x01000000)            /*!< Bit 0 */
#define  QUADSPI_CR_PRESCALER_1                  ((uint32_t)0x02000000)            /*!< Bit 1 */
#define  QUADSPI_CR_PRESCALER_2                  ((uint32_t)0x04000000)            /*!< Bit 2 */
#define  QUADSPI_CR_PRESCALER_3                  ((uint32_t)0x08000000)            /*!< Bit 3 */
#define  QUADSPI_CR_PRESCALER_4                  ((uint32_t)0x10000000)            /*!< Bit 4 */
#define  QUADSPI_CR_PRESCALER_5                  ((uint32_t)0x20000000)            /*!< Bit 5 */
#define  QUADSPI_CR_PRESCALER_6                  ((uint32_t)0x40000000)            /*!< Bit 6 */
#define  QUADSPI_CR_PRESCALER_7                  ((uint32_t)0x80000000)            /*!< Bit 7 */

/*****************  Bit definition for QUADSPI_DCR register  ******************/
#define  QUADSPI_DCR_CKMODE                      ((uint32_t)0x00000001)            /*!< Mode 0 / Mode 3 */
#define  QUADSPI_DCR_CSHT                        ((uint32_t)0x00000700)            /*!< CSHT[2:0]: ChipSelect High Time */
#define  QUADSPI_DCR_CSHT_0                      ((uint32_t)0x00000100)            /*!< Bit 0 */
#define  QUADSPI_DCR_CSHT_1                      ((uint32_t)0x00000200)            /*!< Bit 1 */
#define  QUADSPI_DCR_CSHT_2                      ((uint32_t)0x00000400)            /*!< Bit 2 */
#define  QUADSPI_DCR_FSIZE                       ((uint32_t)0x001F0000)            /*!< FSIZE[4:0]: Flash Size */
#define  QUADSPI_DCR_FSIZE_0                     ((uint32_t)0x00010000)            /*!< Bit 0 */
#define  QUADSPI_DCR_FSIZE_1                     ((uint32_t)0x00020000)            /*!< Bit 1 */
#define  QUADSPI_DCR_FSIZE_2                     ((uint32_t)0x00040000)            /*!< Bit 2 */
#define  QUADSPI_DCR_FSIZE_3                     ((uint32_t)0x00080000)            /*!< Bit 3 */
#define  QUADSPI_DCR_FSIZE_4                     ((uint32_t)0x00100000)            /*!< Bit 4 */

/******************  Bit definition for QUADSPI_SR register  *******************/
#define  QUADSPI_SR_TEF                          ((uint32_t)0x00000001)             /*!< Transfer Error Flag */
#define  QUADSPI_SR_TCF                          ((uint32_t)0x00000002)             /*!< Transfer Complete Flag */
#define  QUADSPI_SR_FTF                          ((uint32_t)0x00000004)             /*!< FIFO Threshlod Flag */
#define  QUADSPI_SR_SMF                          ((uint32_t)0x00000008)             /*!< Status Match Flag */
#define  QUADSPI_SR_TOF                          ((uint32_t)0x00000010)             /*!< Timeout Flag */
#define  QUADSPI_SR_BUSY                         ((uint32_t)0x00000020)             /*!< Busy */
#define  QUADSPI_SR_FLEVEL                       ((uint32_t)0x00001F00)             /*!< FIFO Threshlod Flag */
#define  QUADSPI_SR_FLEVEL_0                     ((uint32_t)0x00000100)             /*!< Bit 0 */
#define  QUADSPI_SR_FLEVEL_1                     ((uint32_t)0x00000200)             /*!< Bit 1 */
#define  QUADSPI_SR_FLEVEL_2                     ((uint32_t)0x00000400)             /*!< Bit 2 */
#define  QUADSPI_SR_FLEVEL_3                     ((uint32_t)0x00000800)             /*!< Bit 3 */
#define  QUADSPI_SR_FLEVEL_4                     ((uint32_t)0x00001000)             /*!< Bit 4 */

/******************  Bit definition for QUADSPI_FCR register  ******************/
#define  QUADSPI_FCR_CTEF                        ((uint32_t)0x00000001)             /*!< Clear Transfer Error Flag */
#define  QUADSPI_FCR_CTCF                        ((uint32_t)0x00000002)             /*!< Clear Transfer Complete Flag */
#define  QUADSPI_FCR_CSMF                        ((uint32_t)0x00000008)             /*!< Clear Status Match Flag */
#define  QUADSPI_FCR_CTOF                        ((uint32_t)0x00000010)             /*!< Clear Timeout Flag */

/******************  Bit definition for QUADSPI_DLR register  ******************/
#define  QUADSPI_DLR_DL                        ((uint32_t)0xFFFFFFFF)               /*!< DL[31:0]: Data Length */

/******************  Bit definition for QUADSPI_CCR register  ******************/
#define  QUADSPI_CCR_INSTRUCTION                  ((uint32_t)0x000000FF)            /*!< INSTRUCTION[7:0]: Instruction */
#define  QUADSPI_CCR_INSTRUCTION_0                ((uint32_t)0x00000001)            /*!< Bit 0 */
#define  QUADSPI_CCR_INSTRUCTION_1                ((uint32_t)0x00000002)            /*!< Bit 1 */
#define  QUADSPI_CCR_INSTRUCTION_2                ((uint32_t)0x00000004)            /*!< Bit 2 */
#define  QUADSPI_CCR_INSTRUCTION_3                ((uint32_t)0x00000008)            /*!< Bit 3 */
#define  QUADSPI_CCR_INSTRUCTION_4                ((uint32_t)0x00000010)            /*!< Bit 4 */
#define  QUADSPI_CCR_INSTRUCTION_5                ((uint32_t)0x00000020)            /*!< Bit 5 */
#define  QUADSPI_CCR_INSTRUCTION_6                ((uint32_t)0x00000040)            /*!< Bit 6 */
#define  QUADSPI_CCR_INSTRUCTION_7                ((uint32_t)0x00000080)            /*!< Bit 7 */
#define  QUADSPI_CCR_IMODE                        ((uint32_t)0x00000300)            /*!< IMODE[1:0]: Instruction Mode */
#define  QUADSPI_CCR_IMODE_0                      ((uint32_t)0x00000100)            /*!< Bit 0 */
#define  QUADSPI_CCR_IMODE_1                      ((uint32_t)0x00000200)            /*!< Bit 1 */
#define  QUADSPI_CCR_ADMODE                       ((uint32_t)0x00000C00)            /*!< ADMODE[1:0]: Address Mode */
#define  QUADSPI_CCR_ADMODE_0                     ((uint32_t)0x00000400)            /*!< Bit 0 */
#define  QUADSPI_CCR_ADMODE_1                     ((uint32_t)0x00000800)            /*!< Bit 1 */
#define  QUADSPI_CCR_ADSIZE                       ((uint32_t)0x00003000)            /*!< ADSIZE[1:0]: Address Size */
#define  QUADSPI_CCR_ADSIZE_0                     ((uint32_t)0x00001000)            /*!< Bit 0 */
#define  QUADSPI_CCR_ADSIZE_1                     ((uint32_t)0x00002000)            /*!< Bit 1 */
#define  QUADSPI_CCR_ABMODE                       ((uint32_t)0x0000C000)            /*!< ABMODE[1:0]: Alternate Bytes Mode */
#define  QUADSPI_CCR_ABMODE_0                     ((uint32_t)0x00004000)            /*!< Bit 0 */
#define  QUADSPI_CCR_ABMODE_1                     ((uint32_t)0x00008000)            /*!< Bit 1 */
#define  QUADSPI_CCR_ABSIZE                       ((uint32_t)0x00030000)            /*!< ABSIZE[1:0]: Instruction Mode */
#define  QUADSPI_CCR_ABSIZE_0                     ((uint32_t)0x00010000)            /*!< Bit 0 */
#define  QUADSPI_CCR_ABSIZE_1                     ((uint32_t)0x00020000)            /*!< Bit 1 */
#define  QUADSPI_CCR_DCYC                         ((uint32_t)0x007C0000)            /*!< DCYC[4:0]: Dummy Cycles */
#define  QUADSPI_CCR_DCYC_0                       ((uint32_t)0x00040000)            /*!< Bit 0 */
#define  QUADSPI_CCR_DCYC_1                       ((uint32_t)0x00080000)            /*!< Bit 1 */
#define  QUADSPI_CCR_DCYC_2                       ((uint32_t)0x00100000)            /*!< Bit 2 */
#define  QUADSPI_CCR_DCYC_3                       ((uint32_t)0x00200000)            /*!< Bit 3 */
#define  QUADSPI_CCR_DCYC_4                       ((uint32_t)0x00400000)            /*!< Bit 4 */
#define  QUADSPI_CCR_DMODE                        ((uint32_t)0x03000000)            /*!< DMODE[1:0]: Data Mode */
#define  QUADSPI_CCR_DMODE_0                      ((uint32_t)0x01000000)            /*!< Bit 0 */
#define  QUADSPI_CCR_DMODE_1                      ((uint32_t)0x02000000)            /*!< Bit 1 */
#define  QUADSPI_CCR_FMODE                        ((uint32_t)0x0C000000)            /*!< FMODE[1:0]: Functional Mode */
#define  QUADSPI_CCR_FMODE_0                      ((uint32_t)0x04000000)            /*!< Bit 0 */
#define  QUADSPI_CCR_FMODE_1                      ((uint32_t)0x08000000)            /*!< Bit 1 */
#define  QUADSPI_CCR_SIOO                         ((uint32_t)0x10000000)            /*!< SIOO: Send Instruction Only Once Mode */
#define  QUADSPI_CCR_DDRM                         ((uint32_t)0x80000000)            /*!< DDRM: Double Data Rate Mode */

/******************  Bit definition for QUADSPI_AR register  *******************/
#define  QUADSPI_AR_ADDRESS                       ((uint32_t)0xFFFFFFFF)            /*!< ADDRESS[31:0]: Address */

/******************  Bit definition for QUADSPI_ABR register  ******************/
#define  QUADSPI_ABR_ALTERNATE                    ((uint32_t)0xFFFFFFFF)            /*!< ALTERNATE[31:0]: Alternate Bytes */

/******************  Bit definition for QUADSPI_DR register  *******************/
#define  QUADSPI_DR_DATA                          ((uint32_t)0xFFFFFFFF)            /*!< DATA[31:0]: Data */

/******************  Bit definition for QUADSPI_PSMKR register  ****************/
#define  QUADSPI_PSMKR_MASK                       ((uint32_t)0xFFFFFFFF)            /*!< MASK[31:0]: Status Mask */

/******************  Bit definition for QUADSPI_PSMAR register  ****************/
#define  QUADSPI_PSMAR_MATCH                      ((uint32_t)0xFFFFFFFF)            /*!< MATCH[31:0]: Status Match */

/******************  Bit definition for QUADSPI_PIR register  *****************/
#define  QUADSPI_PIR_INTERVAL                     ((uint32_t)0x0000FFFF)            /*!< INTERVAL[15:0]: Polling Interval */

/******************  Bit definition for QUADSPI_LPTR register  *****************/
#define  QUADSPI_LPTR_TIMEOUT                     ((uint32_t)0x0000FFFF)            /*!< TIMEOUT[15:0]: Timeout period */

/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCFG_MEMRMP register  ***************/
#define SYSCFG_MEMRMP_MEM_MODE          ((uint32_t)0x00000007) /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_MEMRMP_MEM_MODE_0        ((uint32_t)0x00000001)
#define SYSCFG_MEMRMP_MEM_MODE_1        ((uint32_t)0x00000002)
#define SYSCFG_MEMRMP_MEM_MODE_2        ((uint32_t)0x00000004)

#define SYSCFG_MEMRMP_FB_MODE           ((uint32_t)0x00000100) /*!< Flash Bank mode selection */


/******************  Bit definition for SYSCFG_CFGR1 register  ******************/
#define SYSCFG_CFGR1_FWDIS                  ((uint32_t)0x00000001) /*!< FIREWALL access enable*/
#define SYSCFG_CFGR1_BOOSTEN                ((uint32_t)0x00000100) /*!< I/O analog switch voltage booster enable */
#define SYSCFG_CFGR1_I2C_PB6_FMP            ((uint32_t)0x00010000) /*!< I2C PB6 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB7_FMP            ((uint32_t)0x00020000) /*!< I2C PB7 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB8_FMP            ((uint32_t)0x00040000) /*!< I2C PB8 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB9_FMP            ((uint32_t)0x00080000) /*!< I2C PB9 Fast mode plus */
#define SYSCFG_CFGR1_I2C1_FMP               ((uint32_t)0x00100000) /*!< I2C1 Fast mode plus */
#define SYSCFG_CFGR1_I2C2_FMP               ((uint32_t)0x00200000) /*!< I2C2 Fast mode plus */
#define SYSCFG_CFGR1_I2C3_FMP               ((uint32_t)0x00400000) /*!< I2C3 Fast mode plus */
#define SYSCFG_CFGR1_FPU_IE_0               ((uint32_t)0x04000000) /*!<  Invalid operation Interrupt enable */
#define SYSCFG_CFGR1_FPU_IE_1               ((uint32_t)0x08000000) /*!<  Divide-by-zero Interrupt enable */
#define SYSCFG_CFGR1_FPU_IE_2               ((uint32_t)0x10000000) /*!<  Underflow Interrupt enable */
#define SYSCFG_CFGR1_FPU_IE_3               ((uint32_t)0x20000000) /*!<  Overflow Interrupt enable */
#define SYSCFG_CFGR1_FPU_IE_4               ((uint32_t)0x40000000) /*!<  Input denormal Interrupt enable */
#define SYSCFG_CFGR1_FPU_IE_5               ((uint32_t)0x80000000) /*!<  Inexact Interrupt enable (interrupt disabled at reset) */

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0            ((uint32_t)0x00000007) /*!<EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1            ((uint32_t)0x00000070) /*!<EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2            ((uint32_t)0x00000700) /*!<EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3            ((uint32_t)0x00007000) /*!<EXTI 3 configuration */
/**
  * @brief   EXTI0 configuration
  */
#define SYSCFG_EXTICR1_EXTI0_PA         ((uint32_t)0x00000000) /*!<PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB         ((uint32_t)0x00000001) /*!<PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC         ((uint32_t)0x00000002) /*!<PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD         ((uint32_t)0x00000003) /*!<PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE         ((uint32_t)0x00000004) /*!<PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PF         ((uint32_t)0x00000005) /*!<PF[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PG         ((uint32_t)0x00000006) /*!<PG[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PH         ((uint32_t)0x00000007) /*!<PH[0] pin */


/**
  * @brief   EXTI1 configuration
  */
#define SYSCFG_EXTICR1_EXTI1_PA         ((uint32_t)0x00000000) /*!<PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB         ((uint32_t)0x00000010) /*!<PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC         ((uint32_t)0x00000020) /*!<PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD         ((uint32_t)0x00000030) /*!<PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE         ((uint32_t)0x00000040) /*!<PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PF         ((uint32_t)0x00000050) /*!<PF[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PG         ((uint32_t)0x00000060) /*!<PG[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PH         ((uint32_t)0x00000070) /*!<PH[1] pin */

/**
  * @brief   EXTI2 configuration
  */
#define SYSCFG_EXTICR1_EXTI2_PA         ((uint32_t)0x00000000) /*!<PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB         ((uint32_t)0x00000100) /*!<PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC         ((uint32_t)0x00000200) /*!<PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD         ((uint32_t)0x00000300) /*!<PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE         ((uint32_t)0x00000400) /*!<PE[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PF         ((uint32_t)0x00000500) /*!<PF[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PG         ((uint32_t)0x00000600) /*!<PG[2] pin */


/**
  * @brief   EXTI3 configuration
  */
#define SYSCFG_EXTICR1_EXTI3_PA         ((uint32_t)0x00000000) /*!<PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB         ((uint32_t)0x00001000) /*!<PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC         ((uint32_t)0x00002000) /*!<PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD         ((uint32_t)0x00003000) /*!<PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE         ((uint32_t)0x00004000) /*!<PE[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PF         ((uint32_t)0x00005000) /*!<PF[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PG         ((uint32_t)0x00006000) /*!<PG[3] pin */


/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4            ((uint32_t)0x00000007) /*!<EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5            ((uint32_t)0x00000070) /*!<EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6            ((uint32_t)0x00000700) /*!<EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7            ((uint32_t)0x00007000) /*!<EXTI 7 configuration */
/**
  * @brief   EXTI4 configuration
  */
#define SYSCFG_EXTICR2_EXTI4_PA         ((uint32_t)0x00000000) /*!<PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB         ((uint32_t)0x00000001) /*!<PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC         ((uint32_t)0x00000002) /*!<PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD         ((uint32_t)0x00000003) /*!<PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE         ((uint32_t)0x00000004) /*!<PE[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PF         ((uint32_t)0x00000005) /*!<PF[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PG         ((uint32_t)0x00000006) /*!<PG[4] pin */

/**
  * @brief   EXTI5 configuration
  */
#define SYSCFG_EXTICR2_EXTI5_PA         ((uint32_t)0x00000000) /*!<PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB         ((uint32_t)0x00000010) /*!<PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC         ((uint32_t)0x00000020) /*!<PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD         ((uint32_t)0x00000030) /*!<PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE         ((uint32_t)0x00000040) /*!<PE[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PF         ((uint32_t)0x00000050) /*!<PF[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PG         ((uint32_t)0x00000060) /*!<PG[5] pin */

/**
  * @brief   EXTI6 configuration
  */
#define SYSCFG_EXTICR2_EXTI6_PA         ((uint32_t)0x00000000) /*!<PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB         ((uint32_t)0x00000100) /*!<PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC         ((uint32_t)0x00000200) /*!<PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD         ((uint32_t)0x00000300) /*!<PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE         ((uint32_t)0x00000400) /*!<PE[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PF         ((uint32_t)0x00000500) /*!<PF[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PG         ((uint32_t)0x00000600) /*!<PG[6] pin */

/**
  * @brief   EXTI7 configuration
  */
#define SYSCFG_EXTICR2_EXTI7_PA         ((uint32_t)0x00000000) /*!<PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB         ((uint32_t)0x00001000) /*!<PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC         ((uint32_t)0x00002000) /*!<PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD         ((uint32_t)0x00003000) /*!<PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE         ((uint32_t)0x00004000) /*!<PE[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PF         ((uint32_t)0x00005000) /*!<PF[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PG         ((uint32_t)0x00006000) /*!<PG[7] pin */


/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8            ((uint32_t)0x00000007) /*!<EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9            ((uint32_t)0x00000070) /*!<EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10           ((uint32_t)0x00000700) /*!<EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11           ((uint32_t)0x00007000) /*!<EXTI 11 configuration */

/**
  * @brief   EXTI8 configuration
  */
#define SYSCFG_EXTICR3_EXTI8_PA         ((uint32_t)0x00000000) /*!<PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB         ((uint32_t)0x00000001) /*!<PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC         ((uint32_t)0x00000002) /*!<PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD         ((uint32_t)0x00000003) /*!<PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE         ((uint32_t)0x00000004) /*!<PE[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PF         ((uint32_t)0x00000005) /*!<PF[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PG         ((uint32_t)0x00000006) /*!<PG[8] pin */

/**
  * @brief   EXTI9 configuration
  */
#define SYSCFG_EXTICR3_EXTI9_PA         ((uint32_t)0x00000000) /*!<PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB         ((uint32_t)0x00000010) /*!<PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC         ((uint32_t)0x00000020) /*!<PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD         ((uint32_t)0x00000030) /*!<PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE         ((uint32_t)0x00000040) /*!<PE[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PF         ((uint32_t)0x00000050) /*!<PF[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PG         ((uint32_t)0x00000060) /*!<PG[9] pin */

/**
  * @brief   EXTI10 configuration
  */
#define SYSCFG_EXTICR3_EXTI10_PA        ((uint32_t)0x00000000) /*!<PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB        ((uint32_t)0x00000100) /*!<PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC        ((uint32_t)0x00000200) /*!<PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD        ((uint32_t)0x00000300) /*!<PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE        ((uint32_t)0x00000400) /*!<PE[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PF        ((uint32_t)0x00000500) /*!<PF[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PG        ((uint32_t)0x00000600) /*!<PG[10] pin */

/**
  * @brief   EXTI11 configuration
  */
#define SYSCFG_EXTICR3_EXTI11_PA        ((uint32_t)0x00000000) /*!<PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB        ((uint32_t)0x00001000) /*!<PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC        ((uint32_t)0x00002000) /*!<PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD        ((uint32_t)0x00003000) /*!<PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE        ((uint32_t)0x00004000) /*!<PE[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PF        ((uint32_t)0x00005000) /*!<PF[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PG        ((uint32_t)0x00006000) /*!<PG[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  ***************/
#define SYSCFG_EXTICR4_EXTI12           ((uint32_t)0x00000007) /*!<EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13           ((uint32_t)0x00000070) /*!<EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14           ((uint32_t)0x00000700) /*!<EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15           ((uint32_t)0x00007000) /*!<EXTI 15 configuration */
/**
  * @brief   EXTI12 configuration
  */
#define SYSCFG_EXTICR4_EXTI12_PA        ((uint32_t)0x00000000) /*!<PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB        ((uint32_t)0x00000001) /*!<PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC        ((uint32_t)0x00000002) /*!<PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD        ((uint32_t)0x00000003) /*!<PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE        ((uint32_t)0x00000004) /*!<PE[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PF        ((uint32_t)0x00000005) /*!<PF[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PG        ((uint32_t)0x00000006) /*!<PG[12] pin */

/**
  * @brief   EXTI13 configuration
  */
#define SYSCFG_EXTICR4_EXTI13_PA        ((uint32_t)0x00000000) /*!<PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB        ((uint32_t)0x00000010) /*!<PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC        ((uint32_t)0x00000020) /*!<PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD        ((uint32_t)0x00000030) /*!<PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE        ((uint32_t)0x00000040) /*!<PE[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PF        ((uint32_t)0x00000050) /*!<PF[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PG        ((uint32_t)0x00000060) /*!<PG[13] pin */

/**
  * @brief   EXTI14 configuration
  */
#define SYSCFG_EXTICR4_EXTI14_PA        ((uint32_t)0x00000000) /*!<PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB        ((uint32_t)0x00000100) /*!<PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC        ((uint32_t)0x00000200) /*!<PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD        ((uint32_t)0x00000300) /*!<PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE        ((uint32_t)0x00000400) /*!<PE[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PF        ((uint32_t)0x00000500) /*!<PF[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PG        ((uint32_t)0x00000600) /*!<PG[14] pin */

/**
  * @brief   EXTI15 configuration
  */
#define SYSCFG_EXTICR4_EXTI15_PA        ((uint32_t)0x00000000) /*!<PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB        ((uint32_t)0x00001000) /*!<PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC        ((uint32_t)0x00002000) /*!<PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD        ((uint32_t)0x00003000) /*!<PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE        ((uint32_t)0x00004000) /*!<PE[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PF        ((uint32_t)0x00005000) /*!<PF[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PG        ((uint32_t)0x00006000) /*!<PG[15] pin */

/******************  Bit definition for SYSCFG_SCSR register  ****************/
#define SYSCFG_SCSR_SRAM2ER                 ((uint32_t)0x00000001) /*!< SRAM2 Erase Request */
#define SYSCFG_SCSR_SRAM2BSY                ((uint32_t)0x00000002) /*!< SRAM2 Erase Ongoing */

/******************  Bit definition for SYSCFG_CFGR2 register  ****************/
#define SYSCFG_CFGR2_CLL                 ((uint32_t)0x00000001) /*!< Core Lockup Lock */
#define SYSCFG_CFGR2_SPL                 ((uint32_t)0x00000002) /*!< SRAM Parity Lock*/
#define SYSCFG_CFGR2_PVDL                ((uint32_t)0x00000004) /*!<  PVD Lock */
#define SYSCFG_CFGR2_ECCL                ((uint32_t)0x00000008) /*!< ECC Lock*/
#define SYSCFG_CFGR2_SPF                 ((uint32_t)0x00000100) /*!< SRAM Parity Flag */

/******************  Bit definition for SYSCFG_SWPR register  ****************/
#define SYSCFG_SWPR_PAGE0          ((uint32_t)0x00000001) /*!< SRAM2 Write protection page 0 */
#define SYSCFG_SWPR_PAGE1          ((uint32_t)0x00000002) /*!< SRAM2 Write protection page 1 */
#define SYSCFG_SWPR_PAGE2          ((uint32_t)0x00000004) /*!< SRAM2 Write protection page 2 */
#define SYSCFG_SWPR_PAGE3          ((uint32_t)0x00000008) /*!< SRAM2 Write protection page 3 */
#define SYSCFG_SWPR_PAGE4          ((uint32_t)0x00000010) /*!< SRAM2 Write protection page 4 */
#define SYSCFG_SWPR_PAGE5          ((uint32_t)0x00000020) /*!< SRAM2 Write protection page 5 */
#define SYSCFG_SWPR_PAGE6          ((uint32_t)0x00000040) /*!< SRAM2 Write protection page 6 */
#define SYSCFG_SWPR_PAGE7          ((uint32_t)0x00000080) /*!< SRAM2 Write protection page 7 */
#define SYSCFG_SWPR_PAGE8          ((uint32_t)0x00000100) /*!< SRAM2 Write protection page 8 */
#define SYSCFG_SWPR_PAGE9          ((uint32_t)0x00000200) /*!< SRAM2 Write protection page 9 */
#define SYSCFG_SWPR_PAGE10         ((uint32_t)0x00000400) /*!< SRAM2 Write protection page 10*/
#define SYSCFG_SWPR_PAGE11         ((uint32_t)0x00000800) /*!< SRAM2 Write protection page 11*/
#define SYSCFG_SWPR_PAGE12         ((uint32_t)0x00001000) /*!< SRAM2 Write protection page 12*/
#define SYSCFG_SWPR_PAGE13         ((uint32_t)0x00002000) /*!< SRAM2 Write protection page 13*/
#define SYSCFG_SWPR_PAGE14         ((uint32_t)0x00004000) /*!< SRAM2 Write protection page 14*/
#define SYSCFG_SWPR_PAGE15         ((uint32_t)0x00008000) /*!< SRAM2 Write protection page 15*/
#define SYSCFG_SWPR_PAGE16         ((uint32_t)0x00010000) /*!< SRAM2 Write protection page 16*/
#define SYSCFG_SWPR_PAGE17         ((uint32_t)0x00020000) /*!< SRAM2 Write protection page 17*/
#define SYSCFG_SWPR_PAGE18         ((uint32_t)0x00040000) /*!< SRAM2 Write protection page 18*/
#define SYSCFG_SWPR_PAGE19         ((uint32_t)0x00080000) /*!< SRAM2 Write protection page 19*/
#define SYSCFG_SWPR_PAGE20         ((uint32_t)0x00100000) /*!< SRAM2 Write protection page 20*/
#define SYSCFG_SWPR_PAGE21         ((uint32_t)0x00200000) /*!< SRAM2 Write protection page 21*/
#define SYSCFG_SWPR_PAGE22         ((uint32_t)0x00400000) /*!< SRAM2 Write protection page 22*/
#define SYSCFG_SWPR_PAGE23         ((uint32_t)0x00800000) /*!< SRAM2 Write protection page 23*/
#define SYSCFG_SWPR_PAGE24         ((uint32_t)0x01000000) /*!< SRAM2 Write protection page 24*/
#define SYSCFG_SWPR_PAGE25         ((uint32_t)0x02000000) /*!< SRAM2 Write protection page 25*/
#define SYSCFG_SWPR_PAGE26         ((uint32_t)0x04000000) /*!< SRAM2 Write protection page 26*/
#define SYSCFG_SWPR_PAGE27         ((uint32_t)0x08000000) /*!< SRAM2 Write protection page 27*/
#define SYSCFG_SWPR_PAGE28         ((uint32_t)0x10000000) /*!< SRAM2 Write protection page 28*/
#define SYSCFG_SWPR_PAGE29         ((uint32_t)0x20000000) /*!< SRAM2 Write protection page 29*/
#define SYSCFG_SWPR_PAGE30         ((uint32_t)0x40000000) /*!< SRAM2 Write protection page 30*/
#define SYSCFG_SWPR_PAGE31         ((uint32_t)0x80000000) /*!< SRAM2 Write protection page 31*/

/******************  Bit definition for SYSCFG_SKR register  ****************/
#define SYSCFG_SKR_KEY             ((uint32_t)0x000000FF) /*!<  SRAM2 write protection key for software erase  */




/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define  TIM_CR1_CEN               ((uint32_t)0x00000001)            /*!<Counter enable */
#define  TIM_CR1_UDIS              ((uint32_t)0x00000002)            /*!<Update disable */
#define  TIM_CR1_URS               ((uint32_t)0x00000004)            /*!<Update request source */
#define  TIM_CR1_OPM               ((uint32_t)0x00000008)            /*!<One pulse mode */
#define  TIM_CR1_DIR               ((uint32_t)0x00000010)            /*!<Direction */

#define  TIM_CR1_CMS               ((uint32_t)0x00000060)            /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CR1_CMS_0             ((uint32_t)0x00000020)            /*!<Bit 0 */
#define  TIM_CR1_CMS_1             ((uint32_t)0x00000040)            /*!<Bit 1 */

#define  TIM_CR1_ARPE              ((uint32_t)0x00000080)            /*!<Auto-reload preload enable */

#define  TIM_CR1_CKD               ((uint32_t)0x00000300)            /*!<CKD[1:0] bits (clock division) */
#define  TIM_CR1_CKD_0             ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_CR1_CKD_1             ((uint32_t)0x00000200)            /*!<Bit 1 */

#define  TIM_CR1_UIFREMAP          ((uint32_t)0x00000800)            /*!<Update interrupt flag remap */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define  TIM_CR2_CCPC              ((uint32_t)0x00000001)            /*!<Capture/Compare Preloaded Control */
#define  TIM_CR2_CCUS              ((uint32_t)0x00000004)            /*!<Capture/Compare Control Update Selection */
#define  TIM_CR2_CCDS              ((uint32_t)0x00000008)            /*!<Capture/Compare DMA Selection */

#define  TIM_CR2_MMS               ((uint32_t)0x00000070)            /*!<MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS_0             ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CR2_MMS_1             ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CR2_MMS_2             ((uint32_t)0x00000040)            /*!<Bit 2 */

#define  TIM_CR2_TI1S              ((uint32_t)0x00000080)            /*!<TI1 Selection */
#define  TIM_CR2_OIS1              ((uint32_t)0x00000100)            /*!<Output Idle state 1 (OC1 output) */
#define  TIM_CR2_OIS1N             ((uint32_t)0x00000200)            /*!<Output Idle state 1 (OC1N output) */
#define  TIM_CR2_OIS2              ((uint32_t)0x00000400)            /*!<Output Idle state 2 (OC2 output) */
#define  TIM_CR2_OIS2N             ((uint32_t)0x00000800)            /*!<Output Idle state 2 (OC2N output) */
#define  TIM_CR2_OIS3              ((uint32_t)0x00001000)            /*!<Output Idle state 3 (OC3 output) */
#define  TIM_CR2_OIS3N             ((uint32_t)0x00002000)            /*!<Output Idle state 3 (OC3N output) */
#define  TIM_CR2_OIS4              ((uint32_t)0x00004000)            /*!<Output Idle state 4 (OC4 output) */
#define  TIM_CR2_OIS5              ((uint32_t)0x00010000)            /*!<Output Idle state 5 (OC5 output) */
#define  TIM_CR2_OIS6              ((uint32_t)0x00040000)            /*!<Output Idle state 6 (OC6 output) */

#define  TIM_CR2_MMS2              ((uint32_t)0x00F00000)            /*!<MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS2_0            ((uint32_t)0x00100000)            /*!<Bit 0 */
#define  TIM_CR2_MMS2_1            ((uint32_t)0x00200000)            /*!<Bit 1 */
#define  TIM_CR2_MMS2_2            ((uint32_t)0x00400000)            /*!<Bit 2 */
#define  TIM_CR2_MMS2_3            ((uint32_t)0x00800000)            /*!<Bit 2 */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define  TIM_SMCR_SMS              ((uint32_t)0x00010007)            /*!<SMS[2:0] bits (Slave mode selection) */
#define  TIM_SMCR_SMS_0            ((uint32_t)0x00000001)            /*!<Bit 0 */
#define  TIM_SMCR_SMS_1            ((uint32_t)0x00000002)            /*!<Bit 1 */
#define  TIM_SMCR_SMS_2            ((uint32_t)0x00000004)            /*!<Bit 2 */
#define  TIM_SMCR_SMS_3            ((uint32_t)0x00010000)            /*!<Bit 3 */

#define  TIM_SMCR_OCCS             ((uint32_t)0x00000008)            /*!< OCREF clear selection */

#define  TIM_SMCR_TS               ((uint32_t)0x00000070)            /*!<TS[2:0] bits (Trigger selection) */
#define  TIM_SMCR_TS_0             ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_SMCR_TS_1             ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_SMCR_TS_2             ((uint32_t)0x00000040)            /*!<Bit 2 */

#define  TIM_SMCR_MSM              ((uint32_t)0x00000080)            /*!<Master/slave mode */

#define  TIM_SMCR_ETF              ((uint32_t)0x00000F00)            /*!<ETF[3:0] bits (External trigger filter) */
#define  TIM_SMCR_ETF_0            ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_SMCR_ETF_1            ((uint32_t)0x00000200)            /*!<Bit 1 */
#define  TIM_SMCR_ETF_2            ((uint32_t)0x00000400)            /*!<Bit 2 */
#define  TIM_SMCR_ETF_3            ((uint32_t)0x00000800)            /*!<Bit 3 */

#define  TIM_SMCR_ETPS             ((uint32_t)0x00003000)            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_SMCR_ETPS_0           ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_SMCR_ETPS_1           ((uint32_t)0x00002000)            /*!<Bit 1 */

#define  TIM_SMCR_ECE              ((uint32_t)0x00004000)            /*!<External clock enable */
#define  TIM_SMCR_ETP              ((uint32_t)0x00008000)            /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define  TIM_DIER_UIE              ((uint32_t)0x00000001)            /*!<Update interrupt enable */
#define  TIM_DIER_CC1IE            ((uint32_t)0x00000002)            /*!<Capture/Compare 1 interrupt enable */
#define  TIM_DIER_CC2IE            ((uint32_t)0x00000004)            /*!<Capture/Compare 2 interrupt enable */
#define  TIM_DIER_CC3IE            ((uint32_t)0x00000008)            /*!<Capture/Compare 3 interrupt enable */
#define  TIM_DIER_CC4IE            ((uint32_t)0x00000010)            /*!<Capture/Compare 4 interrupt enable */
#define  TIM_DIER_COMIE            ((uint32_t)0x00000020)            /*!<COM interrupt enable */
#define  TIM_DIER_TIE              ((uint32_t)0x00000040)            /*!<Trigger interrupt enable */
#define  TIM_DIER_BIE              ((uint32_t)0x00000080)            /*!<Break interrupt enable */
#define  TIM_DIER_UDE              ((uint32_t)0x00000100)            /*!<Update DMA request enable */
#define  TIM_DIER_CC1DE            ((uint32_t)0x00000200)            /*!<Capture/Compare 1 DMA request enable */
#define  TIM_DIER_CC2DE            ((uint32_t)0x00000400)            /*!<Capture/Compare 2 DMA request enable */
#define  TIM_DIER_CC3DE            ((uint32_t)0x00000800)            /*!<Capture/Compare 3 DMA request enable */
#define  TIM_DIER_CC4DE            ((uint32_t)0x00001000)            /*!<Capture/Compare 4 DMA request enable */
#define  TIM_DIER_COMDE            ((uint32_t)0x00002000)            /*!<COM DMA request enable */
#define  TIM_DIER_TDE              ((uint32_t)0x00004000)            /*!<Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  ********************/
#define  TIM_SR_UIF                ((uint32_t)0x00000001)            /*!<Update interrupt Flag */
#define  TIM_SR_CC1IF              ((uint32_t)0x00000002)            /*!<Capture/Compare 1 interrupt Flag */
#define  TIM_SR_CC2IF              ((uint32_t)0x00000004)            /*!<Capture/Compare 2 interrupt Flag */
#define  TIM_SR_CC3IF              ((uint32_t)0x00000008)            /*!<Capture/Compare 3 interrupt Flag */
#define  TIM_SR_CC4IF              ((uint32_t)0x00000010)            /*!<Capture/Compare 4 interrupt Flag */
#define  TIM_SR_COMIF              ((uint32_t)0x00000020)            /*!<COM interrupt Flag */
#define  TIM_SR_TIF                ((uint32_t)0x00000040)            /*!<Trigger interrupt Flag */
#define  TIM_SR_BIF                ((uint32_t)0x00000080)            /*!<Break interrupt Flag */
#define  TIM_SR_B2IF               ((uint32_t)0x00000100)            /*!<Break 2 interrupt Flag */
#define  TIM_SR_CC1OF              ((uint32_t)0x00000200)            /*!<Capture/Compare 1 Overcapture Flag */
#define  TIM_SR_CC2OF              ((uint32_t)0x00000400)            /*!<Capture/Compare 2 Overcapture Flag */
#define  TIM_SR_CC3OF              ((uint32_t)0x00000800)            /*!<Capture/Compare 3 Overcapture Flag */
#define  TIM_SR_CC4OF              ((uint32_t)0x00001000)            /*!<Capture/Compare 4 Overcapture Flag */
#define  TIM_SR_SBIF               ((uint32_t)0x00002000)            /*!<System Break interrupt Flag */
#define  TIM_SR_CC5IF              ((uint32_t)0x00010000)            /*!<Capture/Compare 5 interrupt Flag */
#define  TIM_SR_CC6IF              ((uint32_t)0x00020000)            /*!<Capture/Compare 6 interrupt Flag */


/*******************  Bit definition for TIM_EGR register  ********************/
#define  TIM_EGR_UG                ((uint32_t)0x00000001)            /*!<Update Generation */
#define  TIM_EGR_CC1G              ((uint32_t)0x00000002)            /*!<Capture/Compare 1 Generation */
#define  TIM_EGR_CC2G              ((uint32_t)0x00000004)            /*!<Capture/Compare 2 Generation */
#define  TIM_EGR_CC3G              ((uint32_t)0x00000008)            /*!<Capture/Compare 3 Generation */
#define  TIM_EGR_CC4G              ((uint32_t)0x00000010)            /*!<Capture/Compare 4 Generation */
#define  TIM_EGR_COMG              ((uint32_t)0x00000020)            /*!<Capture/Compare Control Update Generation */
#define  TIM_EGR_TG                ((uint32_t)0x00000040)            /*!<Trigger Generation */
#define  TIM_EGR_BG                ((uint32_t)0x00000080)            /*!<Break Generation */
#define  TIM_EGR_B2G               ((uint32_t)0x00000100)            /*!<Break 2 Generation */


/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  TIM_CCMR1_CC1S            ((uint32_t)0x00000003)            /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CCMR1_CC1S_0          ((uint32_t)0x00000001)            /*!<Bit 0 */
#define  TIM_CCMR1_CC1S_1          ((uint32_t)0x00000002)            /*!<Bit 1 */

#define  TIM_CCMR1_OC1FE           ((uint32_t)0x00000004)            /*!<Output Compare 1 Fast enable */
#define  TIM_CCMR1_OC1PE           ((uint32_t)0x00000008)            /*!<Output Compare 1 Preload enable */

#define  TIM_CCMR1_OC1M            ((uint32_t)0x00010070)            /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
#define  TIM_CCMR1_OC1M_0          ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CCMR1_OC1M_1          ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CCMR1_OC1M_2          ((uint32_t)0x00000040)            /*!<Bit 2 */
#define  TIM_CCMR1_OC1M_3          ((uint32_t)0x00010000)            /*!<Bit 3 */

#define  TIM_CCMR1_OC1CE           ((uint32_t)0x00000080)            /*!<Output Compare 1 Clear Enable */

#define  TIM_CCMR1_CC2S            ((uint32_t)0x00000300)            /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CCMR1_CC2S_0          ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_CCMR1_CC2S_1          ((uint32_t)0x00000200)            /*!<Bit 1 */

#define  TIM_CCMR1_OC2FE           ((uint32_t)0x00000400)            /*!<Output Compare 2 Fast enable */
#define  TIM_CCMR1_OC2PE           ((uint32_t)0x00000800)            /*!<Output Compare 2 Preload enable */

#define  TIM_CCMR1_OC2M            ((uint32_t)0x01007000)            /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define  TIM_CCMR1_OC2M_0          ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_CCMR1_OC2M_1          ((uint32_t)0x00002000)            /*!<Bit 1 */
#define  TIM_CCMR1_OC2M_2          ((uint32_t)0x00004000)            /*!<Bit 2 */
#define  TIM_CCMR1_OC2M_3          ((uint32_t)0x01000000)            /*!<Bit 3 */

#define  TIM_CCMR1_OC2CE           ((uint32_t)0x00008000)            /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/
#define  TIM_CCMR1_IC1PSC          ((uint32_t)0x0000000C)            /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_CCMR1_IC1PSC_0        ((uint32_t)0x00000004)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1PSC_1        ((uint32_t)0x00000008)            /*!<Bit 1 */

#define  TIM_CCMR1_IC1F            ((uint32_t)0x000000F0)            /*!<IC1F[3:0] bits (Input Capture 1 Filter) */
#define  TIM_CCMR1_IC1F_0          ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1F_1          ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CCMR1_IC1F_2          ((uint32_t)0x00000040)            /*!<Bit 2 */
#define  TIM_CCMR1_IC1F_3          ((uint32_t)0x00000080)            /*!<Bit 3 */

#define  TIM_CCMR1_IC2PSC          ((uint32_t)0x00000C00)            /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define  TIM_CCMR1_IC2PSC_0        ((uint32_t)0x00000400)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2PSC_1        ((uint32_t)0x00000800)            /*!<Bit 1 */

#define  TIM_CCMR1_IC2F            ((uint32_t)0x0000F000)            /*!<IC2F[3:0] bits (Input Capture 2 Filter) */
#define  TIM_CCMR1_IC2F_0          ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2F_1          ((uint32_t)0x00002000)            /*!<Bit 1 */
#define  TIM_CCMR1_IC2F_2          ((uint32_t)0x00004000)            /*!<Bit 2 */
#define  TIM_CCMR1_IC2F_3          ((uint32_t)0x00008000)            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  TIM_CCMR2_CC3S            ((uint32_t)0x00000003)            /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define  TIM_CCMR2_CC3S_0          ((uint32_t)0x00000001)            /*!<Bit 0 */
#define  TIM_CCMR2_CC3S_1          ((uint32_t)0x00000002)            /*!<Bit 1 */

#define  TIM_CCMR2_OC3FE           ((uint32_t)0x00000004)            /*!<Output Compare 3 Fast enable */
#define  TIM_CCMR2_OC3PE           ((uint32_t)0x00000008)            /*!<Output Compare 3 Preload enable */

#define  TIM_CCMR2_OC3M            ((uint32_t)0x00010070)            /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_CCMR2_OC3M_0          ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CCMR2_OC3M_1          ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CCMR2_OC3M_2          ((uint32_t)0x00000040)            /*!<Bit 2 */
#define  TIM_CCMR2_OC3M_3          ((uint32_t)0x00010000)            /*!<Bit 3 */

#define  TIM_CCMR2_OC3CE           ((uint32_t)0x00000080)            /*!<Output Compare 3 Clear Enable */

#define  TIM_CCMR2_CC4S            ((uint32_t)0x00000300)            /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CCMR2_CC4S_0          ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_CCMR2_CC4S_1          ((uint32_t)0x00000200)            /*!<Bit 1 */

#define  TIM_CCMR2_OC4FE           ((uint32_t)0x00000400)            /*!<Output Compare 4 Fast enable */
#define  TIM_CCMR2_OC4PE           ((uint32_t)0x00000800)            /*!<Output Compare 4 Preload enable */

#define  TIM_CCMR2_OC4M            ((uint32_t)0x01007000)            /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR2_OC4M_0          ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_CCMR2_OC4M_1          ((uint32_t)0x00002000)            /*!<Bit 1 */
#define  TIM_CCMR2_OC4M_2          ((uint32_t)0x00004000)            /*!<Bit 2 */
#define  TIM_CCMR2_OC4M_3          ((uint32_t)0x01000000)            /*!<Bit 3 */

#define  TIM_CCMR2_OC4CE           ((uint32_t)0x00008000)            /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/
#define  TIM_CCMR2_IC3PSC          ((uint32_t)0x0000000C)            /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_CCMR2_IC3PSC_0        ((uint32_t)0x00000004)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3PSC_1        ((uint32_t)0x00000008)            /*!<Bit 1 */

#define  TIM_CCMR2_IC3F            ((uint32_t)0x000000F0)            /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_CCMR2_IC3F_0          ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3F_1          ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CCMR2_IC3F_2          ((uint32_t)0x00000040)            /*!<Bit 2 */
#define  TIM_CCMR2_IC3F_3          ((uint32_t)0x00000080)            /*!<Bit 3 */

#define  TIM_CCMR2_IC4PSC          ((uint32_t)0x00000C00)            /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_CCMR2_IC4PSC_0        ((uint32_t)0x00000400)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4PSC_1        ((uint32_t)0x00000800)            /*!<Bit 1 */

#define  TIM_CCMR2_IC4F            ((uint32_t)0x0000F000)            /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_CCMR2_IC4F_0          ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4F_1          ((uint32_t)0x00002000)            /*!<Bit 1 */
#define  TIM_CCMR2_IC4F_2          ((uint32_t)0x00004000)            /*!<Bit 2 */
#define  TIM_CCMR2_IC4F_3          ((uint32_t)0x00008000)            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR3 register  *******************/
#define  TIM_CCMR3_OC5FE           ((uint32_t)0x00000004)            /*!<Output Compare 5 Fast enable */
#define  TIM_CCMR3_OC5PE           ((uint32_t)0x00000008)            /*!<Output Compare 5 Preload enable */

#define  TIM_CCMR3_OC5M            ((uint32_t)0x00010070)            /*!<OC5M[3:0] bits (Output Compare 5 Mode) */
#define  TIM_CCMR3_OC5M_0          ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CCMR3_OC5M_1          ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CCMR3_OC5M_2          ((uint32_t)0x00000040)            /*!<Bit 2 */
#define  TIM_CCMR3_OC5M_3          ((uint32_t)0x00010000)            /*!<Bit 3 */

#define  TIM_CCMR3_OC5CE           ((uint32_t)0x00000080)            /*!<Output Compare 5 Clear Enable */

#define  TIM_CCMR3_OC6FE           ((uint32_t)0x00000400)            /*!<Output Compare 6 Fast enable */
#define  TIM_CCMR3_OC6PE           ((uint32_t)0x00000800)            /*!<Output Compare 6 Preload enable */

#define  TIM_CCMR3_OC6M            ((uint32_t)0x01007000)            /*!<OC6M[3:0] bits (Output Compare 6 Mode) */
#define  TIM_CCMR3_OC6M_0          ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_CCMR3_OC6M_1          ((uint32_t)0x00002000)            /*!<Bit 1 */
#define  TIM_CCMR3_OC6M_2          ((uint32_t)0x00004000)            /*!<Bit 2 */
#define  TIM_CCMR3_OC6M_3          ((uint32_t)0x01000000)            /*!<Bit 3 */

#define  TIM_CCMR3_OC6CE           ((uint32_t)0x00008000)            /*!<Output Compare 6 Clear Enable */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CCER_CC1E             ((uint32_t)0x00000001)            /*!<Capture/Compare 1 output enable */
#define  TIM_CCER_CC1P             ((uint32_t)0x00000002)            /*!<Capture/Compare 1 output Polarity */
#define  TIM_CCER_CC1NE            ((uint32_t)0x00000004)            /*!<Capture/Compare 1 Complementary output enable */
#define  TIM_CCER_CC1NP            ((uint32_t)0x00000008)            /*!<Capture/Compare 1 Complementary output Polarity */
#define  TIM_CCER_CC2E             ((uint32_t)0x00000010)            /*!<Capture/Compare 2 output enable */
#define  TIM_CCER_CC2P             ((uint32_t)0x00000020)            /*!<Capture/Compare 2 output Polarity */
#define  TIM_CCER_CC2NE            ((uint32_t)0x00000040)            /*!<Capture/Compare 2 Complementary output enable */
#define  TIM_CCER_CC2NP            ((uint32_t)0x00000080)            /*!<Capture/Compare 2 Complementary output Polarity */
#define  TIM_CCER_CC3E             ((uint32_t)0x00000100)            /*!<Capture/Compare 3 output enable */
#define  TIM_CCER_CC3P             ((uint32_t)0x00000200)            /*!<Capture/Compare 3 output Polarity */
#define  TIM_CCER_CC3NE            ((uint32_t)0x00000400)            /*!<Capture/Compare 3 Complementary output enable */
#define  TIM_CCER_CC3NP            ((uint32_t)0x00000800)            /*!<Capture/Compare 3 Complementary output Polarity */
#define  TIM_CCER_CC4E             ((uint32_t)0x00001000)            /*!<Capture/Compare 4 output enable */
#define  TIM_CCER_CC4P             ((uint32_t)0x00002000)            /*!<Capture/Compare 4 output Polarity */
#define  TIM_CCER_CC4NP            ((uint32_t)0x00008000)            /*!<Capture/Compare 4 Complementary output Polarity */
#define  TIM_CCER_CC5E             ((uint32_t)0x00010000)            /*!<Capture/Compare 5 output enable */
#define  TIM_CCER_CC5P             ((uint32_t)0x00020000)            /*!<Capture/Compare 5 output Polarity */
#define  TIM_CCER_CC6E             ((uint32_t)0x00100000)            /*!<Capture/Compare 6 output enable */
#define  TIM_CCER_CC6P             ((uint32_t)0x00200000)            /*!<Capture/Compare 6 output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT_CNT               ((uint32_t)0xFFFFFFFF)            /*!<Counter Value */
#define  TIM_CNT_UIFCPY            ((uint32_t)0x80000000)            /*!<Update interrupt flag copy (if UIFREMAP=1) */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC_PSC               ((uint32_t)0x0000FFFF)            /*!<Prescaler Value */

/*******************  Bit definition for TIM_ARR register  ********************/
#define  TIM_ARR_ARR               ((uint32_t)0xFFFFFFFF)            /*!<Actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define  TIM_RCR_REP               ((uint32_t)0x0000FFFF)            /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define  TIM_CCR1_CCR1             ((uint32_t)0x0000FFFF)            /*!<Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define  TIM_CCR2_CCR2             ((uint32_t)0x0000FFFF)            /*!<Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define  TIM_CCR3_CCR3             ((uint32_t)0x0000FFFF)            /*!<Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define  TIM_CCR4_CCR4             ((uint32_t)0x0000FFFF)            /*!<Capture/Compare 4 Value */

/*******************  Bit definition for TIM_CCR5 register  *******************/
#define  TIM_CCR5_CCR5             ((uint32_t)0xFFFFFFFF)            /*!<Capture/Compare 5 Value */
#define  TIM_CCR5_GC5C1            ((uint32_t)0x20000000)            /*!<Group Channel 5 and Channel 1 */
#define  TIM_CCR5_GC5C2            ((uint32_t)0x40000000)            /*!<Group Channel 5 and Channel 2 */
#define  TIM_CCR5_GC5C3            ((uint32_t)0x80000000)            /*!<Group Channel 5 and Channel 3 */

/*******************  Bit definition for TIM_CCR6 register  *******************/
#define  TIM_CCR6_CCR6             ((uint32_t)0x0000FFFF)            /*!<Capture/Compare 6 Value */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_BDTR_DTG              ((uint32_t)0x000000FF)            /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_BDTR_DTG_0            ((uint32_t)0x00000001)            /*!<Bit 0 */
#define  TIM_BDTR_DTG_1            ((uint32_t)0x00000002)            /*!<Bit 1 */
#define  TIM_BDTR_DTG_2            ((uint32_t)0x00000004)            /*!<Bit 2 */
#define  TIM_BDTR_DTG_3            ((uint32_t)0x00000008)            /*!<Bit 3 */
#define  TIM_BDTR_DTG_4            ((uint32_t)0x00000010)            /*!<Bit 4 */
#define  TIM_BDTR_DTG_5            ((uint32_t)0x00000020)            /*!<Bit 5 */
#define  TIM_BDTR_DTG_6            ((uint32_t)0x00000040)            /*!<Bit 6 */
#define  TIM_BDTR_DTG_7            ((uint32_t)0x00000080)            /*!<Bit 7 */

#define  TIM_BDTR_LOCK             ((uint32_t)0x00000300)            /*!<LOCK[1:0] bits (Lock Configuration) */
#define  TIM_BDTR_LOCK_0           ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_BDTR_LOCK_1           ((uint32_t)0x00000200)            /*!<Bit 1 */

#define  TIM_BDTR_OSSI             ((uint32_t)0x00000400)            /*!<Off-State Selection for Idle mode */
#define  TIM_BDTR_OSSR             ((uint32_t)0x00000800)            /*!<Off-State Selection for Run mode */
#define  TIM_BDTR_BKE              ((uint32_t)0x00001000)            /*!<Break enable for Break 1 */
#define  TIM_BDTR_BKP              ((uint32_t)0x00002000)            /*!<Break Polarity for Break 1 */
#define  TIM_BDTR_AOE              ((uint32_t)0x00004000)            /*!<Automatic Output enable */
#define  TIM_BDTR_MOE              ((uint32_t)0x00008000)            /*!<Main Output enable */

#define  TIM_BDTR_BKF              ((uint32_t)0x000F0000)            /*!<Break Filter for Break 1 */
#define  TIM_BDTR_BK2F             ((uint32_t)0x00F00000)            /*!<Break Filter for Break 2 */

#define  TIM_BDTR_BK2E             ((uint32_t)0x01000000)            /*!<Break enable for Break 2 */
#define  TIM_BDTR_BK2P             ((uint32_t)0x02000000)            /*!<Break Polarity for Break 2 */

/*******************  Bit definition for TIM_DCR register  ********************/
#define  TIM_DCR_DBA               ((uint32_t)0x0000001F)            /*!<DBA[4:0] bits (DMA Base Address) */
#define  TIM_DCR_DBA_0             ((uint32_t)0x00000001)            /*!<Bit 0 */
#define  TIM_DCR_DBA_1             ((uint32_t)0x00000002)            /*!<Bit 1 */
#define  TIM_DCR_DBA_2             ((uint32_t)0x00000004)            /*!<Bit 2 */
#define  TIM_DCR_DBA_3             ((uint32_t)0x00000008)            /*!<Bit 3 */
#define  TIM_DCR_DBA_4             ((uint32_t)0x00000010)            /*!<Bit 4 */

#define  TIM_DCR_DBL               ((uint32_t)0x00001F00)            /*!<DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DCR_DBL_0             ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_DCR_DBL_1             ((uint32_t)0x00000200)            /*!<Bit 1 */
#define  TIM_DCR_DBL_2             ((uint32_t)0x00000400)            /*!<Bit 2 */
#define  TIM_DCR_DBL_3             ((uint32_t)0x00000800)            /*!<Bit 3 */
#define  TIM_DCR_DBL_4             ((uint32_t)0x00001000)            /*!<Bit 4 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define  TIM_DMAR_DMAB             ((uint32_t)0x0000FFFF)            /*!<DMA register for burst accesses */

/*******************  Bit definition for TIM1_OR1 register  *******************/
#define TIM1_OR1_ETR_ADC1_RMP      ((uint32_t)0x00000003)            /*!<ETR_ADC1_RMP[1:0] bits (TIM1 ETR remap on ADC1) */
#define TIM1_OR1_ETR_ADC1_RMP_0    ((uint32_t)0x00000001)            /*!<Bit 0 */
#define TIM1_OR1_ETR_ADC1_RMP_1    ((uint32_t)0x00000002)            /*!<Bit 1 */

#define TIM1_OR1_ETR_ADC3_RMP      ((uint32_t)0x0000000C)            /*!<ETR_ADC3_RMP[1:0] bits (TIM1 ETR remap on ADC3) */
#define TIM1_OR1_ETR_ADC3_RMP_0    ((uint32_t)0x00000004)            /*!<Bit 0 */
#define TIM1_OR1_ETR_ADC3_RMP_1    ((uint32_t)0x00000008)            /*!<Bit 1 */

#define TIM1_OR1_TI1_RMP           ((uint32_t)0x00000010)            /*!<TIM1 Input Capture 1 remap */

/*******************  Bit definition for TIM1_OR2 register  *******************/
#define TIM1_OR2_BKINE             ((uint32_t)0x00000001)            /*!<BRK BKIN input enable */
#define TIM1_OR2_BKCMP1E           ((uint32_t)0x00000002)            /*!<BRK COMP1 enable */
#define TIM1_OR2_BKCMP2E           ((uint32_t)0x00000004)            /*!<BRK COMP2 enable */
#define TIM1_OR2_BKDFBK0E          ((uint32_t)0x00000100)            /*!<BRK DFSDM_BREAK[0] enable */
#define TIM1_OR2_BKINP             ((uint32_t)0x00000200)            /*!<BRK BKIN input polarity */
#define TIM1_OR2_BKCMP1P           ((uint32_t)0x00000400)            /*!<BRK COMP1 input polarity */
#define TIM1_OR2_BKCMP2P           ((uint32_t)0x00000800)            /*!<BRK COMP2 input polarity */

#define TIM1_OR2_ETRSEL            ((uint32_t)0x0001C000)            /*!<ETRSEL[2:0] bits (TIM1 ETR source selection) */
#define TIM1_OR2_ETRSEL_0          ((uint32_t)0x00004000)            /*!<Bit 0 */
#define TIM1_OR2_ETRSEL_1          ((uint32_t)0x00008000)            /*!<Bit 1 */
#define TIM1_OR2_ETRSEL_2          ((uint32_t)0x00010000)            /*!<Bit 2 */

/*******************  Bit definition for TIM1_OR3 register  *******************/
#define TIM1_OR3_BK2INE            ((uint32_t)0x00000001)            /*!<BRK2 BKIN2 input enable */
#define TIM1_OR3_BK2CMP1E          ((uint32_t)0x00000002)            /*!<BRK2 COMP1 enable */
#define TIM1_OR3_BK2CMP2E          ((uint32_t)0x00000004)            /*!<BRK2 COMP2 enable */
#define TIM1_OR3_BK2DFBK1E         ((uint32_t)0x00000100)            /*!<BRK2 DFSDM_BREAK[1] enable */
#define TIM1_OR3_BK2INP            ((uint32_t)0x00000200)            /*!<BRK2 BKIN2 input polarity */
#define TIM1_OR3_BK2CMP1P          ((uint32_t)0x00000400)            /*!<BRK2 COMP1 input polarity */
#define TIM1_OR3_BK2CMP2P          ((uint32_t)0x00000800)            /*!<BRK2 COMP2 input polarity */

/*******************  Bit definition for TIM8_OR1 register  *******************/
#define TIM8_OR1_ETR_ADC2_RMP      ((uint32_t)0x00000003)            /*!<ETR_ADC2_RMP[1:0] bits (TIM8 ETR remap on ADC2) */
#define TIM8_OR1_ETR_ADC2_RMP_0    ((uint32_t)0x00000001)            /*!<Bit 0 */
#define TIM8_OR1_ETR_ADC2_RMP_1    ((uint32_t)0x00000002)            /*!<Bit 1 */

#define TIM8_OR1_ETR_ADC3_RMP      ((uint32_t)0x0000000C)            /*!<ETR_ADC3_RMP[1:0] bits (TIM8 ETR remap on ADC3) */
#define TIM8_OR1_ETR_ADC3_RMP_0    ((uint32_t)0x00000004)            /*!<Bit 0 */
#define TIM8_OR1_ETR_ADC3_RMP_1    ((uint32_t)0x00000008)            /*!<Bit 1 */

#define TIM8_OR1_TI1_RMP           ((uint32_t)0x00000010)            /*!<TIM8 Input Capture 1 remap */

/*******************  Bit definition for TIM8_OR2 register  *******************/
#define TIM8_OR2_BKINE             ((uint32_t)0x00000001)            /*!<BRK BKIN input enable */
#define TIM8_OR2_BKCMP1E           ((uint32_t)0x00000002)            /*!<BRK COMP1 enable */
#define TIM8_OR2_BKCMP2E           ((uint32_t)0x00000004)            /*!<BRK COMP2 enable */
#define TIM8_OR2_BKDFBK2E          ((uint32_t)0x00000100)            /*!<BRK DFSDM_BREAK[2] enable */
#define TIM8_OR2_BKINP             ((uint32_t)0x00000200)            /*!<BRK BKIN input polarity */
#define TIM8_OR2_BKCMP1P           ((uint32_t)0x00000400)            /*!<BRK COMP1 input polarity */
#define TIM8_OR2_BKCMP2P           ((uint32_t)0x00000800)            /*!<BRK COMP2 input polarity */

#define TIM8_OR2_ETRSEL            ((uint32_t)0x0001C000)            /*!<ETRSEL[2:0] bits (TIM8 ETR source selection) */
#define TIM8_OR2_ETRSEL_0          ((uint32_t)0x00004000)            /*!<Bit 0 */
#define TIM8_OR2_ETRSEL_1          ((uint32_t)0x00008000)            /*!<Bit 1 */
#define TIM8_OR2_ETRSEL_2          ((uint32_t)0x00010000)            /*!<Bit 2 */

/*******************  Bit definition for TIM8_OR3 register  *******************/
#define TIM8_OR3_BK2INE            ((uint32_t)0x00000001)            /*!<BRK2 BKIN2 input enable */
#define TIM8_OR3_BK2CMP1E          ((uint32_t)0x00000002)            /*!<BRK2 COMP1 enable */
#define TIM8_OR3_BK2CMP2E          ((uint32_t)0x00000004)            /*!<BRK2 COMP2 enable */
#define TIM8_OR3_BK2DFBK3E         ((uint32_t)0x00000100)            /*!<BRK2 DFSDM_BREAK[3] enable */
#define TIM8_OR3_BK2INP            ((uint32_t)0x00000200)            /*!<BRK2 BKIN2 input polarity */
#define TIM8_OR3_BK2CMP1P          ((uint32_t)0x00000400)            /*!<BRK2 COMP1 input polarity */
#define TIM8_OR3_BK2CMP2P          ((uint32_t)0x00000800)            /*!<BRK2 COMP2 input polarity */

/*******************  Bit definition for TIM2_OR1 register  *******************/
#define TIM2_OR1_ITR1_RMP          ((uint32_t)0x00000001)            /*!<TIM2 Internal trigger 1 remap */
#define TIM2_OR1_ETR1_RMP          ((uint32_t)0x00000002)            /*!<TIM2 External trigger 1 remap */

#define TIM2_OR1_TI4_RMP           ((uint32_t)0x0000000C)            /*!<TI4_RMP[1:0] bits (TIM2 Input Capture 4 remap) */
#define TIM2_OR1_TI4_RMP_0         ((uint32_t)0x00000004)            /*!<Bit 0 */
#define TIM2_OR1_TI4_RMP_1         ((uint32_t)0x00000008)            /*!<Bit 1 */

/*******************  Bit definition for TIM2_OR2 register  *******************/
#define TIM2_OR2_ETRSEL            ((uint32_t)0x0001C000)            /*!<ETRSEL[2:0] bits (TIM2 ETR source selection) */
#define TIM2_OR2_ETRSEL_0          ((uint32_t)0x00004000)            /*!<Bit 0 */
#define TIM2_OR2_ETRSEL_1          ((uint32_t)0x00008000)            /*!<Bit 1 */
#define TIM2_OR2_ETRSEL_2          ((uint32_t)0x00010000)            /*!<Bit 2 */

/*******************  Bit definition for TIM3_OR1 register  *******************/
#define TIM3_OR1_TI1_RMP           ((uint32_t)0x00000003)            /*!<TI1_RMP[1:0] bits (TIM3 Input Capture 1 remap) */
#define TIM3_OR1_TI1_RMP_0         ((uint32_t)0x00000001)            /*!<Bit 0 */
#define TIM3_OR1_TI1_RMP_1         ((uint32_t)0x00000002)            /*!<Bit 1 */

/*******************  Bit definition for TIM3_OR2 register  *******************/
#define TIM3_OR2_ETRSEL            ((uint32_t)0x0001C000)            /*!<ETRSEL[2:0] bits (TIM3 ETR source selection) */
#define TIM3_OR2_ETRSEL_0          ((uint32_t)0x00004000)            /*!<Bit 0 */
#define TIM3_OR2_ETRSEL_1          ((uint32_t)0x00008000)            /*!<Bit 1 */
#define TIM3_OR2_ETRSEL_2          ((uint32_t)0x00010000)            /*!<Bit 2 */

/*******************  Bit definition for TIM15_OR1 register  ******************/
#define TIM15_OR1_TI1_RMP          ((uint32_t)0x00000001)            /*!<TIM15 Input Capture 1 remap */

#define TIM15_OR1_ENCODER_MODE     ((uint32_t)0x00000006)            /*!<ENCODER_MODE[1:0] bits (TIM15 Encoder mode) */
#define TIM15_OR1_ENCODER_MODE_0   ((uint32_t)0x00000002)            /*!<Bit 0 */
#define TIM15_OR1_ENCODER_MODE_1   ((uint32_t)0x00000004)            /*!<Bit 1 */

/*******************  Bit definition for TIM15_OR2 register  ******************/
#define TIM15_OR2_BKINE            ((uint32_t)0x00000001)            /*!<BRK BKIN input enable */
#define TIM15_OR2_BKCMP1E          ((uint32_t)0x00000002)            /*!<BRK COMP1 enable */
#define TIM15_OR2_BKCMP2E          ((uint32_t)0x00000004)            /*!<BRK COMP2 enable */
#define TIM15_OR2_BKDFBK0E         ((uint32_t)0x00000100)            /*!<BRK DFSDM_BREAK[0] enable */
#define TIM15_OR2_BKINP            ((uint32_t)0x00000200)            /*!<BRK BKIN input polarity */
#define TIM15_OR2_BKCMP1P          ((uint32_t)0x00000400)            /*!<BRK COMP1 input polarity */
#define TIM15_OR2_BKCMP2P          ((uint32_t)0x00000800)            /*!<BRK COMP2 input polarity */

/*******************  Bit definition for TIM16_OR1 register  ******************/
#define TIM16_OR1_TI1_RMP          ((uint32_t)0x00000003)            /*!<TI1_RMP[1:0] bits (TIM16 Input Capture 1 remap) */
#define TIM16_OR1_TI1_RMP_0        ((uint32_t)0x00000001)            /*!<Bit 0 */
#define TIM16_OR1_TI1_RMP_1        ((uint32_t)0x00000002)            /*!<Bit 1 */

/*******************  Bit definition for TIM16_OR2 register  ******************/
#define TIM16_OR2_BKINE            ((uint32_t)0x00000001)            /*!<BRK BKIN input enable */
#define TIM16_OR2_BKCMP1E          ((uint32_t)0x00000002)            /*!<BRK COMP1 enable */
#define TIM16_OR2_BKCMP2E          ((uint32_t)0x00000004)            /*!<BRK COMP2 enable */
#define TIM16_OR2_BKINP            ((uint32_t)0x00000200)            /*!<BRK BKIN input polarity */
#define TIM16_OR2_BKCMP1P          ((uint32_t)0x00000400)            /*!<BRK COMP1 input polarity */
#define TIM16_OR2_BKCMP2P          ((uint32_t)0x00000800)            /*!<BRK COMP2 input polarity */

/*******************  Bit definition for TIM17_OR1 register  ******************/
#define TIM17_OR1_TI1_RMP          ((uint32_t)0x00000003)            /*!<TI1_RMP[1:0] bits (TIM17 Input Capture 1 remap) */
#define TIM17_OR1_TI1_RMP_0        ((uint32_t)0x00000001)            /*!<Bit 0 */
#define TIM17_OR1_TI1_RMP_1        ((uint32_t)0x00000002)            /*!<Bit 1 */

/*******************  Bit definition for TIM17_OR2 register  ******************/
#define TIM17_OR2_BKINE            ((uint32_t)0x00000001)            /*!<BRK BKIN input enable */
#define TIM17_OR2_BKCMP1E          ((uint32_t)0x00000002)            /*!<BRK COMP1 enable */
#define TIM17_OR2_BKCMP2E          ((uint32_t)0x00000004)            /*!<BRK COMP2 enable */
#define TIM17_OR2_BKINP            ((uint32_t)0x00000200)            /*!<BRK BKIN input polarity */
#define TIM17_OR2_BKCMP1P          ((uint32_t)0x00000400)            /*!<BRK COMP1 input polarity */
#define TIM17_OR2_BKCMP2P          ((uint32_t)0x00000800)            /*!<BRK COMP2 input polarity */

/******************************************************************************/
/*                                                                            */
/*                         Low Power Timer (LPTTIM)                           */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for LPTIM_ISR register  *******************/
#define  LPTIM_ISR_CMPM                         ((uint32_t)0x00000001)            /*!< Compare match */
#define  LPTIM_ISR_ARRM                         ((uint32_t)0x00000002)            /*!< Autoreload match */
#define  LPTIM_ISR_EXTTRIG                      ((uint32_t)0x00000004)            /*!< External trigger edge event */
#define  LPTIM_ISR_CMPOK                        ((uint32_t)0x00000008)            /*!< Compare register update OK */
#define  LPTIM_ISR_ARROK                        ((uint32_t)0x00000010)            /*!< Autoreload register update OK */
#define  LPTIM_ISR_UP                           ((uint32_t)0x00000020)            /*!< Counter direction change down to up */
#define  LPTIM_ISR_DOWN                         ((uint32_t)0x00000040)            /*!< Counter direction change up to down */

/******************  Bit definition for LPTIM_ICR register  *******************/
#define  LPTIM_ICR_CMPMCF                       ((uint32_t)0x00000001)            /*!< Compare match Clear Flag */
#define  LPTIM_ICR_ARRMCF                       ((uint32_t)0x00000002)            /*!< Autoreload match Clear Flag */
#define  LPTIM_ICR_EXTTRIGCF                    ((uint32_t)0x00000004)            /*!< External trigger edge event Clear Flag */
#define  LPTIM_ICR_CMPOKCF                      ((uint32_t)0x00000008)            /*!< Compare register update OK Clear Flag */
#define  LPTIM_ICR_ARROKCF                      ((uint32_t)0x00000010)            /*!< Autoreload register update OK Clear Flag */
#define  LPTIM_ICR_UPCF                         ((uint32_t)0x00000020)            /*!< Counter direction change down to up Clear Flag */
#define  LPTIM_ICR_DOWNCF                       ((uint32_t)0x00000040)            /*!< Counter direction change up to down Clear Flag */

/******************  Bit definition for LPTIM_IER register ********************/
#define  LPTIM_IER_CMPMIE                       ((uint32_t)0x00000001)            /*!< Compare match Interrupt Enable */
#define  LPTIM_IER_ARRMIE                       ((uint32_t)0x00000002)            /*!< Autoreload match Interrupt Enable */
#define  LPTIM_IER_EXTTRIGIE                    ((uint32_t)0x00000004)            /*!< External trigger edge event Interrupt Enable */
#define  LPTIM_IER_CMPOKIE                      ((uint32_t)0x00000008)            /*!< Compare register update OK Interrupt Enable */
#define  LPTIM_IER_ARROKIE                      ((uint32_t)0x00000010)            /*!< Autoreload register update OK Interrupt Enable */
#define  LPTIM_IER_UPIE                         ((uint32_t)0x00000020)            /*!< Counter direction change down to up Interrupt Enable */
#define  LPTIM_IER_DOWNIE                       ((uint32_t)0x00000040)            /*!< Counter direction change up to down Interrupt Enable */

/******************  Bit definition for LPTIM_CFGR register *******************/
#define  LPTIM_CFGR_CKSEL                       ((uint32_t)0x00000001)             /*!< Clock selector */

#define  LPTIM_CFGR_CKPOL                       ((uint32_t)0x00000006)             /*!< CKPOL[1:0] bits (Clock polarity) */
#define  LPTIM_CFGR_CKPOL_0                     ((uint32_t)0x00000002)             /*!< Bit 0 */
#define  LPTIM_CFGR_CKPOL_1                     ((uint32_t)0x00000004)             /*!< Bit 1 */

#define  LPTIM_CFGR_CKFLT                       ((uint32_t)0x00000018)             /*!< CKFLT[1:0] bits (Configurable digital filter for external clock) */
#define  LPTIM_CFGR_CKFLT_0                     ((uint32_t)0x00000008)             /*!< Bit 0 */
#define  LPTIM_CFGR_CKFLT_1                     ((uint32_t)0x00000010)             /*!< Bit 1 */

#define  LPTIM_CFGR_TRGFLT                      ((uint32_t)0x000000C0)             /*!< TRGFLT[1:0] bits (Configurable digital filter for trigger) */
#define  LPTIM_CFGR_TRGFLT_0                    ((uint32_t)0x00000040)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRGFLT_1                    ((uint32_t)0x00000080)             /*!< Bit 1 */

#define  LPTIM_CFGR_PRESC                       ((uint32_t)0x00000E00)             /*!< PRESC[2:0] bits (Clock prescaler) */
#define  LPTIM_CFGR_PRESC_0                     ((uint32_t)0x00000200)             /*!< Bit 0 */
#define  LPTIM_CFGR_PRESC_1                     ((uint32_t)0x00000400)             /*!< Bit 1 */
#define  LPTIM_CFGR_PRESC_2                     ((uint32_t)0x00000800)             /*!< Bit 2 */

#define  LPTIM_CFGR_TRIGSEL                     ((uint32_t)0x0000E000)             /*!< TRIGSEL[2:0]] bits (Trigger selector) */
#define  LPTIM_CFGR_TRIGSEL_0                   ((uint32_t)0x00002000)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRIGSEL_1                   ((uint32_t)0x00004000)             /*!< Bit 1 */
#define  LPTIM_CFGR_TRIGSEL_2                   ((uint32_t)0x00008000)             /*!< Bit 2 */

#define  LPTIM_CFGR_TRIGEN                      ((uint32_t)0x00060000)             /*!< TRIGEN[1:0] bits (Trigger enable and polarity) */
#define  LPTIM_CFGR_TRIGEN_0                    ((uint32_t)0x00020000)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRIGEN_1                    ((uint32_t)0x00040000)             /*!< Bit 1 */

#define  LPTIM_CFGR_TIMOUT                      ((uint32_t)0x00080000)             /*!< Timout enable */
#define  LPTIM_CFGR_WAVE                        ((uint32_t)0x00100000)             /*!< Waveform shape */
#define  LPTIM_CFGR_WAVPOL                      ((uint32_t)0x00200000)             /*!< Waveform shape polarity */
#define  LPTIM_CFGR_PRELOAD                     ((uint32_t)0x00400000)             /*!< Reg update mode */
#define  LPTIM_CFGR_COUNTMODE                   ((uint32_t)0x00800000)             /*!< Counter mode enable */
#define  LPTIM_CFGR_ENC                         ((uint32_t)0x01000000)             /*!< Encoder mode enable */

/******************  Bit definition for LPTIM_CR register  ********************/
#define  LPTIM_CR_ENABLE                        ((uint32_t)0x00000001)             /*!< LPTIMer enable */
#define  LPTIM_CR_SNGSTRT                       ((uint32_t)0x00000002)             /*!< Timer start in single mode */
#define  LPTIM_CR_CNTSTRT                       ((uint32_t)0x00000004)             /*!< Timer start in continuous mode */

/******************  Bit definition for LPTIM_CMP register  *******************/
#define  LPTIM_CMP_CMP                          ((uint32_t)0x0000FFFF)             /*!< Compare register */

/******************  Bit definition for LPTIM_ARR register  *******************/
#define  LPTIM_ARR_ARR                          ((uint32_t)0x0000FFFF)             /*!< Auto reload register */

/******************  Bit definition for LPTIM_CNT register  *******************/
#define  LPTIM_CNT_CNT                          ((uint32_t)0x0000FFFF)             /*!< Counter register */

/******************  Bit definition for LPTIM_OR register  *******************/
#define  LPTIM_OR_OR                           ((uint32_t)0x00000003)               /*!< LPTIMER[1:0] bits (Remap selection) */
#define  LPTIM_OR_OR_0                         ((uint32_t)0x00000001)               /*!< Bit 0 */
#define  LPTIM_OR_OR_1                         ((uint32_t)0x00000002)               /*!< Bit 1 */

/******************************************************************************/
/*                                                                            */
/*                      Analog Comparators (COMP)                             */
/*                                                                            */
/******************************************************************************/
/**********************  Bit definition for COMPx_CSR register  ***************/
#define COMP_CSR_EN                    ((uint32_t)0x00000001) /*!< COMPx enable */

#define COMP_CSR_PWRMODE               ((uint32_t)0x0000000C) /*!< COMPx power mode */
#define COMP_CSR_PWRMODE_0             ((uint32_t)0x00000004) /*!< COMPx power mode bit 0 */
#define COMP_CSR_PWRMODE_1             ((uint32_t)0x00000008) /*!< COMPx power mode bit 1 */

#define COMP_CSR_INMSEL                ((uint32_t)0x00000070) /*!< COMPx inverting input selection */
#define COMP_CSR_INMSEL_0              ((uint32_t)0x00000010) /*!< COMPx inverting input selection bit 0 */
#define COMP_CSR_INMSEL_1              ((uint32_t)0x00000020) /*!< COMPx inverting input selection bit 1 */
#define COMP_CSR_INMSEL_2              ((uint32_t)0x00000040) /*!< COMPx inverting input selection bit 2 */

#define COMP_CSR_INPSEL                ((uint32_t)0x00000080) /*!< COMPx non inverting input selection */
#define COMP_CSR_WINMODE               ((uint32_t)0x00000200) /*!< COMPx window mode */
#define COMP_CSR_POLARITY              ((uint32_t)0x00008000) /*!< COMPx output polarity */

#define COMP_CSR_HYST                  ((uint32_t)0x00030000) /*!< COMPx hysteresis */
#define COMP_CSR_HYST_0                ((uint32_t)0x00010000) /*!< COMPx hysteresis bit 0 */
#define COMP_CSR_HYST_1                ((uint32_t)0x00020000) /*!< COMPx hysteresis bit 1 */

#define COMP_CSR_BLANKING              ((uint32_t)0x001C0000) /*!< COMPx blanking source */
#define COMP_CSR_BLANKING_0            ((uint32_t)0x00040000) /*!< COMPx blanking source bit 0 */
#define COMP_CSR_BLANKING_1            ((uint32_t)0x00080000) /*!< COMPx blanking source bit 1 */
#define COMP_CSR_BLANKING_2            ((uint32_t)0x00100000) /*!< COMPx blanking source bit 2 */

#define COMP_CSR_BRGEN                 ((uint32_t)0x00400000) /*!< COMPx voltage scaler enable */
#define COMP_CSR_SCALEN                ((uint32_t)0x00800000) /*!< COMPx scaler bridge enable */
#define COMP_CSR_VALUE                 ((uint32_t)0x40000000) /*!< COMPx value */
#define COMP_CSR_LOCK                  ((uint32_t)0x80000000) /*!< COMPx lock */

/******************************************************************************/
/*                                                                            */
/*                         Operational Amplifier (OPAMP)                      */
/*                                                                            */
/******************************************************************************/
/*********************  Bit definition for OPAMPx_CSR register  ***************/
#define OPAMP_CSR_OPAMPxEN            ((uint32_t)0x00000001) /*!< OPAMP enable */
#define OPAMP_CSR_OPALPM              ((uint32_t)0x00000002) /*!< Operational amplifier Low Power Mode */

#define OPAMP_CSR_OPAMODE             ((uint32_t)0x0000000C) /*!< Operational amplifier PGA mode */
#define OPAMP_CSR_OPAMODE_0           ((uint32_t)0x00000004) /*!< Bit 0 */
#define OPAMP_CSR_OPAMODE_1           ((uint32_t)0x00000008) /*!< Bit 1 */

#define OPAMP_CSR_PGGAIN             ((uint32_t)0x00000030) /*!< Operational amplifier Programmable amplifier gain value */
#define OPAMP_CSR_PGGAIN_0           ((uint32_t)0x00000010) /*!< Bit 0 */
#define OPAMP_CSR_PGGAIN_1           ((uint32_t)0x00000020) /*!< Bit 1 */

#define OPAMP_CSR_VMSEL               ((uint32_t)0x00000300) /*!< Inverting input selection */
#define OPAMP_CSR_VMSEL_0             ((uint32_t)0x00000100) /*!< Bit 0 */
#define OPAMP_CSR_VMSEL_1             ((uint32_t)0x00000200) /*!< Bit 1 */

#define OPAMP_CSR_VPSEL               ((uint32_t)0x00000400) /*!< Non inverted input selection */
#define OPAMP_CSR_CALON               ((uint32_t)0x00001000) /*!< Calibration mode enable */
#define OPAMP_CSR_CALSEL              ((uint32_t)0x00002000) /*!< Calibration selection */
#define OPAMP_CSR_USERTRIM            ((uint32_t)0x00004000) /*!< User trimming enable */
#define OPAMP_CSR_CALOUT              ((uint32_t)0x00008000) /*!< Operational amplifier1 calibration output */

/*********************  Bit definition for OPAMP1_CSR register  ***************/
#define OPAMP1_CSR_OPAEN               ((uint32_t)0x00000001) /*!< Operational amplifier1 Enable */
#define OPAMP1_CSR_OPALPM              ((uint32_t)0x00000002) /*!< Operational amplifier1 Low Power Mode */

#define OPAMP1_CSR_OPAMODE             ((uint32_t)0x0000000C) /*!< Operational amplifier1 PGA mode */
#define OPAMP1_CSR_OPAMODE_0           ((uint32_t)0x00000004) /*!< Bit 0 */
#define OPAMP1_CSR_OPAMODE_1           ((uint32_t)0x00000008) /*!< Bit 1 */

#define OPAMP1_CSR_PGAGAIN             ((uint32_t)0x00000030) /*!< Operational amplifier1 Programmable amplifier gain value */
#define OPAMP1_CSR_PGAGAIN_0           ((uint32_t)0x00000010) /*!< Bit 0 */
#define OPAMP1_CSR_PGAGAIN_1           ((uint32_t)0x00000020) /*!< Bit 1 */

#define OPAMP1_CSR_VMSEL               ((uint32_t)0x00000300) /*!< Inverting input selection */
#define OPAMP1_CSR_VMSEL_0             ((uint32_t)0x00000100) /*!< Bit 0 */
#define OPAMP1_CSR_VMSEL_1             ((uint32_t)0x00000200) /*!< Bit 1 */

#define OPAMP1_CSR_VPSEL               ((uint32_t)0x00000400) /*!< Non inverted input selection */
#define OPAMP1_CSR_CALON               ((uint32_t)0x00001000) /*!< Calibration mode enable */
#define OPAMP1_CSR_CALSEL              ((uint32_t)0x00002000) /*!< Calibration selection */
#define OPAMP1_CSR_USERTRIM            ((uint32_t)0x00004000) /*!< User trimming enable */
#define OPAMP1_CSR_CALOUT              ((uint32_t)0x00008000) /*!< Operational amplifier1 calibration output */
#define OPAMP1_CSR_OPARANGE            ((uint32_t)0x80000000) /*!< Operational amplifiers power supply range for stability */

/*********************  Bit definition for OPAMP2_CSR register  ***************/
#define OPAMP2_CSR_OPAEN               ((uint32_t)0x00000001) /*!< Operational amplifier2 Enable */
#define OPAMP2_CSR_OPALPM              ((uint32_t)0x00000002) /*!< Operational amplifier2 Low Power Mode */

#define OPAMP2_CSR_OPAMODE             ((uint32_t)0x0000000C) /*!< Operational amplifier2 PGA mode */
#define OPAMP2_CSR_OPAMODE_0           ((uint32_t)0x00000004) /*!< Bit 0 */
#define OPAMP2_CSR_OPAMODE_1           ((uint32_t)0x00000008) /*!< Bit 1 */

#define OPAMP2_CSR_PGAGAIN             ((uint32_t)0x00000030) /*!< Operational amplifier2 Programmable amplifier gain value */
#define OPAMP2_CSR_PGAGAIN_0           ((uint32_t)0x00000010) /*!< Bit 0 */
#define OPAMP2_CSR_PGAGAIN_1           ((uint32_t)0x00000020) /*!< Bit 1 */

#define OPAMP2_CSR_VMSEL               ((uint32_t)0x00000300) /*!< Inverting input selection */
#define OPAMP2_CSR_VMSEL_0             ((uint32_t)0x00000100) /*!< Bit 0 */
#define OPAMP2_CSR_VMSEL_1             ((uint32_t)0x00000200) /*!< Bit 1 */

#define OPAMP2_CSR_VPSEL               ((uint32_t)0x00000400) /*!< Non inverted input selection */
#define OPAMP2_CSR_CALON               ((uint32_t)0x00001000) /*!< Calibration mode enable */
#define OPAMP2_CSR_CALSEL              ((uint32_t)0x00002000) /*!< Calibration selection */
#define OPAMP2_CSR_USERTRIM            ((uint32_t)0x00004000) /*!< User trimming enable */
#define OPAMP2_CSR_CALOUT              ((uint32_t)0x00008000) /*!< Operational amplifier2 calibration output */

/*******************  Bit definition for OPAMP_OTR register  ******************/
#define OPAMP_OTR_TRIMOFFSETN            ((uint32_t)0x0000001F)        /*!< Trim for NMOS differential pairs */
#define OPAMP_OTR_TRIMOFFSETP            ((uint32_t)0x00001F00)        /*!< Trim for PMOS differential pairs */

/*******************  Bit definition for OPAMP1_OTR register  ******************/
#define OPAMP1_OTR_TRIMOFFSETN            ((uint32_t)0x0000001F)        /*!< Trim for NMOS differential pairs */
#define OPAMP1_OTR_TRIMOFFSETP            ((uint32_t)0x00001F00)        /*!< Trim for PMOS differential pairs */

/*******************  Bit definition for OPAMP2_OTR register  ******************/
#define OPAMP2_OTR_TRIMOFFSETN            ((uint32_t)0x0000001F)        /*!< Trim for NMOS differential pairs */
#define OPAMP2_OTR_TRIMOFFSETP            ((uint32_t)0x00001F00)        /*!< Trim for PMOS differential pairs */

/*******************  Bit definition for OPAMP_LPOTR register  ****************/
#define OPAMP_LPOTR_TRIMLPOFFSETN        ((uint32_t)0x0000001F)        /*!< Trim for NMOS differential pairs */
#define OPAMP_LPOTR_TRIMLPOFFSETP        ((uint32_t)0x00001F00)        /*!< Trim for PMOS differential pairs */

/*******************  Bit definition for OPAMP1_LPOTR register  ****************/
#define OPAMP1_LPOTR_TRIMLPOFFSETN        ((uint32_t)0x0000001F)        /*!< Trim for NMOS differential pairs */
#define OPAMP1_LPOTR_TRIMLPOFFSETP        ((uint32_t)0x00001F00)        /*!< Trim for PMOS differential pairs */

/*******************  Bit definition for OPAMP2_LPOTR register  ****************/
#define OPAMP2_LPOTR_TRIMLPOFFSETN        ((uint32_t)0x0000001F)        /*!< Trim for NMOS differential pairs */
#define OPAMP2_LPOTR_TRIMLPOFFSETP        ((uint32_t)0x00001F00)        /*!< Trim for PMOS differential pairs */

/******************************************************************************/
/*                                                                            */
/*                          Touch Sensing Controller (TSC)                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TSC_CR register  *********************/
#define TSC_CR_TSCE                         ((uint32_t)0x00000001)            /*!<Touch sensing controller enable */
#define TSC_CR_START                        ((uint32_t)0x00000002)            /*!<Start acquisition */
#define TSC_CR_AM                           ((uint32_t)0x00000004)            /*!<Acquisition mode */
#define TSC_CR_SYNCPOL                      ((uint32_t)0x00000008)            /*!<Synchronization pin polarity */
#define TSC_CR_IODEF                        ((uint32_t)0x00000010)            /*!<IO default mode */

#define TSC_CR_MCV                          ((uint32_t)0x000000E0)            /*!<MCV[2:0] bits (Max Count Value) */
#define TSC_CR_MCV_0                        ((uint32_t)0x00000020)            /*!<Bit 0 */
#define TSC_CR_MCV_1                        ((uint32_t)0x00000040)            /*!<Bit 1 */
#define TSC_CR_MCV_2                        ((uint32_t)0x00000080)            /*!<Bit 2 */

#define TSC_CR_PGPSC                        ((uint32_t)0x00007000)            /*!<PGPSC[2:0] bits (Pulse Generator Prescaler) */
#define TSC_CR_PGPSC_0                      ((uint32_t)0x00001000)            /*!<Bit 0 */
#define TSC_CR_PGPSC_1                      ((uint32_t)0x00002000)            /*!<Bit 1 */
#define TSC_CR_PGPSC_2                      ((uint32_t)0x00004000)            /*!<Bit 2 */

#define TSC_CR_SSPSC                        ((uint32_t)0x00008000)            /*!<Spread Spectrum Prescaler */
#define TSC_CR_SSE                          ((uint32_t)0x00010000)            /*!<Spread Spectrum Enable */

#define TSC_CR_SSD                          ((uint32_t)0x00FE0000)            /*!<SSD[6:0] bits (Spread Spectrum Deviation) */
#define TSC_CR_SSD_0                        ((uint32_t)0x00020000)            /*!<Bit 0 */
#define TSC_CR_SSD_1                        ((uint32_t)0x00040000)            /*!<Bit 1 */
#define TSC_CR_SSD_2                        ((uint32_t)0x00080000)            /*!<Bit 2 */
#define TSC_CR_SSD_3                        ((uint32_t)0x00100000)            /*!<Bit 3 */
#define TSC_CR_SSD_4                        ((uint32_t)0x00200000)            /*!<Bit 4 */
#define TSC_CR_SSD_5                        ((uint32_t)0x00400000)            /*!<Bit 5 */
#define TSC_CR_SSD_6                        ((uint32_t)0x00800000)            /*!<Bit 6 */

#define TSC_CR_CTPL                         ((uint32_t)0x0F000000)            /*!<CTPL[3:0] bits (Charge Transfer pulse low) */
#define TSC_CR_CTPL_0                       ((uint32_t)0x01000000)            /*!<Bit 0 */
#define TSC_CR_CTPL_1                       ((uint32_t)0x02000000)            /*!<Bit 1 */
#define TSC_CR_CTPL_2                       ((uint32_t)0x04000000)            /*!<Bit 2 */
#define TSC_CR_CTPL_3                       ((uint32_t)0x08000000)            /*!<Bit 3 */

#define TSC_CR_CTPH                         ((uint32_t)0xF0000000)            /*!<CTPH[3:0] bits (Charge Transfer pulse high) */
#define TSC_CR_CTPH_0                       ((uint32_t)0x10000000)            /*!<Bit 0 */
#define TSC_CR_CTPH_1                       ((uint32_t)0x20000000)            /*!<Bit 1 */
#define TSC_CR_CTPH_2                       ((uint32_t)0x40000000)            /*!<Bit 2 */
#define TSC_CR_CTPH_3                       ((uint32_t)0x80000000)            /*!<Bit 3 */

/*******************  Bit definition for TSC_IER register  ********************/
#define TSC_IER_EOAIE                       ((uint32_t)0x00000001)            /*!<End of acquisition interrupt enable */
#define TSC_IER_MCEIE                       ((uint32_t)0x00000002)            /*!<Max count error interrupt enable */

/*******************  Bit definition for TSC_ICR register  ********************/
#define TSC_ICR_EOAIC                       ((uint32_t)0x00000001)            /*!<End of acquisition interrupt clear */
#define TSC_ICR_MCEIC                       ((uint32_t)0x00000002)            /*!<Max count error interrupt clear */

/*******************  Bit definition for TSC_ISR register  ********************/
#define TSC_ISR_EOAF                        ((uint32_t)0x00000001)            /*!<End of acquisition flag */
#define TSC_ISR_MCEF                        ((uint32_t)0x00000002)            /*!<Max count error flag */

/*******************  Bit definition for TSC_IOHCR register  ******************/
#define TSC_IOHCR_G1_IO1                    ((uint32_t)0x00000001)            /*!<GROUP1_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO2                    ((uint32_t)0x00000002)            /*!<GROUP1_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO3                    ((uint32_t)0x00000004)            /*!<GROUP1_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO4                    ((uint32_t)0x00000008)            /*!<GROUP1_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO1                    ((uint32_t)0x00000010)            /*!<GROUP2_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO2                    ((uint32_t)0x00000020)            /*!<GROUP2_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO3                    ((uint32_t)0x00000040)            /*!<GROUP2_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO4                    ((uint32_t)0x00000080)            /*!<GROUP2_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO1                    ((uint32_t)0x00000100)            /*!<GROUP3_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO2                    ((uint32_t)0x00000200)            /*!<GROUP3_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO3                    ((uint32_t)0x00000400)            /*!<GROUP3_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO4                    ((uint32_t)0x00000800)            /*!<GROUP3_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO1                    ((uint32_t)0x00001000)            /*!<GROUP4_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO2                    ((uint32_t)0x00002000)            /*!<GROUP4_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO3                    ((uint32_t)0x00004000)            /*!<GROUP4_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO4                    ((uint32_t)0x00008000)            /*!<GROUP4_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO1                    ((uint32_t)0x00010000)            /*!<GROUP5_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO2                    ((uint32_t)0x00020000)            /*!<GROUP5_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO3                    ((uint32_t)0x00040000)            /*!<GROUP5_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO4                    ((uint32_t)0x00080000)            /*!<GROUP5_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO1                    ((uint32_t)0x00100000)            /*!<GROUP6_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO2                    ((uint32_t)0x00200000)            /*!<GROUP6_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO3                    ((uint32_t)0x00400000)            /*!<GROUP6_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO4                    ((uint32_t)0x00800000)            /*!<GROUP6_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO1                    ((uint32_t)0x01000000)            /*!<GROUP7_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO2                    ((uint32_t)0x02000000)            /*!<GROUP7_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO3                    ((uint32_t)0x04000000)            /*!<GROUP7_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO4                    ((uint32_t)0x08000000)            /*!<GROUP7_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G8_IO1                    ((uint32_t)0x10000000)            /*!<GROUP8_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G8_IO2                    ((uint32_t)0x20000000)            /*!<GROUP8_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G8_IO3                    ((uint32_t)0x40000000)            /*!<GROUP8_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G8_IO4                    ((uint32_t)0x80000000)            /*!<GROUP8_IO4 schmitt trigger hysteresis mode */

/*******************  Bit definition for TSC_IOASCR register  *****************/
#define TSC_IOASCR_G1_IO1                   ((uint32_t)0x00000001)            /*!<GROUP1_IO1 analog switch enable */
#define TSC_IOASCR_G1_IO2                   ((uint32_t)0x00000002)            /*!<GROUP1_IO2 analog switch enable */
#define TSC_IOASCR_G1_IO3                   ((uint32_t)0x00000004)            /*!<GROUP1_IO3 analog switch enable */
#define TSC_IOASCR_G1_IO4                   ((uint32_t)0x00000008)            /*!<GROUP1_IO4 analog switch enable */
#define TSC_IOASCR_G2_IO1                   ((uint32_t)0x00000010)            /*!<GROUP2_IO1 analog switch enable */
#define TSC_IOASCR_G2_IO2                   ((uint32_t)0x00000020)            /*!<GROUP2_IO2 analog switch enable */
#define TSC_IOASCR_G2_IO3                   ((uint32_t)0x00000040)            /*!<GROUP2_IO3 analog switch enable */
#define TSC_IOASCR_G2_IO4                   ((uint32_t)0x00000080)            /*!<GROUP2_IO4 analog switch enable */
#define TSC_IOASCR_G3_IO1                   ((uint32_t)0x00000100)            /*!<GROUP3_IO1 analog switch enable */
#define TSC_IOASCR_G3_IO2                   ((uint32_t)0x00000200)            /*!<GROUP3_IO2 analog switch enable */
#define TSC_IOASCR_G3_IO3                   ((uint32_t)0x00000400)            /*!<GROUP3_IO3 analog switch enable */
#define TSC_IOASCR_G3_IO4                   ((uint32_t)0x00000800)            /*!<GROUP3_IO4 analog switch enable */
#define TSC_IOASCR_G4_IO1                   ((uint32_t)0x00001000)            /*!<GROUP4_IO1 analog switch enable */
#define TSC_IOASCR_G4_IO2                   ((uint32_t)0x00002000)            /*!<GROUP4_IO2 analog switch enable */
#define TSC_IOASCR_G4_IO3                   ((uint32_t)0x00004000)            /*!<GROUP4_IO3 analog switch enable */
#define TSC_IOASCR_G4_IO4                   ((uint32_t)0x00008000)            /*!<GROUP4_IO4 analog switch enable */
#define TSC_IOASCR_G5_IO1                   ((uint32_t)0x00010000)            /*!<GROUP5_IO1 analog switch enable */
#define TSC_IOASCR_G5_IO2                   ((uint32_t)0x00020000)            /*!<GROUP5_IO2 analog switch enable */
#define TSC_IOASCR_G5_IO3                   ((uint32_t)0x00040000)            /*!<GROUP5_IO3 analog switch enable */
#define TSC_IOASCR_G5_IO4                   ((uint32_t)0x00080000)            /*!<GROUP5_IO4 analog switch enable */
#define TSC_IOASCR_G6_IO1                   ((uint32_t)0x00100000)            /*!<GROUP6_IO1 analog switch enable */
#define TSC_IOASCR_G6_IO2                   ((uint32_t)0x00200000)            /*!<GROUP6_IO2 analog switch enable */
#define TSC_IOASCR_G6_IO3                   ((uint32_t)0x00400000)            /*!<GROUP6_IO3 analog switch enable */
#define TSC_IOASCR_G6_IO4                   ((uint32_t)0x00800000)            /*!<GROUP6_IO4 analog switch enable */
#define TSC_IOASCR_G7_IO1                   ((uint32_t)0x01000000)            /*!<GROUP7_IO1 analog switch enable */
#define TSC_IOASCR_G7_IO2                   ((uint32_t)0x02000000)            /*!<GROUP7_IO2 analog switch enable */
#define TSC_IOASCR_G7_IO3                   ((uint32_t)0x04000000)            /*!<GROUP7_IO3 analog switch enable */
#define TSC_IOASCR_G7_IO4                   ((uint32_t)0x08000000)            /*!<GROUP7_IO4 analog switch enable */
#define TSC_IOASCR_G8_IO1                   ((uint32_t)0x10000000)            /*!<GROUP8_IO1 analog switch enable */
#define TSC_IOASCR_G8_IO2                   ((uint32_t)0x20000000)            /*!<GROUP8_IO2 analog switch enable */
#define TSC_IOASCR_G8_IO3                   ((uint32_t)0x40000000)            /*!<GROUP8_IO3 analog switch enable */
#define TSC_IOASCR_G8_IO4                   ((uint32_t)0x80000000)            /*!<GROUP8_IO4 analog switch enable */

/*******************  Bit definition for TSC_IOSCR register  ******************/
#define TSC_IOSCR_G1_IO1                    ((uint32_t)0x00000001)            /*!<GROUP1_IO1 sampling mode */
#define TSC_IOSCR_G1_IO2                    ((uint32_t)0x00000002)            /*!<GROUP1_IO2 sampling mode */
#define TSC_IOSCR_G1_IO3                    ((uint32_t)0x00000004)            /*!<GROUP1_IO3 sampling mode */
#define TSC_IOSCR_G1_IO4                    ((uint32_t)0x00000008)            /*!<GROUP1_IO4 sampling mode */
#define TSC_IOSCR_G2_IO1                    ((uint32_t)0x00000010)            /*!<GROUP2_IO1 sampling mode */
#define TSC_IOSCR_G2_IO2                    ((uint32_t)0x00000020)            /*!<GROUP2_IO2 sampling mode */
#define TSC_IOSCR_G2_IO3                    ((uint32_t)0x00000040)            /*!<GROUP2_IO3 sampling mode */
#define TSC_IOSCR_G2_IO4                    ((uint32_t)0x00000080)            /*!<GROUP2_IO4 sampling mode */
#define TSC_IOSCR_G3_IO1                    ((uint32_t)0x00000100)            /*!<GROUP3_IO1 sampling mode */
#define TSC_IOSCR_G3_IO2                    ((uint32_t)0x00000200)            /*!<GROUP3_IO2 sampling mode */
#define TSC_IOSCR_G3_IO3                    ((uint32_t)0x00000400)            /*!<GROUP3_IO3 sampling mode */
#define TSC_IOSCR_G3_IO4                    ((uint32_t)0x00000800)            /*!<GROUP3_IO4 sampling mode */
#define TSC_IOSCR_G4_IO1                    ((uint32_t)0x00001000)            /*!<GROUP4_IO1 sampling mode */
#define TSC_IOSCR_G4_IO2                    ((uint32_t)0x00002000)            /*!<GROUP4_IO2 sampling mode */
#define TSC_IOSCR_G4_IO3                    ((uint32_t)0x00004000)            /*!<GROUP4_IO3 sampling mode */
#define TSC_IOSCR_G4_IO4                    ((uint32_t)0x00008000)            /*!<GROUP4_IO4 sampling mode */
#define TSC_IOSCR_G5_IO1                    ((uint32_t)0x00010000)            /*!<GROUP5_IO1 sampling mode */
#define TSC_IOSCR_G5_IO2                    ((uint32_t)0x00020000)            /*!<GROUP5_IO2 sampling mode */
#define TSC_IOSCR_G5_IO3                    ((uint32_t)0x00040000)            /*!<GROUP5_IO3 sampling mode */
#define TSC_IOSCR_G5_IO4                    ((uint32_t)0x00080000)            /*!<GROUP5_IO4 sampling mode */
#define TSC_IOSCR_G6_IO1                    ((uint32_t)0x00100000)            /*!<GROUP6_IO1 sampling mode */
#define TSC_IOSCR_G6_IO2                    ((uint32_t)0x00200000)            /*!<GROUP6_IO2 sampling mode */
#define TSC_IOSCR_G6_IO3                    ((uint32_t)0x00400000)            /*!<GROUP6_IO3 sampling mode */
#define TSC_IOSCR_G6_IO4                    ((uint32_t)0x00800000)            /*!<GROUP6_IO4 sampling mode */
#define TSC_IOSCR_G7_IO1                    ((uint32_t)0x01000000)            /*!<GROUP7_IO1 sampling mode */
#define TSC_IOSCR_G7_IO2                    ((uint32_t)0x02000000)            /*!<GROUP7_IO2 sampling mode */
#define TSC_IOSCR_G7_IO3                    ((uint32_t)0x04000000)            /*!<GROUP7_IO3 sampling mode */
#define TSC_IOSCR_G7_IO4                    ((uint32_t)0x08000000)            /*!<GROUP7_IO4 sampling mode */
#define TSC_IOSCR_G8_IO1                    ((uint32_t)0x10000000)            /*!<GROUP8_IO1 sampling mode */
#define TSC_IOSCR_G8_IO2                    ((uint32_t)0x20000000)            /*!<GROUP8_IO2 sampling mode */
#define TSC_IOSCR_G8_IO3                    ((uint32_t)0x40000000)            /*!<GROUP8_IO3 sampling mode */
#define TSC_IOSCR_G8_IO4                    ((uint32_t)0x80000000)            /*!<GROUP8_IO4 sampling mode */

/*******************  Bit definition for TSC_IOCCR register  ******************/
#define TSC_IOCCR_G1_IO1                    ((uint32_t)0x00000001)            /*!<GROUP1_IO1 channel mode */
#define TSC_IOCCR_G1_IO2                    ((uint32_t)0x00000002)            /*!<GROUP1_IO2 channel mode */
#define TSC_IOCCR_G1_IO3                    ((uint32_t)0x00000004)            /*!<GROUP1_IO3 channel mode */
#define TSC_IOCCR_G1_IO4                    ((uint32_t)0x00000008)            /*!<GROUP1_IO4 channel mode */
#define TSC_IOCCR_G2_IO1                    ((uint32_t)0x00000010)            /*!<GROUP2_IO1 channel mode */
#define TSC_IOCCR_G2_IO2                    ((uint32_t)0x00000020)            /*!<GROUP2_IO2 channel mode */
#define TSC_IOCCR_G2_IO3                    ((uint32_t)0x00000040)            /*!<GROUP2_IO3 channel mode */
#define TSC_IOCCR_G2_IO4                    ((uint32_t)0x00000080)            /*!<GROUP2_IO4 channel mode */
#define TSC_IOCCR_G3_IO1                    ((uint32_t)0x00000100)            /*!<GROUP3_IO1 channel mode */
#define TSC_IOCCR_G3_IO2                    ((uint32_t)0x00000200)            /*!<GROUP3_IO2 channel mode */
#define TSC_IOCCR_G3_IO3                    ((uint32_t)0x00000400)            /*!<GROUP3_IO3 channel mode */
#define TSC_IOCCR_G3_IO4                    ((uint32_t)0x00000800)            /*!<GROUP3_IO4 channel mode */
#define TSC_IOCCR_G4_IO1                    ((uint32_t)0x00001000)            /*!<GROUP4_IO1 channel mode */
#define TSC_IOCCR_G4_IO2                    ((uint32_t)0x00002000)            /*!<GROUP4_IO2 channel mode */
#define TSC_IOCCR_G4_IO3                    ((uint32_t)0x00004000)            /*!<GROUP4_IO3 channel mode */
#define TSC_IOCCR_G4_IO4                    ((uint32_t)0x00008000)            /*!<GROUP4_IO4 channel mode */
#define TSC_IOCCR_G5_IO1                    ((uint32_t)0x00010000)            /*!<GROUP5_IO1 channel mode */
#define TSC_IOCCR_G5_IO2                    ((uint32_t)0x00020000)            /*!<GROUP5_IO2 channel mode */
#define TSC_IOCCR_G5_IO3                    ((uint32_t)0x00040000)            /*!<GROUP5_IO3 channel mode */
#define TSC_IOCCR_G5_IO4                    ((uint32_t)0x00080000)            /*!<GROUP5_IO4 channel mode */
#define TSC_IOCCR_G6_IO1                    ((uint32_t)0x00100000)            /*!<GROUP6_IO1 channel mode */
#define TSC_IOCCR_G6_IO2                    ((uint32_t)0x00200000)            /*!<GROUP6_IO2 channel mode */
#define TSC_IOCCR_G6_IO3                    ((uint32_t)0x00400000)            /*!<GROUP6_IO3 channel mode */
#define TSC_IOCCR_G6_IO4                    ((uint32_t)0x00800000)            /*!<GROUP6_IO4 channel mode */
#define TSC_IOCCR_G7_IO1                    ((uint32_t)0x01000000)            /*!<GROUP7_IO1 channel mode */
#define TSC_IOCCR_G7_IO2                    ((uint32_t)0x02000000)            /*!<GROUP7_IO2 channel mode */
#define TSC_IOCCR_G7_IO3                    ((uint32_t)0x04000000)            /*!<GROUP7_IO3 channel mode */
#define TSC_IOCCR_G7_IO4                    ((uint32_t)0x08000000)            /*!<GROUP7_IO4 channel mode */
#define TSC_IOCCR_G8_IO1                    ((uint32_t)0x10000000)            /*!<GROUP8_IO1 channel mode */
#define TSC_IOCCR_G8_IO2                    ((uint32_t)0x20000000)            /*!<GROUP8_IO2 channel mode */
#define TSC_IOCCR_G8_IO3                    ((uint32_t)0x40000000)            /*!<GROUP8_IO3 channel mode */
#define TSC_IOCCR_G8_IO4                    ((uint32_t)0x80000000)            /*!<GROUP8_IO4 channel mode */

/*******************  Bit definition for TSC_IOGCSR register  *****************/
#define TSC_IOGCSR_G1E                      ((uint32_t)0x00000001)            /*!<Analog IO GROUP1 enable */
#define TSC_IOGCSR_G2E                      ((uint32_t)0x00000002)            /*!<Analog IO GROUP2 enable */
#define TSC_IOGCSR_G3E                      ((uint32_t)0x00000004)            /*!<Analog IO GROUP3 enable */
#define TSC_IOGCSR_G4E                      ((uint32_t)0x00000008)            /*!<Analog IO GROUP4 enable */
#define TSC_IOGCSR_G5E                      ((uint32_t)0x00000010)            /*!<Analog IO GROUP5 enable */
#define TSC_IOGCSR_G6E                      ((uint32_t)0x00000020)            /*!<Analog IO GROUP6 enable */
#define TSC_IOGCSR_G7E                      ((uint32_t)0x00000040)            /*!<Analog IO GROUP7 enable */
#define TSC_IOGCSR_G8E                      ((uint32_t)0x00000080)            /*!<Analog IO GROUP8 enable */
#define TSC_IOGCSR_G1S                      ((uint32_t)0x00010000)            /*!<Analog IO GROUP1 status */
#define TSC_IOGCSR_G2S                      ((uint32_t)0x00020000)            /*!<Analog IO GROUP2 status */
#define TSC_IOGCSR_G3S                      ((uint32_t)0x00040000)            /*!<Analog IO GROUP3 status */
#define TSC_IOGCSR_G4S                      ((uint32_t)0x00080000)            /*!<Analog IO GROUP4 status */
#define TSC_IOGCSR_G5S                      ((uint32_t)0x00100000)            /*!<Analog IO GROUP5 status */
#define TSC_IOGCSR_G6S                      ((uint32_t)0x00200000)            /*!<Analog IO GROUP6 status */
#define TSC_IOGCSR_G7S                      ((uint32_t)0x00400000)            /*!<Analog IO GROUP7 status */
#define TSC_IOGCSR_G8S                      ((uint32_t)0x00800000)            /*!<Analog IO GROUP8 status */

/*******************  Bit definition for TSC_IOGXCR register  *****************/
#define TSC_IOGXCR_CNT                      ((uint32_t)0x00003FFF)            /*!<CNT[13:0] bits (Counter value) */

/******************************************************************************/
/*                                                                            */
/*      Universal Synchronous Asynchronous Receiver Transmitter (USART)       */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_UE                        ((uint32_t)0x00000001)            /*!< USART Enable */
#define  USART_CR1_UESM                      ((uint32_t)0x00000002)            /*!< USART Enable in STOP Mode */
#define  USART_CR1_RE                        ((uint32_t)0x00000004)            /*!< Receiver Enable */
#define  USART_CR1_TE                        ((uint32_t)0x00000008)            /*!< Transmitter Enable */
#define  USART_CR1_IDLEIE                    ((uint32_t)0x00000010)            /*!< IDLE Interrupt Enable */
#define  USART_CR1_RXNEIE                    ((uint32_t)0x00000020)            /*!< RXNE Interrupt Enable */
#define  USART_CR1_TCIE                      ((uint32_t)0x00000040)            /*!< Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     ((uint32_t)0x00000080)            /*!< TXE Interrupt Enable */
#define  USART_CR1_PEIE                      ((uint32_t)0x00000100)            /*!< PE Interrupt Enable */
#define  USART_CR1_PS                        ((uint32_t)0x00000200)            /*!< Parity Selection */
#define  USART_CR1_PCE                       ((uint32_t)0x00000400)            /*!< Parity Control Enable */
#define  USART_CR1_WAKE                      ((uint32_t)0x00000800)            /*!< Receiver Wakeup method */
#define  USART_CR1_M                         ((uint32_t)0x10001000)            /*!< Word length */
#define  USART_CR1_M0                        ((uint32_t)0x00001000)            /*!< Word length - Bit 0 */
#define  USART_CR1_MME                       ((uint32_t)0x00002000)            /*!< Mute Mode Enable */
#define  USART_CR1_CMIE                      ((uint32_t)0x00004000)            /*!< Character match interrupt enable */
#define  USART_CR1_OVER8                     ((uint32_t)0x00008000)            /*!< Oversampling by 8-bit or 16-bit mode */
#define  USART_CR1_DEDT                      ((uint32_t)0x001F0000)            /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
#define  USART_CR1_DEDT_0                    ((uint32_t)0x00010000)            /*!< Bit 0 */
#define  USART_CR1_DEDT_1                    ((uint32_t)0x00020000)            /*!< Bit 1 */
#define  USART_CR1_DEDT_2                    ((uint32_t)0x00040000)            /*!< Bit 2 */
#define  USART_CR1_DEDT_3                    ((uint32_t)0x00080000)            /*!< Bit 3 */
#define  USART_CR1_DEDT_4                    ((uint32_t)0x00100000)            /*!< Bit 4 */
#define  USART_CR1_DEAT                      ((uint32_t)0x03E00000)            /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
#define  USART_CR1_DEAT_0                    ((uint32_t)0x00200000)            /*!< Bit 0 */
#define  USART_CR1_DEAT_1                    ((uint32_t)0x00400000)            /*!< Bit 1 */
#define  USART_CR1_DEAT_2                    ((uint32_t)0x00800000)            /*!< Bit 2 */
#define  USART_CR1_DEAT_3                    ((uint32_t)0x01000000)            /*!< Bit 3 */
#define  USART_CR1_DEAT_4                    ((uint32_t)0x02000000)            /*!< Bit 4 */
#define  USART_CR1_RTOIE                     ((uint32_t)0x04000000)            /*!< Receive Time Out interrupt enable */
#define  USART_CR1_EOBIE                     ((uint32_t)0x08000000)            /*!< End of Block interrupt enable */
#define  USART_CR1_M1                        ((uint32_t)0x10000000)            /*!< Word length - Bit 1 */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADDM7                     ((uint32_t)0x00000010)            /*!< 7-bit or 4-bit Address Detection */
#define  USART_CR2_LBDL                      ((uint32_t)0x00000020)            /*!< LIN Break Detection Length */
#define  USART_CR2_LBDIE                     ((uint32_t)0x00000040)            /*!< LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      ((uint32_t)0x00000100)            /*!< Last Bit Clock pulse */
#define  USART_CR2_CPHA                      ((uint32_t)0x00000200)            /*!< Clock Phase */
#define  USART_CR2_CPOL                      ((uint32_t)0x00000400)            /*!< Clock Polarity */
#define  USART_CR2_CLKEN                     ((uint32_t)0x00000800)            /*!< Clock Enable */
#define  USART_CR2_STOP                      ((uint32_t)0x00003000)            /*!< STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    ((uint32_t)0x00001000)            /*!< Bit 0 */
#define  USART_CR2_STOP_1                    ((uint32_t)0x00002000)            /*!< Bit 1 */
#define  USART_CR2_LINEN                     ((uint32_t)0x00004000)            /*!< LIN mode enable */
#define  USART_CR2_SWAP                      ((uint32_t)0x00008000)            /*!< SWAP TX/RX pins */
#define  USART_CR2_RXINV                     ((uint32_t)0x00010000)            /*!< RX pin active level inversion */
#define  USART_CR2_TXINV                     ((uint32_t)0x00020000)            /*!< TX pin active level inversion */
#define  USART_CR2_DATAINV                   ((uint32_t)0x00040000)            /*!< Binary data inversion */
#define  USART_CR2_MSBFIRST                  ((uint32_t)0x00080000)            /*!< Most Significant Bit First */
#define  USART_CR2_ABREN                     ((uint32_t)0x00100000)            /*!< Auto Baud-Rate Enable*/
#define  USART_CR2_ABRMODE                   ((uint32_t)0x00600000)            /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
#define  USART_CR2_ABRMODE_0                 ((uint32_t)0x00200000)            /*!< Bit 0 */
#define  USART_CR2_ABRMODE_1                 ((uint32_t)0x00400000)            /*!< Bit 1 */
#define  USART_CR2_RTOEN                     ((uint32_t)0x00800000)            /*!< Receiver Time-Out enable */
#define  USART_CR2_ADD                       ((uint32_t)0xFF000000)            /*!< Address of the USART node */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       ((uint32_t)0x00000001)            /*!< Error Interrupt Enable */
#define  USART_CR3_IREN                      ((uint32_t)0x00000002)            /*!< IrDA mode Enable */
#define  USART_CR3_IRLP                      ((uint32_t)0x00000004)            /*!< IrDA Low-Power */
#define  USART_CR3_HDSEL                     ((uint32_t)0x00000008)            /*!< Half-Duplex Selection */
#define  USART_CR3_NACK                      ((uint32_t)0x00000010)            /*!< SmartCard NACK enable */
#define  USART_CR3_SCEN                      ((uint32_t)0x00000020)            /*!< SmartCard mode enable */
#define  USART_CR3_DMAR                      ((uint32_t)0x00000040)            /*!< DMA Enable Receiver */
#define  USART_CR3_DMAT                      ((uint32_t)0x00000080)            /*!< DMA Enable Transmitter */
#define  USART_CR3_RTSE                      ((uint32_t)0x00000100)            /*!< RTS Enable */
#define  USART_CR3_CTSE                      ((uint32_t)0x00000200)            /*!< CTS Enable */
#define  USART_CR3_CTSIE                     ((uint32_t)0x00000400)            /*!< CTS Interrupt Enable */
#define  USART_CR3_ONEBIT                    ((uint32_t)0x00000800)            /*!< One sample bit method enable */
#define  USART_CR3_OVRDIS                    ((uint32_t)0x00001000)            /*!< Overrun Disable */
#define  USART_CR3_DDRE                      ((uint32_t)0x00002000)            /*!< DMA Disable on Reception Error */
#define  USART_CR3_DEM                       ((uint32_t)0x00004000)            /*!< Driver Enable Mode */
#define  USART_CR3_DEP                       ((uint32_t)0x00008000)            /*!< Driver Enable Polarity Selection */
#define  USART_CR3_SCARCNT                   ((uint32_t)0x000E0000)            /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
#define  USART_CR3_SCARCNT_0                 ((uint32_t)0x00020000)            /*!< Bit 0 */
#define  USART_CR3_SCARCNT_1                 ((uint32_t)0x00040000)            /*!< Bit 1 */
#define  USART_CR3_SCARCNT_2                 ((uint32_t)0x00080000)            /*!< Bit 2 */
#define  USART_CR3_WUS                       ((uint32_t)0x00300000)            /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
#define  USART_CR3_WUS_0                     ((uint32_t)0x00100000)            /*!< Bit 0 */
#define  USART_CR3_WUS_1                     ((uint32_t)0x00200000)            /*!< Bit 1 */
#define  USART_CR3_WUFIE                     ((uint32_t)0x00400000)            /*!< Wake Up Interrupt Enable */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_FRACTION              ((uint16_t)0x000F)                /*!< Fraction of USARTDIV */
#define  USART_BRR_DIV_MANTISSA              ((uint16_t)0xFFF0)                /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      ((uint32_t)0x000000FF)            /*!< PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_GT                       ((uint32_t)0x0000FF00)            /*!< GT[7:0] bits (Guard time value) */


/*******************  Bit definition for USART_RTOR register  *****************/
#define  USART_RTOR_RTO                      ((uint32_t)0x00FFFFFF)            /*!< Receiver Time Out Value */
#define  USART_RTOR_BLEN                     ((uint32_t)0xFF000000)            /*!< Block Length */

/*******************  Bit definition for USART_RQR register  ******************/
#define  USART_RQR_ABRRQ                     ((uint16_t)0x0001)                /*!< Auto-Baud Rate Request */
#define  USART_RQR_SBKRQ                     ((uint16_t)0x0002)                /*!< Send Break Request */
#define  USART_RQR_MMRQ                      ((uint16_t)0x0004)                /*!< Mute Mode Request */
#define  USART_RQR_RXFRQ                     ((uint16_t)0x0008)                /*!< Receive Data flush Request */
#define  USART_RQR_TXFRQ                     ((uint16_t)0x0010)                /*!< Transmit data flush Request */

/*******************  Bit definition for USART_ISR register  ******************/
#define  USART_ISR_PE                        ((uint32_t)0x00000001)            /*!< Parity Error */
#define  USART_ISR_FE                        ((uint32_t)0x00000002)            /*!< Framing Error */
#define  USART_ISR_NE                        ((uint32_t)0x00000004)            /*!< Noise detected Flag */
#define  USART_ISR_ORE                       ((uint32_t)0x00000008)            /*!< OverRun Error */
#define  USART_ISR_IDLE                      ((uint32_t)0x00000010)            /*!< IDLE line detected */
#define  USART_ISR_RXNE                      ((uint32_t)0x00000020)            /*!< Read Data Register Not Empty */
#define  USART_ISR_TC                        ((uint32_t)0x00000040)            /*!< Transmission Complete */
#define  USART_ISR_TXE                       ((uint32_t)0x00000080)            /*!< Transmit Data Register Empty */
#define  USART_ISR_LBDF                      ((uint32_t)0x00000100)            /*!< LIN Break Detection Flag */
#define  USART_ISR_CTSIF                     ((uint32_t)0x00000200)            /*!< CTS interrupt flag */
#define  USART_ISR_CTS                       ((uint32_t)0x00000400)            /*!< CTS flag */
#define  USART_ISR_RTOF                      ((uint32_t)0x00000800)            /*!< Receiver Time Out */
#define  USART_ISR_EOBF                      ((uint32_t)0x00001000)            /*!< End Of Block Flag */
#define  USART_ISR_ABRE                      ((uint32_t)0x00004000)            /*!< Auto-Baud Rate Error */
#define  USART_ISR_ABRF                      ((uint32_t)0x00008000)            /*!< Auto-Baud Rate Flag */
#define  USART_ISR_BUSY                      ((uint32_t)0x00010000)            /*!< Busy Flag */
#define  USART_ISR_CMF                       ((uint32_t)0x00020000)            /*!< Character Match Flag */
#define  USART_ISR_SBKF                      ((uint32_t)0x00040000)            /*!< Send Break Flag */
#define  USART_ISR_RWU                       ((uint32_t)0x00080000)            /*!< Receive Wake Up from mute mode Flag */
#define  USART_ISR_WUF                       ((uint32_t)0x00100000)            /*!< Wake Up from stop mode Flag */
#define  USART_ISR_TEACK                     ((uint32_t)0x00200000)            /*!< Transmit Enable Acknowledge Flag */
#define  USART_ISR_REACK                     ((uint32_t)0x00400000)            /*!< Receive Enable Acknowledge Flag */

/*******************  Bit definition for USART_ICR register  ******************/
#define  USART_ICR_PECF                      ((uint32_t)0x00000001)            /*!< Parity Error Clear Flag */
#define  USART_ICR_FECF                      ((uint32_t)0x00000002)            /*!< Framing Error Clear Flag */
#define  USART_ICR_NCF                       ((uint32_t)0x00000004)            /*!< Noise detected Clear Flag */
#define  USART_ICR_ORECF                     ((uint32_t)0x00000008)            /*!< OverRun Error Clear Flag */
#define  USART_ICR_IDLECF                    ((uint32_t)0x00000010)            /*!< IDLE line detected Clear Flag */
#define  USART_ICR_TCCF                      ((uint32_t)0x00000040)            /*!< Transmission Complete Clear Flag */
#define  USART_ICR_LBDCF                     ((uint32_t)0x00000100)            /*!< LIN Break Detection Clear Flag */
#define  USART_ICR_CTSCF                     ((uint32_t)0x00000200)            /*!< CTS Interrupt Clear Flag */
#define  USART_ICR_RTOCF                     ((uint32_t)0x00000800)            /*!< Receiver Time Out Clear Flag */
#define  USART_ICR_EOBCF                     ((uint32_t)0x00001000)            /*!< End Of Block Clear Flag */
#define  USART_ICR_CMCF                      ((uint32_t)0x00020000)            /*!< Character Match Clear Flag */
#define  USART_ICR_WUCF                      ((uint32_t)0x00100000)            /*!< Wake Up from stop mode Clear Flag */

/*******************  Bit definition for USART_RDR register  ******************/
#define  USART_RDR_RDR                       ((uint16_t)0x01FF)                /*!< RDR[8:0] bits (Receive Data value) */

/*******************  Bit definition for USART_TDR register  ******************/
#define  USART_TDR_TDR                       ((uint16_t)0x01FF)                /*!< TDR[8:0] bits (Transmit Data value) */

/******************************************************************************/
/*                                                                            */
/*           Single Wire Protocol Master Interface (SWPMI)                    */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for SWPMI_CR register   ********************/
#define  SWPMI_CR_RXDMA                      ((uint32_t)0x00000001)        /*!<Reception DMA enable                                 */
#define  SWPMI_CR_TXDMA                      ((uint32_t)0x00000002)        /*!<Transmission DMA enable                              */
#define  SWPMI_CR_RXMODE                     ((uint32_t)0x00000004)        /*!<Reception buffering mode                             */
#define  SWPMI_CR_TXMODE                     ((uint32_t)0x00000008)        /*!<Transmission buffering mode                          */
#define  SWPMI_CR_LPBK                       ((uint32_t)0x00000010)        /*!<Loopback mode enable                                 */
#define  SWPMI_CR_SWPACT                     ((uint32_t)0x00000020)        /*!<Single wire protocol master interface activate       */
#define  SWPMI_CR_DEACT                      ((uint32_t)0x00000400)        /*!<Single wire protocol master interface deactivate     */

/*******************  Bit definition for SWPMI_BRR register  ********************/
#define  SWPMI_BRR_BR                        ((uint32_t)0x0000003F)        /*!<BR[5:0] bits (Bitrate prescaler) */

/*******************  Bit definition for SWPMI_ISR register  ********************/
#define  SWPMI_ISR_RXBFF                     ((uint32_t)0x00000001)        /*!<Receive buffer full flag        */
#define  SWPMI_ISR_TXBEF                     ((uint32_t)0x00000002)        /*!<Transmit buffer empty flag      */
#define  SWPMI_ISR_RXBERF                    ((uint32_t)0x00000004)        /*!<Receive CRC error flag          */
#define  SWPMI_ISR_RXOVRF                    ((uint32_t)0x00000008)        /*!<Receive overrun error flag      */
#define  SWPMI_ISR_TXUNRF                    ((uint32_t)0x00000010)        /*!<Transmit underrun error flag    */
#define  SWPMI_ISR_RXNE                      ((uint32_t)0x00000020)        /*!<Receive data register not empty */
#define  SWPMI_ISR_TXE                       ((uint32_t)0x00000040)        /*!<Transmit data register empty    */
#define  SWPMI_ISR_TCF                       ((uint32_t)0x00000080)        /*!<Transfer complete flag          */
#define  SWPMI_ISR_SRF                       ((uint32_t)0x00000100)        /*!<Slave resume flag               */
#define  SWPMI_ISR_SUSP                      ((uint32_t)0x00000200)        /*!<SUSPEND flag                    */
#define  SWPMI_ISR_DEACTF                    ((uint32_t)0x00000400)        /*!<DEACTIVATED flag                */

/*******************  Bit definition for SWPMI_ICR register  ********************/
#define  SWPMI_ICR_CRXBFF                    ((uint32_t)0x00000001)        /*!<Clear receive buffer full flag       */
#define  SWPMI_ICR_CTXBEF                    ((uint32_t)0x00000002)        /*!<Clear transmit buffer empty flag     */
#define  SWPMI_ICR_CRXBERF                   ((uint32_t)0x00000004)        /*!<Clear receive CRC error flag         */
#define  SWPMI_ICR_CRXOVRF                   ((uint32_t)0x00000008)        /*!<Clear receive overrun error flag     */
#define  SWPMI_ICR_CTXUNRF                   ((uint32_t)0x00000010)        /*!<Clear transmit underrun error flag   */
#define  SWPMI_ICR_CTCF                      ((uint32_t)0x00000080)        /*!<Clear transfer complete flag         */
#define  SWPMI_ICR_CSRF                      ((uint32_t)0x00000100)        /*!<Clear slave resume flag              */

/*******************  Bit definition for SWPMI_IER register  ********************/
#define  SWPMI_IER_SRIE                      ((uint32_t)0x00000100)        /*!<Slave resume interrupt enable               */
#define  SWPMI_IER_TCIE                      ((uint32_t)0x00000080)        /*!<Transmit complete interrupt enable          */
#define  SWPMI_IER_TIE                       ((uint32_t)0x00000040)        /*!<Transmit interrupt enable                   */
#define  SWPMI_IER_RIE                       ((uint32_t)0x00000020)        /*!<Receive interrupt enable                    */
#define  SWPMI_IER_TXUNRIE                   ((uint32_t)0x00000010)        /*!<Transmit underrun error interrupt enable    */
#define  SWPMI_IER_RXOVRIE                   ((uint32_t)0x00000008)        /*!<Receive overrun error interrupt enable      */
#define  SWPMI_IER_RXBERIE                   ((uint32_t)0x00000004)        /*!<Receive CRC error interrupt enable          */
#define  SWPMI_IER_TXBEIE                    ((uint32_t)0x00000002)        /*!<Transmit buffer empty interrupt enable      */
#define  SWPMI_IER_RXBFIE                    ((uint32_t)0x00000001)        /*!<Receive buffer full interrupt enable        */

/*******************  Bit definition for SWPMI_RFL register  ********************/
#define  SWPMI_RFL_RFL                       ((uint32_t)0x0000001F)        /*!<RFL[4:0] bits (Receive Frame length) */
#define  SWPMI_RFL_RFL_0_1                   ((uint32_t)0x00000003)        /*!<RFL[1:0] bits (number of relevant bytes for the last SWPMI_RDR register read.) */

/*******************  Bit definition for SWPMI_TDR register  ********************/
#define  SWPMI_TDR_TD                        ((uint32_t)0xFFFFFFFF)        /*!<Transmit Data Register         */

/*******************  Bit definition for SWPMI_RDR register  ********************/
#define  SWPMI_RDR_RD                        ((uint32_t)0xFFFFFFFF)        /*!<Receive Data Register          */

/*******************  Bit definition for SWPMI_OR register  ********************/
#define  SWPMI_OR_TBYP                       ((uint32_t)0x00000001)        /*!<SWP Transceiver Bypass */
#define  SWPMI_OR_CLASS                      ((uint32_t)0x00000002)        /*!<SWP Voltage Class selection */

/******************************************************************************/
/*                                                                            */
/*                                 VREFBUF                                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for VREFBUF_CSR register  ****************/
#define  VREFBUF_CSR_ENVR                    ((uint32_t)0x00000001)        /*!<Voltage reference buffer enable */
#define  VREFBUF_CSR_HIZ                     ((uint32_t)0x00000002)        /*!<High impedance mode             */
#define  VREFBUF_CSR_VRS                     ((uint32_t)0x00000004)        /*!<Voltage reference scale         */
#define  VREFBUF_CSR_VRR                     ((uint32_t)0x00000008)        /*!<Voltage reference buffer ready  */

/*******************  Bit definition for VREFBUF_CCR register  ******************/
#define  VREFBUF_CCR_TRIM                    ((uint32_t)0x0000003F)        /*!<TRIM[5:0] bits (Trimming code)  */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
#define  WWDG_CR_T                           ((uint32_t)0x0000007F)        /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CR_T_0                         ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  WWDG_CR_T_1                         ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  WWDG_CR_T_2                         ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  WWDG_CR_T_3                         ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  WWDG_CR_T_4                         ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  WWDG_CR_T_5                         ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  WWDG_CR_T_6                         ((uint32_t)0x00000040)        /*!<Bit 6 */

#define  WWDG_CR_WDGA                        ((uint32_t)0x00000080)        /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define  WWDG_CFR_W                          ((uint32_t)0x0000007F)        /*!<W[6:0] bits (7-bit window value) */
#define  WWDG_CFR_W_0                        ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  WWDG_CFR_W_1                        ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  WWDG_CFR_W_2                        ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  WWDG_CFR_W_3                        ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  WWDG_CFR_W_4                        ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  WWDG_CFR_W_5                        ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  WWDG_CFR_W_6                        ((uint32_t)0x00000040)        /*!<Bit 6 */

#define  WWDG_CFR_WDGTB                      ((uint32_t)0x00000180)        /*!<WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFR_WDGTB_0                    ((uint32_t)0x00000080)        /*!<Bit 0 */
#define  WWDG_CFR_WDGTB_1                    ((uint32_t)0x00000100)        /*!<Bit 1 */

#define  WWDG_CFR_EWI                        ((uint32_t)0x00000200)        /*!<Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define  WWDG_SR_EWIF                        ((uint32_t)0x00000001)        /*!<Early Wakeup Interrupt Flag */


/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DBGMCU_IDCODE register  *************/
#define  DBGMCU_IDCODE_DEV_ID                ((uint32_t)0x0000FFFF)
#define  DBGMCU_IDCODE_REV_ID                ((uint32_t)0xFFFF0000)

/********************  Bit definition for DBGMCU_CR register  *****************/
#define  DBGMCU_CR_DBG_SLEEP                 ((uint32_t)0x00000001)
#define  DBGMCU_CR_DBG_STOP                  ((uint32_t)0x00000002)
#define  DBGMCU_CR_DBG_STANDBY               ((uint32_t)0x00000004)
#define  DBGMCU_CR_TRACE_IOEN                ((uint32_t)0x00000020)

#define  DBGMCU_CR_TRACE_MODE                ((uint32_t)0x000000C0)
#define  DBGMCU_CR_TRACE_MODE_0              ((uint32_t)0x00000040)/*!<Bit 0 */
#define  DBGMCU_CR_TRACE_MODE_1              ((uint32_t)0x00000080)/*!<Bit 1 */

/********************  Bit definition for DBGMCU_APB1FZR1 register  ***********/
#define  DBGMCU_APB1FZR1_DBG_TIM2_STOP       ((uint32_t)0x00000001)
#define  DBGMCU_APB1FZR1_DBG_TIM3_STOP       ((uint32_t)0x00000002)
#define  DBGMCU_APB1FZR1_DBG_TIM4_STOP       ((uint32_t)0x00000004)
#define  DBGMCU_APB1FZR1_DBG_TIM5_STOP       ((uint32_t)0x00000008)
#define  DBGMCU_APB1FZR1_DBG_TIM6_STOP       ((uint32_t)0x00000010)
#define  DBGMCU_APB1FZR1_DBG_TIM7_STOP       ((uint32_t)0x00000020)
#define  DBGMCU_APB1FZR1_DBG_RTC_STOP        ((uint32_t)0x00000400)
#define  DBGMCU_APB1FZR1_DBG_WWDG_STOP       ((uint32_t)0x00000800)
#define  DBGMCU_APB1FZR1_DBG_IWDG_STOP       ((uint32_t)0x00001000)
#define  DBGMCU_APB1FZR1_DBG_I2C1_STOP       ((uint32_t)0x00200000)
#define  DBGMCU_APB1FZR1_DBG_I2C2_STOP       ((uint32_t)0x00400000)
#define  DBGMCU_APB1FZR1_DBG_I2C3_STOP       ((uint32_t)0x00800000)
#define  DBGMCU_APB1FZR1_DBG_CAN_STOP        ((uint32_t)0x02000000)
#define  DBGMCU_APB1FZR1_DBG_LPTIM1_STOP     ((uint32_t)0x80000000)

/********************  Bit definition for DBGMCU_APB1FZR2 register  **********/
#define  DBGMCU_APB1FZR2_DBG_LPTIM2_STOP     ((uint32_t)0x00000020)

/********************  Bit definition for DBGMCU_APB2FZ register  ************/
#define  DBGMCU_APB2FZ_DBG_TIM1_STOP         ((uint32_t)0x00000800)
#define  DBGMCU_APB2FZ_DBG_TIM8_STOP         ((uint32_t)0x00002000)
#define  DBGMCU_APB2FZ_DBG_TIM15_STOP        ((uint32_t)0x00010000)
#define  DBGMCU_APB2FZ_DBG_TIM16_STOP        ((uint32_t)0x00020000)
#define  DBGMCU_APB2FZ_DBG_TIM17_STOP        ((uint32_t)0x00040000)

/******************************************************************************/
/*                                                                            */
/*                                       USB_OTG                              */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for USB_OTG_GOTGCTL register  ********************/
#define USB_OTG_GOTGCTL_SRQSCS                  ((uint32_t)0x00000001)         /*!< Session request success */
#define USB_OTG_GOTGCTL_SRQ                     ((uint32_t)0x00000002)         /*!< Session request */
#define USB_OTG_GOTGCTL_VBVALOEN                ((uint32_t)0x00000004)         /*!< VBUS valid override enable */
#define USB_OTG_GOTGCTL_VBVALOVAL               ((uint32_t)0x00000008)         /*!< VBUS valid override value */
#define USB_OTG_GOTGCTL_AVALOEN                 ((uint32_t)0x00000010)         /*!< A-peripheral session valid override enable */
#define USB_OTG_GOTGCTL_AVALOVAL                ((uint32_t)0x00000020)         /*!< A-peripheral session valid override value */
#define USB_OTG_GOTGCTL_BVALOEN                 ((uint32_t)0x00000040)         /*!< B-peripheral session valid override enable */
#define USB_OTG_GOTGCTL_BVALOVAL                ((uint32_t)0x00000080)         /*!< B-peripheral session valid override value  */
#define USB_OTG_GOTGCTL_BSESVLD                 ((uint32_t)0x00080000)         /*!<  B-session valid*/

/********************  Bit definition for USB_OTG_HCFG register  ********************/

#define USB_OTG_HCFG_FSLSPCS                 ((uint32_t)0x00000003)            /*!< FS/LS PHY clock select */
#define USB_OTG_HCFG_FSLSPCS_0               ((uint32_t)0x00000001)            /*!<Bit 0 */
#define USB_OTG_HCFG_FSLSPCS_1               ((uint32_t)0x00000002)            /*!<Bit 1 */
#define USB_OTG_HCFG_FSLSS                   ((uint32_t)0x00000004)            /*!< FS- and LS-only support */

/********************  Bit definition for USB_OTG_DCFG register  ********************/

#define USB_OTG_DCFG_DSPD                    ((uint32_t)0x00000003)            /*!< Device speed */
#define USB_OTG_DCFG_DSPD_0                  ((uint32_t)0x00000001)            /*!<Bit 0 */
#define USB_OTG_DCFG_DSPD_1                  ((uint32_t)0x00000002)            /*!<Bit 1 */
#define USB_OTG_DCFG_NZLSOHSK                ((uint32_t)0x00000004)            /*!< Nonzero-length status OUT handshake */
#define USB_OTG_DCFG_DAD                     ((uint32_t)0x000007F0)            /*!< Device address */
#define USB_OTG_DCFG_DAD_0                   ((uint32_t)0x00000010)            /*!<Bit 0 */
#define USB_OTG_DCFG_DAD_1                   ((uint32_t)0x00000020)            /*!<Bit 1 */
#define USB_OTG_DCFG_DAD_2                   ((uint32_t)0x00000040)            /*!<Bit 2 */
#define USB_OTG_DCFG_DAD_3                   ((uint32_t)0x00000080)            /*!<Bit 3 */
#define USB_OTG_DCFG_DAD_4                   ((uint32_t)0x00000100)            /*!<Bit 4 */
#define USB_OTG_DCFG_DAD_5                   ((uint32_t)0x00000200)            /*!<Bit 5 */
#define USB_OTG_DCFG_DAD_6                   ((uint32_t)0x00000400)            /*!<Bit 6 */
#define USB_OTG_DCFG_PFIVL                   ((uint32_t)0x00001800)            /*!< Periodic (micro)frame interval */
#define USB_OTG_DCFG_PFIVL_0                 ((uint32_t)0x00000800)            /*!<Bit 0 */
#define USB_OTG_DCFG_PFIVL_1                 ((uint32_t)0x00001000)            /*!<Bit 1 */
#define USB_OTG_DCFG_PERSCHIVL               ((uint32_t)0x03000000)            /*!< Periodic scheduling interval */
#define USB_OTG_DCFG_PERSCHIVL_0             ((uint32_t)0x01000000)            /*!<Bit 0 */
#define USB_OTG_DCFG_PERSCHIVL_1             ((uint32_t)0x02000000)            /*!<Bit 1 */

/********************  Bit definition for USB_OTG_PCGCR register  ********************/
#define USB_OTG_PCGCR_STPPCLK                 ((uint32_t)0x00000001)           /*!< Stop PHY clock */
#define USB_OTG_PCGCR_GATEHCLK                ((uint32_t)0x00000002)           /*!< Gate HCLK */
#define USB_OTG_PCGCR_PHYSUSP                 ((uint32_t)0x00000010)           /*!< PHY suspended */

/********************  Bit definition for USB_OTG_GOTGINT register  ********************/
#define USB_OTG_GOTGINT_SEDET                   ((uint32_t)0x00000004)         /*!< Session end detected */
#define USB_OTG_GOTGINT_SRSSCHG                 ((uint32_t)0x00000100)         /*!< Session request success status change */
#define USB_OTG_GOTGINT_HNSSCHG                 ((uint32_t)0x00000200)         /*!< Host negotiation success status change */
#define USB_OTG_GOTGINT_HNGDET                  ((uint32_t)0x00020000)         /*!< Host negotiation detected */
#define USB_OTG_GOTGINT_ADTOCHG                 ((uint32_t)0x00040000)         /*!< A-device timeout change */
#define USB_OTG_GOTGINT_DBCDNE                  ((uint32_t)0x00080000)         /*!< Debounce done */

/********************  Bit definition for USB_OTG_DCTL register  ********************/
#define USB_OTG_DCTL_RWUSIG                  ((uint32_t)0x00000001)            /*!< Remote wakeup signaling */
#define USB_OTG_DCTL_SDIS                    ((uint32_t)0x00000002)            /*!< Soft disconnect */
#define USB_OTG_DCTL_GINSTS                  ((uint32_t)0x00000004)            /*!< Global IN NAK status */
#define USB_OTG_DCTL_GONSTS                  ((uint32_t)0x00000008)            /*!< Global OUT NAK status */

#define USB_OTG_DCTL_TCTL                    ((uint32_t)0x00000070)            /*!< Test control */
#define USB_OTG_DCTL_TCTL_0                  ((uint32_t)0x00000010)            /*!<Bit 0 */
#define USB_OTG_DCTL_TCTL_1                  ((uint32_t)0x00000020)            /*!<Bit 1 */
#define USB_OTG_DCTL_TCTL_2                  ((uint32_t)0x00000040)            /*!<Bit 2 */
#define USB_OTG_DCTL_SGINAK                  ((uint32_t)0x00000080)            /*!< Set global IN NAK */
#define USB_OTG_DCTL_CGINAK                  ((uint32_t)0x00000100)            /*!< Clear global IN NAK */
#define USB_OTG_DCTL_SGONAK                  ((uint32_t)0x00000200)            /*!< Set global OUT NAK */
#define USB_OTG_DCTL_CGONAK                  ((uint32_t)0x00000400)            /*!< Clear global OUT NAK */
#define USB_OTG_DCTL_POPRGDNE                ((uint32_t)0x00000800)            /*!< Power-on programming done */

/********************  Bit definition for USB_OTG_HFIR register  ********************/
#define USB_OTG_HFIR_FRIVL                   ((uint32_t)0x0000FFFF)            /*!< Frame interval */

/********************  Bit definition for USB_OTG_HFNUM register  ********************/
#define USB_OTG_HFNUM_FRNUM                   ((uint32_t)0x0000FFFF)           /*!< Frame number */
#define USB_OTG_HFNUM_FTREM                   ((uint32_t)0xFFFF0000)           /*!< Frame time remaining */

/********************  Bit definition for USB_OTG_DSTS register  ********************/
#define USB_OTG_DSTS_SUSPSTS                 ((uint32_t)0x00000001)            /*!< Suspend status */

#define USB_OTG_DSTS_ENUMSPD                 ((uint32_t)0x00000006)            /*!< Enumerated speed */
#define USB_OTG_DSTS_ENUMSPD_0               ((uint32_t)0x00000002)            /*!<Bit 0 */
#define USB_OTG_DSTS_ENUMSPD_1               ((uint32_t)0x00000004)            /*!<Bit 1 */
#define USB_OTG_DSTS_EERR                    ((uint32_t)0x00000008)            /*!< Erratic error */
#define USB_OTG_DSTS_FNSOF                   ((uint32_t)0x003FFF00)            /*!< Frame number of the received SOF */

/********************  Bit definition for USB_OTG_GAHBCFG register  ********************/
#define USB_OTG_GAHBCFG_GINT                    ((uint32_t)0x00000001)         /*!< Global interrupt mask */
#define USB_OTG_GAHBCFG_HBSTLEN                 ((uint32_t)0x0000001E)         /*!< Burst length/type */
#define USB_OTG_GAHBCFG_HBSTLEN_0               ((uint32_t)0x00000002)         /*!<Bit 0 */
#define USB_OTG_GAHBCFG_HBSTLEN_1               ((uint32_t)0x00000004)         /*!<Bit 1 */
#define USB_OTG_GAHBCFG_HBSTLEN_2               ((uint32_t)0x00000008)         /*!<Bit 2 */
#define USB_OTG_GAHBCFG_HBSTLEN_3               ((uint32_t)0x00000010)         /*!<Bit 3 */
#define USB_OTG_GAHBCFG_DMAEN                   ((uint32_t)0x00000020)         /*!< DMA enable */
#define USB_OTG_GAHBCFG_TXFELVL                 ((uint32_t)0x00000080)         /*!< TxFIFO empty level */
#define USB_OTG_GAHBCFG_PTXFELVL                ((uint32_t)0x00000100)         /*!< Periodic TxFIFO empty level */

/********************  Bit definition for USB_OTG_GUSBCFG register  ********************/

#define USB_OTG_GUSBCFG_TOCAL                   ((uint32_t)0x00000007)         /*!< FS timeout calibration */
#define USB_OTG_GUSBCFG_TOCAL_0                 ((uint32_t)0x00000001)         /*!<Bit 0 */
#define USB_OTG_GUSBCFG_TOCAL_1                 ((uint32_t)0x00000002)         /*!<Bit 1 */
#define USB_OTG_GUSBCFG_TOCAL_2                 ((uint32_t)0x00000004)         /*!<Bit 2 */
#define USB_OTG_GUSBCFG_PHYSEL                  ((uint32_t)0x00000040)         /*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select */
#define USB_OTG_GUSBCFG_SRPCAP                  ((uint32_t)0x00000100)         /*!< SRP-capable */
#define USB_OTG_GUSBCFG_HNPCAP                  ((uint32_t)0x00000200)         /*!< HNP-capable */
#define USB_OTG_GUSBCFG_TRDT                    ((uint32_t)0x00003C00)         /*!< USB turnaround time */
#define USB_OTG_GUSBCFG_TRDT_0                  ((uint32_t)0x00000400)         /*!<Bit 0 */
#define USB_OTG_GUSBCFG_TRDT_1                  ((uint32_t)0x00000800)         /*!<Bit 1 */
#define USB_OTG_GUSBCFG_TRDT_2                  ((uint32_t)0x00001000)         /*!<Bit 2 */
#define USB_OTG_GUSBCFG_TRDT_3                  ((uint32_t)0x00002000)         /*!<Bit 3 */
#define USB_OTG_GUSBCFG_PHYLPCS                 ((uint32_t)0x00008000)         /*!< PHY Low-power clock select */
#define USB_OTG_GUSBCFG_ULPIFSLS                ((uint32_t)0x00020000)         /*!< ULPI FS/LS select */
#define USB_OTG_GUSBCFG_ULPIAR                  ((uint32_t)0x00040000)         /*!< ULPI Auto-resume */
#define USB_OTG_GUSBCFG_ULPICSM                 ((uint32_t)0x00080000)         /*!< ULPI Clock SuspendM */
#define USB_OTG_GUSBCFG_ULPIEVBUSD              ((uint32_t)0x00100000)         /*!< ULPI External VBUS Drive */
#define USB_OTG_GUSBCFG_ULPIEVBUSI              ((uint32_t)0x00200000)         /*!< ULPI external VBUS indicator */
#define USB_OTG_GUSBCFG_TSDPS                   ((uint32_t)0x00400000)         /*!< TermSel DLine pulsing selection */
#define USB_OTG_GUSBCFG_PCCI                    ((uint32_t)0x00800000)         /*!< Indicator complement */
#define USB_OTG_GUSBCFG_PTCI                    ((uint32_t)0x01000000)         /*!< Indicator pass through */
#define USB_OTG_GUSBCFG_ULPIIPD                 ((uint32_t)0x02000000)         /*!< ULPI interface protect disable */
#define USB_OTG_GUSBCFG_FHMOD                   ((uint32_t)0x20000000)         /*!< Forced host mode */
#define USB_OTG_GUSBCFG_FDMOD                   ((uint32_t)0x40000000)         /*!< Forced peripheral mode */
#define USB_OTG_GUSBCFG_CTXPKT                  ((uint32_t)0x80000000)         /*!< Corrupt Tx packet */

/********************  Bit definition for USB_OTG_GRSTCTL register  ********************/
#define USB_OTG_GRSTCTL_CSRST                   ((uint32_t)0x00000001)         /*!< Core soft reset */
#define USB_OTG_GRSTCTL_HSRST                   ((uint32_t)0x00000002)         /*!< HCLK soft reset */
#define USB_OTG_GRSTCTL_FCRST                   ((uint32_t)0x00000004)         /*!< Host frame counter reset */
#define USB_OTG_GRSTCTL_RXFFLSH                 ((uint32_t)0x00000010)         /*!< RxFIFO flush */
#define USB_OTG_GRSTCTL_TXFFLSH                 ((uint32_t)0x00000020)         /*!< TxFIFO flush */
#define USB_OTG_GRSTCTL_TXFNUM                  ((uint32_t)0x000007C0)         /*!< TxFIFO number */
#define USB_OTG_GRSTCTL_TXFNUM_0                ((uint32_t)0x00000040)         /*!<Bit 0 */
#define USB_OTG_GRSTCTL_TXFNUM_1                ((uint32_t)0x00000080)         /*!<Bit 1 */
#define USB_OTG_GRSTCTL_TXFNUM_2                ((uint32_t)0x00000100)         /*!<Bit 2 */
#define USB_OTG_GRSTCTL_TXFNUM_3                ((uint32_t)0x00000200)         /*!<Bit 3 */
#define USB_OTG_GRSTCTL_TXFNUM_4                ((uint32_t)0x00000400)         /*!<Bit 4 */
#define USB_OTG_GRSTCTL_DMAREQ                  ((uint32_t)0x40000000)         /*!< DMA request signal */
#define USB_OTG_GRSTCTL_AHBIDL                  ((uint32_t)0x80000000)         /*!< AHB master idle */

/********************  Bit definition for USB_OTG_DIEPMSK register  ********************/
#define USB_OTG_DIEPMSK_XFRCM                   ((uint32_t)0x00000001)         /*!< Transfer completed interrupt mask */
#define USB_OTG_DIEPMSK_EPDM                    ((uint32_t)0x00000002)         /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DIEPMSK_TOM                     ((uint32_t)0x00000008)         /*!< Timeout condition mask (nonisochronous endpoints) */
#define USB_OTG_DIEPMSK_ITTXFEMSK               ((uint32_t)0x00000010)         /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DIEPMSK_INEPNMM                 ((uint32_t)0x00000020)         /*!< IN token received with EP mismatch mask */
#define USB_OTG_DIEPMSK_INEPNEM                 ((uint32_t)0x00000040)         /*!< IN endpoint NAK effective mask */
#define USB_OTG_DIEPMSK_TXFURM                  ((uint32_t)0x00000100)         /*!< FIFO underrun mask */
#define USB_OTG_DIEPMSK_BIM                     ((uint32_t)0x00000200)         /*!< BNA interrupt mask */

/********************  Bit definition for USB_OTG_HPTXSTS register  ********************/
#define USB_OTG_HPTXSTS_PTXFSAVL                ((uint32_t)0x0000FFFF)         /*!< Periodic transmit data FIFO space available */
#define USB_OTG_HPTXSTS_PTXQSAV                 ((uint32_t)0x00FF0000)         /*!< Periodic transmit request queue space available */
#define USB_OTG_HPTXSTS_PTXQSAV_0               ((uint32_t)0x00010000)         /*!<Bit 0 */
#define USB_OTG_HPTXSTS_PTXQSAV_1               ((uint32_t)0x00020000)         /*!<Bit 1 */
#define USB_OTG_HPTXSTS_PTXQSAV_2               ((uint32_t)0x00040000)         /*!<Bit 2 */
#define USB_OTG_HPTXSTS_PTXQSAV_3               ((uint32_t)0x00080000)         /*!<Bit 3 */
#define USB_OTG_HPTXSTS_PTXQSAV_4               ((uint32_t)0x00100000)         /*!<Bit 4 */
#define USB_OTG_HPTXSTS_PTXQSAV_5               ((uint32_t)0x00200000)         /*!<Bit 5 */
#define USB_OTG_HPTXSTS_PTXQSAV_6               ((uint32_t)0x00400000)         /*!<Bit 6 */
#define USB_OTG_HPTXSTS_PTXQSAV_7               ((uint32_t)0x00800000)         /*!<Bit 7 */

#define USB_OTG_HPTXSTS_PTXQTOP                 ((uint32_t)0xFF000000)         /*!< Top of the periodic transmit request queue */
#define USB_OTG_HPTXSTS_PTXQTOP_0               ((uint32_t)0x01000000)         /*!<Bit 0 */
#define USB_OTG_HPTXSTS_PTXQTOP_1               ((uint32_t)0x02000000)         /*!<Bit 1 */
#define USB_OTG_HPTXSTS_PTXQTOP_2               ((uint32_t)0x04000000)         /*!<Bit 2 */
#define USB_OTG_HPTXSTS_PTXQTOP_3               ((uint32_t)0x08000000)         /*!<Bit 3 */
#define USB_OTG_HPTXSTS_PTXQTOP_4               ((uint32_t)0x10000000)         /*!<Bit 4 */
#define USB_OTG_HPTXSTS_PTXQTOP_5               ((uint32_t)0x20000000)         /*!<Bit 5 */
#define USB_OTG_HPTXSTS_PTXQTOP_6               ((uint32_t)0x40000000)         /*!<Bit 6 */
#define USB_OTG_HPTXSTS_PTXQTOP_7               ((uint32_t)0x80000000)         /*!<Bit 7 */

/********************  Bit definition for USB_OTG_HAINT register  ********************/
#define USB_OTG_HAINT_HAINT                   ((uint32_t)0x0000FFFF)           /*!< Channel interrupts */

/********************  Bit definition for USB_OTG_DOEPMSK register  ********************/
#define USB_OTG_DOEPMSK_XFRCM                   ((uint32_t)0x00000001)         /*!< Transfer completed interrupt mask */
#define USB_OTG_DOEPMSK_EPDM                    ((uint32_t)0x00000002)         /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DOEPMSK_STUPM                   ((uint32_t)0x00000008)         /*!< SETUP phase done mask */
#define USB_OTG_DOEPMSK_OTEPDM                  ((uint32_t)0x00000010)         /*!< OUT token received when endpoint disabled mask */
#define USB_OTG_DOEPMSK_B2BSTUP                 ((uint32_t)0x00000040)         /*!< Back-to-back SETUP packets received mask */
#define USB_OTG_DOEPMSK_OPEM                    ((uint32_t)0x00000100)         /*!< OUT packet error mask */
#define USB_OTG_DOEPMSK_BOIM                    ((uint32_t)0x00000200)         /*!< BNA interrupt mask */

/********************  Bit definition for USB_OTG_GINTSTS register  ********************/
#define USB_OTG_GINTSTS_CMOD                    ((uint32_t)0x00000001)         /*!< Current mode of operation */
#define USB_OTG_GINTSTS_MMIS                    ((uint32_t)0x00000002)         /*!< Mode mismatch interrupt */
#define USB_OTG_GINTSTS_OTGINT                  ((uint32_t)0x00000004)         /*!< OTG interrupt */
#define USB_OTG_GINTSTS_SOF                     ((uint32_t)0x00000008)         /*!< Start of frame */
#define USB_OTG_GINTSTS_RXFLVL                  ((uint32_t)0x00000010)         /*!< RxFIFO nonempty */
#define USB_OTG_GINTSTS_NPTXFE                  ((uint32_t)0x00000020)         /*!< Nonperiodic TxFIFO empty */
#define USB_OTG_GINTSTS_GINAKEFF                ((uint32_t)0x00000040)         /*!< Global IN nonperiodic NAK effective */
#define USB_OTG_GINTSTS_BOUTNAKEFF              ((uint32_t)0x00000080)         /*!< Global OUT NAK effective */
#define USB_OTG_GINTSTS_ESUSP                   ((uint32_t)0x00000400)         /*!< Early suspend */
#define USB_OTG_GINTSTS_USBSUSP                 ((uint32_t)0x00000800)         /*!< USB suspend */
#define USB_OTG_GINTSTS_USBRST                  ((uint32_t)0x00001000)         /*!< USB reset */
#define USB_OTG_GINTSTS_ENUMDNE                 ((uint32_t)0x00002000)         /*!< Enumeration done */
#define USB_OTG_GINTSTS_ISOODRP                 ((uint32_t)0x00004000)         /*!< Isochronous OUT packet dropped interrupt */
#define USB_OTG_GINTSTS_EOPF                    ((uint32_t)0x00008000)         /*!< End of periodic frame interrupt */
#define USB_OTG_GINTSTS_IEPINT                  ((uint32_t)0x00040000)         /*!< IN endpoint interrupt */
#define USB_OTG_GINTSTS_OEPINT                  ((uint32_t)0x00080000)         /*!< OUT endpoint interrupt */
#define USB_OTG_GINTSTS_IISOIXFR                ((uint32_t)0x00100000)         /*!< Incomplete isochronous IN transfer */
#define USB_OTG_GINTSTS_PXFR_INCOMPISOOUT       ((uint32_t)0x00200000)         /*!< Incomplete periodic transfer */
#define USB_OTG_GINTSTS_DATAFSUSP               ((uint32_t)0x00400000)         /*!< Data fetch suspended */
#define USB_OTG_GINTSTS_HPRTINT                 ((uint32_t)0x01000000)         /*!< Host port interrupt */
#define USB_OTG_GINTSTS_HCINT                   ((uint32_t)0x02000000)         /*!< Host channels interrupt */
#define USB_OTG_GINTSTS_PTXFE                   ((uint32_t)0x04000000)         /*!< Periodic TxFIFO empty */
#define USB_OTG_GINTSTS_LPMINT                   ((uint32_t)0x08000000)        /*!< LPM interrupt */
#define USB_OTG_GINTSTS_CIDSCHG                 ((uint32_t)0x10000000)         /*!< Connector ID status change */
#define USB_OTG_GINTSTS_DISCINT                 ((uint32_t)0x20000000)         /*!< Disconnect detected interrupt */
#define USB_OTG_GINTSTS_SRQINT                  ((uint32_t)0x40000000)         /*!< Session request/new session detected interrupt */
#define USB_OTG_GINTSTS_WKUINT                  ((uint32_t)0x80000000)         /*!< Resume/remote wakeup detected interrupt */

/********************  Bit definition for USB_OTG_GINTMSK register  ********************/

#define USB_OTG_GINTMSK_MMISM                   ((uint32_t)0x00000002)         /*!< Mode mismatch interrupt mask */
#define USB_OTG_GINTMSK_OTGINT                  ((uint32_t)0x00000004)         /*!< OTG interrupt mask */
#define USB_OTG_GINTMSK_SOFM                    ((uint32_t)0x00000008)         /*!< Start of frame mask */
#define USB_OTG_GINTMSK_RXFLVLM                 ((uint32_t)0x00000010)         /*!< Receive FIFO nonempty mask */
#define USB_OTG_GINTMSK_NPTXFEM                 ((uint32_t)0x00000020)         /*!< Nonperiodic TxFIFO empty mask */
#define USB_OTG_GINTMSK_GINAKEFFM               ((uint32_t)0x00000040)         /*!< Global nonperiodic IN NAK effective mask */
#define USB_OTG_GINTMSK_GONAKEFFM               ((uint32_t)0x00000080)         /*!< Global OUT NAK effective mask */
#define USB_OTG_GINTMSK_ESUSPM                  ((uint32_t)0x00000400)         /*!< Early suspend mask */
#define USB_OTG_GINTMSK_USBSUSPM                ((uint32_t)0x00000800)         /*!< USB suspend mask */
#define USB_OTG_GINTMSK_USBRST                  ((uint32_t)0x00001000)         /*!< USB reset mask */
#define USB_OTG_GINTMSK_ENUMDNEM                ((uint32_t)0x00002000)         /*!< Enumeration done mask */
#define USB_OTG_GINTMSK_ISOODRPM                ((uint32_t)0x00004000)         /*!< Isochronous OUT packet dropped interrupt mask */
#define USB_OTG_GINTMSK_EOPFM                   ((uint32_t)0x00008000)         /*!< End of periodic frame interrupt mask */
#define USB_OTG_GINTMSK_EPMISM                  ((uint32_t)0x00020000)         /*!< Endpoint mismatch interrupt mask */
#define USB_OTG_GINTMSK_IEPINT                  ((uint32_t)0x00040000)         /*!< IN endpoints interrupt mask */
#define USB_OTG_GINTMSK_OEPINT                  ((uint32_t)0x00080000)         /*!< OUT endpoints interrupt mask */
#define USB_OTG_GINTMSK_IISOIXFRM               ((uint32_t)0x00100000)         /*!< Incomplete isochronous IN transfer mask */
#define USB_OTG_GINTMSK_PXFRM_IISOOXFRM         ((uint32_t)0x00200000)         /*!< Incomplete periodic transfer mask */
#define USB_OTG_GINTMSK_FSUSPM                  ((uint32_t)0x00400000)         /*!< Data fetch suspended mask */
#define USB_OTG_GINTMSK_PRTIM                   ((uint32_t)0x01000000)         /*!< Host port interrupt mask */
#define USB_OTG_GINTMSK_HCIM                    ((uint32_t)0x02000000)         /*!< Host channels interrupt mask */
#define USB_OTG_GINTMSK_PTXFEM                  ((uint32_t)0x04000000)         /*!< Periodic TxFIFO empty mask */
#define USB_OTG_GINTMSK_LPMINTM                 ((uint32_t)0x08000000)         /*!< LPM interrupt Mask */
#define USB_OTG_GINTMSK_CIDSCHGM                ((uint32_t)0x10000000)         /*!< Connector ID status change mask */
#define USB_OTG_GINTMSK_DISCINT                 ((uint32_t)0x20000000)         /*!< Disconnect detected interrupt mask */
#define USB_OTG_GINTMSK_SRQIM                   ((uint32_t)0x40000000)         /*!< Session request/new session detected interrupt mask */
#define USB_OTG_GINTMSK_WUIM                    ((uint32_t)0x80000000)         /*!< Resume/remote wakeup detected interrupt mask */

/********************  Bit definition for USB_OTG_DAINT register  ********************/
#define USB_OTG_DAINT_IEPINT                  ((uint32_t)0x0000FFFF)            /*!< IN endpoint interrupt bits */
#define USB_OTG_DAINT_OEPINT                  ((uint32_t)0xFFFF0000)            /*!< OUT endpoint interrupt bits */

/********************  Bit definition for USB_OTG_HAINTMSK register  ********************/
#define USB_OTG_HAINTMSK_HAINTM                  ((uint32_t)0x0000FFFF)        /*!< Channel interrupt mask */

/********************  Bit definition for USB_OTG_GRXSTSP register  ********************/
#define USB_OTG_GRXSTSP_EPNUM                    ((uint32_t)0x0000000F)        /*!< IN EP interrupt mask bits */
#define USB_OTG_GRXSTSP_BCNT                     ((uint32_t)0x00007FF0)        /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_DPID                     ((uint32_t)0x00018000)        /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_PKTSTS                   ((uint32_t)0x001E0000)        /*!< OUT EP interrupt mask bits */

/********************  Bit definition for USB_OTG_DAINTMSK register  ********************/
#define USB_OTG_DAINTMSK_IEPM                    ((uint32_t)0x0000FFFF)        /*!< IN EP interrupt mask bits */
#define USB_OTG_DAINTMSK_OEPM                    ((uint32_t)0xFFFF0000)        /*!< OUT EP interrupt mask bits */

/********************  Bit definition for OTG register  ********************/

#define USB_OTG_CHNUM                   ((uint32_t)0x0000000F)                 /*!< Channel number */
#define USB_OTG_CHNUM_0                 ((uint32_t)0x00000001)                 /*!<Bit 0 */
#define USB_OTG_CHNUM_1                 ((uint32_t)0x00000002)                 /*!<Bit 1 */
#define USB_OTG_CHNUM_2                 ((uint32_t)0x00000004)                 /*!<Bit 2 */
#define USB_OTG_CHNUM_3                 ((uint32_t)0x00000008)                 /*!<Bit 3 */
#define USB_OTG_BCNT                    ((uint32_t)0x00007FF0)                 /*!< Byte count */                                                                             
#define USB_OTG_DPID                    ((uint32_t)0x00018000)                 /*!< Data PID */
#define USB_OTG_DPID_0                  ((uint32_t)0x00008000)                 /*!<Bit 0 */
#define USB_OTG_DPID_1                  ((uint32_t)0x00010000)                 /*!<Bit 1 */                                                                           
#define USB_OTG_PKTSTS                  ((uint32_t)0x001E0000)                 /*!< Packet status */
#define USB_OTG_PKTSTS_0                ((uint32_t)0x00020000)                 /*!<Bit 0 */
#define USB_OTG_PKTSTS_1                ((uint32_t)0x00040000)                 /*!<Bit 1 */
#define USB_OTG_PKTSTS_2                ((uint32_t)0x00080000)                 /*!<Bit 2 */
#define USB_OTG_PKTSTS_3                ((uint32_t)0x00100000)                 /*!<Bit 3 */                                                                         
#define USB_OTG_EPNUM                   ((uint32_t)0x0000000F)                 /*!< Endpoint number */
#define USB_OTG_EPNUM_0                 ((uint32_t)0x00000001)                 /*!<Bit 0 */
#define USB_OTG_EPNUM_1                 ((uint32_t)0x00000002)                 /*!<Bit 1 */
#define USB_OTG_EPNUM_2                 ((uint32_t)0x00000004)                 /*!<Bit 2 */
#define USB_OTG_EPNUM_3                 ((uint32_t)0x00000008)                 /*!<Bit 3 */                                                                            
#define USB_OTG_FRMNUM                  ((uint32_t)0x01E00000)                 /*!< Frame number */
#define USB_OTG_FRMNUM_0                ((uint32_t)0x00200000)                 /*!<Bit 0 */
#define USB_OTG_FRMNUM_1                ((uint32_t)0x00400000)                 /*!<Bit 1 */
#define USB_OTG_FRMNUM_2                ((uint32_t)0x00800000)                 /*!<Bit 2 */
#define USB_OTG_FRMNUM_3                ((uint32_t)0x01000000)                 /*!<Bit 3 */

/********************  Bit definition for OTG register  ********************/

#define USB_OTG_CHNUM                   ((uint32_t)0x0000000F)                 /*!< Channel number */
#define USB_OTG_CHNUM_0                 ((uint32_t)0x00000001)                 /*!<Bit 0 */
#define USB_OTG_CHNUM_1                 ((uint32_t)0x00000002)                 /*!<Bit 1 */
#define USB_OTG_CHNUM_2                 ((uint32_t)0x00000004)                 /*!<Bit 2 */
#define USB_OTG_CHNUM_3                 ((uint32_t)0x00000008)                 /*!<Bit 3 */
#define USB_OTG_BCNT                    ((uint32_t)0x00007FF0)                 /*!< Byte count */                                                                         
#define USB_OTG_DPID                    ((uint32_t)0x00018000)                 /*!< Data PID */
#define USB_OTG_DPID_0                  ((uint32_t)0x00008000)                 /*!<Bit 0 */
#define USB_OTG_DPID_1                  ((uint32_t)0x00010000)                 /*!<Bit 1 */                                                                         
#define USB_OTG_PKTSTS                  ((uint32_t)0x001E0000)                 /*!< Packet status */
#define USB_OTG_PKTSTS_0                ((uint32_t)0x00020000)                 /*!<Bit 0 */
#define USB_OTG_PKTSTS_1                ((uint32_t)0x00040000)                 /*!<Bit 1 */
#define USB_OTG_PKTSTS_2                ((uint32_t)0x00080000)                 /*!<Bit 2 */
#define USB_OTG_PKTSTS_3                ((uint32_t)0x00100000)                 /*!<Bit 3 */                                                                            
#define USB_OTG_EPNUM                   ((uint32_t)0x0000000F)                 /*!< Endpoint number */
#define USB_OTG_EPNUM_0                 ((uint32_t)0x00000001)                 /*!<Bit 0 */
#define USB_OTG_EPNUM_1                 ((uint32_t)0x00000002)                 /*!<Bit 1 */
#define USB_OTG_EPNUM_2                 ((uint32_t)0x00000004)                 /*!<Bit 2 */
#define USB_OTG_EPNUM_3                 ((uint32_t)0x00000008)                 /*!<Bit 3 */                                                                             
#define USB_OTG_FRMNUM                  ((uint32_t)0x01E00000)                 /*!< Frame number */
#define USB_OTG_FRMNUM_0                ((uint32_t)0x00200000)                 /*!<Bit 0 */
#define USB_OTG_FRMNUM_1                ((uint32_t)0x00400000)                 /*!<Bit 1 */
#define USB_OTG_FRMNUM_2                ((uint32_t)0x00800000)                 /*!<Bit 2 */
#define USB_OTG_FRMNUM_3                ((uint32_t)0x01000000)                 /*!<Bit 3 */

/********************  Bit definition for USB_OTG_GRXFSIZ register  ********************/
#define USB_OTG_GRXFSIZ_RXFD                    ((uint32_t)0x0000FFFF)         /*!< RxFIFO depth */

/********************  Bit definition for USB_OTG_DVBUSDIS register  ********************/
#define USB_OTG_DVBUSDIS_VBUSDT                  ((uint32_t)0x0000FFFF)        /*!< Device VBUS discharge time */

/********************  Bit definition for OTG register  ********************/
#define USB_OTG_NPTXFSA                 ((uint32_t)0x0000FFFF)                 /*!< Nonperiodic transmit RAM start address */
#define USB_OTG_NPTXFD                  ((uint32_t)0xFFFF0000)                 /*!< Nonperiodic TxFIFO depth */
#define USB_OTG_TX0FSA                  ((uint32_t)0x0000FFFF)                 /*!< Endpoint 0 transmit RAM start address */
#define USB_OTG_TX0FD                   ((uint32_t)0xFFFF0000)                 /*!< Endpoint 0 TxFIFO depth */

/********************  Bit definition for USB_OTG_DVBUSPULSE register  ********************/
#define USB_OTG_DVBUSPULSE_DVBUSP                  ((uint32_t)0x00000FFF)      /*!< Device VBUS pulsing time */

/********************  Bit definition for USB_OTG_GNPTXSTS register  ********************/
#define USB_OTG_GNPTXSTS_NPTXFSAV                ((uint32_t)0x0000FFFF)        /*!< Nonperiodic TxFIFO space available */

#define USB_OTG_GNPTXSTS_NPTQXSAV                ((uint32_t)0x00FF0000)        /*!< Nonperiodic transmit request queue space available */
#define USB_OTG_GNPTXSTS_NPTQXSAV_0              ((uint32_t)0x00010000)        /*!<Bit 0 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_1              ((uint32_t)0x00020000)        /*!<Bit 1 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_2              ((uint32_t)0x00040000)        /*!<Bit 2 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_3              ((uint32_t)0x00080000)        /*!<Bit 3 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_4              ((uint32_t)0x00100000)        /*!<Bit 4 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_5              ((uint32_t)0x00200000)        /*!<Bit 5 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_6              ((uint32_t)0x00400000)        /*!<Bit 6 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_7              ((uint32_t)0x00800000)        /*!<Bit 7 */

#define USB_OTG_GNPTXSTS_NPTXQTOP                ((uint32_t)0x7F000000)        /*!< Top of the nonperiodic transmit request queue */
#define USB_OTG_GNPTXSTS_NPTXQTOP_0              ((uint32_t)0x01000000)        /*!<Bit 0 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_1              ((uint32_t)0x02000000)        /*!<Bit 1 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_2              ((uint32_t)0x04000000)        /*!<Bit 2 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_3              ((uint32_t)0x08000000)        /*!<Bit 3 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_4              ((uint32_t)0x10000000)        /*!<Bit 4 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_5              ((uint32_t)0x20000000)        /*!<Bit 5 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_6              ((uint32_t)0x40000000)        /*!<Bit 6 */

/********************  Bit definition for USB_OTG_DTHRCTL register  ***************/
#define USB_OTG_DTHRCTL_NONISOTHREN             ((uint32_t)0x00000001)         /*!< Nonisochronous IN endpoints threshold enable */
#define USB_OTG_DTHRCTL_ISOTHREN                ((uint32_t)0x00000002)         /*!< ISO IN endpoint threshold enable */

#define USB_OTG_DTHRCTL_TXTHRLEN                ((uint32_t)0x000007FC)         /*!< Transmit threshold length */
#define USB_OTG_DTHRCTL_TXTHRLEN_0              ((uint32_t)0x00000004)         /*!<Bit 0 */
#define USB_OTG_DTHRCTL_TXTHRLEN_1              ((uint32_t)0x00000008)         /*!<Bit 1 */
#define USB_OTG_DTHRCTL_TXTHRLEN_2              ((uint32_t)0x00000010)         /*!<Bit 2 */
#define USB_OTG_DTHRCTL_TXTHRLEN_3              ((uint32_t)0x00000020)         /*!<Bit 3 */
#define USB_OTG_DTHRCTL_TXTHRLEN_4              ((uint32_t)0x00000040)         /*!<Bit 4 */
#define USB_OTG_DTHRCTL_TXTHRLEN_5              ((uint32_t)0x00000080)         /*!<Bit 5 */
#define USB_OTG_DTHRCTL_TXTHRLEN_6              ((uint32_t)0x00000100)         /*!<Bit 6 */
#define USB_OTG_DTHRCTL_TXTHRLEN_7              ((uint32_t)0x00000200)         /*!<Bit 7 */
#define USB_OTG_DTHRCTL_TXTHRLEN_8              ((uint32_t)0x00000400)         /*!<Bit 8 */
#define USB_OTG_DTHRCTL_RXTHREN                 ((uint32_t)0x00010000)         /*!< Receive threshold enable */

#define USB_OTG_DTHRCTL_RXTHRLEN                ((uint32_t)0x03FE0000)         /*!< Receive threshold length */
#define USB_OTG_DTHRCTL_RXTHRLEN_0              ((uint32_t)0x00020000)         /*!<Bit 0 */
#define USB_OTG_DTHRCTL_RXTHRLEN_1              ((uint32_t)0x00040000)         /*!<Bit 1 */
#define USB_OTG_DTHRCTL_RXTHRLEN_2              ((uint32_t)0x00080000)         /*!<Bit 2 */
#define USB_OTG_DTHRCTL_RXTHRLEN_3              ((uint32_t)0x00100000)         /*!<Bit 3 */
#define USB_OTG_DTHRCTL_RXTHRLEN_4              ((uint32_t)0x00200000)         /*!<Bit 4 */
#define USB_OTG_DTHRCTL_RXTHRLEN_5              ((uint32_t)0x00400000)         /*!<Bit 5 */
#define USB_OTG_DTHRCTL_RXTHRLEN_6              ((uint32_t)0x00800000)         /*!<Bit 6 */
#define USB_OTG_DTHRCTL_RXTHRLEN_7              ((uint32_t)0x01000000)         /*!<Bit 7 */
#define USB_OTG_DTHRCTL_RXTHRLEN_8              ((uint32_t)0x02000000)         /*!<Bit 8 */
#define USB_OTG_DTHRCTL_ARPEN                   ((uint32_t)0x08000000)         /*!< Arbiter parking enable */

/********************  Bit definition for USB_OTG_DIEPEMPMSK register  ***************/
#define USB_OTG_DIEPEMPMSK_INEPTXFEM               ((uint32_t)0x0000FFFF)      /*!< IN EP Tx FIFO empty interrupt mask bits */

/********************  Bit definition for USB_OTG_DEACHINT register  ********************/
#define USB_OTG_DEACHINT_IEP1INT                 ((uint32_t)0x00000002)        /*!< IN endpoint 1interrupt bit */
#define USB_OTG_DEACHINT_OEP1INT                 ((uint32_t)0x00020000)        /*!< OUT endpoint 1 interrupt bit */

/********************  Bit definition for USB_OTG_GCCFG register  ********************/
#define USB_OTG_GCCFG_DCDET                  ((uint32_t)0x00000001)            /*!< Data contact detection (DCD) status */
#define USB_OTG_GCCFG_PDET                   ((uint32_t)0x00000002)            /*!< Primary detection (PD) status */
#define USB_OTG_GCCFG_SDET                   ((uint32_t)0x00000004)            /*!< Secondary detection (SD) status */
#define USB_OTG_GCCFG_PS2DET                 ((uint32_t)0x00000008)            /*!< DM pull-up detection status */
#define USB_OTG_GCCFG_PWRDWN                 ((uint32_t)0x00010000)            /*!< Power down */
#define USB_OTG_GCCFG_BCDEN                  ((uint32_t)0x00020000)            /*!< Battery charging detector (BCD) enable */
#define USB_OTG_GCCFG_DCDEN                  ((uint32_t)0x00040000)            /*!< Data contact detection (DCD) mode enable*/
#define USB_OTG_GCCFG_PDEN                   ((uint32_t)0x00080000)            /*!< Primary detection (PD) mode enable*/
#define USB_OTG_GCCFG_SDEN                   ((uint32_t)0x00100000)            /*!< Secondary detection (SD) mode enable */
#define USB_OTG_GCCFG_VBDEN                  ((uint32_t)0x00200000)            /*!< Secondary detection (SD) mode enable */

/********************  Bit definition for USB_OTG_GPWRDN) register  ********************/
#define USB_OTG_GPWRDN_DISABLEVBUS                 ((uint32_t)0x00000040)      /*!< Power down */

/********************  Bit definition for USB_OTG_DEACHINTMSK register  ********************/
#define USB_OTG_DEACHINTMSK_IEP1INTM                ((uint32_t)0x00000002)     /*!< IN Endpoint 1 interrupt mask bit */
#define USB_OTG_DEACHINTMSK_OEP1INTM                ((uint32_t)0x00020000)     /*!< OUT Endpoint 1 interrupt mask bit */

/********************  Bit definition for USB_OTG_CID register  ********************/
#define USB_OTG_CID_PRODUCT_ID              ((uint32_t)0xFFFFFFFF)             /*!< Product ID field */


/********************  Bit definition for USB_OTG_GHWCFG3 register  ********************/
#define  USB_OTG_GHWCFG3_LPMMode              ((uint32_t)0x00004000)           /* LPM mode specified for Mode of Operation */ 

/********************  Bit definition for USB_OTG_GLPMCFG register  ********************/
#define  USB_OTG_GLPMCFG_ENBESL                      ((uint32_t)0x10000000)    /* Enable best effort service latency */ 
#define  USB_OTG_GLPMCFG_LPMRCNTSTS                  ((uint32_t)0x0E000000)    /* LPM retry count status */ 
#define  USB_OTG_GLPMCFG_SNDLPM                      ((uint32_t)0x01000000)    /* Send LPM transaction */ 
#define  USB_OTG_GLPMCFG_LPMRCNT                     ((uint32_t)0x00E00000)    /* LPM retry count */ 
#define  USB_OTG_GLPMCFG_LPMCHIDX                    ((uint32_t)0x001E0000)    /* LPMCHIDX: */ 
#define  USB_OTG_GLPMCFG_L1ResumeOK                  ((uint32_t)0x00010000)    /* Sleep State Resume OK */ 
#define  USB_OTG_GLPMCFG_SLPSTS                      ((uint32_t)0x00008000)    /* Port sleep status */ 
#define  USB_OTG_GLPMCFG_LPMRSP                      ((uint32_t)0x00006000)    /* LPM response */ 
#define  USB_OTG_GLPMCFG_L1DSEN                      ((uint32_t)0x00001000)    /* L1 deep sleep enable */ 
#define  USB_OTG_GLPMCFG_BESLTHRS                    ((uint32_t)0x00000F00)    /* BESL threshold */ 
#define  USB_OTG_GLPMCFG_L1SSEN                      ((uint32_t)0x00000080)    /* L1 shallow sleep enable */ 
#define  USB_OTG_GLPMCFG_REMWAKE                     ((uint32_t)0x00000040)    /* bRemoteWake value received with last ACKed LPM Token */ 
#define  USB_OTG_GLPMCFG_BESL                        ((uint32_t)0x0000003C)    /* BESL value received with last ACKed LPM Token  */
#define  USB_OTG_GLPMCFG_LPMACK                      ((uint32_t)0x00000002)    /* LPM Token acknowledge enable*/
#define  USB_OTG_GLPMCFG_LPMEN                       ((uint32_t)0x00000001)    /* LPM support enable  */


/********************  Bit definition for USB_OTG_DIEPEACHMSK1 register  ********************/
#define USB_OTG_DIEPEACHMSK1_XFRCM                   ((uint32_t)0x00000001)    /*!< Transfer completed interrupt mask */
#define USB_OTG_DIEPEACHMSK1_EPDM                    ((uint32_t)0x00000002)    /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DIEPEACHMSK1_TOM                     ((uint32_t)0x00000008)    /*!< Timeout condition mask (nonisochronous endpoints) */
#define USB_OTG_DIEPEACHMSK1_ITTXFEMSK               ((uint32_t)0x00000010)    /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DIEPEACHMSK1_INEPNMM                 ((uint32_t)0x00000020)    /*!< IN token received with EP mismatch mask */
#define USB_OTG_DIEPEACHMSK1_INEPNEM                 ((uint32_t)0x00000040)    /*!< IN endpoint NAK effective mask */
#define USB_OTG_DIEPEACHMSK1_TXFURM                  ((uint32_t)0x00000100)    /*!< FIFO underrun mask */
#define USB_OTG_DIEPEACHMSK1_BIM                     ((uint32_t)0x00000200)    /*!< BNA interrupt mask */
#define USB_OTG_DIEPEACHMSK1_NAKM                    ((uint32_t)0x00002000)    /*!< NAK interrupt mask */

/********************  Bit definition for USB_OTG_HPRT register  ********************/
#define USB_OTG_HPRT_PCSTS                   ((uint32_t)0x00000001)            /*!< Port connect status */
#define USB_OTG_HPRT_PCDET                   ((uint32_t)0x00000002)            /*!< Port connect detected */
#define USB_OTG_HPRT_PENA                    ((uint32_t)0x00000004)            /*!< Port enable */
#define USB_OTG_HPRT_PENCHNG                 ((uint32_t)0x00000008)            /*!< Port enable/disable change */
#define USB_OTG_HPRT_POCA                    ((uint32_t)0x00000010)            /*!< Port overcurrent active */
#define USB_OTG_HPRT_POCCHNG                 ((uint32_t)0x00000020)            /*!< Port overcurrent change */
#define USB_OTG_HPRT_PRES                    ((uint32_t)0x00000040)            /*!< Port resume */
#define USB_OTG_HPRT_PSUSP                   ((uint32_t)0x00000080)            /*!< Port suspend */
#define USB_OTG_HPRT_PRST                    ((uint32_t)0x00000100)            /*!< Port reset */

#define USB_OTG_HPRT_PLSTS                   ((uint32_t)0x00000C00)            /*!< Port line status */
#define USB_OTG_HPRT_PLSTS_0                 ((uint32_t)0x00000400)            /*!<Bit 0 */
#define USB_OTG_HPRT_PLSTS_1                 ((uint32_t)0x00000800)            /*!<Bit 1 */
#define USB_OTG_HPRT_PPWR                    ((uint32_t)0x00001000)            /*!< Port power */

#define USB_OTG_HPRT_PTCTL                   ((uint32_t)0x0001E000)            /*!< Port test control */
#define USB_OTG_HPRT_PTCTL_0                 ((uint32_t)0x00002000)            /*!<Bit 0 */
#define USB_OTG_HPRT_PTCTL_1                 ((uint32_t)0x00004000)            /*!<Bit 1 */
#define USB_OTG_HPRT_PTCTL_2                 ((uint32_t)0x00008000)            /*!<Bit 2 */
#define USB_OTG_HPRT_PTCTL_3                 ((uint32_t)0x00010000)            /*!<Bit 3 */

#define USB_OTG_HPRT_PSPD                    ((uint32_t)0x00060000)            /*!< Port speed */
#define USB_OTG_HPRT_PSPD_0                  ((uint32_t)0x00020000)            /*!<Bit 0 */
#define USB_OTG_HPRT_PSPD_1                  ((uint32_t)0x00040000)            /*!<Bit 1 */

/********************  Bit definition for USB_OTG_DOEPEACHMSK1 register  ********************/
#define USB_OTG_DOEPEACHMSK1_XFRCM                   ((uint32_t)0x00000001)    /*!< Transfer completed interrupt mask */
#define USB_OTG_DOEPEACHMSK1_EPDM                    ((uint32_t)0x00000002)    /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DOEPEACHMSK1_TOM                     ((uint32_t)0x00000008)    /*!< Timeout condition mask */
#define USB_OTG_DOEPEACHMSK1_ITTXFEMSK               ((uint32_t)0x00000010)    /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DOEPEACHMSK1_INEPNMM                 ((uint32_t)0x00000020)    /*!< IN token received with EP mismatch mask */
#define USB_OTG_DOEPEACHMSK1_INEPNEM                 ((uint32_t)0x00000040)    /*!< IN endpoint NAK effective mask */
#define USB_OTG_DOEPEACHMSK1_TXFURM                  ((uint32_t)0x00000100)    /*!< OUT packet error mask */
#define USB_OTG_DOEPEACHMSK1_BIM                     ((uint32_t)0x00000200)    /*!< BNA interrupt mask */
#define USB_OTG_DOEPEACHMSK1_BERRM                   ((uint32_t)0x00001000)    /*!< Bubble error interrupt mask */
#define USB_OTG_DOEPEACHMSK1_NAKM                    ((uint32_t)0x00002000)    /*!< NAK interrupt mask */
#define USB_OTG_DOEPEACHMSK1_NYETM                   ((uint32_t)0x00004000)    /*!< NYET interrupt mask */

/********************  Bit definition for USB_OTG_HPTXFSIZ register  ********************/
#define USB_OTG_HPTXFSIZ_PTXSA                   ((uint32_t)0x0000FFFF)        /*!< Host periodic TxFIFO start address */
#define USB_OTG_HPTXFSIZ_PTXFD                   ((uint32_t)0xFFFF0000)        /*!< Host periodic TxFIFO depth */

/********************  Bit definition for USB_OTG_DIEPCTL register  ********************/
#define USB_OTG_DIEPCTL_MPSIZ                   ((uint32_t)0x000007FF)         /*!< Maximum packet size */
#define USB_OTG_DIEPCTL_USBAEP                  ((uint32_t)0x00008000)         /*!< USB active endpoint */
#define USB_OTG_DIEPCTL_EONUM_DPID              ((uint32_t)0x00010000)         /*!< Even/odd frame */
#define USB_OTG_DIEPCTL_NAKSTS                  ((uint32_t)0x00020000)         /*!< NAK status */
                                                                             
#define USB_OTG_DIEPCTL_EPTYP                   ((uint32_t)0x000C0000)         /*!< Endpoint type */
#define USB_OTG_DIEPCTL_EPTYP_0                 ((uint32_t)0x00040000)         /*!<Bit 0 */
#define USB_OTG_DIEPCTL_EPTYP_1                 ((uint32_t)0x00080000)         /*!<Bit 1 */
#define USB_OTG_DIEPCTL_STALL                   ((uint32_t)0x00200000)         /*!< STALL handshake */
                                                                             
#define USB_OTG_DIEPCTL_TXFNUM                  ((uint32_t)0x03C00000)         /*!< TxFIFO number */
#define USB_OTG_DIEPCTL_TXFNUM_0                ((uint32_t)0x00400000)         /*!<Bit 0 */
#define USB_OTG_DIEPCTL_TXFNUM_1                ((uint32_t)0x00800000)         /*!<Bit 1 */
#define USB_OTG_DIEPCTL_TXFNUM_2                ((uint32_t)0x01000000)         /*!<Bit 2 */
#define USB_OTG_DIEPCTL_TXFNUM_3                ((uint32_t)0x02000000)         /*!<Bit 3 */
#define USB_OTG_DIEPCTL_CNAK                    ((uint32_t)0x04000000)         /*!< Clear NAK */
#define USB_OTG_DIEPCTL_SNAK                    ((uint32_t)0x08000000)         /*!< Set NAK */
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM          ((uint32_t)0x10000000)         /*!< Set DATA0 PID */
#define USB_OTG_DIEPCTL_SODDFRM                 ((uint32_t)0x20000000)         /*!< Set odd frame */
#define USB_OTG_DIEPCTL_EPDIS                   ((uint32_t)0x40000000)         /*!< Endpoint disable */
#define USB_OTG_DIEPCTL_EPENA                   ((uint32_t)0x80000000)         /*!< Endpoint enable */

/********************  Bit definition for USB_OTG_HCCHAR register  ********************/
#define USB_OTG_HCCHAR_MPSIZ                   ((uint32_t)0x000007FF)          /*!< Maximum packet size */
                                                                              
#define USB_OTG_HCCHAR_EPNUM                   ((uint32_t)0x00007800)          /*!< Endpoint number */
#define USB_OTG_HCCHAR_EPNUM_0                 ((uint32_t)0x00000800)          /*!<Bit 0 */
#define USB_OTG_HCCHAR_EPNUM_1                 ((uint32_t)0x00001000)          /*!<Bit 1 */
#define USB_OTG_HCCHAR_EPNUM_2                 ((uint32_t)0x00002000)          /*!<Bit 2 */
#define USB_OTG_HCCHAR_EPNUM_3                 ((uint32_t)0x00004000)          /*!<Bit 3 */
#define USB_OTG_HCCHAR_EPDIR                   ((uint32_t)0x00008000)          /*!< Endpoint direction */
#define USB_OTG_HCCHAR_LSDEV                   ((uint32_t)0x00020000)          /*!< Low-speed device */
                                                                              
#define USB_OTG_HCCHAR_EPTYP                   ((uint32_t)0x000C0000)          /*!< Endpoint type */
#define USB_OTG_HCCHAR_EPTYP_0                 ((uint32_t)0x00040000)          /*!<Bit 0 */
#define USB_OTG_HCCHAR_EPTYP_1                 ((uint32_t)0x00080000)          /*!<Bit 1 */
                                                                              
#define USB_OTG_HCCHAR_MC                      ((uint32_t)0x00300000)          /*!< Multi Count (MC) / Error Count (EC) */
#define USB_OTG_HCCHAR_MC_0                    ((uint32_t)0x00100000)          /*!<Bit 0 */
#define USB_OTG_HCCHAR_MC_1                    ((uint32_t)0x00200000)          /*!<Bit 1 */
                                                                              
#define USB_OTG_HCCHAR_DAD                     ((uint32_t)0x1FC00000)          /*!< Device address */
#define USB_OTG_HCCHAR_DAD_0                   ((uint32_t)0x00400000)          /*!<Bit 0 */
#define USB_OTG_HCCHAR_DAD_1                   ((uint32_t)0x00800000)          /*!<Bit 1 */
#define USB_OTG_HCCHAR_DAD_2                   ((uint32_t)0x01000000)          /*!<Bit 2 */
#define USB_OTG_HCCHAR_DAD_3                   ((uint32_t)0x02000000)          /*!<Bit 3 */
#define USB_OTG_HCCHAR_DAD_4                   ((uint32_t)0x04000000)          /*!<Bit 4 */
#define USB_OTG_HCCHAR_DAD_5                   ((uint32_t)0x08000000)          /*!<Bit 5 */
#define USB_OTG_HCCHAR_DAD_6                   ((uint32_t)0x10000000)          /*!<Bit 6 */
#define USB_OTG_HCCHAR_ODDFRM                  ((uint32_t)0x20000000)          /*!< Odd frame */
#define USB_OTG_HCCHAR_CHDIS                   ((uint32_t)0x40000000)          /*!< Channel disable */
#define USB_OTG_HCCHAR_CHENA                   ((uint32_t)0x80000000)          /*!< Channel enable */

/********************  Bit definition for USB_OTG_HCSPLT register  ********************/

#define USB_OTG_HCSPLT_PRTADDR                 ((uint32_t)0x0000007F)          /*!< Port address */
#define USB_OTG_HCSPLT_PRTADDR_0               ((uint32_t)0x00000001)          /*!<Bit 0 */
#define USB_OTG_HCSPLT_PRTADDR_1               ((uint32_t)0x00000002)          /*!<Bit 1 */
#define USB_OTG_HCSPLT_PRTADDR_2               ((uint32_t)0x00000004)          /*!<Bit 2 */
#define USB_OTG_HCSPLT_PRTADDR_3               ((uint32_t)0x00000008)          /*!<Bit 3 */
#define USB_OTG_HCSPLT_PRTADDR_4               ((uint32_t)0x00000010)          /*!<Bit 4 */
#define USB_OTG_HCSPLT_PRTADDR_5               ((uint32_t)0x00000020)          /*!<Bit 5 */
#define USB_OTG_HCSPLT_PRTADDR_6               ((uint32_t)0x00000040)          /*!<Bit 6 */
                                                                               
#define USB_OTG_HCSPLT_HUBADDR                 ((uint32_t)0x00003F80)          /*!< Hub address */
#define USB_OTG_HCSPLT_HUBADDR_0               ((uint32_t)0x00000080)          /*!<Bit 0 */
#define USB_OTG_HCSPLT_HUBADDR_1               ((uint32_t)0x00000100)          /*!<Bit 1 */
#define USB_OTG_HCSPLT_HUBADDR_2               ((uint32_t)0x00000200)          /*!<Bit 2 */
#define USB_OTG_HCSPLT_HUBADDR_3               ((uint32_t)0x00000400)          /*!<Bit 3 */
#define USB_OTG_HCSPLT_HUBADDR_4               ((uint32_t)0x00000800)          /*!<Bit 4 */
#define USB_OTG_HCSPLT_HUBADDR_5               ((uint32_t)0x00001000)          /*!<Bit 5 */
#define USB_OTG_HCSPLT_HUBADDR_6               ((uint32_t)0x00002000)          /*!<Bit 6 */
                                                                               
#define USB_OTG_HCSPLT_XACTPOS                 ((uint32_t)0x0000C000)          /*!< XACTPOS */
#define USB_OTG_HCSPLT_XACTPOS_0               ((uint32_t)0x00004000)          /*!<Bit 0 */
#define USB_OTG_HCSPLT_XACTPOS_1               ((uint32_t)0x00008000)          /*!<Bit 1 */
#define USB_OTG_HCSPLT_COMPLSPLT               ((uint32_t)0x00010000)          /*!< Do complete split */
#define USB_OTG_HCSPLT_SPLITEN                 ((uint32_t)0x80000000)          /*!< Split enable */

/********************  Bit definition for USB_OTG_HCINT register  ********************/
#define USB_OTG_HCINT_XFRC                    ((uint32_t)0x00000001)           /*!< Transfer completed */
#define USB_OTG_HCINT_CHH                     ((uint32_t)0x00000002)           /*!< Channel halted */
#define USB_OTG_HCINT_AHBERR                  ((uint32_t)0x00000004)           /*!< AHB error */
#define USB_OTG_HCINT_STALL                   ((uint32_t)0x00000008)           /*!< STALL response received interrupt */
#define USB_OTG_HCINT_NAK                     ((uint32_t)0x00000010)           /*!< NAK response received interrupt */
#define USB_OTG_HCINT_ACK                     ((uint32_t)0x00000020)           /*!< ACK response received/transmitted interrupt */
#define USB_OTG_HCINT_NYET                    ((uint32_t)0x00000040)           /*!< Response received interrupt */
#define USB_OTG_HCINT_TXERR                   ((uint32_t)0x00000080)           /*!< Transaction error */
#define USB_OTG_HCINT_BBERR                   ((uint32_t)0x00000100)           /*!< Babble error */
#define USB_OTG_HCINT_FRMOR                   ((uint32_t)0x00000200)           /*!< Frame overrun */
#define USB_OTG_HCINT_DTERR                   ((uint32_t)0x00000400)           /*!< Data toggle error */

/********************  Bit definition for USB_OTG_DIEPINT register  ********************/
#define USB_OTG_DIEPINT_XFRC                    ((uint32_t)0x00000001)         /*!< Transfer completed interrupt */
#define USB_OTG_DIEPINT_EPDISD                  ((uint32_t)0x00000002)         /*!< Endpoint disabled interrupt */
#define USB_OTG_DIEPINT_TOC                     ((uint32_t)0x00000008)         /*!< Timeout condition */
#define USB_OTG_DIEPINT_ITTXFE                  ((uint32_t)0x00000010)         /*!< IN token received when TxFIFO is empty */
#define USB_OTG_DIEPINT_INEPNE                  ((uint32_t)0x00000040)         /*!< IN endpoint NAK effective */
#define USB_OTG_DIEPINT_TXFE                    ((uint32_t)0x00000080)         /*!< Transmit FIFO empty */
#define USB_OTG_DIEPINT_TXFIFOUDRN              ((uint32_t)0x00000100)         /*!< Transmit Fifo Underrun */
#define USB_OTG_DIEPINT_BNA                     ((uint32_t)0x00000200)         /*!< Buffer not available interrupt */
#define USB_OTG_DIEPINT_PKTDRPSTS               ((uint32_t)0x00000800)         /*!< Packet dropped status */
#define USB_OTG_DIEPINT_BERR                    ((uint32_t)0x00001000)         /*!< Babble error interrupt */
#define USB_OTG_DIEPINT_NAK                     ((uint32_t)0x00002000)         /*!< NAK interrupt */

/********************  Bit definition for USB_OTG_HCINTMSK register  ********************/
#define USB_OTG_HCINTMSK_XFRCM                   ((uint32_t)0x00000001)        /*!< Transfer completed mask */
#define USB_OTG_HCINTMSK_CHHM                    ((uint32_t)0x00000002)        /*!< Channel halted mask */
#define USB_OTG_HCINTMSK_AHBERR                  ((uint32_t)0x00000004)        /*!< AHB error */
#define USB_OTG_HCINTMSK_STALLM                  ((uint32_t)0x00000008)        /*!< STALL response received interrupt mask */
#define USB_OTG_HCINTMSK_NAKM                    ((uint32_t)0x00000010)        /*!< NAK response received interrupt mask */
#define USB_OTG_HCINTMSK_ACKM                    ((uint32_t)0x00000020)        /*!< ACK response received/transmitted interrupt mask */
#define USB_OTG_HCINTMSK_NYET                    ((uint32_t)0x00000040)        /*!< response received interrupt mask */
#define USB_OTG_HCINTMSK_TXERRM                  ((uint32_t)0x00000080)        /*!< Transaction error mask */
#define USB_OTG_HCINTMSK_BBERRM                  ((uint32_t)0x00000100)        /*!< Babble error mask */
#define USB_OTG_HCINTMSK_FRMORM                  ((uint32_t)0x00000200)        /*!< Frame overrun mask */
#define USB_OTG_HCINTMSK_DTERRM                  ((uint32_t)0x00000400)        /*!< Data toggle error mask */

/********************  Bit definition for USB_OTG_DIEPTSIZ register  ********************/

#define USB_OTG_DIEPTSIZ_XFRSIZ                  ((uint32_t)0x0007FFFF)        /*!< Transfer size */
#define USB_OTG_DIEPTSIZ_PKTCNT                  ((uint32_t)0x1FF80000)        /*!< Packet count */
#define USB_OTG_DIEPTSIZ_MULCNT                  ((uint32_t)0x60000000)        /*!< Packet count */
/********************  Bit definition for USB_OTG_HCTSIZ register  ********************/
#define USB_OTG_HCTSIZ_XFRSIZ                    ((uint32_t)0x0007FFFF)        /*!< Transfer size */
#define USB_OTG_HCTSIZ_PKTCNT                    ((uint32_t)0x1FF80000)        /*!< Packet count */
#define USB_OTG_HCTSIZ_DOPING                    ((uint32_t)0x80000000)        /*!< Do PING */
#define USB_OTG_HCTSIZ_DPID                      ((uint32_t)0x60000000)        /*!< Data PID */
#define USB_OTG_HCTSIZ_DPID_0                    ((uint32_t)0x20000000)        /*!<Bit 0 */
#define USB_OTG_HCTSIZ_DPID_1                    ((uint32_t)0x40000000)        /*!<Bit 1 */

/********************  Bit definition for USB_OTG_DIEPDMA register  ********************/
#define USB_OTG_DIEPDMA_DMAADDR                  ((uint32_t)0xFFFFFFFF)        /*!< DMA address */

/********************  Bit definition for USB_OTG_HCDMA register  ********************/
#define USB_OTG_HCDMA_DMAADDR                    ((uint32_t)0xFFFFFFFF)        /*!< DMA address */

/********************  Bit definition for USB_OTG_DTXFSTS register  ********************/
#define USB_OTG_DTXFSTS_INEPTFSAV                ((uint32_t)0x0000FFFF)        /*!< IN endpoint TxFIFO space avail */

/********************  Bit definition for USB_OTG_DIEPTXF register  ********************/
#define USB_OTG_DIEPTXF_INEPTXSA                 ((uint32_t)0x0000FFFF)        /*!< IN endpoint FIFOx transmit RAM start address */
#define USB_OTG_DIEPTXF_INEPTXFD                 ((uint32_t)0xFFFF0000)        /*!< IN endpoint TxFIFO depth */

/********************  Bit definition for USB_OTG_DOEPCTL register  ********************/

#define USB_OTG_DOEPCTL_MPSIZ                     ((uint32_t)0x000007FF)       /*!< Maximum packet size */          /*!<Bit 1 */
#define USB_OTG_DOEPCTL_USBAEP                    ((uint32_t)0x00008000)       /*!< USB active endpoint */
#define USB_OTG_DOEPCTL_NAKSTS                    ((uint32_t)0x00020000)       /*!< NAK status */
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM            ((uint32_t)0x10000000)       /*!< Set DATA0 PID */
#define USB_OTG_DOEPCTL_SODDFRM                   ((uint32_t)0x20000000)       /*!< Set odd frame */
#define USB_OTG_DOEPCTL_EPTYP                     ((uint32_t)0x000C0000)       /*!< Endpoint type */
#define USB_OTG_DOEPCTL_EPTYP_0                   ((uint32_t)0x00040000)       /*!<Bit 0 */
#define USB_OTG_DOEPCTL_EPTYP_1                   ((uint32_t)0x00080000)       /*!<Bit 1 */
#define USB_OTG_DOEPCTL_SNPM                      ((uint32_t)0x00100000)       /*!< Snoop mode */
#define USB_OTG_DOEPCTL_STALL                     ((uint32_t)0x00200000)       /*!< STALL handshake */
#define USB_OTG_DOEPCTL_CNAK                      ((uint32_t)0x04000000)       /*!< Clear NAK */
#define USB_OTG_DOEPCTL_SNAK                      ((uint32_t)0x08000000)       /*!< Set NAK */
#define USB_OTG_DOEPCTL_EPDIS                     ((uint32_t)0x40000000)       /*!< Endpoint disable */
#define USB_OTG_DOEPCTL_EPENA                     ((uint32_t)0x80000000)       /*!< Endpoint enable */

/********************  Bit definition for USB_OTG_DOEPINT register  ********************/
#define USB_OTG_DOEPINT_XFRC                    ((uint32_t)0x00000001)         /*!< Transfer completed interrupt */
#define USB_OTG_DOEPINT_EPDISD                  ((uint32_t)0x00000002)         /*!< Endpoint disabled interrupt */
#define USB_OTG_DOEPINT_STUP                    ((uint32_t)0x00000008)         /*!< SETUP phase done */
#define USB_OTG_DOEPINT_OTEPDIS                 ((uint32_t)0x00000010)         /*!< OUT token received when endpoint disabled */
#define USB_OTG_DOEPINT_B2BSTUP                 ((uint32_t)0x00000040)         /*!< Back-to-back SETUP packets received */
#define USB_OTG_DOEPINT_NYET                    ((uint32_t)0x00004000)         /*!< NYET interrupt */

/********************  Bit definition for USB_OTG_DOEPTSIZ register  ********************/

#define USB_OTG_DOEPTSIZ_XFRSIZ                  ((uint32_t)0x0007FFFF)        /*!< Transfer size */
#define USB_OTG_DOEPTSIZ_PKTCNT                  ((uint32_t)0x1FF80000)        /*!< Packet count */

#define USB_OTG_DOEPTSIZ_STUPCNT                 ((uint32_t)0x60000000)        /*!< SETUP packet count */
#define USB_OTG_DOEPTSIZ_STUPCNT_0               ((uint32_t)0x20000000)        /*!<Bit 0 */
#define USB_OTG_DOEPTSIZ_STUPCNT_1               ((uint32_t)0x40000000)        /*!<Bit 1 */

/********************  Bit definition for PCGCCTL register  ********************/
#define USB_OTG_PCGCCTL_STOPCLK                 ((uint32_t)0x00000001)         /*!< SETUP packet count */
#define USB_OTG_PCGCCTL_GATECLK                 ((uint32_t)0x00000002)         /*!<Bit 0 */
#define USB_OTG_PCGCCTL_PHYSUSP                 ((uint32_t)0x00000010)         /*!<Bit 1 */


/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup Exported_macros
  * @{
  */

/******************************* ADC Instances ********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) (((INSTANCE) == ADC1) || \
                                       ((INSTANCE) == ADC2) || \
                                       ((INSTANCE) == ADC3))

#define IS_ADC_MULTIMODE_MASTER_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)

#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC123_COMMON)

/******************************** CAN Instances ******************************/
#define IS_CAN_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CAN)

/******************************** COMP Instances ******************************/
#define IS_COMP_ALL_INSTANCE(INSTANCE) (((INSTANCE) == COMP1) || \
                                        ((INSTANCE) == COMP2))

/******************** COMP Instances with window mode capability **************/
#define IS_COMP_WINDOWMODE_INSTANCE(INSTANCE) ((INSTANCE) == COMP2)

/******************************* CRC Instances ********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)

/******************************* DAC Instances ********************************/
#define IS_DAC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == DAC1)

/****************************** DFSDM Instances *******************************/
#define IS_DFSDM_FILTER_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DFSDM_Filter0) || \
                                                ((INSTANCE) == DFSDM_Filter1) || \
                                                ((INSTANCE) == DFSDM_Filter2) || \
                                                ((INSTANCE) == DFSDM_Filter3))

#define IS_DFSDM_CHANNEL_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DFSDM_Channel0) || \
                                                 ((INSTANCE) == DFSDM_Channel1) || \
                                                 ((INSTANCE) == DFSDM_Channel2) || \
                                                 ((INSTANCE) == DFSDM_Channel3) || \
                                                 ((INSTANCE) == DFSDM_Channel4) || \
                                                 ((INSTANCE) == DFSDM_Channel5) || \
                                                 ((INSTANCE) == DFSDM_Channel6) || \
                                                 ((INSTANCE) == DFSDM_Channel7))

/******************************** DMA Instances *******************************/
#define IS_DMA_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Channel1) || \
                                       ((INSTANCE) == DMA1_Channel2) || \
                                       ((INSTANCE) == DMA1_Channel3) || \
                                       ((INSTANCE) == DMA1_Channel4) || \
                                       ((INSTANCE) == DMA1_Channel5) || \
                                       ((INSTANCE) == DMA1_Channel6) || \
                                       ((INSTANCE) == DMA1_Channel7) || \
                                       ((INSTANCE) == DMA2_Channel1) || \
                                       ((INSTANCE) == DMA2_Channel2) || \
                                       ((INSTANCE) == DMA2_Channel3) || \
                                       ((INSTANCE) == DMA2_Channel4) || \
                                       ((INSTANCE) == DMA2_Channel5) || \
                                       ((INSTANCE) == DMA2_Channel6) || \
                                       ((INSTANCE) == DMA2_Channel7))

/******************************* GPIO Instances *******************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                        ((INSTANCE) == GPIOB) || \
                                        ((INSTANCE) == GPIOC) || \
                                        ((INSTANCE) == GPIOD) || \
                                        ((INSTANCE) == GPIOE) || \
                                        ((INSTANCE) == GPIOF) || \
                                        ((INSTANCE) == GPIOG) || \
                                        ((INSTANCE) == GPIOH))

/******************************* GPIO AF Instances ****************************/
/* On L4, all GPIO Bank support AF */
#define IS_GPIO_AF_INSTANCE(INSTANCE)   IS_GPIO_ALL_INSTANCE(INSTANCE)

/**************************** GPIO Lock Instances *****************************/
/* On L4, all GPIO Bank support the Lock mechanism */
#define IS_GPIO_LOCK_INSTANCE(INSTANCE) IS_GPIO_ALL_INSTANCE(INSTANCE)

/******************************** I2C Instances *******************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                       ((INSTANCE) == I2C2) || \
                                       ((INSTANCE) == I2C3))

/******************************* LCD Instances ********************************/
#define IS_LCD_ALL_INSTANCE(INSTANCE) ((INSTANCE) == LCD)

/******************************* HCD Instances *******************************/
#define IS_HCD_ALL_INSTANCE(INSTANCE) ((INSTANCE) == USB_OTG_FS)

/****************************** OPAMP Instances *******************************/
#define IS_OPAMP_ALL_INSTANCE(INSTANCE) (((INSTANCE) == OPAMP1) || \
                                         ((INSTANCE) == OPAMP2))

/******************************* PCD Instances *******************************/
#define IS_PCD_ALL_INSTANCE(INSTANCE) ((INSTANCE) == USB_OTG_FS)

/******************************* QSPI Instances *******************************/
#define IS_QSPI_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == QUADSPI)

/******************************* RNG Instances ********************************/
#define IS_RNG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RNG)

/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)

/******************************** SAI Instances *******************************/
#define IS_SAI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SAI1_Block_A) || \
                                       ((INSTANCE) == SAI1_Block_B) || \
                                       ((INSTANCE) == SAI2_Block_A) || \
                                       ((INSTANCE) == SAI2_Block_B))

/****************************** SDMMC Instances *******************************/
#define IS_SDMMC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == SDMMC1)

/****************************** SMBUS Instances *******************************/
#define IS_SMBUS_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                         ((INSTANCE) == I2C2) || \
                                         ((INSTANCE) == I2C3))

/******************************** SPI Instances *******************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) || \
                                       ((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))

/******************************** SWPMI Instances *****************************/
#define IS_SWPMI_INSTANCE(INSTANCE)  ((INSTANCE) == SWPMI1)

/****************** LPTIM Instances : All supported instances *****************/
#define IS_LPTIM_INSTANCE(INSTANCE)     (((INSTANCE) == LPTIM1) || \
                                         ((INSTANCE) == LPTIM2))

/****************** TIM Instances : All supported instances *******************/
#define IS_TIM_INSTANCE(INSTANCE)       (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM3)   || \
                                         ((INSTANCE) == TIM4)   || \
                                         ((INSTANCE) == TIM5)   || \
                                         ((INSTANCE) == TIM6)   || \
                                         ((INSTANCE) == TIM7)   || \
                                         ((INSTANCE) == TIM8)   || \
                                         ((INSTANCE) == TIM15)  || \
                                         ((INSTANCE) == TIM16)  || \
                                         ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting 32 bits counter ****************/
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) (((INSTANCE) == TIM2)   || \
                                               ((INSTANCE) == TIM5))

/****************** TIM Instances : supporting the break function *************/
#define IS_TIM_BREAK_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)    || \
                                            ((INSTANCE) == TIM8)    || \
                                            ((INSTANCE) == TIM15)   || \
                                            ((INSTANCE) == TIM16)   || \
                                            ((INSTANCE) == TIM17))

/************** TIM Instances : supporting Break source selection *************/
#define IS_TIM_BREAKSOURCE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                               ((INSTANCE) == TIM8)   || \
                                               ((INSTANCE) == TIM15)  || \
                                               ((INSTANCE) == TIM16)  || \
                                               ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting 2 break inputs *****************/
#define IS_TIM_BKIN2_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)    || \
                                            ((INSTANCE) == TIM8))

/************* TIM Instances : at least 1 capture/compare channel *************/
#define IS_TIM_CC1_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM3)   || \
                                         ((INSTANCE) == TIM4)   || \
                                         ((INSTANCE) == TIM5)   || \
                                         ((INSTANCE) == TIM8)   || \
                                         ((INSTANCE) == TIM15)  || \
                                         ((INSTANCE) == TIM16)  || \
                                         ((INSTANCE) == TIM17))

/************ TIM Instances : at least 2 capture/compare channels *************/
#define IS_TIM_CC2_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM3)   || \
                                         ((INSTANCE) == TIM4)   || \
                                         ((INSTANCE) == TIM5)   || \
                                         ((INSTANCE) == TIM8)   || \
                                         ((INSTANCE) == TIM15))

/************ TIM Instances : at least 3 capture/compare channels *************/
#define IS_TIM_CC3_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM3)   || \
                                         ((INSTANCE) == TIM4)   || \
                                         ((INSTANCE) == TIM5)   || \
                                         ((INSTANCE) == TIM8))

/************ TIM Instances : at least 4 capture/compare channels *************/
#define IS_TIM_CC4_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM3)   || \
                                         ((INSTANCE) == TIM4)   || \
                                         ((INSTANCE) == TIM5)   || \
                                         ((INSTANCE) == TIM8))

/****************** TIM Instances : at least 5 capture/compare channels *******/
#define IS_TIM_CC5_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM8))

/****************** TIM Instances : at least 6 capture/compare channels *******/
#define IS_TIM_CC6_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM8))

/************ TIM Instances : DMA requests generation (TIMx_DIER.COMDE) *******/
#define IS_TIM_CCDMA_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM8)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16)  || \
                                            ((INSTANCE) == TIM17))

/****************** TIM Instances : DMA requests generation (TIMx_DIER.UDE) ***/
#define IS_TIM_DMA_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM3)   || \
                                            ((INSTANCE) == TIM4)   || \
                                            ((INSTANCE) == TIM5)   || \
                                            ((INSTANCE) == TIM6)   || \
                                            ((INSTANCE) == TIM7)   || \
                                            ((INSTANCE) == TIM8)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16)  || \
                                            ((INSTANCE) == TIM17))

/************ TIM Instances : DMA requests generation (TIMx_DIER.CCxDE) *******/
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM3)   || \
                                            ((INSTANCE) == TIM4)   || \
                                            ((INSTANCE) == TIM5)   || \
                                            ((INSTANCE) == TIM8)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16)  || \
                                            ((INSTANCE) == TIM17))

/******************** TIM Instances : DMA burst feature ***********************/
#define IS_TIM_DMABURST_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM3)   || \
                                            ((INSTANCE) == TIM4)   || \
                                            ((INSTANCE) == TIM5)   || \
                                            ((INSTANCE) == TIM8)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16)  || \
                                            ((INSTANCE) == TIM17))

/******************* TIM Instances : output(s) available **********************/
#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) \
    ((((INSTANCE) == TIM1) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4) ||          \
      ((CHANNEL) == TIM_CHANNEL_5) ||          \
      ((CHANNEL) == TIM_CHANNEL_6)))           \
     ||                                        \
     (((INSTANCE) == TIM2) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
     ||                                        \
     (((INSTANCE) == TIM3) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
     ||                                        \
     (((INSTANCE) == TIM4) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
     ||                                        \
     (((INSTANCE) == TIM5) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
     ||                                        \
     (((INSTANCE) == TIM8) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4) ||          \
      ((CHANNEL) == TIM_CHANNEL_5) ||          \
      ((CHANNEL) == TIM_CHANNEL_6)))           \
     ||                                        \
     (((INSTANCE) == TIM15) &&                 \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2)))           \
     ||                                        \
     (((INSTANCE) == TIM16) &&                 \
     (((CHANNEL) == TIM_CHANNEL_1)))           \
     ||                                        \
     (((INSTANCE) == TIM17) &&                 \
      (((CHANNEL) == TIM_CHANNEL_1))))

/****************** TIM Instances : supporting complementary output(s) ********/
#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM1) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3)))            \
    ||                                          \
    (((INSTANCE) == TIM8) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3)))            \
    ||                                          \
    (((INSTANCE) == TIM15) &&                   \
     ((CHANNEL) == TIM_CHANNEL_1))           \
    ||                                          \
    (((INSTANCE) == TIM16) &&                   \
     ((CHANNEL) == TIM_CHANNEL_1))              \
    ||                                          \
    (((INSTANCE) == TIM17) &&                   \
     ((CHANNEL) == TIM_CHANNEL_1)))

/****************** TIM Instances : supporting clock division *****************/
#define IS_TIM_CLOCK_DIVISION_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)    || \
                                                    ((INSTANCE) == TIM2)    || \
                                                    ((INSTANCE) == TIM3)    || \
                                                    ((INSTANCE) == TIM4)    || \
                                                    ((INSTANCE) == TIM5)    || \
                                                    ((INSTANCE) == TIM8)    || \
                                                    ((INSTANCE) == TIM15)   || \
                                                    ((INSTANCE) == TIM16)   || \
                                                    ((INSTANCE) == TIM17))

/****** TIM Instances : supporting external clock mode 1 for ETRF input *******/
#define IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3) || \
                                                        ((INSTANCE) == TIM4) || \
                                                        ((INSTANCE) == TIM5) || \
                                                        ((INSTANCE) == TIM8) || \
                                                        ((INSTANCE) == TIM15))

/****** TIM Instances : supporting external clock mode 2 for ETRF input *******/
#define IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3) || \
                                                        ((INSTANCE) == TIM4) || \
                                                        ((INSTANCE) == TIM5) || \
                                                        ((INSTANCE) == TIM8))

/****************** TIM Instances : supporting external clock mode 1 for TIX inputs*/
#define IS_TIM_CLOCKSOURCE_TIX_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3) || \
                                                        ((INSTANCE) == TIM4) || \
                                                        ((INSTANCE) == TIM5) || \
                                                        ((INSTANCE) == TIM8) || \
                                                        ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting internal trigger inputs(ITRX) *******/
#define IS_TIM_CLOCKSOURCE_ITRX_INSTANCE(INSTANCE)     (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3) || \
                                                        ((INSTANCE) == TIM4) || \
                                                        ((INSTANCE) == TIM5) || \
                                                        ((INSTANCE) == TIM8) || \
                                                        ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting combined 3-phase PWM mode ******/
#define IS_TIM_COMBINED3PHASEPWM_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                                     ((INSTANCE) == TIM8))

/****************** TIM Instances : supporting commutation event generation ***/
#define IS_TIM_COMMUTATION_EVENT_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                                     ((INSTANCE) == TIM8)   || \
                                                     ((INSTANCE) == TIM15)  || \
                                                     ((INSTANCE) == TIM16)  || \
                                                     ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting counting mode selection ********/
#define IS_TIM_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3) || \
                                                        ((INSTANCE) == TIM4) || \
                                                        ((INSTANCE) == TIM5) || \
                                                        ((INSTANCE) == TIM8))

/****************** TIM Instances : supporting encoder interface **************/
#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1)  || \
                                                      ((INSTANCE) == TIM2)  || \
                                                      ((INSTANCE) == TIM3)  || \
                                                      ((INSTANCE) == TIM4)  || \
                                                      ((INSTANCE) == TIM5)  || \
                                                      ((INSTANCE) == TIM8))

/**************** TIM Instances : external trigger input available ************/
#define IS_TIM_ETR_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2)  || \
                                            ((INSTANCE) == TIM3)  || \
                                            ((INSTANCE) == TIM4)  || \
                                            ((INSTANCE) == TIM5)  || \
                                            ((INSTANCE) == TIM8))

/************* TIM Instances : supporting ETR source selection ***************/
#define IS_TIM_ETRSEL_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)  || \
                                             ((INSTANCE) == TIM2)  || \
                                             ((INSTANCE) == TIM3)  || \
                                             ((INSTANCE) == TIM8))
      
/****** TIM Instances : Master mode available (TIMx_CR2.MMS available )********/
#define IS_TIM_MASTER_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2)  || \
                                            ((INSTANCE) == TIM3)  || \
                                            ((INSTANCE) == TIM4)  || \
                                            ((INSTANCE) == TIM5)  || \
                                            ((INSTANCE) == TIM6)  || \
                                            ((INSTANCE) == TIM7)  || \
                                            ((INSTANCE) == TIM8)  || \
                                            ((INSTANCE) == TIM15))

/*********** TIM Instances : Slave mode available (TIMx_SMCR available )*******/
#define IS_TIM_SLAVE_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2)  || \
                                            ((INSTANCE) == TIM3)  || \
                                            ((INSTANCE) == TIM4)  || \
                                            ((INSTANCE) == TIM5)  || \
                                            ((INSTANCE) == TIM8)  || \
                                            ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting OCxREF clear *******************/
#define IS_TIM_OCXREF_CLEAR_INSTANCE(INSTANCE)        (((INSTANCE) == TIM1) || \
                                                       ((INSTANCE) == TIM2) || \
                                                       ((INSTANCE) == TIM3) || \
                                                       ((INSTANCE) == TIM4) || \
                                                       ((INSTANCE) == TIM5) || \
                                                       ((INSTANCE) == TIM8))

/****************** TIM Instances : remapping capability **********************/
#define IS_TIM_REMAP_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2)  || \
                                            ((INSTANCE) == TIM3)  || \
                                            ((INSTANCE) == TIM8)  || \
                                            ((INSTANCE) == TIM15) || \
                                            ((INSTANCE) == TIM16) || \
                                            ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting repetition counter *************/
#define IS_TIM_REPETITION_COUNTER_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1)  || \
                                                       ((INSTANCE) == TIM8)  || \
                                                       ((INSTANCE) == TIM15) || \
                                                       ((INSTANCE) == TIM16) || \
                                                       ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting synchronization ****************/
#define IS_TIM_SYNCHRO_INSTANCE(INSTANCE)  IS_TIM_MASTER_INSTANCE(INSTANCE)

/****************** TIM Instances : supporting ADC triggering through TRGO2 ***/
#define IS_TIM_TRGO2_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)    || \
                                            ((INSTANCE) == TIM8))

/******************* TIM Instances : Timer input XOR function *****************/
#define IS_TIM_XOR_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM3)   || \
                                            ((INSTANCE) == TIM4)   || \
                                            ((INSTANCE) == TIM5)   || \
                                            ((INSTANCE) == TIM8)   || \
                                            ((INSTANCE) == TIM15))

/****************************** TSC Instances *********************************/
#define IS_TSC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == TSC)

/******************** UART Instances : Asynchronous mode **********************/
#define IS_UART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2) || \
                                    ((INSTANCE) == USART3) || \
                                    ((INSTANCE) == UART4)  || \
                                    ((INSTANCE) == UART5)  || \
                                    ((INSTANCE) == LPUART1))


/******************** USART Instances : Synchronous mode **********************/
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                     ((INSTANCE) == USART2) || \
                                     ((INSTANCE) == USART3))

/****************** UART Instances : Hardware Flow control ********************/
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                           ((INSTANCE) == USART2) || \
                                           ((INSTANCE) == USART3) || \
                                           ((INSTANCE) == UART4)  || \
                                           ((INSTANCE) == UART5)  || \
                                           ((INSTANCE) == LPUART1))


/********************* USART Instances : Smard card mode ***********************/
#define IS_SMARTCARD_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                         ((INSTANCE) == USART2) || \
                                         ((INSTANCE) == USART3))

/****************** UART Instances : Auto Baud Rate detection ****************/
#define IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                                            ((INSTANCE) == USART2) || \
                                                            ((INSTANCE) == USART3) || \
                                                            ((INSTANCE) == UART4)  || \
                                                            ((INSTANCE) == UART5))

/******************** UART Instances : Half-Duplex mode **********************/
#define IS_UART_HALFDUPLEX_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                                 ((INSTANCE) == USART2) || \
                                                 ((INSTANCE) == USART3) || \
                                                 ((INSTANCE) == UART4)  || \
                                                 ((INSTANCE) == UART5)  || \
                                                 ((INSTANCE) == LPUART1))

/******************** UART Instances : LIN mode **********************/
#define IS_UART_LIN_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                          ((INSTANCE) == USART2) || \
                                          ((INSTANCE) == USART3) || \
                                          ((INSTANCE) == UART4)  || \
                                          ((INSTANCE) == UART5))

/******************** UART Instances : Wake-up from Stop mode **********************/
#define IS_UART_WAKEUP_FROMSTOP_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                                      ((INSTANCE) == USART2) || \
                                                      ((INSTANCE) == USART3) || \
                                                      ((INSTANCE) == UART4)  || \
                                                      ((INSTANCE) == UART5)  || \
                                                      ((INSTANCE) == LPUART1))

/****************** UART Instances : Driver Enable *****************/
#define IS_UART_DRIVER_ENABLE_INSTANCE(INSTANCE)     (((INSTANCE) == USART1) || \
                                                      ((INSTANCE) == USART2) || \
                                                      ((INSTANCE) == USART3) || \
                                                      ((INSTANCE) == UART4)  || \
                                                      ((INSTANCE) == UART5)  || \
                                                      ((INSTANCE) == LPUART1))

/******************** UART Instances : Half-Duplex mode **********************/
#define IS_UART_HALFDUPLEX_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                                 ((INSTANCE) == USART2) || \
                                                 ((INSTANCE) == USART3) || \
                                                 ((INSTANCE) == UART4)  || \
                                                 ((INSTANCE) == UART5)  || \
                                                 ((INSTANCE) == LPUART1))

/******************** UART Instances : LIN mode **********************/
#define IS_UART_LIN_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                          ((INSTANCE) == USART2) || \
                                          ((INSTANCE) == USART3) || \
                                          ((INSTANCE) == UART4)  || \
                                          ((INSTANCE) == UART5))

/*********************** UART Instances : IRDA mode ***************************/
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2) || \
                                    ((INSTANCE) == USART3) || \
                                    ((INSTANCE) == UART4)  || \
                                    ((INSTANCE) == UART5))

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** WWDG Instances ********************************/
#define IS_WWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == WWDG)

/**
  * @}
  */


/******************************************************************************/
/*  For a painless codes migration between the STM32L4xx device product       */
/*  lines, the aliases defined below are put in place to overcome the         */
/*  differences in the interrupt handlers and IRQn definitions.               */
/*  No need to update developed interrupt code when moving across             */ 
/*  product lines within the same STM32L4 Family                              */
/******************************************************************************/

/* Aliases for __IRQn */
#define TIM8_IRQn                      TIM8_UP_IRQn

/* Aliases for __IRQHandler */
#define TIM8_IRQHandler                TIM8_UP_IRQHandler


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32L476xx_H */

/**
  * @}
  */

  /**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
