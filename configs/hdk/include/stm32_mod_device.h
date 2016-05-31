/**
 * Copyright (c) 2015-2016 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __STM32L4XX_MOD_DEVICE_H
#define __STM32L4XX_MOD_DEVICE_H

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <greybus.h>

#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_flash.h>
#include <stm32l4xx_hal_uart.h>

#define STM32_UID_BASE                0x1fff7590

#define BOARD_REVISION                (MOD_BOARDID_PID & 0x0000FFFF)

#define GPIO_MODS_SL_BPLUS_AIN_PORT   GPIOC
#define GPIO_MODS_SL_BPLUS_AIN_PIN    GPIO_PIN_5
#define EXTI_MODS_SL_BPLUS_AIN_IRQ    EXTI1_IRQn

#define GPIO_PORT_WAKE_N              GPIOB
#define GPIO_PIN_WAKE_N               GPIO_PIN_0

#define GPIO_PORT_RFR                 GPIOC
#define GPIO_PIN_RFR                  GPIO_PIN_2

#define GPIO_PORT_MUC_INT             GPIOC
#define GPIO_PIN_MUC_INT              GPIO_PIN_13

#define GPIO_PORT_SPI_CS_N            GPIOB
#define GPIO_PIN_SPI_CS_N             GPIO_PIN_12

#define MOD_TO_BASE_SPI               SPI2
#define MOD_DEBUG_USART               USART3
#define MOD_TO_SPI_FLASH              SPI1

#define GPIO_PORT_MUC_SPI_SEL         GPIOC
#define GPIO_PIN_MUC_SPI_SEL          GPIO_PIN_6

#define GPIO_PORT_SPI1_CS_N           GPIOA
#define GPIO_PIN_SPI1_CS_N            GPIO_PIN_4

#if defined(CONFIG_GPIO_PORT_FORCE_FLASH) && \
    defined(CONFIG_GPIO_PIN_FORCE_FLASH)
# define GPIO_PORT_FORCE_FLASH         CONFIG_GPIO_PORT_FORCE_FLASH
# define GPIO_PIN_FORCE_FLASH          CONFIG_GPIO_PIN_FORCE_FLASH
#endif

#define GPIO_PORT_CONSOLE_TX          GPIOC
#define GPIO_PIN_CONSOLE_TX           GPIO_PIN_10

#define GPIO_PORT_CONSOLE_RX          GPIOC
#define GPIO_PIN_CONSOLE_RX           GPIO_PIN_11

#define GPIO_PORT_PCARD_DET_N         GPIOD
#define GPIO_PIN_PCARD_DET_N          GPIO_PIN_9
#define EXTI_IRQ_PCARD_DET_N          EXTI9_5_IRQn

#define GPIO_PORT_APBE_PWR_EN         GPIOG
#define GPIO_PIN_APBE_PWR_EN          GPIO_PIN_14

#define GPIO_PORT_APBE_RST_N          GPIOH
#define GPIO_PIN_APBE_RST_N           GPIO_PIN_1

#endif /* __STM32L4XX_MOD_DEVICE_H */
