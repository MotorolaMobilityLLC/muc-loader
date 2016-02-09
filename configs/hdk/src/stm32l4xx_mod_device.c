/**
 * Copyright (c) 2015 Motorola.
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

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <greybus.h>

#include <stm32_hal_mod.h>
#include <stm32_mod_device.h>

void mods_gpio_clk_enable(void)
{
  /* GPIO Ports Clock Enable */

  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
}

void device_console_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __USART3_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_CONSOLE_TX;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIO_PORT_CONSOLE_TX, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_CONSOLE_RX;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIO_PORT_CONSOLE_RX, &GPIO_InitStruct);
}

void device_console_deinit(void)
{
    __USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIO_PORT_CONSOLE_TX, GPIO_PIN_CONSOLE_TX);
    HAL_GPIO_DeInit(GPIO_PORT_CONSOLE_RX, GPIO_PIN_CONSOLE_RX);
}

void device_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure GPIO pin : FORCE FLASH */
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = GPIO_PIN_FORCE_FLASH;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_PORT_FORCE_FLASH, &GPIO_InitStruct);

  /* Attach Detection */
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = GPIO_MODS_SL_BPLUS_AIN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_MODS_SL_BPLUS_AIN_PORT, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI_MODS_SL_BPLUS_AIN_IRQ, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI_MODS_SL_BPLUS_AIN_IRQ);
}

PinState mods_force_flash_get(void)
{
  return HAL_GPIO_ReadPin(GPIO_PORT_FORCE_FLASH, GPIO_PIN_FORCE_FLASH);
}

void mod_dev_base_spi_reset(void)
{
  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();
}

/* Is the mod currently attached to a base? */
bool mod_dev_is_attached(void)
{
    PinState ps_attached;

    ps_attached = HAL_GPIO_ReadPin(GPIO_MODS_SL_BPLUS_AIN_PORT,
                                   GPIO_MODS_SL_BPLUS_AIN_PIN);
    return (ps_attached == PIN_SET);
}
