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

#include <muc_config.h>
#include <stm32_hal_mod.h>
#include <stm32_mod_device.h>

void mods_gpio_clk_enable(void)
{
  /* GPIO Ports Clock Enable */

  /* For USART */
   __GPIOA_CLK_ENABLE();

  /* For GPIOs */
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
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
  GPIO_InitStruct.Pin = GPIO_MODS_SL_BPLUS_EN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIO_MODS_SL_BPLUS_EN_PORT, &GPIO_InitStruct);
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

    ps_attached = HAL_GPIO_ReadPin(GPIO_MODS_SL_BPLUS_EN_PORT,
                                   GPIO_MODS_SL_BPLUS_EN_PIN);
    return (ps_attached == PIN_SET);
}
