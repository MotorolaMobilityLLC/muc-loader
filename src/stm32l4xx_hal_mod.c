/**
 * Copyright (c) 2016 Motorola.
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

#include <stm32_hal_mod.h>
#include <stm32_mod_device.h>

#define STM32_UID_BASE       0x1fff7590     /* 0x1fff7590-0x1fff759b: UID */
#define getreg32(a)         (*(volatile uint32_t *)(a))

void mods_rfr_set(PinState pstate)
{
    /* On this board, RFR is active high */
    HAL_GPIO_WritePin(GPIO_PORT_RFR, GPIO_PIN_RFR, pstate);
}

PinState mods_rfr_get(void)
{
    /* On this board, RFR is active high */
    return HAL_GPIO_ReadPin(GPIO_PORT_RFR, GPIO_PIN_RFR);
}

void mods_wake_n_set(PinState pstate)
{
    /* On this board, WAKE_N is active low */
    HAL_GPIO_WritePin(GPIO_PORT_WAKE_N, GPIO_PIN_0, pstate);
}

PinState mods_wake_n_get(void)
{
    /* On this board, WAKE_N is active low */
    return HAL_GPIO_ReadPin(GPIO_PORT_WAKE_N, GPIO_PIN_0);
}

void mods_muc_int_set(PinState pstate)
{
    /* On this board, MUC_INT is active high */
    HAL_GPIO_WritePin(GPIO_PORT_MUC_INT, GPIO_PIN_MUC_INT, pstate);
}

PinState mods_muc_int_get(void)
{
    /* On this board, MUC_INT is active high */
    return HAL_GPIO_ReadPin(GPIO_PORT_MUC_INT, GPIO_PIN_MUC_INT);
}

void SPI_NSS_INT_CTRL_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  /* GPIOB has to be enabled by now */

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_SPI_CS_N;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_PORT_SPI_CS_N, &GPIO_InitStruct);

  /* Enable and set EXTI line 12 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

