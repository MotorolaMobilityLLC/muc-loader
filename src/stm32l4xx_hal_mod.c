/**
 * Copyright (c) 2016 Motorola Mobility, LLC.
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
#include <mod_device.h>

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
    HAL_GPIO_WritePin(GPIO_PORT_WAKE_N, GPIO_PIN_WAKE_N, pstate);
}

PinState mods_wake_n_get(void)
{
    /* On this board, WAKE_N is active low */
    return HAL_GPIO_ReadPin(GPIO_PORT_WAKE_N, GPIO_PIN_WAKE_N);
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

/**
 * Get the time in milliseconds.
 */
uint32_t mods_getms(void)
{
    return HAL_GetTick();
}

#ifdef CONFIG_DATALINK_SPI
/**
 * Set the MISO line to reflect the ACK or NACK of the
 * message from the core.
 */
void mods_ack_received(bool rx_success)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    PinState pstate = rx_success ? PIN_SET : PIN_RESET;

    GPIO_InitStruct.Pin = GPIO_MODS_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIO_MODS_SPI_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIO_MODS_SPI_PORT, GPIO_MODS_SPI_MISO_PIN, pstate);
}

/**
 * Set the MOSI line as a gpio input so we can
 * read the ack back from the base
 */
void mods_ack_transmitted_setup(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_MODS_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIO_MODS_SPI_PORT, &GPIO_InitStruct);
}

/**
 * See if the transmitted value has been ACK'ed
 */
uint8_t mods_ack_transmitted_get(void)
{
    return HAL_GPIO_ReadPin(GPIO_MODS_SPI_PORT, GPIO_MODS_SPI_MOSI_PIN);
}

/**
 * Undo the configuration of the MISO and MOSI lines as a gpios.
 */
void mods_spi_restore(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_MODS_SPI_MISO_PIN | GPIO_MODS_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIO_MODS_SPI_PORT, &GPIO_InitStruct);
}
#endif

void SPI_NSS_INT_CTRL_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure GPIO pin : MUC_INT */
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = GPIO_PIN_MUC_INT;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIO_PORT_MUC_INT, &GPIO_InitStruct);

#ifdef CONFIG_DATALINK_SPI
  /*Configure GPIO pin : PB12 */
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = GPIO_PIN_SPI_CS_N;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_PORT_SPI_CS_N, &GPIO_InitStruct);
#endif

  /* Enable and set EXTI line 12 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
