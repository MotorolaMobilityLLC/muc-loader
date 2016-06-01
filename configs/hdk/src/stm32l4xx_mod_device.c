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

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <greybus.h>

#include <stm32_hal_mod.h>
#include <stm32_mod_device.h>
#include <stm32l476xx.h>

void mods_gpio_clk_enable(void)
{
  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  /* needed for port g */
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_PWR_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
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

#ifdef GPIO_PIN_FORCE_FLASH
  /* Configure GPIO pin : FORCE FLASH */
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = GPIO_PIN_FORCE_FLASH;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_PORT_FORCE_FLASH, &GPIO_InitStruct);
#else
  /* Use INT GPIO as 'force flash' */
  /* will be changed to output during spi setup in SPI_NSS_INT_CTRL_Config */
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = GPIO_PIN_MUC_INT;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_PORT_MUC_INT, &GPIO_InitStruct);
#endif

  /* Attach Detection */
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = GPIO_MODS_SL_BPLUS_AIN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_MODS_SL_BPLUS_AIN_PORT, &GPIO_InitStruct);

  /* Configure GPIO pin : MUC_SPI_SEL */
  GPIO_InitStruct.Pin = GPIO_PIN_MUC_SPI_SEL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIO_PORT_MUC_SPI_SEL, &GPIO_InitStruct);

  /* APBE power enable and reset */
  GPIO_InitStruct.Pin = GPIO_PIN_APBE_PWR_EN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIO_PORT_APBE_PWR_EN, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_APBE_RST_N;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIO_PORT_APBE_RST_N, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI_MODS_SL_BPLUS_AIN_IRQ, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI_MODS_SL_BPLUS_AIN_IRQ);

#ifdef CONFIG_EEPROM_IDS
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = GPIO_PIN_PCARD_DET_N;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING | GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(GPIO_PORT_PCARD_DET_N, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI_IRQ_PCARD_DET_N, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI_IRQ_PCARD_DET_N);
#endif
}

#ifdef CONFIG_EEPROM_IDS
static bool device_is_pcard_attach(uint16_t GPIO_Pin)
{
    return (GPIO_Pin == GPIO_PIN_9);
}

static void device_handle_pcard_attach()
{
    HAL_NVIC_SystemReset();
}
#endif

void device_handle_exti(uint16_t GPIO_Pin)
{
#ifdef CONFIG_EEPROM_IDS
  if (device_is_pcard_attach(GPIO_Pin)) {
      device_handle_pcard_attach();
  }
#endif
}

void device_spi_mod_init(SPI_HandleTypeDef *_hspi)
{
  _hspi->Instance = MOD_TO_BASE_SPI;
  _hspi->Init.Mode = SPI_MODE_SLAVE;
  _hspi->Init.Direction = SPI_DIRECTION_2LINES;
  _hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  _hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  _hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  _hspi->Init.NSS = SPI_NSS_HARD_INPUT;
  _hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  _hspi->Init.TIMode = SPI_TIMODE_DISABLED;
  _hspi->Init.CRCCalculation = SPI_CRCCALCULATION_ENABLED;
  _hspi->Init.CRCPolynomial = 0x8005;
  _hspi->Init.CRCLength = SPI_CRC_LENGTH_16BIT;
  _hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLED;

  HAL_SPI_Init(_hspi);

  /* Enable Software Slave Management to prevent spurious receives */
  _hspi->Instance->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
}

void device_dma_init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* SPI1 DMA used for spi flash stage*/
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

void device_spi_flash_init(SPI_HandleTypeDef *_hspi)
{
  apbe_reset();

  /* Set spi sel gpio to enable muc to flash spi interface */
  mods_muc_set_spi_sel(PIN_SET);

  _hspi->Instance = MOD_TO_SPI_FLASH;
  _hspi->Init.Mode = SPI_MODE_MASTER;
  _hspi->Init.Direction = SPI_DIRECTION_2LINES;
  _hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  _hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  _hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  _hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  _hspi->Init.NSS = SPI_NSS_HARD_OUTPUT;;
  _hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  _hspi->Init.TIMode = SPI_TIMODE_DISABLED;
  _hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  _hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLED;

  HAL_SPI_Init(_hspi);
}

PinState mods_force_flash_get(void)
{
#ifdef GPIO_PIN_FORCE_FLASH
  return HAL_GPIO_ReadPin(GPIO_PORT_FORCE_FLASH, GPIO_PIN_FORCE_FLASH);
#else
  return HAL_GPIO_ReadPin(GPIO_PORT_MUC_INT, GPIO_PIN_MUC_INT);
#endif
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

/* Hold apbe in reset while spi flashing */
void apbe_reset()
{
  HAL_GPIO_WritePin(GPIO_PORT_APBE_PWR_EN, GPIO_PIN_APBE_PWR_EN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIO_PORT_APBE_RST_N, GPIO_PIN_APBE_RST_N, GPIO_PIN_RESET);
}

PinState mods_muc_get_spi_sel(void)
{
    return HAL_GPIO_ReadPin(GPIO_PORT_MUC_SPI_SEL, GPIO_PIN_MUC_SPI_SEL);
}

void mods_muc_set_spi_sel(PinState pstate)
{
     HAL_GPIO_WritePin(GPIO_PORT_MUC_SPI_SEL, GPIO_PIN_MUC_SPI_SEL, pstate);
}

void mods_muc_set_spi1_cs(PinState pstate)
{
    HAL_GPIO_WritePin(GPIO_PORT_SPI1_CS_N, GPIO_PIN_SPI1_CS_N, pstate);
}
