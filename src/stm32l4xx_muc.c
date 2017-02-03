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

#include <chipapi.h>
#include <debug.h>
#include <stm32_hal_mod.h>
#include <stm32_mod_device.h>

#include <stm32_defs.h>

#ifdef CONFIG_DEBUG
static UART_HandleTypeDef huart;
#endif

#define OTP_BASE_ADDR      0x1FFF7000
#define OTP_WRPA_ADDR      0x1FFF7818

/* Each OTP block is 8 bytes(double word) */
#define MAX_OTP_BLOCKS  128
#define OTP_BLOCK_SIZE    8
#define MAX_KEY_REVOKES  16

struct otp_registers {
  uint64_t key_revoked[MAX_KEY_REVOKES];
  uint64_t reserved[MAX_OTP_BLOCKS - MAX_KEY_REVOKES];
};

void reset_systick(void)
{
    SysTick->CTRL = 0UL;
    SysTick->LOAD = 0UL;
    SysTick->VAL = 0UL;
}

/* manually clear all of the ARM NVIC extern interrupts */
void chip_reset_irqs(void)
{
  volatile static uint32_t *nvic_ictr = ARM_NVIC_ICTR;
  volatile static uint32_t *nvic_icer = ARM_NVIC_ICER;
  uint32_t r;

  for (r = 0; r <  *nvic_ictr; r++) {
      nvic_icer[r] = 0xffffffff;
  }
}

#ifdef CONFIG_DATALINK_SPI
bool mods_is_spi_csn(uint16_t pin)
{
  return (pin == GPIO_PIN_SPI_CS_N);
}
#endif

int get_chip_uid(uint64_t *uid_high, uint64_t *uid_low)
{
  uint32_t regval;

  regval = getreg32(STM32_UID_BASE);
  *uid_low = regval;

  regval = getreg32(STM32_UID_BASE + 4);
  *uid_low |= ((uint64_t)regval) << 32;

  regval = getreg32(STM32_UID_BASE + 8);
  *uid_high = regval;

  return 0;
}

/** System Clock Configuration
 *  80 MHz
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

#ifdef _DEBUG
/* USART init function */
void MX_USART_UART_Init(void)
{
  device_console_init();

  huart.Instance = MOD_DEBUG_USART;
  huart.Init.BaudRate = 115200;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits = UART_STOPBITS_1;
  huart.Init.Parity = UART_PARITY_NONE;
  huart.Init.Mode = UART_MODE_TX;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart.Init.OverSampling = UART_OVERSAMPLING_16;
  huart.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart);
}
#endif

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

  device_dma_init();
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  mods_gpio_clk_enable();

  /*Configure GPIO pin : RDY/RFR */
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = GPIO_PIN_RFR;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_PORT_RFR, &GPIO_InitStruct);

  mods_muc_int_set(PIN_RESET);
  mods_rfr_set(PIN_RESET);

  /*Configure GPIO pin : WAKE_N (input) */
  memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
  GPIO_InitStruct.Pin = GPIO_PIN_WAKE_N;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_PORT_WAKE_N, &GPIO_InitStruct);

  device_gpio_init();
}

int chip_is_key_revoked(int index)
{
  struct otp_registers *otp_regs;
  uint32_t data;

  otp_regs = (struct otp_registers *)(OTP_BASE_ADDR);
  data = (__IO uint32_t)otp_regs->key_revoked[index];

  dbgprintx32("OTP Revoke: ", data, "\r\n");

  if (data == 0)
  {
    return 1;
  }
  return 0;
}

/**
 * Is the bootloader read only
 */
bool chip_bootloader_is_readonly(void)
{
#ifdef CONFIG_RUN_FROM_FLASH
    return true;
#else
    uint32_t wrp1a = getreg32(OTP_WRPA_ADDR);
    uint32_t wrp1a_start_page = (wrp1a) & 0x000000FF;

    /* Are any pages in the bootloader write protected */
    if ((wrp1a_start_page < 16))
        return true;
    else
        return false;
#endif
}
