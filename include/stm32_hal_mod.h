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

#ifndef __STM32_HAL_MOD_H
#define __STM32_HAL_MOD_H

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <greybus.h>
#include <stm32_mod_device.h>

#define getreg32(a)         (*(volatile uint32_t *)(a))

typedef enum
{
  PIN_RESET = 0,
  PIN_SET
} PinState;

#ifdef CONFIG_DATALINK_SPI
extern bool mods_is_spi_csn(uint16_t pin);
#endif
extern void mods_rfr_set(PinState pstate);
extern PinState mods_rfr_get(void);
extern void mods_wake_n_set(PinState pstate);
extern PinState mods_wake_n_get(void);
extern void mods_muc_int_set(PinState pstate);
extern PinState mods_muc_get_spi_sel(void);
extern void mods_muc_spi_sel(PinState pstate);
extern PinState mods_muc_int_get(void);
extern uint32_t mods_getms(void);
extern void mods_ack_received(bool rx_success);
extern uint8_t mods_ack_transmitted_get(void);
extern void mods_ack_transmitted_setup(void);
extern void mods_spi_restore(void);
extern void SPI_NSS_INT_CTRL_Config(void);

extern void mods_gpio_clk_enable(void);
extern void device_gpio_init(void);
extern void device_console_init(void);
extern void device_console_deinit(void);
extern void device_dma_init(void);
#ifdef CONFIG_DATALINK_SPI
extern void device_spi_mod_init(SPI_HandleTypeDef *_hspi);
#endif
#ifdef CONFIG_DATALINK_I2C
extern void device_i2c_mod_init(I2C_HandleTypeDef *_hi2c);
#endif
extern void device_spi_flash_init(SPI_HandleTypeDef *_hspi);
extern void device_handle_exti(uint16_t GPIO_Pin);
extern PinState mods_force_flash_get(void);
extern void mod_dev_base_spi_reset(void);
/* Is the mod currently attached to a base? */
extern bool mod_dev_is_attached(void);

/* apbe reset */
extern void apbe_reset(void);

/* spi flash init */
extern void MX_SPI_FLASH_Init(void);

/* spi flash selection gpio get/set functions*/
extern PinState mods_muc_get_spi_sel(void);
extern void mods_muc_set_spi_sel(PinState ps);

/* set spi flash interface chip select PIN */
extern void mods_muc_set_spi1_cs(PinState ps);

/* from main.c */
extern void SystemClock_Config(void);
#ifdef _DEBUG
extern void MX_USART_UART_Init(void);
#else
static inline void MX_USART_UART_Init(void) {}
#endif
extern void MX_GPIO_Init(void);
extern void MX_DMA_Init(void);
extern int get_chip_uid(uint64_t *uid_high, uint64_t *uid_low);
extern void reset_systick(void);

void ErasePage(uint32_t pageAddress);
int flash_erase(uint32_t start_addr, uint32_t size);
int program_flash_data(uint32_t start, uint32_t size, uint8_t *data);
int program_flash_lock(void);
int program_flash_unlock(void);
int program_flash_dword(const uint64_t *dword);
uint32_t mod_get_tftf_addr(void);
extern uint32_t mod_get_flashmode_addr(void);
extern uint32_t mod_get_flashmode_addr(void);
extern uint32_t mod_get_program_start_addr(void);
extern uint32_t mod_get_program_end_addr(void);
#ifdef CONFIG_DEBUG_FLASH
extern void dump_partitions(void);
#endif

# ifdef CONFIG_SLAVE_APBE
extern void apbe_power_on_flash(void);
extern void apbe_power_off(void);
# else
static inline void apbe_power_on_flash(void) {}
static inline void apbe_power_off(void) {}
# endif

#endif /* __STM32_HAL_MOD_H */
