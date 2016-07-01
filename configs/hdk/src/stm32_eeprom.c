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

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <greybus.h>
#include <debug.h>

#include <stm32_hal_mod.h>
#include <stm32_mod_device.h>
#include <stm32l476xx.h>

#define EEPROM_GPIO_PIN_SCL     GPIO_PIN_10
#define EEPROM_GPIO_PORT_SCL    GPIOB
#define EEPROM_GPIO_MODE_SCL    GPIO_MODE_AF_OD
#define EEPROM_GPIO_PULL_SCL    GPIO_NOPULL
#define EEPROM_GPIO_AF_SCL      GPIO_AF4_I2C2
#define EEPROM_GPIO_SPEED_SCL   GPIO_SPEED_HIGH

#define EEPROM_GPIO_PIN_SDA     GPIO_PIN_11
#define EEPROM_GPIO_PORT_SDA    GPIOB
#define EEPROM_GPIO_MODE_SDA    GPIO_MODE_AF_OD
#define EEPROM_GPIO_PULL_SDA    GPIO_NOPULL
#define EEPROM_GPIO_AF_SDA      GPIO_AF4_I2C2
#define EEPROM_GPIO_SPEED_SDA   GPIO_SPEED_HIGH

#define EEPROM_GPIO_PIN_WP      GPIO_PIN_9
#define EEPROM_GPIO_PORT_WP     GPIOG
#define EEPROM_GPIO_MODE_WP     GPIO_MODE_OUTPUT_PP
#define EEPROM_GPIO_PULL_WP     GPIO_NOPULL
#define EEPROM_GPIO_SPEED_WP    GPIO_SPEED_HIGH
#define EEPROM_GPIO_AF_WP       0

#define EEPROM_INSTANCE         I2C2
#define EEPROM_I2C_CLK_ENABLE() __HAL_RCC_I2C2_CLK_ENABLE()


/* Timings from Table 181 for RM0351 for 16MHz clock */
#define EEPROM_PRESC           (0x03 << 28)
#define EEPROM_SCLDEL          (0x04 << 20)
#define EEPROM_SDADEL          (0x02 << 16)
#define EEPROM_SCLH            (0x0f <<  8)
#define EEPROM_SCLL            (0x13 <<  0)

#define EEPROM_ADDR                   0xa2
#define EEPROM_WRITE_DELAY_MS           50

#define EEPROM_TIMINGR (EEPROM_PRESC | EEPROM_SCLDEL | EEPROM_SDADEL | EEPROM_SCLH | EEPROM_SCLL)

static I2C_HandleTypeDef hi2c;

/**
 * device_eeprom_read
 * Read data from eeprom
 * @param addr: offset to read
 * @param data: results written to address provided for data
 * @param size: the number of bytes to read
 * @returns the number of bytes read
 */
static int device_eeprom_read(uint8_t addr, uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_OK;
#ifdef CONFIG_EEPROM128
    uint8_t msg[2] = { 0x00, addr };
#else
    uint8_t msg[1] = { addr };
#endif
    PinState pcard_det;

    pcard_det = HAL_GPIO_ReadPin(GPIO_PORT_PCARD_DET_N, GPIO_PIN_PCARD_DET_N);
    if (pcard_det == PIN_SET) {
        /* Skip read since pcard is not present */
        return 0;
    }

    status = HAL_I2C_IsDeviceReady(&hi2c, EEPROM_ADDR, 3, 1000);
    if (status != HAL_OK) {
        dbgprintx32("i2c: Device NOT ready 0x", status, "\r\n");
        return 0;
    }

    status = HAL_I2C_Master_Transmit(&hi2c, EEPROM_ADDR, msg, sizeof(msg), 1000);
    if (status != HAL_OK) {
        dbgprintx32("i2c: FAIL ", status, "\r\n");
        dbgprintx32("i2c: EC ", hi2c.ErrorCode, "\r\n");
        return 0;
    }

    status = HAL_I2C_Master_Receive(&hi2c, EEPROM_ADDR, data, size, 1000);
    if (status != HAL_OK) {
        dbgprintx32("i2c: FAIL ", status, "\r\n");
        dbgprintx32("i2c: EC ", hi2c.ErrorCode, "\r\n");
        return 0;
    }

    return size;
}

static int device_eeprom_write(uint8_t addr, uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint16_t written = 0;
    uint16_t i;
#ifdef CONFIG_EEPROM128
    uint8_t msg[3] = { 0x00, addr , 0x00};
#else
    uint8_t msg[2] = { addr , 0x00};
#endif

    status = HAL_I2C_IsDeviceReady(&hi2c, EEPROM_ADDR, 3, 1000);
    if (status != HAL_OK) {
        dbgprintx32("i2c: Device NOT ready 0x", status, "\r\n");
        return 0;
    }

    HAL_GPIO_WritePin(EEPROM_GPIO_PORT_WP, EEPROM_GPIO_PIN_WP, GPIO_PIN_RESET);

    for (i = 0; i < size; i++) {
#ifdef CONFIG_EEPROM128
        msg[2] = data[i];
        msg[1] = addr + i;
#else
        msg[1] = data[i];
        msg[0] = addr + i;
#endif
        status = HAL_I2C_Master_Transmit(&hi2c, EEPROM_ADDR, msg, sizeof(msg), 1000);
        if (status != HAL_OK) {
            dbgprintx32("i2c: FAIL ", status, "\r\n");
            dbgprintx32("i2c: FAIL EC ", hi2c.ErrorCode, "\r\n");
            goto out;
        }
        HAL_Delay(EEPROM_WRITE_DELAY_MS);
    }

    written = size;

out:
    HAL_GPIO_WritePin(EEPROM_GPIO_PORT_WP, EEPROM_GPIO_PIN_WP, GPIO_PIN_SET);

    return written;
}

int eeprom_get_vid(uint32_t *vid)
{
  int count;

  count = device_eeprom_read(0x00, (uint8_t *)vid, sizeof(*vid));
  if (count != sizeof(*vid)) {
      return -1;
  }
  return 0;
}

int eeprom_get_pid(uint32_t *pid)
{
  int count;

  count = device_eeprom_read(0x04, (uint8_t *)pid, sizeof(*pid));
  if (count != sizeof(*pid)) {
      return -1;
  }
  return 0;
}

static uint32_t eeprom_set_ids(uint32_t vid, uint32_t pid)
{
  int count;
  uint8_t ids[8];

  memcpy(&ids[0], &vid, 4);
  memcpy(&ids[4], &pid, 4);

  count = device_eeprom_write(0x00, (uint8_t *)&ids, sizeof(ids));
  if (count != sizeof(ids))
    return -1;
  return 0;
}

int device_eeprom_program_ids(uint32_t tvid, uint32_t tpid)
{
  uint32_t evid;
  uint32_t epid;

  if (eeprom_get_vid(&evid) || eeprom_get_pid(&epid)) {
      dbgprint("error reading eeprom\r\n");
      return -1;

  }
  dbgprintx32("tvid 0x", tvid, "\r\n");
  dbgprintx32("tpid 0x", tpid, "\r\n");
  dbgprintx32("evid 0x", evid, "\r\n");
  dbgprintx32("epid 0x", epid, "\r\n");

  if ((tvid != evid) | (tpid != epid)) {
      if (eeprom_set_ids(tvid, tpid)) {
          dbgprint("error writing ids\r\n");
          return -1;
      }
      eeprom_get_vid(&evid);
      eeprom_get_pid(&epid);
      dbgprintx32("wrote 0x", evid, ":");
      dbgprintx32("0x", epid, "\r\n");
  }

  return 0;
}

void device_eeprom_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    EEPROM_I2C_CLK_ENABLE();

    GPIO_InitStruct.Pin = EEPROM_GPIO_PIN_SCL;
    GPIO_InitStruct.Mode = EEPROM_GPIO_MODE_SCL;
    GPIO_InitStruct.Pull = EEPROM_GPIO_PULL_SCL;
    GPIO_InitStruct.Speed = EEPROM_GPIO_SPEED_SCL;
    GPIO_InitStruct.Alternate = EEPROM_GPIO_AF_SCL;
    HAL_GPIO_Init(EEPROM_GPIO_PORT_SCL, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EEPROM_GPIO_PIN_SDA;
    GPIO_InitStruct.Mode = EEPROM_GPIO_MODE_SDA;
    GPIO_InitStruct.Pull = EEPROM_GPIO_PULL_SDA;
    GPIO_InitStruct.Speed = EEPROM_GPIO_SPEED_SDA;
    GPIO_InitStruct.Alternate = EEPROM_GPIO_AF_SDA;
    HAL_GPIO_Init(EEPROM_GPIO_PORT_SCL, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EEPROM_GPIO_PIN_WP;
    GPIO_InitStruct.Mode = EEPROM_GPIO_MODE_WP;
    GPIO_InitStruct.Pull = EEPROM_GPIO_PULL_WP;
    GPIO_InitStruct.Speed = EEPROM_GPIO_SPEED_WP;
    GPIO_InitStruct.Alternate = EEPROM_GPIO_AF_WP;
    HAL_GPIO_Init(EEPROM_GPIO_PORT_WP, &GPIO_InitStruct);

    hi2c.Instance = EEPROM_INSTANCE;

    hi2c.Init.Timing = EEPROM_TIMINGR;
    hi2c.Init.OwnAddress1 = 0;
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c.Init.OwnAddress2 = 0;
    hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(&hi2c);
}
