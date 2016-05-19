/**
  ******************************************************************************
  * File Name          : stm32l4xx_flash.c
  * Description        : flash driver
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 Motorola Mobility
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
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <debug.h>
#include <greybus.h>
#include "stm32l4xx_hal.h"
#include <tftf.h>

#include <stm32l4xx_flash.h>

#include <stm32_hal_mod.h>

#define FLASH_BOUNDARY           (FLASH_BASE + FLASH_SIZE)

#define PARTITION_BOOT_START      0x08000000
#define PARTITION_BOOT_END        0x08007fff
#define PARTITION_TFTF_START      0x08008000
#define PARTITION_TFTF_END        (PARTITION_TFTF_START + TFTF_HEADER_SIZE - 1)
#define PARTITION_MAIN_START      (PARTITION_TFTF_START + TFTF_HEADER_SIZE)
#define PARTITION_MAIN_END        (FLASH_BOUNDARY - FLASH_PAGE_SIZE - 1)
#define PARTITION_FLASHMODE_START (FLASH_BOUNDARY - FLASH_PAGE_SIZE)
#define PARTITION_FLASHMODE_END   (FLASH_BOUNDARY - 1)

/* Private function prototypes -----------------------------------------------*/
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);

#ifdef CONFIG_DEBUG_FLASH
void dump_partitions(void)
{
  dbgprintx32("FLASH_BOUNDARY            0x", FLASH_BOUNDARY           , "\r\n");
  dbgprintx32("PARTITION_BOOT_START      0x", PARTITION_BOOT_START     , "\r\n");
  dbgprintx32("PARTITION_BOOT_END        0x", PARTITION_BOOT_END       , "\r\n");
  dbgprintx32("PARTITION_TFTF_START      0x", PARTITION_TFTF_START     , "\r\n");
  dbgprintx32("PARTITION_TFTF_END        0x", PARTITION_TFTF_END       , "\r\n");
  dbgprintx32("PARTITION_MAIN_START      0x", PARTITION_MAIN_START     , "\r\n");
  dbgprintx32("PARTITION_MAIN_END        0x", PARTITION_MAIN_END       , "\r\n");
  dbgprintx32("PARTITION_FLASHMODE_START 0x", PARTITION_FLASHMODE_START, "\r\n");
  dbgprintx32("PARTITION_FLASHMODE_END   0x", PARTITION_FLASHMODE_END  , "\r\n");
}
#endif

uint32_t mod_get_tftf_addr(void)
{
  return (uint32_t)(PARTITION_TFTF_START);
}

uint32_t mod_get_flashmode_addr(void)
{
    return (uint32_t)(PARTITION_FLASHMODE_START);
}

uint32_t mod_get_program_start_addr(void)
{
    return PARTITION_MAIN_START;
}

uint32_t mod_get_program_end_addr(void)
{
    return PARTITION_MAIN_END;
}

void ErasePage(uint32_t pageAddress)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0, PAGEError = 0;

  /* Unlock the Flash to enable the flash control register access */
  HAL_FLASH_Unlock();

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  /* Get the 1st page to erase */
  FirstPage = GetPage(pageAddress);
  /* Get the number of pages to erase from 1st page */
  NbOfPages = 1;
  /* Get the bank */
  BankNumber = GetBank(pageAddress);
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = BankNumber;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;

  HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);

  HAL_FLASH_Lock();
}

int flash_erase(uint32_t start_addr, uint32_t size)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0, PAGEError = 0;
  uint32_t end_addr = start_addr + size;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area */

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  /* Get the 1st page to erase */
  FirstPage = GetPage(start_addr);
  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage(end_addr) - FirstPage + 1;
  /* Get the bank */
  BankNumber = GetBank(start_addr);
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = BankNumber;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;

  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
  {
    /*
      Error occurred while page erase.
      User can add here some code to deal with this error.
      PAGEError will contain the faulty page and then to know the code error on this page,
      user can call function 'HAL_FLASH_GetError()'
    */
    /* Infinite loop */
    while (1) {
    }
  }
  return 0;
}

int program_flash_unlock(void)
{
    HAL_FLASH_Unlock();
    return 0;
}

int program_flash_lock(void)
{
    HAL_FLASH_Lock();
    return 0;
}

int program_flash_dword(const uint64_t *dword)
{
    int rv;

    dbgprintx32("program_flash_dword ", PARTITION_FLASHMODE_START, "\r\n");
    rv = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, PARTITION_FLASHMODE_START, *dword);
    return rv;
}

int program_flash_data(uint32_t start, uint32_t size, uint8_t *data)
{
  uint64_t data64;
  uint32_t offset = 0;

  while (offset < size) {
    data64 = 0xFFFFFFFFFFFFFFFF;
    if((size - offset) < 8) {
      memcpy((uint8_t *)&data64, (uint8_t *)&data[offset], (size - offset));
    } else {
      memcpy((uint8_t *)&data64, (uint8_t *)&data[offset], 8);
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start, data64) == HAL_OK) {
      start += 8;
      offset += 8;
    } else {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error */
      while (1) {
      }
    }
  }
  return 0;
}

/**
 * @brief Erase the partition containing the TFTF Header
 */
void erase_tftf_header(void)
{
  ErasePage((uint32_t)(PARTITION_TFTF_START));
}

/**
 * @brief Writes the contents fo the TFTF header to the TFTF_PARTITION
 */
int program_tftf_header(uint8_t *data, uint32_t size)
{
  HAL_FLASH_Unlock();
  program_flash_data((uint32_t)(PARTITION_TFTF_START), size, data);
  HAL_FLASH_Lock();

  return 0;
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
    /* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
    /* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}
