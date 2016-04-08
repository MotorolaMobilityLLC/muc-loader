/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  * COPYRIGHT(c) 2015 Motorola, LLC.
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
#include <chipapi.h>
#include <debug.h>
#include <greybus.h>
#include <version.h>
#include <boot_main.h>
#include "crypto.h"
#include "datalink.h"
#include "tftf.h"

#include <stm32_hal_mod.h>
#include <spi_flash_write.h>

/* Private typedef -----------------------------------------------------------*/
typedef void (*Function_Pointer)(void);

/* Private define ------------------------------------------------------------*/
#define BOOT_PARTITION_INDEX	0
#define JUMP_ADDRESS_OFFSET	4
#define MMAP_PARTITION_NUM	4

/* Private variables ---------------------------------------------------------*/
struct memory_map {
  char *pname;
  uint32_t partition_start_address;
  uint32_t partition_end_address;
};
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static const char bootmode_flag[8] =  {'B', 'O', 'O', 'T', 'M', 'O', 'D', 'E'};
static const char flashing_flag[8] =  {'F', 'L', 'A', 'S', 'H', 'I', 'N', 'G'};

/* debug variable for why we went into flash */
#define FLASH_REASON_BOOTMODE         1
#define FLASH_REASON_FLASHING         2
#define FLASH_REASON_FLASHPIN         3
#define FLASH_REASON_INVALID_HDR      4
#define FLASH_REASON_INVALID_SIGN     5
#define FLASH_REASON_INVALID_ADDR     6

static uint32_t flash_reason;

uint32_t get_flash_reason(void)
{
    return flash_reason;
}

static const struct memory_map mmap[MMAP_PARTITION_NUM] = {
  {"nuttx", ((uint32_t)0x08008000), ((uint32_t)0x0807f800)},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
};

static uint32_t Boot2Partition(int pIndex)
{
  Function_Pointer  pJumpToFunction;
  uint32_t jumpAddress;
  uint32_t imageAddress;
#ifdef CONFIG_MOD_SIGNATURE_VALIDATION
  uint16_t sIndex = 0;

  tftf_header *tf_header = (tftf_header *)(mod_get_tftf_addr());
#endif

  if(mmap[pIndex].pname == NULL)
  {
    return FLASH_REASON_INVALID_ADDR;
  }

  imageAddress = mmap[pIndex].partition_start_address;

  if(imageAddress == 0)
  {
    return FLASH_REASON_INVALID_ADDR;
  }

  jumpAddress = *(__IO uint32_t*)(imageAddress + JUMP_ADDRESS_OFFSET);

  if((jumpAddress >= mmap[pIndex].partition_start_address)
		&& (jumpAddress <= mmap[pIndex].partition_end_address))
  {
#ifdef CONFIG_MOD_SIGNATURE_VALIDATION
    if(!valid_tftf_header(tf_header))
    {
      dbgprint("valid_tftf_header failed\r\n");
      return FLASH_REASON_INVALID_HDR;
    }

    if(validate_image_signature(tf_header, &sIndex))
    {
      dbgprint("validate_image_signature failed\r\n");
      return FLASH_REASON_INVALID_SIGN;
    }
#endif
    /* Return Clock Configuration to Default before booting */
    HAL_RCC_DeInit();
    reset_systick();
    chip_reset_irqs();

    __set_PRIMASK(0);

    /* Initialize the Stack Pointer */
    __set_MSP(*(__IO uint32_t*)imageAddress);

    pJumpToFunction = (Function_Pointer)jumpAddress;
    pJumpToFunction();
  }
  return FLASH_REASON_INVALID_ADDR;
}

enum BootState CheckFlashMode(void)
{
  char *bootModeFlag;
  enum BootState bootState = BOOT_STATE_NORMAL;

  MX_GPIO_Init();

  /* Check For Flash Mode Bit */
  bootModeFlag = (char *)(FLASHMODE_FLAG_PAGE);
  if (!memcmp(bootModeFlag, bootmode_flag, sizeof(bootmode_flag)))
  {
    flash_reason = FLASH_REASON_BOOTMODE;
    bootState = BOOT_STATE_REQUEST_FLASH;
  }

  if (!memcmp(bootModeFlag, flashing_flag, sizeof(flashing_flag)))
  {
    flash_reason = FLASH_REASON_FLASHING;
    bootState = BOOT_STATE_FLASHING;
  }

  if (mods_force_flash_get() == PIN_SET)
  {
    flash_reason = FLASH_REASON_FLASHPIN;
    bootState = BOOT_STATE_REQUEST_FLASH;
  }

  return bootState;
}

static void _init(void)
{
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART_UART_Init();
#ifdef CONFIG_APBE_FLASH
  spi_flash_hal_init();
#endif
  /* Config SPI NSS in interrupt mode */
  SPI_NSS_INT_CTRL_Config();

  dl_init();
}

int main(void)
{
  enum BootState bootState;

  SystemClock_Config();
  bootState = CheckFlashMode();

  switch(bootState) {
  case BOOT_STATE_NORMAL:
    MX_USART_UART_Init();
    flash_reason = Boot2Partition(BOOT_PARTITION_INDEX);
  case BOOT_STATE_REQUEST_FLASH:
    /* Erase the Flash Mode Barker */
    ErasePage((uint32_t)(FLASHMODE_FLAG_PAGE));
    /* fall through */
  case BOOT_STATE_FLASHING:
    /* fall through */
  default:
    break;
  }

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  _init();

  dbgprint("\r\n--[MuC Loader v" CONFIG_VERSION_STRING ":" CONFIG_VERSION_BUILD "]\r\n");
  dbgprintx32("-Flash Mode (", flash_reason, ")\r\n");

  while (1) {
    if (!mod_dev_is_attached()) {
      dbgprint("Detached - STOP2\r\n");
      HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
      _init();
      dbgprint("Back\r\n");
    }

    setup_exchange();
  }
}

int get_board_id(uint32_t *vend_id, uint32_t *prod_id)
{
  if (vend_id) {
    *vend_id = MOD_BOARDID_VID;
  }

  if (prod_id) {
    *prod_id = MOD_BOARDID_PID;
  }

  return 0;
}

int get_chip_id(uint32_t *mfg_id, uint32_t *prod_id)
{
  if (mfg_id) {
    /* MIPI Manufacturer ID from http://mid.mipi.org/ */
    *mfg_id = 0x0104;
  }

  if (prod_id) {
    *prod_id = HAL_GetDEVID();
  }

  return 0;
}

int set_flashing_flag(void)
{
  char *bootModeFlag;

  /* Flash Mode Flag */
  bootModeFlag = (char *)(FLASHMODE_FLAG_PAGE);
  if (memcmp(bootModeFlag, flashing_flag, sizeof(flashing_flag)))
  {
    /* write the flashmode flag */
    return program_flash_data((uint32_t)(FLASHMODE_FLAG_PAGE),
                      sizeof(flashing_flag), (uint8_t *)&flashing_flag[0]);
  } else {
    return 0;
  }
}

int set_request_flash(void)
{
  int rv;
  program_flash_unlock();
  rv = program_flash_dword((uint64_t *)&bootmode_flag[0]);
  program_flash_lock();
  return rv;
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (mods_is_spi_csn(GPIO_Pin)) {
    mods_rfr_set(PIN_RESET);
    mods_muc_int_set(PIN_RESET);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *_hspi)
{
  if (_hspi->Instance == MOD_TO_BASE_SPI) {
    dl_spi_error_handler(_hspi);
  }
#ifdef CONFIG_APBE_FLASH
  else if (_hspi->Instance == MOD_TO_SPI_FLASH) {
    spi_flash_error_handler(_hspi);
  }
#endif
  else {
    dbgprintx32("ERR Invalid hspi ", (uint32_t)_hspi->Instance, "\r\n");
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *_hspi)
{
  if (_hspi->Instance == MOD_TO_BASE_SPI) {
    dl_spi_transfer_complete(_hspi);
  }
#ifdef CONFIG_APBE_FLASH
  else if (_hspi->Instance == MOD_TO_SPI_FLASH) {
    spi_flash_transfer_complete(_hspi);
  }
#endif
  else {
    dbgprintx32("TxRx Invalid hspi ", (uint32_t)_hspi->Instance, "\r\n");
  }
}

#ifdef USE_FULL_ASSERT
/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

  HAL_NVIC_SystemReset();
}
#endif
