/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  * Copyright (c) 2015-2016 Motorola Mobility, LLC.
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
#include <mod_ids.h>
#include <version.h>
#include <boot_main.h>
#include "crypto.h"
#include "datalink.h"
#include "ramlog.h"
#include "tftf.h"
#include "eeprom.h"

#include <stm32_hal_mod.h>
#include <spi_flash_write.h>

/* Private typedef -----------------------------------------------------------*/
typedef void (*Function_Pointer)(void);

/* Private define ------------------------------------------------------------*/
#define JUMP_ADDRESS_OFFSET	4

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static const char bootmode_flag[8] =  {'B', 'O', 'O', 'T', 'M', 'O', 'D', 'E'};
static const char flashing_flag[8] =  {'F', 'L', 'A', 'S', 'H', 'I', 'N', 'G'};
static uint32_t tvid;
static uint32_t tpid;
static uint32_t evid;
static uint32_t epid;
#ifdef CONFIG_VALIDATE_PID
static bool validate_pid = 1;
#else
static bool validate_pid;
#endif
#ifdef CONFIG_VALIDATE_VID
static bool validate_vid = 1;
#else
static bool validate_vid;
#endif

/* debug variable for why we went into flash */
#define FLASH_REASON_BOOTMODE         1
#define FLASH_REASON_FLASHING         2
#define FLASH_REASON_FLASHPIN         3
#define FLASH_REASON_INVALID_HDR      4
#define FLASH_REASON_INVALID_SIGN     5
#define FLASH_REASON_INVALID_ADDR     6
#define FLASH_REASON_NEW_IDS          7

static uint32_t flash_reason;

uint32_t get_flash_reason(void)
{
    return flash_reason;
}

static uint32_t Boot2Partition(void)
{
  Function_Pointer  pJumpToFunction;
  uint32_t jumpAddress;
  uint32_t imageAddress;
#ifdef CONFIG_MOD_SIGNATURE_VALIDATION
  uint16_t sIndex = 0;
#endif
  tftf_header *tf_header = (tftf_header *)(mod_get_tftf_addr());

  if (!valid_tftf_header(tf_header))
  {
     dbgprint("valid_tftf_header failed\r\n");
     return FLASH_REASON_INVALID_HDR;
  }

  if (tftf_get_load_addr(tf_header, TFTF_SECTION_RAW_CODE, &imageAddress))
    return FLASH_REASON_INVALID_ADDR;

  jumpAddress = *(__IO uint32_t*)(imageAddress + JUMP_ADDRESS_OFFSET);

  if ((jumpAddress >= mod_get_program_start_addr()) &&
      (jumpAddress <= mod_get_program_end_addr()))
  {
#ifdef CONFIG_MOD_SIGNATURE_VALIDATION
    if (validate_image_signature(tf_header, &sIndex))
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


static inline bool pcard_changed(void)
{
  bool changed = false;

  tftf_header *hdr = (tftf_header *)(mod_get_tftf_addr());
  tvid = tftf_get_vid(hdr);
  tpid = tftf_get_pid(hdr);

  /* Read the eeprom vid/pid, if that fails all we can
   * do is fall back to the base
   */
  get_board_id(&evid, &epid);

  if ((validate_vid && (tvid != evid)) ||
      (validate_pid && (tpid != epid))) {
    changed = true;
  }
  return changed;
}

enum BootState CheckFlashMode(void)
{
  char *bootModeFlag;
  enum BootState bootState = BOOT_STATE_NORMAL;

  MX_GPIO_Init();

  if (pcard_changed())
  {
      flash_reason = FLASH_REASON_NEW_IDS;
      bootState = BOOT_STATE_REQUEST_FLASH;
      return bootState;
  }

  /* Check For Flash Mode Bit */
  bootModeFlag = (char *)mod_get_flashmode_addr();
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

static void _init(bool stay_in_bl)
{
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART_UART_Init();

  if (stay_in_bl)
  {
    MX_DMA_Init();

#ifdef CONFIG_APBE_FLASH
    spi_flash_hal_init();
#endif
    /* Config SPI NSS in interrupt mode */
    SPI_NSS_INT_CTRL_Config();

    dl_init();
  }
}

#ifdef CONFIG_EEPROM_PROGRAMMING
void do_eeprom_programming(void)
{
#ifdef CONFIG_EEPROM_PROGRAMMING_ERASE_IDS
  uint32_t tvid = 0xffffffff;
  uint32_t tpid = 0xffffffff;
#else
  tftf_header *hdr = (tftf_header *)(mod_get_tftf_addr());
  uint32_t tvid = tftf_get_vid(hdr);
  uint32_t tpid = tftf_get_pid(hdr);
#endif

  if (device_eeprom_program_ids(tvid, tpid))
      dbgprint("error programming ids\r\n");
}
#endif

int main(void)
{
  enum BootState bootState;

  SystemClock_Config();

#if defined(CONFIG_EEPROM_PROGRAMMING) || defined(CONFIG_EEPROM_IDS)
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  _init(false);

  device_eeprom_init();

#ifdef CONFIG_EEPROM_PROGRAMMING
  do_eeprom_programming();
#endif
#endif

  bootState = CheckFlashMode();

  switch(bootState) {
  case BOOT_STATE_NORMAL:
    MX_USART_UART_Init();
    flash_reason = Boot2Partition();
    break;
  case BOOT_STATE_REQUEST_FLASH:
    /* Erase the Flash Mode Barker */
    clr_flash_barker();
    /* fall through */
  case BOOT_STATE_FLASHING:
    /* fall through */
  default:
    break;
  }

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  _init(true);

  dbgprint("\r\n--[MuC Loader v" CONFIG_VERSION_STRING ":" CONFIG_VERSION_BUILD "]\r\n");
  dbgprintx32("-Flash Mode (", flash_reason, ")\r\n");
  if (flash_reason == FLASH_REASON_NEW_IDS) {
      dbgprint("        TFTF    BOOTLOADER  CHECKED    \r\n");
      dbgprintx32("VID: 0x", tvid, "\t"); dbgprintx32("0x", evid, "");
      if (validate_vid)
          dbgprint("   YES\r\n");
      else
          dbgprint("   NO\r\n");
      dbgprintx32("PID: 0x", tpid, "\t"); dbgprintx32("0x", epid, "");
      if (validate_pid)
          dbgprint("   YES\r\n");
      else
          dbgprint("   NO\r\n");
  }

  ramlog_reason();

  while (1) {
    if (!mod_dev_is_attached()) {
      dl_exit();
#ifdef CONFIG_SPIN_WHILE_DETACHED
      dbgprint("Detached - SPINNING\r\n");
      while (!mod_dev_is_attached());
#else
      dbgprint("Detached - STOP2\r\n");
      HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
#endif
      _init(true);
      dbgprint("Back\r\n");
    }

    setup_exchange();
  }
}

int get_board_id(uint32_t *vid, uint32_t *pid)
{
#ifdef CONFIG_EEPROM_IDS
  if (!eeprom_get_vid(vid) && !eeprom_get_pid(pid))
      return 0;
#endif

  if (vid) {
    *vid = _ids.board_vid;
  }

  if (pid) {
    *pid = _ids.board_pid;
  }

  return 0;
}

int get_chip_id(uint32_t *mfg, uint32_t *pid)
{
  if (mfg) {
    /* MIPI Manufacturer ID from http://mid.mipi.org/ */
    *mfg = _ids.chip_mfg;
  }

  if (pid) {
    *pid = _ids.chip_pid;
  }

  return 0;
}

int set_flashing_flag(void)
{
  int ret = 0;
  char *bootModeFlag;

  /* Flash Mode Flag */
  bootModeFlag = (char *)mod_get_flashmode_addr();
  if (memcmp(bootModeFlag, flashing_flag, sizeof(flashing_flag)))
  {
    /* write the flashmode flag */
    program_flash_unlock();
    ret = program_flash_data(mod_get_flashmode_addr(),
                             sizeof(flashing_flag),
                             (uint8_t *)&flashing_flag[0]);
    program_flash_lock();
  }
  return ret;
}

void clr_flash_barker(void)
{
    ErasePage(mod_get_flashmode_addr());
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
#ifdef CONFIG_DATALINK_SPI
  if (mods_is_spi_csn(GPIO_Pin)) {
    mods_rfr_set(PIN_RESET);
    mods_muc_int_set(PIN_RESET);
  } else
#endif
  {
      device_handle_exti(GPIO_Pin);
  }
}

#if defined(CONFIG_DATALINK_SPI) || defined(CONFIG_APBE_FLASH)
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *_hspi)
{
#ifdef CONFIG_DATALINK_SPI
  if (_hspi->Instance == MOD_TO_BASE_SPI) {
    dl_spi_error_handler(_hspi);
  } else
#endif
#ifdef CONFIG_APBE_FLASH
  if (_hspi->Instance == MOD_TO_SPI_FLASH) {
    spi_flash_error_handler(_hspi);
  } else
#endif
  {
    dbgprintx32("ERR Invalid hspi ", (uint32_t)_hspi->Instance, "\r\n");
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *_hspi)
{
#ifdef CONFIG_DATALINK_SPI
  if (_hspi->Instance == MOD_TO_BASE_SPI) {
    dl_spi_transfer_complete(_hspi);
  } else
#endif
#ifdef CONFIG_APBE_FLASH
  if (_hspi->Instance == MOD_TO_SPI_FLASH) {
    spi_flash_transfer_complete(_hspi);
  } else
#endif
  {
    dbgprintx32("TxRx Invalid hspi ", (uint32_t)_hspi->Instance, "\r\n");
  }
}
#endif

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
