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
#include <debug.h>
#include <greybus.h>
#include <version.h>
#include <boot_main.h>
#include "crypto.h"
#include "datalink.h"
#include "tftf.h"

#include <stm32_hal_mod.h>


/* Private typedef -----------------------------------------------------------*/
typedef void (*Function_Pointer)(void);

/* Private define ------------------------------------------------------------*/
#define BOOT_PARTITION_INDEX	0
#define FLASH_LOADER_INDEX	2
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
#define FLASH_REASON_BOOTMODE 1
#define FLASH_REASON_FLASHING 2
#define FLASH_REASON_FLASHPIN 3
#define FLASH_REASON_BOOTFAIL 4

static uint32_t flash_reason;

static const struct memory_map mmap[MMAP_PARTITION_NUM] = {
  {"nuttx", ((uint32_t)0x08008000), ((uint32_t)0x0807f800)},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
};

SPI_HandleTypeDef hspi;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

/* Buffer used for transmission */
uint8_t aTxBuffer[MAX_DMA_BUF_SIZE];
/* Buffer used for reception */
uint8_t aRxBuffer[MAX_DMA_BUF_SIZE];

volatile bool  armDMA = false;
volatile bool respReady = true;
uint16_t negotiated_pl_size;

/* USER CODE END PV */
struct ring_buf *txp_rb;
e_armDMAtype armDMAtype;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static int process_network_msg(struct mods_spi_msg *spi_msg);
void Boot2Partition(int pIndex);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* Private functions ---------------------------------------------------------*/
void Boot2Partition(int pIndex)
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
    return;
  }

  imageAddress = mmap[pIndex].partition_start_address;

  if(imageAddress == 0)
  {
    return;
  }

  jumpAddress = *(__IO uint32_t*)(imageAddress + JUMP_ADDRESS_OFFSET);

  if((jumpAddress >= mmap[pIndex].partition_start_address)
		&& (jumpAddress <= mmap[pIndex].partition_end_address))
  {
#ifdef CONFIG_MOD_SIGNATURE_VALIDATION
    if(!valid_tftf_header(tf_header))
    {
      dbgprint("valid_tftf_header failed\r\n");
      return;
    }

    if(validate_image_signature(tf_header, &sIndex))
    {
      dbgprint("validate_image_signature failed\r\n");
      return;
    }
#endif
    __set_PRIMASK(0);

    /* Initialize the Stack Pointer */
    __set_MSP(*(__IO uint32_t*)imageAddress);

    pJumpToFunction = (Function_Pointer)jumpAddress;
    pJumpToFunction();
  }
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

/* USER CODE END 0 */
void setup_exchange(void)
{
  uint32_t buf_size;

  if (!mod_dev_is_attached()) {
    dbgprint("Detached\r\n");
    HAL_NVIC_SystemReset();
  }

  /* Start the Full Duplex Communication process */
  /* While the SPI in TransmitReceive process, user can transmit data through
     "aTxBuffer" buffer & receive data through "aRxBuffer" */
  if (armDMA == true) {
    /* Response is ready, signal INT to base */
    if (respReady == true) {
       mods_muc_int_set(PIN_SET);
    }

    /* Wait for WAKE_N to arm DMA */
    if (mods_wake_n_get() == PIN_SET) {
      return;
    }

    dbgprint("WKE-L\r\n");

    /* select DMA buffer size */
    if(armDMAtype == initial)
      buf_size = INITIAL_DMA_BUF_SIZE + DL_HEADER_BITS_SIZE;
    else
      buf_size = negotiated_pl_size + DL_HEADER_BITS_SIZE;

    if (HAL_SPI_TransmitReceive_DMA(&hspi, (uint8_t*)aTxBuffer,
                                    (uint8_t *)aRxBuffer, buf_size) != HAL_OK) {
      /* Transfer error in transmission process */
      Error_Handler();
    }

    dbgprinthex32(buf_size);
    dbgprint("--ARMED\r\n");

    armDMA = false;
    mods_rfr_set(PIN_SET);
  }
}

int main(void)
{
  enum BootState bootState = CheckFlashMode();

  switch(bootState) {
  case BOOT_STATE_NORMAL:
    MX_USART_UART_Init();
    Boot2Partition(BOOT_PARTITION_INDEX);
    flash_reason = FLASH_REASON_BOOTFAIL;
  case BOOT_STATE_REQUEST_FLASH:
    /* Erase the Flash Mode Barker */
    ErasePage((uint32_t)(FLASHMODE_FLAG_PAGE));
    /* fall through */
  case BOOT_STATE_FLASHING:
    /* fall through */
  default:
    break;
  }

#ifdef CONFIG_NO_FLASH
  /* fallback to booting to flash loader */
  dbgprint("\r\nFLASHING\r\n");
  Boot2Partition(FLASH_LOADER_INDEX);
#endif

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI_Init();
  MX_USART_UART_Init();

  dbgprint("\r\n--[MuC Loader v" CONFIG_VERSION_STRING ":" CONFIG_VERSION_BUILD "]\r\n");
  dbgprintx32("-Flash Mode (", flash_reason, ")\r\n");

  /* Config SPI NSS in interrupt mode */
  SPI_NSS_INT_CTRL_Config();

  armDMA = true;
  respReady = false;
  armDMAtype = initial;
  dl_init();
  negotiated_pl_size = INITIAL_DMA_BUF_SIZE;

  while (1) {
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

static int process_network_msg(struct mods_spi_msg *spi_msg)
{
  struct mods_spi_msg *dl = spi_msg;

  if (spi_msg->hdr_bits & HDR_BIT_VALID) {
    if ((spi_msg->hdr_bits & HDR_BIT_TYPE) == MSG_TYPE_NW ) {
      /* Process mods message */
      process_mods_msg(&spi_msg->m_msg);
    } else if ((spi_msg->hdr_bits & HDR_BIT_TYPE) == MSG_TYPE_DL ) {
      process_mods_dl_msg(&dl->dl_msg);
    } else {
      return 0;
    }
  } else if (respReady) {
    /* we were sending a message so handle it */
    respReady = false;
    process_sent_complete();
  } else {
    dbgprint("UNEXPECTED MSG!!\r\n");
  }
  return 0;
}

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  memset(aTxBuffer,0,MAX_DMA_BUF_SIZE);
  process_network_msg((struct mods_spi_msg *)aRxBuffer);
  memset(aRxBuffer,0,MAX_DMA_BUF_SIZE);

  armDMA = true;
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

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  dbgprint("SPIERR\r\n");

  /* reset spi */
  HAL_SPI_DeInit(hspi);
  dbgprint("DeInit called\r\n");
  mod_dev_base_spi_reset();
  MX_SPI_Init();
  dbgprint("Re-Init\r\n");
  memset(aTxBuffer, 0, MAX_DMA_BUF_SIZE);
  memset(aRxBuffer, 0, MAX_DMA_BUF_SIZE);
  armDMA = true;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  dbgprint("FTL\r\n");
  HAL_NVIC_SystemReset();
}

#ifdef MOD_SLAVE_APBE
int slave_pwrctrl_set_mode(enum slave_pwrctrl_mode mode)
{
    switch (mode) {
    case SLAVE_PWRCTRL_POWER_ON:
        dbgprint("POWER_ON\r\n");
    case SLAVE_PWRCTRL_POWER_FLASH_MODE:
        dbgprint("POWER_FLASH_MODE\r\n");
        apbe_power_on_flash();
        break;
    case SLAVE_PWRCTRL_POWER_OFF:
        dbgprint("POWER_OFF\r\n");
    default:
        apbe_power_off();
        break;
    }
    return 0;
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
