/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include <stdbool.h>
#include <string.h>
#include <stm32l4xx_hal.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
typedef void (*Function_Pointer)(void);

/* Private define ------------------------------------------------------------*/
#define BOOT_PARTITION_INDEX	0
#define FLASH_LOADER_INDEX	2
#define JUMP_ADDRESS_OFFSET	4

#define FORCE_FLASH_GPIO_PIN    GPIO_PIN_9
#define FORCE_FLASH_GPIO_PORT   GPIOB

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

enum BootState {
    BOOT_STATE_NORMAL,        /* Boot main program */
    BOOT_STATE_REQUEST_FLASH, /* Boot flashing program */
    BOOT_STATE_FLASHING,      /* Flashing in progress  */
};

static const struct memory_map mmap[4] = {
  {"nuttx", ((uint32_t)0x08008000), ((uint32_t)0x0807f800)},
  {0, 0, 0},
  {"nuttx-flash", ((uint32_t)0x080E0000), ((uint32_t)0x080ff800)},
  {0, 0, 0},
};

uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
uint32_t Address = 0, PAGEError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Boot2Partition(int pIndex);
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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

void ErasePage(uint32_t pageAddress)
{
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

void Boot2Partition(int pIndex)
{
  Function_Pointer  pJumpToFunction;
  uint32_t jumpAddress;
  uint32_t imageAddress;

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
  bootModeFlag = (char *)(FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE);
  if (!memcmp(bootModeFlag, bootmode_flag, sizeof(bootmode_flag)))
  {
    bootState = BOOT_STATE_REQUEST_FLASH;
  }

  if (!memcmp(bootModeFlag, flashing_flag, sizeof(flashing_flag)))
  {
    bootState = BOOT_STATE_FLASHING;
  }

  if (HAL_GPIO_ReadPin(FORCE_FLASH_GPIO_PORT, FORCE_FLASH_GPIO_PIN)
                        == GPIO_PIN_SET)
  {
    bootState = BOOT_STATE_REQUEST_FLASH;
  }

  return bootState;
}

int main(void)
{
  enum BootState bootState = CheckFlashMode();

  switch(bootState) {
  case BOOT_STATE_REQUEST_FLASH:
     /* Erase the Flash Mode Barker */
     ErasePage((uint32_t)(FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE));
     /* fall through */
  case BOOT_STATE_FLASHING:
     Boot2Partition(FLASH_LOADER_INDEX);
  break;
  case BOOT_STATE_NORMAL:
  default:
     Boot2Partition(BOOT_PARTITION_INDEX);
  break;
  }

  /* fallback to booting to flash loader */
  Boot2Partition(FLASH_LOADER_INDEX);

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  __PWR_CLK_ENABLE();

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

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
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB9 - FORCE FLASH MODE */
  GPIO_InitStruct.Pin = FORCE_FLASH_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FORCE_FLASH_GPIO_PORT, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
