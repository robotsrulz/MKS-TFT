/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "boot_conf.h"

#include "stm32f1xx_hal.h"
#include "fatfs.h"

#if defined(STM32F107xC) && defined(MKS_TFT)

SPI_HandleTypeDef hspi1;	// SD-card
TIM_HandleTypeDef htim2;	// Buzzer
char SPISD_Path[4];     /* USER logical drive path */

static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void ShortBeep();

#elif defined(STM32F103xE) && defined(CZMINI)
# include "bsp_driver_sd.h"

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;
char SD_Path[4];        /* SD logical drive path */

static void MX_SDIO_SD_Init(void);
static inline void ShortBeep() {}
#endif

FATFS sdFileSystem;		// 0:/

void SystemClock_Config(void);
void Error_Handler(void);

static void MX_GPIO_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void osDelay(__IO uint32_t Delay)
{
    HAL_Delay(Delay);
}

inline void moveVectorTable(uint32_t Offset)
{
    // __disable_irq();
    SCB->VTOR = FLASH_BASE | Offset;
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();

#if defined(STM32F107xC) && defined(MKS_TFT)
	MX_TIM2_Init();
	MX_SPI1_Init();
#endif

#if defined(STM32F103xE) && defined(CZMINI)
	MX_SDIO_SD_Init();
#endif

	MX_FATFS_Init();
	ShortBeep();

	if (FR_OK == f_mount(&sdFileSystem, SPISD_Path, 1) && FR_OK == flash("0:/firmware.bin"))
    {
        f_mount(NULL, SPISD_Path, 1);
        ShortBeep();

        HAL_SPI_MspDeInit(&hspi1);
        HAL_TIM_Base_MspDeInit(&htim2);

        __HAL_RCC_GPIOA_CLK_DISABLE();
        __HAL_RCC_GPIOB_CLK_DISABLE();
        __HAL_RCC_GPIOC_CLK_DISABLE();
        __HAL_RCC_GPIOD_CLK_DISABLE();
        __HAL_RCC_GPIOE_CLK_DISABLE();

        HAL_DeInit();

        // Disabling SysTick interrupt
        SysTick->CTRL = 0;
        moveVectorTable(MAIN_PR_OFFSET);
        // Setting initial value to stack pointer
        __set_MSP(*mcuFirstPageAddr);
        // booting really

        Callable resetHandler = (Callable) (*(mcuFirstPageAddr + 1) );
        resetHandler();
	}

	while (1) {
		ShortBeep();
		HAL_Delay(5000);
	}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
#if defined(STM32F107xC) && defined(MKS_TFT)
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
#endif

    SET_BIT(RCC->CR, RCC_CR_HSEON);
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET) {}

#if defined(STM32F107xC) && defined(MKS_TFT)
    __HAL_RCC_PLL2_DISABLE();
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY) != RESET) {}

    __HAL_RCC_HSE_PREDIV2_CONFIG(RCC_HSE_PREDIV2_DIV5);
    __HAL_RCC_PLL2_CONFIG(RCC_PLL2_MUL8);
    __HAL_RCC_PLL2_ENABLE();
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY) == RESET) {}
#endif

    __HAL_RCC_PLL_DISABLE();
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  != RESET) {}

#if defined(STM32F107xC) && defined(MKS_TFT)
    SET_BIT(RCC->CFGR2, RCC_PREDIV1_SOURCE_PLL2);
    __HAL_RCC_HSE_PREDIV_CONFIG(RCC_HSE_PREDIV_DIV5);
#endif
#if defined(STM32F103xE) && defined(CZMINI)
    __HAL_RCC_HSE_PREDIV_CONFIG(RCC_HSE_PREDIV_DIV1);
#endif

    __HAL_RCC_PLL_CONFIG(RCC_PLLSOURCE_HSE, RCC_PLL_MUL9);
    __HAL_RCC_PLL_ENABLE();
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)  == RESET) {}

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();

#if defined(STM32F107xC) && defined(MKS_TFT)
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        Error_Handler();
#endif

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
#if defined(STM32F107xC) && defined(MKS_TFT)
    __HAL_RCC_PLLI2S_ENABLE();
#endif
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

#if defined(STM32F107xC) && defined(MKS_TFT)
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; // was: 32
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        Error_Handler();
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 14399;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC3REF;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	sConfigOC.Pulse = 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim2);
}

#elif defined(STM32F103xE) && defined(CZMINI)
static void MX_SDIO_SD_Init(void)
{
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = 2;
}

#endif

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

#if defined(STM32F107xC) && defined(MKS_TFT)
	HAL_GPIO_WritePin(SDCARD_nCS_GPIO_Port, SDCARD_nCS_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = SDCARD_nCS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SDCARD_nCS_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = FLASH_nCS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(FLASH_nCS_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_SET);
#elif defined(STM32F103xE) && defined(CZMINI)

#endif
}

#if defined(STM32F107xC) && defined(MKS_TFT)
void ShortBeep()
{
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_Delay(20);
	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
}
#endif

void MX_FATFS_Init(void)
{
#if defined(STM32F107xC) && defined(MKS_TFT)
    /*## FatFS: Link the USER driver ###########################*/
    FATFS_LinkDriver(&SPISD_Driver, SPISD_Path);    // 0:/
#elif defined(STM32F103xE) && defined(CZMINI)
    FATFS_LinkDriver(&SD_Driver, SD_Path);
#endif
}

#if defined(STM32F107xC) && defined(MKS_TFT)
void deviceSelect(dselect_t device)  {

	if (device == SPI_SDCARD) {
		HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SDCARD_nCS_GPIO_Port, SDCARD_nCS_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(SDCARD_nCS_GPIO_Port, SDCARD_nCS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_RESET);
	}
}

void deviceDeselect() {
	HAL_GPIO_WritePin(SDCARD_nCS_GPIO_Port, SDCARD_nCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FLASH_nCS_GPIO_Port, FLASH_nCS_Pin, GPIO_PIN_SET);
}
#endif

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
    /* USER CODE BEGIN get_fattime */
    return 0;
    /* USER CODE END get_fattime */
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
	// dive into hard fault handler
	*(BYTE *) 0 = 0;
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

}

#endif

void HAL_MspInit(void)
{
	__HAL_RCC_AFIO_CLK_ENABLE();
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);

	/** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
	 */
	__HAL_AFIO_REMAP_SWJ_NOJTAG();
}

#if defined(STM32F103xE) && defined(CZMINI)
void HAL_SD_MspInit(SD_HandleTypeDef* hsd)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(hsd->Instance==SDIO)
	{
		__HAL_RCC_SDIO_CLK_ENABLE();

		/** SDIO GPIO Configuration
			PC8     ------> SDIO_D0
			PC9     ------> SDIO_D1
			PC10    ------> SDIO_D2
			PC11    ------> SDIO_D3
			PC12    ------> SDIO_CK
			PD2     ------> SDIO_CMD
			*/
		GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
							  |GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(SDIO_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(SDIO_IRQn);
	}
}

void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd)
{
	if(hsd->Instance==SDIO)
	{
		__HAL_RCC_SDIO_CLK_DISABLE();

		/**SDIO GPIO Configuration
			PC8     ------> SDIO_D0
			PC9     ------> SDIO_D1
			PC10    ------> SDIO_D2
			PC11    ------> SDIO_D3
			PC12    ------> SDIO_CK
			PD2     ------> SDIO_CMD
			*/
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
							  |GPIO_PIN_12);

		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
		HAL_NVIC_DisableIRQ(SDIO_IRQn);
	}
}
#endif

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
	GPIO_InitTypeDef GPIO_InitStruct;
#if defined(STM32F107xC) && defined(MKS_TFT)
	if(hspi->Instance==SPI1)
	{
		__HAL_RCC_SPI1_CLK_ENABLE();

		/**SPI1 GPIO Configuration
			PA5     ------> SPI1_SCK
			PA6     ------> SPI1_MISO
			PA7     ------> SPI1_MOSI
			*/
		GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
#elif defined(STM32F103xE) && defined(CZMINI)
	if(hspi->Instance==SPI2)
	{
		__HAL_RCC_SPI2_CLK_ENABLE();

		/**SPI2 GPIO Configuration
			PB13     ------> SPI2_SCK
			PB14     ------> SPI2_MISO
			PB15     ------> SPI2_MOSI
			*/
		GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_14;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
#endif
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
#if defined(STM32F107xC) && defined(MKS_TFT)
	if(hspi->Instance==SPI1)
	{
		__HAL_RCC_SPI1_CLK_DISABLE();

		/**SPI1 GPIO Configuration
			PA5     ------> SPI1_SCK
			PA6     ------> SPI1_MISO
			PA7     ------> SPI1_MOSI
			*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	}
#elif defined(STM32F103xE) && defined(CZMINI)
	if(hspi->Instance==SPI2)
	{
		__HAL_RCC_SPI2_CLK_DISABLE();

		/**SPI2 GPIO Configuration
			PB13     ------> SPI2_SCK
			PB14     ------> SPI2_MISO
			PB15     ------> SPI2_MOSI
			*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	}
#endif
}

#if defined(STM32F107xC) && defined(MKS_TFT)
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
	if(htim_base->Instance==TIM2)
	{
		__HAL_RCC_TIM2_CLK_ENABLE();

		HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
	}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
	if(htim_base->Instance==TIM2)
	{
		__HAL_RCC_TIM2_CLK_DISABLE();

		HAL_NVIC_DisableIRQ(TIM2_IRQn);
	}
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(htim->Instance==TIM2)
	{
		/**TIM2 GPIO Configuration
			PA2     ------> TIM2_CH3
			*/
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
