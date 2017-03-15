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
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "ui.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static osThreadId uiTaskHandlerHandle;	// UI thread
static osThreadId comm1TaskHandle;		// printing thread
static osThreadId comm2TaskHandle;		// wi-fi/bt thread
static osThreadId touchHandlerHandle;	// touch screen finger up/down
static osThreadId sdcardHandlerHandle;	// sd card insert/remove

QueueHandle_t xUIEventQueue;
QueueHandle_t xPCommEventQueue;

#define MAXCOMM1SIZE    0xff                // Biggest string the user will type
static uint8_t comm1RxBuffer = '\000';      // where we store that one character that just came in
static uint8_t comm1RxString[MAXCOMM1SIZE]; // where we build our string from characters coming in
static int comm1RxIndex = 0;                // index for going though comm1RxString

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void StartUITask(void const * argument);
void StartComm1Task(void const * argument);
void StartComm2Task(void const * argument);
void StartTouchHandlerTask(void const * argument);
void StartSDHandlerTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static SemaphoreHandle_t xTouchSemaphore;
static SemaphoreHandle_t xSDSemaphore;
static SemaphoreHandle_t xComm1Semaphore;
static SemaphoreHandle_t xComm2Semaphore;
/* USER CODE END 0 */

int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
    MX_DMA_Init();
	MX_TIM2_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_SPI3_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	xTouchSemaphore = xSemaphoreCreateBinary();
	xSDSemaphore    = xSemaphoreCreateBinary();
	xComm1Semaphore = xSemaphoreCreateBinary();
	xComm2Semaphore = xSemaphoreCreateBinary();
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of uiTask */

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	osThreadDef(touchHandlerTask, StartTouchHandlerTask, osPriorityNormal, 0, 128);
	touchHandlerHandle = osThreadCreate(osThread(touchHandlerTask), NULL);

	osThreadDef(sdcardHandlerTask, StartSDHandlerTask, osPriorityNormal, 0,	128);
	sdcardHandlerHandle = osThreadCreate(osThread(sdcardHandlerTask), NULL);

	osThreadDef(comm1Task, StartComm1Task, osPriorityNormal, 0,	512);
	comm1TaskHandle = osThreadCreate(osThread(comm1Task), NULL);

	osThreadDef(comm2Task, StartComm2Task, osPriorityNormal, 0,	128);
	comm2TaskHandle = osThreadCreate(osThread(comm2Task), NULL);

	osThreadDef(uiTask, StartUITask, osPriorityNormal, 0, 14 * 1024 / 4);
	uiTaskHandlerHandle = osThreadCreate(osThread(uiTask), NULL);
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	xUIEventQueue = xQueueCreate(1, sizeof(xUIEvent_t));
	if (xUIEventQueue == NULL) {
		/* Queue was not created and must not be used. */
	}

	xPCommEventQueue = xQueueCreate(10, sizeof(xUIEvent_t));
	if (xPCommEventQueue == NULL) {
		/* Queue was not created and must not be used. */
	}
	/* USER CODE END RTOS_QUEUES */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = I2C_FAST_MODE_MAX_CLK;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
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
  {
    Error_Handler();
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
						  |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
						  |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
						  |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LCD_nWR_Pin|FLASH_nCS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, SDCARD_nCS_Pin|LCD_RS_Pin|LCD_BACKLIGHT_Pin|LCD_nRD_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LCD_nCS_Pin|TOUCH_nCS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PE2 PE3 PE4 PE5
						   PE6 PE7 PE8 PE9
						   PE10 PE11 PE12 PE13
						   PE14 PE15 PE0 PE1 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
						  |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
						  |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
						  |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : TOUCH_DI_Pin */
	GPIO_InitStruct.Pin = TOUCH_DI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; // FALLING - touch, RISING - no touch
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : FILAMENT_DI_Pin POWER_DI_Pin */
	GPIO_InitStruct.Pin = FILAMENT_DI_Pin|POWER_DI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_nWR_Pin FLASH_nCS_Pin */
	GPIO_InitStruct.Pin = LCD_nWR_Pin|FLASH_nCS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : SDCARD_nCS_Pin LCD_RS_Pin LCD_BACKLIGHT_Pin LCD_nRD_Pin */
	GPIO_InitStruct.Pin = SDCARD_nCS_Pin|LCD_RS_Pin|LCD_BACKLIGHT_Pin|LCD_nRD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_nCS_Pin */
	GPIO_InitStruct.Pin = LCD_nCS_Pin|TOUCH_nCS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : WIFI_DI_Pin */
	GPIO_InitStruct.Pin = WIFI_DI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(WIFI_DI_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SDCARD_DETECT_Pin */
	GPIO_InitStruct.Pin = SDCARD_DETECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init */
	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);	// sdcard detect irq
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);	// touch irq
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName) {

	// dive into hard fault handler
	* (BYTE *) 0 = 0;
}

void vApplicationMallocFailedHook(void) {

	// dive into hard fault handler
	* (BYTE *) 0 = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	switch (GPIO_Pin) {
	case TOUCH_DI_Pin:
		xSemaphoreGiveFromISR(xTouchSemaphore, &xHigherPriorityTaskWoken);
		break;
	case SDCARD_DETECT_Pin:
		xSemaphoreGiveFromISR(xSDSemaphore, &xHigherPriorityTaskWoken);
		break;
	default:
		break;
	}
}

static uint16_t xTouchX = 0;
static uint16_t xTouchY = 0;

void StartTouchHandlerTask(void const * argument) {

	uint8_t pTxData[3] = { 0xd4, 0, 0 };
	uint8_t pRxData[3];

	/* warmup */
	HAL_SPI_TransmitReceive(&hspi3, pTxData, pRxData, 3, 1000);
	pTxData[0] = 0x94;
	HAL_SPI_TransmitReceive(&hspi3, pTxData, pRxData, 3, 1000);

	while (1) {

		if(xSemaphoreTake(xTouchSemaphore, portMAX_DELAY ) == pdTRUE ) {

			osDelay(1);
			if (HAL_GPIO_ReadPin(TOUCH_DI_GPIO_Port, TOUCH_DI_Pin) == GPIO_PIN_RESET) {

				/*
				 * datasheet: https://ldm-systems.ru/f/doc/catalog/HY-TFT-2,8/XPT2046.pdf
				 * for calculation method see the following:
				 *	http://www.ti.com/lit/an/sbaa155a/sbaa155a.pdf
				 *	http://e2e.ti.com/support/other_analog/touch/f/750/t/202636
				 * */

				uint16_t x[3], y[3], i;

				HAL_GPIO_WritePin(TOUCH_nCS_GPIO_Port, TOUCH_nCS_Pin, GPIO_PIN_RESET);

				if (0) {
					for (i = 0; i < 3; i++) {
						pTxData[0] = /* i < 2 ? 0xd7 : */0xd4;
						HAL_SPI_TransmitReceive(&hspi3, pTxData, pRxData, 3,
								1000);
						y[i] = (unsigned int) (pRxData[1] << 8) + pRxData[2];
					}

					for (i = 0; i < 3; i++) {
						pTxData[0] = /* i < 2 ? 0x97 : */0x94;
						HAL_SPI_TransmitReceive(&hspi3, pTxData, pRxData, 3,
								1000);
						x[i] = (unsigned int) (pRxData[1] << 8) + pRxData[2];
					}
				} else {
					for (i = 0; i < 3; i++) {
						pTxData[0] = /* i < 2 ? 0xd7 : */0xd4;
						HAL_SPI_TransmitReceive(&hspi3, pTxData, pRxData, 3,
								1000);
						y[i] = (unsigned int) (pRxData[1] << 8) + pRxData[2];

						pTxData[0] = /* i < 2 ? 0x97 : */0x94;
						HAL_SPI_TransmitReceive(&hspi3, pTxData, pRxData, 3,
								1000);
						x[i] = (unsigned int) (pRxData[1] << 8) + pRxData[2];
					}
				}

				HAL_GPIO_WritePin(TOUCH_nCS_GPIO_Port, TOUCH_nCS_Pin, GPIO_PIN_SET);

				xUIEvent_t event;
				event.ucEventID = TOUCH_DOWN_EVENT;

				xTouchX = Lcd_Touch_Get_Closest_Average(x);
				xTouchY = Lcd_Touch_Get_Closest_Average(y);
				event.ucData.touchXY = ((unsigned int) xTouchX << 16) + xTouchY;
				xQueueSendToBack(xUIEventQueue, &event, 1000);

				// TODO: continuous gesture recognition here!
				osDelay(125); // limit touch event rate

			} else {

                if (xTouchX && xTouchY) {
                    xUIEvent_t event;
                    event.ucEventID = TOUCH_UP_EVENT;
                    event.ucData.touchXY = ((unsigned int) xTouchX << 16) + xTouchY;
                    xQueueSendToBack(xUIEventQueue, &event, 1000);

                    xTouchX = 0;
                    xTouchY = 0;

                    // TODO: continuous gesture recognition here!
                    osDelay(125);
                }
			}
		}
	}
}

void StartSDHandlerTask(void const * argument) {

	while (1) {

		if (xSemaphoreTake(xSDSemaphore, portMAX_DELAY ) == pdTRUE) {

			osDelay(100);

			xUIEvent_t event;
			event.ucEventID =
					(HAL_GPIO_ReadPin(SDCARD_DETECT_GPIO_Port, SDCARD_DETECT_Pin)
							== GPIO_PIN_RESET) ? SDCARD_INSERT : SDCARD_REMOVE;
			xQueueSendToBack(xUIEventQueue, &event, 1000);
		}
	}
}

void StartComm1Task(void const * argument) {

    __HAL_UART_FLUSH_DRREGISTER(&huart2);
    HAL_UART_Receive_DMA(&huart2, &comm1RxBuffer, 1);

	while (1) {
	    if(xSemaphoreTake(xComm1Semaphore, 5000 /* FIXME */ ) == pdTRUE ) {

            xUIEvent_t event;
            event.ucEventID = SHOW_STATUS;
            xQueueSendToBack(xUIEventQueue, &event, 1000);
 	    }
        else
        {
            // osDelay(1);
            static const char m115[] = "M119\n";

            HAL_UART_Transmit(&huart2, m115, /* sizeof(m115)*/ 5, 1000);
            osDelay(20);
        }
	}
}

void StartComm2Task(void const * argument) {

	while (1) {
		osDelay(1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
        __HAL_UART_FLUSH_DRREGISTER(&huart2); // Clear the buffer to prevent overrun

        int i = 0;

        if (comm1RxBuffer == '\n' || comm1RxBuffer == '\r') // If Enter
        {
            if (comm1RxString[0] != 'o' || comm1RxString[1] != 'k') {
                snprintf(statString, MAXSTATSIZE, "%s", comm1RxString);
            }

            comm1RxString[comm1RxIndex] = 0;
            comm1RxIndex = 0;
            for (i = 0; i < MAXSTATSIZE; i++) comm1RxString[i] = 0; // Clear the string buffer

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(xComm1Semaphore, &xHigherPriorityTaskWoken);
        }
        else
        {
            comm1RxString[comm1RxIndex] = comm1RxBuffer; // Add that character to the string
            comm1RxIndex++;
            if (comm1RxIndex > MAXSTATSIZE) // User typing too much, we can't have commands that big
            {
                comm1RxIndex = 0;
                for (i = 0; i < MAXSTATSIZE; i++) comm1RxString[i] = 0; // Clear the string buffer
            }
        }
	    //
	}
}
/* USER CODE END 4 */

/* StartUITask function */
void StartUITask(void const * argument) {
	/* init code for FATFS */
	MX_FATFS_Init();

	/* init code for USB_HOST */
	MX_USB_HOST_Init();

	/* USER CODE BEGIN 5 */

	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	osDelay(50);
	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);

	xUIEvent_t event;
	event.ucEventID = INIT_EVENT;
	(*processEvent) (&event);


	/* Infinite loop */
	for (;;) {
		if (xUIEventQueue != 0) {

			xUIEvent_t event;

			if (xQueueReceive(xUIEventQueue, &event, (TickType_t ) 500)) {

				(*processEvent) (&event);
			} else {
				/*
				 * No events received
				 * */
			}
		}
	}
	/* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */

	// dive into hard fault handler
	*(BYTE *) 0 = 0;

	/* USER CODE END Error_Handler */
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

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
