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

#include "mks_conf.h"

#if defined(STM32F107xC) && defined(MKS_TFT)
# include "usb_host.h"
# include "Buzzer.h"

FATFS flashFileSystem;	// 0:/
FATFS usbFileSystem;	// 2:/

UART_HandleTypeDef huart3;
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef htim2;

static void MX_USART3_UART_Init(void);

static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);

static void MX_TIM2_Init(void);

#elif defined(STM32F103xE) && defined(CZMINI)
# include "bsp_driver_sd.h"

SD_HandleTypeDef hsd;
SRAM_HandleTypeDef hsram1;
HAL_SD_CardInfoTypedef SDCardInfo;

SPI_HandleTypeDef hspi2;

static void MX_SDIO_SD_Init(void);
static void MX_FSMC_Init(void);
static void MX_SPI2_Init(void);

#endif

FATFS sdFileSystem;		// 1:/

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

static osThreadId uiTaskHandle;      // UI thread
static osThreadId serviceTaskHandle; // low latency tasks

// static osThreadId comm1TaskHandle;		// printing thread
// static osThreadId comm2TaskHandle;		// wi-fi/bt thread
// static osThreadId sdcardHandlerHandle;	// sd card insert/remove

// QueueHandle_t xUIEventQueue;
// QueueHandle_t xPCommEventQueue;

void SystemClock_Config(void);
void Error_Handler(void);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);

void StartUITask(void const * argument);
void StartServiceTask(void const * argument);

// void StartComm1Task(void const * argument);
// void StartComm2Task(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

SemaphoreHandle_t xSDSemaphore;
SemaphoreHandle_t xUSBSemaphore;
volatile uint8_t usbEvent;

// static SemaphoreHandle_t xComm1Semaphore;
// static SemaphoreHandle_t xComm2Semaphore;

int PanelDueMain(void);

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
    MX_DMA_Init();

#if defined(STM32F107xC) && defined(MKS_TFT)
	MX_TIM2_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_SPI3_Init();

	MX_USART3_UART_Init();
#endif

#if defined(STM32F103xE) && defined(CZMINI)
	MX_SDIO_SD_Init();
	MX_FSMC_Init();
	MX_SPI2_Init();
#endif

	MX_FATFS_Init();
#if defined(STM32F107xC) && defined(MKS_TFT)
	MX_USB_HOST_Init();
#endif
	xSDSemaphore    = xSemaphoreCreateBinary();
	xUSBSemaphore   = xSemaphoreCreateBinary();

//	xComm1Semaphore = xSemaphoreCreateBinary();
//	xComm2Semaphore = xSemaphoreCreateBinary();

//	osThreadDef(comm1Task, StartComm1Task, osPriorityNormal, 0,	512);
//	comm1TaskHandle = osThreadCreate(osThread(comm1Task), NULL);

//	osThreadDef(comm2Task, StartComm2Task, osPriorityNormal, 0,	128);
//	comm2TaskHandle = osThreadCreate(osThread(comm2Task), NULL);

	osThreadDef(serviceHandlerTask, StartServiceTask, osPriorityNormal, 0, 128);
	serviceTaskHandle = osThreadCreate(osThread(serviceHandlerTask), NULL);

	osThreadDef(uiTask, StartUITask, osPriorityNormal, 0, 12 * 1024 / 4);
	uiTaskHandle = osThreadCreate(osThread(uiTask), NULL);

//	xUIEventQueue = xQueueCreate(UI_QUEUE_SIZE, sizeof(xUIEvent_t));
//	if (xUIEventQueue == NULL) {
//		/* Queue was not created and must not be used. */
//	}
//
//	xPCommEventQueue = xQueueCreate(COMM_QUEUE_SIZE, sizeof(xCommEvent_t));
//	if (xPCommEventQueue == NULL) {
//		/* Queue was not created and must not be used. */
//	}

	osKernelStart();
	while (1) {}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
#if defined(STM32F107xC) && defined(MKS_TFT)
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
#endif
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
#if defined(STM32F107xC) && defined(MKS_TFT)
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
    RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
    RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
    RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
    RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
#endif
#if defined(STM32F103xE) && defined(CZMINI)
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
#endif
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

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
        Error_Handler();
}

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

static void MX_SPI3_Init(void)
{
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
        Error_Handler();
}

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

static void MX_SPI2_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
        Error_Handler();
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{
    FSMC_NORSRAM_TimingTypeDef Timing;

    /** Perform the SRAM1 memory initialization sequence
    */
    hsram1.Instance = FSMC_NORSRAM_DEVICE;
    hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
    /* hsram1.Init */
    hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
    hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
    hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
    hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
    hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
    hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
    hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
    hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
    hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
    hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
    /* Timing */
    Timing.AddressSetupTime = 1;
    Timing.AddressHoldTime = 1;
    Timing.DataSetupTime = 2;
    Timing.BusTurnAroundDuration = 0;
    Timing.CLKDivision = 16;
    Timing.DataLatency = 17;
    Timing.AccessMode = FSMC_ACCESS_MODE_A;
    /* ExtTiming */

    if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
        Error_Handler();

    /** Disconnect NADV
    */

    __HAL_AFIO_FSMCNADV_DISCONNECTED();
}

#endif

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

#if defined(STM32F107xC) && defined(MKS_TFT)
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
						  |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
						  |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
						  |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
						  |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
						  |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
						  |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#elif defined(STM32F103xE) && defined(CZMINI)
    HAL_GPIO_WritePin(GPIOE, LCD_RESET_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = LCD_RESET_Pin;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#endif
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : TOUCH_DI_Pin */
	GPIO_InitStruct.Pin = TOUCH_DI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TOUCH_DI_GPIO_Port, &GPIO_InitStruct);

#if defined(STM32F107xC) && defined(MKS_TFT)
	HAL_GPIO_WritePin(GPIOB, LCD_nWR_Pin|FLASH_nCS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, SDCARD_nCS_Pin|LCD_RS_Pin|LCD_BACKLIGHT_Pin|LCD_nRD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, LCD_nCS_Pin|TOUCH_nCS_Pin, GPIO_PIN_RESET);

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

#elif defined(STM32F103xE) && defined(CZMINI)

    /*Configure GPIO pin : TOUCH_nCS_Pin */
    GPIO_InitStruct.Pin = TOUCH_nCS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TOUCH_nCS_GPIO_Port, &GPIO_InitStruct);
#endif
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

#if defined(STM32F107xC) && defined(MKS_TFT)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	switch (GPIO_Pin) {

	case SDCARD_DETECT_Pin:
		xSemaphoreGiveFromISR(xSDSemaphore, &xHigherPriorityTaskWoken);
		break;

	default:
		break;
	}
}
#endif

void StartServiceTask(void const * argument) {

#if defined(STM32F107xC) && defined(MKS_TFT)
    f_mount(&flashFileSystem, SPIFL_Path, 1);  // mount flash

    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(SDCARD_DETECT_GPIO_Port, SDCARD_DETECT_Pin))
        f_mount(&sdFileSystem, SPISD_Path, 1);
#elif defined(STM32F103xE) && defined(CZMINI)
    f_mount(&sdFileSystem, SD_Path, 1);
#endif

	while (1) {
#if defined(STM32F107xC) && defined(MKS_TFT)
   		BuzzerCheckStop();
        BYTE power;

		if (xSemaphoreTake(xSDSemaphore, 1) == pdTRUE) {
            switch (HAL_GPIO_ReadPin(SDCARD_DETECT_GPIO_Port, SDCARD_DETECT_Pin)) {
            case GPIO_PIN_RESET:
                f_mount(&sdFileSystem, SPISD_Path, 1);
                break;

            default:
                f_mount(NULL, SPISD_Path, 1);

                power = 0;
                (*SPISD_Driver.disk_ioctl)(0, CTRL_POWER, &power);
                break;
            }
		}
#endif

#if defined(STM32F107xC) && defined(MKS_TFT)
		if (xSemaphoreTake(xUSBSemaphore, 1) == pdTRUE) {
			switch (usbEvent) {
            case HOST_USER_DISCONNECTION:
            case HOST_USER_CLASS_ACTIVE:
                f_mount((HOST_USER_DISCONNECTION == usbEvent) ? NULL : &usbFileSystem, USBH_Path, 1);
                break;

            default:
                break;
			}
		}
#endif

#if defined(STM32F103xE) && defined(CZMINI)
        osDelay(1);
#endif
	}
}

/*

void StartComm1Task(void const * argument) {

    __HAL_UART_FLUSH_DRREGISTER(&huart2);
    HAL_UART_Receive_DMA(&huart2, &comm1RxBuffer, 1);

	while (1) {

   		if (xPCommEventQueue != 0) {

//			xCommEvent_t event;
//			BaseType_t hasEvent = xQueueReceive(xPCommEventQueue, &event, (TickType_t ) 0);

            osDelay(1);
 	    }
	}
}

void StartComm2Task(void const * argument) {

	while (1) {
		osDelay(1);
	}
}

*/

/* USER CODE END 4 */

/* StartUITask function */
void StartUITask(void const * argument) {

//	xUIEvent_t event;
//	event.ucEventID = INIT_EVENT;
// 	(*processEvent) (&event);


	/* Infinite loop */

	PanelDueMain();

	for (;;) {
//		if (xUIEventQueue != 0) {

//			xUIEvent_t event;
//			if (xQueueReceive(xUIEventQueue, &event, (TickType_t ) 500)) {

//				(*processEvent) (&event);
//			} else {
				/*
				 * No events received
				 * */
//			}
		}
//	}
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
