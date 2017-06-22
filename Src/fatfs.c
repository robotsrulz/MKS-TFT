/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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

#include "fatfs.h"
#include "mks_conf.h"

#if defined(STM32F107xC) && defined(MKS_TFT)
char SPISD_Path[4];     /* USER logical drive path */
char SPIFL_Path[4];     /* SPI Flash logical drive path */
char USBH_Path[4];      /* USB stick logical drive path */
#elif defined(STM32F103xE) && defined(CZMINI)
char SD_Path[4];        /* SD logical drive path */
#endif

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

FRESULT transferFile(const TCHAR *source, const TCHAR *dest, uint8_t overwrite) {

	FIL		*pSourceFile = NULL;	// >4k
	FIL		*pDestFile = NULL;		// >4k
	BYTE	*pBuffer = NULL;		// _MAX_SS = 4k

	FRESULT res = FR_OK;

	if (NULL != (pSourceFile = (FIL *)pvPortMalloc(sizeof(FIL))) &&
			NULL != (pDestFile = (FIL *)pvPortMalloc(sizeof(FIL))) &&
			NULL != (pBuffer = (BYTE *)pvPortMalloc(_MIN_SS))) {

		if (FR_OK == (res = f_open(pSourceFile, source, FA_READ))) {
			if (FR_OK == (res = f_open(pDestFile, dest,	FA_WRITE
									| (overwrite ? FA_CREATE_ALWAYS : FA_CREATE_NEW)))) {

				size_t bytes = (size_t) -1, wbytes;
				do {
					if (FR_OK
							== (res = f_read(pSourceFile, pBuffer, _MIN_SS,
									&bytes)) && bytes) {
						res = f_write(pDestFile, pBuffer, bytes, &wbytes);
					}
				} while (FR_OK == res && bytes && wbytes);

				f_close(pDestFile);
			}

			f_close(pSourceFile);
		}

	} else {
		res = FR_NOT_ENOUGH_CORE;
	}

	if (pBuffer) vPortFree(pBuffer);
	if (pDestFile) vPortFree(pDestFile);
	if (pSourceFile) vPortFree(pSourceFile);

	return res;
}

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
#if defined(STM32F107xC) && defined(MKS_TFT)
    /*## FatFS: Link the USER driver ###########################*/
    FATFS_LinkDriver(&SPIFLASH_Driver, SPIFL_Path);	// 0:/
    FATFS_LinkDriver(&SPISD_Driver, SPISD_Path);    // 1:/
    FATFS_LinkDriver(&USBH_Driver, USBH_Path);		// 2:/
#elif defined(STM32F103xE) && defined(CZMINI)
    FATFS_LinkDriver(&SD_Driver, SD_Path);
#endif
}

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

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
