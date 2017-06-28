/**
  ******************************************************************************
  * File Name          : boot_conf.h
  * Description        : This file contains firmware configuration
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 Roman Stepanov
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOOT_CONF_H
#define __BOOT_CONF_H

#include "stm32f1xx_hal.h"
#include "fatfs.h"

#if defined(STM32F107xC) && defined(MKS_TFT)
/**
 * Makerbase MKS-TFT32
 */
 #define SPEAKER_Pin             GPIO_PIN_2
 #define SPEAKER_GPIO_Port       GPIOA
 #define SDCARD_nCS_Pin          GPIO_PIN_11
 #define SDCARD_nCS_GPIO_Port    GPIOD
 #define FLASH_nCS_Pin           GPIO_PIN_9
 #define FLASH_nCS_GPIO_Port     GPIOB

extern FATFS sdFileSystem;		// 0:/

#elif defined(STM32F103xE) && defined(CZMINI)

#endif

typedef enum
{
	FLASH_RESULT_OK = 0,
	FLASH_RESULT_FILE_ERROR,
	FLASH_RESULT_FLASH_ERROR,
	FLASH_FILE_NOT_EXISTS,
	FLASH_FILE_CANNOT_OPEN,
	FLASH_FILE_INVALID_HASH,
	FLASH_FILE_TOO_BIG
} FlashResult;

FlashResult flash(const char *fname);
extern const uint32_t *mcuFirstPageAddr;

typedef void (*Callable)();

#define MAIN_PR_OFFSET 0x8000

#endif /* __BOOT_CONF_H */
/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/
