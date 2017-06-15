/**
  ******************************************************************************
  * File Name          : eeprom.h
  * Description        : This file contains useful stuff for i2c EEPROM
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 Roman Stepanov
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
#ifndef __EEPROM_H
#define __EEPROM_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#define EEPROM_ADDRESS		0xA0	// A0 = A1 = A2 = 0
#define EEPROM_MAXPKT		1		// page size (faulty eeprom? page size 8 doesn't work)
#define EEPROM_WRITE		5		// time to wait in ms
#define EEPROM_TIMEOUT		(5 * EEPROM_WRITE)  // timeout while writing
#define EEPROM_SIZE			256

extern I2C_HandleTypeDef hi2c1;

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef readEEPROM(uint16_t address, uint8_t* MemTarget,
		uint16_t Size);
HAL_StatusTypeDef writeEEPROM(uint16_t address, uint8_t* MemTarget,
		uint16_t Size);

#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/**
  * @}
*/

#endif /* __EEPROM_H */
/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/
