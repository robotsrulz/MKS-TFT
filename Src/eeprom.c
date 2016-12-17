/**
  ******************************************************************************
  * File Name          : eeprom.c
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

#include "eeprom.h"

HAL_StatusTypeDef readEEPROM(uint16_t address, uint8_t* MemTarget,
		uint16_t Size)
{
	taskENTER_CRITICAL();

	HAL_StatusTypeDef res = HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, address,
			I2C_MEMADD_SIZE_8BIT, MemTarget, Size, EEPROM_TIMEOUT);

	taskEXIT_CRITICAL();

	osDelay(EEPROM_WRITE / 2);
	return res;
}

HAL_StatusTypeDef writeEEPROM(uint16_t address, uint8_t* MemTarget,
		uint16_t Size)
{
	uint16_t counter = 0;
	HAL_StatusTypeDef res = HAL_OK;

	while (counter < Size && res == HAL_OK)
	{
		uint16_t writebytes = Size - counter;
		if (writebytes > EEPROM_MAXPKT) writebytes = EEPROM_MAXPKT;

		taskENTER_CRITICAL();

		res = HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS,
				address + counter, I2C_MEMADD_SIZE_8BIT,
				&MemTarget[counter], writebytes, EEPROM_TIMEOUT);

		taskEXIT_CRITICAL();

		if (res != HAL_OK)
			return res;

		counter += writebytes;
		osDelay(EEPROM_WRITE);
	}

	return res;
}

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
