/**
  ******************************************************************************
  * @file    spiflash_w25q16dv.h
  * @author  Roman Stepanov
  * @version V1.3.0
  * @date    09-Dec-2016
  * @brief   SPI Flash I/O driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef __SPIFLASH_W25Q16DV_H
#define __SPIFLASH_W25Q16DV_H

#include "ff_gen_drv.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#define WFLASH_PAGE_SIZE	0x100	// 256b
#define WFLASH_SECTOR_SIZE	0x1000	// 4k
#define WFLASH_BLOCK_SIZE	0x10000	// 64k

#define WFLASH_FAST_READ	0		// 0 - use cmd 0x03 to read, 1 - 0x0b
#define WFLASH_STATIC_BUF	0		// 0 - use pvPortMalloc, 1 - static buffer

/*
 * generic interface
 *
 * */

HAL_StatusTypeDef readFLASH(uint32_t address, uint8_t* buffer, size_t size);
HAL_StatusTypeDef writeEnableFLASH();
HAL_StatusTypeDef eraseFLASHSector(uint32_t address);
HAL_StatusTypeDef programFLASHPage(uint32_t address, const uint8_t* buffer, uint16_t size);
HAL_StatusTypeDef powerOnFLASH();
HAL_StatusTypeDef powerOffFLASH();

/*
 * FatFs interface
 *
 * */

extern Diskio_drvTypeDef SPIFLASH_Driver;

#endif /* __SPIFLASH_W25Q16DV_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
