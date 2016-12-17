/**
  ******************************************************************************
  * @file    spiflash_w25q16dv.c
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

#include "ff_gen_drv.h"
#include "spiflash_w25q16dv.h"
#include "fatfs.h"

#include <string.h>

#define _IO_SIZE	WFLASH_SECTOR_SIZE
#define _IO_TIMEOUT	2000

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* Private function prototypes -----------------------------------------------*/
DSTATUS SPIFLASH_initialize (BYTE);
DSTATUS SPIFLASH_status (BYTE);
DRESULT SPIFLASH_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
  DRESULT SPIFLASH_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT SPIFLASH_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

Diskio_drvTypeDef SPIFLASH_Driver =
{
		SPIFLASH_initialize,
		SPIFLASH_status,
		SPIFLASH_read,
#if  _USE_WRITE == 1
		SPIFLASH_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
		SPIFLASH_ioctl,
#endif /* _USE_IOCTL == 1 */
};

static BYTE PowerFlag = 0; /* indicates if "power" is on */

#define CMD_PAGE_PROGRAM	0x02
#define CMD_READ			0x03
#define CMD_WRITE_DISABLE	0x04
#define CMD_READ_STATUS_R1	0x05
#define CMD_WRITE_ENABLE	0x06
#define CMD_FASTREAD		0x0b
#define CMD_SECTOR_ERASE	0x20
#define CMD_READ_STATUS_R2	0x35
#define CMD_POWER_DOWN		0xb9
#define CMD_POWER_UP		0xab
#define CMD_JEDEC_ID		0x9f

#if WFLASH_STATIC_BUF == 1
 static uint8_t	wbuf[_IO_SIZE + 5];
 static uint8_t	rbuf[_IO_SIZE + 5];
#endif /* WFLASH_STATIC_BUF == 1 */

static uint8_t readFLASHId1() {

	HAL_StatusTypeDef res = HAL_OK;
	uint8_t	id = 0xff;

#if WFLASH_STATIC_BUF == 0
	uint8_t	wbuf[5];
	uint8_t	rbuf[5];
#endif /* WFLASH_STATIC_BUF == 0 */

	wbuf[0] = CMD_POWER_UP;

	taskENTER_CRITICAL();
	deviceSelect(SPI_FLASH);

	res = HAL_SPI_TransmitReceive(&hspi1, wbuf, rbuf, 5, _IO_TIMEOUT);

	deviceDeselect();
	taskEXIT_CRITICAL();

	if (res == HAL_OK)
		id = rbuf[4];

	return id;
}

static uint16_t readFLASH_JEDEC_Id() {

	HAL_StatusTypeDef res = HAL_OK;
	uint16_t	id = 0xffffu;

#if WFLASH_STATIC_BUF == 0
	uint8_t	wbuf[4];
	uint8_t	rbuf[4];
#endif /* WFLASH_STATIC_BUF == 0 */

	wbuf[0] = CMD_JEDEC_ID;

	taskENTER_CRITICAL();
	deviceSelect(SPI_FLASH);

	res = HAL_SPI_TransmitReceive(&hspi1, wbuf, rbuf, 4, _IO_TIMEOUT);

	deviceDeselect();
	taskEXIT_CRITICAL();

	if (res == HAL_OK)
		id = ((uint16_t)rbuf[2] << 8) + rbuf[3];

	return id;
}


HAL_StatusTypeDef readFLASH(uint32_t address, uint8_t* buffer, size_t size) {

	HAL_StatusTypeDef res = HAL_OK;
	size_t counter = 0;
	size_t datastart;

#if WFLASH_STATIC_BUF == 0
	size_t alloc = (size > _IO_SIZE) ? _IO_SIZE + 5 : size + 5;
	uint8_t	*wbuf = pvPortMalloc(alloc);
	if (!wbuf) return HAL_ERROR;
	uint8_t	*rbuf = pvPortMalloc(alloc);

	if (!rbuf) {
		vPortFree(wbuf);
		return HAL_ERROR;
	}
#endif /* WFLASH_STATIC_BUF == 0 */

#if WFLASH_FAST_READ == 1
	wbuf[0] = CMD_FASTREAD;
	datastart = 5;
#else
	wbuf[0] = CMD_READ;
	datastart = 4;
#endif /* WFLASH_FAST_READ == 1 */

	wbuf[1] = (address >> 16) & 0xffu;
	wbuf[2] = (address >> 8) & 0xffu;
	wbuf[3] = address & 0xffu;

	taskENTER_CRITICAL();
	deviceSelect(SPI_FLASH);

	while(counter < size && res == HAL_OK) {

		size_t bytes = size - counter;
		if (bytes > _IO_SIZE) bytes = _IO_SIZE;

		res = HAL_SPI_TransmitReceive(&hspi1, wbuf, rbuf, datastart + bytes, _IO_TIMEOUT);
		memcpy(buffer + counter, rbuf + datastart, bytes);
		counter += bytes;
	}

	deviceDeselect();
	taskEXIT_CRITICAL();

#if WFLASH_STATIC_BUF == 0
	vPortFree(rbuf);
	vPortFree(wbuf);
#endif /* WFLASH_STATIC_BUF == 0 */

	return res;
}

static HAL_StatusTypeDef sendFLASHcmd(uint8_t cmd) {

	HAL_StatusTypeDef res = HAL_OK;

	taskENTER_CRITICAL();
	deviceSelect(SPI_FLASH);

	res = HAL_SPI_Transmit(&hspi1, &cmd, 1, _IO_TIMEOUT);

	deviceDeselect();
	taskEXIT_CRITICAL();

	return res;
}

HAL_StatusTypeDef writeEnableFLASH() {

	return sendFLASHcmd(CMD_WRITE_ENABLE);
}

HAL_StatusTypeDef powerOnFLASH() {

	HAL_StatusTypeDef res = sendFLASHcmd(CMD_POWER_UP);
	PowerFlag = 1;
	return res;
}

HAL_StatusTypeDef powerOffFLASH() {

	HAL_StatusTypeDef res = sendFLASHcmd(CMD_POWER_DOWN);
	PowerFlag = 0;
	return res;
}

static
int checkFLASHPower(void) /* Flash power state: 0=off, 1=on */
{
	return PowerFlag;
}

HAL_StatusTypeDef eraseFLASHSector(uint32_t address) {

	HAL_StatusTypeDef res = HAL_OK;

#if WFLASH_STATIC_BUF == 0
	uint8_t	wbuf[4];
	uint8_t	rbuf[4];
#endif /* WFLASH_STATIC_BUF == 0 */

	wbuf[0] = CMD_SECTOR_ERASE;
	wbuf[1] = (address >> 16) & 0xffu;
	wbuf[2] = (address >> 8) & 0xffu;
	wbuf[3] = address & 0xffu;

	taskENTER_CRITICAL();
	deviceSelect(SPI_FLASH);

	res = HAL_SPI_Transmit(&hspi1, wbuf, 4, _IO_TIMEOUT);

	deviceDeselect();
	deviceSelect(SPI_FLASH);

	wbuf[0] = CMD_READ_STATUS_R1;
	do {
		res = HAL_SPI_TransmitReceive(&hspi1, wbuf, rbuf, 2, _IO_TIMEOUT);
	} while ((res == HAL_OK) && (rbuf[1] & 1));

	deviceDeselect();
	taskEXIT_CRITICAL();

	return res;
}

HAL_StatusTypeDef programFLASHPage(uint32_t address, const uint8_t* buffer, uint16_t size) {

	HAL_StatusTypeDef res = HAL_OK;

#if WFLASH_STATIC_BUF == 0
	size_t alloc = (size > WFLASH_PAGE_SIZE) ? WFLASH_PAGE_SIZE + 4 : size + 4;
	uint8_t	*wbuf = pvPortMalloc(alloc);
	if (!wbuf) return HAL_ERROR;
	uint8_t	*rbuf = pvPortMalloc(alloc);

	if (!rbuf) {
		vPortFree(wbuf);
		return HAL_ERROR;
	}
#endif /* WFLASH_STATIC_BUF == 0 */

	taskENTER_CRITICAL();
	deviceSelect(SPI_FLASH);

	wbuf[0] = CMD_PAGE_PROGRAM;
	wbuf[1] = (address >> 16) & 0xffu;
	wbuf[2] = (address >> 8) & 0xffu;
	wbuf[3] = address & 0xffu;

	if (size > WFLASH_PAGE_SIZE) size = WFLASH_PAGE_SIZE;
	memcpy(&wbuf[4], buffer, size);

	res = HAL_SPI_Transmit(&hspi1, wbuf, 4 + size, _IO_TIMEOUT);

	deviceDeselect();
	deviceSelect(SPI_FLASH);

	wbuf[0] = CMD_READ_STATUS_R1;
	do {
		res = HAL_SPI_TransmitReceive(&hspi1, wbuf, rbuf, 2, _IO_TIMEOUT);
	} while ((res == HAL_OK) && (rbuf[1] & 1));

	deviceDeselect();
	taskEXIT_CRITICAL();

#if WFLASH_STATIC_BUF == 0
	vPortFree(rbuf);
	vPortFree(wbuf);
#endif /* WFLASH_STATIC_BUF == 0 */

	return res;
}

HAL_StatusTypeDef waitFLASHReady() {

	HAL_StatusTypeDef res = HAL_OK;

#if WFLASH_STATIC_BUF == 0
	uint8_t	wbuf[2];
	uint8_t	rbuf[2];
#endif /* WFLASH_STATIC_BUF == 0 */

	taskENTER_CRITICAL();
	deviceSelect(SPI_FLASH);

	wbuf[0] = CMD_READ_STATUS_R1;
	do {
		res = HAL_SPI_TransmitReceive(&hspi1, wbuf, rbuf, 2, _IO_TIMEOUT);
	} while ((res == HAL_OK) && (rbuf[1] & 1));

	deviceDeselect();
	taskEXIT_CRITICAL();

	return res;
}

DSTATUS SPIFLASH_initialize (BYTE drv /* Physical drive nmuber (0) */) {

	if (drv)
		return STA_NOINIT; /* Supports only single drive */

	if (readFLASHId1() == 0x14) {/* Initialization succeded */

		uint16_t id2 = readFLASH_JEDEC_Id();
		if (id2 == 0x4015) {

			powerOnFLASH();
			Stat &= ~STA_NOINIT; /* Clear STA_NOINIT */
		}

	} else
		/* Initialization failed */
		powerOffFLASH();

	return Stat;
}

DSTATUS SPIFLASH_status (BYTE lun) {

	if (lun)
		return STA_NOINIT; /* Supports only single drive */

	return Stat;
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */

DRESULT SPIFLASH_read (BYTE lun, BYTE *buff, DWORD sector, UINT count) {

	if (lun || !count)
		return RES_PARERR;
	if (Stat & STA_NOINIT)
		return RES_NOTRDY;

	HAL_StatusTypeDef res = readFLASH(sector * WFLASH_SECTOR_SIZE, buff, count * WFLASH_SECTOR_SIZE);
	return (res != HAL_OK) ? RES_ERROR : RES_OK;
}

DRESULT SPIFLASH_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count) {

	if (pdrv || !count)
		return RES_PARERR;
	if (Stat & STA_NOINIT)
		return RES_NOTRDY;
	if (Stat & STA_PROTECT)
		return RES_WRPRT;

	HAL_StatusTypeDef res = HAL_OK;

	while (res == HAL_OK && count) {

		if ((res = writeEnableFLASH()) == HAL_OK)
			res = eraseFLASHSector(sector * WFLASH_SECTOR_SIZE);

		for (size_t p = 0;
				(res == HAL_OK) && p < (WFLASH_SECTOR_SIZE / WFLASH_PAGE_SIZE);
				p++) {

			if ((res = writeEnableFLASH()) == HAL_OK) {

				res = programFLASHPage(
						sector * WFLASH_SECTOR_SIZE + p * WFLASH_PAGE_SIZE,
						buff, WFLASH_PAGE_SIZE);

				buff += WFLASH_PAGE_SIZE;
			}
		}

		sector++;
		count--;
	}

	return (!count && (res == HAL_OK)) ? RES_OK : RES_ERROR;
}

DRESULT SPIFLASH_ioctl (BYTE drv, BYTE ctrl, void *buff) {

	DRESULT res;
	BYTE *ptr = buff;

	if (drv)
		return RES_PARERR;

	res = RES_ERROR;

	if (ctrl == CTRL_POWER) {
		switch (*ptr) {
		case 0: /* Sub control code == 0 (POWER_OFF) */
			if (checkFLASHPower())
				powerOffFLASH(); /* Power off */
			res = RES_OK;
			break;
		case 1: /* Sub control code == 1 (POWER_ON) */
			powerOnFLASH(); /* Power on */
			res = RES_OK;
			break;
		case 2: /* Sub control code == 2 (POWER_GET) */
			*(ptr + 1) = (BYTE) checkFLASHPower();
			res = RES_OK;
			break;
		default:
			res = RES_PARERR;
		}
	} else {
		if (Stat & STA_NOINIT)
			return RES_NOTRDY;

		switch (ctrl) {
		case GET_SECTOR_COUNT: /* Get number of sectors on the disk (DWORD) */
			*(DWORD*) buff = 512;
			res = RES_OK;
			break;

		case GET_SECTOR_SIZE: /* Get sectors on the disk (WORD) */
			*(WORD*) buff = WFLASH_SECTOR_SIZE;
			res = RES_OK;
			break;

		case CTRL_SYNC: /* Make sure that data has been written */
			if (!waitFLASHReady())
				res = RES_OK;
			break;

		default:
			res = RES_PARERR;
			break;
		}
	}
	return res;
}

/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/

