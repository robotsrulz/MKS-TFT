
#include "stm32f1xx_hal.h"
#include "boot_conf.h"

extern unsigned int _isr_real;
const uint32_t *mcuFirstPageAddr = (const uint32_t *) (0x8000000 + MAIN_PR_OFFSET);

// uint32_t mcuLastPageAddr = ((uint32_t) &_isr_real) - FLASH_PAGE_SIZE;

uint8_t buffer[FLASH_PAGE_SIZE];
uint32_t bufferLen = 0;

static FIL fil;
static FILINFO info;

// uint32_t getMaxFlashImageSize()
// {
// 	return (uint32_t) &_isr_real - mcuFirstPageAddr;
// }

HAL_StatusTypeDef erase(uint32_t from, uint32_t to)
{
	HAL_StatusTypeDef res = HAL_OK;

	for (uint32_t i = from; i <= to; i += FLASH_PAGE_SIZE)
	{
		FLASH_EraseInitTypeDef erase;

		erase.TypeErase = FLASH_TYPEERASE_PAGES;
		erase.Banks = FLASH_BANK_1;
		erase.PageAddress = i;
		erase.NbPages = 1;

		uint32_t error = 0;
		res = HAL_FLASHEx_Erase(&erase, &error);

		if (res != HAL_OK) {
			return res;
		}
	}
	return res;
}

FRESULT readNextPage(uint8_t *target, uint32_t *read)
{
	uint16_t blocksCount = 16;
	uint16_t fileBlock = FLASH_PAGE_SIZE / blocksCount;

	*read = 0;
	UINT readNow = 0;
	FRESULT res = FR_OK;

	for (uint16_t i = 0; i<blocksCount; i++)
	{
		res = f_read(&fil, target, fileBlock, &readNow);

		target += readNow;
		*read += readNow;

		if (res != FR_OK || readNow != fileBlock)
			break;
	}
	return res;
}

HAL_StatusTypeDef flashWrite(uint32_t position, uint8_t *data, uint32_t size)
{
	HAL_StatusTypeDef res = HAL_OK;
	for (uint32_t i=0; i<size; i+=2)
	{
		res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, position + i, *(uint16_t*)&data[i]);
		if (res != HAL_OK)
			break;
	}
	return res;
}

FlashResult flash(const char *fname)
{
	FRESULT res = f_stat(fname, &info);
	if (res != FR_OK)
		return FLASH_FILE_NOT_EXISTS;

	// Checking file size
//	if (info.fsize > getMaxFlashImageSize())
//		return FLASH_FILE_TOO_BIG;

	res = f_open(&fil, fname, FA_OPEN_EXISTING | FA_READ);

	if (res != FR_OK)
		return FLASH_RESULT_FILE_ERROR;

	uint32_t position = (uint32_t) mcuFirstPageAddr;

	if (HAL_OK != HAL_FLASH_Unlock())
		return FLASH_RESULT_FLASH_ERROR;

	if (HAL_OK != erase((uint32_t) mcuFirstPageAddr, info.fsize + (uint32_t)mcuFirstPageAddr))
		return FLASH_RESULT_FLASH_ERROR;

	do {
		readNextPage(buffer, &bufferLen);
		if (HAL_OK != flashWrite(position, buffer, bufferLen))
			return FLASH_RESULT_FLASH_ERROR;

		position += bufferLen;
	} while (bufferLen != 0);

	f_close(&fil);
	HAL_FLASH_Lock();

	return FR_OK;
}

