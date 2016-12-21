/**
  ******************************************************************************
  * File Name          : screenstates.c
  * Description        : This file contains screen state machine definitions
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

#include <string.h>

#include "ui.h"
#include "lcd.h"
#include "fatfs.h"
#include "eeprom.h"

static FATFS flashFileSystem;	// 0:/
static FATFS sdFileSystem;		// 1:/
static FATFS usbFileSystem;		// 2:/

extern TIM_HandleTypeDef htim2;

#define MKS_PIC_SD	"1:/mks_pic"
#define MKS_PIC_FL	"0:/mks_pic"

/*
 * profiling stuff here
 */

UBaseType_t uxHighWaterMark;

/*
 * user callback declaration
 */

void uiInitialize (xEvent_t *pxEvent);
void uiFileBrowse (xEvent_t *pxEvent);

void (*volatile processEvent) (xEvent_t *pxEvent) = uiInitialize;

/*
 * service routines declaration
 */

static void uiMediaStateChange(uint16_t event);
static void uiRedrawFileList(int raw_x, int raw_y);
static void uiDrawProgressBar(uint32_t scale, uint16_t color);
static void uiUpdateProgressBar(uint32_t progress);
static void uiDrawBinIcon(const TCHAR *path, uint16_t x, uint16_t y,
		uint16_t width, uint16_t height, uint8_t resetWindow);
static void uiShortBeep();

/*
 * user callback definition
 */

void uiInitialize (xEvent_t *pxEvent) {

	DIR dir;
	uint16_t x, y;

	switch (pxEvent->ucEventID) {
	case INIT_EVENT:

		Lcd_Init(LCD_LANDSCAPE_CL);
		Lcd_Fill_Screen(Lcd_Get_RGB565(0, 0, 0));

		// mount internal flash, format if needed
		if (FR_NO_FILESYSTEM == f_mount(&flashFileSystem, SPIFL_Path, 1)) {

			BYTE *work = pvPortMalloc(_MAX_SS);
			if (work) {

				f_mkfs ("0:", FM_ANY, 0, work, _MAX_SS);	/* Create a FAT volume */
				vPortFree(work);

				f_mount(&flashFileSystem, SPIFL_Path, 1);
			}
		}

		f_mount(&sdFileSystem, SPISD_Path, 1);	// mount sd card, mount usb later

		if (flashFileSystem.fs_type && sdFileSystem.fs_type
				&& FR_OK == f_opendir(&dir, MKS_PIC_SD)) {

			FRESULT res = FR_OK; /* Open the directory */

			res = f_mkdir(MKS_PIC_FL);
			if (res == FR_OK || res == FR_EXIST) {

				FILINFO fno;
				size_t count = 0;
				while (FR_OK == f_readdir(&dir, &fno) && fno.fname[0]) {
					if (((fno.fattrib & AM_DIR) == 0) && strstr(fno.fname, ".bin"))
						count++;
				}

				f_rewinddir(&dir);
				if (count) {

					Lcd_Put_Text(56, 80, 16, "Copying files to FLASH...", Lcd_Get_RGB565(0, 63, 0));
					uiDrawProgressBar(count, Lcd_Get_RGB565(0, 63, 0));

					// TODO: format flash again?

					count = 0;
					res = FR_OK;

					while (FR_OK == f_readdir(&dir, &fno) && fno.fname[0]) {
						if (((fno.fattrib & AM_DIR) == 0) && strstr(fno.fname, ".bin")) {

							char src[50];
							char dst[50];

							Lcd_Fill_Rect(0, 232, 319, 240, 0);
							snprintf(src, sizeof(src), "%02u", res);
							Lcd_Put_Text(304, 232, 8, src,
									res == FR_OK ? Lcd_Get_RGB565(0, 63, 0) : Lcd_Get_RGB565(31, 0, 0));

							snprintf(src, sizeof(src), MKS_PIC_SD "/%s", fno.fname);
							snprintf(dst, sizeof(dst), MKS_PIC_FL "/%s", fno.fname);

							Lcd_Put_Text(0, 232, 8, src, Lcd_Get_RGB565(0, 63, 0));
							res = transferFile(src, dst, 1);

							uiUpdateProgressBar(++count);
						}
					}

					Lcd_Fill_Screen(Lcd_Get_RGB565(0, 0, 0));
				}
			}

			f_closedir(&dir);
			f_rename(MKS_PIC_SD, MKS_PIC_SD ".old");
		}

		uiDrawBinIcon(MKS_PIC_FL "/bmp_preHeat.bin",	1, 16, 78, 104, 0);
		uiDrawBinIcon(MKS_PIC_FL "/bmp_mov.bin",	   81, 16, 78, 104, 0);
		uiDrawBinIcon(MKS_PIC_FL "/bmp_zero.bin",	  161, 16, 78, 104, 0);
		uiDrawBinIcon(MKS_PIC_FL "/bmp_printing.bin", 241, 16, 78, 104, 0);

		uiDrawBinIcon(MKS_PIC_FL "/bmp_extruct.bin",	1, 18 + 104, 78, 104, 0);
		uiDrawBinIcon(MKS_PIC_FL "/bmp_fan.bin",	   81, 18 + 104, 78, 104, 0);
		uiDrawBinIcon(MKS_PIC_FL "/bmp_set.bin",	  161, 18 + 104, 78, 104, 0);
		uiDrawBinIcon(MKS_PIC_FL "/bmp_More.bin",	  241, 18 + 104, 78, 104, 1);

		Lcd_Put_Text(0, 0, 16, "NotReadyPrint", 0xffffu);
		break;

	case TOUCH_DOWN_EVENT:

		uiShortBeep();
		Lcd_Translate_Touch_Pos((pxEvent->ucData.touchXY) >> 16 & 0x7fffu,
				pxEvent->ucData.touchXY & 0x7fffu, &x, &y);

		if (x > 240 && y < (16 + 104) && y > 16) {
			processEvent = uiFileBrowse;
			pxEvent->ucEventID = INIT_EVENT;
			xQueueSendToBack(xEventQueue, pxEvent, 1000);
		}
		break;

	case SDCARD_INSERT:
	case SDCARD_REMOVE:
	case USBDRIVE_INSERT:
	case USBDRIVE_REMOVE:
		uiMediaStateChange(pxEvent->ucEventID);
		break;

	default:
		break;
	}
}

static TCHAR fname_table[FLIST_SIZE][NAMELEN];
static int row_selected = -1;

static TCHAR cwd[_MAX_LFN + 1];

void uiFileBrowse(xEvent_t *pxEvent) {

	switch (pxEvent->ucEventID) {
	case TOUCH_DOWN_EVENT:
		uiShortBeep();

		uiRedrawFileList((pxEvent->ucData.touchXY) >> 16 & 0x7fffu,
				pxEvent->ucData.touchXY & 0x7fffu);
		break;

	case SDCARD_INSERT:
	case SDCARD_REMOVE:
	case USBDRIVE_INSERT:
	case USBDRIVE_REMOVE:
		uiMediaStateChange(pxEvent->ucEventID);
		uiRedrawFileList(-1, -1);
		break;

	case INIT_EVENT:
		sprintf(cwd, "1:/");
		uiRedrawFileList(-1, -1);
		break;

		// TODO: process other events
	default:
		break;
	}
}

static FRESULT uiGetFileList(DIR *dir, uint8_t rootFolder) {

	FILINFO *pFno;
	FRESULT res = FR_OK;
	size_t i=0;

	for (; i<FLIST_SIZE; i++)
		fname_table[i][0] = '\0';

	if (!rootFolder)
		sprintf(fname_table[0], ">..");

	if ((pFno = pvPortMalloc(sizeof(FILINFO))) != NULL) {
		for (i = (rootFolder ? 0 : 1); i < FLIST_SIZE; i++) {

			if (FR_OK == f_readdir(dir, pFno) && pFno->fname[0]) {

				fname_table[i][0] = (pFno->fattrib & AM_DIR) ? '>' : ' ';
				size_t k = 0;

				for (; k < (NAMELEN - 1) && pFno->fname[k]; k++)
					fname_table[i][k + 1] = pFno->fname[k];

				fname_table[i][k + 1] = '\0';
			} else {
				res = FR_NO_FILE;	// ???
				break;
			}
		}

		vPortFree(pFno);
	}

	return res;
}

static void uiRedrawFileList(int raw_x, int raw_y) {

	uint16_t x, y;

	int row = -1;
	int fs_type = 0;

	// check that fs is mounted
	switch (cwd[0]) {
	case '1':	// SPI SD card
		fs_type = sdFileSystem.fs_type;
		if (fs_type) f_chdrive (SPISD_Path);
		break;

	case '2':	// USB stick
		fs_type = usbFileSystem.fs_type;
		if (fs_type) f_chdrive (USBH_Path);
		break;

	case '0':	// SPI Flash
	default:	// default path?
		fs_type = flashFileSystem.fs_type;
		if (fs_type) f_chdrive (SPIFL_Path);
		break;
	}

	// TODO: get correct FATFS object!
	if (!fs_type) {

		Lcd_Fill_Screen(0x001fu);
		return;	// fs not mounted
	}

	if (raw_x >= 0 && raw_y >= 0) {

		Lcd_Translate_Touch_Pos(raw_x, raw_y, &x, &y);

		row = y / FL_FONT_SIZE;
		if (row == row_selected && row_selected >= 0) {
			return;
		}

		if (fname_table[row][0] == '>' && fname_table[row][1]) {

			if (FR_OK == f_chdir(&fname_table[row][1])) {

				f_getcwd(cwd, sizeof(cwd));
				row = -1;
			} else {
				row = 99;
			}
		}
	}

	// TODO: move below directory read
	Lcd_Fill_Screen(0x001fu);
	Lcd_Put_Text(0, LCD_MAX_Y - 9, 8, cwd, 0xffffu);

	char buffer[12];
//	sprintf(buffer, "%05u:%05u", x, y);
//	Lcd_Put_Text(10, LCD_MAX_Y - 9, 8, buffer, 0xffffu);

	if (row == -1							// screen not touched, new folder?
			|| row_selected == -1			// first run
			|| !(fname_table[0][0])) {		// file list empty

		DIR dir;

		FRESULT res = f_opendir(&dir, cwd); /* Open the directory */
		if (res == FR_OK) {
			int is_root = !strcmp(cwd + 1, ":/");
			res = uiGetFileList(&dir, is_root);
			// if (res != FR_OK) {
				sprintf(buffer, "=%04u", res);
				Lcd_Put_Text(200, LCD_MAX_Y - 9, 8, buffer, 0xffffu);
			// }
			f_closedir(&dir);
		} else {
			sprintf(buffer, "*%04u", res);
			Lcd_Put_Text(200, LCD_MAX_Y - 9, 8, buffer, 0xffffu);
		}
	}
	else {
		sprintf(buffer, "#%04d", row);
		Lcd_Put_Text(200, LCD_MAX_Y - 9, 8, buffer, 0xffffu);
	}

	row_selected = row;
	int file;
	for (file=0; file<FLIST_SIZE; file++) {

		if (fname_table[file][0] == '\0')
			break; /* Break on end of table */

		Lcd_Put_Text(0, file * FL_FONT_SIZE, FL_FONT_SIZE, fname_table[file],
				(file == row_selected) ? Lcd_Get_RGB565(31, 63, 0) : 0xffffu);
	}

	sprintf(buffer, "F%04u", file);
	Lcd_Put_Text(240, LCD_MAX_Y - 9, 8, buffer, 0xffffu);
}

/*
 * service routines definition
 */

static void uiMediaStateChange(uint16_t event) {

	switch (event) {
	case SDCARD_INSERT:
		f_mount(&sdFileSystem, SPISD_Path, 1);
		break;

	case SDCARD_REMOVE:
		f_mount(NULL, SPISD_Path, 1);
		{
			BYTE poweroff = 0;
			(*SPISD_Driver.disk_ioctl)(0, CTRL_POWER, &poweroff);
		}
		break;

	case USBDRIVE_INSERT:
		f_mount(&usbFileSystem, USBH_Path, 1);
		break;

	case USBDRIVE_REMOVE:
		f_mount(NULL, USBH_Path, 1);
		break;
	}
}

static uint32_t pBarScale = 0;
static uint16_t pBarColor = 0xffffu;
static uint32_t pBarProgress = 0;

static void uiDrawProgressBar(uint32_t scale, uint16_t color) {

	Lcd_Fill_Rect(10 - 2, 100 - 2, 310 + 2, 140 + 2, color);
	Lcd_Fill_Rect(10 - 1, 100 - 1, 310 + 1, 140 + 1, 0);

	pBarScale = scale;
	pBarColor = color;
	pBarProgress = 0;
}

static void uiUpdateProgressBar(uint32_t progress) {

	uint16_t x0 = 300 * pBarProgress / pBarScale;
	pBarProgress = progress;
	uint16_t x1 = 300 * pBarProgress / pBarScale;

	if (x0 < x1) {
		Lcd_Fill_Rect(10 + x0, 100, 10 + x1, 140, pBarColor);
	}
}

static void uiDrawBinIcon(const TCHAR *path, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t resetWindow) {

	FIL *pIconFile = NULL;
	BYTE *pBuffer = NULL;

	Lcd_Com_Data ((Lcd_Orientation() & 1) ? 0x0052 : 0x0050, x);
	Lcd_Com_Data ((Lcd_Orientation() & 1) ? 0x0050 : 0x0052, y);
	Lcd_Com_Data ((Lcd_Orientation() & 1) ? 0x0053 : 0x0051, x + width - 1);
	Lcd_Com_Data ((Lcd_Orientation() & 1) ? 0x0051 : 0x0053, y + height - 1);

	if ((pIconFile = pvPortMalloc(sizeof(FIL))) != NULL
			&& (pBuffer = pvPortMalloc(_MIN_SS)) != NULL) {

		if (f_open(pIconFile, path, FA_READ) == FR_OK) {

			size_t bytes = (size_t) -1;

			Lcd_Go_XY(x, y);
			Lcd_Com (0x0022);

			HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LCD_nRD_GPIO_Port, LCD_nRD_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LCD_RS_GPIO_Port,  LCD_RS_Pin,  GPIO_PIN_SET);

			do {
				f_read(pIconFile, pBuffer, _MIN_SS, &bytes);
				if (bytes) {

					for (size_t i=0; i<bytes; i+=2) {

						GPIOE->ODR = *(uint16_t *)&pBuffer[i];
					    GPIOB->BSRR = (uint32_t)LCD_nWR_Pin << 16;
						GPIOB->BSRR = LCD_nWR_Pin;
					}
				}
			} while (bytes);

			f_close(pIconFile);
		}
	}

	if (pIconFile) vPortFree(pIconFile);
	if (pBuffer) vPortFree(pBuffer);

	if (resetWindow) {
		Lcd_Com_Data(0x0050, 0x0000);		  // Window Horizontal RAM Address Start (R50h)
		Lcd_Com_Data(0x0051, 239);			  // Window Horizontal RAM Address End (R51h)
		Lcd_Com_Data(0x0052, 0x0000);		  // Window Vertical RAM Address Start (R52h)
		Lcd_Com_Data(0x0053, 319);			  // Window Vertical RAM Address End (R53h)
	}
}

static void uiShortBeep() {

	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	osDelay(100);
	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
}

/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/
