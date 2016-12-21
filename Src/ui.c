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
#include "tga.h"
#include "fatfs.h"
#include "eeprom.h"

static FATFS flashFileSystem;	// 0:/
static FATFS sdFileSystem;		// 1:/
static FATFS usbFileSystem;		// 2:/


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
static void uiDrawIcon(const TCHAR *path, uint16_t x, uint16_t y);
static void uiDrawBinIcon(const TCHAR *path, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t resetWindow);

static const char *iconsTga[97] = {
		"about.bin", "Add.bin", "adj.bin",
		"baud115200.bin", "baud115200_sel.bin", "baud250000.bin",
		"baud250000_sel.bin", "baud57600.bin", "baud57600_sel.bin",
		"baud9600.bin", "baud9600_sel.bin", "bed.bin", "bed_no_words.bin",
		"bmp_auto_off.bin", "bmp_manual_off.bin", "Close.bin",
		"connect.bin", "custom1.bin", "custom2.bin", "custom3.bin",
		"custom4.bin", "custom5.bin", "custom6.bin", "custom7.bin",
		"Dec.bin", "delta.bin", "delta_sel.bin", "dir.bin", "en.bin",
		"en_sel.bin", "extru1_no_word.bin", "extru2_no_word.bin",
		"extruct.bin", "extruct_sel.bin", "Extrusor1.bin", "Extrusor2.bin",
		"Fan.bin", "Fan_move.bin", "Fan_no_words.bin", "file.bin",
		"fileSys.bin", "Home.bin", "In.bin", "lang.bin", "machine.bin",
		"More.bin", "MotorOff.bin", "mov.bin", "mov_sel.bin", "norm.bin",
		"norm_sel.bin", "Option.bin", "Out.bin", "pageDown.bin",
		"pageUp.bin", "pause.bin", "PreHeat.bin", "Print.bin",
		"resume.bin", "Return.bin", "sd.bin", "sd_sel.bin", "Set.bin",
		"simple.bin", "simple_sel.bin", "speed.bin", "speed0.bin",
		"speed127.bin", "speed255.bin", "Speed_high.bin",
		"Speed_normal.bin", "Speed_slow.bin", "Splash.bin",
		"Step10_degree.bin", "Step10_mm.bin", "Step1_degree.bin",
		"Step1_mm.bin", "Step5_degree.bin", "Step5_mm.bin",
		"Step_move0_1.bin", "Step_move1.bin", "Step_move10.bin", "stop.bin",
		"temp.bin", "usb.bin", "usb_sel.bin", "wifi.bin", "xAdd.bin",
		"xDec.bin", "yAdd.bin", "yDec.bin", "zAdd.bin", "zDec.bin",
		"zeroA.bin", "zeroX.bin", "zeroY.bin", "zeroZ.bin"
};

/*
 * user callback definition
 */

void uiInitialize (xEvent_t *pxEvent) {

	switch (pxEvent->ucEventID) {
	case INIT_EVENT:

		// mount internal flash, format if needed
		if (FR_NO_FILESYSTEM == f_mount(&flashFileSystem, SPIFL_Path, 1)) {

			BYTE *work = pvPortMalloc(_MAX_SS);
			if (work) {

				f_mkfs ("0:", FM_ANY, 0, work, _MAX_SS);	/* Create a FAT volume */
				vPortFree(work);
			}
		}

		f_mount(&sdFileSystem, SPISD_Path, 1);	// mount sd card, mount usb later

#if 0
		if ( flashFileSystem.fs_type) {

			f_mkdir("0:/images");

			for (int i=0; i<97; i++) {

				char src[40];
				char dst[40];

				snprintf(src, sizeof(src), "1:/Tga_Images/%s", iconsTga[i]);
				snprintf(dst, sizeof(dst), PATH "%s", iconsTga[i]);
				transferFile(src, dst, 0);
			}
		}
#endif

		Lcd_Init(LCD_LANDSCAPE_CL);
		Lcd_Fill_Screen(Lcd_Get_RGB565(0, 0, 0)); // gray

#define PATH	"1:/mks_pic/bmp_"

//		uiDrawBinIcon(PATH "logo.bin", 0, 0, 320, 240, 1);

		uiDrawBinIcon(PATH "preHeat.bin", 0, 16, 78, 104, 1);
		uiDrawBinIcon(PATH "mov.bin", 78, 16, 78, 104, 0);
		uiDrawBinIcon(PATH "zero.bin", 78 * 2, 16, 78, 104, 0);
		uiDrawBinIcon(PATH "printing.bin", 78 * 3, 16, 78, 104, 0);

		uiDrawBinIcon(PATH "extruct.bin", 0, 16 + 104, 78, 104, 0);
		uiDrawBinIcon(PATH "fan.bin", 78, 16 + 104, 78, 104, 0);
		uiDrawBinIcon(PATH "set.bin", 78 * 2, 16 + 104, 78, 104, 0);
		uiDrawBinIcon(PATH "More.bin", 78 * 3, 16 + 104, 78, 104, 1);

		Lcd_Put_Text(0, 0, 16, "NotReadyPrint", 0xffffu);
		break;

	case TOUCH_DOWN_EVENT:
		processEvent = uiFileBrowse;
		pxEvent->ucEventID = INIT_EVENT;
		xQueueSendToBack(xEventQueue, pxEvent, 1000);
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
		sprintf(cwd, "0:/");
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

static void uiDrawProgressBar(uint32_t scale, uint16_t color) {

}

static void uiUpdateProgressBar(uint32_t progress) {

}

static void uiDrawIcon(const TCHAR *path, uint16_t x, uint16_t y) {

	FIL *pIconFile;

	if ((pIconFile = pvPortMalloc(sizeof(FIL))) != NULL) {
		if (f_open(pIconFile, path, FA_READ) == FR_OK) {

			unsigned short height = 20, width;
			if (!read_tga_direct(pIconFile, x, y, &width, &height)) {
				// success
			}

			f_close(pIconFile);
		}
		vPortFree(pIconFile);
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

			FRESULT res = FR_OK;
			size_t bytes = (size_t) -1;

			Lcd_Go_XY(x, y);
			Lcd_Com (0x0022);

			HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LCD_nRD_GPIO_Port, LCD_nRD_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LCD_RS_GPIO_Port,  LCD_RS_Pin,  GPIO_PIN_SET);

			do {
				res = f_read(pIconFile, pBuffer, _MIN_SS, &bytes);
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


/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/
