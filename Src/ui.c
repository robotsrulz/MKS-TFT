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

#define READY_PRINT	"Магнум"

uint8_t statString[MAXSTATSIZE+1];

uint8_t moveStep = MOVE_10;
uint8_t offMode = MANUAL_OFF;
uint8_t connectSpeed = CONNECT_115200;
uint8_t preheatDev = PR_EXTRUDER_1;
uint8_t extrudeDev = EXTRUDER_1;
uint8_t preheatSelDegree = STEP_1_DEGREE;
uint8_t extrudeDistance = DISTANCE_1;
uint8_t extrudeSelSpeed = SPEED_SLOW;
uint8_t selectedFs = FS_SD;

/*
 * user callback declaration
 */

void uiInitialize (xUIEvent_t *pxEvent);
void uiMainMenu   (xUIEvent_t *pxEvent);
void uiSetupMenu  (xUIEvent_t *pxEvent);

typedef void (*volatile eventProcessor_t) (xUIEvent_t *);
eventProcessor_t processEvent = uiInitialize;

/*
 * service routines declaration
 */

static void uiMediaStateChange(uint16_t event);
static void uiDrawProgressBar(uint32_t scale, uint16_t color);
static void uiUpdateProgressBar(uint32_t progress);
static void uiDrawBinIcon(const TCHAR *path, uint16_t x, uint16_t y,
		uint16_t width, uint16_t height, uint8_t resetWindow);

__STATIC_INLINE void uiNextState(void (*volatile next) (xUIEvent_t *pxEvent)) {
	processEvent = next;
	xUIEvent_t event = { INIT_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

__STATIC_INLINE void uiShortBeep() {
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	osDelay(12);
	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
}

typedef struct {
    uint16_t                x1, y1, x2, y2;
    uint16_t                color;
    const char              *label;
	const eventProcessor_t	pEventProcessor;
} xButton_t;

__STATIC_INLINE void uiDrawMenu(const xButton_t *pMenu, size_t menuSize) {

	for (int s=0; s<menuSize; pMenu++, s++) {
        Lcd_Fill_Rect(pMenu->x1, pMenu->y1, pMenu->x2, pMenu->y2, pMenu->color);

        uint16_t    x = (pMenu->x2 - pMenu->x1)/2 + pMenu->x1 - (strlen(pMenu->label) << 2),
                    y = (pMenu->y2 - pMenu->y1)/2 + pMenu->y1 - 8;

        Lcd_Put_Text(x, y, 16, pMenu->label, 0xffffu);
	}
}

static uint16_t touchX, touchY;

__STATIC_INLINE void uiMenuHandleEventDefault(const xButton_t *pMenu, size_t menuSize, xUIEvent_t *pxEvent) {

	if (pxEvent) {
		switch (pxEvent->ucEventID) {
		case SDCARD_INSERT:
		case SDCARD_REMOVE:
		case USBDRIVE_INSERT:
		case USBDRIVE_REMOVE:
			uiMediaStateChange(pxEvent->ucEventID);
			break;

        case SHOW_STATUS:
            Lcd_Fill_Rect(0, LCD_MAX_Y - 9, LCD_MAX_X, LCD_MAX_Y, 0);
            Lcd_Put_Text(10, LCD_MAX_Y - 9, 8, statString, 0xffffu);
            break;

		case INIT_EVENT:
			if (pMenu) {
				Lcd_Fill_Screen(Lcd_Get_RGB565(0, 0, 0));
				uiDrawMenu(pMenu, menuSize);
#if 0
				char buffer[12];
				sprintf(buffer, "%03u:%03u", touchX, touchY);
				Lcd_Put_Text(10, LCD_MAX_Y - 9, 8, buffer, 0xffffu);
#endif
			}
			break;

//		case TOUCH_DOWN_EVENT:
		case TOUCH_UP_EVENT:
			if (pMenu) {
				eventProcessor_t p = NULL;

#if 0
				char buffer[12];
                sprintf(buffer, "%05u:%05u", (pxEvent->ucData.touchXY) >> 16 & 0x7fffu,
						pxEvent->ucData.touchXY & 0x7fffu);
                Lcd_Put_Text(80, LCD_MAX_Y - 9, 8, buffer, 0xffffu);
#endif
				Lcd_Translate_Touch_Pos((pxEvent->ucData.touchXY) >> 16 & 0x7fffu,
						pxEvent->ucData.touchXY & 0x7fffu, &touchX, &touchY);

                for (int s=0; s<menuSize; s++) {

                    if (touchX >= pMenu[s].x1 && touchX <= pMenu[s].x2
                        && touchY >= pMenu[s].y1 && touchY <= pMenu[s].y2)
                    {
                        p = pMenu[s].pEventProcessor;
                        break;
                    }
                }

				if (p) uiNextState(p);
			}

			uiShortBeep();
			break;

		default:
			break;
		}
	}
}

/*
 * user callback definition
 */

void uiInitialize (xUIEvent_t *pxEvent) {

	if (INIT_EVENT == pxEvent->ucEventID) {

		DIR dir;

		Lcd_Init(LCD_LANDSCAPE_CL);
		Lcd_Fill_Screen(Lcd_Get_RGB565(0, 0, 0));

		// mount internal flash, format if needed
		f_mount(&flashFileSystem, SPIFL_Path, 1);  // mount flash
		f_mount(&sdFileSystem, SPISD_Path, 1);      // mount sd card, mount usb later

		if (sdFileSystem.fs_type && FR_OK == f_opendir(&dir, MKS_PIC_SD)) {

			FRESULT res = FR_OK; /* Open the directory */
			BYTE *work = pvPortMalloc(_MAX_SS);
			if (work) {

				f_mkfs ("0:", FM_ANY, 0, work, _MAX_SS);	/* Create a FAT volume */
				vPortFree(work);

				f_mount(&flashFileSystem, SPIFL_Path, 1);
			}

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

		uiNextState(uiMainMenu);
	} else
		uiMenuHandleEventDefault(NULL, 0, pxEvent);
}

void uiMainMenu (xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 0, 0, 320, 110, Lcd_Get_RGB565(0, 0, 0), "МАГНУМ", NULL },
        { 0, 110, 320, 150, Lcd_Get_RGB565(0, 0, 0), "Иконки и статусы температуры", NULL },
        { 20, 150, 150, 210, Lcd_Get_RGB565(10, 20, 31), "Печать", NULL },
        { 170, 150, 300, 210, Lcd_Get_RGB565(10, 20, 31), "Настройки", uiSetupMenu },
    };

	uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
}

void uiSetupMenu (xUIEvent_t *pxEvent) {

    xButton_t menu[] = {
        { 5,  20,  80, 80, Lcd_Get_RGB565(31, 20, 10), "T Off", NULL },
        { 85,  20, 160, 80, Lcd_Get_RGB565(31, 20, 10), "ДВ Off", NULL },
        { 165, 20, 240, 80, Lcd_Get_RGB565(31, 20, 10), "Ст. вниз", NULL },
        { 245, 20, 315, 80, Lcd_Get_RGB565(31, 20, 10), "Парк XY", NULL },
        { 20, 100, 300, 130, Lcd_Get_RGB565(0, 0, 0), "Текущий IP: 192.168.0.253", NULL },
        { 20, 150, 130, 210, Lcd_Get_RGB565(10, 20, 31), "Пруток", NULL },
        { 150, 150, 300, 210, Lcd_Get_RGB565(10, 20, 31), "Движение по осям", NULL },
    };

	uiMenuHandleEventDefault(menu, sizeof(menu)/sizeof(xButton_t), pxEvent);
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

	if (!path)
		return;

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

/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/
