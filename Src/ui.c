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

#define READY_PRINT	"MyFirmware"

uint8_t moveStep = MOVE_10;
uint8_t offMode = MANUAL_OFF;
uint8_t connectSpeed = CONNECT_115200;
uint8_t preheatDev = PR_EXTRUDER_1;
uint8_t extrudeDev = EXTRUDER_1;
uint8_t preheatSelDegree = STEP_1_DEGREE;
uint8_t extrudeDistance = DISTANCE_1;
uint8_t extrudeSelSpeed = SPEED_SLOW;

/*
 * user callback declaration
 */

void uiInitialize (xEvent_t *pxEvent);
void uiMainMenu   (xEvent_t *pxEvent);
void uiPreheatMenu(xEvent_t *pxEvent);
void uiHomeMenu   (xEvent_t *pxEvent);

void uiPreheatMenu(xEvent_t *pxEvent);
void uiPreheatSelectDev(xEvent_t *pxEvent);
void uiPreheatSelectStep(xEvent_t *pxEvent);

void uiExtrudeMenu(xEvent_t *pxEvent);
void uiExtrudeSelectDev(xEvent_t *pxEvent);
void uiExtrudeSelectStep(xEvent_t *pxEvent);
void uiExtrudeSelectSpeed(xEvent_t *pxEvent);

void uiMoveMenu   (xEvent_t *pxEvent);
void uiMoveMenuStepChange(xEvent_t *pxEvent);

void uiSetupMenu  (xEvent_t *pxEvent);
void uiSetupMenuOffMode(xEvent_t *pxEvent);
void uiSetupConnectMenu(xEvent_t *pxEvent);
void uiSetupConnect9600(xEvent_t *pxEvent);
void uiSetupConnect57600(xEvent_t *pxEvent);
void uiSetupConnect115200(xEvent_t *pxEvent);
void uiSetupConnect250000(xEvent_t *pxEvent);
void uiSetupWifi  (xEvent_t *pxEvent);
void uiSetupAbout (xEvent_t *pxEvent);

void uiMoreMenu   (xEvent_t *pxEvent);
void uiFileBrowse (xEvent_t *pxEvent);

typedef void (*volatile eventProcessor_t) (xEvent_t *);
eventProcessor_t processEvent = uiInitialize;

/*
 * service routines declaration
 */

static void uiMediaStateChange(uint16_t event);
static void uiRedrawFileList(int raw_x, int raw_y);
static void uiDrawProgressBar(uint32_t scale, uint16_t color);
static void uiUpdateProgressBar(uint32_t progress);
static void uiDrawBinIcon(const TCHAR *path, uint16_t x, uint16_t y,
		uint16_t width, uint16_t height, uint8_t resetWindow);


__STATIC_INLINE void uiNextState(void (*volatile next) (xEvent_t *pxEvent)) {
	processEvent = next;
	xEvent_t event = { INIT_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

__STATIC_INLINE void uiShortBeep() {
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	osDelay(12);
	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
}

typedef struct {

	const char				*pIconFile;
	const eventProcessor_t	pEventProcessor;
} xMenuItem_t;

__STATIC_INLINE void uiDrawMenu(const xMenuItem_t *pMenu) {

	if (pMenu) {
		uiDrawBinIcon(pMenu[0].pIconFile,	1, 16, 78, 104, 0);
		uiDrawBinIcon(pMenu[1].pIconFile,  81, 16, 78, 104, 0);
		uiDrawBinIcon(pMenu[2].pIconFile, 161, 16, 78, 104, 0);
		uiDrawBinIcon(pMenu[3].pIconFile, 241, 16, 78, 104, 0);

		uiDrawBinIcon(pMenu[4].pIconFile,	1, 18 + 104, 78, 104, 0);
		uiDrawBinIcon(pMenu[5].pIconFile,  81, 18 + 104, 78, 104, 0);
		uiDrawBinIcon(pMenu[6].pIconFile, 161, 18 + 104, 78, 104, 0);
		uiDrawBinIcon(pMenu[7].pIconFile, 241, 18 + 104, 78, 104, 1);
	}
}

static uint16_t touchX, touchY;

__STATIC_INLINE void uiMenuHandleEventDefault(const xMenuItem_t *pMenu, xEvent_t *pxEvent) {

	if (pxEvent) {
		switch (pxEvent->ucEventID) {
		case SDCARD_INSERT:
		case SDCARD_REMOVE:
		case USBDRIVE_INSERT:
		case USBDRIVE_REMOVE:
			uiMediaStateChange(pxEvent->ucEventID);
			break;

		case INIT_EVENT:
			if (pMenu) {
				Lcd_Fill_Screen(Lcd_Get_RGB565(0, 0, 0));
				uiDrawMenu(pMenu);
#if 1
				char buffer[12];
				sprintf(buffer, "%05u:%05u", touchX, touchY);
				Lcd_Put_Text(10, LCD_MAX_Y - 9, 8, buffer, 0xffffu);
#endif
			}
			break;

//		case TOUCH_DOWN_EVENT:
		case TOUCH_UP_EVENT:
			if (pMenu) {
				eventProcessor_t p = NULL;

				Lcd_Translate_Touch_Pos((pxEvent->ucData.touchXY) >> 16 & 0x7fffu,
						pxEvent->ucData.touchXY & 0x7fffu, &touchX, &touchY);

				uiShortBeep();
				if (touchY >= 16 && touchY <= 16 + 104) {
					p = pMenu[(touchX - 1) / 80].pEventProcessor;

				} else if (touchY >= 16 + 104 && touchY <= 16 + 104 + 80) {
					p = pMenu[4 + (touchX - 1) / 80].pEventProcessor;
				}
				if (p) uiNextState(p);
			}
			break;

		case UPDATE1_EVENT: uiDrawBinIcon(pMenu[0].pIconFile,	1, 16, 78, 104, 1); break;
		case UPDATE2_EVENT: uiDrawBinIcon(pMenu[1].pIconFile,  81, 16, 78, 104, 1); break;
		case UPDATE3_EVENT: uiDrawBinIcon(pMenu[2].pIconFile, 161, 16, 78, 104, 1); break;
		case UPDATE4_EVENT: uiDrawBinIcon(pMenu[3].pIconFile, 241, 16, 78, 104, 1); break;
		case UPDATE5_EVENT: uiDrawBinIcon(pMenu[4].pIconFile,	1, 18 + 104, 78, 104, 1); break;
		case UPDATE6_EVENT: uiDrawBinIcon(pMenu[5].pIconFile,  81, 18 + 104, 78, 104, 1); break;
		case UPDATE7_EVENT: uiDrawBinIcon(pMenu[6].pIconFile, 161, 18 + 104, 78, 104, 1); break;
		case UPDATE8_EVENT: uiDrawBinIcon(pMenu[7].pIconFile, 241, 18 + 104, 78, 104, 1); break;

		case UPDATE14_EVENT:
			uiDrawBinIcon(pMenu[0].pIconFile,	1, 16, 78, 104, 0);
			uiDrawBinIcon(pMenu[1].pIconFile,  81, 16, 78, 104, 0);
			uiDrawBinIcon(pMenu[2].pIconFile, 161, 16, 78, 104, 0);
			uiDrawBinIcon(pMenu[3].pIconFile, 241, 16, 78, 104, 1);
			break;

		default:
			break;
		}
	}
}

/*
 * user callback definition
 */

void uiInitialize (xEvent_t *pxEvent) {

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
		uiMenuHandleEventDefault(NULL, pxEvent);
}

void uiMainMenu (xEvent_t *pxEvent) {

	static const xMenuItem_t mainMenu[8] = {
		{ MKS_PIC_FL "/bmp_preHeat.bin", uiPreheatMenu },
		{ MKS_PIC_FL "/bmp_mov.bin", uiMoveMenu },
		{ MKS_PIC_FL "/bmp_zero.bin", uiHomeMenu },
		{ MKS_PIC_FL "/bmp_printing.bin", uiFileBrowse },
		{ MKS_PIC_FL "/bmp_extruct.bin", uiExtrudeMenu },
		{ MKS_PIC_FL "/bmp_fan.bin", NULL },
		{ MKS_PIC_FL "/bmp_set.bin", uiSetupMenu },
		{ MKS_PIC_FL "/bmp_More.bin", uiMoreMenu }
	};

	uiMenuHandleEventDefault(mainMenu, pxEvent);
	if (INIT_EVENT == pxEvent->ucEventID)
		Lcd_Put_Text(0, 0, 16, READY_PRINT, 0xffffu);
}

static xMenuItem_t setupMenu[8] = {
	{ MKS_PIC_FL "/bmp_fileSys.bin", NULL },
	{ MKS_PIC_FL "/bmp_adj.bin", NULL },
	{ MKS_PIC_FL "/bmp_wifi.bin", uiSetupWifi },
	{ MKS_PIC_FL "/bmp_connect.bin", uiSetupConnectMenu },
	{ MKS_PIC_FL "/bmp_about.bin", uiSetupAbout },
	{ MKS_PIC_FL "/bmp_lang.bin", NULL },
	{ MKS_PIC_FL "/bmp_manual_off.bin", uiSetupMenuOffMode },
	{ MKS_PIC_FL "/bmp_return.bin", uiMainMenu }
};

void uiSetupMenu (xEvent_t *pxEvent) {

	switch(offMode) {
	case MANUAL_OFF:
		setupMenu[6].pIconFile = MKS_PIC_FL "/bmp_auto_off.bin";
		break;

	default:
		setupMenu[6].pIconFile = MKS_PIC_FL "/bmp_manual_off.bin";
		break;
	}

	uiMenuHandleEventDefault(setupMenu, pxEvent);
	if (INIT_EVENT == pxEvent->ucEventID)
		Lcd_Put_Text(0, 0, 16, READY_PRINT ">Set", 0xffffu);
}

void uiSetupMenuOffMode(xEvent_t *pxEvent) {

	if (INIT_EVENT == pxEvent->ucEventID) {
		switch(offMode) {
		case MANUAL_OFF:
			offMode = AUTO_OFF;
			break;

		default:
			offMode = MANUAL_OFF;
			break;
		}
	}

	processEvent = uiSetupMenu;
	xEvent_t event = { UPDATE7_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

static xMenuItem_t setupConnectMenu[8] = {
	{ NULL, uiSetupConnect9600 },
	{ NULL, uiSetupConnect57600 },
	{ NULL, uiSetupConnect115200 },
	{ NULL, uiSetupConnect250000 },
	{ NULL, NULL },
	{ NULL, NULL },
	{ NULL, NULL },
	{ MKS_PIC_FL "/bmp_return.bin", uiSetupMenu }
};

void uiSetupConnectMenu(xEvent_t *pxEvent) {

	setupConnectMenu[0].pIconFile =
			(connectSpeed == CONNECT_9600) ?
					MKS_PIC_FL "/bmp_baud9600.bin" :
					MKS_PIC_FL "/bmp_baud9600_sel.bin";
	setupConnectMenu[1].pIconFile =
			(connectSpeed == CONNECT_57600) ?
					MKS_PIC_FL "/bmp_baud57600.bin" :
					MKS_PIC_FL "/bmp_baud57600_sel.bin";
	setupConnectMenu[2].pIconFile =
			(connectSpeed == CONNECT_115200) ?
					MKS_PIC_FL "/bmp_baud115200.bin" :
					MKS_PIC_FL "/bmp_baud115200_sel.bin";
	setupConnectMenu[3].pIconFile =
			(connectSpeed == CONNECT_250000) ?
					MKS_PIC_FL "/bmp_baud250000.bin" :
					MKS_PIC_FL "/bmp_baud250000_sel.bin";

	uiMenuHandleEventDefault(setupConnectMenu, pxEvent);
	if (INIT_EVENT == pxEvent->ucEventID)
		Lcd_Put_Text(0, 0, 16, READY_PRINT ">Set>ConnectSpeed", 0xffffu);
}

void uiSetupConnect9600(xEvent_t *pxEvent) {
	connectSpeed = CONNECT_9600;
	processEvent = uiSetupConnectMenu;
	xEvent_t event = { UPDATE14_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

void uiSetupConnect57600(xEvent_t *pxEvent) {
	connectSpeed = CONNECT_57600;
	processEvent = uiSetupConnectMenu;
	xEvent_t event = { UPDATE14_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

void uiSetupConnect115200(xEvent_t *pxEvent) {
	connectSpeed = CONNECT_115200;
	processEvent = uiSetupConnectMenu;
	xEvent_t event = { UPDATE14_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

void uiSetupConnect250000(xEvent_t *pxEvent) {
	connectSpeed = CONNECT_250000;
	processEvent = uiSetupConnectMenu;
	xEvent_t event = { UPDATE14_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

void uiSetupWifi (xEvent_t *pxEvent) {

	static const xMenuItem_t setupWifiMenu[8] = {
		{ NULL, NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ MKS_PIC_FL "/bmp_return.bin", uiSetupMenu }
	};

	uiMenuHandleEventDefault(setupWifiMenu, pxEvent);
	if (INIT_EVENT == pxEvent->ucEventID)
		Lcd_Put_Text(0, 0, 16, READY_PRINT ">Set>Wifi", 0xffffu);
}

void uiSetupAbout(xEvent_t *pxEvent) {

	static const xMenuItem_t setupAboutMenu[8] = {
		{ NULL, NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ MKS_PIC_FL "/bmp_return.bin", uiSetupMenu }
	};

	uiMenuHandleEventDefault(setupAboutMenu, pxEvent);
	if (INIT_EVENT == pxEvent->ucEventID)
		Lcd_Put_Text(0, 0, 16, READY_PRINT ">Set>About", 0xffffu);
}

void uiHomeMenu (xEvent_t *pxEvent) {

	static const xMenuItem_t homeMenu[8] = {
			{ MKS_PIC_FL "/bmp_zeroA.bin", NULL },
			{ MKS_PIC_FL "/bmp_zeroX.bin", NULL },
			{ MKS_PIC_FL "/bmp_zeroY.bin", NULL },
			{ MKS_PIC_FL "/bmp_zeroZ.bin", NULL },
			{ NULL, NULL },
			{ NULL, NULL },
			{ NULL, NULL },
			{ MKS_PIC_FL "/bmp_return.bin", uiMainMenu }
	};

	uiMenuHandleEventDefault(homeMenu, pxEvent);
	if (INIT_EVENT == pxEvent->ucEventID)
		Lcd_Put_Text(0, 0, 16, READY_PRINT ">Home", 0xffffu);
}

static xMenuItem_t moveMenu[8] = {
		{ MKS_PIC_FL "/bmp_xAdd.bin", NULL },
		{ MKS_PIC_FL "/bmp_yAdd.bin", NULL },
		{ MKS_PIC_FL "/bmp_zAdd.bin", NULL },
		{ MKS_PIC_FL "/bmp_step10_mm.bin", uiMoveMenuStepChange },
		{ MKS_PIC_FL "/bmp_xDec.bin", NULL },
		{ MKS_PIC_FL "/bmp_yDec.bin", NULL },
		{ MKS_PIC_FL "/bmp_zDec.bin", NULL },
		{ MKS_PIC_FL "/bmp_return.bin", uiMainMenu }
};

void uiMoveMenu (xEvent_t *pxEvent) {

	switch(moveStep) {
	case MOVE_10:
		moveMenu[3].pIconFile = MKS_PIC_FL "/bmp_step_move0_1.bin";
		break;
	case MOVE_01:
		moveMenu[3].pIconFile = MKS_PIC_FL "/bmp_step_move1.bin";
		break;
	case MOVE_1:
		moveMenu[3].pIconFile = MKS_PIC_FL "/bmp_step5_mm.bin";
		break;
	default:
		moveMenu[3].pIconFile = MKS_PIC_FL "/bmp_step10_mm.bin";
		break;
	}

	uiMenuHandleEventDefault(moveMenu, pxEvent);
	if (INIT_EVENT == pxEvent->ucEventID)
		Lcd_Put_Text(0, 0, 16, READY_PRINT ">Move", 0xffffu);
}

static xMenuItem_t preheatMenu[8] = {
		{ MKS_PIC_FL "/bmp_Add.bin", NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ MKS_PIC_FL "/bmp_Dec.bin", NULL },
		{ MKS_PIC_FL "/bmp_extru1.bin", uiPreheatSelectDev },
		{ MKS_PIC_FL "/bmp_step1_degree.bin", uiPreheatSelectStep },
		{ MKS_PIC_FL "/bmp_manual_off.bin", NULL },
		{ MKS_PIC_FL "/bmp_return.bin", uiMainMenu }
};

void uiPreheatMenu(xEvent_t *pxEvent) {

    switch(preheatDev)
    {
    case PR_EXTRUDER_1:
        preheatMenu[4].pIconFile = MKS_PIC_FL "/bmp_extru1.bin";
        break;

    case PR_EXTRUDER_2:
        preheatMenu[4].pIconFile = MKS_PIC_FL "/bmp_extru2.bin";
        break;

    default:
        preheatMenu[4].pIconFile = MKS_PIC_FL "/bmp_bed.bin";
        break;
    }

    switch (preheatSelDegree)
    {
    case STEP_1_DEGREE:
        preheatMenu[5].pIconFile = MKS_PIC_FL "/bmp_step1_degree.bin";
        break;

    case STEP_5_DEGREE:
        preheatMenu[5].pIconFile = MKS_PIC_FL "/bmp_step5_degree.bin";
        break;

    case STEP_10_DEGREE:
    default:
        preheatMenu[5].pIconFile = MKS_PIC_FL "/bmp_step10_degree.bin";
        break;
    }

    uiMenuHandleEventDefault(preheatMenu, pxEvent);
    if (INIT_EVENT == pxEvent->ucEventID)
        Lcd_Put_Text(0, 0, 16, READY_PRINT ">Preheat", 0xffffu);
}

void uiPreheatSelectDev(xEvent_t *pxEvent) {

    if (INIT_EVENT == pxEvent->ucEventID) {
        switch(preheatDev) {
        case PR_EXTRUDER_1:
            preheatDev = PR_EXTRUDER_2;
            break;

        case PR_EXTRUDER_2:
            preheatDev = PR_HEATBED;
            break;

        default:
            preheatDev = PR_EXTRUDER_1;
            break;
        }
    }

    processEvent = uiPreheatMenu;
	xEvent_t event = { UPDATE5_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

void uiPreheatSelectStep(xEvent_t *pxEvent) {

    if (INIT_EVENT == pxEvent->ucEventID) {
        switch (preheatSelDegree) {
        case STEP_1_DEGREE:
            preheatSelDegree = STEP_5_DEGREE;
            break;

        case STEP_5_DEGREE:
            preheatSelDegree = STEP_10_DEGREE;
            break;

        case STEP_10_DEGREE:
        default:
            preheatSelDegree = STEP_1_DEGREE;
            break;
        }
    }

    processEvent = uiPreheatMenu;
	xEvent_t event = { UPDATE6_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

static xMenuItem_t extrudeMenu[8] = {
		{ MKS_PIC_FL "/bmp_in.bin", NULL },
		{ NULL, NULL },
		{ NULL, NULL },
		{ MKS_PIC_FL "/bmp_out.bin", NULL },
		{ MKS_PIC_FL "/bmp_extru1.bin", uiExtrudeSelectDev },
		{ MKS_PIC_FL "/bmp_step1_mm.bin", uiExtrudeSelectStep },
		{ MKS_PIC_FL "/bmp_speed_slow.bin", uiExtrudeSelectSpeed },
		{ MKS_PIC_FL "/bmp_return.bin", uiMainMenu }
};

void uiExtrudeMenu(xEvent_t *pxEvent) {

    switch(extrudeDev)
    {
    case EXTRUDER_1:
        extrudeMenu[4].pIconFile = MKS_PIC_FL "/bmp_extru1.bin";
        break;

    default:
        extrudeMenu[4].pIconFile = MKS_PIC_FL "/bmp_extru2.bin";
        break;
    }

    switch (extrudeDistance)
    {
    case DISTANCE_1:
        extrudeMenu[5].pIconFile = MKS_PIC_FL "/bmp_step1_mm.bin";
        break;

    case DISTANCE_5:
        extrudeMenu[5].pIconFile = MKS_PIC_FL "/bmp_step5_mm.bin";
        break;

    case DISTANCE_10:
    default:
        extrudeMenu[5].pIconFile = MKS_PIC_FL "/bmp_step10_mm.bin";
        break;
    }

    switch (extrudeSelSpeed)
    {
    case SPEED_SLOW:
        extrudeMenu[6].pIconFile = MKS_PIC_FL "/bmp_speed_slow.bin";
        break;

    case SPEED_NORMAL:
        extrudeMenu[6].pIconFile = MKS_PIC_FL "/bmp_speed_normal.bin";
        break;

    case SPEED_HIGH:
    default:
        extrudeMenu[6].pIconFile = MKS_PIC_FL "/bmp_speed_high.bin";
        break;
    }

    uiMenuHandleEventDefault(extrudeMenu, pxEvent);
    if (INIT_EVENT == pxEvent->ucEventID)
        Lcd_Put_Text(0, 0, 16, READY_PRINT ">Extrude", 0xffffu);
}

void uiExtrudeSelectDev(xEvent_t *pxEvent) {

    if (INIT_EVENT == pxEvent->ucEventID) {
        switch(extrudeDev) {
        case EXTRUDER_1:
            extrudeDev = EXTRUDER_2;
            break;

        case EXTRUDER_2:
        default:
            extrudeDev = EXTRUDER_1;
            break;
        }
    }

    processEvent = uiExtrudeMenu;
	xEvent_t event = { UPDATE5_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

void uiExtrudeSelectStep(xEvent_t *pxEvent) {

    if (INIT_EVENT == pxEvent->ucEventID) {
        switch (extrudeDistance) {
        case DISTANCE_1:
            extrudeDistance = DISTANCE_5;
            break;

        case DISTANCE_5:
            extrudeDistance = DISTANCE_10;
            break;

        case DISTANCE_10:
        default:
            extrudeDistance = DISTANCE_1;
            break;
        }
    }

    processEvent = uiExtrudeMenu;
	xEvent_t event = { UPDATE6_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

void uiExtrudeSelectSpeed(xEvent_t *pxEvent) {

    if (INIT_EVENT == pxEvent->ucEventID) {
        switch (extrudeSelSpeed) {
        case SPEED_SLOW:
            extrudeSelSpeed = SPEED_NORMAL;
            break;

        case SPEED_NORMAL:
            extrudeSelSpeed = SPEED_HIGH;
            break;

        case SPEED_HIGH:
        default:
            extrudeSelSpeed = SPEED_SLOW;
            break;
        }
    }

    processEvent = uiExtrudeMenu;
	xEvent_t event = { UPDATE7_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

void uiMoveMenuStepChange (xEvent_t *pxEvent) {

	if (INIT_EVENT == pxEvent->ucEventID) {
		switch(moveStep) {
		case MOVE_10:
			moveStep = MOVE_01;
			break;
		case MOVE_01:
			moveStep = MOVE_1;
			break;
		case MOVE_1:
			moveStep = MOVE_5;
			break;
		default:
			moveStep = MOVE_10;
			break;
		}
	}

	processEvent = uiMoveMenu;
	xEvent_t event = { UPDATE4_EVENT };
	xQueueSendToFront(xUIEventQueue, &event, 1000);
}

void uiMoreMenu (xEvent_t *pxEvent) {

	static const xMenuItem_t moreMenu[8] = {
			{ MKS_PIC_FL /* "/bmp_morefunc1.bin" */ "/bmp_custom1.bin", NULL },
			{ MKS_PIC_FL /* "/bmp_morefunc2.bin" */ "/bmp_custom2.bin", NULL },
			{ MKS_PIC_FL /* "/bmp_morefunc3.bin" */ "/bmp_custom3.bin", NULL },
			{ MKS_PIC_FL /* "/bmp_morefunc4.bin" */ "/bmp_custom4.bin", NULL },
			{ MKS_PIC_FL /* "/bmp_morefunc5.bin" */ "/bmp_custom5.bin", NULL },
			{ MKS_PIC_FL /* "/bmp_morefunc6.bin" */ "/bmp_custom6.bin", NULL },
			{ NULL, NULL },
			{ MKS_PIC_FL "/bmp_return.bin", uiMainMenu }
	};

	uiMenuHandleEventDefault(moreMenu, pxEvent);
	if (INIT_EVENT == pxEvent->ucEventID)
		Lcd_Put_Text(0, 0, 16, READY_PRINT ">More", 0xffffu);
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
