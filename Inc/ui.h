/**
  ******************************************************************************
  * File Name          : ui.h
  * Description        : This file contains user interface definitions
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
#ifndef __UI_H
#define __UI_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "ffconf.h"

#define FL_FONT_SIZE	16
#define FLIST_SIZE		((240 - 20) / FL_FONT_SIZE)

#if _USE_LFN != 0
# define NAMELEN	(_MAX_LFN + 1 + 1)
#else
# define NAMELEN	13
#endif

typedef struct
{
    enum {
    	INIT_EVENT = 0,
		UPDATE1_EVENT,
		UPDATE2_EVENT,
		UPDATE3_EVENT,
		UPDATE4_EVENT,
		UPDATE5_EVENT,
		UPDATE6_EVENT,
		UPDATE7_EVENT,
		UPDATE8_EVENT,
		UPDATE14_EVENT,
    	TOUCH_DOWN_EVENT,
		TOUCH_UP_EVENT,
		SDCARD_INSERT,
		SDCARD_REMOVE,
		USBDRIVE_INSERT,
		USBDRIVE_REMOVE
    } ucEventID;
    union {
    	unsigned int touchXY;
    } ucData;
} xEvent_t;

extern QueueHandle_t xUIEventQueue;

typedef void (*volatile eventProcessor_t) (xEvent_t *);
extern eventProcessor_t processEvent;

typedef enum {
	MOVE_01 = 0,
	MOVE_1,
	MOVE_5,
	MOVE_10
} xMoveStep_t;
extern uint8_t moveStep;

typedef enum {
	MANUAL_OFF = 0,
	AUTO_OFF
} xOffMode_t;
extern uint8_t offMode;

typedef enum {
	CONNECT_9600 = 0,
	CONNECT_57600,
	CONNECT_115200,
	CONNECT_250000
} xConnectSpeed_t;
extern uint8_t connectSpeed;

#endif /* __UI_H */
/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/
