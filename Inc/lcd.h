/**
  ******************************************************************************
  * File Name          : lcd.h
  * Description        : This file contains useful stuff for ILI9325 LCD
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
#ifndef __LCD_H
#define __LCD_H

#include "stm32f1xx_hal.h"

#define LCD_MAX_X	((Lcd_Orientation() & 1) ?  320 : 240)
#define LCD_MAX_Y	((Lcd_Orientation() & 1) ?  240 : 320)

typedef enum {
	LCD_PORTRAIT_CDN = 0,	/* portrait, connector side is down */
	LCD_LANDSCAPE_CR = 1,	/* landscape connector right side   */
	LCD_PORTRAIT_CUP = 2,	/* portrait, connector side is up   */
	LCD_LANDSCAPE_CL = 3	/* landscape connector left side    */
} lcd_orientation_t;

#define MIN(a,b) (((a) < (b)) ? (a) : (b))

uint8_t Lcd_Orientation(void);

#define TOUCH_X_LOW		1700
#define TOUCH_X_HIGH	30000
#define TOUCH_Y_LOW		2000
#define TOUCH_Y_HIGH	30000

short Lcd_Touch_Get_Closest_Average(uint16_t *d3);
void Lcd_Translate_Touch_Pos(uint16_t raw_x, uint16_t raw_y, uint16_t *x,
		uint16_t *y);

void Lcd_Init(uint8_t orientation);
void Lcd_Com(uint16_t com);
void Lcd_Set_Data(uint16_t data);
uint16_t Lcd_Get_Data();
void Lcd_Com_Data(uint16_t addr, uint16_t data);
int Lcd_Read_Id(void);
void Lcd_Go_XY(uint16_t x, uint16_t y);
void Lcd_Put_Pix(uint16_t x, uint16_t y, uint16_t col);
uint16_t Lcd_Get_Pix(uint16_t x, uint16_t y);
void Lcd_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t c);
void Lcd_Fill_Screen(uint16_t color);
void Lcd_Fill_Rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
		uint16_t color);
void Lcd_Copy_Region(uint16_t x1, uint16_t y1, uint16_t width, uint16_t height,
		uint16_t x2, uint16_t y2);
uint16_t Lcd_Get_RGB565(uint8_t red, uint8_t green, uint8_t blue);

extern unsigned char cp866_8x8_psf[256][8];
extern unsigned char cp866_8x14_psf[256][14];
extern unsigned char cp866_8x16_psf[256][16];

typedef enum {
	FONT_8x8  = 8,
	FONT_8x14 = 14,
	FONT_8x16 = 16
} font_size_t;

void Lcd_Render_Bitmap_8xN(uint16_t x, uint16_t y, uint8_t height, uint8_t *bitmap,
		uint16_t color);
void Lcd_Put_Text(uint16_t x, uint16_t y, uint8_t height, char *text, uint16_t color);

/**
  * @}
  */

/**
  * @}
*/

#endif /* __LCD_H */
/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/
