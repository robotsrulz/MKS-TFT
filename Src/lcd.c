/**
  ******************************************************************************
  * @file   lcd.c
  * @brief  This file contains useful stuff for ILI9325 LCD
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

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "lcd.h"

static uint8_t lcd_orientation = 0;

void Lcd_Com(uint16_t addr) {

	HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_nRD_GPIO_Port, LCD_nRD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);

	GPIOE->ODR = addr;
	HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_SET);
}

void Lcd_Set_Data(uint16_t data) {

	HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_nRD_GPIO_Port, LCD_nRD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);

	GPIOE->ODR = data;
	HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_SET);
}

uint16_t Lcd_Get_Data() {

	uint16_t data;

	HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_nRD_GPIO_Port, LCD_nRD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);

	GPIOE->CRH = 0x44444444u;
	GPIOE->CRL = 0x44444444u;
	HAL_GPIO_WritePin(LCD_nRD_GPIO_Port, LCD_nRD_Pin, GPIO_PIN_RESET);

	data = GPIOE->IDR;
	GPIOE->CRH = 0x33333333u;
	GPIOE->CRL = 0x33333333u;

	HAL_GPIO_WritePin(LCD_nRD_GPIO_Port, LCD_nRD_Pin, GPIO_PIN_SET);
	return data;
}

void Lcd_Com_Data(uint16_t addr, uint16_t data) {

	HAL_GPIO_WritePin(LCD_nRD_GPIO_Port, LCD_nRD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);

	GPIOE->ODR = addr;
	HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);

	GPIOE->ODR = data;
	HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_nWR_GPIO_Port, LCD_nWR_Pin, GPIO_PIN_SET);
}

void Lcd_Fill_Screen(uint16_t color) {
	Lcd_Com_Data(0x20, 0);
	Lcd_Com_Data(0x22, 0);

	Lcd_Com(0x0022);

	int j = 320 * 240;

	while (j--) {
		Lcd_Set_Data(color);
	}
}

uint16_t Lcd_Get_RGB565(uint8_t red, uint8_t green, uint8_t blue) {

	uint16_t c = blue & 0x1f;
	c += ((uint16_t) green & 0x3f) << 5;
	c += ((uint16_t) red & 0x1f) << 11;

	return c;
}

void Lcd_Init(uint8_t orientation) {

	uint16_t R01h, R03h, R60h;

	lcd_orientation = orientation & 7;

	// turn on backlight
	HAL_GPIO_WritePin(LCD_BACKLIGHT_GPIO_Port, LCD_BACKLIGHT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_nCS_GPIO_Port, LCD_nCS_Pin, GPIO_PIN_RESET);

	switch (orientation) {
	case LCD_LANDSCAPE_CR:
		R01h = (0 << 8) | (0 << 10);// SS = 0, SM = 0,  from S1 to S720 (see also  GS bit (R60h))
		R03h = (1 << 12) | (1 << 5) | (1 << 4) | (1 << 3);// TRI=0, DFM=0, BGR=1, ORG=0, I/D[1:0]=11, AM=1
		R60h = (1 << 15) | (0x27 << 8);	// Gate Scan Control (R60h) GS=1(G320) NL[5:0]=0x27 (320 lines)
		break;
	case LCD_LANDSCAPE_CL:
		R01h = (1 << 8) | (0 << 10);// SS = 1, SM = 0,  from S720 to S1 (see also  GS bit (R60h))
		R03h = (1 << 12) | (1 << 5) | (1 << 4) | (1 << 3);// TRI=0, DFM=0, BGR=1, ORG=0, I/D[1:0]=11, AM=1
		R60h = (0 << 15) | (0x27 << 8);	// Gate Scan Control (R60h) GS=0(G1) NL[5:0]=0x27 (320 lines)
		break;
	case LCD_PORTRAIT_CUP:
		R01h = (0 << 8) | (0 << 10);// SS = 0, SM = 0,  from S1 to S720 (see also  GS bit (R60h))
		R03h = (1 << 12) | (1 << 5) | (1 << 4) | (0 << 3);// TRI=0, DFM=0, BGR=1, ORG=0, I/D[1:0]=11, AM=0
		R60h = (0 << 15) | (0x27 << 8);	// Gate Scan Control (R60h) GS=0(G1) NL[5:0]=0x27 (320 lines)
		break;
	case LCD_PORTRAIT_CDN:
	default:
		R01h = (1 << 8) | (0 << 10);// SS = 1, SM = 0,  from S720 to S1 (see also  GS bit (R60h))
		R03h = (1 << 12) | (0 << 5) | (1 << 4) | (0 << 3);// TRI=0, DFM=0, BGR=1, ORG=0, I/D[1:0]=01, AM=0
		R60h = (1 << 15) | (0x27 << 8);	// Gate Scan Control (R60h) GS=1(G320) NL[5:0]=0x27 (320 lines)
		break;
	}

	Lcd_Com_Data(0x0001, R01h);			// Driver Output Control Register (R01h)
	Lcd_Com_Data(0x0002, 0x0700);		// LCD Driving Waveform Control (R02h)
	Lcd_Com_Data(0x0003, R03h);			// Entry Mode (R03h)
	Lcd_Com_Data(0x0004, 0x0000);		// Resizing Control Register (R04h), W
	Lcd_Com_Data(0x0008, 0x0202);

	Lcd_Com_Data(0x0009, 0x0000);
	Lcd_Com_Data(0x000a, 0x0000);

	Lcd_Com_Data(0x0010, 0x0000);         // Power Control 1 (R10h)
	Lcd_Com_Data(0x0011, 0x0007);         // Power Control 2 (R11h)
	Lcd_Com_Data(0x0012, 0x0000);         // Power Control 3 (R12h)
	Lcd_Com_Data(0x0013, 0x0000);         // Power Control 4 (R13h)

	osDelay(200);

	Lcd_Com_Data(0x0010, 0x14B0);         // Power Control 1 (R10h)
	osDelay(50);
	Lcd_Com_Data(0x0011, 0x0007);         // Power Control 2 (R11h)
	osDelay(50);
	Lcd_Com_Data(0x0012, 0x008E);         // Power Control 3 (R12h)
	Lcd_Com_Data(0x0013, 0x0C00);         // Power Control 4 (R13h)

	Lcd_Com_Data(0x0029, 0x0015);         // NVM read data 2 (R29h)
	osDelay(50);

	Lcd_Com_Data(0x0030, 0x0000);         // Gamma Control 1
	Lcd_Com_Data(0x0031, 0x0107);         // Gamma Control 2
	Lcd_Com_Data(0x0032, 0x0000);         // Gamma Control 3
	Lcd_Com_Data(0x0035, 0x0203);         // Gamma Control 4
	Lcd_Com_Data(0x0036, 0x0402);         // Gamma Control 5
	Lcd_Com_Data(0x0037, 0x0000);         // Gamma Control 6
	Lcd_Com_Data(0x0038, 0x0207);         // Gamma Control 7
	Lcd_Com_Data(0x0039, 0x0000);         // Gamma Control 8
	Lcd_Com_Data(0x003c, 0x0203);         // Gamma Control 9
	Lcd_Com_Data(0x003d, 0x0403);         // Gamma Control 10

	Lcd_Com_Data(0x0050, 0x0000);		  // Window Horizontal RAM Address Start (R50h)
	Lcd_Com_Data(0x0051, 239);			  // Window Horizontal RAM Address End (R51h)
	Lcd_Com_Data(0x0052, 0x0000);		  // Window Vertical RAM Address Start (R52h)
	Lcd_Com_Data(0x0053, 319);			  // Window Vertical RAM Address End (R53h)

	Lcd_Com_Data(0x0060, R60h);			  // Driver Output Control (R60h)
	Lcd_Com_Data(0x0061, 0x0001);		  // Driver Output Control (R61h)
	Lcd_Com_Data(0x0090, 0x0010);		  // Panel Interface Control 1 (R90h)

	Lcd_Com_Data(0x0007, 0x0133);		  // Display Control 1 (R07h) W,
	osDelay(100);
}

void Lcd_Go_XY (uint16_t x, uint16_t y)
{
	Lcd_Com_Data ((lcd_orientation & 1) ? 0x0021 : 0x0020, x);
	Lcd_Com_Data ((lcd_orientation & 1) ? 0x0020 : 0x0021, y);
}

void Lcd_Put_Pix (uint16_t x, uint16_t y, uint16_t col)
{
	Lcd_Go_XY (x, y);
	Lcd_Com_Data (0x0022, col);  // col - цвет пикселя
}

uint16_t Lcd_Get_Pix(uint16_t x, uint16_t y) {

	Lcd_Go_XY (x, y);
	Lcd_Com(0x0022);

	return Lcd_Get_Data();
}

static uint16_t copy_buf[320 << 1];

void Lcd_Copy_Region(uint16_t x1, uint16_t y1, uint16_t width, uint16_t height,
		uint16_t x2, uint16_t y2) {

	if (x1 > LCD_MAX_X || y1 > LCD_MAX_Y || x2 > LCD_MAX_X || y2 > LCD_MAX_Y) {
		return;
	}

	for (uint16_t y = y1;
			y < MIN(y1 + height, LCD_MAX_Y) && y - y1 + y2 < LCD_MAX_Y; y++) {

		uint16_t *pb = copy_buf;
		for (uint16_t x = x1; x < MIN(x1 + width, LCD_MAX_X); x++, pb++) {
			*pb = Lcd_Get_Pix(x, y);
		}

		Lcd_Go_XY (x2, y - y1 + y2);
		Lcd_Com(0x0022);

		pb = copy_buf;

		for (uint16_t x = x2; x < MIN(x2 + width, LCD_MAX_X); x++, pb++) {
			Lcd_Set_Data(*pb);
		}
	}
}

void Lcd_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t c)
{
	int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

	dx = x2 - x1;
	dy = y2 - y1;
	dx1 = dx > 0 ? dx : -dx;
	dy1 = dy > 0 ? dy : -dy;
	px = 2 * dy1 - dx1;
	py = 2 * dx1 - dy1;
	if (dy1 <= dx1) {
		if (dx >= 0) {
			x = x1;
			y = y1;
			xe = x2;
		} else {
			x = x2;
			y = y2;
			xe = x1;
		}
		Lcd_Put_Pix(x, y, c);
		for (i = 0; x < xe; i++) {
			x = x + 1;
			if (px < 0) {
				px = px + 2 * dy1;
			} else {
				if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
					y = y + 1;
				} else {
					y = y - 1;
				}
				px = px + 2 * (dy1 - dx1);
			}
			Lcd_Put_Pix(x, y, c);
		}
	} else {
		if (dy >= 0) {
			x = x1;
			y = y1;
			ye = y2;
		} else {
			x = x2;
			y = y2;
			ye = y1;
		}
		Lcd_Put_Pix(x, y, c);
		for (i = 0; y < ye; i++) {
			y = y + 1;
			if (py <= 0) {
				py = py + 2 * dx1;
			} else {
				if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
					x = x + 1;
				} else {
					x = x - 1;
				}
				py = py + 2 * (dx1 - dy1);
			}
			Lcd_Put_Pix(x, y, c);
		}
	}
}

void Lcd_Render_Bitmap_8xN(uint16_t x, uint16_t y, uint8_t height, uint8_t *bitmap, uint16_t color)
{
	for (int y1=0; y1 < height; y1++) {

		if ((y1 + y < LCD_MAX_Y) && bitmap[y1]) {

    	    for (int x1=0; x1<8; x1++) {

    	    	if ((x1 + x < LCD_MAX_X) && (bitmap[y1] & 1 << x1)) {

					Lcd_Put_Pix (x + x1, y + y1, color);
				}
			}
    	}
    }
}

__STATIC_INLINE char _R( const char c ) {
    if ((uint8_t) c > 239)
            return (char) ((uint8_t) c - 16);
    if ((uint8_t) c > 191)
            return (char) ((uint8_t) c - 64);
    return c;
}

void Lcd_Put_Text(uint16_t x, uint16_t y, uint8_t height, char *text, uint16_t color) {

	uint8_t *bitmap;

	for(; *text; x += 8, text++) {

		int idx = _R(* (uint8_t *) text);

		switch (height) {
		case 14: bitmap = cp866_8x14_psf[idx]; break;
		case 16: bitmap = cp866_8x16_psf[idx]; break;
		case 8:	 bitmap = cp866_8x8_psf[idx];  break;
		default: return;
			break;
		}

		Lcd_Render_Bitmap_8xN(x, y, height, bitmap, color);
	}
}

void Lcd_Fill_Rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {

	if ((int)x2 - x1 <= 0 || (int)y2 - y1 <= 0 || x1 >= LCD_MAX_X || y1 >= LCD_MAX_Y)
		return;

	if (x2 >= LCD_MAX_X) x2 = LCD_MAX_X - 1;
	if (y2 >= LCD_MAX_Y) y2 = LCD_MAX_Y - 1;

	for (int y = y1; y <= y2; y++) {

		Lcd_Go_XY (x1, y);
		Lcd_Com (0x0022);

		for (int x = x1; x <= x2; x++) {
			Lcd_Set_Data (color);
		}
	}
}

uint8_t Lcd_Orientation() {
	return lcd_orientation;
}

void Lcd_Translate_Touch_Pos(uint16_t raw_x, uint16_t raw_y, uint16_t *x,
		uint16_t *y) {

	if (raw_x < TOUCH_X_LOW)  raw_x = TOUCH_X_LOW;
	if (raw_x > TOUCH_X_HIGH) raw_x = TOUCH_X_HIGH;
	if (raw_y < TOUCH_Y_LOW)  raw_y = TOUCH_Y_LOW;
	if (raw_y > TOUCH_Y_HIGH) raw_y = TOUCH_Y_HIGH;

	raw_x -= TOUCH_X_LOW;
	raw_y -= TOUCH_Y_LOW;

	switch (lcd_orientation) {
	case LCD_LANDSCAPE_CL:
		*x = 320 - (raw_x * 320 / (TOUCH_X_HIGH - TOUCH_X_LOW));
		if (*x > 320) *x = 1;
		*y = raw_y * 240 / (TOUCH_Y_HIGH - TOUCH_Y_LOW);
		break;
	case LCD_LANDSCAPE_CR:
		*x = raw_x * 320 / (TOUCH_X_HIGH - TOUCH_X_LOW);
		*y = 240 - (raw_y * 240 / (TOUCH_Y_HIGH - TOUCH_Y_LOW));
		if (*y > 240) *y = 1;
		break;
	case LCD_PORTRAIT_CDN:
		*x = raw_y * 240 / (TOUCH_Y_HIGH - TOUCH_Y_LOW);
		*y = raw_x * 320 / (TOUCH_X_HIGH - TOUCH_X_LOW);
		break;
	case LCD_PORTRAIT_CUP:
	default:
		*x = 240 - (raw_y * 240 / (TOUCH_Y_HIGH - TOUCH_Y_LOW));
		if (*x > 240) *x = 1;
		*y = 320 - (raw_x * 320 / (TOUCH_X_HIGH - TOUCH_X_LOW));
		if (*y > 320) *y = 1;
		break;
	}
}

short Lcd_Touch_Get_Closest_Average(uint16_t *d3) {

	int d[3];
	int i, av = 0;

	d[0] = d3[0] - d3[1]; // get the differentials
	d[1] = d3[1] - d3[2];
	d[2] = d3[2] - d3[0];

	for (i = 0; i < 3; i++) { // get the absolute differentials
		if (d[i] < 0) { d[i] = -d[i]; }
	}
	if (d[0] < d[1]) { // when d[0] < d[1]
		if (d[0] < d[2]) { // compare d[0] to d[2]
			av = d3[0] + d3[1]; // d[0] is the smallest if d[0] < d[2] also
		} else {
			av = d3[0] + d3[2]; // d[2] is the smallest if d[2] <= d[0] < d[1]
		}
	} else { // otherwise, i.e.: when d[1] <= d[0]
		if (d[1] < d[2]) { // compare d[1] to d[2]
			av = d3[1] + d3[2]; // d[1] is the smallest if d[1] < d[2] also
		} else {
			av = d3[0] + d3[2]; // d[2] is the smallest if d[2] <= d[1] <= d[0]
		}
	}
	av >>= 1;
	return av;
}

/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/
