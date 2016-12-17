/**
  ******************************************************************************
  * File Name          : tga.c
  * Description        : Interface for TrueVision (TGA) image file loader
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

#include "tga.h"
#include "lcd.h"
#include "string.h"
#include "stdlib.h"

#define RLE_PACKETSIZE 0x80

#if TGA_STATIC_BUF == 1
 static unsigned short cmap16[0x100];
 static unsigned char data[0x100 << 2];
#else
 static unsigned short *cmap16 = NULL;
 static unsigned char *data = NULL;
#endif /* TGA_STATIC_BUF == 1 */

inline uint16_t blend_rgb16_argb(uint16_t rgb, uint8_t a8, uint8_t r8, uint8_t g8, uint8_t b8) {

	uint8_t a6 = a8 >> 2;	// convert to 6-bit;
	a8 >>= 3;				// convert to 5-bit

	uint16_t r = (((rgb & 0xf800) >> 11) * (0x1fu - a8) + (r8 >> 3) * a8) >> 5;
	uint16_t g = (((rgb & 0x07e0) >> 6)  * (0x3fu - a6) + (g8 >> 2) * a6) >> 6;
	uint16_t b = ((rgb & 0x001f) * (0x1fu - a8) + (b8 >> 3) * a8) >> 5;

	return Lcd_Get_RGB565(r, g, b);
}

static int
rle_fread(unsigned char *p, size_t datasize, size_t nelems, short x, short y, FIL *fp)
{
	int j, k;
	int buflen, bytes;
	unsigned char count;
	unsigned int br;

	/* Scale the buffer length. */
	buflen = nelems * datasize;

	j = 0;
	while (j < buflen) {
		/* Decode the next packet. */
		if (f_read(fp, &count, 1, &br) != FR_OK) {
			return j / datasize;
		}

		/* Scale the byte length to the size of the data. */
		bytes = ((count & ~RLE_PACKETSIZE) + 1) * datasize;

		if (count & RLE_PACKETSIZE) {
			/* Fill the buffer with the next value. */
			if (f_read(fp, p, datasize, &br) != FR_OK || br != datasize) {
				return j / datasize;
			}

			/* Optimized case for single-byte encoded data. */
			for (k=0; k<bytes; k += datasize) {
				// TODO: check if x coordinate is sane value
				if (j / datasize + x + k
						>= 0&& j / datasize + x + k < LCD_MAX_X && y >= 0 && y < LCD_MAX_Y) {

					switch (datasize) {
					case 1:
						Lcd_Set_Data(cmap16[*p]);
						break;
					case 3:
						Lcd_Set_Data(Lcd_Get_RGB565(p[2] >> 3, p[1] >> 2, p[0] >> 3));
						break;
					case 4:
						switch(p[3]) {
						case 0:
							Lcd_Set_Data(Lcd_Get_Data()); // advance 1px
							break;
						case 0xff:
							Lcd_Set_Data(Lcd_Get_RGB565(p[2] >> 3, p[1] >> 2, p[0] >> 3));
							break;
						default: /* do alpha blending */
							Lcd_Set_Data(blend_rgb16_argb(Lcd_Get_Data(),
									p[3], p[2], p[1], p[0]));
							break;
						}
						break;
					default:
						break;
					}
				}
			}
		} else {
			/* Read in the buffer. */
			if (f_read(fp, p, bytes, &br) != FR_OK || br != bytes) {
				return j / datasize;
			}

			for (k=0; k<bytes; k += datasize) {
				if (j / datasize + x + k / datasize
						>= 0&& j / datasize + x + k / datasize < LCD_MAX_X && y >= 0 && y < LCD_MAX_Y) {

					switch (datasize) {
					case 1:
						Lcd_Set_Data(cmap16[*(p + k)]);
						break;
					case 3:
						Lcd_Set_Data(Lcd_Get_RGB565(p[k * 3 + 2] >> 3, p[k * 3 + 1] >> 2, p[k * 3] >> 3));
						break;
					case 4:
						switch(p[k * 4 + 3]) {
						case 0:
							Lcd_Set_Data(Lcd_Get_Data()); // advance 1px
							break;
						case 0xff:
							Lcd_Set_Data(Lcd_Get_RGB565(
									p[k * 4 + 2] >> 3,
									p[k * 4 + 1] >> 2,
									p[k * 4]     >> 3));
							break;
						default: /* do alpha blending */
							Lcd_Set_Data(
									blend_rgb16_argb(Lcd_Get_Data(),
											p[k * 4 + 3], p[k * 4 + 2], p[k * 4 + 1], p[k * 4]));
							break;
						}
						break;
					default:
						break;
					}
				}
			}
		}

		j += bytes;
	}

	return nelems;
}

int read_tga_direct(FIL *fp, short x, short y, unsigned short *pwidth,
		unsigned short *pheight) {
	TgaHeader tgaHeader;
	char horzrev, vertrev;
	int width, height, bpp;
	int start, end, dir;
	int i, j;
	int pelbytes;
	int rle, format;
	int index, length;
	unsigned int br;

	if (f_lseek(fp, 0)
			|| f_read(fp, &tgaHeader, sizeof(tgaHeader), &br) != FR_OK) {
		return -1;
	}

	if (tgaHeader.idLength
			&& f_lseek(fp, sizeof(tgaHeader) + tgaHeader.idLength) != FR_OK) {
		return -1;
	}

	/* Reassemble the multi-byte values correctly, regardless of
	 host endianness. */
	width = (tgaHeader.widthHi << 8) | tgaHeader.widthLo;
	height = (tgaHeader.heightHi << 8) | tgaHeader.heightLo;

	if (width > LCD_MAX_X || height > LCD_MAX_Y) {
		return -1;
	}

	bpp = tgaHeader.bpp;
	horzrev = !(tgaHeader.descriptor & TGA_DESC_HORIZONTAL);
	vertrev = !(tgaHeader.descriptor & TGA_DESC_VERTICAL);

	if (!horzrev) {
		return -1; // too slow!
	}

	rle = 0;
	switch (tgaHeader.imageType) {
	case TGA_TYPE_MAPPED_RLE:
		rle = 1;
	case TGA_TYPE_MAPPED:
		/* Test for alpha channel. */
		format = TGA_COLOR_INDEX;
		pelbytes = 1;
		break;

	case TGA_TYPE_COLOR_RLE:
		rle = 1;
	case TGA_TYPE_COLOR:
		/* Test for alpha channel. */
		if (bpp == 32) {
			format = TGA_BGRA_EXT;
			pelbytes = 4;
		} else {
			format = TGA_BGR_EXT;
			pelbytes = 3;
		}
		break;

	case TGA_TYPE_GRAY_RLE:
	case TGA_TYPE_GRAY:
	default:
		return -1;
	}

	if ((format == TGA_BGRA_EXT && (bpp != 32))
			|| (format == TGA_BGR_EXT && bpp != 24)
			|| ((format == TGA_LUMINANCE || format == TGA_COLOR_INDEX) && bpp != 8)) {
		return -1;
	}

	/* Check that we have a color map only when we need it. */
	if (format == TGA_COLOR_INDEX) {
		if (tgaHeader.colorMapType != 1) {
			return -1;
		}
	} else if (tgaHeader.colorMapType != 0) {
		return -1;
	}

	if (tgaHeader.colorMapType == 1) {
		/* We need to read in the colormap. */
		index = (tgaHeader.colorMapIndexHi << 8) | tgaHeader.colorMapIndexLo;
		length = (tgaHeader.colorMapLengthHi << 8) | tgaHeader.colorMapLengthLo;

		if (!length || tgaHeader.colorMapSize != 24) {
			return -1;
		}

#if TGA_STATIC_BUF == 0
		data = pvPortMalloc(0x100 << 2);
		if (!data)
			return -1;
		cmap16 = pvPortMalloc(0x200);
		if (!cmap16) {
			vPortFree(data);
			return -1;
		}
#endif /* TGA_STATIC_BUF == 0 */

		/* Read in the rest of the colormap. */
		if (((length + index) * 3) > (0x100 << 2)
				|| f_read(fp, data, 3 * length, &br) != FR_OK) {
#if TGA_STATIC_BUF == 0
			vPortFree(data);
			vPortFree(cmap16);
#endif /* TGA_STATIC_BUF == 0 */
			return -1;
		}

		/* Rearrange the colors from BGR to 16-bit RGB(565). */
		for (j = index; j < length * 3; j += 3) {
			cmap16[j / 3] = Lcd_Get_RGB565(data[j + 2] >> 3, data[j + 1] >> 2,
					data[j] >> 3);
		}
	}

	if (!vertrev) {
		start = 0;
		end = height;
		dir = 1;
	} else {
		/* We need to reverse loading order of rows. */
		start = height - 1;
		end = -1;
		dir = -1;
	}

	for (i = start; i != end; i += dir) {

		/* Suck in the data one row at a time. */
		if (rle) {
			if ((i + y) >= 0 && (i + y) < LCD_MAX_Y) {

				int k = (x < 0) ? -x : 0;

				if ((k + x) < LCD_MAX_X) {

					Lcd_Go_XY (x + k, i + y);
					Lcd_Com (0x0022);
				}
			}

			if (rle_fread(data, pelbytes, width, x, i + y, fp) != width) {
#if TGA_STATIC_BUF == 0
				if (tgaHeader.colorMapType == 1 && cmap16) {
					vPortFree(cmap16);
				}
				vPortFree(data);
#endif /* TGA_STATIC_BUF == 0 */
				return -1;
			}
		} else {
			if (f_read(fp, data, width * pelbytes, &br) != FR_OK
					&& br != width) {
#if TGA_STATIC_BUF == 0
				if (tgaHeader.colorMapType == 1 && cmap16) {
					vPortFree(cmap16);
				}
				vPortFree(data);
#endif /* TGA_STATIC_BUF == 0 */
				return -1;
			}

			/* put row on screen */
			if ((i + y) >= 0 && (i + y) < LCD_MAX_Y) {

				int k = (x < 0) ? -x : 0;

				if ((k + x) < LCD_MAX_X) {

					Lcd_Go_XY (x + k, i + y);
					Lcd_Com (0x0022);

					for (; (k < width) && ((k + x) < LCD_MAX_X); k++) {
						unsigned int color = 0;

						switch(format) {
						case TGA_COLOR_INDEX:
							color = cmap16[data[k]];
							break;
						case TGA_BGR_EXT:
							color = Lcd_Get_RGB565(
									data[k * 3 + 2] >> 3,
									data[k * 3 + 1] >> 2,
									data[k * 3]     >> 3);
							break;
						case TGA_BGRA_EXT:
							switch(data[k * 4 + 3]) {
							case 0:
								color = Lcd_Get_Data(); // return background color
								break;
							case 0xff:
								color = Lcd_Get_RGB565(
										data[k * 4 + 2] >> 3,
										data[k * 4 + 1] >> 2,
										data[k * 4]     >> 3);
								break;
							default: /* do alpha blending */
								color = blend_rgb16_argb(Lcd_Get_Data(),
										data[k * 4 + 3], data[k * 4 + 2], data[k * 4 + 1], data[k * 4]);
								break;
							}
							break;
						default:
							// TODO: implement grayscale
							break;
						}
						Lcd_Set_Data (color);

						// prevent GCC to optimize out this cycle
					    __asm__ __volatile__("");
					}
				}
			}
		}
	}

	if (pwidth) *pwidth = width;
	if (pheight) *pheight = height;

#if TGA_STATIC_BUF == 0
	if (tgaHeader.colorMapType == 1 && cmap16) {
		vPortFree(cmap16);
	}
	vPortFree(data);
#endif /* TGA_STATIC_BUF == 0 */

	return 0;
}
/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/

