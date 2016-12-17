/**
  ******************************************************************************
  * File Name          : tga.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TGA_H
#define __TGA_H

#include "cmsis_os.h"
#include "fatfs.h"

typedef struct {
  unsigned char idLength;
  unsigned char colorMapType;

  /* The image type. */
#define TGA_TYPE_MAPPED 1
#define TGA_TYPE_COLOR 2
#define TGA_TYPE_GRAY 3
#define TGA_TYPE_MAPPED_RLE 9
#define TGA_TYPE_COLOR_RLE 10
#define TGA_TYPE_GRAY_RLE 11
  unsigned char imageType;

  /* Color Map Specification. */
  /* We need to separately specify high and low bytes to avoid endianness
     and alignment problems. */
  unsigned char colorMapIndexLo, colorMapIndexHi;
  unsigned char colorMapLengthLo, colorMapLengthHi;
  unsigned char colorMapSize;

  /* Image Specification. */
  unsigned char xOriginLo, xOriginHi;
  unsigned char yOriginLo, yOriginHi;

  unsigned char widthLo, widthHi;
  unsigned char heightLo, heightHi;

  unsigned char bpp;

  /* Image descriptor.
     3-0: attribute bpp
     4:   left-to-right ordering
     5:   top-to-bottom ordering
     7-6: zero
     */
#define TGA_DESC_ABITS 0x0f
#define TGA_DESC_HORIZONTAL 0x10
#define TGA_DESC_VERTICAL 0x20
  unsigned char descriptor;

} TgaHeader;

typedef struct {
  unsigned int extensionAreaOffset;
  unsigned int developerDirectoryOffset;
#define TGA_SIGNATURE "TRUEVISION-XFILE"
  char signature[16];
  char dot;
  char null;
} TgaFooter;

typedef enum {
	TGA_COLOR_INDEX = 0, TGA_LUMINANCE, TGA_BGRA_EXT, TGA_BGR_EXT
} TgaImageType;

#define TGA_STATIC_BUF	1		// 0 - use pvPortMalloc, 1 - static buffer

#ifdef __fatfs_H
int read_tga_direct(FIL *fp, short x, short y, unsigned short *pwidth,
		unsigned short *pheight);
#endif

#endif /* __TGA_H */
/************************ (C) COPYRIGHT Roman Stepanov *****END OF FILE****/
