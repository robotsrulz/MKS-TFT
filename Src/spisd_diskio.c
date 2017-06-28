/**
  ******************************************************************************
  * @file    spi_diskio.c
  * @author  Roman Stepanov
  * @version V1.3.0
  * @date    09-Dec-2016
  * @brief   SD Disk I/O driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "ff_gen_drv.h"
#include "spisd_diskio.h"

#ifdef BOOTLOADER
 #include "boot_conf.h"
#else
 #include "mks_conf.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Block Size in Bytes */
#define BLOCK_SIZE                512

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* Private function prototypes -----------------------------------------------*/
DSTATUS SPISD_initialize (BYTE);
DSTATUS SPISD_status (BYTE);
DRESULT SPISD_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
  DRESULT SPISD_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT SPISD_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

Diskio_drvTypeDef SPISD_Driver =
{
  SPISD_initialize,
  SPISD_status,
  SPISD_read,
#if  _USE_WRITE == 1
  SPISD_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  SPISD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Definitions for MMC/SDC command */

#define CMD0		(0x40+0)     /* GO_IDLE_STATE */
#define CMD1		(0x40+1)     /* SEND_OP_COND */
#define CMD8		(0x40+8)     /* SEND_IF_COND */
#define CMD9		(0x40+9)     /* SEND_CSD */
#define CMD10		(0x40+10)    /* SEND_CID */
#define CMD12		(0x40+12)    /* STOP_TRANSMISSION */
#define CMD16		(0x40+16)    /* SET_BLOCKLEN */
#define CMD17		(0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18		(0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23		(0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24		(0x40+24)    /* WRITE_BLOCK */
#define CMD25		(0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD41		(0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55		(0x40+55)    /* APP_CMD */
#define CMD58		(0x40+58)    /* READ_OCR */

/*--------------------------------------------------------------------------

 Module Private Functions

 ---------------------------------------------------------------------------*/

static BYTE CardType; /* b0:MMC, b1:SDC, b2:Block addressing */
static BYTE PowerFlag = 0; /* indicates if "power" is on */

static
void xmit_spi(BYTE Data)
{
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi1, &Data, 1, 5000);
}

static BYTE rcvr_spi(void)
{
	unsigned char Dummy, Data;
	Dummy = 0xFF;
	Data = 0;
	while ((HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY));
	HAL_SPI_TransmitReceive(&hspi1, &Dummy, &Data, 1, 5000);

	return Data;
}

static void rcvr_spi_m(BYTE *dst) {
	*dst = rcvr_spi();
}

/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------z------------------------------------------*/

static BYTE wait_ready(void) {

	BYTE res;
	uint16_t timer = 500;

	rcvr_spi();
	do {
		if ((res = rcvr_spi()) == 0xFF)
			break;

		osDelay(1);
	} while (timer--);

	return res;
}

/*-----------------------------------------------------------------------*/
/* Power Control  (Platform dependent)                                   */
/*-----------------------------------------------------------------------*/
/* When the target system does not support socket power control, there   */
/* is nothing to do in these functions and chk_power always returns 1.   */

static void power_on(void) {
	unsigned char i, cmd_arg[6];
	unsigned int Count = 0x1FFF;

	deviceDeselect();

	for (i = 0; i < 10; i++)
		xmit_spi(0xFF);

	deviceSelect(SPI_SDCARD);

	cmd_arg[0] = (CMD0 | 0x40);
	cmd_arg[1] = 0;
	cmd_arg[2] = 0;
	cmd_arg[3] = 0;
	cmd_arg[4] = 0;
	cmd_arg[5] = 0x95;

	for (i = 0; i < 6; i++)
		xmit_spi(cmd_arg[i]);

	while ((rcvr_spi() != 0x01) && Count)
		Count--;

	deviceDeselect();
	xmit_spi(0xff);

	PowerFlag = 1;
}

static void power_off(void) {

	PowerFlag = 0;
	Stat = STA_NOINIT;
}

static int chk_power(void)  {/* Socket power state: 0=off, 1=on */

	return PowerFlag;
}

/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE == 1

static uint8_t xmit_datablock(const BYTE *buff, BYTE token) {

	BYTE resp = 0, wc;
	uint32_t i = 0;

	if (wait_ready() != 0xFF)
		return 0;

	xmit_spi(token); /* Xmit data token */
	if (token != 0xFD) { /* Is data token */
		wc = 0;
		do { /* Xmit the 512 byte data block to MMC */
			xmit_spi(*buff++);
			xmit_spi(*buff++);
		} while (--wc);

		rcvr_spi();
		rcvr_spi();

		while (i <= 64) {
			resp = rcvr_spi(); /* Receive data response */
			if ((resp & 0x1F) == 0x05) /* If not accepted, return with error */
				break;
			i++;
		}
		while (rcvr_spi() == 0) ;
	}

	if ((resp & 0x1F) == 0x05)
		return 1;

	return 0;
}
#endif /* _USE_WRITE == 1 */

/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static uint8_t rcvr_datablock(BYTE *buff, UINT btr) {

	BYTE token;
	BYTE timer = 100;
	do { /* Wait for data packet in timeout of 100ms */

		if ((token = rcvr_spi()) != 0xff) {
			break;
		}

		osDelay(1);
	} while (timer--);

	if (token != 0xfe)
		return 0; /* If not valid data token, return with error */

	do { /* Receive the data block into buffer */
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
	} while (btr -= 2);
	rcvr_spi(); /* Discard CRC */
	rcvr_spi();

	return 1; /* Return with success */
}

/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static BYTE send_cmd(BYTE cmd, /* Command byte */
DWORD arg /* Argument */
) {
	BYTE n, res;

	if (wait_ready() != 0xFF)
		return 0xFF;

	/* Send command packet */
	xmit_spi(cmd); /* Command */
	xmit_spi((BYTE) (arg >> 24)); /* Argument[31..24] */
	xmit_spi((BYTE) (arg >> 16)); /* Argument[23..16] */
	xmit_spi((BYTE) (arg >> 8)); /* Argument[15..8] */
	xmit_spi((BYTE) arg); /* Argument[7..0] */
	n = 0;
	if (cmd == CMD0)
		n = 0x95; /* CRC for CMD0(0) */
	if (cmd == CMD8)
		n = 0x87; /* CRC for CMD8(0x1AA) */
	xmit_spi(n);

	/* Receive command response */
	if (cmd == CMD12)
		rcvr_spi(); /* Skip a stuff byte when stop reading */
	n = 10; /* Wait for a valid response in timeout of 10 attempts */
	do
		res = rcvr_spi();
	while ((res & 0x80) && --n);

	return res; /* Return with the response value */
}

/*--------------------------------------------------------------------------

 Public Functions

 ---------------------------------------------------------------------------*/
/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SPISD_initialize(BYTE drv /* Physical drive nmuber (0) */
) {
	BYTE n, ty, ocr[4];

	if (drv)
		return STA_NOINIT; /* Supports only single drive */
	if (Stat & STA_NODISK)
		return Stat; /* No card in the socket */

	power_on(); /* Force socket power on */
	//send_initial_clock_train();

	deviceSelect(SPI_SDCARD); /* CS = L */
	ty = 0;

	if (send_cmd(CMD0, 0) == 1) { /* Enter Idle state */

		uint16_t timer = 1000; /* Initialization timeout of 1000 msec */

		if (send_cmd(CMD8, 0x1AA) == 1) { /* SDC Ver2+ */

			for (n = 0; n < 4; n++)
				ocr[n] = rcvr_spi();

			if (ocr[2] == 0x01 && ocr[3] == 0xAA) { /* The card can work at vdd range of 2.7-3.6V */

				do {

					if (send_cmd(CMD55, 0) <= 1	&& send_cmd(CMD41, 1UL << 30) == 0)
						break; /* ACMD41 with HCS bit */

					osDelay(1);
				} while (timer--);


				if (timer && send_cmd(CMD58, 0) == 0) { /* Check CCS bit */

					for (n = 0; n < 4; n++)
						ocr[n] = rcvr_spi();

					ty = (ocr[0] & 0x40) ? 6 : 2;
				}
			}
		} else { /* SDC Ver1 or MMC */
			ty = (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 0) <= 1) ? 2 : 1; /* SDC : MMC */

			do {
				if (ty == 2) {
					if (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 0) == 0)
						break; /* ACMD41 */
				} else {
					if (send_cmd(CMD1, 0) == 0)
						break; /* CMD1 */
				}

				osDelay(1);
			} while (timer--);

			if (!timer || send_cmd(CMD16, 512) != 0) /* Select R/W block length */
				ty = 0;
		}
	}
	CardType = ty;
	deviceDeselect(); /* CS = H */
	rcvr_spi(); /* Idle (Release DO) */

	if (ty) /* Initialization succeded */
		Stat &= ~STA_NOINIT; /* Clear STA_NOINIT */
	else
		/* Initialization failed */
		power_off();

	return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SPISD_status(BYTE lun)
{
	if (lun)
		return STA_NOINIT; /* Supports only single drive */

	return Stat;
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT SPISD_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
	if (pdrv || !count)
		return RES_PARERR;
	if (Stat & STA_NOINIT) {

        if (SPISD_initialize(pdrv))
		return RES_NOTRDY;
	}

	if (!(CardType & 4))
		sector *= 512; /* Convert to byte address if needed */

	deviceSelect(SPI_SDCARD); /* CS = L */

	if (count == 1) { /* Single block read */

		if ((send_cmd(CMD17, sector) == 0) /* READ_SINGLE_BLOCK */
				&& rcvr_datablock(buff, 512))
			count = 0;

	} else { /* Multiple block read */

		if (send_cmd(CMD18, sector) == 0) { /* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512))
					break;

				buff += 512;
			} while (--count);

			send_cmd(CMD12, 0); /* STOP_TRANSMISSION */
		}
	}

	deviceDeselect(); /* CS = H */
	rcvr_spi(); /* Idle (Release DO) */

	return count ? RES_ERROR : RES_OK;
}

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1

DRESULT SPISD_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
	if (pdrv || !count)
		return RES_PARERR;
	if (Stat & STA_NOINIT) {

        if (SPISD_initialize(pdrv))
		return RES_NOTRDY;
    }

	if (Stat & STA_PROTECT)
		return RES_WRPRT;

	if (!(CardType & 4))
		sector *= 512; /* Convert to byte address if needed */

	deviceSelect(SPI_SDCARD); /* CS = L */

	if (count == 1) { /* Single block write */

		if ((send_cmd(CMD24, sector) == 0) /* WRITE_BLOCK */
				&& xmit_datablock(buff, 0xFE))
			count = 0;

	} else { /* Multiple block write */

		if (CardType & 2) {
			send_cmd(CMD55, 0);
			send_cmd(CMD23, count); /* ACMD23 */
		}

		if (send_cmd(CMD25, sector) == 0) { /* WRITE_MULTIPLE_BLOCK */

			do {
				if (!xmit_datablock(buff, 0xFC))
					break;
				buff += 512;
			} while (--count);

			if (!xmit_datablock(0, 0xFD)) /* STOP_TRAN token */
				count = 1;
		}
	}

	deviceDeselect(); /* CS = H */
	rcvr_spi(); /* Idle (Release DO) */

	return count ? RES_ERROR : RES_OK;
}

#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT SPISD_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
	DRESULT res;
	BYTE n, csd[16], *ptr = buff;
	WORD csize;

	if (drv)
		return RES_PARERR;

	res = RES_ERROR;

	if (ctrl == CTRL_POWER) {
		switch (*ptr) {
		case 0: /* Sub control code == 0 (POWER_OFF) */
			if (chk_power())
				power_off(); /* Power off */
			res = RES_OK;
			break;
		case 1: /* Sub control code == 1 (POWER_ON) */
			power_on(); /* Power on */
			res = RES_OK;
			break;
		case 2: /* Sub control code == 2 (POWER_GET) */
			*(ptr + 1) = (BYTE) chk_power();
			res = RES_OK;
			break;
		default:
			res = RES_PARERR;
		}
	} else {
		if (Stat & STA_NOINIT) {

            if (SPISD_initialize(drv))
			return RES_NOTRDY;
		}

		deviceSelect(SPI_SDCARD); /* CS = L */

		switch (ctrl) {
		case GET_SECTOR_COUNT: /* Get number of sectors on the disk (DWORD) */

			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {

				if ((csd[0] >> 6) == 1) { /* SDC ver 2.00 */

					csize = csd[9] + ((WORD) csd[8] << 8) + 1;
					*(DWORD*) buff = (DWORD) csize << 10;

				} else { /* MMC or SDC ver 1.XX */

					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
					csize = (csd[8] >> 6) + ((WORD) csd[7] << 2) + ((WORD) (csd[6] & 3) << 10) + 1;
					*(DWORD*) buff = (DWORD) csize << (n - 9);

				}
				res = RES_OK;
			}
			break;

		case GET_SECTOR_SIZE: /* Get sectors on the disk (WORD) */
			*(WORD*) buff = 512;
			res = RES_OK;
			break;

		case CTRL_SYNC: /* Make sure that data has been written */
			if (wait_ready() == 0xFF)
				res = RES_OK;
			break;

		case MMC_GET_CSD: /* Receive CSD as a data block (16 bytes) */
			if (send_cmd(CMD9, 0) == 0 /* READ_CSD */
					&& rcvr_datablock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_CID: /* Receive CID as a data block (16 bytes) */
			if (send_cmd(CMD10, 0) == 0 /* READ_CID */
					&& rcvr_datablock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_OCR: /* Receive OCR as an R3 resp (4 bytes) */
			if (send_cmd(CMD58, 0) == 0) { /* READ_OCR */
				for (n = 0; n < 4; n++)
					*ptr++ = rcvr_spi();
				res = RES_OK;
			}
			break;

		default:
			res = RES_PARERR;
			break;
		}

		deviceDeselect(); /* CS = H */
		rcvr_spi(); /* Idle (Release DO) */
	}

	return res;
}
#endif /* _USE_IOCTL == 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

