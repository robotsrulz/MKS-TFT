/**
  ******************************************************************************
  * @file    usbh_diskio.h
  * @author  Roman Stepanov
  * @version V1.3.0
  * @date    09-Dec-2016
  * @brief   USBH I/O driver
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

#ifndef __USBH_DISKIO_H
#define __USBH_DISKIO_H

#include "ff_gen_drv.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "ffconf.h"
#include "usbh_diskio.h"
#include "usbh_msc.h"

extern USBH_HandleTypeDef hUsbHostFS;

/*
 * FatFs interface
 *
 * */

extern Diskio_drvTypeDef USBH_Driver;

#endif /* __USBH_DISKIO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

