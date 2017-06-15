/*
 * Reset.h
 *
 * Created: 07/11/2015 11:46:58
 *  Author: David
 */


#ifndef RESET_H_
#define RESET_H_

#include "stm32f1xx_hal.h"

// Restart the hardware
inline void Restart()
{
	NVIC_SystemReset();
}

#endif /* RESET_H_ */
