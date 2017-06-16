/*
 * Buzzer.cpp
 *
 * Created: 13/11/2014 22:56:18
 *  Author: David
 * The piezo sounder is connected to the complementary outputs of PWM channel 0, PWMH0 and PWML0, aka PB0 peripheral A and PB5 peripheral B.
 * The backlight control is included in this module because it also uses PWM. Output PWMH1 (aka PB1 peripheral A) drives the backlight pin.
 */

#include <cstring>

#include "ecv.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "Buzzer.h"

TimerHandle_t xBuzzerTimer;
extern TIM_HandleTypeDef htim2;

extern "C" void vBuzzerTimerCallback(TimerHandle_t xTimer) {

    xTimerStop(xBuzzerTimer, 10);
}

namespace Buzzer
{
	// Generate a beep of the given length and frequency. The volume goes from 0 to MaxVolume.
	void Beep(uint32_t ms)
	{
	    if (!xBuzzerTimer) {
            xBuzzerTimer = xTimerCreate("Buzzer", ms, false, (void *) 0, vBuzzerTimerCallback);
            HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	    }
	    else if ( !Noisy() )
        {
            xTimerChangePeriod(xBuzzerTimer, ms, 10);
            HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
        }
	}

	// Return true if the buzzer is (or should be) still sounding
	bool Noisy()
	{
		return !xBuzzerTimer ? false : (xTimerIsTimerActive(xBuzzerTimer) != pdFALSE);
	}

	void CheckStop()
	{
	    if ( !Noisy() )
            HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
	}
}

extern "C" void BuzzerCheckStop() {

    Buzzer::CheckStop();
}

// End
