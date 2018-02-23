/*
 * generalHAL.c
 *
 *  Created on: Feb 17, 2018
 *      Author: owner
 */

#include <stdint.h>
#include "AT86RF212B_Settings.h"

#if RASPBERRY_PI
#include "wiringPi.h"
#endif

#if STM32
#include "stm32f4xx_hal.h"
#endif

void DelayMs(uint32_t timeMs){
#if RASPBERRY_PI
	delay(timeMs);
#endif

#if STM32
	HAL_Delay(timeMs);
#endif
	return;
}

void DelayUs(uint32_t timeUs){
#if RASPBERRY_PI
	delayMicroseconds(timeUs);
#endif

#if STM32
	//Clear the counter
	DWT->CYCCNT = 0;
	uint32_t stopTime = timeUs*(HAL_RCC_GetHCLKFreq()/1000000);
	//Start the counter
	DWT->CTRL |= 1;
	while(DWT->CYCCNT < stopTime);
	//Stop the counter
	DWT->CTRL |= 0;
#endif
	return;
}
