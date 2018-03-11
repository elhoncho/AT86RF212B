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



uint32_t GeneralGetMs(uint32_t timeMs){
#if RASPBERRY_PI
	return millis();
#endif

#if STM32
	return HAL_GetTick();
#endif
}

uint32_t GeneralGetUs(uint32_t timeUs){
#if RASPBERRY_PI
	return micros();
#endif

#if STM32
	//Start the counter
	DWT->CTRL |= 1;
	return DWT->CYCCNT;
#endif
}

void GeneralDelayMs(uint32_t timeMs){
#if RASPBERRY_PI
	delay(timeMs);
#endif

#if STM32
	HAL_Delay(timeMs);
#endif
	return;
}

void GeneralDelayUs(uint32_t timeUs){
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
#endif
	return;
}
