/*
 * AT86RF212B_HAL.h
 *
 *  Created on: Feb 15, 2018
 *      Author: owner
 */

#ifndef MYINC_AT86RF212B_HAL_H_
#define MYINC_AT86RF212B_HAL_H_

#include "AT86RF212B.h"

void AT86RF212B_OpenHAL(uint32_t time_out);
void AT86RF212B_SPIreadAndWriteHAL(uint8_t * pTxData, uint8_t * pRxValue, uint16_t size);
void AT86RF212B_WritePinHAL(uint8_t pin, uint8_t state);
uint8_t AT86RF212B_ReadPinHAL(uint8_t pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void AT86RF212B_FrameWriteHAL(uint8_t * pTxData, uint16_t size);
void AT86RF212B_FrameReadHAL(uint8_t * pRxData);
uint32_t HALGetMs();
uint32_t HALGetUs();
void HALDelayMs(uint32_t timeMs);
void HALDelayUs(uint32_t timeUs);
void ReadInputHAL();
void WriteToOutputHAL(uint8_t * pTxData, uint32_t length);
#endif /* MYINC_AT86RF212B_HAL_H_ */
