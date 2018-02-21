/*
 * AT86RF212B_HAL.h
 *
 *  Created on: Feb 15, 2018
 *      Author: owner
 */

#ifndef MYINC_AT86RF212B_HAL_H_
#define MYINC_AT86RF212B_HAL_H_

#include "MyInc/AT86RF212B.h"

void AT86RF212B_OpenHAL(uint32_t time_out);
void AT86RF212B_CloseHAL();
void AT86RF212B_RegReadAndWriteHAL(uint8_t * pTxData, uint8_t * pRxValue, uint16_t size);
void AT86RF212B_WritePinHAL(uint8_t pin, uint8_t state);
uint8_t AT86RF212B_ReadPinHAL(uint8_t pin);
void AT86RF212B_DelayHAL(uint8_t time, AT86RF212B_Config config);
uint32_t AT86RF212B_SysTickMsHAL();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void AT86RF212B_FrameWriteHAL(uint8_t * pTxData, uint16_t size);


#endif /* MYINC_AT86RF212B_HAL_H_ */
