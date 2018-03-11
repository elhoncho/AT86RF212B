/*
 * terminalHAL.h
 *
 *  Created on: Feb 12, 2018
 *      Author: owner
 */

#ifndef MYINC_TERMINALHAL_H_
#define MYINC_TERMINALHAL_H_

#include <stdint.h>

void InterfaceWriteToLogHAL(uint8_t *txStr, uint16_t length);
void InterfaceWriteToDataOutputHAL(uint8_t * pTxData, uint32_t length);
uint8_t InterfacePopFromInputBufferHAL();
uint8_t InterfacePushToInputBufferHAL(char rxChar);
void SetEchoInput(uint8_t condition);
void InterfaceReadInput();

#endif /* MYINC_TERMINALHAL_H_ */
