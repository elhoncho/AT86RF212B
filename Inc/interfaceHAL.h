/*
 * terminalHAL.h
 *
 *  Created on: Feb 12, 2018
 *      Author: owner
 */

#ifndef MYINC_TERMINALHAL_H_
#define MYINC_TERMINALHAL_H_

#include <stdint.h>

void InterfaceWriteHAL(char *txStr);
uint16_t InterfacePopFromInputBufferHAL();
uint8_t InterfacePushToInputBufferHAL(char rxChar);
void SetEchoInput(uint8_t condition);

#endif /* MYINC_TERMINALHAL_H_ */
