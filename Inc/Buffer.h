/*
 * Buffer.h
 *
 *  Created on: Apr 25, 2018
 *      Author: owner
 */

#ifndef AT86RF212B_INC_BUFFER_H_
#define AT86RF212B_INC_BUFFER_H_

uint8_t PopFromRxBufferHAL();
uint8_t PushToRxBufferHAL(char rxChar);
void SetEchoInput(uint8_t condition);

#endif /* AT86RF212B_INC_BUFFER_H_ */
