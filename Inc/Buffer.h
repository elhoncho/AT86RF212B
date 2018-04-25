/*
 * Buffer.h
 *
 *  Created on: Apr 25, 2018
 *      Author: owner
 */

#ifndef AT86RF212B_INC_BUFFER_H_
#define AT86RF212B_INC_BUFFER_H_

uint8_t PopFromRxBuffer();
uint8_t PushToRxBuffer(char rxChar);
uint8_t PopFromTxBuffer();
uint8_t PushToTxBuffer(char rxChar);
uint8_t PopFromInputBuffer();
uint8_t PushToInputBuffer(char rxChar);
void SetEchoInput(uint8_t condition);

#endif /* AT86RF212B_INC_BUFFER_H_ */
