/*
 * rawMode.c
 *
 *  Created on: Feb 26, 2018
 *      Author: owner
 */

#include "interfaceHAL.h"
#include "AT86RF212B.h"

void RawModeOpen(){
	SetEchoInput(0);
}

void RawModeMain(){
	static uint8_t txData[128];
	uint8_t i;
	//Must be uint16_t so tath 256 is a valid value to return signifying InterfacePopFromInputBufferHAL had no results
	uint16_t tmpChar;
	for(i = 0; i < 128; i++){
		tmpChar = InterfacePopFromInputBufferHAL();
		if(tmpChar == 256){
			if(i){
				break;
			}
			else{
				return;
			}
		}
		else{
			txData[i] = tmpChar;
		}
	}

	AT86RF212B_TxData(txData, i);
	return;
}
