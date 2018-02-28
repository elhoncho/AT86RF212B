/*
 * rawMode.c
 *
 *  Created on: Feb 26, 2018
 *      Author: owner
 */

#include "interfaceHAL.h"
#include "AT86RF212B.h"
#include <AT86RF212B_Settings.h>

#if STM32
#include<usbd_cdc.h>
#endif
void RawModeOpen(){
	SetEchoInput(0);
}


#define MAX_PACKET 125
void RawModeMain(){
	static uint8_t txData[MAX_PACKET];
	uint8_t i;
	uint8_t tmpChar;
	for(i = 0; i < MAX_PACKET; i++){
		uint8_t bufferStatus = InterfacePopFromInputBufferHAL(&tmpChar);
		if(bufferStatus == 0){
			//Buffer empty
			if(i){
				//Break out of loop and send txData
				break;
			}
			else{
				return;
			}
		}
		else if(bufferStatus == 1){
			if(i < MAX_PACKET-2){
				txData[i] = tmpChar;
			}
			else if(i == MAX_PACKET-1){
				//Buffer not empty
				//Send current packet length is i+1 because i is at a 0 offset
				AT86RF212B_TxData(txData, i+1);
				//Continue clearing buffer
				RawModeMain();
			}
		}
	}

	AT86RF212B_TxData(txData, i);
#if STM32
	CDC_Enable_USB_Packet();
#endif
	return;
}
