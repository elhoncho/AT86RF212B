/*
 * rawMode.c
 *
 *  Created on: Feb 26, 2018
 *      Author: owner
 */

#include "interfaceHAL.h"
#include "AT86RF212B.h"
#include "AT86RF212B_Constants.h"
#include "MainController.h"
#include "../Settings/AT86RF212B_Settings.h"
#include "../Settings/TerminalSettings.h"

#if STM32
#include<usbd_cdc_if.h>
#endif


void RawModeOpen(){
	SetEchoInput(ECHO_INPUT);
	switch(MainControllerGetMode()){
		case MODE_RAW_RX:
			AT86RF212B_PhyStateChange(RX_AACK_ON);
			break;
		case MODE_RAW_TX:
			AT86RF212B_PhyStateChange(TX_ARET_ON);
			break;
		case MODE_RAW_RX_TX:
			AT86RF212B_PhyStateChange(RX_AACK_ON);
			break;
	}
}


void RawModeMain(){

	static uint8_t txData[AT86RF212B_MAX_DATA];
	uint8_t i = 0;
	uint8_t tmpChar;
	for(i = 0; i < AT86RF212B_MAX_DATA; i++){
		uint8_t bufferStatus = InterfacePopFromInputBufferHAL(&tmpChar);
		if(bufferStatus == 0){
			//Buffer empty
			if(i){
				//Break out of loop and send txData
				break;
			}
			else{
				InterfaceReadInput();
				return;
			}
		}
		else if(bufferStatus == 1){
			if(i < AT86RF212B_MAX_DATA-1){
				txData[i] = tmpChar;
			}
			else if(i == AT86RF212B_MAX_DATA-1){
				//Buffer not empty
				//Send current packet length is i+1 because i is at a 0 offset
				AT86RF212B_TxData(txData, i+1);

				//Continue clearing buffer
				RawModeMain();
				return;
			}
		}
	}

	if(i){
		AT86RF212B_TxData(txData, i);
		if(MainControllerGetMode() == MODE_RAW_RX_TX){
			AT86RF212B_PhyStateChange(RX_AACK_ON);
		}
	}
	InterfaceReadInput();
	return;
}
