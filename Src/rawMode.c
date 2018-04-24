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
#include "errors_and_logging.h"
#include "../Settings/AT86RF212B_Settings.h"
#include "../Settings/TerminalSettings.h"

#if STM32
#include<usbd_cdc_if.h>
#endif

#define MAX_CONTINUOSU_CLEAR 5


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

	uint8_t txData[AT86RF212B_MAX_DATA];
	uint8_t i = 0;
	uint8_t tmpChar;
	uint8_t keepReading = 1;
	static uint8_t roundsStarved = 0;

	//Pull Data off buffer
	for(i = 0; i < AT86RF212B_MAX_DATA; i++){
		uint8_t bufferStatus = InterfacePopFromInputBufferHAL(&tmpChar);

		//Buffer is empty
		if(bufferStatus == 0){
			keepReading = 0;
			break;
		}
		//Buffer is filling up
		else if(bufferStatus == 1){
			txData[i] = tmpChar;
		}
	}

	if(i){
		AT86RF212B_TxData(txData, i, 0);
	}

	if(keepReading){
		if(roundsStarved < MAX_CONTINUOSU_CLEAR){
			RawModeMain();
			roundsStarved++;
			return;
		}
		else{
			roundsStarved = 0;
		}
	}

	//Change state depending on mode
	if(MainControllerGetMode() == MODE_RAW_RX_TX){
		if(AT86RF212B_GetState() != RX_AACK_ON){
			AT86RF212B_PhyStateChange(RX_AACK_ON);
		}
	}
	InterfaceReadInput();
	return;
}
