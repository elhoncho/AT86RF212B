/*
 * rawMode.c
 *
 *  Created on: Feb 26, 2018
 *      Author: owner
 */
#include "../Inc/AT86RF212B.h"
#include "../Inc/AT86RF212B_Constants.h"
#include "../Inc/MainController.h"
#include "../Inc/errors_and_logging.h"
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

	//Pull Data off buffer
	for(i = 0; i < AT86RF212B_MAX_DATA; i++){
		uint8_t bufferStatus = PopFromRxBufferHAL(&tmpChar);

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
		RawModeMain();
		return;
	}

	//Change state depending on mode
	if(MainControllerGetMode() == MODE_RAW_RX_TX){
		if(AT86RF212B_GetState() != RX_AACK_ON){
			AT86RF212B_PhyStateChange(RX_AACK_ON);
		}
	}
	ReadInputHAL();
	return;
}
