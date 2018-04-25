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
#include "../Inc/Buffer.h"
#include "../Inc/AT86RF212B_HAL.h"
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
	uint8_t dataToSend = 0;
	uint8_t tmpChar;
	uint8_t radioMode;

	//Push data to txBuffer
	while(PopFromInputBuffer(&tmpChar)){
		//TODO: May cause loss of data if the pop is successful but the push fails
		PushToTxBuffer(tmpChar);
		dataToSend = 1;
	}

	radioMode = MainControllerGetMode();

	if(dataToSend){
		//If there is data to be sent switch radio to tx mode
		if(radioMode == MODE_RAW_RX_TX || radioMode == MODE_RAW_TX){
			if(AT86RF212B_GetState() != TX_ARET_ON){
				AT86RF212B_PhyStateChange(TX_ARET_ON);
			}
		}
	}
	//No data to be sent make sure the radio is in RX mode
	else if(radioMode == MODE_RAW_RX_TX){
		if(AT86RF212B_GetState() != RX_AACK_ON){
			AT86RF212B_PhyStateChange(RX_AACK_ON);
		}
	}

	ReadInputHAL();
	return;
}
