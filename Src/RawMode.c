/*
 * rawMode.c
 *
 *  Created on: Feb 26, 2018
 *      Author: owner
 */
#include "../Inc/AT86RF212B.h"
#include "../Inc/AT86RF212B_Constants.h"
#include "../Inc/MainController.h"
#include "../Inc/Buffer.h"
#include "../Inc/AT86RF212B_HAL.h"
#include "../Inc/ErrorsAndLogging.h"
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

#define CODE 0
#define TXDATA 1

void RawModeMain(){
	uint8_t tmpChar;
	static uint8_t radioMode;
    static uint8_t code[10];
    static uint8_t iter = 0;
    static uint8_t length = 0;
    //0 = waiting for command 1 = tx data
    static uint8_t mode = CODE;
    static uint16_t lenReceaved = 0;


    switch(mode){
		case CODE:
			while(PopFromInputBuffer(&tmpChar)){
				code[iter] = tmpChar;
				iter++;
				if(iter == 5){
					iter = 0;
					if(strncmp((char*)code, "TX", 2) == 0){
						mode = TXDATA;
						code[5] = '\0';
						length = atoi((char*)&code[2]);
						break;
					}
					else{
						//Clear Buffer
						while(PopFromInputBuffer(&tmpChar));
					}
				}
			}
			//No data to send so make sure the radio is in RX mode
			radioMode = MainControllerGetMode();
			if(radioMode == MODE_RAW_RX_TX){
				if(AT86RF212B_GetState() != RX_AACK_ON){
					AT86RF212B_PhyStateChange(RX_AACK_ON);
				}
			}
			break;
		case TXDATA:
			if(PopFromInputBuffer(&tmpChar)){
				PushToTxBuffer(tmpChar);
				lenReceaved ++;
			}
			if(lenReceaved == length){
				lenReceaved = 0;
				mode = CODE;
				//Switch radio to tx mode
				radioMode = MainControllerGetMode();
				if(radioMode == MODE_RAW_RX_TX || radioMode == MODE_RAW_TX){
					if(AT86RF212B_GetState() != TX_ARET_ON){
						AT86RF212B_PhyStateChange(TX_ARET_ON);
					}
				}
			}
			break;
    }

    if(PopFromRxBuffer(&tmpChar)){
    	WriteToOutputHAL((uint8_t*)"FRAME", 5);
		while(PopFromRxBuffer(&tmpChar)){
			WriteToOutputHAL(&tmpChar, 1);
		}
    }
	ReadInputHAL();
	return;
}
