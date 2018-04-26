/*
 * logging.c
 *
 *  Created on: Feb 18, 2018
 *      Author: owner
 */

#include "../Inc/ErrorsAndLogging.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../Inc/AT86RF212B_HAL.h"
#include "../Inc/Terminal.h"

#define MIN_LOG_LVL LOG_LVL_INFO

static uint8_t logging = 0;

void LOG(LOG_LVL lvl, uint8_t * message){
	if(lvl >= MIN_LOG_LVL){
		WriteToOutputHAL(message, strlen((char*)message));
	}
}

void AssertError(char* fileName, int lineNumber){
	uint8_t tmpStr[32];
	sprintf((char*)tmpStr, "Error in file :");
	LOG(LOG_LVL_ERROR, tmpStr);
	LOG(LOG_LVL_ERROR, fileName);
	sprintf((char*)tmpStr, "\r\nOn line: %d\r\n", lineNumber);
	LOG(LOG_LVL_ERROR, tmpStr);
	//exit(0);
}

uint8_t IsLogging(){
	return logging;
}

void ToggleLogging(){
	if(logging){
		logging = 0;
	}
	else{
		logging = 1;
	}
}
