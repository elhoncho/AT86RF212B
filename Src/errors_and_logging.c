/*
 * logging.c
 *
 *  Created on: Feb 18, 2018
 *      Author: owner
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../Inc/errors_and_logging.h"
#include "../Inc/terminal.h"
#include "../Inc/AT86RF212B_HAL.h"

#define MIN_LOG_LVL LOG_LVL_INFO

uint8_t logging = 0;

void LOG(LOG_LVL lvl, uint8_t * message){
	if(lvl >= MIN_LOG_LVL){
		WriteToOutputHAL(message, strlen((char*)message));
	}
}

void AssertError(uint8_t * fileName, int lineNumber){
	uint8_t tmpStr[32];
	sprintf((char*)tmpStr, "Error in file :");
	LOG(LOG_LVL_ERROR, tmpStr);
	LOG(LOG_LVL_ERROR, fileName);
	sprintf((char*)tmpStr, "\r\nOn line: %d\r\n", lineNumber);
	LOG(LOG_LVL_ERROR, tmpStr);
	//exit(0);
}
