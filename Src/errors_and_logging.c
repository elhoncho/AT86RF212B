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
#include "errors_and_logging.h"
#include "terminal.h"
#include "interfaceHAL.h"

#define MIN_LOG_LVL LOG_LVL_ERROR

uint8_t logging = 1;

void LOG(LOG_LVL lvl, char * message){
	if(lvl >= MIN_LOG_LVL){
		InterfaceWriteToLogHAL(message, strlen(message));
	}
}

void AssertError(char * fileName, int lineNumber){
	char tmpStr[32];
	sprintf(tmpStr, "Error in file :");
	LOG(LOG_LVL_ERROR, tmpStr);
	LOG(LOG_LVL_ERROR, fileName);
	sprintf(tmpStr, "\r\nOn line: %d\r\n", lineNumber);
	LOG(LOG_LVL_ERROR, tmpStr);
	//exit(0);
}
