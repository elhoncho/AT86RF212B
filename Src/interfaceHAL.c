/*
 * terminalHAL.c
 *
 *  Created on: Feb 12, 2018
 *      Author: owner
 */


#include <stdint.h>
#include <stdio.h>
#include "terminal.h"
#include "AT86RF212B_Settings.h"

#if STM32
#include "usb_device.h"
#include "usbd_cdc_if.h"
static uint8_t echoInput = 1;
#endif

#if RASPBERRY_PI
static uint8_t echoInput = 0;
#endif

//Max amount of characters to buffer on the rx
#define BUFFER_LENGTH 25


struct circleBuffer{
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile char buffer[BUFFER_LENGTH];
};

extern uint8_t newCmd;

static struct circleBuffer rxBuffer;

static int8_t pushToBuffer(struct circleBuffer *b, const char inChar);
static int8_t popFromBuffer(struct circleBuffer *b, char *outChar);

void SetEchoInput(uint8_t condition){
	echoInput = condition;
}

void InterfaceWriteHAL(char *txStr){
#if RASPBERRY_PI
	printf("%s", txStr);
	fflush(stdout);
#endif

#if STM32
	if(hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED){
		if((strcmp(txStr, "\r") == 0) || (strcmp(txStr, "\n") == 0)){
			sprintf(txStr, "\r\n");
		}
		//TODO: Fix this, if too much data too fast can lock up here
		while(CDC_Transmit_HS((uint8_t*)txStr, strlen(txStr)) == USBD_BUSY);
	}
#endif
}

int8_t InterfacePopFromInputBufferHAL(){
	char rxChar;
	if(popFromBuffer(&rxBuffer, &rxChar) == 0){
		return rxChar;
	}
	else{
		return -1;
	}
}

void InterfacePushToInputBufferHAL(char rxChar){
	pushToBuffer(&rxBuffer, rxChar);

	char tmpStr[2] = {rxChar, '\0'};

	if(echoInput){
		InterfaceWriteHAL(tmpStr);
	}

	if(rxChar == '\r' || rxChar == '\n'){
		newCmd = 1;
	}
}

static int8_t pushToBuffer(struct circleBuffer *b, const char inChar){
    if(b->head == BUFFER_LENGTH-1){
        b->head = 0;
    }
    else{
        b->head++;
    }

    if(b->head != b->tail){
        b->buffer[b->head] = inChar;
        return 0;
    }
    else{
        //Make sure head and tail are not both 0
        if(b->head == 0){
            b->head = BUFFER_LENGTH-1;
        }
        else{
            b->head--;
        }
        return -1;
    }
}

static int8_t popFromBuffer(struct circleBuffer *b, char *outChar){
    if(b->tail != b->head){
        if(b->tail == BUFFER_LENGTH-1){
            b->tail = 0;
        }
        else{
            b->tail++;
        }

        *outChar = b->buffer[b->tail];
        return 0;
    }
    else{
        //Head equals tail, therefore nothing on the buffer
        return -1;
    }
}