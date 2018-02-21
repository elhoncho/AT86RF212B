/*
 * terminalHAL.c
 *
 *  Created on: Feb 12, 2018
 *      Author: owner
 */


#include <stdint.h>
#include "MyInc/terminal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

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

void terminalWriteHAL(char *txStr){
	if(hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED){
		//TODO: Fix this, can lock up here
		while(CDC_Transmit_HS((uint8_t*)txStr, strlen(txStr)) == USBD_BUSY);
	}
}

int8_t terminalReadRXCharHAL(){
	char rxChar;
	if(popFromBuffer(&rxBuffer, &rxChar) == 0){
		return rxChar;
	}
	else{
		return -1;
	}
}

void terminalWriteRXCharHAL(char rxChar){
	pushToBuffer(&rxBuffer, rxChar);

	char tmpStr[2] = {rxChar, '\0'};
	terminalWriteHAL(tmpStr);
	if(strcmp(tmpStr, "\r") == 0){
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
