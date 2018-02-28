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
#define BUFFER_LENGTH 512


struct circleBuffer{
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint8_t buffer[BUFFER_LENGTH];
};

extern uint8_t newCmd;

static struct circleBuffer rxBuffer;

static uint8_t pushToBuffer(struct circleBuffer *b, const char inChar);
static uint8_t popFromBuffer(struct circleBuffer *b, char *outChar);

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

uint8_t InterfacePopFromInputBufferHAL(uint8_t* rxByte){
	if(popFromBuffer(&rxBuffer, rxByte)){
		return 1;
	}
	else{
		//return a number beyond one byte range to indicate a failure
		return 0;
	}
}

uint8_t InterfacePushToInputBufferHAL(char rxChar){
	if(pushToBuffer(&rxBuffer, rxChar) == 1){
		char tmpStr[2] = {rxChar, '\0'};

		if(echoInput){
			InterfaceWriteHAL(tmpStr);
		}

		if(rxChar == '\r' || rxChar == '\n'){
			newCmd = 1;
		}
		if(rxBuffer.head > rxBuffer.tail){
			if(rxBuffer.head - rxBuffer.tail <= 127){
				//Buffer is fine
				return 1;
			}
			else{
				//Buffer filling up
				return 2;
			}
		}
		//Buffer wrapped around
		else{
			if(((BUFFER_LENGTH - rxBuffer.tail)+rxBuffer.head) <= 127){
				return 1;
			}
			else{
				//Buffer filling up
				return 2;
			}
		}
	}
	else{
		//Failed to push to buffer
		return 0;
	}
}

static uint8_t pushToBuffer(struct circleBuffer *b, const char inChar){
    if(b->head == BUFFER_LENGTH-1){
        b->head = 0;
    }
    else{
        b->head++;
    }

    if(b->head != b->tail){
        b->buffer[b->head] = inChar;
        return 1;
    }
    else{
        //Make sure head and tail are not both 0
        if(b->head == 0){
            b->head = BUFFER_LENGTH-1;
        }
        else{
            b->head--;
        }
        return 0;
    }
}

static uint8_t popFromBuffer(struct circleBuffer *b, char *outChar){
    if(b->tail != b->head){
        if(b->tail == BUFFER_LENGTH-1){
            b->tail = 0;
        }
        else{
            b->tail++;
        }

        *outChar = b->buffer[b->tail];
        return 1;
    }
    else{
        //Head equals tail, therefore nothing on the buffer
        return 0;
    }
}
