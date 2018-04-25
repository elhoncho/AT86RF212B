#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include "../Inc/terminal.h"
#include "../Settings/AT86RF212B_Settings.h"
#include "../Inc/AT86RF212B_HAL.h"

static uint8_t echoInput = 0;
extern uint8_t newCmd;

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint8_t buffer[RX_BUFFER_LENGTH];
}circleBuffer;


static circleBuffer rxBuffer = {
	.head = 0,
	.tail = 0,
};

static uint8_t pushToBuffer(circleBuffer *b, const uint8_t inChar);
static uint8_t popFromBuffer(circleBuffer *b, uint8_t *outChar);

void SetEchoInput(uint8_t condition){
	echoInput = condition;
}

uint8_t PopFromRxBufferHAL(uint8_t* rxByte){
	return popFromBuffer(&rxBuffer, rxByte);
}

uint8_t PushToRxBufferHAL(char rxChar){
	if(pushToBuffer(&rxBuffer, rxChar) == 1){
		if(echoInput){
			uint8_t tmpStr[2] = {rxChar, '\0'};
			WriteToOutputHAL(tmpStr, 2);
		}

		if(rxChar == '\r' || rxChar == '\n'){
			newCmd = 1;
		}

		//Check if buffer is filling up
		//Buffer has not wrapped yet
		if(rxBuffer.head > rxBuffer.tail){
			if(rxBuffer.head - rxBuffer.tail <= AT86RF212B_MAX_DATA){
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
			if(((RX_BUFFER_LENGTH - rxBuffer.tail)+rxBuffer.head) <= AT86RF212B_MAX_DATA){
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

static uint8_t pushToBuffer(circleBuffer *b, const uint8_t inChar){
    if(b->head == RX_BUFFER_LENGTH-1){
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
            b->head = RX_BUFFER_LENGTH-1;
        }
        else{
            b->head--;
        }
        return 0;
    }
}

static uint8_t popFromBuffer(circleBuffer *b, uint8_t *outChar){
    if(b->tail != b->head){
        if(b->tail == RX_BUFFER_LENGTH-1){
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
