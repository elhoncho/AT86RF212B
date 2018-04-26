/*
 * AT86RF212B_HAL.c
 *
 *  Created on: Feb 15, 2018
 *      Author: owner
 */

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include "../Inc/AT86RF212B.h"
#include "../Inc/AT86RF212B_HAL.h"

#include "../Inc/ErrorsAndLogging.h"
#include "../Inc/Terminal.h"
#include "../Inc/Buffer.h"
#include "../Settings/AT86RF212B_Settings.h"
#include "../Settings/HAL_Settings.h"

#if STM32
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"

#define SPI_NSS_PORT GPIOG
#define SPI_NSS_PIN GPIO_PIN_15

#define CLKM_PORT GPIOG
#define CLKM_PIN GPIO_PIN_14

#define IRQ_PORT GPIOG
#define IRQ_PIN GPIO_PIN_13

#define SLP_TR_PORT GPIOG
#define SLP_TR_PIN GPIO_PIN_12

#define RST_PORT GPIOG
#define RST_PIN GPIO_PIN_11

#define DIG2_PORT GPIOG
#define DIG2_PIN GPIO_PIN_10

uint32_t timeout = 1000;
extern SPI_HandleTypeDef hspi3;

SPI_HandleTypeDef hspi;
#endif

#if RASPBERRY_PI
#include "wiringPi.h"
#include "wiringPiSPI.h"

#define SPI_NSS_PIN 3
#define CLKM_PIN 0
#define IRQ_PIN 6
#define SLP_TR_PIN 5
#define RST_PIN 7
#define DIG2_PIN 2

#define SPI_CLK 4000000
#define SPI_CHANNEL 1

void HAL_Callback();

#endif

//-----------------Implement changing of GPIO pin
void AT86RF212B_WritePinHAL(uint8_t pin, uint8_t state){
	uint16_t GPIO_PIN;

#if RASPBERRY_PI
	switch(pin){
		case AT86RF212B_PIN_CLKM:
			GPIO_PIN = CLKM_PIN;
			break;
		case AT86RF212B_PIN_IRQ:
			GPIO_PIN = IRQ_PIN;
			break;
		case AT86RF212B_PIN_SLP_TR:
			GPIO_PIN = SLP_TR_PIN;
			break;
		case AT86RF212B_PIN_RST:
			GPIO_PIN = RST_PIN;
			break;
		case AT86RF212B_PIN_DIG2:
			GPIO_PIN = DIG2_PIN;
			break;
		default:
			ASSERT(0);
			LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown Pin");
			return;
	}
	(state) ? digitalWrite(GPIO_PIN, HIGH) : digitalWrite(GPIO_PIN, LOW);
#endif

#if STM32
	GPIO_TypeDef * GPIO_PORT;
	switch(pin){
		case AT86RF212B_PIN_CLKM:
			GPIO_PORT = GPIOG;
			GPIO_PIN = CLKM_PIN;
			break;
		case AT86RF212B_PIN_IRQ:
			GPIO_PORT = IRQ_PORT;
			GPIO_PIN = IRQ_PIN;
			break;
		case AT86RF212B_PIN_SLP_TR:
			GPIO_PORT = SLP_TR_PORT;
			GPIO_PIN = SLP_TR_PIN;
			break;
		case AT86RF212B_PIN_RST:
			GPIO_PORT = RST_PORT;
			GPIO_PIN = RST_PIN;
			break;
		case AT86RF212B_PIN_DIG2:
			GPIO_PORT = DIG2_PORT;
			GPIO_PIN = DIG2_PIN;
			break;
		default:
			ASSERT(0);
			LOG(LOG_LVL_ERROR, "Unknown Pin");
			return;
	}
	(state) ? HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN, GPIO_PIN_RESET);
#endif
	return;
}

//-----------------Implement reading GPIO pin
uint8_t AT86RF212B_ReadPinHAL(uint8_t pin){
	uint16_t GPIO_PIN;

#if RASPBERRY_PI
	switch(pin){
		case AT86RF212B_PIN_CLKM:
			GPIO_PIN = CLKM_PIN;
			break;
		case AT86RF212B_PIN_IRQ:
			GPIO_PIN = IRQ_PIN;
			break;
		case AT86RF212B_PIN_SLP_TR:
			GPIO_PIN = SLP_TR_PIN;
			break;
		case AT86RF212B_PIN_RST:
			GPIO_PIN = RST_PIN;
			break;
		case AT86RF212B_PIN_DIG2:
			GPIO_PIN = DIG2_PIN;
			break;
		default:
			ASSERT(0);
			LOG(LOG_LVL_ERROR, (uint8_t*)"Unknown Pin");
			return 0;
	}
	return digitalRead(GPIO_PIN);
#endif

#if STM32
	GPIO_TypeDef * GPIO_PORT;
	switch(pin){
		case AT86RF212B_PIN_CLKM:
			GPIO_PORT = GPIOG;
			GPIO_PIN = CLKM_PIN;
			break;
		case AT86RF212B_PIN_IRQ:
			GPIO_PORT = IRQ_PORT;
			GPIO_PIN = IRQ_PIN;
			break;
		case AT86RF212B_PIN_SLP_TR:
			GPIO_PORT = SLP_TR_PORT;
			GPIO_PIN = SLP_TR_PIN;
			break;
		case AT86RF212B_PIN_RST:
			GPIO_PORT = RST_PORT;
			GPIO_PIN = RST_PIN;
			break;
		case AT86RF212B_PIN_DIG2:
			GPIO_PORT = DIG2_PORT;
			GPIO_PIN = DIG2_PIN;
			break;
		default:
			ASSERT(0);
			LOG(LOG_LVL_ERROR, "Unknown Pin");
			return 0;
	}
	return HAL_GPIO_ReadPin(GPIO_PORT, GPIO_PIN);
#endif
}

//-----------------Implement platform specific initialization
//TODO: Change the returns from void to an indicator, that means the functions need to validate a successful operation or not
void AT86RF212B_OpenHAL(uint32_t time_out){

#if RASPBERRY_PI
	wiringPiSetup ();

	pinMode(SPI_NSS_PIN, OUTPUT);
	pinMode(CLKM_PIN, INPUT);
	pinMode(IRQ_PIN, INPUT);
	pinMode(SLP_TR_PIN, OUTPUT);
	pinMode(RST_PIN, OUTPUT);
	pinMode(DIG2_PIN, INPUT);

	digitalWrite(SPI_NSS_PIN, HIGH);
	digitalWrite(SLP_TR_PIN, LOW);

	wiringPiSPISetup(SPI_CHANNEL, SPI_CLK);

	wiringPiISR(IRQ_PIN, INT_EDGE_RISING, &HAL_Callback);
#endif

#if STM32
	hspi = hspi3;
	timeout = time_out;
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SLP_TR_PORT, SLP_TR_PIN, GPIO_PIN_RESET);
#endif
}


//-----------------Implement SPI read/write
void AT86RF212B_SPIreadAndWriteHAL(uint8_t * pTxData, uint8_t * pRxData, uint16_t size){
//Register = register to read
//pTxData = pointer to the data to send
//pRxValue = pointer to rx data array
//size = amount of data to be sent and received

#if RASPBERRY_PI
	digitalWrite(SPI_NSS_PIN, LOW);
	wiringPiSPIDataRW(SPI_CHANNEL, pTxData, size);
	digitalWrite(SPI_NSS_PIN, HIGH);
	memcpy(pRxData, pTxData, size);
#endif

#if STM32
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi , pTxData, pRxData, size, timeout);
	//TODO: This probably needs to be changed, could lock up here.
	while(hspi.State == HAL_SPI_STATE_BUSY);
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_SET);
#endif
}

//-----------------Implement Interrupt Callback, need to call AT86RF212B_ISR_Callback() when an interrupt is detected
#if RASPBERRY_PI
void HAL_Callback(){
	AT86RF212B_ISR_Callback();
}
#endif

#if STM32
//This function overrides the default callback for the STM32 HAL and its name should not be changed
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == IRQ_PIN){
		AT86RF212B_ISR_Callback();
	}
}
#endif

//-----------------Implement mili sec count
uint32_t HALGetMs(uint32_t timeMs){
#if RASPBERRY_PI
	return millis();
#endif

#if STM32
	return HAL_GetTick();
#endif
}

//-----------------Implement micro sec count
uint32_t HALGetUs(uint32_t timeUs){
#if RASPBERRY_PI
	return micros();
#endif

#if STM32
	//Start the counter
	DWT->CTRL |= 1;
	return DWT->CYCCNT;
#endif
}

//-----------------Implement mili sec delay
void HALDelayMs(uint32_t timeMs){
#if RASPBERRY_PI
	delay(timeMs);
#endif

#if STM32
	HAL_Delay(timeMs);
#endif
	return;
}

//-----------------Implement micro sec delay
void HALDelayUs(uint32_t timeUs){
#if RASPBERRY_PI
	delayMicroseconds(timeUs);
#endif

#if STM32
	//Clear the counter
	DWT->CYCCNT = 0;
	uint32_t stopTime = timeUs*(HAL_RCC_GetHCLKFreq()/1000000);
	//Start the counter
	DWT->CTRL |= 1;
	while(DWT->CYCCNT < stopTime);
#endif
	return;
}

//-----------------Implement reading from hardware data input to the TX buffer
void ReadInputHAL(){
#if RASPBERRY_PI
	char inChar;
	while(read(0, &inChar, 1) > 0){
		PushToInputBuffer(inChar);
	}
#endif

#if STM32
	CDC_Enable_USB_Packet();
#endif
}

//-----------------Implement writing RX buffer to the hardware output
void WriteToOutputHAL(uint8_t * pTxData, uint32_t length){
	#if RASPBERRY_PI
//	static uint8_t prevData[256];

//	if(memcmp(prevData, pTxData) == 0){
//		fwrite(prevData, sizeof(uint8_t), length, stdout);
//	}
//	memcpy(prevData, pTxData, length);

	fwrite(pTxData, sizeof(uint8_t), length, stdout);
	fflush(stdout);
	#endif

	#if STM32
	if(hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED){
		while(CDC_Transmit_HS(pTxData, length) == USBD_BUSY);
	}
	#endif
}
