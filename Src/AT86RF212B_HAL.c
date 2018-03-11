/*
 * AT86RF212B_HAL.c
 *
 *  Created on: Feb 15, 2018
 *      Author: owner
 */

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "AT86RF212B.h"
#include "generalHAL.h"
#include "errors_and_logging.h"
#include "AT86RF212B_Settings.h"
#include "terminal.h"
#include "AT86RF212B_HAL.h"

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

#define SPI_CLK 8000000
#define SPI_CHANNEL 1

void HAL_Callback();

#endif

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
			LOG(LOG_LVL_ERROR, "Unknown Pin");
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
			LOG(LOG_LVL_ERROR, "Unknown Pin");
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

void AT86RF212B_CloseHAL(){

}


//Register = register to read
//pTxData = pointer to the data to send
//pRxValue = pointer to rx data array
//size = amount of data to be sent and received
void AT86RF212B_ReadAndWriteHAL(uint8_t * pTxData, uint8_t * pRxData, uint16_t size){
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

void AT86RF212B_StartReadAndWriteHAL(uint8_t * pTxData, uint8_t * pRxData, uint16_t size){
#if RASPBERRY_PI
	digitalWrite(SPI_NSS_PIN, LOW);
	wiringPiSPIDataRW(SPI_CHANNEL, pTxData, size);
	memcpy(pRxData, pTxData, size);
#endif

#if STM32
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi , pTxData, pRxData, size, timeout);
	//TODO: This probably needs to be changed, could lock up here.
	while(hspi.State == HAL_SPI_STATE_BUSY);
#endif
}

void AT86RF212B_ContinueReadAndWriteHAL(uint8_t * pTxData, uint8_t * pRxData, uint16_t size){
#if RASPBERRY_PI
	wiringPiSPIDataRW(SPI_CHANNEL, pTxData, size);
	memcpy(pRxData, pTxData, size);
#endif

#if STM32
	HAL_SPI_TransmitReceive(&hspi , pTxData, pRxData, size, timeout);
	//TODO: This probably needs to be changed, could lock up here.
	while(hspi.State == HAL_SPI_STATE_BUSY);
#endif
}

void AT86RF212B_StopReadAndWriteHAL(uint8_t * pTxData, uint8_t * pRxData, uint16_t size){
#if RASPBERRY_PI
	if(size > 0){
		wiringPiSPIDataRW(SPI_CHANNEL, pTxData, size);
	}
	digitalWrite(SPI_NSS_PIN, HIGH);
	memcpy(pRxData, pTxData, size);
#endif

#if STM32
	if(size > 0){
		HAL_SPI_TransmitReceive(&hspi , pTxData, pRxData, size, timeout);
		//TODO: This probably needs to be changed, could lock up here.
		while(hspi.State == HAL_SPI_STATE_BUSY);
	}
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_SET);
#endif
}

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

void AT86RF212B_Delay(uint8_t time, AT86RF212B_Config config){
	switch(time){
		case AT86RF212B_t7:
			//t7 	SLP_TR pulse width
			//    62.5 ns
			GeneralDelayUs(1);
			break;
		case AT86RF212B_t8:
			//t8 	SPI idle time: SEL rising to falling edge
			//    250 ns
			GeneralDelayUs(1);
			break;
		case AT86RF212B_t8a:
			//t8a 	SPI idle time: SEL rising to falling edge
			//    500 ns
			GeneralDelayUs(1);
			break;
		case AT86RF212B_t9:
			//t9 	SCLK rising edge LSB to /SEL rising edge
			//    250 ns
			GeneralDelayUs(1);
			break;
		case AT86RF212B_t10:
			//t10 	Reset pulse width
			//    625 ns
			GeneralDelayUs(1);
			break;
		case AT86RF212B_t12:
			//t12 	AES core cycle time
			//    24 탎
			GeneralDelayUs(24);
			break;
		case AT86RF212B_t13:
			//t13 	Dynamic frame buffer protection: IRQ latency
			//    750 ns
			GeneralDelayUs(1);
			break;
		case AT86RF212B_tTR1:
			//tTR1 	State transition from P_ON until CLKM is available
			//    330 탎

			//However the datasheet (7.1.4.1) says 420 탎 typical and 1ms max
			GeneralDelayMs(1);
			break;
		case AT86RF212B_tTR2:
			//tTR2 	State transition from SLEEP to TRX_OFF
			//    380 탎
			GeneralDelayUs(380);
			break;
		case AT86RF212B_tTR3:
			//tTR3 	State transition from TRX_OFF to SLEEP
			//    35 CLKM cycles

			//TODO: Implement this better
			GeneralDelayUs(2);
			break;
		case AT86RF212B_tTR4:
			//tTR4 	State transition from TRX_OFF to PLL_ON
			//    110 탎
			GeneralDelayUs(110);
			break;
		case AT86RF212B_tTR5:
			//tTR5 	State transition from PLL_ON to TRX_OFF
			//    1 탎
			GeneralDelayUs(1);
			break;
		case AT86RF212B_tTR6:
			//tTR6 	State transition from TRX_OFF to RX_ON
			//    110 탎
			GeneralDelayUs(110);
			break;
		case AT86RF212B_tTR7:
			//tTR7 	State transition from RX_ON to TRX_OFF
			//    1 탎
			GeneralDelayUs(1);
			break;
		case AT86RF212B_tTR8:
			//tTR8 	State transition from PLL_ON to RX_ON
			//    1 탎
			GeneralDelayUs(1);
			break;
		case AT86RF212B_tTR9:
			//tTR9 	State transition from RX_ON to PLL_ON
			//    1 탎
			GeneralDelayUs(1);
			break;
		case AT86RF212B_tTR10:
			//tTR10 	State transition from PLL_ON to BUSY_TX
			//    1 symbol

			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					GeneralDelayUs(50);
					break;
				case AT86RF212B_BPSK_40:
					GeneralDelayUs(25);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					GeneralDelayUs(40);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					GeneralDelayUs(16);
					break;
				default:
					ASSERT(0);
					LOG(LOG_LVL_ERROR, "Unknown Phy Mode");
					break;
			}

			break;
		case AT86RF212B_tTR12:
			//tTR12 	Transition from all states to TRX_OFF
			//    1 탎
			GeneralDelayUs(1);
			break;
		case AT86RF212B_tTR13:
			//tTR13 	State transition from RESET to TRX_OFF
			//    26 탎
			GeneralDelayUs(26);
			break;
		case AT86RF212B_tTR14:
			//tTR14 	Transition from various states to PLL_ON
			//    1 탎
			GeneralDelayUs(1);
			break;
		case AT86RF212B_tTR16:
			//tTR16 	FTN calibration time
			//    25 탎
			GeneralDelayUs(25);
			break;
		case AT86RF212B_tTR20:
			//tTR20 	PLL settling time on channel switch
			//    11 탎
			GeneralDelayUs(11);
			break;
		case AT86RF212B_tTR21:
			//tTR21 	PLL CF calibration time
			//    8 탎
			GeneralDelayUs(8);
			break;
		case AT86RF212B_tTR25:
			//tTR25 	RSSI update interval
			//    32 탎 : BPSK20
			//    24 탎 : BPSK40
			//    8 탎 : OQPSK

			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					GeneralDelayUs(32);
					break;
				case AT86RF212B_BPSK_40:
					GeneralDelayUs(24);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_400:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					GeneralDelayUs(8);
					break;
				default:
					ASSERT(0);
					LOG(LOG_LVL_ERROR, "Unknown Phy Mode");
					break;
			}
			break;
		case AT86RF212B_tTR26:
			//tTR26 	ED measurement time
			//    8 symbol : Low Data Rate Mode (LDRM) and manual measurement in High Data Rate Mode (HDRM)
			//    2 symbol : automatic measurement in High Data Rate Mode (HDRM)

			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					//    8 symbol
					GeneralDelayUs(400);
					break;
				case AT86RF212B_BPSK_40:
					//    8 symbol
					GeneralDelayUs(200);
					break;
				case AT86RF212B_O_QPSK_100:
					//    8 symbol
					GeneralDelayUs(320);
					break;
				case AT86RF212B_O_QPSK_200:
					//    2 symbol
					GeneralDelayUs(80);
					break;
				case AT86RF212B_O_QPSK_250:
					//    8 symbol
					GeneralDelayUs(128);
					break;
				case AT86RF212B_O_QPSK_400:
					//    2 symbol
					GeneralDelayUs(80);
					break;
				case AT86RF212B_O_QPSK_500:
					//    2 symbol
					GeneralDelayUs(32);
					break;
				case AT86RF212B_O_QPSK_1000:
					//    2 symbol
					GeneralDelayUs(32);
					break;
				default:
					ASSERT(0);
					LOG(LOG_LVL_ERROR, "Unknown Phy Mode");
					break;
			}
			break;
		case AT86RF212B_tTR28:
			//tTR28 	CCA measurement time
			//    8 symbol
			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					GeneralDelayUs(400);
					break;
				case AT86RF212B_BPSK_40:
					GeneralDelayUs(200);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					GeneralDelayUs(320);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					GeneralDelayUs(128);
					break;
				default:
					ASSERT(0);
					LOG(LOG_LVL_ERROR, "Unknown Phy Mode");
					break;
			}
			break;
		case AT86RF212B_tTR29:
			//tTR29 	SR_RND_VALUE update time
			//    1 탎
			GeneralDelayUs(1);
			break;
		case AT86RF212B_tMSNC:
			//tMSNC 	Minimum time to synchronize to a preamble and receive an SFD
			//    2 symbol
			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					GeneralDelayUs(100);
					break;
				case AT86RF212B_BPSK_40:
					GeneralDelayUs(400);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					GeneralDelayUs(80);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					GeneralDelayUs(32);
					break;
			}
			break;
		case AT86RF212B_tFrame:
			//tMSNC 	Minimum time to synchronize to a preamble and receive an SFD
			//    2 symbol
			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					GeneralDelayMs(52);
					break;
				case AT86RF212B_BPSK_40:
					GeneralDelayMs(26);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					GeneralDelayMs(11);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					GeneralDelayUs(5);
					break;
				default:
					ASSERT(0);
					LOG(LOG_LVL_ERROR, "Unknown Phy Mode");
					break;
			}
			break;
		default:
			ASSERT(0);
			LOG(LOG_LVL_ERROR, "Unknown Time Mode");
			break;
			return;
	}
	return;
}
