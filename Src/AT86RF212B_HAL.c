/*
 * AT86RF212B_HAL.c
 *
 *  Created on: Feb 15, 2018
 *      Author: owner
 */

#include "stm32f4xx_hal.h"
#include "MyInc/AT86RF212B.h"
#include "MyInc/generalHAL.h"
#include "MyInc/errors_and_logging.h"

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

void AT86RF212B_WritePinHAL(uint8_t pin, uint8_t state){
	uint16_t GPIO_PIN;
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
	return;
}

uint8_t AT86RF212B_ReadPinHAL(uint8_t pin){
	uint16_t GPIO_PIN;
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
}

//TODO: Change the returns from void to an indicator, that means the functions need to validate a successful operation or not
void AT86RF212B_OpenHAL(uint32_t time_out){
	hspi = hspi3;
	timeout = time_out;
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SLP_TR_PORT, SLP_TR_PIN, GPIO_PIN_RESET);
}

void AT86RF212B_CloseHAL(){

}


//Register = register to read
//pTxData = pointer to the data to send
//pRxValue = pointer to rx data array
//size = amount of data to be sent and received
void AT86RF212B_RegReadAndWriteHAL(uint8_t * pTxData, uint8_t * pRxData, uint16_t size){
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi , pTxData, pRxData, size, timeout);
	//TODO: This probably needs to be changed, could lock up here.
	while(hspi.State == HAL_SPI_STATE_BUSY);
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_SET);
}

void AT86RF212B_FrameWriteHAL(uint8_t * pTxData, uint16_t size){
	uint8_t pRxData[128];
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_RESET);
	//Transmit the Command
	HAL_SPI_Transmit(&hspi , 0x20, 1, timeout);
	//Transmit the PHR
	HAL_SPI_Transmit(&hspi , &size, 1, timeout);
	HAL_SPI_TransmitReceive(&hspi , pTxData, pRxData, size, timeout);
	//TODO: This probably needs to be changed, could lock up here.
	while(hspi.State == HAL_SPI_STATE_BUSY);
	HAL_GPIO_WritePin(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_PIN_SET);
}

uint32_t AT86RF212B_SysTickMsHAL(){
	return HAL_GetTick();
}

//This function overrides the default callback for the STM32 HAL and its name should not be changed
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == IRQ_PIN){
		AT86RF212B_ISR_Callback();
	}
}
void AT86RF212B_DelayHAL(uint8_t time, AT86RF212B_Config config){
	switch(time){
		case AT86RF212B_t7:
			//t7 	SLP_TR pulse width
			//    62.5 ns
			DelayUs(1);
			break;
		case AT86RF212B_t8:
			//t8 	SPI idle time: SEL rising to falling edge
			//    250 ns
			DelayUs(1);
			break;
		case AT86RF212B_t8a:
			//t8a 	SPI idle time: SEL rising to falling edge
			//    500 ns
			DelayUs(1);
			break;
		case AT86RF212B_t9:
			//t9 	SCLK rising edge LSB to /SEL rising edge
			//    250 ns
			DelayUs(1);
			break;
		case AT86RF212B_t10:
			//t10 	Reset pulse width
			//    625 ns
			DelayUs(1);
			break;
		case AT86RF212B_t12:
			//t12 	AES core cycle time
			//    24 탎
			DelayUs(24);
			break;
		case AT86RF212B_t13:
			//t13 	Dynamic frame buffer protection: IRQ latency
			//    750 ns
			DelayUs(1);
			break;
		case AT86RF212B_tTR1:
			//tTR1 	State transition from P_ON until CLKM is available
			//    330 탎

			//However the datasheet (7.1.4.1) says 420 탎 typical and 1ms max
			DelayMs(1);
			break;
		case AT86RF212B_tTR2:
			//tTR2 	State transition from SLEEP to TRX_OFF
			//    380 탎
			DelayUs(380);
			break;
		case AT86RF212B_tTR3:
			//tTR3 	State transition from TRX_OFF to SLEEP
			//    35 CLKM cycles

			//TODO: Implement this better
			DelayUs(2);
			break;
		case AT86RF212B_tTR4:
			//tTR4 	State transition from TRX_OFF to PLL_ON
			//    110 탎
			DelayUs(110);
			break;
		case AT86RF212B_tTR5:
			//tTR5 	State transition from PLL_ON to TRX_OFF
			//    1 탎
			DelayUs(1);
			break;
		case AT86RF212B_tTR6:
			//tTR6 	State transition from TRX_OFF to RX_ON
			//    110 탎
			DelayUs(110);
			break;
		case AT86RF212B_tTR7:
			//tTR7 	State transition from RX_ON to TRX_OFF
			//    1 탎
			DelayUs(1);
			break;
		case AT86RF212B_tTR8:
			//tTR8 	State transition from PLL_ON to RX_ON
			//    1 탎
			DelayUs(1);
			break;
		case AT86RF212B_tTR9:
			//tTR9 	State transition from RX_ON to PLL_ON
			//    1 탎
			DelayUs(1);
			break;
		case AT86RF212B_tTR10:
			//tTR10 	State transition from PLL_ON to BUSY_TX
			//    1 symbol

			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					DelayUs(50);
					break;
				case AT86RF212B_BPSK_40:
					DelayUs(25);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					DelayUs(40);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					DelayUs(16);
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
			DelayUs(1);
			break;
		case AT86RF212B_tTR13:
			//tTR13 	State transition from RESET to TRX_OFF
			//    26 탎
			DelayUs(26);
			break;
		case AT86RF212B_tTR14:
			//tTR14 	Transition from various states to PLL_ON
			//    1 탎
			DelayUs(1);
			break;
		case AT86RF212B_tTR16:
			//tTR16 	FTN calibration time
			//    25 탎
			DelayUs(25);
			break;
		case AT86RF212B_tTR20:
			//tTR20 	PLL settling time on channel switch
			//    11 탎
			DelayUs(11);
			break;
		case AT86RF212B_tTR21:
			//tTR21 	PLL CF calibration time
			//    8 탎
			DelayUs(8);
			break;
		case AT86RF212B_tTR25:
			//tTR25 	RSSI update interval
			//    32 탎 : BPSK20
			//    24 탎 : BPSK40
			//    8 탎 : OQPSK

			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					DelayUs(32);
					break;
				case AT86RF212B_BPSK_40:
					DelayUs(24);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_400:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					DelayUs(8);
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
					DelayUs(400);
					break;
				case AT86RF212B_BPSK_40:
					//    8 symbol
					DelayUs(200);
					break;
				case AT86RF212B_O_QPSK_100:
					//    8 symbol
					DelayUs(320);
					break;
				case AT86RF212B_O_QPSK_200:
					//    2 symbol
					DelayUs(80);
					break;
				case AT86RF212B_O_QPSK_250:
					//    8 symbol
					DelayUs(128);
					break;
				case AT86RF212B_O_QPSK_400:
					//    2 symbol
					DelayUs(80);
					break;
				case AT86RF212B_O_QPSK_500:
					//    2 symbol
					DelayUs(32);
					break;
				case AT86RF212B_O_QPSK_1000:
					//    2 symbol
					DelayUs(32);
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
					DelayUs(400);
					break;
				case AT86RF212B_BPSK_40:
					DelayUs(200);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					DelayUs(320);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					DelayUs(128);
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
			DelayUs(1);
			break;
		case AT86RF212B_tMSNC:
			//tMSNC 	Minimum time to synchronize to a preamble and receive an SFD
			//    2 symbol
			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					DelayUs(100);
					break;
				case AT86RF212B_BPSK_40:
					DelayUs(400);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					DelayUs(80);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					DelayUs(32);
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
