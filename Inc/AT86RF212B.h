/*
 * AT86RF212B.h
 *
 *  Created on: Feb 15, 2018
 *      Author: owner
 */

#ifndef MYINC_AT86RF212B_H_
#define MYINC_AT86RF212B_H_

#include <stdint.h>

typedef struct{
	//Current State
	uint8_t state;

	//What RF mode to operate in
	uint8_t phyMode;

	//ID vars
	uint8_t partid;
	uint8_t version;
	uint8_t manid0;
	uint8_t manid1;

	//RF mode vars
	uint8_t useOQPSK;
	uint8_t submode;
	uint8_t OQPSK_Rate;
	uint8_t scramen;
	uint8_t rcen;
	uint8_t gctxOffset;

	//TX Power
	uint8_t txPower;

	//RX Sensitivity
	uint8_t rxSensLvl;

	//Enable crc
	uint8_t txCrc;

	//Enable Rx Safe Mode
	uint8_t rxSafeMode;

	//Enables reserved frame types to be filtered just like normal frames
	uint8_t AACK_FLTR_RES_FT;

	//Enables reserved frame types to be processed
	uint8_t AACK_UPLD_RES_FT;

	//Enables the IRQ pin to be used as a frame buffer indicator during frame buffer reads
	uint8_t RX_BL_CTRL;

	//Reduces the ACK time from 12 symbol periods to 2 or 3 symbol periods depending on the TX mode
	uint8_t AACK_ACK_TIME;


	//Address Filter
	uint8_t panId_7_0;
	uint8_t panId_15_8;
	uint8_t shortAddr_7_0;
	uint8_t shortAddr_15_8;
	uint8_t extAddr_7_0;
	uint8_t extAddr_15_8;
	uint8_t extAddr_23_16;
	uint8_t extAddr_31_24;
	uint8_t extAddr_39_32;
	uint8_t extAddr_47_40;
	uint8_t extAddr_55_48;
	uint8_t extAddr_63_56;

	uint8_t maxFrameRetries;
	uint8_t CSMA_LBT_Mode;
	uint8_t maxCSMA_Retries;
	uint8_t minBe;
	uint8_t maxBe;
	uint8_t slottedOperatin;
	uint8_t AACK_I_AmCoord;
	uint8_t AACK_SetPd;

}AT86RF212B_Config;

enum pin{
	AT86RF212B_PIN_CLKM,
	AT86RF212B_PIN_IRQ,
	AT86RF212B_PIN_SLP_TR,
	AT86RF212B_PIN_RST,
	AT86RF212B_PIN_DIG2
};
enum state{
	AT86RF212B_PIN_STATE_LOW,
	AT86RF212B_PIN_STATE_HIGH
};

enum delayTimes{
  AT86RF212B_t7,
  AT86RF212B_t8,
  AT86RF212B_t8a,
  AT86RF212B_t9,
  AT86RF212B_t10,
  AT86RF212B_t12,
  AT86RF212B_t13,
  AT86RF212B_tTR1,
  AT86RF212B_tTR2,
  AT86RF212B_tTR3,
  AT86RF212B_tTR4,
  AT86RF212B_tTR5,
  AT86RF212B_tTR6,
  AT86RF212B_tTR7,
  AT86RF212B_tTR8,
  AT86RF212B_tTR9,
  AT86RF212B_tTR10,
  AT86RF212B_tTR12,
  AT86RF212B_tTR13,
  AT86RF212B_tTR14,
  AT86RF212B_tTR16,
  AT86RF212B_tTR20,
  AT86RF212B_tTR21,
  AT86RF212B_tTR25,
  AT86RF212B_tTR26,
  AT86RF212B_tTR28,
  AT86RF212B_tTR29,
  AT86RF212B_tMSNC,
  AT86RF212B_tFrame
};

enum PhyMode{
	AT86RF212B_BPSK_20,
	AT86RF212B_BPSK_40,
	AT86RF212B_O_QPSK_100,
	AT86RF212B_O_QPSK_200,
	AT86RF212B_O_QPSK_250,
	AT86RF212B_O_QPSK_400,
	AT86RF212B_O_QPSK_500,
	AT86RF212B_O_QPSK_1000,
};

//------------Public Function Prototypes----------------//
void AT86RF212B_Open();
void AT86RF212B_Main();
uint8_t AT86RF212B_RegRead(uint8_t reg);
uint8_t AT86RF212B_RegWrite(uint8_t reg, uint8_t value);
void AT86RF212B_ISR_Callback();
void AT86RF212B_ID();
uint8_t AT86RF212B_BitRead (uint8_t addr, uint8_t mask, uint8_t pos);
void AT86RF212B_FrameRead();
void AT86RF212B_PhyStateChange(uint8_t newState);
uint8_t AT86RF212B_GetState();
#endif /* MYINC_AT86RF212B_H_ */
