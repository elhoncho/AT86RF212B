#include <stdint.h>
#include <stdio.h>
#include "AT86RF212B_HAL.h"
#include "AT86RF212B.h"
#include "AT86RF212B_Regesters.h"
#include "AT86RF212B_Constants.h"
#include "generalHAL.h"
#include "errors_and_logging.h"
#include "AT86RF212B_Settings.h"
#include <string.h>

#define MSKMODE_SHOW_INT
#define MSKMODE_DONT_SHOW_INT

//------------Private Function Prototypes----------------//
//TODO:These should be uncommented as they are static
//static void AT86RF212B_ID();
static void AT86RF212B_PowerOnReset();
//static void AT86RF212B_TRX_Reset();
static void 	AT86RF212B_StateMachineReset();
static void 	AT86RF212B_BitWrite(uint8_t reg, uint8_t mask, uint8_t pos, uint8_t value);
static void 	AT86RF212B_AES_Io (uint8_t mode, uint8_t cmd, uint8_t start, uint8_t *idata, uint8_t *odata);
static void 	AT86RF212B_AES_Read (uint8_t cmd, uint8_t *odata);
static void 	AT86RF212B_AES_Write (uint8_t cmd, uint8_t start, uint8_t *idata);
static void 	AT86RF212B_AES_Wrrd (uint8_t cmd, uint8_t start, uint8_t *idata, uint8_t *odata);
static uint8_t 	AT86RF212B_FrameLengthRead();
static uint8_t 	AT86RF212B_FrameReadBlm (uint8_t *d);
static void 	AT86RF212B_IrqInit ();
static void 	AT86RF212B_SramRead (uint8_t addr, uint8_t length, uint8_t *data);
static void 	AT86RF212B_Sramrite (uint8_t addr, uint8_t length, uint8_t *data);
static void 	AT86RF212B_SetPhyMode();
static void 	AT86RF212B_PhySetChannel();
static void 	PhyCalibrateFTN();
static void 	PhyCalibratePll();
static void 	AT86RF212B_WaitForIRQ(uint8_t expectedIRQ);
static uint8_t 	StateChangeCheck(uint8_t newState);
static void 	UpdateState();
static uint8_t 	IsStateActive();
static uint8_t 	IsStatePllActive();
static uint8_t 	IsStateRxBasic();
static uint8_t 	IsStateRxActive();
static uint8_t 	IsStateConfig();
static uint8_t 	IsStateTxBusy();
static uint8_t 	IsStateRxBusy();
static uint8_t 	IsStateBusy();
static uint8_t 	IsStateCmd();
static void 	AT86RF212B_CheckForIRQ();
static void AT86RF212B_FrameWrite(uint8_t * frame, uint8_t length);
//-----------External Variables--------------------//
extern uint8_t logging;


//------------Private Global Variables----------------//
static AT86RF212B_Config config;
static volatile uint8_t interupt = 0;

//==============================================================================================//
//                                       Public Functions                                       //
//==============================================================================================//

void AT86RF212B_Open(){
	//------------Power On Settings-------------//
	//Initial State
	config.state = P_ON;
	//Change this to set the RF mode
	config.phyMode = AT86RF212B_PHY_MODE;
	//scrambler configuration for O-QPSK_{400,1000}; values { 0: disabled, 1: enabled (default)}.
	config.scramen = AT86RF212B_SCRAMEN;
	//transmit signal pulse shaping for O-QPSK_{250,500,1000}; values {0 : half-sine filtering (default), 1 : RC-0.8 filtering}.
	config.rcen = AT86RF212B_RCEN;
	//Set the TX power level (Table 9-15) 0x03 = 0dBm
	config.txPower = AT86RF212B_TX_POWER;
	//Set the RX sensitivity RX threshold = RSSI_BAS_VAL + rxSensLvl * 3
	//rxSensLvl = 0 - 15, 0 = max sensitivity
	config.rxSensLvl = AT86RF212B_RX_SENSE_LVL;
	//Enable TX CRC generation 1 = on 0 = off
	config.txCrc = AT86RF212B_TX_CRC;
	//Address Filtering
	config.panId_7_0 = AT86RF212B_PAN_ID_7_0;
	config.panId_15_8 = AT86RF212B_PAN_ID_15_8;
	config.shortAddr_7_0 = AT86RF212B_SHORT_ADDR_7_0;
	config.shortAddr_15_8 = AT86RF212B_SHORT_ADDR_15_8;
	config.extAddr_7_0 = AT86RF212B_EXT_ADDR_7_0;
	config.extAddr_15_8 = AT86RF212B_EXT_ADDR_15_8;
	config.extAddr_23_16 = AT86RF212B_EXT_ADDR_23_16;
	config.extAddr_31_24 = AT86RF212B_EXT_ADDR_31_24;
	config.extAddr_39_32 = AT86RF212B_EXT_ADDR_39_32;
	config.extAddr_47_40 = AT86RF212B_EXT_ADDR_47_40;
	config.extAddr_55_48 = AT86RF212B_EXT_ADDR_55_48;
	config.extAddr_63_56 = AT86RF212B_EXT_ADDR_63_56;

	AT86RF212B_OpenHAL(1000);

	//Time to wait after power on
	AT86RF212B_DelayHAL(AT86RF212B_tTR1, config);
	//Run power on reset sequence
	AT86RF212B_PowerOnReset();

	AT86RF212B_SetPhyMode();

	AT86RF212B_PhySetChannel();

	//TODO: This needs to change to TRX_OFF
	PhyStateToRxOn();
}

void AT86RF212B_ISR_Callback(){
	interupt = 1;
}

void AT86RF212B_Main(){
	//Main State Machine
	switch(config.state){
		case P_ON:
			break;
		case TRX_OFF:
			PhyStateToRxOn();
			break;
		case SLEEP:
			break;
		case RX_ON:
			AT86RF212B_CheckForIRQ();
			break;
		case PLL_ON:
			PhyStateToRxOn();
			break;
		case BUSY_TX:
			break;
		case BUSY_RX:
			break;
		case RX_ON_NOCLK:
			break;
	}
}
//-------------------Primitive Functions from AT86RF212 Programming Manual----------------------//

uint8_t AT86RF212B_RegRead(uint8_t reg){
	uint8_t pRxData[2] = {0, 0};
	uint8_t pTxData[2] = {0, 0};

	//Set the MSB and MSB-1 of the 8 bit register to a 1 0 for read access
	reg |= 1 << 7;
	reg &= ~(1 << 6);
	pTxData[0] = reg;
	AT86RF212B_ReadAndWriteHAL(pTxData, pRxData, 2);
	//First byte is a configurable status and the 2nd byte is the register value
	return pRxData[1];
}

uint8_t AT86RF212B_RegWrite(uint8_t reg, uint8_t value){
	uint8_t pRxData[2] = {0, 0};
	uint8_t pTxData[2] = {0, 0};

	//Set the MSB and MSB-1 of the 8 bit register to a 1 1 for write access
	reg |= 1 << 7;
	reg |= 1 << 6;
	pTxData[0] = reg;
	pTxData[1] = value;
	AT86RF212B_ReadAndWriteHAL(pTxData, pRxData, 2);

	return pRxData[1];
}

void AT86RF212B_TxData(uint8_t * frame, uint8_t length){
	if(config.state != PLL_ON){
		PhyStateToPllOn();
	}

	if(config.state == PLL_ON){
		if(length > 128){
			ASSERT(0);
			if(logging){
				LOG(LOG_LVL_ERROR, "Frame Too Large");
			}
			return;
		}
		//TODO: The speed of transmission can be improved by not waiting for all the data to be written before starting the TX phase
		AT86RF212B_FrameWrite(frame, length);

		AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_HIGH);
		AT86RF212B_DelayHAL(AT86RF212B_t7, config);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_LOW);
		AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);
		PhyStateToPllOn();
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function\r\n");
		}
	}
}

//==============================================================================================//
//                                     Private Functions                                        //
//==============================================================================================//

//-------------------Primitive Functions from AT86RF212 Programming Manual----------------------//

static void AT86RF212B_BitWrite(uint8_t reg, uint8_t mask, uint8_t pos, uint8_t value){
	uint8_t pRxData[2] = {0, 0};
	uint8_t pTxData[2] = {0, 0};

	uint8_t currentValue = AT86RF212B_RegRead(reg);

	//Set the MSB and MSB-1 of the 8 bit register to a 1 1 for write access
	reg |= 1 << 7;
	reg |= 1 << 6;
	pTxData[0] = reg;

	pTxData[1] = (currentValue & (~mask)) | (value << pos);
	AT86RF212B_ReadAndWriteHAL(pTxData, pRxData, 2);

	return;
}

uint8_t AT86RF212B_BitRead (uint8_t addr, uint8_t mask, uint8_t pos){
	uint8_t bitMask = 1 << pos;

	uint8_t currentValue = AT86RF212B_RegRead(addr);

	if(currentValue & bitMask){
		return 1;
	}
	else{
		return 0;
	}
}

static void 	AT86RF212B_AES_Io (uint8_t mode, uint8_t cmd, uint8_t start, uint8_t *idata, uint8_t *odata){

}
static void 	AT86RF212B_AES_Read (uint8_t cmd, uint8_t *odata){

}
static void 	AT86RF212B_AES_Write (uint8_t cmd, uint8_t start, uint8_t *idata){

}
static void 	AT86RF212B_AES_Wrrd (uint8_t cmd, uint8_t start, uint8_t *idata, uint8_t *odata){

}
static uint8_t 	AT86RF212B_FrameReadBlm (uint8_t *d){

}

static void 	AT86RF212B_SramRead (uint8_t addr, uint8_t length, uint8_t *data){

}
static void 	AT86RF212B_Sramrite (uint8_t addr, uint8_t length, uint8_t *data){

}

static uint8_t 	AT86RF212B_FrameLengthRead(){
	uint8_t pTxData[2] = {0x20, 0};
	uint8_t pRxData[2] = {0};
	AT86RF212B_ReadAndWriteHAL(pTxData, pRxData, 2);
	return pRxData[1];
}

uint8_t	AT86RF212B_FrameRead(){
	//TODO: Add CRC code
	//if(AT86RF212B_BitRead(SR_RX_CRC_VALID)){
	uint8_t length = AT86RF212B_FrameLengthRead();
	if(length <= 128){
		uint8_t pTxData[132] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
		uint8_t pRxData[132] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

		pTxData[0] = 0x20;

		AT86RF212B_ReadAndWriteHAL(pTxData, pRxData, length+5);
		if(logging){
			//First char is the PHY_STATUS bit
			LOG(LOG_LVL_DEBUG, (char *)&pRxData[2]);
		}
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Frame too large\r\n");
		}
	}
	return AT86RF212B_FrameLengthRead();
}

static void AT86RF212B_FrameWrite(uint8_t * frame, uint8_t length){
	uint8_t pTxData[length+2];
	uint8_t pRxData[7] = {0};

	pTxData[0] = 0x60;
	pTxData[1] = length;
	memcpy(&pTxData[2], frame, length);

	AT86RF212B_ReadAndWriteHAL(pTxData, pRxData, length+2);
}

static void 	AT86RF212B_IrqInit (){

	//Set IRQ Polarity to active high
	AT86RF212B_BitWrite(SR_IRQ_POLARITY, 0);
	//Enable Awake IRQ
	AT86RF212B_RegWrite(RG_IRQ_MASK, (TRX_IRQ_AWAKE_END | TRX_IRQ_PLL_LOCK | TRX_IRQ_TRX_END));
	//Only show enabled interrupts in the IRQ register
	AT86RF212B_BitWrite(SR_IRQ_MASK_MODE, 0);

}

//--------------------------Routines from AT86RF212 Programming Manual--------------------------//

static void AT86RF212B_PowerOnReset(){
	//The following programming sequence should be executed after power-on to
	//completely reset the radio transceiver. The MCU can not count on CLKM
	//before finalization of this sequence.

	/* AT86RF212::P_ON */
	if(config.state == P_ON){
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_LOW);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_RST, AT86RF212B_PIN_STATE_HIGH);
		DelayUs(400);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_RST, AT86RF212B_PIN_STATE_LOW);
		AT86RF212B_DelayHAL(AT86RF212B_t10, config);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_RST, AT86RF212B_PIN_STATE_HIGH);
		//Turn off CLKM clock (available as a clock reference if needed)
		AT86RF212B_RegWrite(RG_TRX_CTRL_0, 0x18);

		//*************Enable interrupts****************//
		//      These don't look like they work for     //
		//      the P_ON -> TRX_OFF transition          //
		//      but work after this transition          //
		//**********************************************//
		AT86RF212B_IrqInit();

		//Change to TRX_OFF state
		AT86RF212B_RegWrite(RG_TRX_STATE, CMD_FORCE_TRX_OFF);

		AT86RF212B_DelayHAL(AT86RF212B_tTR13, config);

		if(logging){
			char tmpStr[32];
			sprintf(tmpStr, "IRQ Mask Reg: 0x%02X\r\n", AT86RF212B_RegRead(RG_IRQ_MASK));
			LOG(LOG_LVL_DEBUG, tmpStr);
		}

		/* AT86RF212::TRX_OFF */
		StateChangeCheck(TRX_OFF);
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function\r\n");
		}
	}
}

//TODO: This should be static
void AT86RF212B_ID(){
	config.partid = AT86RF212B_RegRead(RG_PART_NUM);
	config.version = AT86RF212B_RegRead(RG_VERSION_NUM);
	config.manid0 = AT86RF212B_RegRead(RG_MAN_ID_0);
	config.manid1 = AT86RF212B_RegRead(RG_MAN_ID_1);

	if(logging){
		char tmpStr[32];
		sprintf(tmpStr, "Part ID: 0x%02X\r\n", config.partid);
		LOG(LOG_LVL_DEBUG, tmpStr);
		sprintf(tmpStr, "Version: 0x%02X\r\n", config.version);
		LOG(LOG_LVL_DEBUG, tmpStr);
		sprintf(tmpStr, "ManID0:  0x%02X\r\n", config.manid0);
		LOG(LOG_LVL_DEBUG, tmpStr);
		sprintf(tmpStr, "ManID1:  0x%02X\r\n", config.manid1);
		LOG(LOG_LVL_DEBUG, tmpStr);
	}
}

//TODO: Should be static
void AT86RF212B_TRX_Reset(){
	//This routine will bring the radio transceiver into a known state,
	//e.g. in case of a fatal error. The use case assumes, that the
	//radio transceiver is in one of the [ACTIVE] states (any state except P_ON and SLEEP)
	//and will do a reset, so that all registers get initialized
	//with their default values.

	/* AT86RF212::[ACTIVE] */
	if(IsStateActive()){
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_RST, AT86RF212B_PIN_STATE_LOW);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_LOW);
		AT86RF212B_DelayHAL(AT86RF212B_t10, config);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_RST, AT86RF212B_PIN_STATE_HIGH);
		AT86RF212B_IrqInit();
		AT86RF212B_DelayHAL(AT86RF212B_tTR13, config);
		/* AT86RF212::TRX_OFF */
		StateChangeCheck(TRX_OFF);
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function\r\n");
		}
	}
}

void PhyStateToPllOn(){
	UpdateState();
	if(config.state == PLL_ON){
		return;
	}
	if(config.state == TRX_OFF){
		/* AT86RF212::TRX_OFF */
		AT86RF212B_BitWrite(SR_TRX_CMD, CMD_PLL_ON);
		AT86RF212B_WaitForIRQ(TRX_IRQ_PLL_LOCK);
		StateChangeCheck(PLL_ON);
	}
	else if(IsStatePllActive()){
		/* AT86RF212::[PLL_ACTIVE] */
		AT86RF212B_BitWrite(SR_TRX_CMD, CMD_PLL_ON);
		AT86RF212B_DelayHAL(AT86RF212B_tTR9, config);
		StateChangeCheck(PLL_ON);
	}
	else if(IsStateBusy()){
		/* AT86RF212::[BUSY] */
		AT86RF212B_BitWrite(SR_TRX_CMD, CMD_PLL_ON);
		AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);
		StateChangeCheck(PLL_ON);
	}
	else if(config.state == BUSY_RX_AACK){
		/* AT86RF212::BUSY_RX_AACK */
		AT86RF212B_BitWrite(SR_TRX_CMD, CMD_PLL_ON);
		AT86RF212B_DelayHAL(AT86RF212B_tFrame, config);
		StateChangeCheck(PLL_ON);
	}
	//TODO: May need to add a state to force to pll on, force to pll on is an unimplemented transition in the programmers guide
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function\r\n");
		}
	}
}

void PhyStateToRxOn(){
	UpdateState();
	//TODO:Remove this condition, was used for a terminal test
	if(config.state == RX_ON){
		return;
	}
	/* AT86RF212::TRX_OFF */
	if(config.state == TRX_OFF){
		AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_ON);
		AT86RF212B_DelayHAL(AT86RF212B_tTR6, config);
		StateChangeCheck(RX_ON);
	}
	 /* AT86RF212::PLL_ON */
	else if(config.state ==  PLL_ON){
		AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_ON);
		AT86RF212B_DelayHAL(AT86RF212B_tTR8, config);
		StateChangeCheck(RX_ON);
	}
	/* AT86RF212::BUSY_TX */
	else if(IsStateTxBusy()){
		AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_ON);
		AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);
		StateChangeCheck(RX_ON);
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function\r\n");
		}
	}
}

static void PhyCalibrateFTN(){
	/* AT86RF212::[CONFIG] */
	if(IsStateConfig()){
		AT86RF212B_BitWrite(SR_FTN_START, 1);
		AT86RF212B_DelayHAL(AT86RF212B_tTR16, config);
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function\r\n");
		}
	}
}

static void PhyCalibratePll(){
	/* AT86RF212::PLL_ON */
	if(config.state == PLL_ON){
		AT86RF212B_BitWrite(SR_PLL_DCU_START, 1);
		AT86RF212B_BitWrite(SR_PLL_CF_START, 1);;
		AT86RF212B_DelayHAL(AT86RF212B_tTR21, config);
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function\r\n");
		}
	}
}

static void AT86RF212B_PhySetChannel(){
	/* AT86RF212::TRX_OFF */
	if(config.state == TRX_OFF){
		//CC_BAND (Table 9-34) 5 for 902.02MHz - 927.5 MHz
		AT86RF212B_BitWrite(SR_CC_BAND, 5);
		//(9.8.2) Fc[MHz] = 906[MHz] + 2[MHz] x (k � 1), for k = 1, 2, ..., 10
		AT86RF212B_BitWrite(SR_CHANNEL, 0);
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function\r\n");
		}
	}
}

static void AT86RF212B_StateMachineReset(){
	/* AT86RF212::[ACTIVE] */
	if(IsStateActive()){
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_LOW);
		AT86RF212B_BitWrite(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
		AT86RF212B_DelayHAL(AT86RF212B_tTR12, config);
		/* AT86RF212::TRX_OFF */
		StateChangeCheck(TRX_OFF);
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function\r\n");
		}
	}
}

static void AT86RF212B_SetPhyMode(){
	/* AT86RF212::TRX_OFF */
	if(config.state == TRX_OFF){
		//TODO: Need to implement the exceptions for the gctxOffset for the OQPSK-RC-(100,200,400) cases (Table 9-15)
		switch(config.phyMode){
		case AT86RF212B_BPSK_20:
			config.useOQPSK = 0;
			config.submode = 0;
			config.OQPSK_Rate = 0;
			config.gctxOffset = 3;
			break;
		case AT86RF212B_BPSK_40:
			config.useOQPSK = 0;
			config.submode = 1;
			config.OQPSK_Rate = 0;
			config.gctxOffset = 3;
			break;
		case AT86RF212B_O_QPSK_100:
			config.useOQPSK = 1;
			config.submode = 0;
			config.OQPSK_Rate = 0;
			config.gctxOffset = 2;
			break;
		case AT86RF212B_O_QPSK_200:
			config.useOQPSK = 1;
			config.submode = 0;
			config.OQPSK_Rate = 1;
			config.gctxOffset = 2;
			break;
		case AT86RF212B_O_QPSK_250:
			config.useOQPSK = 1;
			config.submode = 0;
			config.OQPSK_Rate = 2;
			config.gctxOffset = 2;
			break;
		case AT86RF212B_O_QPSK_400:
			config.useOQPSK = 1;
			config.submode = 1;
			config.OQPSK_Rate = 0;
			config.gctxOffset = 2;
			break;
		case AT86RF212B_O_QPSK_500:
			config.useOQPSK = 1;
			config.submode = 1;
			config.OQPSK_Rate = 1;
			config.gctxOffset = 2;
			break;
		case AT86RF212B_O_QPSK_1000:
			config.useOQPSK = 1;
			config.submode = 1;
			config.OQPSK_Rate = 2;
			config.gctxOffset = 2;
			break;
		default:
			ASSERT(0);
			if(logging){
				LOG(LOG_LVL_ERROR, "Unknown Phy Configuration\r\n");
			}
			return;
		}

		AT86RF212B_BitWrite(SR_BPSK_OQPSK, config.useOQPSK);
		AT86RF212B_BitWrite(SR_SUB_MODE, config.submode);
		AT86RF212B_BitWrite(SR_OQPSK_DATA_RATE, config.OQPSK_Rate);
		AT86RF212B_BitWrite(SR_OQPSK_SCRAM_EN, config.scramen);
		AT86RF212B_BitWrite(SR_OQPSK_SUB1_RC_EN, config.rcen);
		AT86RF212B_BitWrite(SR_GC_TX_OFFS, config.gctxOffset);
		AT86RF212B_RegWrite(RG_PHY_TX_PWR, config.txPower);
		AT86RF212B_BitWrite(SR_RX_PDT_LEVEL, config.rxSensLvl);
		AT86RF212B_BitWrite(SR_TX_AUTO_CRC_ON, config.txCrc);
		AT86RF212B_RegWrite(RG_PAN_ID_0, config.panId_7_0);
		AT86RF212B_RegWrite(RG_PAN_ID_1, config.panId_15_8);
		AT86RF212B_RegWrite(RG_SHORT_ADDR_0, config.shortAddr_7_0);
		AT86RF212B_RegWrite(RG_SHORT_ADDR_1, config.shortAddr_15_8);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_0, config.extAddr_7_0);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_1, config.extAddr_15_8);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_2, config.extAddr_23_16);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_3, config.extAddr_31_24);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_4, config.extAddr_39_32);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_5, config.extAddr_47_40);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_6, config.extAddr_55_48);
		AT86RF212B_RegWrite(RG_IEEE_ADDR_7, config.extAddr_63_56);
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function\r\n");
		}
	}
	return;
}

//---------------------------------Custom Functions---------------------------------//

//TODO: Remove this it is just for testing
void AT86RF212B_TestSleep(){
	AT86RF212B_TRX_Reset();
	AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_HIGH);
	DelayMs(1);
	AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_LOW);

	AT86RF212B_WaitForIRQ(TRX_IRQ_AWAKE_END);
}

static void AT86RF212B_CheckForIRQ(){
	if(interupt){
		//Clear the interrupt flag
		interupt = 0;

		uint8_t irqState = AT86RF212B_RegRead(RG_IRQ_STATUS);

		if(logging){
			char tmpStr[20];
			sprintf(tmpStr, "IRQ Recieved: %d\r\n", irqState);
			LOG(LOG_LVL_ERROR, tmpStr);
		}
	}
	return;
}

static void AT86RF212B_WaitForIRQ(uint8_t expectedIRQ){
	//Max time in ms to wait for an IRQ before timing out
	//TODO: Change this delay, it is set to 1 sec so the USB can connect and display logging in time
	uint32_t maxTime = 1000;

	//TODO: What happens if the timer rolls
	uint32_t timeout = AT86RF212B_SysTickMsHAL()+maxTime;
	while(!interupt){
		if(AT86RF212B_SysTickMsHAL() > timeout){
			ASSERT(0);
			if(logging){
				LOG(LOG_LVL_ERROR, "Timeout while waiting for IRQ\r\n");
			}
			return;
		}
	}
	//Clear the interrupt flag
	interupt = 0;

	uint8_t irqState = AT86RF212B_RegRead(RG_IRQ_STATUS);

	if(!(irqState & expectedIRQ)){
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Something very strange happened\r\n");
		}
		AT86RF212B_WaitForIRQ(expectedIRQ);
	}
	else if(logging){
		LOG(LOG_LVL_DEBUG, "Expected IRQ Received!\r\n");
	}
}

static uint8_t StateChangeCheck(uint8_t newState){
	UpdateState();

	if(config.state != newState){
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "State Change Failed\r\n");
		}
		return 0;
	}
	else if(logging){
		LOG(LOG_LVL_DEBUG, "State Change Success!\r\n");
		return 1;
	}
}

static void UpdateState(){
	config.state = AT86RF212B_RegRead(RG_TRX_STATUS);
}

static uint8_t IsStateActive(){
//	any state except P_ON and SLEEP
	return ((config.state == P_ON) || (config.state == SLEEP)) ? 0: 1;
}

static uint8_t IsStatePllActive(){
//	RX_ON, PLL_ON, TX_ARET_ON, RX_AACK_ON
	return ((config.state == RX_ON) || (config.state == PLL_ON) || (config.state == TX_ARET_ON) || (config.state == RX_AACK_ON)) ? 1: 0;
}

static uint8_t IsStateRxBasic(){
//RX_ON, BUSY_RX
	return ((config.state == RX_ON) || (config.state == BUSY_RX)) ? 1: 0;
}

static uint8_t IsStateRxActive(){
//RX_ON, RX_AACK_ON
	return ((config.state == RX_ON) || (config.state == RX_AACK_ON)) ? 1: 0;
}

static uint8_t IsStateConfig(){
//	TRX_OFF, PLL_ON, TX_ARET_ON

//[CONFIG] can be extended to states
//RX_ON and RX_AACK_ON if it can be ensured,
//that the radio is not in [RX_BUSY], see also
//use case PREAMBLE_DETECTOR_DISABLE.
	return ((config.state == TRX_OFF) || (config.state == PLL_ON) || (config.state == TX_ARET_ON)) ? 1: 0;
}

static uint8_t IsStateTxBusy(){
//	BUSY_TX, BUSY_TX_ARET
	return ((config.state == BUSY_TX) || (config.state == BUSY_TX_ARET)) ? 1: 0;
}

static uint8_t IsStateRxBusy(){
//	BUSY_RX, BUSY_RX_AACK
	return ((config.state == BUSY_RX) || (config.state == BUSY_RX_AACK)) ? 1: 0;
}

static uint8_t IsStateBusy(){
//	[TX_BUSY], [RX_BUSY]
	return (IsStateTxBusy() || IsStateRxBusy()) ? 1: 0;
}

static uint8_t IsStateCmd(){
//TRX_OFF, RX_ON, PLL_ON, TX_ARET_ON, RX_AACK_ON
//	(all states which can be entered by writing to the SR_TRX_CMD sub register)
	return ((config.state == TRX_OFF) || (config.state == PLL_ON) || (config.state == TX_ARET_ON) || (config.state == RX_AACK_ON)) ? 1: 0;
}
