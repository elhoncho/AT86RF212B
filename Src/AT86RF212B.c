#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "generalHAL.h"
#include "AT86RF212B.h"
#include "interfaceHAL.h"
#include "AT86RF212B_HAL.h"
#include "MainController.h"
#include "errors_and_logging.h"
#include "AT86RF212B_Settings.h"
#include "AT86RF212B_Regesters.h"
#include "AT86RF212B_Constants.h"

//TODO: *******NEED TO WRITE FUNCTIONALITY TO RECALIBRATE EVERY FIVE MINUITS*****************

#define BEACON_TX_INTERVAL 1000

//------------Private Function Prototypes----------------//
static void 	AT86RF212B_PowerOnReset();
static void 	AT86RF212B_BitWrite(uint8_t reg, uint8_t mask, uint8_t pos, uint8_t value);
static void 	AT86RF212B_IrqInit ();
static uint8_t 	AT86RF212B_FrameLengthRead();
static void 	AT86RF212B_SetPhyMode();
static void 	AT86RF212B_PhySetChannel();
static void 	AT86RF212B_WaitForIRQ(uint8_t expectedIRQ);
static uint8_t 	StateChangeCheck(uint8_t newState);
static void 	UpdateState();
static uint8_t 	IsStateActive();
static uint8_t 	IsStatePllActive();
static uint8_t 	IsStateTxBusy();
static uint8_t 	IsStateRxBusy();
static uint8_t 	IsStateBusy();
static uint8_t 	AT86RF212B_CheckForIRQ(uint8_t desiredIRQ);
static void 	AT86RF212B_FrameWrite(uint8_t * frame, uint8_t length, uint8_t sequenceNumber);
static void 	AT86RF212B_Delay(uint8_t time);
static void 	AT86RF212B_WrongStateError();
static void 	AT86RF212B_SetRegisters();
static void 	AT86RF212B_SendBeacon();
static void 	AT86RF212B_PrintBuffer(uint8_t nLength, uint8_t* pData);
static void 	AT86RF212B_SendACK(uint8_t sequenceNumber);
static uint8_t 	AT86RF212B_WaitForACK(uint8_t sequenceNumber);



//-----------External Variables--------------------//
extern uint8_t logging;
extern uint8_t AT86RF212B_Mode;

//------------Private Global Variables----------------//
static AT86RF212B_Config config;
static volatile uint8_t interupt = 0;
static uint32_t nextBeaconUpdate = 0;
static uint8_t beaconFalures;
static uint8_t ackReceived = 0;

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
	//Enables Rx Safe Mode
	config.rxSafeMode = AT86RF212B_RX_SAFE_MODE;
	config.AACK_UPLD_RES_FT = AT86RF212B_AACK_UPLD_RES_FT;
	config.AACK_FLTR_RES_FT = AT86RF212B_AACK_FLTR_RES_FT;
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
	config.maxFrameRetries = AT86RF212B_MAX_FRAME_RETRIES;
	config.CSMA_LBT_Mode = AT86RF212B_CSMA_LBT_MODE;
	config.maxCSMA_Retries = AT86RF212B_MAX_CSMA_RETRIES;
	config.minBe = AT86RF212B_MIN_BE;
	config.maxBe = AT86RF212B_MAX_BE;
	config.slottedOperatin = AT86RF212B_SLOTTED_OPERATION;
	config.AACK_I_AmCoord = AT86RF212B_AACK_I_AM_COORD;
	config.AACK_SetPd = AT86RF212B_AACK_SET_PD;

	AT86RF212B_OpenHAL(1000);

	//Time to wait after power on
	AT86RF212B_Delay(AT86RF212B_tTR1);

	AT86RF212B_PowerOnReset();
	AT86RF212B_SetRegisters();
}

static void AT86RF212B_SetRegisters(){
	if(logging){
		LOG(LOG_LVL_DEBUG, "Registers initiated\r\n");
	}
	AT86RF212B_SetPhyMode();
	AT86RF212B_PhySetChannel();
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
			AT86RF212B_PhyStateChange(RX_ON);
			break;
		case SLEEP:
			break;
		case RX_ON:
			//Check for Beacon
			if(AT86RF212B_SysTickMsHAL() > nextBeaconUpdate){
				if(logging){
					LOG(LOG_LVL_DEBUG, "Beacon Failed\r\n");
				}
				beaconFalures++;
				nextBeaconUpdate = AT86RF212B_SysTickMsHAL() + 2000;

				if(beaconFalures > 9){
					MainControllerSetMode(MODE_TERMINAL);
					beaconFalures = 0;
				}
			}
			if(AT86RF212B_CheckForIRQ(TRX_IRQ_TRX_END)){
				AT86RF212B_FrameRead();
			}
			break;
		case PLL_ON:
			//Send Beacon
			if(AT86RF212B_SysTickMsHAL() > nextBeaconUpdate){
				if(logging){
					LOG(LOG_LVL_DEBUG, "Sending Beacon\r\n");
				}
				AT86RF212B_SendBeacon();
				nextBeaconUpdate = AT86RF212B_SysTickMsHAL() + BEACON_TX_INTERVAL;
			}
			break;
		case TX_ARET_ON:
//			//Send Beacon
//			if(AT86RF212B_SysTickMsHAL() > nextBeaconUpdate){
//				if(logging){
//					LOG(LOG_LVL_DEBUG, "Sending Beacon\r\n");
//				}
//				AT86RF212B_SendBeacon();
//				nextBeaconUpdate = AT86RF212B_SysTickMsHAL() + BEACON_TX_INTERVAL;
//			}
			break;
		case BUSY_RX_AACK:
			break;
		case RX_AACK_ON:
//			//Check for Beacon
//			if(AT86RF212B_SysTickMsHAL() > nextBeaconUpdate){
//				if(logging){
//					LOG(LOG_LVL_DEBUG, "Beacon Failed\r\n");
//				}
//				beaconFalures++;
//				nextBeaconUpdate = AT86RF212B_SysTickMsHAL() + 2000;
//
//				if(beaconFalures > 9){
//					MainControllerSetMode(MODE_TERMINAL);
//					beaconFalures = 0;
//				}
//			}
//			//Careful if you chage this to AMI or the start of RX because you will need to check the FCF. If the FCF is not valid a TRX_END will not be generatedCRC Failed
//			if(AT86RF212B_CheckForIRQ(TRX_IRQ_TRX_END)){
//				//AT86RF212B_BitRead(SR_TRAC_STATUS);
//				AT86RF212B_FrameRead();
//			}
			break;
		case BUSY_TX:
			break;
		case BUSY_TX_ARET:
			break;
		case BUSY_RX:
			break;
		case RX_ON_NOCLK:
			break;
		default:
			ASSERT(0);
			if(logging){
				LOG(LOG_LVL_ERROR, "Unknown state, changing to RX_ON\r\n");
			}
			AT86RF212B_PhyStateChange(RX_ON);
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
static uint8_t AT86RF212B_WaitForACK(uint8_t sequenceNumber){
	//TODO: What happens when timer rolls
	uint32_t timeout = AT86RF212B_SysTickMsHAL() + 100;
	ackReceived = 0;
	AT86RF212B_PhyStateChange(RX_ON);
	while(ackReceived == 0){
		if(AT86RF212B_SysTickMsHAL() > timeout){
			return 0;
		}
		if(AT86RF212B_CheckForIRQ(TRX_IRQ_TRX_END)){
			AT86RF212B_FrameRead();
		}
	}
	AT86RF212B_PhyStateChange(PLL_ON);
	return 1;
}
static void AT86RF212B_SendACK(uint8_t sequenceNumber){
	UpdateState();
	if(config.state != PLL_ON){
		AT86RF212B_PhyStateChange(PLL_ON);
		AT86RF212B_SendACK(sequenceNumber);
	}
	else if(config.state == PLL_ON){
		if(logging){
			LOG(LOG_LVL_DEBUG, "Sending ACK\r\n");
		}

		//The length here has to be the length of the data and header plus 2 for the command and PHR plus 2 for the frame check sequence if enabled
		#if AT86RF212B_TX_CRC
			uint8_t nLength = 11;
		#else
			uint8_t nLength = 9;
		#endif

		uint8_t pRxData[nLength];

		//Frame write command
		uint8_t pTxData[11] = {0x60,
		//PHR (PHR is just the length of the data and header and does not include one for the command or one the PHR its self so it is nLength-2)
		nLength-2,
		//FCF !!!BE CAREFUL OF BYTE ORDER, MSB IS ON THE RIGHT IN THE DATASHEET!!!
		0x02,
		0x08,
		//Sequence number
		sequenceNumber,
		//Target PAN
		AT86RF212B_PAN_ID_7_0,
		AT86RF212B_PAN_ID_15_8,
		//Target ID
		AT86RF212B_SHORT_ADDR_TARGET_7_0,
		AT86RF212B_SHORT_ADDR_TARGET_15_8};

		AT86RF212B_ReadAndWriteHAL(pTxData, pRxData, nLength);

		AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_HIGH);
		AT86RF212B_Delay(AT86RF212B_t7);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_LOW);
		AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);

		AT86RF212B_PhyStateChange(RX_ON);
	}
}

static void AT86RF212B_SendBeacon(){
	UpdateState();
		if(config.state != PLL_ON){
			AT86RF212B_PhyStateChange(PLL_ON);
			AT86RF212B_SendBeacon();
		}
		else if(config.state == PLL_ON){
			//The length here has to be the length of the data and header plus 2 for the command and PHR plus 2 for the frame check sequence if enabled
			#if AT86RF212B_TX_CRC
				uint8_t nLength = 13;
			#else
				uint8_t nLength = 11;
			#endif

			uint8_t pRxData[nLength];

			//Frame write command
			uint8_t pTxData[11] = {0x60,
			//PHR (PHR is just the length of the data and header and does not include one for the command or one the PHR its self so it is nLength-2)
			nLength-2,
			//FCF !!!BE CAREFUL OF BYTE ORDER, MSB IS ON THE RIGHT IN THE DATASHEET!!!
			0x04,
			0x08,
			//Sequence number
			0x00,
			//Target PAN
			AT86RF212B_PAN_ID_7_0,
			AT86RF212B_PAN_ID_15_8,
			//Target ID
			AT86RF212B_SHORT_ADDR_TARGET_7_0,
			AT86RF212B_SHORT_ADDR_TARGET_15_8,
			'A',
			'A'};

			AT86RF212B_ReadAndWriteHAL(pTxData, pRxData, nLength);

			AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_HIGH);
			AT86RF212B_Delay(AT86RF212B_t7);
			AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_LOW);
			AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);
		}
}
//Length is the lengt of the data to send frame = 1234abcd length = 8, no adding to the length for the header that gets added later
void AT86RF212B_TxData(uint8_t * frame, uint8_t length){
	static uint8_t sequenceNumber = 0;
	uint8_t status;
	UpdateState();
	if(config.state != PLL_ON){
		AT86RF212B_PhyStateChange(PLL_ON);
		AT86RF212B_TxData(frame, length);
	}
	else if(config.state == PLL_ON){
		if(length > AT86RF212B_MAX_DATA){
			ASSERT(0);
			if(logging){
				LOG(LOG_LVL_ERROR, "Frame Too Large\r\n");
			}
			return;
		}
		else if(length == 0){
			ASSERT(0);
			if(logging){
				LOG(LOG_LVL_ERROR, "No data to send\r\n");
			}
			return;
		}
		//TODO: The speed of transmission can be improved by not waiting for all the data to be written before starting the TX phase
		AT86RF212B_FrameWrite(frame, length, sequenceNumber);

		AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_HIGH);
		AT86RF212B_Delay(AT86RF212B_t7);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_SLP_TR, AT86RF212B_PIN_STATE_LOW);
		//Wait untill done transmitting data
		//TODO: This may affect speed
		AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);

		if(AT86RF212B_WaitForACK(sequenceNumber)){
			if(logging){
				LOG(LOG_LVL_INFO, "Frame TX Success\r\n");
			}
			sequenceNumber++;
		}
		else{
			if(logging){
				LOG(LOG_LVL_DEBUG, "No ACK on frame, retransmitting\r\n");
			}
			AT86RF212B_TxData(frame, length);
		}
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
	uint8_t currentValue = AT86RF212B_RegRead(addr);
	currentValue = currentValue & mask;
	currentValue = currentValue >> pos;
	return currentValue;
}

static uint8_t 	AT86RF212B_FrameLengthRead(){
	uint8_t pTxData[2] = {0x20, 0};
	uint8_t pRxData[2] = {0};
	AT86RF212B_ReadAndWriteHAL(pTxData, pRxData, 2);
	return pRxData[1];
}

static void AT86RF212B_PrintBuffer(uint8_t nLength, uint8_t* pData) {
	char tmpStr[20];
	int i = 0;
	for (i = 0; i < nLength; i++) {
		if (pData[i] < 32 || pData[i] > 126) {
			sprintf(tmpStr, "0x%02X : \r\n", pData[i]);
		} else {
			sprintf(tmpStr, "0x%02X : %c\r\n", pData[i], pData[i]);
		}
		LOG(LOG_LVL_INFO, tmpStr);
	}
	LOG(LOG_LVL_INFO, "\r\n");
}

void AT86RF212B_FrameRead(){
	if(config.txCrc){
		if(!AT86RF212B_BitRead(SR_RX_CRC_VALID)){
			if(logging){
				LOG(LOG_LVL_DEBUG, "CRC Failed\r\n");
			}
			//Enable preamble detector to start receiving again
			AT86RF212B_BitWrite(SR_RX_PDT_DIS, 0);
			return;
		}
		else{
			if(logging){
				LOG(LOG_LVL_DEBUG, "CRC Passed\r\n");
			}
		}
	}

	uint8_t length = AT86RF212B_FrameLengthRead();
	if(length == 0){
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "No data on frame\r\n");
		}
		//Enable preamble detector to start receiving again
		AT86RF212B_BitWrite(SR_RX_PDT_DIS, 0);
		return;
	}
	else if(length > 127){
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Frame too large\r\n");
		}
		//Enable preamble detector to start receiving again
		AT86RF212B_BitWrite(SR_RX_PDT_DIS, 0);
		return;
	}
	else{
		if(logging){
			char tmpStr[20];
			sprintf(tmpStr, "Reading frame of size %i\r\n", length);
			LOG(LOG_LVL_DEBUG, tmpStr);
		}

		//Length received is the length of the data plus two bytes for the command and PRI bytes
		//add 3 to the length for the ED LQI and RX_STATUS bytes
		uint8_t nLength = length+3;
		uint8_t pTxData[nLength];
		uint8_t pRxData[nLength];

		pTxData[0] = 0x20;

		AT86RF212B_ReadAndWriteHAL(pTxData, pRxData, nLength);

		//Check if ACK is requested
		if(pRxData[2] & 0x20){
			AT86RF212B_SendACK(pRxData[3]);
		}

		//Check if it is a data frame
		if((pRxData[2] & 0x07) == 1){
			//Wait twice as long as beacons are being sent out
			nextBeaconUpdate = AT86RF212B_SysTickMsHAL()+BEACON_TX_INTERVAL+BEACON_TX_INTERVAL;
			beaconFalures = 0;

			//length - AT86RF212B_DATA_OFFSET (header bytes)
			uint8_t dataLength = length-AT86RF212B_DATA_OFFSET;
			uint8_t data[dataLength];
			memcpy(data, &pRxData[AT86RF212B_DATA_OFFSET], dataLength);
			InterfaceWriteToDataOutputHAL(data, dataLength);
		}
		//Check if it is a beacon frame
		else if((pRxData[2] & 0x07) == 4){
			//Wait twice as long as beacons are being sent out
			nextBeaconUpdate = AT86RF212B_SysTickMsHAL()+BEACON_TX_INTERVAL+BEACON_TX_INTERVAL;
			beaconFalures = 0;
		}
		//Check if it is an ACK
		else if((pRxData[2] & 0x07) == 2){
			//Wait twice as long as beacons are being sent out
			nextBeaconUpdate = AT86RF212B_SysTickMsHAL()+BEACON_TX_INTERVAL+BEACON_TX_INTERVAL;
			beaconFalures = 0;
			ackReceived = 1;
		}
		else{
			if(logging){
				ASSERT(0);
				LOG(LOG_LVL_ERROR, "Unknown Frame Type\r\n");
			}
		}

		//Enable preamble detector to start receiving again
		AT86RF212B_BitWrite(SR_RX_PDT_DIS, 0);
	}
	return;
}

static void AT86RF212B_FrameWrite(uint8_t * frame, uint8_t length, uint8_t sequenceNumber){
	//The length here has to be the length of the data and header plus 2 for the command and PHR plus 2 for the frame check sequence if enabled
#if AT86RF212B_TX_CRC
	uint8_t nLength = length+11;
#else
	uint8_t nLength = length+9;
#endif

	uint8_t pTxData[nLength];
	uint8_t pRxData[nLength];

	//Frame write command
	pTxData[0] = 0x60;

	//PHR (PHR is just the length of the data and header and does not include one for the command or one the PHR its self so it is nLength-2)
	pTxData[1] = nLength-2;

	//FCF !!!BE CAREFUL OF BYTE ORDER, MSB IS ON THE RIGHT IN THE DATASHEET!!!
	pTxData[2] = 0x21;
	pTxData[3] = 0x08;
	//Sequence number
	pTxData[4] = sequenceNumber;
	//Target PAN
	pTxData[5] = AT86RF212B_PAN_ID_7_0;
	pTxData[6] = AT86RF212B_PAN_ID_15_8;
	//Target ID
	pTxData[7] = AT86RF212B_SHORT_ADDR_TARGET_7_0;
	pTxData[8] = AT86RF212B_SHORT_ADDR_TARGET_15_8;

	//Source PAN
	//pTxData[9] = AT86RF212B_PAN_ID_7_0;
	//pTxData[10] = AT86RF212B_PAN_ID_15_8;
	//Source ID
	//pTxData[11] = AT86RF212B_SHORT_ADDR_7_0;
	//pTxData[12] = AT86RF212B_SHORT_ADDR_15_8;

	memcpy(&pTxData[AT86RF212B_DATA_OFFSET], frame, length);

	AT86RF212B_ReadAndWriteHAL(pTxData, pRxData, nLength);

	sequenceNumber += 1;

	if(logging){
		LOG(LOG_LVL_DEBUG, "\r\nData Sent: \r\n");
		AT86RF212B_PrintBuffer(nLength, pTxData);
	}
}

static void 	AT86RF212B_IrqInit (){

	//Set IRQ Polarity to active high
	AT86RF212B_BitWrite(SR_IRQ_POLARITY, 0);
	//Enable Awake IRQ
	//AT86RF212B_RegWrite(RG_IRQ_MASK, (TRX_IRQ_AWAKE_END | TRX_IRQ_PLL_LOCK | TRX_IRQ_TRX_END | TRX_IRQ_RX_START | TRX_IRQ_AMI));
	AT86RF212B_RegWrite(RG_IRQ_MASK, 0xFF);
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
		AT86RF212B_Delay(AT86RF212B_t10);
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

		AT86RF212B_Delay(AT86RF212B_tTR13);

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
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function: Resetting\r\n");
		}
		AT86RF212B_TRX_Reset();
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
		AT86RF212B_Delay(AT86RF212B_t10);
		AT86RF212B_WritePinHAL(AT86RF212B_PIN_RST, AT86RF212B_PIN_STATE_HIGH);
		AT86RF212B_IrqInit();
		AT86RF212B_Delay(AT86RF212B_tTR13);
		/* AT86RF212::TRX_OFF */
		StateChangeCheck(TRX_OFF);

		AT86RF212B_SetRegisters();
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function: This is real bad\r\n");
		}
	}
}

void AT86RF212B_PhyStateChange(uint8_t newState){
	UpdateState();
	if(config.state == P_ON){
		if(logging){
			LOG(LOG_LVL_DEBUG, "Power on startup beginning\r\n");
		}
		AT86RF212B_SetRegisters();
		AT86RF212B_PhyStateChange(newState);
	}
	switch(newState){
	case PLL_ON:
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
			AT86RF212B_Delay(AT86RF212B_tTR9);
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
			AT86RF212B_Delay(AT86RF212B_tFrame);
			StateChangeCheck(PLL_ON);
		}
		//TODO: May need to add a state to force to pll on, force to pll on is an unimplemented transition in the programmers guide
		else{
			AT86RF212B_WrongStateError();
		}
		break;


	case RX_AACK_ON:
		if(config.state == RX_AACK_ON){
			return;
		}
		/* AT86RF212::TRX_OFF */
		if(config.state == TRX_OFF){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_AACK_ON);
			AT86RF212B_Delay(AT86RF212B_tTR4);
			StateChangeCheck(RX_AACK_ON);
		}
		 /* AT86RF212::PLL_ON */
		else if(config.state ==  PLL_ON){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_AACK_ON);
			AT86RF212B_Delay(AT86RF212B_tTR8);
			StateChangeCheck(CMD_RX_AACK_ON);
		}
		/* AT86RF212::BUSY_TX */
		else if(IsStateTxBusy()){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_AACK_ON);
			AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);
			StateChangeCheck(CMD_RX_AACK_ON);
		}
		else{
			AT86RF212B_WrongStateError();
		}
		break;
	case TX_ARET_ON:
		//TODO:Remove this condition, was used for a terminal test
		if(config.state == TX_ARET_ON){
			return;
		}
		/* AT86RF212::TRX_OFF */
		if(config.state == TRX_OFF){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_TX_ARET_ON);
			AT86RF212B_Delay(AT86RF212B_tTR4);
			StateChangeCheck(CMD_TX_ARET_ON);
		}
		 /* AT86RF212::PLL_ON */
		else if(config.state ==  PLL_ON){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_TX_ARET_ON);
			AT86RF212B_Delay(AT86RF212B_tTR8);
			StateChangeCheck(CMD_TX_ARET_ON);
		}
		 /* AT86RF212::RX_AACK_ON */
		else if(config.state ==  RX_AACK_ON){
			AT86RF212B_PhyStateChange(PLL_ON);
			AT86RF212B_PhyStateChange(TX_ARET_ON);
		}
		 /* AT86RF212::RX_ON */
		else if(config.state ==  RX_ON){
			AT86RF212B_PhyStateChange(PLL_ON);
			AT86RF212B_PhyStateChange(TX_ARET_ON);
		}
		/* AT86RF212::BUSY_TX */
		else if(IsStateTxBusy()){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_TX_ARET_ON);
			AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);
			StateChangeCheck(CMD_TX_ARET_ON);
		}
		else{
			AT86RF212B_WrongStateError();
		}
		break;
	case RX_ON:
		//TODO:Remove this condition, was used for a terminal test
		if(config.state == RX_ON){
			return;
		}
		 /* AT86RF212::TX_ARET_ON */
		else if(config.state ==  TX_ARET_ON){
			AT86RF212B_PhyStateChange(PLL_ON);
			AT86RF212B_PhyStateChange(TX_ARET_ON);
		}
		/* AT86RF212::TRX_OFF */
		if(config.state == TRX_OFF){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_ON);
			AT86RF212B_Delay(AT86RF212B_tTR6);
			StateChangeCheck(RX_ON);
		}
		 /* AT86RF212::PLL_ON */
		else if(config.state ==  PLL_ON){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_ON);
			AT86RF212B_Delay(AT86RF212B_tTR8);
			StateChangeCheck(RX_ON);
		}
		/* AT86RF212::BUSY_TX */
		else if(IsStateTxBusy()){
			AT86RF212B_BitWrite(SR_TRX_CMD, CMD_RX_ON);
			AT86RF212B_WaitForIRQ(TRX_IRQ_TRX_END);
			StateChangeCheck(RX_ON);
		}
		else{
			AT86RF212B_WrongStateError();
		}
		break;
	}
}

static void AT86RF212B_WrongStateError(){
	ASSERT(0);
	if(logging){
		LOG(LOG_LVL_ERROR, "Incorrect State to Run Function: Resetting\r\n");
	}
	AT86RF212B_TRX_Reset();
}

static void AT86RF212B_PhySetChannel(){
	/* AT86RF212::TRX_OFF */
	if(config.state == TRX_OFF){
		//CC_BAND (Table 9-34) 5 for 902.02MHz - 927.5 MHz
		AT86RF212B_BitWrite(SR_CC_BAND, 5);
		//(9.8.2) Fc[MHz] = 906[MHz] + 2[MHz] x (k 占� 1), for k = 1, 2, ..., 10
		AT86RF212B_BitWrite(SR_CHANNEL, 0);
	}
	else{
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "Incorrect State to Run Function: Resetting\r\n");
		}
		AT86RF212B_TRX_Reset();
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
		AT86RF212B_BitWrite(SR_MAX_FRAME_RETRIES, config.maxFrameRetries);
		AT86RF212B_BitWrite(SR_CSMA_LBT_MODE, config.CSMA_LBT_Mode);
		AT86RF212B_BitWrite(SR_MAX_CSMA_RETRIES, config.maxCSMA_Retries);
		AT86RF212B_BitWrite(SR_MIN_BE, config.minBe);
		AT86RF212B_BitWrite(SR_MAX_BE, config.maxBe);
		AT86RF212B_BitWrite(SR_SLOTTED_OPERATION, config.slottedOperatin);
		AT86RF212B_BitWrite(SR_AACK_I_AM_COORD, config.AACK_I_AmCoord);
		AT86RF212B_BitWrite(SR_AACK_SET_PD, config.AACK_SetPd);
		AT86RF212B_BitWrite(SR_AACK_UPLD_RES_FT, config.AACK_UPLD_RES_FT);
		AT86RF212B_BitWrite(SR_AACK_FLTR_RES_FT, config.AACK_FLTR_RES_FT);
		AT86RF212B_BitWrite(SR_RX_SAFE_MODE, config.rxSafeMode);
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

static uint8_t AT86RF212B_CheckForIRQ(uint8_t desiredIRQ){
	if(interupt){
		//Clear the interrupt flag
		interupt = 0;

		uint8_t irqState = AT86RF212B_RegRead(RG_IRQ_STATUS);

		if(desiredIRQ & irqState){
			if(logging){
				char tmpStr[40];
				sprintf(tmpStr, "IRQ checking for Received: 0x%02x\r\n", irqState);
				LOG(LOG_LVL_INFO, tmpStr);
			}

			if((irqState & desiredIRQ) == TRX_IRQ_TRX_END){
				//Disable preamble detector to prevent receiving another frame before the current one is read
				AT86RF212B_BitWrite(SR_RX_PDT_DIS, 1);
			}
			return 1;
		}
		//Not the IRQ being checked for but an IRQ was received
		if(logging){
			char tmpStr[20];
			sprintf(tmpStr, "IRQ Received: 0x%02x\r\n", irqState);
			LOG(LOG_LVL_DEBUG, tmpStr);
		}
	}
	return 0;
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
				LOG(LOG_LVL_DEBUG, "Timeout while waiting for IRQ\r\n");
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
			uint8_t tmpStr[20];
			sprintf((char *)tmpStr, "Wrong Interrupt: %02X\r\n", irqState);
			LOG(LOG_LVL_ERROR, (char *)tmpStr);
		}
		AT86RF212B_WaitForIRQ(expectedIRQ);
	}
	else if(logging){
		LOG(LOG_LVL_DEBUG, "Expected IRQ received, exiting loop!\r\n");
	}
}

static uint8_t StateChangeCheck(uint8_t newState){
	UpdateState();

	if(config.state != newState){
		ASSERT(0);
		if(logging){
			LOG(LOG_LVL_ERROR, "State Change Failed Trying Again\r\n");
		}

		if(newState == PLL_ON){
			AT86RF212B_PhyStateChange(PLL_ON);
		}

		return 0;
	}
	else if(logging){
		LOG(LOG_LVL_DEBUG, "State Change Success!\r\n");
		return 1;
	}
	return 0;
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

static void AT86RF212B_Delay(uint8_t time){
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
			}
			break;
		case AT86RF212B_tFrame:
			//tMSNC 	Minimum time to synchronize to a preamble and receive an SFD
			//    2 symbol
			switch(config.phyMode){
				case AT86RF212B_BPSK_20:
					DelayMs(52);
					break;
				case AT86RF212B_BPSK_40:
					DelayMs(26);
					break;
				case AT86RF212B_O_QPSK_100:
				case AT86RF212B_O_QPSK_200:
				case AT86RF212B_O_QPSK_400:
					DelayMs(11);
					break;
				case AT86RF212B_O_QPSK_250:
				case AT86RF212B_O_QPSK_500:
				case AT86RF212B_O_QPSK_1000:
					DelayUs(5);
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
