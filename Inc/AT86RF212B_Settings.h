/*
 * AT86RF212B_Settings.h
 *
 *  Created on: Feb 22, 2018
 *      Author: owner
 */

#ifndef AT86RF212B_INC_AT86RF212B_SETTINGS_H_
#define AT86RF212B_INC_AT86RF212B_SETTINGS_H_

#include "AT86RF212B.h"

//Hardware back end, ONLY ONE SHOUDL BE TRUE
#define STM32 1
#define RASPBERRY_PI 0

//Settings

#define AT86RF212B_MAX_DATA 114
#define AT86RF212B_DATA_OFFSET 9

//Change this to set the RF mode
#define AT86RF212B_PHY_MODE 		AT86RF212B_O_QPSK_100;
//scrambler configuration for O-QPSK_{400,1000}; values { 0: disabled, 1: enabled (default)}.
#define AT86RF212B_SCRAMEN 			1
//transmit signal pulse shaping for O-QPSK_{250,500,1000}; values {0 : half-sine filtering (default), 1 : RC-0.8 filtering}.
#define AT86RF212B_RCEN 			0
//Set the TX power level (Table 9-15) 0x03 = 0dBm
#define AT86RF212B_TX_POWER			0x03
//Set the RX sensitivity RX threshold = RSSI_BAS_VAL + rxSensLvl * 3
//rxSensLvl = 0 - 15, 0 = max sensitivity
#define AT86RF212B_RX_SENSE_LVL		0
//Enable TX CRC generation 1 = on 0 = off
#define AT86RF212B_TX_CRC 			1
//Sets the number of attempts to retransmit a frame when it was not acknowledged by the recipient (0x00 - 0x07)
#define AT86RF212B_MAX_FRAME_RETRIES 0X03
//Switches between CSMA-CA Listen Before Talk algorithm within TX_ARET mode (0 [CSMA-CA] or 1 [LBT])
#define AT86RF212B_CSMA_LBT_MODE 	 0x00
//Number of retries in TX_ARET mode to repeat the CSMA-CA procedure before the transaction gets cancled (0x00[no retries] - 0x05[five retries] or 0x07[immedate frame transmission withou breforming CSMA-CA])
#define AT86RF212B_MAX_CSMA_RETRIES  0X04
//Maximum backoff exponent in the CSMA-CA algorithm (0x00 - 0x08)
#define AT86RF212B_MIN_BE 			0x05
//Minimum backoff exponent in the CSMA-CA algorithm (MAX_BE-1 to MAX_BE or 0x00)
#define AT86RF212B_MAX_BE			0x03
// 0 = unslotted mode, ack frame is automatically sent if requested 1 = slotted, transmission of an ack fram has to be controlled by the microcontroller
#define AT86RF212B_SLOTTED_OPERATION 0x00
// 0 = This node is NOT a PAN coordinator; 1 = This node is a pan coordinator
#define AT86RF212B_AACK_I_AM_COORD 	0x00
// 0 = off 1 = on, Prevents overwriting received data with new received data before having read the current data
#define AT86RF212B_RX_SAFE_MODE		0x00
#define AT86RF212B_AACK_UPLD_RES_FT	0x01
#define AT86RF212B_AACK_FLTR_RES_FT 0x01
//0 = no pending data 0 ; 1 = pending data
#define AT86RF212B_AACK_SET_PD		0x00
//Address Filtering
#define AT86RF212B_PAN_ID_7_0 		0xCC
#define AT86RF212B_PAN_ID_15_8 		0xCC

#if STM32
#define AT86RF212B_SHORT_ADDR_7_0	0xAA
#define AT86RF212B_SHORT_ADDR_15_8 	0xAA
#endif
#if RASPBERRY_PI
#define AT86RF212B_SHORT_ADDR_7_0	0xBB
#define AT86RF212B_SHORT_ADDR_15_8 	0xBB
#endif

#if STM32
#define AT86RF212B_SHORT_ADDR_TARGET_7_0	0xBB
#define AT86RF212B_SHORT_ADDR_TARGET_15_8 	0xBB
#endif

#if RASPBERRY_PI
#define AT86RF212B_SHORT_ADDR_TARGET_7_0	0xAA
#define AT86RF212B_SHORT_ADDR_TARGET_15_8 	0xAA
#endif

#define AT86RF212B_EXT_ADDR_7_0		0x00
#define AT86RF212B_EXT_ADDR_15_8 	0x00
#define AT86RF212B_EXT_ADDR_23_16	0x00
#define AT86RF212B_EXT_ADDR_31_24 	0x00
#define AT86RF212B_EXT_ADDR_39_32	0x00
#define AT86RF212B_EXT_ADDR_47_40 	0x00
#define AT86RF212B_EXT_ADDR_55_48 	0x00
#define AT86RF212B_EXT_ADDR_63_56 	0x00

#endif /* AT86RF212B_INC_AT86RF212B_SETTINGS_H_ */
