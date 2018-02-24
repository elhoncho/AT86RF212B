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
//Address Filtering
#define AT86RF212B_PAN_ID_7_0 		1
#define AT86RF212B_PAN_ID_15_8 		0x01
#define AT86RF212B_SHORT_ADDR_7_0	0x00
#define AT86RF212B_SHORT_ADDR_15_8 	0x01
#define AT86RF212B_EXT_ADDR_7_0		0x00
#define AT86RF212B_EXT_ADDR_15_8 	0x01
#define AT86RF212B_EXT_ADDR_23_16	0x00
#define AT86RF212B_EXT_ADDR_31_24 	0x01
#define AT86RF212B_EXT_ADDR_39_32	0x00
#define AT86RF212B_EXT_ADDR_47_40 	0x01
#define AT86RF212B_EXT_ADDR_55_48 	0x00
#define AT86RF212B_EXT_ADDR_63_56 	0x01

#endif /* AT86RF212B_INC_AT86RF212B_SETTINGS_H_ */
