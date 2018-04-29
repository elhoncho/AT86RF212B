/*
 * terminal.c
 *
 *  Created on: Feb 12, 2018
 *      Author: owner
 *
 *   HowTo:  -put the terminalOpen() where it will execute when the terminal is opened
 *           -put the terminalMain() function in the main loop
 *           -make newCmd true when a command has been entered (usually when recieved a cr and or nl)
 */

#include "../Inc/Terminal.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "../Inc/AT86RF212B.h"
#include "../Inc/AT86RF212B_Regesters.h"
#include "../Inc/AT86RF212B_Constants.h"
#include "../Inc/AT86RF212B_HAL.h"
#include "../Inc/RawMode.h"
#include "../Inc/MainController.h"
#include "../Inc/Buffer.h"
#include "../Inc/ErrorsAndLogging.h"
#include "../Settings/AT86RF212B_Settings.h"
#include "../Settings/HAL_Settings.h"
#include "../Settings/TerminalSettings.h"


#if RASPBERRY_PI
#include "../../main.h"
#endif

#if STM32
#include "main.h"
#include "usbd_cdc_if.h"
#endif

#define MAX_STR_LEN 32

struct commandStruct{
    const uint8_t* name;
    functionPointerType execute;
    const uint8_t* help;
};

//Prototypes for the command functions
static void CmdClear(uint8_t* arg1, uint8_t* arg2);
static void ListCommands(uint8_t* arg1, uint8_t* arg2);
static void ToggelDebug(uint8_t* arg1, uint8_t* arg2);
static void ReadRegister(uint8_t* arg1, uint8_t* arg2);
static void WriteRegister(uint8_t* arg1, uint8_t* arg2);
static void GetIDs(uint8_t* arg1, uint8_t* arg2);
static void TestBit(uint8_t* arg1, uint8_t* arg2);
static void ReadFrame(uint8_t* arg1, uint8_t* arg2);
static void RawModeRx(uint8_t* arg1, uint8_t* arg2);
static void RawModeTx(uint8_t* arg1, uint8_t* arg2);
static void RawModeRxTx(uint8_t* arg1, uint8_t* arg2);
static void ExitProgram(uint8_t* arg1, uint8_t* arg2);

static const struct commandStruct commands[] ={
    {(uint8_t*)"clear", &CmdClear, (uint8_t*)"Clears the screen"},
    {(uint8_t*)"ls", &ListCommands, (uint8_t*)"Run Help Function"},
    {(uint8_t*)"help", &ListCommands, (uint8_t*)"Run Help Function"},
	{(uint8_t*)"logging", &ToggelDebug, (uint8_t*)"Toggles Logging Mode"},
	{(uint8_t*)"rr", &ReadRegister, (uint8_t*)"Reads a register"},
	{(uint8_t*)"rw", &WriteRegister, (uint8_t*)"Writes a value to a register"},
	{(uint8_t*)"id", &GetIDs, (uint8_t*)"get id's"},
	{(uint8_t*)"bt", &TestBit, (uint8_t*)"Test a bit of a reg"},
	{(uint8_t*)"rf", &ReadFrame, (uint8_t*)"Reads the frame buffer"},
	{(uint8_t*)"rmr", &RawModeRx, (uint8_t*)"Run in raw mode rx"},
	{(uint8_t*)"rmt", &RawModeTx, (uint8_t*)"Run in raw mode tx"},
	{(uint8_t*)"rmrt", &RawModeRxTx, (uint8_t*)"Run in raw mode rx/tx"},
	{(uint8_t*)"exit", &ExitProgram, (uint8_t*)"Exit the Program"},
    {(uint8_t*)"",0,(uint8_t*)""} //End of commands indicator. Must be last.
};

//----------------Commands Functions------------------------//
static void ExitProgram(uint8_t *arg1, uint8_t *arg2){
	exit(0);
}

static void RawModeTx(uint8_t *arg1, uint8_t *arg2){
	MainControllerSetMode(MODE_RAW_TX);
}

static void RawModeRx(uint8_t *arg1, uint8_t *arg2){
	MainControllerSetMode(MODE_RAW_RX);
}

static void RawModeRxTx(uint8_t *arg1, uint8_t *arg2){
	MainControllerSetMode(MODE_RAW_RX_TX);
}

static void ReadFrame(uint8_t *arg1, uint8_t *arg2){
	AT86RF212B_FrameRead();
}

static void TestBit(uint8_t *arg1, uint8_t *arg2){
	uint8_t tmpStr[MAX_STR_LEN];
	sprintf((char*)tmpStr, "%i\r\n", AT86RF212B_BitRead(strtol((char*)arg1, NULL, 16), 0, strtol((char*)arg2, NULL, 10)));
	TerminalWrite((uint8_t*)tmpStr);
}

static void GetIDs(uint8_t *arg1, uint8_t *arg2){
	AT86RF212B_ID();
}

static void WriteRegister(uint8_t *arg1, uint8_t *arg2){
	uint8_t tmpStr[MAX_STR_LEN];
	sprintf((char*)tmpStr, "0x%02X\r\n", AT86RF212B_RegWrite(strtol((char*)arg1, NULL, 16), strtol((char*)arg2, NULL, 16)));
	TerminalWrite((uint8_t*)tmpStr);
}

static void ReadRegister(uint8_t *arg1, uint8_t *arg2){
	uint8_t tmpStr[MAX_STR_LEN];
	sprintf((char*)tmpStr, "0x%02X\r\n", AT86RF212B_RegRead(strtol((char*)arg1, NULL, 16)));
	TerminalWrite((uint8_t*)tmpStr);
}

static void ToggelDebug(uint8_t *arg1, uint8_t *arg2){
    ToggleLogging();
}

static void ListCommands(uint8_t *arg1, uint8_t *arg2){
	uint8_t tmpStr[MAX_STR_LEN];
    uint8_t i = 0;
    while(commands[i].execute){
        strcpy((char*)tmpStr, (char*)commands[i].name);
        TerminalWrite((uint8_t*)tmpStr);
        strcpy((char*)tmpStr, " - ");
        TerminalWrite((uint8_t*)tmpStr);
        strcpy((char*)tmpStr, (char*)commands[i].help);
        TerminalWrite((uint8_t*)tmpStr);
        strcpy((char*)tmpStr,"\r\n");
        TerminalWrite((uint8_t*)tmpStr);
        i++;
    }
}
static void CmdClear(uint8_t *arg1, uint8_t *arg2){
    char tmpStr[MAX_STR_LEN];
    strcpy(tmpStr, "\033[2J\033[;H");
    TerminalWrite((uint8_t*)tmpStr);
}


//----------------Standard Functions------------------------//

void TerminalOpen(){
	AT86RF212B_PhyStateChange(TX_ARET_ON);
    char tmpStr[MAX_STR_LEN];
    strcpy(tmpStr, "\033[2J\033[;H");
    TerminalWrite((uint8_t*)tmpStr);
    strcpy(tmpStr,"Interactive Terminal Mode:\r\n");
    TerminalWrite((uint8_t*)tmpStr);
    strcpy(tmpStr,"\r\n>");
    TerminalWrite((uint8_t*)tmpStr);
}

void TermianlClose(){

}

void TerminalRead(){
	uint8_t tmpChar;
	uint8_t newCmd = 0;
	static uint8_t cmdIter = 0;
	static uint8_t cmdStr[MAX_STR_LEN];
	uint8_t tmpStr[MAX_STR_LEN];

	while(PopFromInputBuffer(&tmpChar)){
		if(cmdIter < MAX_STR_LEN){
			cmdStr[cmdIter] = tmpChar;
			cmdStr[cmdIter+1] = '\0';
			cmdIter++;
			if(tmpChar == '\r' || tmpChar == '\n'){
				newCmd = 1;
				cmdIter = 0;
				break;
			}
		}
		else{
			if(IsLogging()){
				ASSERT(0);
				LOG(LOG_LVL_ERROR, (uint8_t*)"Command string too large\r\n");
			}
		}
	}
    if(newCmd){
        uint8_t arg[3][22];
        uint8_t argNum = 0;
        uint8_t i = 0;
        uint8_t len = 0;

        arg[0][0] = '\0';
        arg[1][0] = '\0';
        arg[2][0] = '\0';

        for(i = 0; i < strlen((char*)cmdStr); i++){
            //Don't store \r or \n or space or .
            if(cmdStr[i] != 0x0D && cmdStr[i] != 0x0A && cmdStr[i] != 0x20 && cmdStr[i] != 0x2E){
                arg[argNum][len] = cmdStr[i];
                arg[argNum][len+1] = '\0';
                len++;
            }
            //space or . => new argument
            else if(cmdStr[i] == 0x20 ||  cmdStr[i] == 0x2E){
                argNum++;
                len = 0;
            }
        }

        if(IsLogging()){
            strcpy((char*)tmpStr, "\r\nArg0: \r\n");
            LOG(LOG_LVL_INFO, tmpStr);
            LOG(LOG_LVL_INFO, arg[0]);

            strcpy((char*)tmpStr, "\r\nArg1: \r\n");
            LOG(LOG_LVL_INFO, tmpStr);
			LOG(LOG_LVL_INFO, arg[1]);

            strcpy((char*)tmpStr, "\r\nArg2: \r\n");
            LOG(LOG_LVL_INFO, tmpStr);
			LOG(LOG_LVL_INFO, arg[2]);
        }

        i = 0;
        if(strlen((char*)arg[0]) >= 1){
            while(commands[i].execute){
                if(strcmp((char*)arg[0], (char*)commands[i].name) == 0){
                    //Execute Command
                    commands[i].execute(arg[1], arg[2]);

                    //Write prompt char >
                    tmpStr[0] = '>';
                    tmpStr[1] = '\0';
                    TerminalWrite((uint8_t*)tmpStr);
                    i = 0;
                    break;
                }
                i++;
            }
            //i is set to 0 if a command is found
            if(i != 0){
                strcpy((char*)tmpStr, "\r\n");
                TerminalWrite((uint8_t*)tmpStr);

                strcpy((char*)tmpStr, "No Command Found \r\n>");
                TerminalWrite((uint8_t*)tmpStr);
            }
        }
    }
}

void TerminalWrite(uint8_t *txStr){
	WriteToOutputHAL(txStr, strlen((char*)txStr));
}

void TerminalMain(){
	ReadInputHAL();
    TerminalRead();
}
