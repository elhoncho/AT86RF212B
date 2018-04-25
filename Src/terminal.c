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

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "../Inc/terminal.h"
#include "../Inc/AT86RF212B.h"
#include "../Inc/errors_and_logging.h"
#include "../Inc/AT86RF212B_Regesters.h"
#include "../Inc/AT86RF212B_Constants.h"
#include "../Inc/AT86RF212B_HAL.h"
#include "../Inc/RawMode.h"
#include "../Inc/MainController.h"
#include "../Inc/Buffer.h"
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

//TODO:Globle variables!...probably need to find a better way to do this
uint8_t volatile newCmd = 0;
extern uint8_t logging;

struct commandStruct{
    const char *name;
    functionPointerType execute;
    const char *help;
};

//Prototypes for the command functions
static void CmdClear(char *arg1, char *arg2);
static void ListCommands(char *arg1, char *arg2);
static void ToggelDebug(char *arg1, char *arg2);
static void ReadRegister(char *arg1, char *arg2);
static void WriteRegister(char *arg1, char *arg2);
static void GetIDs(char *arg1, char *arg2);
static void TestBit(char *arg1, char *arg2);
static void ReadFrame(char *arg1, char *arg2);
static void RawModeRx(char *arg1, char *arg2);
static void RawModeTx(char *arg1, char *arg2);
static void RawModeRxTx(char *arg1, char *arg2);
static void ExitProgram(char *arg1, char *arg2);

static const struct commandStruct commands[] ={
    {"clear", &CmdClear, "Clears the screen"},
    {"ls", &ListCommands, "Run Help Function"},
    {"help", &ListCommands, "Run Help Function"},
	{"logging", &ToggelDebug, "Toggles Logging Mode"},
	{"rr", &ReadRegister, "Reads a register"},
	{"rw", &WriteRegister, "Writes a value to a register"},
	{"id", &GetIDs, "get id's"},
	{"bt", &TestBit, "Test a bit of a reg"},
	{"rf", &ReadFrame, "Reads the frame buffer"},
	{"rmr", &RawModeRx, "Run in raw mode rx"},
	{"rmt", &RawModeTx, "Run in raw mode tx"},
	{"rmrt", &RawModeRxTx, "Run in raw mode rx/tx"},
	{"exit", &ExitProgram, "Exit the Program"},
    {"",0,""} //End of commands indicator. Must be last.
};

//----------------Commands Functions------------------------//
static void ExitProgram(char *arg1, char *arg2){
	exit(0);
}

static void RawModeTx(char *arg1, char *arg2){
	MainControllerSetMode(MODE_RAW_TX);
}

static void RawModeRx(char *arg1, char *arg2){
	MainControllerSetMode(MODE_RAW_RX);
}

static void RawModeRxTx(char *arg1, char *arg2){
	MainControllerSetMode(MODE_RAW_RX_TX);
}

static void ReadFrame(char *arg1, char *arg2){
	AT86RF212B_FrameRead();
}

static void TestBit(char *arg1, char *arg2){
	char tmpStr[MAX_STR_LEN];
	sprintf(tmpStr, "%i\r\n", AT86RF212B_BitRead(strtol(arg1, NULL, 16), 0, strtol(arg2, NULL, 10)));
	TerminalWrite((uint8_t*)tmpStr);
}

static void GetIDs(char *arg1, char *arg2){
	AT86RF212B_ID();
}

static void WriteRegister(char *arg1, char *arg2){
	char tmpStr[MAX_STR_LEN];
	sprintf(tmpStr, "0x%02X\r\n", AT86RF212B_RegWrite((uint8_t)strtol(arg1, NULL, 16), (uint8_t)strtol(arg2, NULL, 16)));
	TerminalWrite((uint8_t*)tmpStr);
}

static void ReadRegister(char *arg1, char *arg2){
	char tmpStr[MAX_STR_LEN];
	sprintf(tmpStr, "0x%02X\r\n", AT86RF212B_RegRead(strtol(arg1, NULL, 16)));
	TerminalWrite((uint8_t*)tmpStr);
}

static void ToggelDebug(char *arg1, char *arg2){
    if(logging){
        logging = 0;
    }
    else{
        logging = 1;
    }
}

static void ListCommands(char *arg1, char *arg2){
    char tmpStr[MAX_STR_LEN];
    uint8_t i = 0;
    while(commands[i].execute){
        strcpy(tmpStr, commands[i].name);
        TerminalWrite((uint8_t*)tmpStr);
        strcpy(tmpStr, " - ");
        TerminalWrite((uint8_t*)tmpStr);
        strcpy(tmpStr, commands[i].help);
        TerminalWrite((uint8_t*)tmpStr);
        strcpy(tmpStr,"\r\n");
        TerminalWrite((uint8_t*)tmpStr);
        i++;
    }
}
static void CmdClear(char *arg1, char *arg2){
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
	//TODO: Need to find a better way to control this than the newCmd switch
    if(newCmd){
        char tmpStr[MAX_STR_LEN];
        static uint8_t i = 0;
        char tmpChar;

        char arg[3][22];

        arg[0][0] = '\0';
        arg[1][0] = '\0';
        arg[2][0] = '\0';

        i = 0;
        while(PopFromRxBuffer(&tmpChar)){
            uint8_t len = strlen(arg[i]);

            //Don't store \r or \n or space or .
            if(tmpChar != 0x0D && tmpChar != 0x0A && tmpChar != 0x20 && tmpChar != 0x2E){
                arg[i][len] = tmpChar;
                arg[i][len+1] = '\0';
            }
            //space or . => new argument
            else if(tmpChar == 0x20 ||  tmpChar == 0x2E){
                i++;
            }
        }

        if(logging){
            strcpy(tmpStr, "\r\nArg0: \r\n");
            LOG(LOG_LVL_INFO, tmpStr);
            LOG(LOG_LVL_INFO, arg[0]);

            strcpy(tmpStr, "\r\nArg1: \r\n");
            LOG(LOG_LVL_INFO, tmpStr);
			LOG(LOG_LVL_INFO, arg[1]);

            strcpy(tmpStr, "\r\nArg2: \r\n");
            LOG(LOG_LVL_INFO, tmpStr);
			LOG(LOG_LVL_INFO, arg[2]);
        }

        i = 0;
        if(strlen(arg[0]) >= 1){
            while(commands[i].execute){
                if(strcmp(arg[0], commands[i].name) == 0){
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
                strcpy(tmpStr, "\r\n");
                TerminalWrite((uint8_t*)tmpStr);

                strcpy(tmpStr, "No Command Found \r\n>");
                TerminalWrite((uint8_t*)tmpStr);
            }
        }

        newCmd = 0;
    }
}

void TerminalWrite(uint8_t *txStr){
	WriteToOutputHAL(txStr, strlen((char*)txStr));
}

void TerminalMain(){
	ReadInputHAL();
    TerminalRead();
}
