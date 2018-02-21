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

#include<stdint.h>
#include<string.h>
#include<stdlib.h>
#include<stdio.h>
#include "MyInc/terminal.h"
#include "MyInc/terminalHAL.h"
#include "MyInc/AT86RF212B.h"
#include "MyInc/generalHAL.h"
#include "MyInc/errors_and_logging.h"

#include "MyInc/AT86RF212B_Regesters.h"
#include "MyInc/AT86RF212B_Constants.h"

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
static void Reset(char *arg1, char *arg2);
static void TestSleep(char *arg1, char *arg2);

static const struct commandStruct commands[] ={
    {"clear", &CmdClear, "Clears the screen"},
    {"ls", &ListCommands, "Run Help Function"},
    {"help", &ListCommands, "Run Help Function"},
	{"logging", &ToggelDebug, "Toggles Logging Mode"},
	{"rr", &ReadRegister, "Reads a register"},
	{"rw", &WriteRegister, "Writes a value to a register"},
	{"id", &GetIDs, "get id's"},
	{"reset", &Reset, "reset from active state"},
	{"sleep", &TestSleep, "Test sleep state"},
    {"",0,""} //End of commands indicator. Must be last.
};

static void TestSleep(char *arg1, char *arg2){
	AT86RF212B_TestSleep();
}

static void Reset(char *arg1, char *arg2){
	AT86RF212B_TRX_Reset();
}

static void GetIDs(char *arg1, char *arg2){
	AT86RF212B_ID();
}

static void WriteRegister(char *arg1, char *arg2){
	char tmpStr[MAX_STR_LEN];
	sprintf(tmpStr, "0x%02X\r\n", AT86RF212B_RegWrite(strtol(arg1, NULL, 16), strtol(arg2, NULL, 16)));
	TerminalWrite(tmpStr);
}

static void ReadRegister(char *arg1, char *arg2){
	char tmpStr[MAX_STR_LEN];
	sprintf(tmpStr, "0x%02X\r\n", AT86RF212B_RegRead(strtol(arg1, NULL, 16)));
	TerminalWrite(tmpStr);
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
        TerminalWrite(tmpStr);
        strcpy(tmpStr, " - ");
        TerminalWrite(tmpStr);
        strcpy(tmpStr, commands[i].help);
        TerminalWrite(tmpStr);
        strcpy(tmpStr,"\r\n");
        TerminalWrite(tmpStr);
        i++;
    }
}
static void CmdClear(char *arg1, char *arg2){
    char tmpStr[MAX_STR_LEN];
    strcpy(tmpStr, "\033[2J\033[;H");
    TerminalWrite(tmpStr);
}

void TerminalOpen(){
	//TODO: Get this to run when the terminal is opened
    char tmpStr[MAX_STR_LEN];
    strcpy(tmpStr, "\033[2J\033[;H");
    TerminalWrite(tmpStr);
    strcpy(tmpStr,"Battle Control Online:");
    TerminalWrite(tmpStr);
    strcpy(tmpStr,"\r\n>");
    TerminalWrite(tmpStr);
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

        tmpChar = terminalReadRXCharHAL();

        arg[0][0] = '\0';
        arg[1][0] = '\0';
        arg[2][0] = '\0';

        i = 0;
        while(tmpChar != 255){
            uint8_t len = strlen(arg[i]);

            //Dont store \r or \n or space or .
            if(tmpChar != 0x0D && tmpChar != 0x0A && tmpChar != 0x20 && tmpChar != 0x2E){
                arg[i][len] = tmpChar;
                arg[i][len+1] = '\0';
            }
            //space or . => new argument
            else if(tmpChar == 0x20 ||  tmpChar == 0x2E){
                i++;
            }

            tmpChar = terminalReadRXCharHAL();
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
                    strcpy(tmpStr, "\r\n");
                    TerminalWrite(tmpStr);

                    commands[i].execute(arg[1], arg[2]);
                    strcpy(tmpStr, ">");
                    TerminalWrite(tmpStr);
                    i = 0;
                    break;
                }
                i++;
            }
            //i is set to 0 if a command is found
            if(i != 0){
                strcpy(tmpStr, "\r\n");
                TerminalWrite(tmpStr);

                strcpy(tmpStr, "No Command Found \r\n>");
                TerminalWrite(tmpStr);
            }
        }

        newCmd = 0;
    }
}

void TerminalWrite(char *txStr){
	terminalWriteHAL(txStr);
}

void TerminalMain(){
    TerminalRead();
}
