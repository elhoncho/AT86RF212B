#include <stdint.h>
#include "../Inc/terminal.h"
#include "../Inc/AT86RF212B.h"
#include "../Inc/AT86RF212B_HAL.h"
#include "../Inc/RawMode.h"
#include "../Inc/MainController.h"
#include "../Settings/TerminalSettings.h"

static uint8_t AT86RF212B_Mode = TERMINAL_MODE;

void MainControllerOpen(){
	AT86RF212B_Open();

	switch(AT86RF212B_Mode){
	case MODE_RAW_TX:
		RawModeOpen();
		break;
	case MODE_RAW_RX:
		RawModeOpen();
		break;
	case MODE_RAW_RX_TX:
		RawModeOpen();
		break;
	case MODE_TERMINAL:
		TerminalOpen();
		break;
	}
}

void MainControllerSetMode(uint8_t newMode){
	AT86RF212B_Mode = newMode;
	MainControllerOpen();
}

uint8_t MainControllerGetMode(){
	return AT86RF212B_Mode;
}

void MainControllerLoop(){
	switch(AT86RF212B_Mode){
		case MODE_RAW_TX:
			RawModeMain();
			break;
		case MODE_RAW_RX:
			break;
		case MODE_RAW_RX_TX:
			RawModeMain();
			break;
		case MODE_TERMINAL:
			TerminalMain();
			break;
		}

		AT86RF212B_Main();
}
