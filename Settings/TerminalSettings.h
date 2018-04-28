/*
 * TerminalSettings.h
 *
 *  Created on: Apr 15, 2018
 *      Author: owner
 */

#ifndef TERMINALSETTINGS_H_
#define TERMINALSETTINGS_H_

#include "HAL_Settings.h"

#define MODE_RAW_TX 0
#define MODE_RAW_RX 1
#define MODE_RAW_RX_TX 2
#define MODE_TERMINAL 3

#if STM32

#define ECHO_INPUT 0
#define TERMINAL_MODE MODE_TERMINAL

#endif

#if RASPBERRY_PI

#define ECHO_INPUT 0
#define TERMINAL_MODE MODE_TERMINAL

#endif

#endif /* TERMINALSETTINGS_H_ */
