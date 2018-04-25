/*
 * HAL_Settings.h
 *
 *  Created on: Apr 15, 2018
 *      Author: owner
 */

#ifndef HAL_SETTINGS_H_
#define HAL_SETTINGS_H_

//Hardware back end, ONLY ONE SHOUDL BE TRUE
#define STM32 0
#define RASPBERRY_PI 1

//Size of the circule buffers used to store incoming and outgoing data
#define RX_BUFFER_LENGTH 256

#endif /* HAL_SETTINGS_H_ */
