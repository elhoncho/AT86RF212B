/*
 * HAL_Settings.h
 *
 *  Created on: Apr 15, 2018
 *      Author: owner
 */

#ifndef HAL_SETTINGS_H_
#define HAL_SETTINGS_H_

//Hardware back end, ONLY ONE SHOUDL BE TRUE
#define STM32 1
#define RASPBERRY_PI 0

#if STM32

#define ECHO_INPUT 0

#endif

#if RASPBERRY_PI

# define ECHO_INPUT 0

#endif

#endif /* HAL_SETTINGS_H_ */
