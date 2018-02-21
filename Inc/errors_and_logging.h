/*
 * logging.h
 *
 *  Created on: Feb 18, 2018
 *      Author: owner
 */

#ifndef MYINC_ERRORS_AND_LOGGING_H_
#define MYINC_ERRORS_AND_LOGGING_H_

#define ASSERT(condition) (condition ? 0 : AssertError(__FILE__, __LINE__))

typedef enum{
	LOG_LVL_INFO,
	LOG_LVL_DEBUG,
	LOG_LVL_ERROR
}LOG_LVL;

void LOG(LOG_LVL lvl, char * message);
void AssertError(char * fileName, int lineNumber);

#endif /* MYINC_ERRORS_AND_LOGGING_H_ */
