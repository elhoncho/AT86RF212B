/*
 * terminalHAL.h
 *
 *  Created on: Feb 12, 2018
 *      Author: owner
 */

#ifndef MYINC_TERMINALHAL_H_
#define MYINC_TERMINALHAL_H_

void terminalWriteHAL(char *txStr);
int8_t terminalReadRXCharHAL();
void terminalWriteRXCharHAL(char rxChar);

#endif /* MYINC_TERMINALHAL_H_ */
