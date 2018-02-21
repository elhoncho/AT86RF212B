/*
 * terminal.h
 *
 *  Created on: Feb 12, 2018
 *      Author: owner
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

typedef void(*functionPointerType)(char *arg1, char *arg2);

void TerminalOpen();
void TermianlClose();
void TerminalRead();
void TerminalWrite(char *txStr);
void TerminalMain();

#endif /* TERMINAL_H_ */
