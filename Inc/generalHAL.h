/*
 * generalHAL.h
 *
 *  Created on: Feb 17, 2018
 *      Author: owner
 */

#ifndef MYINC_GENERALHAL_H_
#define MYINC_GENERALHAL_H_

uint32_t GeneralGetMs();
uint32_t GeneralGetUs();
void GeneralDelayMs(uint32_t timeMs);
void GeneralDelayUs(uint32_t timeUs);
void GeneralReadInput();

#endif /* MYINC_GENERALHAL_H_ */
