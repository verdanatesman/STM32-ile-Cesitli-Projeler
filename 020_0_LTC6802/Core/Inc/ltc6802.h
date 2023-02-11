/*
 * ltc6802.h
 *
 *  Created on: 20 Haz 2021
 *      Author: verdan
 */

#ifndef INC_LTC6802_H_
#define INC_LTC6802_H_

#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <inttypes.h>

typedef uint8_t byte;


//explicit LTC6802(B address, byte csPin);

void cfgRead();
const void cfgWrite(bool broadcast);
const void cfgDebugOutput();
const bool cfgGetWDT();
const bool cfgGetGPIO1() ;
void cfgSetGPIO1(bool gpio);
const bool cfgGetGPIO2();
void cfgSetGPIO2(bool gpio);
const bool cfgGetLVLPL();
void cfgSetLVLPL(bool lvlpl);
int cfgGetCDC();
void cfgSetCDC(uint8_t cdc);
const uint8_t cfgGetDCC();
void cfgSetDCC(uint8_t dcc);
const uint8_t cfgGetMCI();
void cfgSetMCI(uint8_t mci);
const uint8_t cfgGetVUV();
void cfgSetVUV(uint8_t);
const uint8_t cfgGetVOV();
void cfgSetVOV(uint8_t vov);
void temperatureMeasure();
void temperatureRead();
void temperatureDebugOutput() ;
void cellsMeasure();
void cellsRead();
void cellsDebugOutput() ;
void flagsRead();
void flagsDebugOutput();

#define cfgRegisters 6
#define tmpRegisters 5
#define cellRegisters 18
#define flgRegisters 3
#define maxCells 12
uint8_t address;
uint8_t csPin = 10;
uint8_t CFG[cfgRegisters];
uint8_t TMP[tmpRegisters];
uint8_t CV[cellRegisters];
uint8_t FLG[flgRegisters];
void read(uint8_t cmd, uint8_t numOfRegisters, uint8_t * arr);
const void measure(uint8_t cmd, bool broadcast);
void readValues(uint8_t cmd, uint8_t numOfRegisters, uint8_t * arr);




#endif /* INC_LTC6802_H_ */
