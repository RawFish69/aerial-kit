// FHSS for 2.4 GHz (ExpressLRS-compatible)
#pragma once

#include "config.h"

#define FHSS_SEQ_LEN 256

extern uint8_t FHSSsequence[FHSS_SEQ_LEN];
extern volatile uint8_t FHSSptr;
extern uint32_t freqSpread;
extern uint8_t syncChannel;

void FHSSInit(uint32_t seed);
uint32_t FHSSGetInitialFreq();
uint32_t FHSSGetNextFreq();
uint8_t FHSSGetCurrIndex();
void FHSSSetCurrIndex(uint8_t idx);
uint32_t FHSSGetFreqAt(uint8_t idx);
