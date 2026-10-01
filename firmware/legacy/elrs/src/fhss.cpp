#include "fhss.h"
#include <string.h>

// 2.4 GHz ISM: 2400.4 - 2479.4 MHz, 80 channels
// SX1280 FREQ_STEP = 52e6 / 2^18 ≈ 198.36 Hz. Reg = freq_hz / FREQ_STEP.
#define FREQ_STEP 198
#define REG_FREQ(freq_hz) ((uint32_t)((freq_hz) / FREQ_STEP))

uint8_t FHSSsequence[FHSS_SEQ_LEN];
volatile uint8_t FHSSptr = 0;
uint32_t freqSpread = 0;
uint8_t syncChannel = 0;

static uint32_t rngState;
static uint32_t rngNext() {
    rngState = rngState * 1103515245 + 12345;
    return (rngState >> 16) & 0x7FFF;
}

void FHSSInit(uint32_t seed) {
    rngState = seed;
    FHSSptr = 0;
    uint32_t freqCount = ELRS_FREQ_COUNT;
    syncChannel = freqCount / 2;
    freqSpread = (ELRS_FREQ_END_HZ - ELRS_FREQ_START_HZ) * 256 / (freqCount - 1);

    for (uint16_t i = 0; i < FHSS_SEQ_LEN; i++) {
        if (i % freqCount == 0)
            FHSSsequence[i] = syncChannel;
        else if (i % freqCount == syncChannel)
            FHSSsequence[i] = 0;
        else
            FHSSsequence[i] = i % freqCount;
    }
    for (uint16_t i = 0; i < FHSS_SEQ_LEN; i++) {
        if (i % freqCount != 0) {
            uint8_t offset = (i / freqCount) * freqCount;
            uint8_t r = (rngNext() % (freqCount - 1)) + 1;
            uint8_t t = FHSSsequence[i];
            FHSSsequence[i] = FHSSsequence[offset + r];
            FHSSsequence[offset + r] = t;
        }
    }
}

uint32_t FHSSGetInitialFreq() {
    return REG_FREQ(ELRS_FREQ_START_HZ + (syncChannel * freqSpread / 256));
}

uint32_t FHSSGetNextFreq() {
    FHSSptr = (FHSSptr + 1) % FHSS_SEQ_LEN;
    return FHSSGetFreqAt(FHSSptr);
}

uint32_t FHSSGetFreqAt(uint8_t idx) {
    return REG_FREQ(ELRS_FREQ_START_HZ + (FHSSsequence[idx] * freqSpread / 256));
}

uint8_t FHSSGetCurrIndex() {
    return FHSSptr;
}

void FHSSSetCurrIndex(uint8_t idx) {
    FHSSptr = idx % FHSS_SEQ_LEN;
}
