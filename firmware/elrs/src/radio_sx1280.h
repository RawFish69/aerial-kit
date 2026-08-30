// SX1280 radio wrapper using RadioLib (ELRS FLRC 500 Hz, 8-byte packet)
#pragma once

#include "config.h"
#include <RadioLib.h>

#define ELRS_RF_RATE_ENUM 0  // single rate for now

class RadioSX1280 {
public:
    RadioSX1280();
    bool begin(const uint8_t* uid);
    void setFrequency(uint32_t regFreq);
    void transmit(uint8_t* data, uint8_t len);
    void startReceive();
    bool readPacket(uint8_t* data, uint8_t len);
    bool isTxDone();
    bool isRxDone();
    void clearIrq();

private:
    SX1280* radio_;
    Module* mod_;
    bool ok_;
};

extern RadioSX1280 Radio;
