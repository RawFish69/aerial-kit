// CRSF serial: parse from UART (TX input) / send to FC (RX output)
#pragma once

#include <Arduino.h>
#include "config.h"

#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAME_SIZE_RC (4 + 22 + 1)  // addr+len+type + 22 bytes payload + crc

class CrsfSerial {
public:
    CrsfSerial();
    void begin(HardwareSerial* serial, int8_t txPin, int8_t rxPin);
    void update();

    // TX: get latest RC channels from computer (CRSF input)
    void getChannels(uint16_t* channels);
    bool hasNewChannels() const { return hasNewChannels_; }
    void clearNewChannels() { hasNewChannels_ = false; }

    // RX: send RC channels to flight controller
    void sendRcChannels(const uint16_t* channels);

private:
    HardwareSerial* serial_;
    uint16_t channels_[CRSF_NUM_CHANNELS];
    bool hasNewChannels_;
    uint8_t rxBuf_[64];
    uint8_t rxLen_;
    uint8_t crc8(const uint8_t* data, uint8_t len);
    void packRcChannels(const uint16_t* ch, uint8_t* payload);
    void unpackRcChannels(const uint8_t* payload, uint16_t* ch);
};

extern CrsfSerial Crsf;
