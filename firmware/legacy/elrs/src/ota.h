// ELRS OTA packet format (ExpressLRS-compatible OTA4)
#pragma once

#include "config.h"

#define PACKED __attribute__((packed))

#define PACKET_TYPE_RCDATA  0b00
#define PACKET_TYPE_DATA    0b01
#define PACKET_TYPE_SYNC    0b10

// 4 channels packed as 4x10-bit in 5 bytes
typedef struct PACKED {
    uint8_t raw[5];
} OTA_Channels_4x10_t;

typedef struct PACKED {
    uint8_t fhssIndex;
    uint8_t nonce;
    uint8_t rfRateEnum;
    uint8_t switchEncMode : 1, newTlmRatio : 3, geminiMode : 1, otaProtocol : 2, free : 1;
    uint8_t UID4;
    uint8_t UID5;
} OTA_Sync_t;

typedef struct PACKED {
    uint8_t type : 2, crcHigh : 6;
    union {
        struct {
            OTA_Channels_4x10_t ch;
            uint8_t switches : 7, isArmed : 1;
        } rc;
        OTA_Sync_t sync;
    };
    uint8_t crcLow;
} OTA_Packet4_t;

#define OTA_PACKET_SIZE 8
#define OTA_CRC_CALC_LEN (offsetof(OTA_Packet4_t, crcLow))

extern uint16_t OtaCrcInitializer;
extern volatile uint8_t OtaNonce;

void OtaInitCrcFromUid(const uint8_t* uid);
void OtaGenerateCrc(OTA_Packet4_t* pkt);
bool OtaValidateCrc(OTA_Packet4_t* pkt);

void OtaPackRcData(OTA_Packet4_t* pkt, const uint32_t* channelData);
bool OtaUnpackRcData(const OTA_Packet4_t* pkt, uint32_t* channelData);

void OtaPackSync(OTA_Packet4_t* pkt, uint8_t fhssIndex, uint8_t rfRateEnum, const uint8_t* uid);
