#include "ota.h"
#include "config.h"
#include <string.h>

#define ELRS_CRC_POLY 0x07
#define OTA_VERSION_ID 1

uint16_t OtaCrcInitializer = 0;
volatile uint8_t OtaNonce = 0;

static uint8_t crc8_dvb(uint8_t* data, uint8_t len, uint8_t init) {
    uint8_t crc = init;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 1)
                crc = (crc >> 1) ^ ELRS_CRC_POLY;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void OtaInitCrcFromUid(const uint8_t* uid) {
    OtaCrcInitializer = ((uint16_t)uid[4] << 8) | uid[5];
    OtaCrcInitializer ^= (uint16_t)OTA_VERSION_ID << 8;
}

void OtaGenerateCrc(OTA_Packet4_t* pkt) {
    uint16_t nonceVal = (pkt->type == PACKET_TYPE_SYNC) ? 0 : OtaNonce;
    uint16_t init = OtaCrcInitializer ^ (nonceVal << 8);
    uint8_t crc = crc8_dvb((uint8_t*)pkt, OTA_CRC_CALC_LEN, init & 0xFF);
    crc ^= (init >> 8) & 0xFF;
    pkt->crcLow = crc;
    pkt->type = (pkt->type & 0x03) | 0;  // crcHigh = 0 for 8-bit CRC
}

bool OtaValidateCrc(OTA_Packet4_t* pkt) {
    (void)(pkt->type >> 2);  // discard crcHigh
    pkt->type &= 0x03;
    uint16_t nonceVal = (pkt->type == PACKET_TYPE_SYNC) ? 0 : OtaNonce;
    uint16_t init = OtaCrcInitializer ^ (nonceVal << 8);
    uint8_t crc = crc8_dvb((uint8_t*)pkt, OTA_CRC_CALC_LEN, init & 0xFF);
    crc ^= (init >> 8) & 0xFF;
    return crc == pkt->crcLow;
}

static void packChannels4x10(OTA_Channels_4x10_t* ch, const uint32_t* channelData) {
    // CRSF 11-bit -> 10-bit for OTA: use 10 bits per channel (0..1023)
    uint32_t c0 = (channelData[0] * 1023) / (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN + 1);
    uint32_t c1 = (channelData[1] * 1023) / (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN + 1);
    uint32_t c2 = (channelData[2] * 1023) / (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN + 1);
    uint32_t c3 = (channelData[3] * 1023) / (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN + 1);
    if (c0 > 1023) c0 = 1023;
    if (c1 > 1023) c1 = 1023;
    if (c2 > 1023) c2 = 1023;
    if (c3 > 1023) c3 = 1023;
    ch->raw[0] = c0 & 0xFF;
    ch->raw[1] = (c0 >> 8) | ((c1 & 0x3F) << 2);
    ch->raw[2] = (c1 >> 6) | ((c2 & 0x0F) << 4);
    ch->raw[3] = (c2 >> 4) | ((c3 & 0x03) << 6);
    ch->raw[4] = c3 >> 2;
}

static void unpackChannels4x10(const OTA_Channels_4x10_t* ch, uint32_t* channelData) {
    uint32_t c0 = ch->raw[0] | ((ch->raw[1] & 0x03) << 8);
    uint32_t c1 = (ch->raw[1] >> 2) | ((ch->raw[2] & 0x0F) << 6);
    uint32_t c2 = (ch->raw[2] >> 4) | ((ch->raw[3] & 0x3F) << 4);
    uint32_t c3 = (ch->raw[3] >> 6) | (ch->raw[4] << 2);
    channelData[0] = CRSF_CHANNEL_VALUE_MIN + (c0 * (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN + 1)) / 1023;
    channelData[1] = CRSF_CHANNEL_VALUE_MIN + (c1 * (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN + 1)) / 1023;
    channelData[2] = CRSF_CHANNEL_VALUE_MIN + (c2 * (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN + 1)) / 1023;
    channelData[3] = CRSF_CHANNEL_VALUE_MIN + (c3 * (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN + 1)) / 1023;
}

void OtaPackRcData(OTA_Packet4_t* pkt, const uint32_t* channelData) {
    pkt->type = PACKET_TYPE_RCDATA;
    packChannels4x10(&pkt->rc.ch, channelData);
    pkt->rc.switches = 0;
    pkt->rc.isArmed = 0;
}

bool OtaUnpackRcData(const OTA_Packet4_t* pkt, uint32_t* channelData) {
    if ((pkt->type & 0x03) != PACKET_TYPE_RCDATA) return false;
    unpackChannels4x10(&pkt->rc.ch, channelData);
    return true;
}

void OtaPackSync(OTA_Packet4_t* pkt, uint8_t fhssIndex, uint8_t rfRateEnum, const uint8_t* uid) {
    pkt->type = PACKET_TYPE_SYNC;
    pkt->sync.fhssIndex = fhssIndex;
    pkt->sync.nonce = OtaNonce;
    pkt->sync.rfRateEnum = rfRateEnum;
    pkt->sync.switchEncMode = 0;
    pkt->sync.newTlmRatio = 0;
    pkt->sync.geminiMode = 0;
    pkt->sync.otaProtocol = 0;
    pkt->sync.free = 0;
    pkt->sync.UID4 = uid[4];
    pkt->sync.UID5 = uid[5];
}
