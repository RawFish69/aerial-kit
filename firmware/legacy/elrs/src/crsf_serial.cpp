#include "crsf_serial.h"

CrsfSerial Crsf;

CrsfSerial::CrsfSerial()
    : serial_(nullptr)
    , hasNewChannels_(false)
    , rxLen_(0) {
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++)
        channels_[i] = CRSF_CHANNEL_VALUE_MID;
}

void CrsfSerial::begin(HardwareSerial* ser, int8_t txPin, int8_t rxPin) {
    serial_ = ser;
    if (txPin >= 0 && rxPin >= 0)
        serial_->begin(CRSF_UART_BAUD, SERIAL_8N1, rxPin, txPin);
    else
        serial_->begin(CRSF_UART_BAUD);
}

uint8_t CrsfSerial::crc8(const uint8_t* data, uint8_t len) {
    // CRC8 DVB-S2 poly 0xD5 (same as CRSF spec)
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc <<= 1;
        }
    }
    return crc;
}

void CrsfSerial::packRcChannels(const uint16_t* ch, uint8_t* payload) {
    uint16_t v[16];
    for (int i = 0; i < 16; i++) {
        v[i] = ch[i];
        if (v[i] < CRSF_CHANNEL_VALUE_MIN) v[i] = CRSF_CHANNEL_VALUE_MIN;
        if (v[i] > CRSF_CHANNEL_VALUE_MAX) v[i] = CRSF_CHANNEL_VALUE_MAX;
    }
    payload[0]  = v[0] & 0xFF;
    payload[1]  = (v[0] >> 8) | (uint8_t)((v[1] & 0x07) << 3);
    payload[2]  = (v[1] >> 5) | (uint8_t)((v[2] & 0x3F) << 6);
    payload[3]  = v[2] >> 2;
    payload[4]  = (v[2] >> 10) | (uint8_t)((v[3] & 0x01FF) << 1);
    payload[5]  = (v[3] >> 7) | (uint8_t)((v[4] & 0x0F) << 4);
    payload[6]  = (v[4] >> 4) | (uint8_t)((v[5] & 0x01) << 7);
    payload[7]  = v[5] >> 1;
    payload[8]  = (v[5] >> 9) | (uint8_t)((v[6] & 0x3) << 2);
    payload[9]  = (v[6] >> 6) | (uint8_t)((v[7] & 0x1F) << 5);
    payload[10] = v[7] >> 3;
    payload[11] = v[8] & 0xFF;
    payload[12] = (v[8] >> 8) | (uint8_t)((v[9] & 0x07) << 3);
    payload[13] = (v[9] >> 5) | (uint8_t)((v[10] & 0x3F) << 6);
    payload[14] = v[10] >> 2;
    payload[15] = (v[10] >> 10) | (uint8_t)((v[11] & 0x01FF) << 1);
    payload[16] = (v[11] >> 7) | (uint8_t)((v[12] & 0x0F) << 4);
    payload[17] = (v[12] >> 4) | (uint8_t)((v[13] & 0x01) << 7);
    payload[18] = v[13] >> 1;
    payload[19] = (v[13] >> 9) | (uint8_t)((v[14] & 0x3) << 2);
    payload[20] = (v[14] >> 6) | (uint8_t)((v[15] & 0x1F) << 5);
    payload[21] = v[15] >> 3;
}

void CrsfSerial::unpackRcChannels(const uint8_t* payload, uint16_t* ch) {
    ch[0]  = payload[0] | ((uint16_t)(payload[1] & 0x07) << 8);
    ch[1]  = (payload[1] >> 3) | ((uint16_t)(payload[2] & 0x3F) << 5);
    ch[2]  = (payload[2] >> 6) | ((uint16_t)payload[3] << 2) | ((uint16_t)(payload[4] & 0x03) << 10);
    ch[3]  = (payload[4] >> 2) | ((uint16_t)(payload[5] & 0x1F) << 6);
    ch[4]  = (payload[5] >> 5) | ((uint16_t)(payload[6] & 0x7F) << 3);
    ch[5]  = (payload[6] >> 7) | ((uint16_t)payload[7] << 1) | ((uint16_t)(payload[8] & 0x01) << 9);
    ch[6]  = (payload[8] >> 1) | ((uint16_t)(payload[9] & 0x3F) << 5);
    ch[7]  = (payload[9] >> 6) | ((uint16_t)payload[10] << 3) | ((uint16_t)(payload[11] & 0x03) << 11);
    ch[8]  = (payload[11] >> 2) | ((uint16_t)(payload[12] & 0x1F) << 6);
    ch[9]  = (payload[12] >> 5) | ((uint16_t)(payload[13] & 0x7F) << 3);
    ch[10] = (payload[13] >> 7) | ((uint16_t)payload[14] << 1) | ((uint16_t)(payload[15] & 0x01) << 9);
    ch[11] = (payload[15] >> 1) | ((uint16_t)(payload[16] & 0x3F) << 5);
    ch[12] = (payload[16] >> 6) | ((uint16_t)payload[17] << 3) | ((uint16_t)(payload[18] & 0x03) << 11);
    ch[13] = (payload[18] >> 2) | ((uint16_t)(payload[19] & 0x1F) << 6);
    ch[14] = (payload[19] >> 6) | ((uint16_t)payload[20] << 3) | ((uint16_t)(payload[21] & 0x03) << 11);
    ch[15] = (payload[21] >> 2);
}

void CrsfSerial::update() {
    if (!serial_) return;
    while (serial_->available()) {
        uint8_t b = serial_->read();
        if (rxLen_ == 0 && b != CRSF_SYNC_BYTE) continue;
        rxBuf_[rxLen_++] = b;
        if (rxLen_ >= 2) {
            uint8_t frameLen = rxBuf_[1];
            uint8_t totalLen = 2 + frameLen + 1;  // addr+len + payload+crc
            if (totalLen > sizeof(rxBuf_)) { rxLen_ = 0; continue; }
            if (rxLen_ >= totalLen) {
                uint8_t crc = crc8(&rxBuf_[2], frameLen);
                if (crc == rxBuf_[2 + frameLen] && rxBuf_[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && frameLen >= 23) {
                    unpackRcChannels(&rxBuf_[3], channels_);
                    hasNewChannels_ = true;
                }
                rxLen_ = 0;
            }
        }
    }
}

void CrsfSerial::getChannels(uint16_t* ch) {
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++)
        ch[i] = channels_[i];
}

void CrsfSerial::sendRcChannels(const uint16_t* ch) {
    if (!serial_) return;
    uint8_t frame[4 + 22 + 1];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 23;  // type + 22 payload + crc
    frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    packRcChannels(ch, &frame[3]);
    frame[25] = crc8(&frame[2], 23);
    serial_->write(frame, sizeof(frame));
}
