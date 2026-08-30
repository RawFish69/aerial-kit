// ELRS Receiver: RF -> OTA unpack -> UART CRSF out
#if defined(BUILD_ELRS_RX)

#include "config.h"
#include "crsf_serial.h"
#include "ota.h"
#include "fhss.h"
#include "radio_sx1280.h"
#include "hw_timer.h"
#include <Arduino.h>

static uint8_t s_uid[ELRS_UID_LEN];
static uint32_t s_channelData[CRSF_NUM_CHANNELS];
static uint16_t s_crsfChannels[CRSF_NUM_CHANNELS];
static bool s_locked = false;
static uint32_t s_lastPacketMs = 0;

static void rxTimerCallback() {
    if (!s_locked) return;
    Crsf.sendRcChannels(s_crsfChannels);  // Send at packet rate when locked
}

void ELRS_RX_Setup() {
    memset(s_uid, 0, ELRS_UID_LEN);
    size_t len = strlen(ELRS_BIND_PHRASE);
    if (len > ELRS_UID_LEN) len = ELRS_UID_LEN;
    memcpy(s_uid, ELRS_BIND_PHRASE, len);

    OtaInitCrcFromUid(s_uid);
    FHSSInit(((uint32_t)s_uid[0] << 24) | ((uint32_t)s_uid[1] << 16) | ((uint32_t)s_uid[2] << 8) | s_uid[3]);

    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        s_channelData[i] = CRSF_CHANNEL_VALUE_MID;
        s_crsfChannels[i] = CRSF_CHANNEL_VALUE_MID;
    }

    Crsf.begin(&Serial1, CRSF_PIN_TX, CRSF_PIN_RX);

    if (!Radio.begin(s_uid)) {
        Serial.println("[ELRS RX] Radio init failed");
        return;
    }
    Radio.setFrequency(FHSSGetInitialFreq());
    Radio.startReceive();

    HwTimerInit(rxTimerCallback);
    HwTimerSetIntervalUs(ELRS_PACKET_INTERVAL_US);
    HwTimerStart();
    Serial.println("[ELRS RX] Started, waiting for sync");
}

void ELRS_RX_Loop() {
    if (Radio.isRxDone()) {
        OTA_Packet4_t pkt;
        if (Radio.readPacket((uint8_t*)&pkt, ELRS_PACKET_SIZE) && OtaValidateCrc(&pkt)) {
            s_lastPacketMs = millis();
            if ((pkt.type & 0x03) == PACKET_TYPE_SYNC) {
                FHSSSetCurrIndex(pkt.sync.fhssIndex);
                OtaNonce = pkt.sync.nonce;
                s_locked = true;
            } else if ((pkt.type & 0x03) == PACKET_TYPE_RCDATA && s_locked) {
                if (OtaUnpackRcData(&pkt, s_channelData)) {
                    for (int i = 0; i < CRSF_NUM_CHANNELS; i++)
                        s_crsfChannels[i] = (uint16_t)s_channelData[i];
                }
                OtaNonce++;
                if (OtaNonce % ELRS_FHSS_HOP_INTERVAL == 0)
                    Radio.setFrequency(FHSSGetNextFreq());
                else
                    Radio.setFrequency(FHSSGetFreqAt(FHSSGetCurrIndex()));
            }
        }
        Radio.startReceive();
    }

    if (s_locked && (millis() - s_lastPacketMs > 500))
        s_locked = false;
}

#endif
