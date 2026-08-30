#include "radio_sx1280.h"
#include "fhss.h"

static Module* mod = nullptr;
static SX1280* sx = nullptr;

RadioSX1280 Radio;

RadioSX1280::RadioSX1280() : radio_(nullptr), mod_(nullptr), ok_(false) {}

bool RadioSX1280::begin(const uint8_t* uid) {
    (void)uid;
    mod = new Module(RADIO_PIN_NSS, RADIO_PIN_DIO1, RADIO_PIN_RST, RADIO_PIN_BUSY);
    mod_ = mod;
    sx = new SX1280(mod);
    radio_ = sx;

    if (mod->init() != 0) return false;

    pinMode(RADIO_PIN_BUSY, INPUT);
    pinMode(RADIO_PIN_DIO1, INPUT);
    pinMode(RADIO_PIN_RST, OUTPUT);
    digitalWrite(RADIO_PIN_RST, LOW);
    delay(10);
    digitalWrite(RADIO_PIN_RST, HIGH);
    delay(5);

    // FLRC: 650 kbps, CR 1/2, preamble 32 (ExpressLRS-like). Same defaults on TX/RX = same link.
    if (sx->beginFLRC(2400.0, 650, RADIOLIB_SX1280_FLRC_CR_1_2, 10, 32, RADIOLIB_SX1280_FLRC_BT_0_5) != 0)
        return false;

    if (sx->setPacketLength(ELRS_PACKET_SIZE) != 0) return false;

    ok_ = true;
    return true;
}

void RadioSX1280::setFrequency(uint32_t regFreq) {
    if (!ok_ || !sx) return;
    // SX1280: freq = 52e6/2^18 * reg ≈ 198.36 Hz per step. 2400 MHz = base.
    float freqMHz = 2400.0f + (regFreq * 198.364f / 1e6f);
    sx->setFrequency(freqMHz);
}

void RadioSX1280::transmit(uint8_t* data, uint8_t len) {
    if (!ok_ || !sx) return;
    sx->transmit(data, len);
}

void RadioSX1280::startReceive() {
    if (!ok_ || !sx) return;
    sx->startReceive();
}

bool RadioSX1280::readPacket(uint8_t* data, uint8_t len) {
    if (!ok_ || !sx) return false;
    int16_t s = sx->readData(data, len);
    return s == 0;
}

bool RadioSX1280::isTxDone() {
    if (!ok_ || !sx) return false;
    return sx->isTxDone();
}

bool RadioSX1280::isRxDone() {
    if (!ok_ || !sx) return false;
    return sx->isRxDone();
}

void RadioSX1280::clearIrq() {
    if (sx) sx->clearIrqStatus();
}
