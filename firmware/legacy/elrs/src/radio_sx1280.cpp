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

    mod->init();

    pinMode(RADIO_PIN_BUSY, INPUT);
    pinMode(RADIO_PIN_DIO1, INPUT);
    pinMode(RADIO_PIN_RST, OUTPUT);
    digitalWrite(RADIO_PIN_RST, LOW);
    delay(10);
    digitalWrite(RADIO_PIN_RST, HIGH);
    delay(5);

    // FLRC: 650 kbps, CR 1/2, preamble 32 (ExpressLRS-like). Same defaults on TX/RX = same link.
    if (sx->beginFLRC(2400.0, 650, RADIOLIB_SX128X_FLRC_CR_1_2, 10, 32, RADIOLIB_SX128X_FLRC_BT_0_5) != 0)
        return false;

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
    return (sx->getIrqStatus() & RADIOLIB_SX128X_IRQ_TX_DONE) != 0;
}

bool RadioSX1280::isRxDone() {
    if (!ok_ || !sx) return false;
    return (sx->getIrqStatus() & RADIOLIB_SX128X_IRQ_RX_DONE) != 0;
}

void RadioSX1280::clearIrq() {
    // RadioLib 6.x exposes getIrqStatus() but not a public clear method for
    // SX128x; this wrapper is currently unused, so reading the status is enough.
    if (sx) sx->getIrqStatus();
}
