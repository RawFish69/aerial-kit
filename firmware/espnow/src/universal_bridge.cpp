// Universal Protocol Bridge Implementation
// All RC output protocols implemented in one file for easy maintenance

#ifdef BUILD_RX  // Only compile for receiver

#include "universal_bridge.h"

// ============================================================
// Internal Protocol Implementations
// ============================================================

// --- CRSF Protocol ---
class CRSFProtocol : public ProtocolBase {
public:
    CRSFProtocol() : serial(&Serial1), active(false), framesSent(0), lastFrameMs(0) {}
    
    bool init() override {
        serial->begin(CRSF_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
        delay(100);
        active = true;
        
#if !ENABLE_DEBUG_OUTPUT
        Serial.printf("[CRSF] Initialized at %d baud | TX:%d RX:%d | Rate:%dHz\n",
                      CRSF_BAUDRATE, UART_TX_PIN, UART_RX_PIN, CRSF_OUTPUT_FREQUENCY_HZ);
#endif
        return true;
    }
    
    void sendRcChannels(uint16_t* channels) override {
        unsigned long now = millis();
        if (framesSent > 0 && (now - lastFrameMs < (1000 / CRSF_OUTPUT_FREQUENCY_HZ))) return;
        
        uint8_t frame[26];
        frame[0] = 0xC8;  // FC address
        frame[1] = 24;    // Length
        frame[2] = 0x16;  // RC channels type
        
        // Pack 16 channels (11-bit each) into 22 bytes
        packChannels11bit(channels, &frame[3]);
        
        frame[25] = crc8(&frame[2], 23);
        serial->write(frame, 26);
        
        framesSent++;
        lastFrameMs = now;
    }
    
    void update() override {}
    bool isActive() override { return active; }
    const char* getName() override { return "CRSF"; }
    uint32_t getFramesSent() override { return framesSent; }
    
private:
    HardwareSerial* serial;
    bool active;
    uint32_t framesSent;
    unsigned long lastFrameMs;
    
    void packChannels11bit(uint16_t* ch, uint8_t* out) {
        // Clamp channels to CRSF range
        for (int i = 0; i < 16; i++) ch[i] = clampCrsf(ch[i]);
        
        // Pack 11-bit channels (CRSF standard packing)
        out[0]  = ch[0] & 0xFF;
        out[1]  = (ch[0] >> 8) | ((ch[1] & 0x07) << 3);
        out[2]  = (ch[1] >> 5) | ((ch[2] & 0x3F) << 6);
        out[3]  = ch[2] >> 2;
        out[4]  = (ch[2] >> 10) | ((ch[3] & 0x1FF) << 1);
        out[5]  = (ch[3] >> 7) | ((ch[4] & 0x0F) << 4);
        out[6]  = (ch[4] >> 4) | ((ch[5] & 0x01) << 7);
        out[7]  = ch[5] >> 1;
        out[8]  = (ch[5] >> 9) | ((ch[6] & 0x3) << 2);
        out[9]  = (ch[6] >> 6) | ((ch[7] & 0x1F) << 5);
        out[10] = ch[7] >> 3;
        out[11] = ch[8] & 0xFF;
        out[12] = (ch[8] >> 8) | ((ch[9] & 0x07) << 3);
        out[13] = (ch[9] >> 5) | ((ch[10] & 0x3F) << 6);
        out[14] = ch[10] >> 2;
        out[15] = (ch[10] >> 10) | ((ch[11] & 0x1FF) << 1);
        out[16] = (ch[11] >> 7) | ((ch[12] & 0x0F) << 4);
        out[17] = (ch[12] >> 4) | ((ch[13] & 0x01) << 7);
        out[18] = ch[13] >> 1;
        out[19] = (ch[13] >> 9) | ((ch[14] & 0x3) << 2);
        out[20] = (ch[14] >> 6) | ((ch[15] & 0x1F) << 5);
        out[21] = ch[15] >> 3;
    }
    
    uint8_t crc8(const uint8_t* data, uint8_t len) {
        uint8_t crc = 0;
        for (uint8_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
                crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
            }
        }
        return crc;
    }
};

// --- SBUS Protocol ---
class SBUSProtocol : public ProtocolBase {
public:
    SBUSProtocol() : serial(&Serial1), active(false), framesSent(0), lastFrameMs(0) {}
    
    bool init() override {
        serial->begin(SBUS_BAUDRATE, SERIAL_8E2, UART_RX_PIN, UART_TX_PIN);
        delay(100);
        active = true;
        
#if !ENABLE_DEBUG_OUTPUT
        Serial.printf("[SBUS] Initialized at %d baud (8E2) | TX:%d | Rate:%dHz\n",
                      SBUS_BAUDRATE, UART_TX_PIN, SBUS_OUTPUT_FREQUENCY_HZ);
#endif
        return true;
    }
    
    void sendRcChannels(uint16_t* channels) override {
        unsigned long now = millis();
        if (framesSent > 0 && (now - lastFrameMs < (1000 / SBUS_OUTPUT_FREQUENCY_HZ))) return;
        
        uint8_t frame[25];
        frame[0] = 0x0F;  // Header
        
        // Convert CRSF (172-1811) to SBUS (0-2047)
        uint16_t ch[16];
        for (int i = 0; i < 16; i++) {
            ch[i] = map(clampCrsf(channels[i]), 172, 1811, 0, 2047);
        }
        
        // Pack 11-bit channels
        frame[1]  = ch[0] & 0xFF;
        frame[2]  = (ch[0] >> 8) | ((ch[1] & 0x07) << 3);
        frame[3]  = (ch[1] >> 5) | ((ch[2] & 0x3F) << 6);
        frame[4]  = ch[2] >> 2;
        frame[5]  = (ch[2] >> 10) | ((ch[3] & 0x1FF) << 1);
        frame[6]  = (ch[3] >> 7) | ((ch[4] & 0x0F) << 4);
        frame[7]  = (ch[4] >> 4) | ((ch[5] & 0x01) << 7);
        frame[8]  = ch[5] >> 1;
        frame[9]  = (ch[5] >> 9) | ((ch[6] & 0x3) << 2);
        frame[10] = (ch[6] >> 6) | ((ch[7] & 0x1F) << 5);
        frame[11] = ch[7] >> 3;
        frame[12] = ch[8] & 0xFF;
        frame[13] = (ch[8] >> 8) | ((ch[9] & 0x07) << 3);
        frame[14] = (ch[9] >> 5) | ((ch[10] & 0x3F) << 6);
        frame[15] = ch[10] >> 2;
        frame[16] = (ch[10] >> 10) | ((ch[11] & 0x1FF) << 1);
        frame[17] = (ch[11] >> 7) | ((ch[12] & 0x0F) << 4);
        frame[18] = (ch[12] >> 4) | ((ch[13] & 0x01) << 7);
        frame[19] = ch[13] >> 1;
        frame[20] = (ch[13] >> 9) | ((ch[14] & 0x3) << 2);
        frame[21] = (ch[14] >> 6) | ((ch[15] & 0x1F) << 5);
        frame[22] = ch[15] >> 3;
        
        frame[23] = 0x00;  // Flags
        frame[24] = 0x00;  // Footer
        
        serial->write(frame, 25);
        framesSent++;
        lastFrameMs = now;
    }
    
    void update() override {}
    bool isActive() override { return active; }
    const char* getName() override { return "SBUS"; }
    uint32_t getFramesSent() override { return framesSent; }
    
private:
    HardwareSerial* serial;
    bool active;
    uint32_t framesSent;
    unsigned long lastFrameMs;
};

// --- PPM Protocol ---
class PPMProtocol : public ProtocolBase {
public:
    PPMProtocol() : active(false), framesSent(0), timer(nullptr), currentChannel(0), isPulse(false) {
        for (int i = 0; i < 16; i++) channelData[i] = 992;
        instance = this;
    }
    
    ~PPMProtocol() {
        if (timer) {
            timerAlarmDisable(timer);
            timerEnd(timer);
        }
    }
    
    bool init() override {
        pinMode(PPM_OUTPUT_PIN, OUTPUT);
        digitalWrite(PPM_OUTPUT_PIN, LOW);
        
        // Setup hardware timer (1 MHz = 1μs resolution)
        timer = timerBegin(0, 80, true);
        timerAttachInterrupt(timer, &PPMProtocol::timerISR, true);
        timerAlarmWrite(timer, PPM_PULSE_LENGTH_US, true);
        timerAlarmEnable(timer);
        
        active = true;
        
#if !ENABLE_DEBUG_OUTPUT
        Serial.printf("[PPM] Initialized on GPIO%d | Frame:%dus | Channels:%d\n",
                      PPM_OUTPUT_PIN, PPM_FRAME_LENGTH_US, PPM_CHANNEL_COUNT);
#endif
        
        digitalWrite(PPM_OUTPUT_PIN, HIGH);  // Start with pulse
        return true;
    }
    
    void sendRcChannels(uint16_t* channels) override {
        noInterrupts();
        for (int i = 0; i < 16; i++) {
            channelData[i] = clampCrsf(channels[i]);
        }
        interrupts();
    }
    
    void update() override {}
    bool isActive() override { return active; }
    const char* getName() override { return "PPM"; }
    uint32_t getFramesSent() override { return framesSent; }
    
private:
    bool active;
    uint32_t framesSent;
    hw_timer_t* timer;
    volatile uint8_t currentChannel;
    volatile bool isPulse;
    uint16_t channelData[16];
    
    static PPMProtocol* instance;
    
    static void IRAM_ATTR timerISR() {
        if (!instance) return;
        
        if (instance->isPulse) {
            digitalWrite(PPM_OUTPUT_PIN, LOW);
            instance->isPulse = false;
            
            uint16_t channelTime;
            if (instance->currentChannel < PPM_CHANNEL_COUNT) {
                channelTime = instance->crsfToUs(instance->channelData[instance->currentChannel]);
                channelTime -= PPM_PULSE_LENGTH_US;
            } else {
                channelTime = PPM_FRAME_LENGTH_US - (PPM_CHANNEL_COUNT * 2000) - PPM_PULSE_LENGTH_US;
            }
            
            timerAlarmWrite(instance->timer, channelTime, true);
        } else {
            digitalWrite(PPM_OUTPUT_PIN, HIGH);
            instance->isPulse = true;
            instance->currentChannel++;
            
            if (instance->currentChannel > PPM_CHANNEL_COUNT) {
                instance->currentChannel = 0;
                instance->framesSent++;
            }
            
            timerAlarmWrite(instance->timer, PPM_PULSE_LENGTH_US, true);
        }
    }
};

PPMProtocol* PPMProtocol::instance = nullptr;

// --- iBus Protocol ---
class IBUSProtocol : public ProtocolBase {
public:
    IBUSProtocol() : serial(&Serial1), active(false), framesSent(0), lastFrameMs(0) {}
    
    bool init() override {
        serial->begin(IBUS_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
        delay(100);
        active = true;
        
#if !ENABLE_DEBUG_OUTPUT
        Serial.printf("[iBus] Initialized at %d baud | TX:%d | Rate:%dHz\n",
                      IBUS_BAUDRATE, UART_TX_PIN, IBUS_OUTPUT_FREQUENCY_HZ);
#endif
        return true;
    }
    
    void sendRcChannels(uint16_t* channels) override {
        unsigned long now = millis();
        if (framesSent > 0 && (now - lastFrameMs < (1000 / IBUS_OUTPUT_FREQUENCY_HZ))) return;
        
        uint8_t frame[32];
        frame[0] = 0x20;  // Header 1
        frame[1] = 0x40;  // Header 2
        
        // Pack 14 channels (iBus supports max 14)
        for (int i = 0; i < 14; i++) {
            uint16_t value = (i < 16) ? crsfToUs(clampCrsf(channels[i])) : 1500;
            frame[2 + (i * 2)] = value & 0xFF;
            frame[2 + (i * 2) + 1] = (value >> 8) & 0xFF;
        }
        
        // Checksum: 0xFFFF - sum
        uint16_t sum = 0xFFFF;
        for (int i = 0; i < 30; i++) {
            sum -= frame[i];
        }
        frame[30] = sum & 0xFF;
        frame[31] = (sum >> 8) & 0xFF;
        
        serial->write(frame, 32);
        framesSent++;
        lastFrameMs = now;
    }
    
    void update() override {}
    bool isActive() override { return active; }
    const char* getName() override { return "iBus"; }
    uint32_t getFramesSent() override { return framesSent; }
    
private:
    HardwareSerial* serial;
    bool active;
    uint32_t framesSent;
    unsigned long lastFrameMs;
};

// --- FrSky S.PORT Protocol ---
class FrSkyProtocol : public ProtocolBase {
public:
    FrSkyProtocol() : serial(&Serial1), active(false), framesSent(0), lastFrameMs(0), channelGroup(0) {}
    
    bool init() override {
        serial->begin(FRSKY_SPORT_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
        delay(100);
        active = true;
        
#if !ENABLE_DEBUG_OUTPUT
        Serial.printf("[FrSky] Initialized S.PORT at %d baud | TX:%d | Rate:%dHz\n",
                      FRSKY_SPORT_BAUDRATE, UART_TX_PIN, FRSKY_SPORT_OUTPUT_FREQUENCY_HZ);
#endif
        return true;
    }
    
    void sendRcChannels(uint16_t* channels) override {
        unsigned long now = millis();
        if (framesSent > 0 && (now - lastFrameMs < (1000 / FRSKY_SPORT_OUTPUT_FREQUENCY_HZ))) return;
        
        // Pack 4 channels into 32-bit value
        uint32_t data = 0;
        uint8_t baseChannel = channelGroup * 4;
        for (int i = 0; i < 4 && (baseChannel + i) < 16; i++) {
            uint16_t value = map(clampCrsf(channels[baseChannel + i]), 172, 1811, 0, 2047);
            data |= ((uint32_t)value << (i * 8));
        }
        
        // Send S.PORT frame
        serial->write(0x7E);  // Header
        sendByteStuffed(0x1B);  // Sensor ID
        sendByteStuffed(0x10);  // Frame ID
        uint16_t appId = 0x5100 + channelGroup;
        sendByteStuffed(appId & 0xFF);
        sendByteStuffed((appId >> 8) & 0xFF);
        sendByteStuffed(data & 0xFF);
        sendByteStuffed((data >> 8) & 0xFF);
        sendByteStuffed((data >> 16) & 0xFF);
        sendByteStuffed((data >> 24) & 0xFF);
        sendByteStuffed(calculateCRC());
        
        channelGroup = (channelGroup + 1) % 4;
        framesSent++;
        lastFrameMs = now;
    }
    
    void update() override {}
    bool isActive() override { return active; }
    const char* getName() override { return "FrSky S.PORT"; }
    uint32_t getFramesSent() override { return framesSent; }
    
private:
    HardwareSerial* serial;
    bool active;
    uint32_t framesSent;
    unsigned long lastFrameMs;
    uint8_t channelGroup;
    
    void sendByteStuffed(uint8_t byte) {
        if (byte == 0x7E || byte == 0x7D) {
            serial->write(0x7D);
            serial->write(byte ^ 0x20);
        } else {
            serial->write(byte);
        }
    }
    
    uint8_t calculateCRC() {
        // Simplified CRC for this example
        return 0xFF;
    }
};

// ============================================================
// Universal Bridge Implementation
// ============================================================

UniversalBridge::UniversalBridge() : activeProtocol(nullptr) {}

UniversalBridge::~UniversalBridge() {
    if (activeProtocol) {
        delete activeProtocol;
    }
}

ProtocolBase* UniversalBridge::createProtocol() {
    switch (OUTPUT_PROTOCOL) {
        case PROTOCOL_CRSF:
            return new CRSFProtocol();
        case PROTOCOL_SBUS:
            return new SBUSProtocol();
        case PROTOCOL_PPM:
            return new PPMProtocol();
        case PROTOCOL_IBUS:
            return new IBUSProtocol();
        case PROTOCOL_FRSKY_SPORT:
            return new FrSkyProtocol();
        default:
            return nullptr;
    }
}

bool UniversalBridge::init() {
    activeProtocol = createProtocol();
    if (!activeProtocol) return false;
    return activeProtocol->init();
}

void UniversalBridge::sendRcChannels(uint16_t* channels) {
    if (activeProtocol) {
        activeProtocol->sendRcChannels(channels);
    }
}

void UniversalBridge::update() {
    if (activeProtocol) {
        activeProtocol->update();
    }
}

bool UniversalBridge::isActive() {
    return activeProtocol ? activeProtocol->isActive() : false;
}

const char* UniversalBridge::getName() {
    return activeProtocol ? activeProtocol->getName() : "None";
}

uint32_t UniversalBridge::getFramesSent() {
    return activeProtocol ? activeProtocol->getFramesSent() : 0;
}

#endif // BUILD_RX

