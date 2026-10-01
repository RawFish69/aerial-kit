// CRSF Bridge Implementation

#include "crsf_bridge.h"

CRSFBridge::CRSFBridge() 
    : serial(nullptr)
    , active(false)
    , framesSent(0)
    , framesReceived(0)
    , lastFrameMs(0) {
}

bool CRSFBridge::init(HardwareSerial* ser, int8_t txPin, int8_t rxPin) {
    serial = ser;
    
    Serial.println("[CRSF] Initializing...");
    Serial.flush();
    delay(50);
    
    // Make sure we're not using Serial (debug port)
    if (serial == &Serial) {
        Serial.println("[CRSF] ERROR: Cannot use Serial for CRSF (conflicts with debug output)");
        Serial.flush();
        return false;
    }
    
    // Configure UART with explicit pins (ESP32-C3 needs this)
    if (txPin >= 0 && rxPin >= 0) {
        serial->begin(CRSF_BAUDRATE, SERIAL_8N1, rxPin, txPin);
    } else {
        serial->begin(CRSF_BAUDRATE);
    }
    
    delay(200);  // Give Serial1 time to initialize
    
    active = true;
    Serial.printf("[CRSF] Initialized on UART at %d baud\n", CRSF_BAUDRATE);
    
    if (txPin >= 0) Serial.printf("[CRSF] TX Pin: GPIO%d\n", txPin);
    if (rxPin >= 0) Serial.printf("[CRSF] RX Pin: GPIO%d\n", rxPin);
    Serial.flush();
    delay(50);
    
    return true;
}

void CRSFBridge::update() {
    if (!active || !serial) return;
    
    // TODO: Read telemetry from FC if needed
    // For now, we only send RC channels to FC
    
    // Check for timeout
    unsigned long now = millis();
    if (lastFrameMs > 0 && (now - lastFrameMs > 1000)) {
        // No frames sent in 1 second - link might be down
    }
}

void CRSFBridge::sendRcChannels(uint16_t* channels) {
    if (!active || !serial) return;
    
    // CRSF RC Channels Frame:
    // [Address][Length][Type][22 bytes of packed channels][CRC]
    
    uint8_t frame[26];
    uint8_t* payload = &frame[2];
    
    // Header
    frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;  // Destination
    frame[1] = 24;  // Length (Type + Payload + CRC)
    payload[0] = CRSF_FRAMETYPE_RC_CHANNELS;    // Frame type
    
    // Pack 16 channels (11-bit each) into 22 bytes
    packRcChannels(channels, &payload[1]);
    
    // Calculate CRC (over Type + Payload)
    uint8_t crc = crc8(&frame[2], 23);
    frame[25] = crc;
    
    // Send frame
    sendFrame(frame, 26);
    
    framesSent++;
    lastFrameMs = millis();
}

void CRSFBridge::sendBattery(float voltage, float current, uint32_t capacity, uint8_t remaining) {
    if (!active || !serial) return;
    
    // CRSF Battery Sensor Frame:
    // [Address][Length][Type][Voltage(2)][Current(2)][Capacity(3)][Remaining(1)][CRC]
    
    uint8_t frame[13];
    uint8_t* payload = &frame[2];
    
    // Header
    frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame[1] = 10;  // Length
    payload[0] = CRSF_FRAMETYPE_BATTERY_SENSOR;
    
    // Voltage (in 0.1V, big-endian)
    uint16_t voltageValue = (uint16_t)(voltage * 10.0f);
    payload[1] = (voltageValue >> 8) & 0xFF;
    payload[2] = voltageValue & 0xFF;
    
    // Current (in 0.1A, big-endian)
    uint16_t currentValue = (uint16_t)(current * 10.0f);
    payload[3] = (currentValue >> 8) & 0xFF;
    payload[4] = currentValue & 0xFF;
    
    // Capacity (in mAh, big-endian, 24-bit)
    payload[5] = (capacity >> 16) & 0xFF;
    payload[6] = (capacity >> 8) & 0xFF;
    payload[7] = capacity & 0xFF;
    
    // Remaining (in percent)
    payload[8] = remaining;
    
    // CRC
    uint8_t crc = crc8(&frame[2], 9);
    frame[12] = crc;
    
    // Send frame
    sendFrame(frame, 13);
    
    framesSent++;
}

void CRSFBridge::packRcChannels(uint16_t* channels, uint8_t* payload) {
    // Pack 16 channels (11-bit each) into 22 bytes
    // Based on working reference code - exact bit packing
    
    // Clamp all channels
    uint16_t ch[16];
    for (int i = 0; i < 16; i++) {
        ch[i] = channels[i];
        if (ch[i] < CRSF_CHANNEL_VALUE_MIN) ch[i] = CRSF_CHANNEL_VALUE_MIN;
        if (ch[i] > CRSF_CHANNEL_VALUE_MAX) ch[i] = CRSF_CHANNEL_VALUE_MAX;
    }
    
    // Pack according to CRSF specification (11 bits per channel)
    payload[0]  = ch[0] & 0xFF;
    payload[1]  = (ch[0] >> 8) | (uint8_t)((ch[1] & 0x07) << 3);
    payload[2]  = (ch[1] >> 5) | (uint8_t)((ch[2] & 0x3F) << 6);
    payload[3]  = ch[2] >> 2;
    payload[4]  = (ch[2] >> 10) | (uint8_t)((ch[3] & 0x01FF) << 1);
    payload[5]  = (ch[3] >> 7) | (uint8_t)((ch[4] & 0x0F) << 4);
    payload[6]  = (ch[4] >> 4) | (uint8_t)((ch[5] & 0x01) << 7);
    payload[7]  = ch[5] >> 1;
    payload[8]  = (ch[5] >> 9) | (uint8_t)((ch[6] & 0x3) << 2);
    payload[9]  = (ch[6] >> 6) | (uint8_t)((ch[7] & 0x1F) << 5);
    payload[10] = ch[7] >> 3;
    
    payload[11] = ch[8] & 0xFF;
    payload[12] = (ch[8] >> 8) | (uint8_t)((ch[9] & 0x07) << 3);
    payload[13] = (ch[9] >> 5) | (uint8_t)((ch[10] & 0x3F) << 6);
    payload[14] = ch[10] >> 2;
    payload[15] = (ch[10] >> 10) | (uint8_t)((ch[11] & 0x01FF) << 1);
    payload[16] = (ch[11] >> 7) | (uint8_t)((ch[12] & 0x0F) << 4);
    payload[17] = (ch[12] >> 4) | (uint8_t)((ch[13] & 0x01) << 7);
    payload[18] = ch[13] >> 1;
    payload[19] = (ch[13] >> 9) | (uint8_t)((ch[14] & 0x3) << 2);
    payload[20] = (ch[14] >> 6) | (uint8_t)((ch[15] & 0x1F) << 5);
    payload[21] = ch[15] >> 3;
}

void CRSFBridge::sendFrame(uint8_t* frame, uint8_t len) {
    if (!serial) return;
    
    // Send the frame
    serial->write(frame, len);
}

uint8_t CRSFBridge::crc8(const uint8_t* data, uint8_t len) {
    // CRC8 DVB-S2 polynomial: x^8 + x^7 + x^6 + x^4 + x^2 + x + 1
    uint8_t crc = 0;
    
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

