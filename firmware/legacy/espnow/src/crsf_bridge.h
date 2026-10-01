// CRSF Bridge - Interface between custom protocol and CRSF output
// Outputs CRSF RC channels to flight controller

#ifndef CRSF_BRIDGE_H
#define CRSF_BRIDGE_H

#include <Arduino.h>

// CRSF Protocol Constants
#define CRSF_BAUDRATE 420000
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_PAYLOAD_SIZE_MAX 62

// CRSF Frame Types
#define CRSF_FRAMETYPE_RC_CHANNELS 0x16
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08
#define CRSF_FRAMETYPE_GPS 0x02
#define CRSF_FRAMETYPE_ATTITUDE 0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21

// CRSF Addresses
#define CRSF_ADDRESS_BROADCAST 0x00
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA
#define CRSF_ADDRESS_RECEIVER 0xEC

// CRSF RC Channels (packed 11-bit)
#define CRSF_CHANNEL_VALUE_MIN 172
#define CRSF_CHANNEL_VALUE_MID 992
#define CRSF_CHANNEL_VALUE_MAX 1811

class CRSFBridge {
public:
    CRSFBridge();
    
    // Initialize CRSF on specified UART
    bool init(HardwareSerial* serial = &Serial1, int8_t txPin = -1, int8_t rxPin = -1);
    
    // Send RC channels to flight controller
    void sendRcChannels(uint16_t* channels);
    
    // Send telemetry frame (battery)
    void sendBattery(float voltage, float current, uint32_t capacity, uint8_t remaining);
    
    // Update - call this frequently to handle telemetry reception
    void update();
    
    // Check if CRSF is active
    bool isActive() { return active; }
    
    // Get statistics
    uint32_t getFramesSent() { return framesSent; }
    uint32_t getFramesReceived() { return framesReceived; }
    
private:
    HardwareSerial* serial;
    bool active;
    uint32_t framesSent;
    uint32_t framesReceived;
    unsigned long lastFrameMs;
    
    // Frame buffer
    uint8_t frameBuffer[CRSF_FRAME_SIZE_MAX];
    
    // Calculate CRC8 for CRSF
    uint8_t crc8(const uint8_t* data, uint8_t len);
    
    // Send raw CRSF frame
    void sendFrame(uint8_t* frame, uint8_t len);
    
    // Pack 11-bit RC channels into CRSF format
    void packRcChannels(uint16_t* channels, uint8_t* payload);
};

#endif // CRSF_BRIDGE_H

