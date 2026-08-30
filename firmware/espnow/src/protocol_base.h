// Protocol Base Class - Simplified interface for all output protocols
// All protocols inherit from this and implement the required methods

#ifndef PROTOCOL_BASE_H
#define PROTOCOL_BASE_H

#include <Arduino.h>

// Base class for all output protocols
class ProtocolBase {
public:
    virtual ~ProtocolBase() {}
    
    // Initialize the protocol
    virtual bool init() = 0;
    
    // Send RC channels to flight controller
    // channels: array of 16 channels (CRSF format: 172-1811)
    virtual void sendRcChannels(uint16_t* channels) = 0;
    
    // Update - call frequently to handle protocol timing
    virtual void update() = 0;
    
    // Check if protocol is active
    virtual bool isActive() = 0;
    
    // Get protocol name
    virtual const char* getName() = 0;
    
    // Get statistics
    virtual uint32_t getFramesSent() = 0;
    
protected:
    // Helper: Convert CRSF value (172-1811) to microseconds (1000-2000)
    uint16_t crsfToUs(uint16_t crsfValue) {
        return map(crsfValue, 172, 1811, 1000, 2000);
    }
    
    // Helper: Clamp CRSF value to valid range
    uint16_t clampCrsf(uint16_t value) {
        if (value < 172) return 172;
        if (value > 1811) return 1811;
        return value;
    }
};

#endif // PROTOCOL_BASE_H

