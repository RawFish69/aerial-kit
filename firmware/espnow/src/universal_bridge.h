// Universal Protocol Bridge - All RC output protocols in one place
// Supports: CRSF, SBUS, PPM, iBus, FrSky S.PORT

#ifndef UNIVERSAL_BRIDGE_H
#define UNIVERSAL_BRIDGE_H

#ifdef BUILD_RX  // Only compile for receiver

#include "protocol_base.h"
#include "config.h"

// Forward declarations for internal protocol handlers
class CRSFProtocol;
class SBUSProtocol;
class PPMProtocol;
class IBUSProtocol;
class FrSkyProtocol;

// Universal bridge that wraps all protocols
class UniversalBridge : public ProtocolBase {
public:
    UniversalBridge();
    ~UniversalBridge();
    
    // ProtocolBase interface
    bool init() override;
    void sendRcChannels(uint16_t* channels) override;
    void update() override;
    bool isActive() override;
    const char* getName() override;
    uint32_t getFramesSent() override;
    
private:
    ProtocolBase* activeProtocol;
    
    // Create the appropriate protocol based on config
    ProtocolBase* createProtocol();
};

#endif // BUILD_RX
#endif // UNIVERSAL_BRIDGE_H

