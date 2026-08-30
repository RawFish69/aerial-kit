// ESP-NOW Receiver (RX)

#ifndef ESPNOW_RX_H
#define ESPNOW_RX_H

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// Callback for when data is received
typedef void (*OnRxDataReceivedCallback)(const uint8_t* data, size_t len, const uint8_t* mac);

// Callback for when data is sent (for telemetry, acks, etc.)
typedef void (*OnRxDataSentCallback)(const uint8_t* mac, bool success);

class ESPNowRX {
public:
    ESPNowRX();
    ~ESPNowRX();
    
    // Initialize ESP-NOW for receiver
    bool init();
    
    // Set callbacks
    void setOnDataReceived(OnRxDataReceivedCallback callback);
    void setOnDataSent(OnRxDataSentCallback callback);
    
    // Add transmitter peer (for sending telemetry back)
    bool addTransmitter(const uint8_t* transmitterMac);
    
    // Send data back to transmitter (telemetry, etc.)
    bool sendToTransmitter(const uint8_t* data, size_t len);
    
    // Get local MAC address
    void getMacAddress(uint8_t* mac);
    
    // Get statistics
    uint32_t getPacketsReceived() const { return packetsReceived; }
    uint32_t getPacketsSent() const { return packetsSent; }
    uint32_t getSendFailures() const { return sendFailures; }
    
    // Static callbacks (required by ESP-NOW API)
    static void onDataRecvStatic(const uint8_t* mac, const uint8_t* data, int len);
    static void onDataSentStatic(const uint8_t* mac, esp_now_send_status_t status);
    
private:
    bool initialized;
    uint8_t transmitterAddress[6];
    bool hasTransmitter;
    
    OnRxDataReceivedCallback dataReceivedCallback;
    OnRxDataSentCallback dataSentCallback;
    
    uint32_t packetsReceived;
    uint32_t packetsSent;
    uint32_t sendFailures;
    
    static ESPNowRX* instance;
};

#endif // ESPNOW_RX_H

