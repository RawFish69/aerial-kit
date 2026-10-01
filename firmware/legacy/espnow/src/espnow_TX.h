// ESP-NOW Transmitter (TX)

#ifndef ESPNOW_TX_H
#define ESPNOW_TX_H

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// Callback for when data is sent
typedef void (*OnTxDataSentCallback)(const uint8_t* mac, bool success);

// Callback for when data is received (for telemetry, acks, etc.)
typedef void (*OnTxDataReceivedCallback)(const uint8_t* data, size_t len, const uint8_t* mac);

class ESPNowTX {
public:
    ESPNowTX();
    ~ESPNowTX();
    
    // Initialize ESP-NOW for transmitter
    bool init();
    
    // Set callbacks
    void setOnDataSent(OnTxDataSentCallback callback);
    void setOnDataReceived(OnTxDataReceivedCallback callback);
    
    // Add receiver peer
    bool addReceiver(const uint8_t* receiverMac);
    
    // Send data to receiver
    bool send(const uint8_t* data, size_t len);
    
    // Get local MAC address
    void getMacAddress(uint8_t* mac);
    
    // Get statistics
    uint32_t getPacketsSent() const { return packetsSent; }
    uint32_t getPacketsReceived() const { return packetsReceived; }
    uint32_t getSendFailures() const { return sendFailures; }
    
    // Static callbacks (required by ESP-NOW API)
    static void onDataRecvStatic(const uint8_t* mac, const uint8_t* data, int len);
    static void onDataSentStatic(const uint8_t* mac, esp_now_send_status_t status);
    
private:
    bool initialized;
    uint8_t receiverAddress[6];
    
    OnTxDataSentCallback dataSentCallback;
    OnTxDataReceivedCallback dataReceivedCallback;
    
    uint32_t packetsSent;
    uint32_t packetsReceived;
    uint32_t sendFailures;
    
    static ESPNowTX* instance;
};

#endif // ESPNOW_TX_H

