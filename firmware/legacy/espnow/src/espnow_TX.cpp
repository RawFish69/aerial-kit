// ESP-NOW Transmitter Implementation

#include "espnow_TX.h"

ESPNowTX* ESPNowTX::instance = nullptr;
static unsigned long lastTxErrorMs = 0;

ESPNowTX::ESPNowTX() 
    : initialized(false)
    , dataSentCallback(nullptr)
    , dataReceivedCallback(nullptr)
    , packetsSent(0)
    , packetsReceived(0)
    , sendFailures(0) {
    
    // Default broadcast address
    memset(receiverAddress, 0xFF, 6);
    instance = this;
}

ESPNowTX::~ESPNowTX() {
    if (initialized) {
        esp_now_deinit();
    }
    instance = nullptr;
}

bool ESPNowTX::init() {
    Serial.println("[ESP-NOW TX] Initializing...");
    Serial.flush();  // Ensure message is sent before WiFi changes
    delay(100);
    
    // Only deinit if already initialized (prevent crash)
    if (initialized) {
        esp_now_deinit();
        initialized = false;
    }
    
    // CRITICAL: If already in AP_STA mode, DO NOT TOUCH WiFi settings!
    // The AP must stay running for web interface
    wifi_mode_t currentMode = WiFi.getMode();
    if (currentMode == WIFI_AP_STA) {
        // Already in AP_STA mode - don't change ANYTHING about WiFi
        Serial.println("[ESP-NOW TX] WiFi in AP_STA mode - preserving AP, not touching WiFi settings");
        // Just verify AP is still running
        IPAddress apIP = WiFi.softAPIP();
        if (apIP.toString() == "0.0.0.0") {
            Serial.println("[ESP-NOW TX] WARNING: AP IP is 0.0.0.0 but mode is AP_STA!");
        }
    } else {
        // Not in AP_STA mode, safe to configure
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        delay(200);
        esp_wifi_set_ps(WIFI_PS_NONE);
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        delay(100);
    }
    
    // Disable WiFi power saving (safe even in AP_STA mode)
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    // Print MAC address
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("[ESP-NOW TX] MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.flush();
    
    // Initialize ESP-NOW
    esp_err_t result = esp_now_init();
    if (result != ESP_OK) {
        Serial.printf("[ESP-NOW TX] Init failed: %d\n", result);
        Serial.flush();
        return false;
    }
    
    // Register callbacks
    esp_now_register_recv_cb(onDataRecvStatic);
    esp_now_register_send_cb(onDataSentStatic);
    
    initialized = true;
    delay(50);
    Serial.println("[ESP-NOW TX] Initialized successfully on Channel 1");
    Serial.flush();
    
    return true;
}

void ESPNowTX::setOnDataSent(OnTxDataSentCallback callback) {
    dataSentCallback = callback;
}

void ESPNowTX::setOnDataReceived(OnTxDataReceivedCallback callback) {
    dataReceivedCallback = callback;
}

bool ESPNowTX::addReceiver(const uint8_t* receiverMac) {
    if (!initialized) {
        Serial.println("[ESP-NOW TX] Not initialized!");
        return false;
    }
    
    // Store receiver address
    memcpy(receiverAddress, receiverMac, 6);
    
    // Check if peer already exists
    if (esp_now_is_peer_exist(receiverMac)) {
        Serial.println("[ESP-NOW TX] Receiver already exists");
        return true;
    }
    
    // Add receiver as peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 0;  // Use current channel
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW TX] Failed to add receiver!");
        return false;
    }
    
    Serial.printf("[ESP-NOW TX] Added receiver: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  receiverMac[0], receiverMac[1], receiverMac[2], 
                  receiverMac[3], receiverMac[4], receiverMac[5]);
    
    return true;
}

bool ESPNowTX::send(const uint8_t* data, size_t len) {
    if (!initialized) {
        unsigned long now = millis();
        if (now - lastTxErrorMs >= 1000) {
            Serial.println("[ESP-NOW TX] Not initialized!");
            lastTxErrorMs = now;
        }
        return false;
    }
    
    if (len > ESP_NOW_MAX_DATA_LEN) {
        unsigned long now = millis();
        if (now - lastTxErrorMs >= 1000) {
            Serial.printf("[ESP-NOW TX] Data too large: %d bytes (max %d)\n", 
                         len, ESP_NOW_MAX_DATA_LEN);
            lastTxErrorMs = now;
        }
        return false;
    }
    
    esp_err_t result = esp_now_send(receiverAddress, data, len);
    
    if (result != ESP_OK) {
        sendFailures++;
        
        // If error is ESP_ERR_ESPNOW_NOT_INIT, try to reinitialize
        if (result == ESP_ERR_ESPNOW_NOT_INIT) {
            unsigned long now = millis();
            if (now - lastTxErrorMs >= 1000) {
                Serial.println("[ESP-NOW TX] Lost init, attempting recovery...");
                lastTxErrorMs = now;
            }
            initialized = false;
            // Caller should reinit
        } else {
            unsigned long now = millis();
            if (now - lastTxErrorMs >= 1000) {
                Serial.printf("[ESP-NOW TX] Send failed: %d\n", result);
                lastTxErrorMs = now;
            }
        }
        return false;
    }
    
    return true;
}

void ESPNowTX::getMacAddress(uint8_t* mac) {
    WiFi.macAddress(mac);
}

void ESPNowTX::onDataRecvStatic(const uint8_t* mac, const uint8_t* data, int len) {
    if (instance && instance->dataReceivedCallback) {
        instance->packetsReceived++;
        instance->dataReceivedCallback(data, len, mac);
    }
}

void ESPNowTX::onDataSentStatic(const uint8_t* mac, esp_now_send_status_t status) {
    if (instance) {
        bool success = (status == ESP_NOW_SEND_SUCCESS);
        
        if (success) {
            instance->packetsSent++;
        } else {
            instance->sendFailures++;
        }
        
        if (instance->dataSentCallback) {
            instance->dataSentCallback(mac, success);
        }
    }
}

