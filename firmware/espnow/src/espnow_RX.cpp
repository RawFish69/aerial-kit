// ESP-NOW Receiver Implementation

#include "espnow_RX.h"

ESPNowRX* ESPNowRX::instance = nullptr;
static unsigned long lastRxErrorMs = 0;

ESPNowRX::ESPNowRX() 
    : initialized(false)
    , hasTransmitter(false)
    , dataReceivedCallback(nullptr)
    , dataSentCallback(nullptr)
    , packetsReceived(0)
    , packetsSent(0)
    , sendFailures(0) {
    
    memset(transmitterAddress, 0, 6);
    instance = this;
}

ESPNowRX::~ESPNowRX() {
    if (initialized) {
        esp_now_deinit();
    }
    instance = nullptr;
}

bool ESPNowRX::init() {
    Serial.println("[ESP-NOW RX] Initializing...");
    Serial.flush();  // Ensure message is sent before WiFi changes
    delay(100);
    
    // Only deinit if already initialized (prevent crash)
    if (initialized) {
        esp_now_deinit();
        initialized = false;
    }
    
    // Set WiFi mode to station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(200);  // Give WiFi time to stabilize
    
    // Disable WiFi power saving for better stability
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    // Set WiFi to channel 1 for better stability
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    delay(100);
    
    // Print MAC address
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("[ESP-NOW RX] MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.flush();
    
    // Initialize ESP-NOW
    esp_err_t result = esp_now_init();
    if (result != ESP_OK) {
        Serial.printf("[ESP-NOW RX] Init failed: %d\n", result);
        Serial.flush();
        return false;
    }
    
    // Register callbacks
    esp_now_register_recv_cb(onDataRecvStatic);
    esp_now_register_send_cb(onDataSentStatic);
    
    initialized = true;
    delay(50);
    Serial.println("[ESP-NOW RX] Initialized successfully on Channel 1");
    Serial.println("[ESP-NOW RX] Ready to receive data...");
    Serial.flush();
    
    return true;
}

void ESPNowRX::setOnDataReceived(OnRxDataReceivedCallback callback) {
    dataReceivedCallback = callback;
}

void ESPNowRX::setOnDataSent(OnRxDataSentCallback callback) {
    dataSentCallback = callback;
}

bool ESPNowRX::addTransmitter(const uint8_t* transmitterMac) {
    if (!initialized) {
        Serial.println("[ESP-NOW RX] Not initialized!");
        return false;
    }
    
    // Store transmitter address
    memcpy(transmitterAddress, transmitterMac, 6);
    
    // Check if peer already exists
    if (esp_now_is_peer_exist(transmitterMac)) {
        Serial.println("[ESP-NOW RX] Transmitter already exists");
        hasTransmitter = true;
        return true;
    }
    
    // Add transmitter as peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, transmitterMac, 6);
    peerInfo.channel = 0;  // Use current channel
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW RX] Failed to add transmitter!");
        return false;
    }
    
    hasTransmitter = true;
    Serial.printf("[ESP-NOW RX] Added transmitter: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  transmitterMac[0], transmitterMac[1], transmitterMac[2], 
                  transmitterMac[3], transmitterMac[4], transmitterMac[5]);
    
    return true;
}

bool ESPNowRX::sendToTransmitter(const uint8_t* data, size_t len) {
    if (!initialized) {
        unsigned long now = millis();
        if (now - lastRxErrorMs >= 1000) {
            Serial.println("[ESP-NOW RX] Not initialized!");
            lastRxErrorMs = now;
        }
        return false;
    }
    
    if (!hasTransmitter) {
        unsigned long now = millis();
        if (now - lastRxErrorMs >= 1000) {
            Serial.println("[ESP-NOW RX] No transmitter configured!");
            lastRxErrorMs = now;
        }
        return false;
    }
    
    if (len > ESP_NOW_MAX_DATA_LEN) {
        unsigned long now = millis();
        if (now - lastRxErrorMs >= 1000) {
            Serial.printf("[ESP-NOW RX] Data too large: %d bytes (max %d)\n", 
                         len, ESP_NOW_MAX_DATA_LEN);
            lastRxErrorMs = now;
        }
        return false;
    }
    
    esp_err_t result = esp_now_send(transmitterAddress, data, len);
    
    if (result != ESP_OK) {
        sendFailures++;
        
        // If error is ESP_ERR_ESPNOW_NOT_INIT, try to reinitialize
        if (result == ESP_ERR_ESPNOW_NOT_INIT) {
            unsigned long now = millis();
            if (now - lastRxErrorMs >= 1000) {
                Serial.println("[ESP-NOW RX] Lost init, attempting recovery...");
                lastRxErrorMs = now;
            }
            initialized = false;
            // Caller should reinit
        } else {
            unsigned long now = millis();
            if (now - lastRxErrorMs >= 1000) {
                Serial.printf("[ESP-NOW RX] Send failed: %d\n", result);
                lastRxErrorMs = now;
            }
        }
        return false;
    }
    
    return true;
}

void ESPNowRX::getMacAddress(uint8_t* mac) {
    WiFi.macAddress(mac);
}

void ESPNowRX::onDataRecvStatic(const uint8_t* mac, const uint8_t* data, int len) {
    if (instance) {
        instance->packetsReceived++;
        
        // Auto-add transmitter if not already added
        if (!instance->hasTransmitter) {
            instance->addTransmitter(mac);
        }
        
        if (instance->dataReceivedCallback) {
            instance->dataReceivedCallback(data, len, mac);
        }
    }
}

void ESPNowRX::onDataSentStatic(const uint8_t* mac, esp_now_send_status_t status) {
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

