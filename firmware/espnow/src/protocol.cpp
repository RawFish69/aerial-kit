// Custom Protocol Implementation

#include "protocol.h"
#include "espnow_TX.h"
#include "espnow_RX.h"

static uint8_t sequenceNumber = 0;
static uint32_t lastPacketReceivedMs = 0;
static uint32_t lastPacketSentMs = 0;

// RC channel storage
static uint16_t currentRcChannels[16] = {
  992, 992, 172, 992, 172, 992, 992, 992,
  992, 992, 992, 992, 992, 992, 992, 992
};

static ProtocolStats stats = {0};
static TelemetryPayload lastTelemetry = {0};
static ESPNowTX espnowTX;
static ESPNowRX espnowRX;
static uint8_t peerMacAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool isTxMode = true;
static unsigned long lastProtocolErrorMs = 0;
static uint32_t totalSendAttempts = 0;

uint16_t CustomProtocol_CRC16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// Callback for received data
void onESPNowDataReceived(const uint8_t* data, size_t len, const uint8_t* mac) {
  CustomProtocol_ParsePacket(data, len, mac);
}

// Callback for sent data
void onESPNowDataSent(const uint8_t* mac, bool success) {
  if (!success) {
    Serial.println("[Custom] Send failed!");
  }
}

bool CustomProtocol_Init(bool isTx, const uint8_t* peerMac) {
  Serial.println("[Custom] Initializing...");
  
  isTxMode = isTx;
  
  stats.packetsReceived = 0;
  stats.packetsSent = 0;
  stats.crcErrors = 0;
  stats.timeoutErrors = 0;
  stats.lastPacketMs = 0;
  stats.linkActive = false;
  
  memset(&lastTelemetry, 0, sizeof(lastTelemetry));
  
  // Initialize ESP-NOW based on mode
  if (isTxMode) {
    // Initialize transmitter
    if (!espnowTX.init()) {
      Serial.println("[Custom] ESP-NOW TX init failed!");
      return false;
    }
    
    // Set callbacks
    espnowTX.setOnDataReceived(onESPNowDataReceived);
    espnowTX.setOnDataSent(onESPNowDataSent);
    
    // Add receiver peer
    if (peerMac != nullptr) {
      memcpy(peerMacAddress, peerMac, 6);
      espnowTX.addReceiver(peerMac);
      Serial.printf("[Custom] Added receiver: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    peerMac[0], peerMac[1], peerMac[2], peerMac[3], peerMac[4], peerMac[5]);
    } else {
      // Use broadcast address
      memset(peerMacAddress, 0xFF, 6);
      espnowTX.addReceiver(peerMacAddress);
      Serial.println("[Custom] Using broadcast address");
    }
  } else {
    // Initialize receiver
    if (!espnowRX.init()) {
      Serial.println("[Custom] ESP-NOW RX init failed!");
      return false;
    }
    
    // Set callbacks
    espnowRX.setOnDataReceived(onESPNowDataReceived);
    espnowRX.setOnDataSent(onESPNowDataSent);
    
    // Receiver will auto-add transmitter when first packet arrives
  }
  
  Serial.println("[Custom] Initialized");
  return true;
}

bool CustomProtocol_ParsePacket(const uint8_t* data, size_t len, const uint8_t* senderMac) {
  if (len < sizeof(CustomPacket) - CUSTOM_PROTO_MAX_PAYLOAD) {
    return false;
  }
  
  const CustomPacket* pkt = reinterpret_cast<const CustomPacket*>(data);
  
  if (pkt->magic != CUSTOM_PROTO_MAGIC) {
    return false;
  }
  
  if (pkt->version != CUSTOM_PROTO_VERSION) {
    Serial.printf("[Custom] Version mismatch: expected 0x%02X, got 0x%02X\n", 
                  CUSTOM_PROTO_VERSION, pkt->version);
    return false;
  }
  
  size_t expectedLen = 6 + pkt->payloadLength + 2;
  if (len < expectedLen) {
    return false;
  }
  
  // CRC verification
  uint16_t receivedCrc = (uint16_t)data[expectedLen - 2] | ((uint16_t)data[expectedLen - 1] << 8);
  uint16_t calculatedCrc = CustomProtocol_CRC16(data, expectedLen - 2);
  
  if (receivedCrc != calculatedCrc) {
    stats.crcErrors++;
    return false;
  }
  
  // Parse packet
  switch (pkt->packetType) {
    case PKT_RC_COMMAND: {
      if (pkt->payloadLength >= sizeof(RcCommandPayload)) {
        const RcCommandPayload* rcCmd = reinterpret_cast<const RcCommandPayload*>(pkt->payload);
        
        for (int i = 0; i < 16; i++) {
          uint16_t val = rcCmd->channels[i];
          if (val < 172) val = 172;
          if (val > 1811) val = 1811;
          currentRcChannels[i] = val;
        }
        
        lastPacketReceivedMs = millis();
        stats.packetsReceived++;
        stats.linkActive = true;
      }
      break;
    }
    
    case PKT_DIRECT_CMD: {
      if (pkt->payloadLength >= sizeof(DirectCommandPayload)) {
        const DirectCommandPayload* directCmd = reinterpret_cast<const DirectCommandPayload*>(pkt->payload);
        
        // Convert normalized commands to RC channels
        currentRcChannels[0] = (uint16_t)((directCmd->roll + 1.0f) * 819.5f + 172.0f);
        currentRcChannels[1] = (uint16_t)((directCmd->pitch + 1.0f) * 819.5f + 172.0f);
        currentRcChannels[2] = (uint16_t)(directCmd->throttle * 1639.0f + 172.0f);
        // Invert yaw to match Betaflight's expected direction.
        float invertedYaw = -directCmd->yaw;
        currentRcChannels[3] = (uint16_t)((invertedYaw + 1.0f) * 819.5f + 172.0f);
        
        for (int i = 0; i < 4; i++) {
          if (currentRcChannels[i] < 172) currentRcChannels[i] = 172;
          if (currentRcChannels[i] > 1811) currentRcChannels[i] = 1811;
        }
        
        lastPacketReceivedMs = millis();
        stats.packetsReceived++;
        stats.linkActive = true;
      }
      break;
    }
    
    case PKT_HEARTBEAT: {
      lastPacketReceivedMs = millis();
      stats.packetsReceived++;
      stats.linkActive = true;
      break;
    }
    
    case PKT_TELEMETRY_DATA: {
      if (pkt->payloadLength >= sizeof(TelemetryPayload)) {
        const TelemetryPayload* telemetry = reinterpret_cast<const TelemetryPayload*>(pkt->payload);
        memcpy(&lastTelemetry, telemetry, sizeof(TelemetryPayload));
        
        lastPacketReceivedMs = millis();
        stats.packetsReceived++;
        stats.linkActive = true;
      }
      break;
    }
    
    default:
      break;
  }
  
  return true;
}

void CustomProtocol_Update() {
  uint32_t now = millis();
  
  // Timeout detection (1 second)
  if (stats.linkActive && (now - lastPacketReceivedMs > 1000)) {
    stats.linkActive = false;
    stats.timeoutErrors++;
    Serial.println("[Custom] Link timeout - failsafe");
    
    // Failsafe (only for receiver)
    if (!isTxMode) {
      currentRcChannels[0] = 992;
      currentRcChannels[1] = 992;
      currentRcChannels[2] = 172;  // Throttle low
      currentRcChannels[3] = 992;
      for (int i = 4; i < 16; i++) {
        currentRcChannels[i] = 172;
      }
    }
  }
  
  stats.lastPacketMs = now - lastPacketReceivedMs;
}

bool CustomProtocol_SendPacket(CustomPacketType type, const uint8_t* payload, uint16_t payloadLen) {
  if (payloadLen > CUSTOM_PROTO_MAX_PAYLOAD) {
    return false;
  }
  
  uint8_t txBuffer[sizeof(CustomPacket)];
  CustomPacket* pkt = reinterpret_cast<CustomPacket*>(txBuffer);
  
  pkt->magic = CUSTOM_PROTO_MAGIC;
  pkt->version = CUSTOM_PROTO_VERSION;
  pkt->packetType = type;
  pkt->sequence = sequenceNumber++;
  pkt->payloadLength = payloadLen;
  
  if (payload != nullptr && payloadLen > 0) {
    memcpy(pkt->payload, payload, payloadLen);
  }
  
  size_t dataLen = 6 + payloadLen;
  uint16_t crc = CustomProtocol_CRC16(txBuffer, dataLen);
  txBuffer[dataLen] = crc & 0xFF;
  txBuffer[dataLen + 1] = (crc >> 8) & 0xFF;
  
  // Send via ESP-NOW based on mode
  size_t totalLen = dataLen + 2;
  bool sendSuccess = false;
  
  totalSendAttempts++;  // Track total attempts
  
  if (isTxMode) {
    sendSuccess = espnowTX.send(txBuffer, totalLen);
  } else {
    sendSuccess = espnowRX.sendToTransmitter(txBuffer, totalLen);
  }
  
  if (!sendSuccess) {
    stats.sendFailures++;
    
    // Calculate packet loss percentage
    stats.packetLossPercent = (float)stats.sendFailures / (float)totalSendAttempts * 100.0f;
    
    // Rate-limit error messages (max once per second)
    unsigned long now = millis();
    if (now - lastProtocolErrorMs >= 1000) {
      Serial.println("[Custom] Send failed!");
      lastProtocolErrorMs = now;
    }
    return false;
  }
  
  lastPacketSentMs = millis();
  stats.packetsSent++;
  
  // Calculate packet loss percentage
  stats.packetLossPercent = (float)stats.sendFailures / (float)totalSendAttempts * 100.0f;
  
  return true;
}

bool CustomProtocol_SendRcCommand(uint16_t* channels) {
  RcCommandPayload payload;
  for (int i = 0; i < 16; i++) {
    payload.channels[i] = channels[i];
  }
  payload.timestamp = millis();
  payload.flags = 0;
  
  return CustomProtocol_SendPacket(PKT_RC_COMMAND, 
                                   reinterpret_cast<uint8_t*>(&payload), 
                                   sizeof(payload));
}

bool CustomProtocol_SendDirectCommand(float roll, float pitch, float yaw, float throttle) {
  DirectCommandPayload payload;
  payload.roll = roll;
  payload.pitch = pitch;
  payload.yaw = yaw;
  payload.throttle = throttle;
  payload.timestamp = millis();
  
  return CustomProtocol_SendPacket(PKT_DIRECT_CMD, 
                                   reinterpret_cast<uint8_t*>(&payload), 
                                   sizeof(payload));
}

void CustomProtocol_GetRcChannels(uint16_t* channels) {
  for (int i = 0; i < 16; i++) {
    channels[i] = currentRcChannels[i];
  }
}

bool CustomProtocol_IsLinkActive() {
  return stats.linkActive;
}

bool CustomProtocol_SendTelemetry(float voltage, float current, int16_t rssi, uint8_t linkQuality) {
  TelemetryPayload payload;
  payload.voltage = voltage;
  payload.current = current;
  payload.rssi = rssi;
  payload.linkQuality = linkQuality;
  payload.timestamp = millis();
  
  return CustomProtocol_SendPacket(PKT_TELEMETRY_DATA, 
                                   reinterpret_cast<uint8_t*>(&payload), 
                                   sizeof(payload));
}

bool CustomProtocol_SendHeartbeat() {
  return CustomProtocol_SendPacket(PKT_HEARTBEAT, nullptr, 0);
}

void CustomProtocol_GetTelemetry(TelemetryPayload* telemetry) {
  memcpy(telemetry, &lastTelemetry, sizeof(TelemetryPayload));
}

void CustomProtocol_GetStats(ProtocolStats* stats_out) {
  memcpy(stats_out, &stats, sizeof(ProtocolStats));
}
