// Custom Wireless Protocol

#ifndef CUSTOM_PROTOCOL_H
#define CUSTOM_PROTOCOL_H

#include <Arduino.h>

#define CUSTOM_PROTO_MAGIC      0xA5
#define CUSTOM_PROTO_VERSION    0x01
#define CUSTOM_PROTO_MAX_PAYLOAD 128

enum CustomPacketType {
  PKT_RC_COMMAND    = 0x10,  // RC channel commands
  PKT_DIRECT_CMD    = 0x11,  // Direct control commands
  PKT_TELEMETRY_REQ = 0x20,  // Telemetry request
  PKT_TELEMETRY_DATA= 0x21,  // Telemetry data
  PKT_CONFIG        = 0x30,  // Configuration
  PKT_ACK           = 0xF0,  // Acknowledgment
  PKT_NACK          = 0xF1,  // Negative acknowledgment
  PKT_HEARTBEAT     = 0xFF   // Heartbeat
};

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t version;
  uint8_t packetType;
  uint8_t sequence;
  uint16_t payloadLength;
  uint8_t payload[CUSTOM_PROTO_MAX_PAYLOAD];
  uint16_t crc16;
} CustomPacket;

typedef struct __attribute__((packed)) {
  uint16_t channels[16];   // CRSF format: 172-1811
  uint32_t timestamp;
  uint8_t flags;
} RcCommandPayload;

typedef struct __attribute__((packed)) {
  float roll;       // -1.0 to 1.0
  float pitch;      // -1.0 to 1.0
  float yaw;        // -1.0 to 1.0
  float throttle;   // 0.0 to 1.0
  uint32_t timestamp;
} DirectCommandPayload;

typedef struct __attribute__((packed)) {
  float voltage;
  float current;
  int16_t rssi;
  uint8_t linkQuality;
  uint32_t timestamp;
} TelemetryPayload;

typedef struct {
  uint32_t packetsReceived;
  uint32_t packetsSent;
  uint32_t crcErrors;
  uint32_t timeoutErrors;
  uint32_t lastPacketMs;
  bool linkActive;
  uint32_t sendFailures;
  float packetLossPercent;
} ProtocolStats;

bool CustomProtocol_Init(bool isTx = true, const uint8_t* peerMac = nullptr);
void CustomProtocol_Update();
bool CustomProtocol_ParsePacket(const uint8_t* data, size_t len, const uint8_t* senderMac);
bool CustomProtocol_SendPacket(CustomPacketType type, const uint8_t* payload, uint16_t payloadLen);
bool CustomProtocol_SendRcCommand(uint16_t* channels);
bool CustomProtocol_SendDirectCommand(float roll, float pitch, float yaw, float throttle);
bool CustomProtocol_SendTelemetry(float voltage, float current, int16_t rssi, uint8_t linkQuality);
bool CustomProtocol_SendHeartbeat();
void CustomProtocol_GetRcChannels(uint16_t* channels);
void CustomProtocol_GetTelemetry(TelemetryPayload* telemetry);
bool CustomProtocol_IsLinkActive();
void CustomProtocol_GetStats(ProtocolStats* stats);
uint16_t CustomProtocol_CRC16(const uint8_t* data, size_t len);

#endif // CUSTOM_PROTOCOL_H
