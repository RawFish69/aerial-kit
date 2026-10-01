#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#define LORA_FRAME_MAGIC 0x4C  // 'L'
#define LORA_FRAME_VERSION 1

enum LoraPacketType : uint8_t {
  LORA_PKT_HEARTBEAT = 0x01,
  LORA_PKT_TELEMETRY = 0x02,
  LORA_PKT_COMMAND = 0x03,
};

struct __attribute__((packed)) LoraFrame {
  uint8_t magic;
  uint8_t version;
  uint8_t type;
  uint8_t nodeId;
  uint32_t sequence;
  uint16_t payloadLength;
  uint8_t payload[64];
  uint16_t crc16;
};

struct __attribute__((packed)) LoraTelemetryPayload {
  float voltage;
  int16_t rssi;
  uint8_t linkQuality;
  uint32_t uptimeMs;
};

uint16_t loraCrc16(const uint8_t* data, size_t len);
bool loraBuildHeartbeat(LoraFrame& frame, uint8_t nodeId, uint32_t sequence,
                        uint32_t uptimeMs);
bool loraBuildTelemetry(LoraFrame& frame, uint8_t nodeId, uint32_t sequence,
                        float voltage, int16_t rssi, uint8_t linkQuality,
                        uint32_t uptimeMs);
bool loraValidate(const LoraFrame& frame, size_t receivedLength);
