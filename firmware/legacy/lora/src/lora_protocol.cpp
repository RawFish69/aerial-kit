#include "lora_protocol.h"

#include <string.h>

static uint16_t crc16Update(uint16_t crc, uint8_t byte) {
  crc ^= static_cast<uint16_t>(byte) << 8;
  for (int i = 0; i < 8; ++i) {
    if (crc & 0x8000) {
      crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
    } else {
      crc = static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

uint16_t loraCrc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc = crc16Update(crc, data[i]);
  }
  return crc;
}

bool loraBuildHeartbeat(LoraFrame& frame, uint8_t nodeId, uint32_t sequence,
                        uint32_t uptimeMs) {
  memset(&frame, 0, sizeof(frame));
  frame.magic = LORA_FRAME_MAGIC;
  frame.version = LORA_FRAME_VERSION;
  frame.type = LORA_PKT_HEARTBEAT;
  frame.nodeId = nodeId;
  frame.sequence = sequence;

  struct __attribute__((packed)) HeartbeatPayload {
    uint32_t uptimeMs;
  } payload{uptimeMs};

  frame.payloadLength = sizeof(payload);
  memcpy(frame.payload, &payload, sizeof(payload));
  frame.crc16 = loraCrc16(reinterpret_cast<const uint8_t*>(&frame),
                          offsetof(LoraFrame, crc16));
  return true;
}

bool loraBuildTelemetry(LoraFrame& frame, uint8_t nodeId, uint32_t sequence,
                        float voltage, int16_t rssi, uint8_t linkQuality,
                        uint32_t uptimeMs) {
  memset(&frame, 0, sizeof(frame));
  frame.magic = LORA_FRAME_MAGIC;
  frame.version = LORA_FRAME_VERSION;
  frame.type = LORA_PKT_TELEMETRY;
  frame.nodeId = nodeId;
  frame.sequence = sequence;

  LoraTelemetryPayload payload{voltage, rssi, linkQuality, uptimeMs};

  frame.payloadLength = sizeof(payload);
  memcpy(frame.payload, &payload, sizeof(payload));
  frame.crc16 = loraCrc16(reinterpret_cast<const uint8_t*>(&frame),
                          offsetof(LoraFrame, crc16));
  return true;
}

bool loraValidate(const LoraFrame& frame, size_t receivedLength) {
  if (receivedLength < offsetof(LoraFrame, crc16)) {
    return false;
  }
  if (frame.magic != LORA_FRAME_MAGIC || frame.version != LORA_FRAME_VERSION) {
    return false;
  }
  if (frame.payloadLength > sizeof(frame.payload)) {
    return false;
  }

  uint16_t expected = loraCrc16(reinterpret_cast<const uint8_t*>(&frame),
                                offsetof(LoraFrame, crc16));
  return expected == frame.crc16;
}
