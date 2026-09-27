#include "rc_input.h"

#include "config.h"

namespace {

constexpr uint8_t kCrsfSync = 0xC8;
constexpr uint8_t kCrsfRcChannelsPacked = 0x16;
constexpr uint8_t kCrsfRcPayloadLength = 22;
constexpr uint16_t kCrsfChannelMin = 172;
constexpr uint16_t kCrsfChannelMid = 992;
constexpr uint16_t kCrsfChannelMax = 1811;

uint8_t crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x80) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }
  return crc;
}

void unpackChannels(const uint8_t* p, uint16_t* ch) {
  ch[0] = p[0] | ((uint16_t)(p[1] & 0x07) << 8);
  ch[1] = (p[1] >> 3) | ((uint16_t)(p[2] & 0x3F) << 5);
  ch[2] = (p[2] >> 6) | ((uint16_t)p[3] << 2) | ((uint16_t)(p[4] & 0x03) << 10);
  ch[3] = (p[4] >> 2) | ((uint16_t)(p[5] & 0x1F) << 6);
}

float channelToNormalized(uint16_t value, bool throttle) {
  if (throttle) {
    return constrain((value - kCrsfChannelMin) /
                         (float)(kCrsfChannelMax - kCrsfChannelMin),
                     0.0f, 1.0f);
  }
  float centered = (float)value - kCrsfChannelMid;
  float span = kCrsfChannelMax - kCrsfChannelMid;
  return constrain(centered / span, -1.0f, 1.0f);
}

uint8_t rxBuffer[64];
uint8_t rxLength = 0;
uint8_t sbusBuffer[25];
uint8_t sbusLength = 0;

}  // namespace

void rcInputInit(HardwareSerial& serial, uint32_t baud, bool invert) {
#if RC_INPUT_PROTOCOL != 0
#if defined(ARDUINO_ARCH_ESP32)
  serial.begin(baud, SERIAL_8N1, RC_RX_PIN, RC_TX_PIN, invert);
#elif defined(ARDUINO_ARCH_STM32)
  (void)invert;
  serial.begin(baud);
#else
  (void)invert;
  serial.begin(baud);
#endif
#else
  (void)serial;
  (void)baud;
  (void)invert;
#endif
}

bool rcInputReadCrsf(Stream& stream, ControlInput& out) {
  while (stream.available() > 0) {
    uint8_t byte = static_cast<uint8_t>(stream.read());
    if (rxLength == 0 && byte != kCrsfSync) {
      continue;
    }
    rxBuffer[rxLength++] = byte;
    if (rxLength >= 2) {
      uint8_t frameLength = rxBuffer[1];
      uint8_t totalLength = static_cast<uint8_t>(2 + frameLength + 1);
      if (totalLength > sizeof(rxBuffer)) {
        rxLength = 0;
        continue;
      }
      if (rxLength >= totalLength) {
        bool valid = (rxBuffer[2] == kCrsfRcChannelsPacked &&
                      frameLength >= kCrsfRcPayloadLength &&
                      crc8(&rxBuffer[2], frameLength) == rxBuffer[2 + frameLength]);
        rxLength = 0;
        if (!valid) {
          continue;
        }

        uint16_t channels[4];
        unpackChannels(&rxBuffer[3], channels);
        out.roll = channelToNormalized(channels[0], false);
        out.pitch = channelToNormalized(channels[1], false);
        out.throttle = channelToNormalized(channels[2], true);
        out.yaw = channelToNormalized(channels[3], false);
        return true;
      }
    }
  }
  return false;
}

bool rcInputReadSbus(Stream& stream, ControlInput& out) {
  while (stream.available() > 0) {
    uint8_t byte = static_cast<uint8_t>(stream.read());
    if (sbusLength == 0 && byte != 0x0F) {
      continue;
    }
    sbusBuffer[sbusLength++] = byte;
    if (sbusLength == 25) {
      sbusLength = 0;
      if (sbusBuffer[0] != 0x0F || sbusBuffer[24] != 0x00) {
        continue;
      }

      uint16_t channels[16];
      channels[0] = ((uint16_t)sbusBuffer[1] | ((uint16_t)sbusBuffer[2] << 8)) & 0x07FF;
      channels[1] = (((uint16_t)sbusBuffer[2] >> 3) | ((uint16_t)sbusBuffer[3] << 5)) & 0x07FF;
      channels[2] = (((uint16_t)sbusBuffer[3] >> 6) | ((uint16_t)sbusBuffer[4] << 2) |
                     ((uint16_t)sbusBuffer[5] << 10)) & 0x07FF;
      channels[3] = (((uint16_t)sbusBuffer[5] >> 1) | ((uint16_t)sbusBuffer[6] << 7)) & 0x07FF;

      out.roll = channelToNormalized(channels[0], false);
      out.pitch = channelToNormalized(channels[1], false);
      out.throttle = channelToNormalized(channels[2], true);
      out.yaw = channelToNormalized(channels[3], false);
      return true;
    }
  }
  return false;
}
