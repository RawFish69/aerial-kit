// LoRa point-to-point template.
//
// Flash this same firmware onto two boards (give them different
// LORA_NODE_ID values, see config.h) and they'll heartbeat to each other
// over LoRa. It's meant as a starting point to build your own long-range
// telemetry/command link on top of -- swap HeartbeatPacket for your own
// struct and payload once the link itself is working.
//
// SX127x is half-duplex (one antenna, can't TX and RX at once), so this
// alternates: listen continuously via RadioLib's interrupt-driven receive,
// and briefly pause listening every LORA_HEARTBEAT_INTERVAL_MS to send our
// own heartbeat, then resume listening.
#include <Arduino.h>
#include <RadioLib.h>
#include "config.h"
#include "lora_protocol.h"

SX1278 radio = new Module(LORA_PIN_NSS, LORA_PIN_DIO0, LORA_PIN_RST, RADIOLIB_NC);

volatile bool packetReceivedFlag = false;

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void onPacketReceived() {
  packetReceivedFlag = true;
}

uint32_t txCounter = 0;
unsigned long lastHeartbeatMs = 0;

void sendHeartbeat() {
  LoraFrame frame;
  loraBuildHeartbeat(frame, LORA_NODE_ID, txCounter++, millis());

  // transmit() is blocking and reuses the DIO0 line for TX-done, which
  // would otherwise also fire our RX interrupt handler -- detach it for
  // the duration of the send so a completed transmit isn't mistaken for
  // a received packet.
  radio.clearPacketReceivedAction();
  int state = radio.transmit(reinterpret_cast<uint8_t*>(&frame), sizeof(frame));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] TX node=%u counter=%lu\n", frame.nodeId,
                  static_cast<unsigned long>(frame.sequence));
  } else {
    Serial.printf("[LoRa] TX failed, code %d\n", state);
  }

  // transmit() leaves the radio in standby; go back to listening.
  radio.setPacketReceivedAction(onPacketReceived);
  radio.startReceive();
}

void handleReceivedPacket() {
  packetReceivedFlag = false;

  LoraFrame frame;
  memset(&frame, 0, sizeof(frame));
  size_t receivedLength = radio.getPacketLength();
  int state = radio.readData(reinterpret_cast<uint8_t*>(&frame), sizeof(frame));

  if (state == RADIOLIB_ERR_NONE) {
    if (!loraValidate(frame, receivedLength)) {
      Serial.println("[LoRa] RX invalid frame");
      return;
    }

    uint32_t uptimeMs = 0;
    if (frame.type == LORA_PKT_HEARTBEAT && frame.payloadLength >= sizeof(uptimeMs)) {
      memcpy(&uptimeMs, frame.payload, sizeof(uptimeMs));
    } else if (frame.type == LORA_PKT_TELEMETRY &&
               frame.payloadLength >= sizeof(LoraTelemetryPayload)) {
      LoraTelemetryPayload telemetry{};
      memcpy(&telemetry, frame.payload, sizeof(telemetry));
      Serial.printf("[LoRa] TELEM node=%u voltage=%.2fV rssi=%d link=%u%% uptime=%lums\n",
                    frame.nodeId, telemetry.voltage, telemetry.rssi,
                    telemetry.linkQuality, static_cast<unsigned long>(telemetry.uptimeMs));
      return;
    }

    Serial.printf("[LoRa] RX node=%u counter=%lu uptime=%lums  RSSI=%.1fdBm SNR=%.1fdB\n",
                  frame.nodeId, static_cast<unsigned long>(frame.sequence),
                  static_cast<unsigned long>(uptimeMs), radio.getRSSI(), radio.getSNR());
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("[LoRa] RX CRC error");
  } else {
    Serial.printf("[LoRa] RX failed, code %d\n", state);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("LoRa template");

  int state = radio.begin(LORA_FREQ_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR,
                           LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER_DBM);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] begin() failed, code %d\n", state);
    while (true) {
      delay(1000);
    }
  }
  Serial.printf("[LoRa] node=%d freq=%.1fMHz bw=%.0fkHz sf=%d cr=4/%d\n", LORA_NODE_ID,
                LORA_FREQ_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR, LORA_CODING_RATE);

  radio.setPacketReceivedAction(onPacketReceived);
  radio.startReceive();
}

void loop() {
  if (packetReceivedFlag) {
    handleReceivedPacket();
  }

  if (millis() - lastHeartbeatMs >= LORA_HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = millis();
    sendHeartbeat();
  }
}
