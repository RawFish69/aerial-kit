#pragma once

#include <Arduino.h>

#include "control_input.h"

void rcInputInit(HardwareSerial& serial, uint32_t baud, bool invert = false);

// Parse a CRSF RC_CHANNELS_PACKED frame from the supplied stream and map the
// first four channels into roll/pitch/yaw/throttle.
//
// Channel mapping matches the rest of aerial-kit:
//   CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw
bool rcInputReadCrsf(Stream& stream, ControlInput& out);

// Parse an SBUS frame (25 bytes, 16 channels) and map the first four channels
// into roll/pitch/yaw/throttle. SBUS UART is inverted; the init helper sets
// inversion on ESP32 and expects an external inverter on STM32.
bool rcInputReadSbus(Stream& stream, ControlInput& out);
