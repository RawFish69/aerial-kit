// Simple interval timer for ELRS packet rate (ESP32-C3)
#pragma once

#include "config.h"

void HwTimerInit(void (*callback)());
void HwTimerStart();
void HwTimerStop();
void HwTimerSetIntervalUs(uint32_t us);
