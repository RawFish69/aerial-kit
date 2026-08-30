// ELRS Transmitter and Receiver for ESP32-C3
// Computer (UART CRSF) -> TX -> RF (ELRS) -> RX -> Flight controller (UART CRSF)

#include <Arduino.h>
#include "config.h"

#if defined(BUILD_ELRS_TX)
extern void ELRS_TX_Setup();
extern void ELRS_TX_Loop();
#elif defined(BUILD_ELRS_RX)
extern void ELRS_RX_Setup();
extern void ELRS_RX_Loop();
#else
#error "Define BUILD_ELRS_TX or BUILD_ELRS_RX in platformio.ini"
#endif

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("ELRS UAV-Controller");

#if defined(BUILD_ELRS_TX)
    ELRS_TX_Setup();
#else
    ELRS_RX_Setup();
#endif
}

void loop() {
#if defined(BUILD_ELRS_TX)
    ELRS_TX_Loop();
#else
    ELRS_RX_Loop();
#endif
}
