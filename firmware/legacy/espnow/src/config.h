// ============================================================
// UNIFIED CONFIGURATION FILE
// ============================================================
// All configurable parameters for TX/RX in one place
// Modify these settings to configure your system

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// TRANSMITTER (TX) CONFIGURATION
// ============================================================
#ifdef BUILD_TX

// -------- Control Mode Selection --------
// Set TX_CONTROL_MODE to :
// 1 - for the web-based Quad Controller UI
// 0 - to drive the transmitter using the IMU + potentiometer hardware.
#define TX_CONTROL_MODE 0

#if TX_CONTROL_MODE != 0 && TX_CONTROL_MODE != 1
  #error "TX_CONTROL_MODE must be 0 (IMU) or 1 (Web)"
#endif

// -------- TX Communication --------
#define RC_SEND_FREQUENCY_HZ 100        // Options: 50, 100, 150, 250, 500 Hz
#define HEARTBEAT_INTERVAL 500          // Heartbeat interval in ms (2 Hz)
#define STATS_INTERVAL 1000             // Statistics print interval in ms

#if TX_CONTROL_MODE == 0
// -------- IMU & Potentiometer Configuration --------
#define IMU_TYPE_MPU6050 0
#define IMU_TYPE_BNO085 1
#define IMU_TYPE IMU_TYPE_BNO085         // Choose default IMU type
#define IMU_SDA_PIN 4
#define IMU_SCL_PIN 5
#define ROLL_SENSITIVITY 45.0            // Tilt degrees for full deflection
#define PITCH_SENSITIVITY 45.0           // Tilt degrees for full deflection
#define YAW_SENSITIVITY 45.0             // Rotation degrees for full deflection
#define THROTTLE_POT_PIN 0               // GPIO pin for throttle potentiometer
#define POT_CALIBRATION_SAMPLES 50       // Samples per calibration step
#endif

#endif // BUILD_TX

// ============================================================
// RECEIVER (RX) CONFIGURATION
// ============================================================
#ifdef BUILD_RX

// -------- Protocol Selection --------
// Choose ONE protocol to output to flight controller:
// Options: PROTOCOL_CRSF, PROTOCOL_SBUS, PROTOCOL_PPM, PROTOCOL_IBUS, PROTOCOL_FRSKY_SPORT
#define OUTPUT_PROTOCOL PROTOCOL_CRSF
//#define OUTPUT_PROTOCOL PROTOCOL_SBUS
//#define OUTPUT_PROTOCOL PROTOCOL_PPM
//#define OUTPUT_PROTOCOL PROTOCOL_IBUS
//#define OUTPUT_PROTOCOL PROTOCOL_FRSKY_SPORT

// -------- Protocol Pin Configuration --------

// UART-based protocols (CRSF, SBUS, iBus, FrSky) use these pins:
#define UART_TX_PIN 21                 // TX pin to FC RX
#define UART_RX_PIN 20                 // RX pin from FC TX (optional)

// GPIO-based protocols (PPM) use this pin:
#define PPM_OUTPUT_PIN 5               // PPM output pin

// -------- Protocol-Specific Settings --------

// CRSF Settings
#define CRSF_BAUDRATE 420000           // Standard CRSF baud rate
#define CRSF_OUTPUT_FREQUENCY_HZ 100   // CRSF output frequency (match TX)

// SBUS Settings
#define SBUS_BAUDRATE 100000           // Standard SBUS baud rate
#define SBUS_OUTPUT_FREQUENCY_HZ 50    // SBUS is typically 50 Hz (every 20ms)
#define SBUS_INVERT_SIGNAL true        // SBUS uses inverted serial (true for most ESP32)

// PPM Settings
#define PPM_FRAME_LENGTH_US 20000      // PPM frame length (20ms = 50Hz)
#define PPM_PULSE_LENGTH_US 400        // PPM sync pulse length
#define PPM_CHANNEL_COUNT 8            // Number of PPM channels to output

// iBus Settings
#define IBUS_BAUDRATE 115200           // Standard iBus baud rate
#define IBUS_OUTPUT_FREQUENCY_HZ 50    // iBus is typically 50 Hz

// FrSky S.PORT Settings
#define FRSKY_SPORT_BAUDRATE 57600     // Standard FrSky S.PORT baud rate
#define FRSKY_SPORT_OUTPUT_FREQUENCY_HZ 100  // FrSky output frequency

// -------- LED Status Indicator --------
#define LED_PIN 2                      // Built-in LED pin (GPIO 2 for ESP32-C3)
#define LED_BLINK_RATE 500             // Blink interval in ms (blinking = no TX connection)

// -------- Debug Configuration --------
// Set to false for normal operation, true for USB Serial debugging
#define ENABLE_DEBUG_OUTPUT false      // true = USB debug messages, false = production mode

// -------- RX Communication --------
#define RX_TELEMETRY_SEND_INTERVAL_MS 100    // Telemetry send interval in ms (10 Hz)
#define RX_STATS_INTERVAL_MS 1000            // Statistics print interval in ms

#endif // BUILD_RX

// ============================================================
// PROTOCOL DEFINITIONS (Internal - Do not modify)
// ============================================================
enum OutputProtocol {
    PROTOCOL_CRSF = 0,        // CRSF (Crossfire/ELRS)
    PROTOCOL_SBUS = 1,        // SBUS (Futaba)
    PROTOCOL_PPM = 2,         // PPM (Pulse Position Modulation)
    PROTOCOL_IBUS = 3,        // iBus (FlySky)
    PROTOCOL_FRSKY_SPORT = 4  // FrSky S.PORT
};

#endif // CONFIG_H

