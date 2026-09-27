#pragma once

#include <Arduino.h>

// The vehicle profile supplies the airframe pins and any profile-only gains.
// It is selected by the build; see src/profiles/airframe.h.
#include "profiles/airframe.h"

// Optional IMU I2C pins. Adjust for your board/wiring.
#if defined(ARDUINO_ARCH_ESP32)
  #define IMU_SDA_PIN 4
  #define IMU_SCL_PIN 5
#elif defined(ARDUINO_ARCH_STM32)
  #define IMU_SDA_PIN PB7
  #define IMU_SCL_PIN PB6
#else
  #define IMU_SDA_PIN SDA
  #define IMU_SCL_PIN SCL
#endif

// Optional GPS UART pins and baud. Adjust for your board/module.
#if defined(ARDUINO_ARCH_ESP32)
  #define GPS_RX_PIN 6
  #define GPS_TX_PIN 7
#elif defined(ARDUINO_ARCH_STM32)
  #define GPS_RX_PIN PA10
  #define GPS_TX_PIN PA9
#else
  #define GPS_RX_PIN 1
  #define GPS_TX_PIN 0
#endif
#define GPS_BAUD 9600

// Set to 1 once a waypoint source is defined. When enabled, GPS guidance
// overrides the bench serial command input.
#define ENABLE_GPS_GUIDANCE 0
#define WAYPOINT_LAT 0.0
#define WAYPOINT_LON 0.0
#define WAYPOINT_ALT_M 100.0

// RC input protocol: 0 = bench serial, 1 = CRSF, 2 = SBUS.
#define RC_INPUT_PROTOCOL 0
#if RC_INPUT_PROTOCOL != 0
  #if defined(ARDUINO_ARCH_ESP32)
    #if defined(CONFIG_IDF_TARGET_ESP32)
      #define RC_SERIAL Serial2
      #define RC_RX_PIN 16
      #define RC_TX_PIN 17
    #else
      #define RC_SERIAL Serial1
      #define RC_RX_PIN 0
      #define RC_TX_PIN 1
    #endif
  #elif defined(ARDUINO_ARCH_STM32)
    #define RC_SERIAL Serial1
    #define RC_RX_PIN PA10
    #define RC_TX_PIN PA9
  #else
    #define RC_SERIAL Serial
    #define RC_RX_PIN 1
    #define RC_TX_PIN 0
  #endif
  #if RC_INPUT_PROTOCOL == 1
    #define RC_UART_BAUD 420000
    #define RC_UART_INVERT false
  #elif RC_INPUT_PROTOCOL == 2
    #define RC_UART_BAUD 100000
    #define RC_UART_INVERT true
  #endif
#endif

// Normalized actuator ranges.
#define SERVO_NEUTRAL 0.5f
#define SERVO_MIN 0.0f
#define SERVO_MAX 1.0f
#define MOTOR_MIN 0.0f
#define MOTOR_MAX 1.0f

// Mixing gains shared by every profile. Tune after the real control loop and
// sensor drivers exist.
#define ELEVON_ROLL_GAIN 0.5f
#define ELEVON_PITCH_GAIN 0.5f
