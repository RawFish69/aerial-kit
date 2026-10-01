#pragma once

// Vehicle profile: single-motor flying wing.
//
// One motor on the throttle, two elevons for roll and pitch. A single-motor
// wing has no yaw authority, so the controller's yaw output is computed and
// then discarded by this profile's mixer (AK_MOTOR_COUNT == 1).

#include <Arduino.h>

#define AK_AIRFRAME_NAME "single_wing"
#define AK_MOTOR_COUNT 1
#define AK_HAS_YAW 0
// Bench serial fields: "roll,pitch,throttle".
#define AK_CONTROL_VALUES 3

// Board-specific default pins. Adjust for your actual wiring. These were
// MOTOR_PIN / SERVO_L_PIN / SERVO_R_PIN before the runtime was consolidated;
// the names now match the twin-motor profile, one spelling for both wings.
#if defined(ARDUINO_ARCH_ESP32)
  #define AK_MOTOR_PIN 12
  #define AK_ELEVON_L_PIN 14
  #define AK_ELEVON_R_PIN 15
#elif defined(ARDUINO_ARCH_STM32)
  #define AK_MOTOR_PIN PA0
  #define AK_ELEVON_L_PIN PA1
  #define AK_ELEVON_R_PIN PA2
#else
  #define AK_MOTOR_PIN 3
  #define AK_ELEVON_L_PIN 5
  #define AK_ELEVON_R_PIN 6
#endif
