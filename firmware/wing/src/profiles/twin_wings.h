#pragma once

// Vehicle profile: twin-motor flying wing.
//
// Two motors on the throttle, yawing by differential thrust, two elevons for
// roll and pitch. The differential-thrust gain lives here because it has no
// meaning for a single-motor wing.

#include <Arduino.h>

#define AK_AIRFRAME_NAME "twin_wings"
#define AK_MOTOR_COUNT 2
#define AK_HAS_YAW 1
// Bench serial fields: "roll,pitch,yaw,throttle".
#define AK_CONTROL_VALUES 4

// Board-specific default pins. Adjust for your actual wiring.
#if defined(ARDUINO_ARCH_ESP32)
  #define AK_MOTOR_L_PIN 12
  #define AK_MOTOR_R_PIN 13
  #define AK_ELEVON_L_PIN 14
  #define AK_ELEVON_R_PIN 15
#elif defined(ARDUINO_ARCH_STM32)
  #define AK_MOTOR_L_PIN PA0
  #define AK_MOTOR_R_PIN PA1
  #define AK_ELEVON_L_PIN PA2
  #define AK_ELEVON_R_PIN PA3
#else
  #define AK_MOTOR_L_PIN 3
  #define AK_MOTOR_R_PIN 5
  #define AK_ELEVON_L_PIN 6
  #define AK_ELEVON_R_PIN 9
#endif

// Differential thrust per unit of yaw demand.
#define YAW_DIFFERENTIAL_GAIN 0.5f
