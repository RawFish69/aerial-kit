#include "pwm_output.h"

#if defined(ARDUINO_ARCH_ESP32)

struct PwmEntry {
  int pin;
  uint8_t channel;
};

static PwmEntry s_servoEntries[4];
static PwmEntry s_motorEntries[4];
static int s_servoCount = 0;
static int s_motorCount = 0;

static uint8_t ensureChannel(PwmEntry* entries, int* count, int pin, float frequencyHz) {
  for (int i = 0; i < *count; ++i) {
    if (entries[i].pin == pin) {
      return entries[i].channel;
    }
  }

  uint8_t channel = static_cast<uint8_t>(*count);
  ledcSetup(channel, frequencyHz, 12);
  ledcAttachPin(pin, channel);
  entries[*count].pin = pin;
  entries[*count].channel = channel;
  (*count)++;
  return channel;
}

static void writeEsp32(PwmEntry* entries, int* count, int pin, float frequencyHz,
                       float normalized) {
  uint8_t channel = ensureChannel(entries, count, pin, frequencyHz);
  float clamped = constrain(normalized, 0.0f, 1.0f);
  uint32_t duty = static_cast<uint32_t>(clamped * 4095.0f);
  ledcWrite(channel, duty);
}

void pwmBeginServo(int pin) {
  ensureChannel(s_servoEntries, &s_servoCount, pin, 50.0f);
}

void pwmBeginMotor(int pin) {
  ensureChannel(s_motorEntries, &s_motorCount, pin, 400.0f);
}

void pwmWriteServo(int pin, float normalized) {
  writeEsp32(s_servoEntries, &s_servoCount, pin, 50.0f, normalized);
}

void pwmWriteMotor(int pin, float normalized) {
  writeEsp32(s_motorEntries, &s_motorCount, pin, 400.0f, normalized);
}

#elif defined(ARDUINO_ARCH_STM32)

void pwmBeginServo(int pin) {
  analogWriteFrequency(50);
  analogWriteResolution(12);
  pinMode(pin, OUTPUT);
}

void pwmBeginMotor(int pin) {
  analogWriteFrequency(400);
  analogWriteResolution(12);
  pinMode(pin, OUTPUT);
}

void pwmWriteServo(int pin, float normalized) {
  float clamped = constrain(normalized, 0.0f, 1.0f);
  analogWrite(pin, static_cast<int>(clamped * 4095.0f));
}

void pwmWriteMotor(int pin, float normalized) {
  float clamped = constrain(normalized, 0.0f, 1.0f);
  analogWrite(pin, static_cast<int>(clamped * 4095.0f));
}

#else

void pwmBeginServo(int pin) {
  pinMode(pin, OUTPUT);
}

void pwmBeginMotor(int pin) {
  pinMode(pin, OUTPUT);
}

void pwmWriteServo(int pin, float normalized) {
  float clamped = constrain(normalized, 0.0f, 1.0f);
  analogWrite(pin, static_cast<int>(clamped * 255.0f));
}

void pwmWriteMotor(int pin, float normalized) {
  float clamped = constrain(normalized, 0.0f, 1.0f);
  analogWrite(pin, static_cast<int>(clamped * 255.0f));
}

#endif
