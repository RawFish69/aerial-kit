#pragma once

#include <Arduino.h>

void pwmBeginServo(int pin);
void pwmBeginMotor(int pin);
void pwmWriteServo(int pin, float normalized);
void pwmWriteMotor(int pin, float normalized);
