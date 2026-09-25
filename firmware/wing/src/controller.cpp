#include "controller.h"

#include <Arduino.h>

namespace {

constexpr float kMaxBankDeg = 45.0f;
constexpr float kMaxPitchDeg = 30.0f;
constexpr float kMaxYawRateDegPerSec = 60.0f;

constexpr float kpRoll = 1.0f;
constexpr float kpPitch = 1.0f;
constexpr float kpYaw = 1.0f;
constexpr float kdRoll = 0.02f;
constexpr float kdPitch = 0.02f;
constexpr float kdYaw = 0.02f;
constexpr float kiRoll = 0.05f;
constexpr float kiPitch = 0.05f;
constexpr float kiYaw = 0.05f;
constexpr float kIntegralLimit = 0.5f;

float rollIntegral = 0.0f;
float pitchIntegral = 0.0f;
float yawIntegral = 0.0f;

float clampNorm(float value) {
  return constrain(value, -1.0f, 1.0f);
}

}  // namespace

void wingControllerReset() {
  rollIntegral = 0.0f;
  pitchIntegral = 0.0f;
  yawIntegral = 0.0f;
}

void wingControllerUpdate(const ControlInput& cmd, const ImuData& imu,
                          float dtSec, WingActuatorCommand& out) {
  // Proportional attitude/rate skeleton. Tune against PX4/ArduPilot/Betaflight
  // fixed-wing defaults before flight, then add rate damping and integral terms.
  float rollError = cmd.roll - clampNorm(imu.rollDeg / kMaxBankDeg);
  float pitchError = cmd.pitch - clampNorm(imu.pitchDeg / kMaxPitchDeg);
  float yawError = cmd.yaw - clampNorm(imu.gz / kMaxYawRateDegPerSec);

  rollIntegral = constrain(rollIntegral + rollError * dtSec, -kIntegralLimit, kIntegralLimit);
  pitchIntegral = constrain(pitchIntegral + pitchError * dtSec, -kIntegralLimit, kIntegralLimit);
  yawIntegral = constrain(yawIntegral + yawError * dtSec, -kIntegralLimit, kIntegralLimit);

  // Rate damping from gyro readings keeps the proportional attitude loop from
  // oscillating. Tune kd against PX4/ArduPilot/Betaflight fixed-wing defaults.
  out.roll = clampNorm(kpRoll * rollError - kdRoll * imu.gx + kiRoll * rollIntegral);
  out.pitch = clampNorm(kpPitch * pitchError - kdPitch * imu.gy + kiPitch * pitchIntegral);
  out.yaw = clampNorm(kpYaw * yawError - kdYaw * imu.gz + kiYaw * yawIntegral);
  out.throttle = constrain(cmd.throttle, 0.0f, 1.0f);
}
