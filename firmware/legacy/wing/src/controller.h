#pragma once

#include "control_input.h"
#include "sensors.h"

struct WingActuatorCommand {
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float throttle = 0.0f;
};

void wingControllerReset();
void wingControllerUpdate(const ControlInput& cmd, const ImuData& imu,
                          float dtSec, WingActuatorCommand& out);
