#pragma once

#include "control_input.h"

// The one mixer. Which actuators exist is a property of the vehicle profile
// (AK_MOTOR_COUNT), not of a second copy of this file.
void akActuatorBegin();
void akMix(const ControlInput& in);
