#pragma once

#include <Arduino.h>

// L1 lateral guidance: bank angle (radians) to follow a straight path toward the
// next waypoint, given cross-track error and groundspeed.
float l1BankCommandRad(float crossTrackErrorM, float groundspeedMps, float l1DistanceM);

// TECS-lite longitudinal guidance: split altitude/airspeed error into pitch and
// throttle commands. Both outputs are normalized (-1..1 pitch, 0..1 throttle).
void tecsEnergyCommand(float altitudeErrorM, float airspeedErrorMps,
                       float& pitchCmd, float& throttleCmd);
