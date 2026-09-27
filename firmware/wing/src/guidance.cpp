#include "guidance.h"

#include <math.h>

float l1BankCommandRad(float crossTrackErrorM, float groundspeedMps,
                       float l1DistanceM) {
  if (groundspeedMps < 1e-3f || l1DistanceM < 1e-3f) {
    return 0.0f;
  }
  float term = 2.0f * crossTrackErrorM * groundspeedMps /
               (l1DistanceM * l1DistanceM);
  return atanf(term);
}

void tecsEnergyCommand(float altitudeErrorM, float airspeedErrorMps,
                       float& pitchCmd, float& throttleCmd) {
  // Energy error drives throttle, balance error drives pitch. Tune these gains
  // against ArduPlane TECS / PX4 fw_pos_control_l1 before flight.
  constexpr float kAltitudeGain = 0.1f;
  constexpr float kSpeedGain = 0.5f;
  constexpr float kPitchGain = 0.2f;
  constexpr float kThrottleGain = 0.2f;

  float energyError = kAltitudeGain * altitudeErrorM + kSpeedGain * airspeedErrorMps;
  float balanceError = kAltitudeGain * altitudeErrorM - kSpeedGain * airspeedErrorMps;

  pitchCmd = constrain(balanceError * kPitchGain, -1.0f, 1.0f);
  throttleCmd = constrain(energyError * kThrottleGain, 0.0f, 1.0f);
}
