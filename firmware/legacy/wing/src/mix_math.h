#pragma once

// The mix math, with no Arduino and no config header in it. Gains, neutral and
// limits are arguments, so the code the firmware flashes is the code the host
// test compiles — see ../host_test/. Factor it back into mixer.cpp and the test
// would be checking a copy of the arithmetic rather than the arithmetic.
//
// Conventions, unchanged from the two projects this replaces:
//   pitch positive = nose up      -> both elevons the same way
//   roll  positive = right roll   -> elevons opposite ways
//   yaw   positive = nose right   -> left motor faster than right
//   throttle 0..1, servos 0..1 with 0.5 neutral

inline float akClamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Common throttle, no yaw axis. A single-motor profile calls this and never sees
// a yaw term at all.
inline float akCommonThrottle(float throttle, float lo, float hi) {
  return akClamp(throttle, lo, hi);
}

// Differential thrust. Two motors, so yaw is available.
inline void akDifferentialThrust(float throttle, float yaw, float yawGain,
                                 float lo, float hi,
                                 float& left, float& right) {
  left = akClamp(throttle + yaw * yawGain, lo, hi);
  right = akClamp(throttle - yaw * yawGain, lo, hi);
}

// Symmetric pitch, asymmetric roll, about a neutral pulse.
inline void akElevonMix(float pitch, float roll,
                        float pitchGain, float rollGain,
                        float neutral, float lo, float hi,
                        float& left, float& right) {
  left = akClamp(neutral + pitch * pitchGain - roll * rollGain, lo, hi);
  right = akClamp(neutral + pitch * pitchGain + roll * rollGain, lo, hi);
}
