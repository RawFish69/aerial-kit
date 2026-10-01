#pragma once

// Vehicle profile selection.
//
// Exactly one profile is chosen at build time; the PlatformIO environments in
// platformio.ini each set one flag. A build that sets neither, or both, stops
// here rather than silently flying a different wing than the one asked for.

#if defined(AK_AIRFRAME_SINGLE) && defined(AK_AIRFRAME_TWIN)
  #error "define only one of AK_AIRFRAME_SINGLE / AK_AIRFRAME_TWIN"
#elif defined(AK_AIRFRAME_SINGLE)
  #include "single_wing.h"
#elif defined(AK_AIRFRAME_TWIN)
  #include "twin_wings.h"
#else
  #error "no vehicle profile: define AK_AIRFRAME_SINGLE or AK_AIRFRAME_TWIN"
#endif

// The profile's own declarations have to agree with each other. Yaw authority in
// this runtime comes from differential thrust, which exists exactly when the
// profile has two motors, and the bench serial line carries a yaw field exactly
// when there is a yaw axis. A profile that says otherwise is a mistake, and this
// is the line that says so rather than the mixer quietly ignoring the argument.
#ifndef AK_HAS_YAW
  #error "profile does not define AK_HAS_YAW"
#endif
#if AK_HAS_YAW != (AK_MOTOR_COUNT == 2)
  #error "profile is inconsistent: AK_HAS_YAW must be 1 exactly when AK_MOTOR_COUNT is 2"
#endif
#if AK_CONTROL_VALUES != (3 + AK_HAS_YAW)
  #error "profile is inconsistent: AK_CONTROL_VALUES must be 3 without a yaw axis, 4 with one"
#endif
