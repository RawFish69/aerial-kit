#include "mixer.h"

#include <Arduino.h>

#include "config.h"
#include "mix_math.h"
#include "pwm_output.h"

// Mixing model, shared by both wings:
//   pitch_cmd    -> symmetric elevon deflection
//   roll_cmd     -> asymmetric elevon deflection
//   throttle_cmd -> thrust
//   yaw_cmd      -> differential thrust (twin only; no yaw authority otherwise)
//
// The arithmetic itself lives in mix_math.h, which has no Arduino dependency, so
// the host test in ../host_test/ compiles the same functions this file does.

// Motor first, then servos: that is the order both original projects used, and
// on STM32 the PWM frequency is set per timer, so the order is not cosmetic if a
// motor and a servo ever share one.
void akActuatorBegin() {
#if AK_MOTOR_COUNT == 2
  pwmBeginMotor(AK_MOTOR_L_PIN);
  pwmBeginMotor(AK_MOTOR_R_PIN);
#else
  pwmBeginMotor(AK_MOTOR_PIN);
#endif
  pwmBeginServo(AK_ELEVON_L_PIN);
  pwmBeginServo(AK_ELEVON_R_PIN);
}

void akMix(const ControlInput& in) {
  float elevonL, elevonR;
  akElevonMix(in.pitch, in.roll, ELEVON_PITCH_GAIN, ELEVON_ROLL_GAIN,
              SERVO_NEUTRAL, SERVO_MIN, SERVO_MAX, elevonL, elevonR);

#if AK_MOTOR_COUNT == 2
  float motorL, motorR;
  akDifferentialThrust(in.throttle, in.yaw, YAW_DIFFERENTIAL_GAIN,
                       MOTOR_MIN, MOTOR_MAX, motorL, motorR);
  pwmWriteMotor(AK_MOTOR_L_PIN, motorL);
  pwmWriteMotor(AK_MOTOR_R_PIN, motorR);
#else
  // in.yaw is deliberately unused: this profile has no yaw actuator, so the
  // differential-thrust call is not compiled rather than called with a zero.
  pwmWriteMotor(AK_MOTOR_PIN, akCommonThrottle(in.throttle, MOTOR_MIN, MOTOR_MAX));
#endif
  pwmWriteServo(AK_ELEVON_L_PIN, elevonL);
  pwmWriteServo(AK_ELEVON_R_PIN, elevonR);
}
