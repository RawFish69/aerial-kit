#include "mixer.h"

#include <Arduino.h>

#include "config.h"
#include "pwm_output.h"

// Mixing model, shared by both wings:
//   pitch_cmd    -> symmetric elevon deflection
//   roll_cmd     -> asymmetric elevon deflection
//   throttle_cmd -> thrust
//   yaw_cmd      -> differential thrust (twin only; no yaw authority otherwise)

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
#if AK_MOTOR_COUNT == 2
  float motorL = constrain(in.throttle + in.yaw * YAW_DIFFERENTIAL_GAIN,
                           MOTOR_MIN, MOTOR_MAX);
  float motorR = constrain(in.throttle - in.yaw * YAW_DIFFERENTIAL_GAIN,
                           MOTOR_MIN, MOTOR_MAX);
#else
  // in.yaw is deliberately unused: this profile has no yaw actuator.
  float motor = constrain(in.throttle, MOTOR_MIN, MOTOR_MAX);
#endif

  float elevonL = SERVO_NEUTRAL + (in.pitch * ELEVON_PITCH_GAIN) - (in.roll * ELEVON_ROLL_GAIN);
  float elevonR = SERVO_NEUTRAL + (in.pitch * ELEVON_PITCH_GAIN) + (in.roll * ELEVON_ROLL_GAIN);
  elevonL = constrain(elevonL, SERVO_MIN, SERVO_MAX);
  elevonR = constrain(elevonR, SERVO_MIN, SERVO_MAX);

#if AK_MOTOR_COUNT == 2
  pwmWriteMotor(AK_MOTOR_L_PIN, motorL);
  pwmWriteMotor(AK_MOTOR_R_PIN, motorR);
#else
  pwmWriteMotor(AK_MOTOR_PIN, motor);
#endif
  pwmWriteServo(AK_ELEVON_L_PIN, elevonL);
  pwmWriteServo(AK_ELEVON_R_PIN, elevonR);
}
