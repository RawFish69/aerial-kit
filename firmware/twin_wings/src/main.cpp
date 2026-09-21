#include <Arduino.h>
#include "config.h"
#include "pwm_output.h"
#include "control_input.h"
#include "sensors.h"
#include "controller.h"
#include "guidance.h"
#include "navigation.h"
#include "rc_input.h"

// Twin Motor Flying Wing bench demonstration.
//
// Mixing model:
//   throttle_cmd      -> common thrust on both motors
//   yaw_cmd           -> differential thrust (motorL - motorR)
//   pitch_cmd         -> symmetric elevon deflection
//   roll_cmd          -> asymmetric elevon deflection
//
// NOT A FLIGHT CONTROLLER. This is a bench sketch for checking wiring, PWM
// rates, servo directions and mixer signs. An earlier revision of this comment
// said the controller and sensor drivers were "intentionally left as TODOs";
// they were since written, and were unvalidated when they were, so that
// sentence was removed rather than left to age. What is true now: the loop
// period is `delay(20)` plus execution time while the controller is handed a
// hardcoded 0.02f, a failed IMU silently passes stick demands straight to the
// mixer, there is no arming state, and the CRSF length/CRC span is wrong. Do
// not flash this to an aircraft. See ../../README.md.

static void applyTwinWingMix(const ControlInput& in) {
  float motorL = in.throttle + in.yaw * YAW_DIFFERENTIAL_GAIN;
  float motorR = in.throttle - in.yaw * YAW_DIFFERENTIAL_GAIN;
  motorL = constrain(motorL, MOTOR_MIN, MOTOR_MAX);
  motorR = constrain(motorR, MOTOR_MIN, MOTOR_MAX);

  float elevonL = SERVO_NEUTRAL + (in.pitch * ELEVON_PITCH_GAIN) - (in.roll * ELEVON_ROLL_GAIN);
  float elevonR = SERVO_NEUTRAL + (in.pitch * ELEVON_PITCH_GAIN) + (in.roll * ELEVON_ROLL_GAIN);
  elevonL = constrain(elevonL, SERVO_MIN, SERVO_MAX);
  elevonR = constrain(elevonR, SERVO_MIN, SERVO_MAX);

  pwmWriteMotor(MOTOR_L_PIN, motorL);
  pwmWriteMotor(MOTOR_R_PIN, motorR);
  pwmWriteServo(ELEVON_L_PIN, elevonL);
  pwmWriteServo(ELEVON_R_PIN, elevonR);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("twin_wings scaffold");

  pwmBeginMotor(MOTOR_L_PIN);
  pwmBeginMotor(MOTOR_R_PIN);
  pwmBeginServo(ELEVON_L_PIN);
  pwmBeginServo(ELEVON_R_PIN);
  sensorsInit();
  wingControllerReset();
  navigationReset();
  #if RC_INPUT_PROTOCOL != 0
  rcInputInit(RC_SERIAL, RC_UART_BAUD, RC_UART_INVERT);
  #endif

  // Initialize all actuators to safe neutral.
  ControlInput neutral;
  applyTwinWingMix(neutral);
}

void loop() {
  ControlInput cmd;
  readControlInput(cmd);
  #if RC_INPUT_PROTOCOL == 1
  rcInputReadCrsf(RC_SERIAL, cmd);
  #elif RC_INPUT_PROTOCOL == 2
  rcInputReadSbus(RC_SERIAL, cmd);
  #endif
  ImuData imu;
  GpsData gps;
  bool haveImu = imuRead(imu);
  static NavigationState nav;
  if (gpsRead(gps)) {
    navigationUpdate(gps, nav);
    #if ENABLE_GPS_GUIDANCE
    if (gps.fix) {
      GuidanceTarget target{WAYPOINT_LAT, WAYPOINT_LON, WAYPOINT_ALT_M};
      guidanceToTarget(gps, nav, target, cmd);
    }
    #endif
  }

  WingActuatorCommand act;
  if (haveImu) {
    wingControllerUpdate(cmd, imu, 0.02f, act);
  } else {
    act.roll = cmd.roll;
    act.pitch = cmd.pitch;
    act.yaw = cmd.yaw;
    act.throttle = cmd.throttle;
  }
  applyTwinWingMix(ControlInput{act.roll, act.pitch, act.yaw, act.throttle});
  delay(20);
}
