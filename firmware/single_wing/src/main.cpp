#include <Arduino.h>
#include "config.h"
#include "pwm_output.h"
#include "control_input.h"
#include "sensors.h"
#include "controller.h"
#include "guidance.h"
#include "navigation.h"
#include "rc_input.h"

// Single Motor Flying Wing bench demonstration.
//
// Mixing model:
//   throttle_cmd -> single motor
//   pitch_cmd    -> symmetric elevon deflection
//   roll_cmd     -> asymmetric elevon deflection
//
// This is a control-surface placeholder, not a flight-ready implementation, and
// the same warnings as twin_wings/src/main.cpp apply: an unbounded loop period
// against a hardcoded 0.02f dt, stick demands passed straight to the mixer when
// the IMU read fails, no arming state, and a wrong CRSF length/CRC span. Do not
// flash this to an aircraft. See ../../README.md.

static void applySingleWingMix(const ControlInput& in) {
  float motor = constrain(in.throttle, MOTOR_MIN, MOTOR_MAX);
  float servoL = SERVO_NEUTRAL + (in.pitch * ELEVON_PITCH_GAIN) - (in.roll * ELEVON_ROLL_GAIN);
  float servoR = SERVO_NEUTRAL + (in.pitch * ELEVON_PITCH_GAIN) + (in.roll * ELEVON_ROLL_GAIN);
  servoL = constrain(servoL, SERVO_MIN, SERVO_MAX);
  servoR = constrain(servoR, SERVO_MIN, SERVO_MAX);

  pwmWriteMotor(MOTOR_PIN, motor);
  pwmWriteServo(SERVO_L_PIN, servoL);
  pwmWriteServo(SERVO_R_PIN, servoR);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("single_wing scaffold");

  pwmBeginMotor(MOTOR_PIN);
  pwmBeginServo(SERVO_L_PIN);
  pwmBeginServo(SERVO_R_PIN);
  sensorsInit();
  wingControllerReset();
  navigationReset();
  #if RC_INPUT_PROTOCOL != 0
  rcInputInit(RC_SERIAL, RC_UART_BAUD, RC_UART_INVERT);
  #endif

  ControlInput neutral;
  applySingleWingMix(neutral);
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
    act.throttle = cmd.throttle;
  }
  applySingleWingMix(ControlInput{act.roll, act.pitch, act.throttle});
  delay(20);
}
