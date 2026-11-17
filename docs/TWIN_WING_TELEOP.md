# Twin-wing keyboard teleop (ROS, MuJoCo)

Fly the articulated MuJoCo twin wing by keyboard through ROS, at actuator
level. The autonomous path (mission → L1/TECS → AirspeedNav) is unchanged and
still used whenever manual control is not active and fresh.

## Run

```bash
source ~/.local/share/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch sim_fast fixed_wing_mujoco_teleop.launch.py
```

Launch arguments: `viewer` (default `true`), `airframe_spec`, `start_z_m`
(default 60), `start_speed_mps` (default 15), `wind_x_mps`, `wind_y_mps`,
`manual_timeout_sec` (default 0.30). A higher start such as
`start_z_m:=150.0` gives more room to learn the handling.

Two windows open: the MuJoCo viewer (chase camera) and a small Tk window,
"Aerial Kit twin-wing teleop", which lists the keys and shows the live
command. **Keys only work while the Tk window has focus.** The viewer usually
opens second and takes focus; the teleop window asks for it back once after
3 s, but if your window manager refuses, click the teleop window.

The aircraft starts airborne at cruise (a fixed wing cannot hover or take off
vertically) and flies the plant's safe cruise circle until you arm.

## Controls

| Key | Action |
|---|---|
| Enter | arm: manual control takes over |
| Backspace | disarm: the plant's safe cruise circle takes over |
| W / S | nose down / nose up (both elevons together) |
| A / D | bank left / right (differential elevon) |
| Space / Shift | throttle up / down, both motors, while held |
| Q / E | differential-thrust yaw left / right, elevons unchanged |
| X | re-centre everything to trim |
| P | pause: hold trim (not zero throttle) while paused |
| C | toggle the MuJoCo viewer: chase view / on-board nose camera |
| Esc, or close the window | disarm and exit; the whole launch shuts down |

Held keys are latched from press to release. X11 auto-repeat is filtered
(release/press pairs inside 40 ms are ignored), so nothing depends on repeat
events, and P/Enter do not toggle repeatedly.

Centre stick is the plant's own level trim from the shared airframe spec and
aero model: 0.88 N per motor and +1.35° elevon at 15 m/s. That is *not* the
3 N default of the older Matplotlib teleop. Full stick is ±10° elevon
(rate-limited to 8 rad/s by the MuJoCo servos), trim +3 N per motor, and
±0.6 N differential.

This is open-loop actuator control, like flying an RC wing without
stabilisation: a bank does not level itself when you release the key. Counter
with the opposite key, or press X and then Backspace to let the plant level
the wings in its cruise circle.

## Path

```
Tk keys → KeyboardState / fixedwing_command_from_keys   (sim_py.teleop, shared)
       → fixed_wing_teleop_node → /uav/backend/cmd_actuators (uav_msgs/ActuatorCommand, 50 Hz)
                                 → /uav/backend/enable      (std_msgs/Bool)
       → fixedwing_backend_adapter_node: precedence, timeout, finite check, clamp
       → MujocoFixedWingBackend: servo slew → MuJoCo elevon hinges, prop motors,
         thrust lag; aero model evaluated on the measured hinge angles
```

Nothing sets pose, `qpos`, `qvel` or `xfrc_applied` directly; manual commands
take the same actuator path as the autopilot's.

`ActuatorCommand` is in physical order: `thrust_left_n`, `thrust_right_n`,
`elevon_left_rad`, `elevon_right_rad`, trailing-edge-up positive. Equal
positive elevons pitch up; `elevon_right − elevon_left > 0` rolls right.

## Safety behaviour

The adapter chooses a source every 10 ms tick:

1. **Manual**, if the plant is enabled, the last `ActuatorCommand` is
   `active`, and it arrived within `manual_timeout_sec` (0.30 s, by the
   adapter's own receive clock).
2. **AirspeedNav**, if enabled and a valid command arrived within
   `nav_timeout_sec`.
3. **Safe fallback**: cruise airspeed, 25° bank circle, zero climb rate.

- A command containing NaN or Inf is refused and also cancels the previous
  manual command, so the plant falls back at once instead of after the
  timeout. Out-of-range values are clamped to the airframe spec (thrust 0 to
  10 N, elevons ±25°), with a throttled warning.
- If the teleop node dies or stalls, commands go stale and the plant falls
  back within 0.3 s.
- Losing window focus clears held keys and sends trim at once. After 3 s
  unfocused the teleop disarms, so the plant's closed-loop circle takes over.
- Pause sends trim, not idle: a wing at zero thrust would stall.
- If Tk cannot open a window (no display), the node prints why, publishes
  nothing, and exits; the launch then shuts down. Nothing is ever armed.
- Esc, closing the window, or SIGINT/SIGTERM disarms (sending three inactive
  commands and `enable=false`) before exit; the launch then stops the plant.
  The adapter waits for the MuJoCo viewer's render thread before exiting.
- Telemetry `status_text` reports the active source (`manual`,
  `airspeed_nav`, `fallback`). `manual_override_active` is set, and
  `current_mode` is `MODE_MANUAL_OVERRIDE`, while manual.

## Limitations

- Open-loop actuator control only: no rate or attitude stabilisation, no
  angle limits. Easy to over-bank; a full-deflection hold rolls at ~1 rad/s.
- The aero model is unfitted (research defaults); handling is not that of
  any real aircraft.
- Keyboard input is digital (full deflection or none). There is no joystick
  or RC input.
- Ground contact is coarse collision primitives; there is no landing gear,
  takeoff or landing model. A crash leaves the airframe resting on the ground.
- Teleop runs the plant in `estimator_mode` truth; EKF is not wired into
  manual mode (it only affects what guidance sees).
- Gazebo is not part of the teleop launch.
- Nothing here establishes hardware or RC-link readiness.
