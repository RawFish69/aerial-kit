<picture>
  <source media="(prefers-color-scheme: dark)" srcset="apps/configurator/public/brand/aerialkit-rotor-a-logo-dark.svg">
  <img src="apps/configurator/public/brand/aerialkit-rotor-a-logo-light.svg" alt="AerialKit — Rotor A" width="320">
</picture>

# Aerial Kit

**UAS Controller, Planner, Simulator, Firmware & Configurator** — an open-source stack for aerial robots, from planning a path in simulation to configuring the flight controller.

## Demo

### Python Sim & ROS2 Gazebo Sim

Autonomous flight — plan a path, then fly it:

<table>
  <tr>
    <td width="50%" align="center"><img src="docs/forest_rotor.png" alt="Forest RRT* path with the RotorPy backend" width="100%"></td>
    <td width="50%" align="center"><img src="docs/py_sim_planner.png" alt="RRT* path planned over mountain terrain" width="100%"></td>
  </tr>
  <tr>
    <td valign="top"><em>RRT* through forest terrain on the RotorPy backend.</em></td>
    <td valign="top"><em>RRT* over mountains — the planner climbs a 47 m ridge to reach the goal. Planned path dashed in orange, flown trajectory in cyan.</em></td>
  </tr>
</table>

Standalone Python sim (Matplotlib 3D, follow camera):

<table>
  <tr>
    <td width="50%" align="center"><img src="docs/teleop_follow.png" alt="Matplotlib 3D follow camera: quadrotor mid-turn with its 56 m flown path behind it" width="100%"></td>
    <td width="50%" align="center"><img src="docs/fixed_wing.png" alt="Matplotlib 3D follow camera: banked twin-wing following its flown path" width="100%"></td>
  </tr>
  <tr>
    <td valign="top"><em>Native multirotor backend — a 56 m climbing turn, teal trail showing where it has flown. Each propeller is coloured by thrust (grey idle → yellow → orange → red).</em></td>
    <td valign="top"><em><a href="examples/fixed_wing">Twin-wing example</a> on the same renderer — banked 50° through the turn, flown path behind it, motors on the same colour map.</em></td>
  </tr>
</table>

<p>
  <img src="docs/sim_demo_1.png" alt="ROS2 Gazebo simulation demo" width="98%">
</p>

### Hovering & Landing (IMU + Barometer + GPS)

https://github.com/user-attachments/assets/44837663-b281-45db-9803-5aaa9812833d

*Autonomous hover and landing, commanded over the CRSF/ESP-NOW link. The state estimate fuses IMU
attitude, barometric altitude, and GPS position.*

## What's in this repo

- **Controller** — standalone Python and ROS 2 control: PID, LQR, MPC, plus L1/TECS for fixed wing.
- **Planner** — straight, A*, RRT, RRT* and Dubins paths with shared terrain models.
- **Simulator** — Python simulation without ROS, plus ROS 2 / Gazebo backends.
- **AerialKit Firmware** — our custom C flight controller, with STM32F405, ESP32 and AT32F435 ports, host simulation and image checks.
- **Web configurator** — live attitude, diagnostics and parameter editing over USB or a local bridge; includes a demo board.

The ROS-free control package is also on [PyPI](https://pypi.org/project/aerial-kit/): `pip install aerial-kit` or `pip install "aerial-kit[sim]"`.

## Autopilot & firmware support

| Platform | Integration | Status / guide |
| --- | --- | --- |
| **AerialKit** | Custom flight-controller firmware + web configurator | [STM32F405 USB/configuration bench verified; not flown](docs/flight-controller-firmware.md) |
| Betaflight | CRSF over ESP-NOW / UDP relay | [Hardware bridge](docs/HARDWARE.md) |
| PX4 | MAVLink over USB / UART / telemetry radio | [MAVLink bridge](ros2_ws/src/mavlink_bridge/README.md) |
| ArduPilot | MAVLink through the same backend | [Supported, in testing](ros2_ws/src/mavlink_bridge/README.md) |
| ESP32 radio links | ESP-NOW, ELRS, LoRa and GPS | [Firmware index](firmware/README.md) |

## Start here

| Explore | Guide |
| --- | --- |
| Run a simulator or controller | [Software guide](docs/SOFTWARE_GUIDE.md) · [Examples](docs/EXAMPLE_USAGE.md) |
| Build AerialKit firmware | [Firmware quick start & board status](docs/flight-controller-firmware.md) |
| Configure a board | [Web configurator quick start](docs/web-configurator.md) · [Build & deployment](apps/configurator/docs/BUILD-AND-DEPLOY.md) |
| Understand the stack | [Architecture, airframes, ROS 2 and roadmap](docs/STACK-GUIDE.md) |
| Connect hardware | [Hardware guide](docs/HARDWARE.md) · [Radio firmware](firmware/README.md) |
| Use the logo | [Branding & SVG assets](docs/branding.md) |

## Licence

The Python/ROS stack and existing link projects retain the root [MIT licence](LICENSE). AerialKit flight-controller firmware and the web configurator are **GPL-3.0-or-later**, each with its own [firmware](aerialkit/LICENSE) and [configurator](apps/configurator/LICENSE) licence. Third-party notices stay with their components.
