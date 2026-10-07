# <img src="apps/configurator/public/brand/aerialkit-rotor-a.svg" alt="" width="44"> Aerial Kit

**UAS Controller, Planner, Simulator, Firmware & Configurator** - an open-source stack for aerial robots, from planning a path in simulation to configuring the flight controller.

## Demo

### Python simulation

Autonomous flight - plan a path, then fly it:

<table>
  <tr>
    <td width="50%" align="center"><img src="docs/forest_rotor.png" alt="Forest RRT* path with the RotorPy backend" width="100%"></td>
    <td width="50%" align="center"><img src="docs/py_sim_planner.png" alt="RRT* path planned over mountain terrain" width="100%"></td>
  </tr>
  <tr>
    <td valign="top"><em>RRT* through forest terrain on the RotorPy backend.</em></td>
    <td valign="top"><em>RRT* over mountains - the planner climbs a 47 m ridge to reach the goal. Planned path dashed in orange, flown trajectory in cyan.</em></td>
  </tr>
</table>

Standalone Python sim (Matplotlib 3D, follow camera):

<table>
  <tr>
    <td width="50%" align="center"><img src="docs/teleop_follow.png" alt="Matplotlib 3D follow camera: quadrotor mid-turn with its 56 m flown path behind it" width="100%"></td>
    <td width="50%" align="center"><img src="docs/fixed_wing.png" alt="Matplotlib 3D follow camera: banked twin-wing following its flown path" width="100%"></td>
  </tr>
  <tr>
    <td valign="top"><em>Native multirotor backend - a 56 m climbing turn, teal trail showing where it has flown. Each propeller is coloured by thrust (grey idle → yellow → orange → red).</em></td>
    <td valign="top"><em><a href="examples/fixed_wing">Twin-wing example</a> on the same renderer - banked 50° through the turn, flown path behind it, motors on the same colour map.</em></td>
  </tr>
</table>

### Web configurator

[**Open the configurator**](https://rawfish69.github.io/aerial-kit/) · [Guide](docs/web-configurator.md)

Live attitude, receiver, sensors and parameter editing in the browser, over USB. Screenshots use the built-in demo board.

<table>
  <tr>
    <td width="50%"><img src="docs/configurator-classic.jpg" alt="Aerial Kit Configurator in the Classic theme: the demo board's attitude in 3D, with the About text file open" width="100%"></td>
    <td width="50%"><img src="docs/configurator-og.jpg" alt="Aerial Kit Configurator in the OG theme: the same view in 95/98 style" width="100%"></td>
  </tr>
  <tr>
    <td><em>Classic theme.</em></td>
    <td><em>OG theme. A dark Modern theme is the default.</em></td>
  </tr>
</table>

### Hovering & Landing (IMU + Barometer + GPS)

https://github.com/user-attachments/assets/44837663-b281-45db-9803-5aaa9812833d

*Autonomous hover and landing, commanded over the CRSF/ESP-NOW link. The state estimate fuses IMU
attitude, barometric altitude, and GPS position.*

### ROS navigation

<p>
  <img src="docs/sim_demo_1.png" alt="ROS2 Gazebo simulation demo" width="98%">
</p>


## What's in this repo

- **Controller** - standalone Python and ROS 2 control: PID, LQR, MPC, plus L1/TECS for fixed wing.
- **Planner** - straight, A*, RRT, RRT* and Dubins paths with shared terrain models.
- **Simulator** - Python simulation without ROS, plus ROS 2 / Gazebo backends.
- **AK Firmware** - our custom C flight controller, for Feather F405 and ESP32DEV, with a portable control core and image checks.
- **Web configurator** - live attitude, diagnostics and parameter editing over USB or a local bridge; includes a demo board.

The ROS-free control package is also on [PyPI](https://pypi.org/project/aerial-kit/): `pip install aerial-kit` or `pip install "aerial-kit[sim]"`.

## Autopilot & firmware support

| Platform | Integration | Guide |
| --- | --- | --- |
| **AK Firmware** | Custom flight-controller firmware + web configurator | [Guide](docs/flight-controller-firmware.md) |
| Betaflight | CRSF over ESP-NOW / UDP relay | [Guide](docs/HARDWARE.md) |
| PX4 | MAVLink over USB / UART / telemetry radio | [Guide](ros2_ws/src/mavlink_bridge/README.md) |
| ArduPilot | MAVLink through the same backend | [Guide](ros2_ws/src/mavlink_bridge/README.md) |
| ESP32 radio links | ESP-NOW, ELRS, LoRa and GPS | [Guide](firmware/legacy/README.md) |

## Start here

| Explore | Guide |
| --- | --- |
| Run a simulator or controller | [Software guide](docs/SOFTWARE_GUIDE.md) · [Examples](docs/EXAMPLE_USAGE.md) |
| Build AK firmware | [Firmware quick start & board status](docs/flight-controller-firmware.md) |
| Configure a board | [Web configurator quick start](docs/web-configurator.md) · [Build & deployment](apps/configurator/docs/BUILD-AND-DEPLOY.md) |
| Connect Python to firmware | [USB telemetry and attitude example](examples/firmware/) |
| Hardware designs and overlays | [Hardware workspace](hardware/) |
| Understand the stack | [Architecture, airframes, ROS 2 and roadmap](docs/STACK-GUIDE.md) |
| Connect hardware | [Hardware guide](docs/HARDWARE.md) · [Radio firmware](firmware/legacy/README.md) |
| Use the logo | [Branding & SVG assets](docs/branding.md) |

## Licence

The Python/ROS stack and existing link projects retain the root [MIT licence](LICENSE). AK flight-controller firmware and the web configurator are **GPL-3.0-or-later**, each with its own [firmware](firmware/LICENSE) and [configurator](apps/configurator/LICENSE) licence. Third-party notices stay with their components.
