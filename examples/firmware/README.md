# Python and AK firmware

Two small examples connect the public Python package to the firmware's existing config protocol.

From a clone of this repository:

```sh
python -m pip install -e . pyserial
python examples/firmware/read_board.py --port /dev/ttyACM0 --samples 20
```

On Windows, use a port such as `--port COM5`. Close the web configurator's connection first: one program owns the serial port at a time. The script reads board identity, then prints one JSON status sample per line. Identity goes to stderr so stdout can be piped or saved.

For an ESP32 running a TCP config server, replace the serial option with `--host BOARD_ADDRESS:CONFIG_PORT`. Use the address and config port from that firmware build. This is the firmware endpoint, not the configurator's WebSocket bridge.

Convert board attitude to the Python package's quaternion representation:

```sh
python examples/firmware/read_board.py --port /dev/ttyACM0 | python examples/firmware/attitude_to_python.py
```

The conversion changes FRD/NED to FLU/ENU and explicitly labels the output frames. Firmware angles arrive in degrees; Python quaternions use `[w, x, y, z]`. Heading north becomes +Y in ENU, while east becomes +X. Position and velocity are not available from this attitude sample and are not fabricated.

These examples only request `HELLO` and `STATUS`. They do not arm, write parameters, save flash, or command motors. They demonstrate observation and data conversion, not an onboard control loop. A missing/unconfigured IMU can report invalid or uninformative attitude: verify sensor state in the configurator first.

The more advanced `firmware/tools/akcontrol*.py` experiments require an externally built C control library; that host harness is intentionally excluded from this trimmed public tree.

Run the examples' offline checks:

```sh
python -m unittest discover -s examples/firmware -p 'test_*.py'
```

[Configurator guide](../../docs/web-configurator.md) | [Firmware guide](../../docs/flight-controller-firmware.md)
