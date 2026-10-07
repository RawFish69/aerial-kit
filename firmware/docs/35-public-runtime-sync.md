# Public runtime sync - 2026-10-07

Runtime source snapshot: `aa47204069b2781e3fd216d230a800927697d97a`.
The public targets remain Feather F405 and ESP32DEV. Legacy projects are unchanged.
Older documents in this directory describe historical builds and a larger
development tree; commands such as `make host`, `make test`, and
`make contract-vectors` are not targets in the trimmed public Makefile.

## Runtime changes

The runtime includes the microsecond scheduler and timing profiler, filter
library and dynamic/RPM filter primitives, DShot GCR/EDT/capture decoders,
motor health logic, and expanded blackbox encoding. Including a decoder does
not establish that a board has an operational bidirectional DShot capture path.

Feather probes BNO055 addresses 0x28/0x29 followed by LSM6DSO 0x6B/0x6A
on I2C1. The BNO055 uses raw sensor readings in AMG mode; flight attitude
remains the firmware estimator's output. The driver reports its sample rate so
the loop does not assume a fast SPI IMU. See the board header and
[pin map](34-feather-pinmap.md) for wiring and fitted-hardware options.
CRSF link-statistics frames now update receiver diagnostics independently of
RC channel frames. Receiver freshness still depends on channel data.

The Makefile includes the filter source directory and retains its board,
architecture and part guards. The ESP-IDF source list contains every portable
core C file except `src/core/time.c`, replaced by the FreeRTOS port clock.
This includes log encoding, motor health and RPM filter dependencies.

## Client integration

[Protocol reference](16-protocol.md) and `src/core/ak_proto.h` define the wire.
The Python clients and golden contract data have been synchronized.

- Capability bits 12 and 13 describe PERF (0x15) and MOTOR_TELEMETRY (0x16).
  Query capabilities before enabling controls.
- PERF distinguishes loops counted from samples timed; section averages are in
  tenths of a microsecond and load is per mille, a lower bound.
- MOTOR_TELEMETRY distinguishes no capture path, no measurement, and measured
  zero speed. RPM requires its validity flag and the board's pole count;
  do not infer RPM from an assumed pole count. Both public boards currently
  leave the capability clear and answer NONE: this sync does not enable
  physical ESC telemetry.
- Blackbox records are now 87 bytes, with filter and controller fields.
  The Python reader also accepts historical 51-byte and 66-byte records.
- CRSF diagnostics change at the receiver/console layer; the binary
  RC_CHANNELS reply layout is unchanged.
- New parameter rows and source line locations require configurator table and
  citation drift checks. BNO055 is an additional sensor driver name.
- The selected Python client snapshot also fills gaps in output information,
  output tests, log streaming, preflight, calibration and mission helpers.

The configurator is maintained separately in this sync. Its protocol decoder,
feature vocabulary, log reader, parameter inventory and source citations must
be checked against this snapshot before claiming application compatibility.

## Repeatable checks

From the repository root:

```sh
python3 scripts/check_source_scope.py
cd firmware
make all check
python3 tools/contract_check.py
python3 tools/msp_check.py
python3 tools/mavlink_check.py
```

Put ARM GNU Toolchain on PATH for image checks. Keep build variants in separate
OUT directories. Contract checks validate golden vectors without the omitted C
host suite. MSP/MAVLink checks report unavailable external reference checkouts;
those reference comparisons are not claimed when missing.
Simulator-dependent client suites need a matching external simulator and must
not be run against stale binaries left by an earlier publication.

The source-size check counts actual bytes in tracked and nonignored untracked
source files, including legacy code. It groups C with .h headers and TypeScript
with TSX, excludes ignored build/dependency files through Git, and fails if
Python loses its individual-language plurality. No Linguist override is used.
