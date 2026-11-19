# Aerial Kit overnight autonomous run — 2026-11-17

## Final report

Status: milestones 1-5 are complete. Milestone 6's documentation and bench
handoff are complete; FC-in-the-loop is blocked because the public firmware
tree does not contain the host control-library harness it documents.

Branch: `overnight-2026-11-17`, created from `sim-airframes` at
`86056707617108c3d65f1eef53ce035b2ea8bbd0`.

No physical hardware was opened, flashed or commanded. No RF was transmitted.
No sim-agent-owned simulator file was edited or staged. No flight-readiness claim
is made.

### Milestones

- **1, baseline:** complete. Asset checks pass for FPV, micro LR, research hex,
  research octo and twin wing; 12 ROS packages build.
- **2, ELRS host link:** complete. UART0, full-duplex, 400000 baud 8N1,
  non-inverted for the host/TX handset link; 420000 remains the RX-to-FC rate.
  Stock native USB-CDC is not the autonomous CRSF path.
- **3, CRSF library:** complete. RC packing/unpacking, DVB-S2 CRC, five telemetry
  frames, serial/memory transports and malformed-stream recovery are tested.
- **4, ROS backend:** complete for software/SIL. AETR + arm + mode mapping,
  gated arm, telemetry topics, RSSI/LQ output and timeout behavior are tested.
- **5, SIL radio:** complete. FPV at 0% and 10% loss and research hex at 0% all
  reached `Demo: disarm`; a mid-flight cut forced receiver disable and landing.
- **6, firmware HIL and bench:** bench package complete; FC-in-the-loop blocked.
  The public `firmware/Makefile` is ARM-only, has no `sil` or
  `control-experiment` target, and lacks the C ABI header/library and host test
  sources required by the retained Python binding.

### Commits

- `7316b94` — `hardware: integrate ELRS USB TX Rev A3`
- `e21ba10` — `hardware: normalize Rev A3 source file modes`
- `46814bc` — `build: find the local ROS Jazzy underlay`
- `ce2c5ae` — `hardware: document Rev A3 host CRSF link`
- `f8310cd` — `crsf: add pure Python codec and transports`
- `952b981` — `radio: add CRSF backend and ELRS SIL campaign`

### Tests before and after

| Point | Python | ROS |
| --- | ---: | ---: |
| Initial baseline | 514 passed, 84 subtests | 204 passed |
| Concurrent MuJoCo tree at baseline | 517 passed, 84 subtests | 205 passed |
| Final radio tree | **526 passed, 86 subtests** | **209 passed** |
| Radio review remediation, combined tree | **541 passed, 1 xfailed, 89 subtests** | **216 passed** |

The ROS suite retains three non-failing warnings: two deprecated Python
`sre_*` imports in dependencies and an unregistered `slow` marker.

### End-to-end SIL results

All runs used MuJoCo, 150 Hz command and receiver rates, and 8 ms configured
one-way link latency. The latency is a simulated configuration value, not a
wire measurement.

| Run | Result | Max altitude | Max ground speed | Max tilt | Samples |
| --- | --- | ---: | ---: | ---: | ---: |
| FPV, 0% loss | `Demo: disarm` | 3.16 m | 1.21 m/s | 16.0 deg | 9,395 |
| FPV, 10% loss | `Demo: disarm` | 3.17 m | 1.20 m/s | 29.0 deg | 9,357 |
| Research hex, 0% loss | `Demo: disarm` | 3.52 m | 1.36 m/s | 15.5 deg | 9,389 |

The 10% result is one successful current run, not a statistical reliability
claim. An earlier pre-commit 10% attempt also reached the demo disarm marker
but showed large state-estimation drift; it is superseded by the clean rerun
recorded in `radio-sil-fpv-10pct-current-20261117-075346.json`.

Link-loss test:

- Link cut at 2.00 m altitude.
- The receiver disabled after the 0.35 s freshness timeout; backend auto-land
  began 0.36 s after the cut.
- The plant returned to 0.40 m and idle with receiver enable false.
- Evidence: `radio-sil-fpv-link-loss-current-20261117-080436.json`.

Every mission helper used a process-group launch and was followed by a check
showing no `ros2 launch`, ROS node, Gazebo process or mission monitor remained.

### Remaining block and handoff

FC-in-the-loop with the real firmware and MuJoCo is blocked on a retained
development harness, not on radio behavior. Exact missing pieces and command
outputs are in `firmware-hil-feasibility-2026-11-17.txt`.

The human bench sequence is
`bench-checklist-2026-11-17.md`. Begin with props removed, a power-off role-switch
continuity check, an antenna or 50 ohm load, the non-RF diagnostic image and
lowest configured RF power. Then verify image selection, binding, the J3 UART
host link, and finally the real FC failsafe with props removed.

No sim-agent-owned change request is required. The existing concurrent
multirotor-backend auto-land behavior was used as-is and remains unstaged.

## Progress log

### Start

The branch was created without changing or discarding the existing
`sim-airframes` worktree. A pre-existing coherent change set was present:

- Aerial Kit ELRS USB TX Rev A3 hardware sources, previews and checksums.
- Pinned ExpressLRS 4.1.0 diagnostic/TX/RX integration.
- Reproducible target patch, build scripts and bench diagnostic evidence.

That change set is carried into this branch and will be committed separately
after the baseline verification commands pass.

Required architecture, software, hardware and firmware handoff documents were
read before implementation began.

### Milestone 1 — baseline

Status: complete.

Initial failure:

- `./scripts/build.sh` could not find the installed ROS 2 Jazzy underlay. The
  script searched only `/opt/ros`, while this machine uses
  `~/.local/share/ros/jazzy/setup.bash`.
- Fixed `scripts/build.sh` to honor `ROS_SETUP` and use the documented local
  Jazzy prefix before checking `/opt/ros`.

Asset generation checks:

- `fpv_quad_5in`: 45 files match.
- `micro_lr_quad`: 5 files match.
- `research_hex`: 59 files match.
- `research_octo`: 73 files match.
- `twin_wing`: 32 files match.

Workspace:

- `./scripts/build.sh`: 12 ROS packages finished successfully.

Tests:

- `sim_py/.venv/bin/python -m pytest -q sim_py/tests`:
  **514 passed, 84 subtests passed**.
- `./scripts/test_ros2.sh`: **204 passed**. Three warnings: two deprecated
  Python `sre_*` modules in ROS dependencies and the unregistered `slow`
  pytest mark.
- After preserving concurrent changes from the sim-agent-owned MuJoCo work, the
  pre-commit rerun produced **517 passed, 84 subtests passed** and
  **205 passed** respectively. The additional tests came from those
  concurrent simulator changes; no owner-protected file was staged or edited
  by this run.

No ROS launch, simulation node, Gazebo process or RViz process was active
before the baseline.

Commits:

- `7316b94` — `hardware: integrate ELRS USB TX Rev A3`.
- `e21ba10` — `hardware: normalize Rev A3 source file modes`.
- `46814bc` — `build: find the local ROS Jazzy underlay`.

### Milestone 2 — ELRS host-link design

Status: complete.

Pinned source: ExpressLRS `4.1.0`, commit
`a9d4a9cb5b5687c4c9d7e9e7fbdf44ad93651da6`, Targets commit
`8e72245fc2608592b29cd50b4ebd98eea08c1387`.

Decision:

- Host CRSF uses UART0: host TX to J3.4/GPIO44 and host RX from J3.3/GPIO43,
  plus ground.
- The link is full duplex because ESP32-S3 UART0 RX and TX have distinct pins.
- Normal host/handset rate is 400000 baud, 8N1, non-inverted. ExpressLRS TX
  cycles its documented handset baud table when no valid stream is present.
- Stock ESP32-S3 USB-CDC is the TxUSB/configuration stream, not the CRSF
  handset receiver. Autonomous RC input needs a 3.3 V USB-UART adapter.
- TX and RX are separate firmware images; SW2 does not change roles at runtime.
- The existing overlay builds diagnostic, TX and RX images against the pinned
  revisions. No flash or serial-device access occurred in this milestone.

Details: `firmware/elrs-usb-tx-rev-a3/docs/HOST-LINK.md`.

### Milestone 3 — pure-Python CRSF

Status: complete.

`aerial_kit/comms/crsf.py` now provides:

- 16-channel CRSF packing and unpacking in 22 bytes;
- CRC-8/DVB-S2 and incremental framing with bad-CRC resynchronization;
- link statistics, attitude, battery, GPS and flight-mode frames;
- `SerialTransport` with lazy pyserial import and a deterministic
  `MemoryTransport`;
- parser tests for valid frames, arbitrary partial input, garbage, bad CRC and
  interrupted streams.

The codec has no ROS dependency. Eight codec tests are in
`sim_py/tests/test_crsf.py`.

Commit: `f8310cd` — `crsf: add pure Python codec and transports`.

### Milestone 4 — ROS CRSF radio backend

Status: complete for software-in-the-loop.

`crsf_radio_backend_node` maps AETR on channels 1-4, mode on channel 5 and arm
on channel 6, matching AerialKit firmware's zero-based `AK_RC_MODE=4` and
`AK_RC_ARM=5`. It asserts arm only while `/uav/backend/enable` is true.
Disabled output is disarmed with minimum throttle; a stale command while
enabled becomes neutral hover. Attitude, GPS and battery feed the existing
`hw_state_estimator_node`; RSSI, LQ and SNR are additive JSON on
`/uav/radio/link_stats`.

The receiver timeout is 0.35 s. The backend command timeout is 0.5 s. In SIL,
link loss disables the plant and an airborne multirotor auto-lands. For a real
FC the configured failsafe must be verified with props removed; the required
behavior and channel contract are documented in `docs/HARDWARE.md`.

The detailed multirotor registry fallback now accepts spec-driven names such as
`fpv_quad_5in` and `research_hex`. The radio mapping tests cover encode/decode,
disarmed output, spec tilt limits and link-cut polarity.

### Milestone 5 — simulated ELRS link and missions

Status: complete.

The SIL path is:

```text
command manager -> CRSF backend -> simulated ELRS link -> CRSF receiver
    -> MuJoCo plant -> CRSF telemetry -> hardware estimator -> command stack
```

The link applies configured rate, latency and packet loss. The mission helper
uses a 600 s budget and records the monitor's finite-state mission summary.
`run_radio_sil_link_loss.sh` waits for takeoff, cuts the link at 2 m, requires
receiver disable, and requires a controlled return to 0.4 m or below.

Mission evidence:

- `radio-sil-fpv-0pct-long-20261117-073739.json`
- `radio-sil-fpv-10pct-current-20261117-075346.json`
- `radio-sil-hex-0pct-current-20261117-075910.json`
- `radio-sil-fpv-link-loss-current-20261117-080436.json`

All runs ended with `Demo: disarm` except the link-loss probe, where reaching
the ground and receiver disable were the success condition. No ROS launch or
node remained after a run.

### Milestone 6 — firmware HIL feasibility and bench handoff

Status: bench documentation complete; FC-in-the-loop blocked.

Feasibility check:

- `make -n BOARD=HOST` rejects everything except `BOARD=FEATHER_F405`.
- `make -n sil` has no rule.
- The Makefile has no `control-check`, `control-experiment`, `HOST_OUT` or
  `aerialkit-fw-sim` target.
- `tools/akcontrol.py` and its checks remain, but `tools/ak_control.h`, the
  shared C control library and host test sources/models are absent from this
  public checkout.

The exact command outputs are in
`docs/evidence/firmware-hil-feasibility-2026-11-17.txt`. A HIL binary was not
fabricated. Restoring the retained development harness is required before the
firmware mixer can drive MuJoCo motors and simulated IMU data can feed the
firmware.

The ordered human handoff is
`docs/evidence/bench-checklist-2026-11-17.md`. It covers role-switch continuity,
antenna/load and lowest-power order, exact image/flash commands, binding, host
UART testing, the blocked HIL stage, and a props-off real-FC arm/stick/failsafe
test. Radio and SIL usage was added to `docs/HARDWARE.md`,
`docs/SOFTWARE_GUIDE.md` and `docs/STACK-GUIDE.md`.

### Radio review remediation

The review findings B1-B4 and S1-S4 were confirmed against ExpressLRS 4.1.0
and the AerialKit firmware. None was disputed. The final SIL rerun used
`ROS_DOMAIN_ID=61`, the firmware's CH5 mode/CH6 arm defaults, a 400000-baud host
default, and the CRSF receiver-to-FC rate retained at 420000.

- FPV, 0% loss: `Demo: disarm`, closest approach 0.41 m, 6,241 samples.
- FPV, 10% loss: `Demo: disarm`, closest approach 0.04 m, 7,699 samples.
- Mid-flight link cut at 2.02 m: receiver disabled and plant returned to
  0.38 m with enable false.

The SIL exercises decoded manual sticks in the plant's velocity mode. It does
not claim to validate a real FC's Angle-mode response; that remains a props-off
bench item. Full finding-by-finding evidence and source references are in
`radio-defect-fixes-2026-11-17.md`.
