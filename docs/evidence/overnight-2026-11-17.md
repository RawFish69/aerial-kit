# Aerial Kit overnight autonomous run — 2026-11-17

## Final report

Status: in progress.

Branch: `overnight-2026-11-17`, created from `sim-airframes` at
`86056707617108c3d65f1eef53ce035b2ea8bbd0`.

No physical hardware will be opened, flashed or commanded during this run.
No RF will be transmitted. Claude-owned simulator files will not be edited.

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
- After preserving concurrent changes from the Claude-owned MuJoCo work, the
  pre-commit rerun produced **517 passed, 84 subtests passed** and
  **205 passed** respectively. The additional tests came from those
  concurrent simulator changes; no owner-protected file was staged or edited
  by this run.

No ROS launch, simulation node, Gazebo process or RViz process was active
before the baseline.

Commits:

- `7316b94` — `hardware: integrate ELRS USB TX Rev A3`.
