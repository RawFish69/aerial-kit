# Firmware and configurator publication — 2026-10-01

The `firmware/` flight-controller firmware and `apps/configurator/` web app are now components of the public Aerial Kit repository. Their relative layout is retained so source citations, fixtures and drift checks resolve to the same firmware.

Only component source, tests, documentation and SVG assets are included. Toolchains, build outputs, local dependencies, agent files and private host configuration are excluded. Historical bench evidence remains dated to its original observation. Machine-specific absolute paths in the exported documentation/evidence are replaced with portable placeholders; this does not change the measured results. `akcontrol_cascade.py` now defaults to this repository's root rather than a private machine path, while retaining `AK_REPO` as an override.

The root MIT licence remains unchanged. The newly published firmware and configurator retain GPL-3.0-or-later and include component licence files. The selected logo is the original 01 Rotor A.

Publication is source availability. It does not assert flight qualification or deploy a hosted web app.

## Export verification

The publication snapshot includes the configurator changes from source revision `be73890`. On 2026-10-01, the public tree passed:

- STM32F405 WeAct and Feather builds and image/symbol checks, with ARM GNU Toolchain on PATH.
- Firmware `make test`: 3,463 core/register checks, zero failures, plus the host simulation scenarios and tool checks.
- Board resource validator: 36 checks, zero failures. It reports five previously documented ESP32 C3/S2 pin-conflict configurations; those are known limitations, not newly validated hardware.
- Configurator: 285 tests passed, 10 live SITL tests skipped without their external simulator.
- Type checking and static production build, firmware source/table/dialect drift checks, and HTTPS subpath asset checks.
- Python sdist and wheel build; new firmware/app directories stay out of both distributions.
- Local links in the new/relocated guides resolve. This run did not flash or exercise physical hardware.

The existing Vite/Vitest development dependencies produce npm audit findings, including a critical advisory for the Vitest UI server. The test command here runs headlessly, without that server. A dependency upgrade remains separate work; these results do not claim an audit-clean dependency tree.

## Layout and presentation update - 2026-10-01

AK firmware now lives directly in `firmware/`. The older PlatformIO projects moved to `firmware/legacy/`. Configurator citations, fixtures, build helpers and documentation links follow the new layout. The earlier publication used the now-retired `aerialkit/` source directory.

The repository README has one Aerial Kit title, short dashes, configurator screenshots before the hover video, and the ROS navigation image below that video. The docs use the original symbol without a light/dark logo comparison.

## Public source scope - 2026-10-01

The public runtime now includes only Feather F405 and ESP32DEV profiles. Large C host-test/simulator harnesses and unused board/architecture ports are excluded; they remain in the development workspace and prior Git history. Earlier test totals above describe the full publication snapshot, not a host suite available in the trimmed Makefile. The active public checks are the Feather build/image checks and configurator tests/drift checks.

The trimmed Feather F405 image builds and passes symbol/layout checks. Configurator tests (285 passed, 10 external SITL cases skipped) and all three drift checks pass after the move; the legacy wing host test passes 430 checks. ESP32 source inventory now includes `ak_console_link.c`, required by the current main loop. ESP-IDF is not installed on this build host, so a fresh ESP32 link was not run here.

Source-size measurement for `.py`, `.c`, `.h`, `.cpp`, `.ts` and `.tsx`: C/C++ decreases from 3,583,264 to 1,523,541 bytes (57.5% removed). Python is the largest individual language at 1,907,170 bytes, approximately 44.6% of those measured source bytes. This is a plurality, not a claim of more than 50% Python. GitHub Linguist may report different percentages because it classifies and excludes files differently.


## Firmware runtime refresh - 2026-10-07

Firmware source revision: `aa47204069b2781e3fd216d230a800927697d97a`.
The source checkout was pulled with fast-forward only and was already current.
This refresh preserves the Feather F405/ESP32DEV scope and the trimmed build
guards. It includes the new runtime dependencies, selected protocol/Python
clients and contract vectors, current protocol documentation and Feather pin
map. Legacy files are unchanged. No commit, push, flash or hardware operation
was performed as part of this refresh.

See [current runtime and integration notes](../firmware/docs/35-public-runtime-sync.md).
The configurator requires a separate compatibility review for PERF, motor
telemetry capability/status handling, 87-byte blackbox records, parameter-table
changes and source citations. RC_CHANNELS retains its binary layout; CRSF link
statistics were added to runtime diagnostics. Both public boards still report
no motor telemetry capture capability.

Verification on the publication checkout:

- Feather F405 built with ARM GNU Toolchain 13.3.Rel1, `REV=aa47204`,
  in a fresh `OUT=build/public-sync`; all image/vector/symbol checks passed.
  Binary: 157,468 bytes; ELF text/data/BSS: 157,364 / 100 / 103,916 bytes.
  Linker RAM usage: 104,020 / 131,072 bytes (79.36%).
- Invalid board, architecture and part overrides are rejected by the public
  Makefile. The source-scope check passes and verifies the explicit ESP32 core
  inventory, including filter, log encoder and motor-health dependencies.
- Python contract check: 168 checks, zero failures. MSP client checks pass;
  external Betaflight/INAV comparisons are unavailable without their checkouts.
- MAVLink local frame checks pass; external pymavlink comparison is unavailable.
  GUI MSP checks are skipped: no display or virtual-display helper is installed.
- Direct Python parsing checks pass for motor measurement validity, physical
  units, malformed motor replies, 51/66/87-byte log compatibility, rejection of
  partial log extensions, and the PERF reply.
- ESP-IDF was not found in the environment or usual home, optional-software
  and workspace locations. No fresh ESP32 compile/link is claimed.
- No omitted C host suite or simulator was imported or run. Existing ignored
  binaries from an older checkout are not evidence for this runtime.
- Whitespace validation passes. Updated publication files were scanned for
  machine-specific paths; no such paths were introduced.

`python3 scripts/check_source_scope.py` measures real source bytes and enforces
Python's individual-language plurality. At this verification snapshot:

| Language | Source bytes |
| --- | ---: |
| Python | 2,081,484 |
| C, including .h | 2,059,107 |
| TypeScript, including TSX | 846,343 |
| C++ | 184,936 |
| JavaScript | 56,128 |
| Shell | 50,339 |

Python leads C by 22,377 bytes; C and C++ combined are 2,244,043 bytes.
This is an individual-language plurality, not a Python majority or a claim
that Python exceeds combined C/C++. Counts include legacy sources and
nonignored untracked work, and may change while other components are edited.
No `.gitattributes` language overrides or padding were used.
