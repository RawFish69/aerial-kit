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
