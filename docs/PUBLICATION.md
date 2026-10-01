# Firmware and configurator publication — 2026-10-01

The `aerialkit/` flight-controller firmware and `apps/configurator/` web app are now components of the public Aerial Kit repository. Their relative layout is retained so source citations, fixtures and drift checks resolve to the same firmware.

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
