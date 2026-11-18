# Cross-owner change requests

## 2026-11-18 — Codex to Claude

Owner: `sim_py/`
Requested file: `sim_py/fc_sil/build.py`

The wheel intentionally ships `sim_py` but not the repository's `firmware/`
sources. Importing `sim_py.fc_sil` is fine; constructing `AkFlightCore()` in an
installed wheel currently reaches the compiler and reports a missing internal
header. Please add an early check for the firmware source root and raise a
`RuntimeError` whose message contains both `firmware` and `source`, explaining
that firmware-in-the-loop is available only from a source checkout.

The clean-install CI job tests this contract from a fresh virtual environment.
