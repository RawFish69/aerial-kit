#!/usr/bin/env bash
#
# esp32-qemu.sh - build the ESP32 target and run it under QEMU.
#
#   scripts/esp32-qemu.sh [seconds]
#
# This is how the ESP32 port is verified here, with no board: ESP-IDF builds the
# image, and the QEMU that ships with it boots that image, runs the firmware's
# own selftest, and prints its console. The output is the evidence; run it and
# compare.
#
# Needs ESP-IDF (IDF_PATH, default ~/esp-idf with export.sh in it) and the
# qemu-xtensa tool. Neither is installed by this script - see
# docs/17-esp32-port.md for what they cost and how they were set up.
#
set -euo pipefail

seconds="${1:-20}"
root="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
idf="${IDF_PATH:-$HOME/esp-idf}"

[ -f "$idf/export.sh" ] || {
    echo "esp32-qemu: no ESP-IDF at $idf (set IDF_PATH)" >&2
    exit 1
}

# shellcheck disable=SC1090
. "$idf/export.sh" >/dev/null

cd "$root/ports/esp32"
idf.py build >/dev/null

echo "== running under qemu for ${seconds}s"
# A timeout is the normal way this ends: the firmware is a flight controller and
# does not exit.
timeout --signal=INT "$seconds" idf.py qemu 2>&1 | sed -n '/AerialKit Firmware/,$p' || true
