#!/usr/bin/env bash
#
# esp32-proto.sh - drive the config protocol against the ESP32 port in QEMU.
#
# Builds the IDF project, merges a flash image, boots it under QEMU, and talks
# to it with the same client a person or a companion computer would use. See
# tools/esp32_proto_check.py for why that is worth doing at all.
#
set -euo pipefail

root="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
idf="${IDF_PATH:-$HOME/esp-idf}"

[ -f "$idf/export.sh" ] || {
    echo "esp32-proto: no ESP-IDF at $idf (set IDF_PATH)" >&2
    exit 1
}

# shellcheck disable=SC1090
. "$idf/export.sh" >/dev/null

cd "$root"
idf.py -C ports/esp32 build >/dev/null

# Which medium that just built. It has to be the emulated one, because every
# check below talks to QEMU, and QEMU's ESP32 has no radio: a guest built for
# Wi-Fi comes up with no network, and the failure reads as a firmware that
# stopped answering rather than as a build that chose the wrong cable. IDF
# keeps the sdkconfig it generated, so this is also the state a machine is in
# after somebody built the radio here once.
grep -q '^CONFIG_AK_NET_OPENETH=y' ports/esp32/sdkconfig || {
    echo "esp32-proto: ports/esp32/sdkconfig is not the emulated-Ethernet" >&2
    echo "             configuration, and QEMU has no radio. Delete that file" >&2
    echo "             and run this again to regenerate it from sdkconfig.defaults." >&2
    exit 1
}

# The board a real devkit becomes, compiled on every build and never run here.
#
# A devkit has a radio and no PHY, and QEMU has the reverse, so a build carries
# one medium or the other (docs/17-esp32-port.md); on top of that the IMU and
# the battery divider are compile-time flags, and a bare devkit has neither. So
# the configuration that actually flies is one that *nothing* built until this
# line existed - and the evidence is in this repository: `net.c` used the
# core's `ak_strlen()` inside its Wi-Fi half without including the header that
# declares it, and that half had not compiled since the credentials moved into
# the parameter table.
#
# It is a *build*, not a run: QEMU has no radio. The separate build directory
# and sdkconfig are deliberate - IDF keeps the sdkconfig it generated and
# quietly ignores a default that changed behind it, which is the difference
# between this compiling the radio and this compiling the same thing twice.
echo "== the board a real devkit becomes: radio, imu and divider fitted"
idf.py -C ports/esp32 -B ports/esp32/build-wifi \
    -DSDKCONFIG=build-wifi/sdkconfig \
    -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.wifi" \
    -DAK_VARIANT_DEFINES="-DAK_BOARD_IMU_FITTED=1;-DAK_BOARD_VBAT_FITTED=1" \
    build >/dev/null

# And the pair is checked for being the pair it is named as.
#
# Two images leave this script - the emulated-Ethernet one every check below
# boots, and the radio one a devkit is flashed with - and each of them is
# useless in the other's place. Neither mistake is visible from the build log:
# the image that lost its medium still links, still boots, and comes up with no
# network at all, which on a devkit reads as "the configurator cannot see it"
# and sends somebody looking at cables and credentials. So the medium is read
# out of the map file where it cannot be faked: the radio's symbols on one side
# and the PHY's on the other. (`esp_wifi_init` is the first call of the Wi-Fi
# half in `src/arch/esp32/net.c`; `esp_eth_driver_install` is the same thing for
# the emulated MAC. A build that carries both is a build whose sdkconfig says
# one thing and whose defaults say another.)
for map in ports/esp32/build/aerialkit-esp32.map \
           ports/esp32/build-wifi/aerialkit-esp32.map; do
    [ -f "$map" ] || { echo "esp32-proto: no map at $map" >&2; exit 1; }
done
eth_map=ports/esp32/build/aerialkit-esp32.map
wifi_map=ports/esp32/build-wifi/aerialkit-esp32.map
eth_symbols=$(grep -c 'esp_eth_driver_install' "$eth_map" || true)
wifi_symbols=$(grep -c 'esp_wifi_init' "$wifi_map" || true)
stray_wifi=$(grep -c 'esp_wifi_init' "$eth_map" || true)
stray_eth=$(grep -c 'esp_eth_driver_install' "$wifi_map" || true)
if [ "$wifi_symbols" -eq 0 ] || [ "$eth_symbols" -eq 0 ] ||
   [ "$stray_wifi" -ne 0 ] || [ "$stray_eth" -ne 0 ]; then
    echo "esp32-proto: the two builds are not the two media they are named for:" >&2
    echo "  build       (emulated Ethernet): esp_eth_driver_install x$eth_symbols, esp_wifi_init x$stray_wifi" >&2
    echo "  build-wifi  (the radio)        : esp_wifi_init x$wifi_symbols, esp_eth_driver_install x$stray_eth" >&2
    exit 1
fi
echo "   one image per medium: the radio's symbols in build-wifi, the PHY's in build"

python3 tools/esp32_proto_check.py
# And then the same board through a different door: the configurator's window
# over the forwarded socket, which is the only client here that *streams*
# (a console link cannot - the firmware says so). It boots its own guest, so it
# is a second boot of the same image rather than a second build.
python3 tools/akconfig_net_check.py
