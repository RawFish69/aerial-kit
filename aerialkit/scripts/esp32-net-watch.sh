#!/usr/bin/env bash
#
# esp32-net-watch.sh - watch the ESP32's telemetry over the network.
#
#   scripts/esp32-net-watch.sh [seconds] [hz]
#
# This is the configurator's side of the firmware: the same client the NAS talks
# to a board with, a socket instead of a cable, and the stream a person watches
# while somebody else flies. It builds the image, boots it under QEMU with the
# firmware's port forwarded to this machine, and prints what comes back.
#
# Needs ESP-IDF (IDF_PATH, default ~/esp-idf) and the qemu-xtensa tool, the
# same as scripts/esp32-qemu.sh.
#
set -euo pipefail

seconds="${1:-5}"
rate="${2:-10}"
root="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
idf="${IDF_PATH:-$HOME/esp-idf}"
port=5555

[ -f "$idf/export.sh" ] || {
    echo "esp32-net-watch: no ESP-IDF at $idf (set IDF_PATH)" >&2
    exit 1
}

# shellcheck disable=SC1090
. "$idf/export.sh" >/dev/null
cd "$root"

echo "== building"
idf.py -C ports/esp32 build >/dev/null
python3 -c "import sys; sys.path.insert(0, 'tools'); import esp32_proto_check as c; c.build_flash_image()"

qemu=$(python3 -c "import sys; sys.path.insert(0, 'tools'); import esp32_proto_check as c; print(c.find_qemu())")
[ -n "$qemu" ] || { echo "esp32-net-watch: no qemu-system-xtensa" >&2; exit 1; }

echo "== running under qemu, port $port forwarded"
"$qemu" -M esp32 -m 4M \
    -drive "file=ports/esp32/build/qemu_flash.bin,if=mtd,format=raw" \
    -global driver=timer.esp32.timg,property=wdt_disable,value=true \
    -nic "user,model=open_eth,hostfwd=tcp:127.0.0.1:${port}-:${port}" \
    -nographic -monitor none >/dev/null 2>&1 &
qemu_pid=$!
trap 'kill "$qemu_pid" 2>/dev/null || true' EXIT

# Wait for the service, not for the port.
#
# The port answers immediately: QEMU accepts the host's connection before the
# guest has a listener, so a connection test here passes while the guest is
# still booting, and a client that connects into that window is talking to
# nobody. The firmware's own answer to a hello is the only honest signal - and
# it is the same thing this script is about to do anyway.
ready=0
for _ in $(seq 60); do
    if python3 - "$port" <<'PY'
import socket, sys
sys.path.insert(0, "tools")
from akproto import Client, HELLO
try:
    connection = socket.create_connection(("127.0.0.1", int(sys.argv[1])), 1.0)
    connection.settimeout(0.5)
    Client(connection.recv, connection.sendall).request(HELLO)
except Exception:
    sys.exit(1)
sys.exit(0)
PY
    then
        ready=1
        break
    fi
    sleep 0.5
done

if [ "$ready" != 1 ]; then
    echo "esp32-net-watch: the firmware never answered; is QEMU running?" >&2
    exit 1
fi

echo "== telemetry, at ${rate} Hz for ${seconds}s"
timeout --signal=INT "$seconds" python3 tools/akproto.py --host "127.0.0.1:${port}" \
    telemetry "$rate" || true
