#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == /dev/* ]]; then
    mode="dual"
    port="$1"
else
    mode="${1:-dual}"
    port="${2:-}"
fi
if [[ -z "$port" ]]; then
    echo "usage: $0 [/dev/ttyACM0] or $0 diagnostic|tx|rx /dev/ttyACM0" >&2
    exit 2
fi

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vendor="${ELRS_USB_TX_VENDOR:-$root/.vendor}"
pio="${PIO:-$HOME/.local/bin/pio}"
if [[ -z "${ELRS_USB_TX_CACHE:-}" ]]; then
    if [[ -d "$HOME/Projects/elrs-usb-tx/toolchain" ]]; then
        export ELRS_USB_TX_CACHE="$HOME/Projects/elrs-usb-tx/toolchain"
    else
        export ELRS_USB_TX_CACHE="$HOME/.cache/aerial-kit/elrs-usb-tx"
    fi
fi

case "$mode" in
    diagnostic)
        "$pio" run -d "$root/diagnostic" -t upload --upload-port "$port"
        ;;
    tx)
        "$root/scripts/bootstrap.sh"
        export PYTHONPATH="$HOME/.platformio/packages/tool-esptoolpy${PYTHONPATH:+:$PYTHONPATH}"
        "$pio" run -d "$vendor/ExpressLRS/src" \
            -e AERIALKIT_ELRS_USB_TX_REV_A3_via_UART \
            -t upload --upload-port "$port"
        ;;
    rx)
        "$root/scripts/bootstrap.sh"
        export PYTHONPATH="$HOME/.platformio/packages/tool-esptoolpy${PYTHONPATH:+:$PYTHONPATH}"
        "$pio" run -d "$vendor/ExpressLRS/src" \
            -e AERIALKIT_ELRS_USB_TX_REV_A3_RX_via_UART \
            -t upload --upload-port "$port"
        ;;
    dual)
        "$root/scripts/bootstrap.sh"
        artifact="$root/dist/aerialkit-rev-a3-elrs-dual-role.bin"
        manifest="$root/dist/aerialkit-rev-a3-elrs-dual-role.json"
        if [[ ! -f "$artifact" ]]; then
            echo "dual-role artifact not found; run ./scripts/build.sh dual first" >&2
            exit 2
        fi
        expected_sha="$(python3 -c \
            'import json,sys; print(json.load(open(sys.argv[1]))["sha256"])' \
            "$manifest")"
        actual_sha="$(sha256sum "$artifact" | awk '{print $1}')"
        if [[ "$actual_sha" != "$expected_sha" ]]; then
            echo "dual-role artifact hash mismatch; refusing to flash" >&2
            echo "expected $expected_sha" >&2
            echo "actual   $actual_sha" >&2
            exit 2
        fi
        python3 "$vendor/ExpressLRS/src/python/external/esptool/esptool.py" \
            --chip esp32s3 \
            --port "$port" \
            --baud 460800 \
            --before default_reset \
            --after hard_reset \
            write_flash -z \
            --flash_mode dio \
            --flash_freq 80m \
            --flash_size 8MB \
            0x0 "$artifact"
        ;;
    *)
        echo "unknown firmware mode: $mode" >&2
        exit 2
        ;;
esac
