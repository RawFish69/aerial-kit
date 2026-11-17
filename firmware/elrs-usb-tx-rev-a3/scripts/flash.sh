#!/usr/bin/env bash
set -euo pipefail

mode="${1:-}"
port="${2:-}"
if [[ -z "$mode" || -z "$port" ]]; then
    echo "usage: $0 diagnostic|tx|rx /dev/ttyACM0" >&2
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
    *)
        echo "unknown firmware mode: $mode" >&2
        exit 2
        ;;
esac
