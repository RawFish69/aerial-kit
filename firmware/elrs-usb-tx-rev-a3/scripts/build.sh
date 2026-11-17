#!/usr/bin/env bash
set -euo pipefail

mode="${1:-}"
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
        "$pio" run -d "$root/diagnostic"
        mkdir -p "$root/dist"
        cp "$root/diagnostic/.pio/build/aerialkit_rev_a3_diagnostic/firmware.bin" \
           "$root/dist/aerialkit-rev-a3-diagnostic.bin"
        ;;
    tx)
        "$root/scripts/bootstrap.sh"
        export PYTHONPATH="$HOME/.platformio/packages/tool-esptoolpy${PYTHONPATH:+:$PYTHONPATH}"
        "$pio" run -d "$vendor/ExpressLRS/src" \
            -e AERIALKIT_ELRS_USB_TX_REV_A3_via_UART
        mkdir -p "$root/dist"
        cp "$vendor/ExpressLRS/src/.pio/build/AERIALKIT_ELRS_USB_TX_REV_A3_via_UART/firmware.bin" \
           "$root/dist/aerialkit-rev-a3-elrs-tx.bin"
        ;;
    rx)
        "$root/scripts/bootstrap.sh"
        export PYTHONPATH="$HOME/.platformio/packages/tool-esptoolpy${PYTHONPATH:+:$PYTHONPATH}"
        "$pio" run -d "$vendor/ExpressLRS/src" \
            -e AERIALKIT_ELRS_USB_TX_REV_A3_RX_via_UART
        mkdir -p "$root/dist"
        cp "$vendor/ExpressLRS/src/.pio/build/AERIALKIT_ELRS_USB_TX_REV_A3_RX_via_UART/firmware.bin" \
           "$root/dist/aerialkit-rev-a3-elrs-rx.bin"
        ;;
    *)
        echo "usage: $0 diagnostic|tx|rx" >&2
        exit 2
        ;;
esac
sha256sum "$root/dist/"* > "$root/dist/SHA256SUMS"
