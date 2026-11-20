#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vendor="${ELRS_USB_TX_VENDOR:-$root/.vendor}"
express="$vendor/ExpressLRS"
targets="$express/src/hardware"
express_commit="a9d4a9cb5b5687c4c9d7e9e7fbdf44ad93651da6"
targets_commit="8e72245fc2608592b29cd50b4ebd98eea08c1387"
patch_file="$root/patches/expresslrs-4.1.0-aerialkit.patch"
usb_patch_file="$root/patches/expresslrs-4.1.0-native-usb.patch"
handset_patch_file="$root/patches/expresslrs-4.1.0-native-handset.patch"
hardening_patch_file="$root/patches/expresslrs-4.1.0-hardening.patch"
led_state_patch_file="$root/patches/expresslrs-4.1.0-led-state.patch"
rmt_led_patch_file="$root/patches/expresslrs-4.1.0-rmt-led.patch"
rxtx_persistence_patch_file="$root/patches/expresslrs-4.1.0-rxtx-persistence.patch"
stamp_file="$express/.aerialkit-bootstrap.sha256"
mkdir -p "$vendor"

stamp="$({
    find \
        "$root/patches" \
        "$root/expresslrs-overlay" \
        "$root/partitions" \
        "$root/targets" \
        "$root/tools/apply_targets.py" \
        -type f -print0
} | sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1)"

if [[ ! -e "$express/.git" ]]; then
    git clone --depth 1 --branch 4.1.0 \
        https://github.com/ExpressLRS/ExpressLRS.git "$express"
fi
actual="$(git -C "$express" rev-parse HEAD)"
if [[ "$actual" != "$express_commit" ]]; then
    echo "ExpressLRS is at $actual, expected $express_commit" >&2
    exit 1
fi

if [[ ! -e "$targets/.git" ]]; then
    git clone https://github.com/ExpressLRS/targets.git "$targets"
fi
git -C "$targets" fetch --depth 1 origin "$targets_commit"
git -C "$targets" checkout --detach "$targets_commit"

if [[ -f "$stamp_file" ]] && [[ "$(cat "$stamp_file")" == "$stamp" ]]; then
    echo "ExpressLRS vendor tree is current"
else
    echo "Resetting ExpressLRS vendor tree for a clean patch application"
    git -C "$express" reset --hard "$express_commit"
    git -C "$express" clean -fd -e hardware
    git -C "$targets" reset --hard "$targets_commit"
    git -C "$targets" clean -fd

    git -C "$express" apply "$patch_file"
    git -C "$express" apply "$handset_patch_file"
    git -C "$express" apply "$usb_patch_file"
    git -C "$express" apply "$hardening_patch_file"
    git -C "$express" apply "$led_state_patch_file"
    git -C "$express" apply "$rmt_led_patch_file"
    git -C "$express" apply "$rxtx_persistence_patch_file"
    python3 "$root/tools/apply_rf_defaults.py" "$express/src/src/rx_main.cpp"
    printf '%s\n' "$stamp" > "$stamp_file"
fi

cp "$root/partitions/dual-role.csv" \
   "$express/src/aerialkit_dual_role.csv"
mkdir -p "$express/src/include" "$express/src/src"
cp "$root"/expresslrs-overlay/include/* "$express/src/include/"
cp "$root"/expresslrs-overlay/src/* "$express/src/src/"
if [[ -d "$root/expresslrs-overlay/test" ]]; then
    mkdir -p "$express/src/test"
    cp -R "$root"/expresslrs-overlay/test/. "$express/src/test/"
fi

python3 "$root/tools/apply_targets.py" "$targets"
