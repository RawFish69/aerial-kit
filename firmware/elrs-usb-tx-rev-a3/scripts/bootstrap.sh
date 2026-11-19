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
mkdir -p "$vendor"

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

if grep -q 'AERIALKIT_ELRS_USB_TX_REV_A3_via_UART' \
       "$express/src/targets/esp32s3-tx.ini" &&
   grep -q 'AERIALKIT_ELRS_USB_TX_REV_A3_RX_via_UART' \
       "$express/src/targets/esp32s3-rx.ini" &&
   grep -q 'AERIALKIT_DUAL_ROLE=1' "$express/src/targets/esp32s3-tx.ini" &&
   grep -q 'AERIALKIT_DUAL_ROLE=1' "$express/src/targets/esp32s3-rx.ini" &&
   grep -q 'board_build.partitions' "$express/src/targets/esp32s3-tx.ini" &&
   grep -q 'board_build.partitions' "$express/src/targets/esp32s3-rx.ini"; then
    echo "ExpressLRS patch already applied"
elif git -C "$express" apply --reverse --check "$patch_file" 2>/dev/null; then
    echo "ExpressLRS patch already applied"
else
    git -C "$express" apply "$patch_file"
fi

if grep -q 'SetUARTRequestedBaud' "$express/src/lib/Handset/CRSFHandset.h"; then
    echo "Native handset helper already applied"
elif git -C "$express" apply --reverse --check "$handset_patch_file" 2>/dev/null; then
    echo "Native handset helper already applied"
else
    git -C "$express" apply "$handset_patch_file"
fi

if grep -q 'SerialUSB_device' "$express/src/src/rx-serial/devSerialIO.h" &&
   grep -q 'UsbCrsfConnected' "$express/src/src/tx_main.cpp"; then
    echo "Native USB patch already applied"
elif git -C "$express" apply --reverse --check "$usb_patch_file" 2>/dev/null; then
    echo "Native USB patch already applied"
else
    git -C "$express" apply "$usb_patch_file"
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
