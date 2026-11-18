#!/usr/bin/env python3
"""Install the Aerial Kit target layout into a pinned ExpressLRS Targets tree."""

import json
import shutil
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} EXPRESSLRS_TARGETS_DIR", file=sys.stderr)
        return 2

    project = Path(__file__).resolve().parent.parent
    targets_dir = Path(sys.argv[1]).resolve()
    targets_file = targets_dir / "targets.json"
    if not targets_file.is_file():
        print(f"targets.json not found under {targets_dir}", file=sys.stderr)
        return 1

    targets = json.loads(targets_file.read_text(encoding="utf-8"))
    targets["aerialkit"] = {
        "name": "Aerial Kit",
        "rx_2400": {
            "usb_tx_rev_a3": {
                "product_name": "Aerial Kit ELRS USB TX Rev A3 2.4GHz RX",
                "lua_name": "AK USB RX A3",
                "layout_file": "AerialKit ELRS USB TX Rev A3.json",
                "upload_methods": ["uart", "wifi"],
                "min_version": "4.1.0",
                "platform": "esp32-s3",
                "firmware": "Unified_ESP32S3_2400_RX",
            }
        },
        "tx_2400": {
            "usb_tx_rev_a3": {
                "product_name": "Aerial Kit ELRS USB TX Rev A3 2.4GHz TX",
                "lua_name": "AK USB TX A3",
                "layout_file": "AerialKit ELRS USB TX Rev A3.json",
                "upload_methods": ["uart", "wifi"],
                "min_version": "4.1.0",
                "platform": "esp32-s3",
                "firmware": "Unified_ESP32S3_2400_TX",
            }
        },
    }
    targets_file.write_text(
        json.dumps(targets, indent=4, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    for role in ("TX", "RX"):
        source = project / "targets" / role / "AerialKit ELRS USB TX Rev A3.json"
        destination = targets_dir / role / source.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)

    print(f"Installed Aerial Kit target under {targets_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
