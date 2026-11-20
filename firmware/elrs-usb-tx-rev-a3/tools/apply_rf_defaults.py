#!/usr/bin/env python3
"""Set the dual-role RX search to start on the TX default RF rate."""

from __future__ import annotations

from pathlib import Path


MARKER = "aerialkit starts on the TX default rate"
NEEDLE = "    scanIndex = config.GetRateInitialIdx();\n"
INSERT = (
    NEEDLE
    + "#if defined(AERIALKIT_DUAL_ROLE)\n"
    + "    // aerialkit starts on the TX default rate; normal cycling follows.\n"
    + "    scanIndex = enumRatetoIndexSafe(RATE_LORA_2G4_250HZ);\n"
    + "#endif\n"
)


def apply(source: Path) -> None:
    text = source.read_text(encoding="utf-8")
    if MARKER in text:
        return
    if text.count(NEEDLE) != 1:
        raise RuntimeError(
            f"expected one RX rate-initialization site in {source}, "
            f"found {text.count(NEEDLE)}"
        )
    source.write_text(text.replace(NEEDLE, INSERT), encoding="utf-8")


if __name__ == "__main__":
    import sys

    apply(Path(sys.argv[1]))
