#!/usr/bin/env python3
"""Count bytes read from the dual-role TX native USB CDC endpoint."""

from __future__ import annotations

from pathlib import Path


MARKER = "aerialkitRfNoteUsbRead"
NEEDLE = "    TxUSB->readBytes(buf, size);\n"
PROBE = (
    "#if defined(AERIALKIT_DUAL_ROLE)\n"
    + "    const auto usbAvailable = TxUSB->available();\n"
    + "    const auto usbRead = TxUSB->readBytes(buf, size);\n"
    + "    aerialkitRfNoteUsbRead(\n"
    + "        static_cast<uint16_t>(usbAvailable),\n"
    + "        static_cast<uint16_t>(usbRead));\n"
    + "#else\n"
    + NEEDLE
    + "#endif\n"
)


def apply(source: Path) -> None:
    text = source.read_text(encoding="utf-8")
    if MARKER in text:
        return
    # The airport branch reads first; the normal CRSF branch is the last one.
    index = text.rfind(NEEDLE)
    if index < 0:
        raise RuntimeError(
            f"no native USB read site found in {source}"
        )
    source.write_text(
        text[:index] + PROBE + text[index + len(NEEDLE):],
        encoding="utf-8",
    )


if __name__ == "__main__":
    import sys

    apply(Path(sys.argv[1]))
