#!/usr/bin/env python3
"""Wire the dual-role loop entry points to the RF diagnostic sampler."""

from __future__ import annotations

from pathlib import Path


MARKER = "aerialkitRfSample(now);"
SOURCES = {
    "tx_main.cpp": ("  uint32_t now = millis();\n", "  "),
    "rx_main.cpp": ("    unsigned long now = millis();\n", "    "),
}


def apply(path: Path, now_line: str, indent: str) -> None:
    text = path.read_text(encoding="utf-8")
    if MARKER in text:
        return
    index = text.rfind(now_line)
    if index < 0:
        raise RuntimeError(
            f"no loop timestamp found in {path}"
        )
    replacement = now_line + f"{indent}{MARKER}\n"
    path.write_text(
        text[:index] + replacement + text[index + len(now_line):],
        encoding="utf-8",
    )


def apply_tree(source_dir: Path) -> None:
    for filename, (line, indent) in SOURCES.items():
        path = source_dir / filename
        if not path.is_file():
            # RX is compiled from a target overlay on some board families.
            continue
        apply(path, line, indent)


if __name__ == "__main__":
    import sys

    apply_tree(Path(sys.argv[1]))
