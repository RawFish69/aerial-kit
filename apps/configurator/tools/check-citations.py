#!/usr/bin/env python3
"""Resolve every source citation the configurator makes, or fail.

The app's limitations are claims about the firmware, and a claim about the
firmware is worth exactly as much as the reader's ability to check it. This
script is what makes that checkable: it pulls the `citations` arrays out of the
TypeScript and resolves each `<path>:<symbol>` against the firmware tree.

Why symbols and not line numbers, in one line: `ak_proto.c:211-247` was a
citation in this app for weeks after it stopped pointing at anything, and
nothing failed. `ak_proto_io_t.on_change` cannot rot that way.

What this checks, stated plainly so nobody reads more into it than is there:

  1. The cited file exists under `aerialkit/`.
  2. The named symbol appears in that file as a whole word.

It does *not* check that the symbol is a definition rather than a mention, and
it does not check that the sentence around the citation is a fair reading of it.
A checker claiming to do either would be lying about the one thing this project
will not lie about.

Usage:
    python3 tools/check-citations.py [--repo-root DIR] [file ...]

Exits 0 when every citation resolves, 1 otherwise, naming each failure. With no
files it scans every `.ts`/`.tsx` under `apps/configurator/src`.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# A citation is a plain string literal in a `citations: [...]` array.
CITATION = re.compile(r"^[A-Za-z0-9_./-]+\.(?:c|h):[A-Za-z_][A-Za-z0-9_]*$")
CITATIONS_KEY = re.compile(r"\bcitations\s*:\s*\[")


def blank_spans(text: str) -> str:
    """A copy of `text` with every string literal and comment blanked to spaces.

    Same length, so offsets and line numbers carry over. Comments have to go
    because the comment explaining what a citation *is* contains the word
    `citations:`, and a scan that found it would report the prose as a claim.
    """
    out = list(text)
    i, n = 0, len(text)
    while i < n:
        ch = text[i]
        end = None
        if ch in "'\"`":
            j = i + 1
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text[j] == ch:
                    j += 1
                    break
                j += 1
            end = j
        elif text.startswith("//", i):
            j = i
            while j < n and text[j] != "\n":
                j += 1
            end = j
        elif text.startswith("/*", i):
            j = i + 2
            while j < n and not text.startswith("*/", j):
                j += 1
            end = min(j + 2, n)
        if end is None:
            i += 1
            continue
        for k in range(i, end):
            out[k] = " "
        i = end
    return "".join(out)


def collect(text: str) -> list[tuple[int, str]]:
    """Every citation in one file, with the line it sits on."""
    masked = blank_spans(text)
    found: list[tuple[int, str]] = []
    for key in CITATIONS_KEY.finditer(masked):
        depth = 1
        i = key.end()
        while i < len(masked) and depth > 0:
            if masked[i] == "[":
                depth += 1
            elif masked[i] == "]":
                depth -= 1
            i += 1
        body_start, body_end = key.end(), i - 1
        # The literals themselves were blanked in `masked`, so they are read
        # back out of the original text between the two offsets.
        for literal in re.finditer(r"'([^'\\]*)'", text[body_start:body_end]):
            value = literal.group(1)
            line = text.count("\n", 0, body_start + literal.start()) + 1
            found.append((line, value))
    return found


def source_files(root: Path, given: list[str]) -> list[Path]:
    if given:
        return [Path(item) for item in given]
    src = root / "apps" / "configurator" / "src"
    return sorted([*src.rglob("*.ts"), *src.rglob("*.tsx")])


def resolve(root: Path, citation: str) -> str | None:
    """None when the citation resolves, otherwise the sentence saying why not."""
    path_part, symbol = citation.rsplit(":", 1)
    target = root / path_part
    if not target.is_file():
        return f"{path_part} does not exist under the repository root"
    body = target.read_text(encoding="utf-8", errors="replace")
    if not re.search(rf"\b{re.escape(symbol)}\b", body):
        return f"{path_part} no longer mentions {symbol}"
    return None


def find_root() -> Path | None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "aerialkit" / "src").is_dir():
            return parent
    return None


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--repo-root",
        default=None,
        help="the checkout to resolve citations against (default: walk up for aerialkit/)",
    )
    parser.add_argument("files", nargs="*", help="TypeScript files to scan")
    args = parser.parse_args(argv)

    root = Path(args.repo_root).resolve() if args.repo_root else find_root()
    if root is None:
        print("check-citations: no repository root found (no aerialkit/src above this file)")
        return 1

    files = source_files(root, args.files)
    if not files:
        print("check-citations: no TypeScript sources to scan")
        return 1

    total = 0
    failures: list[str] = []
    for path in files:
        text = path.read_text(encoding="utf-8")
        for line, citation in collect(text):
            total += 1
            problem = resolve(root, citation) if CITATION.match(citation) else (
                "is not written as <path>:<symbol>"
            )
            if problem is not None:
                try:
                    shown = path.relative_to(root)
                except ValueError:
                    shown = path
                failures.append(f"{shown}:{line}: {citation} — {problem}")

    if failures:
        print(f"check-citations: {len(failures)} of {total} citation(s) do not resolve")
        for failure in failures:
            print(f"  {failure}")
        return 1

    print(f"check-citations: {total} citation(s) resolved against {root}/aerialkit")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
