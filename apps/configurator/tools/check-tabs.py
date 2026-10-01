#!/usr/bin/env python3
"""Check that the documented tab list is the one the registry actually has.

`src/ui/tabs.tsx` is the rail: the ordered list of sections, which of them can
be opened, and — for the ones that cannot — the sentence saying why. This
script keeps the table in `docs/BUILD-AND-DEPLOY.md` from drifting away from it.

The reason it is a check and not a generator: a generator would have to be run,
and a document that is only correct after someone remembers to run something is
a document that is wrong. This way the two disagree *loudly* or not at all.

What this checks, stated plainly so nobody reads more into it than is there:

  1. The documented tabs are the registry's tabs, in the same order.
  2. The documented `Opens` column agrees with whether the registry marks the
     tab unavailable.

It does **not** compare the reason text. The sentences live in the TypeScript
where they can be read beside the code that decides them, and duplicating them
into a table would create a second copy to go stale. The table's reason column
summarises; the registry states.

Usage:
    python3 tools/check-tabs.py [--repo-root DIR]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

BEGIN = "<!-- tabs:begin -->"
END = "<!-- tabs:end -->"

# One entry of the TABS array begins at `id: '...'`. Everything up to the next
# one belongs to it, which is how `unavailable` is attributed to a tab without
# parsing TypeScript.
ENTRY = re.compile(r"\bid\s*:\s*'([^']+)'")
LABEL = re.compile(r"\blabel\s*:\s*'([^']+)'")
UNAVAILABLE = re.compile(r"\bunavailable\s*:")


def registry(text: str) -> list[tuple[str, str, bool]]:
    """(id, label, isUnavailable) for each entry of the `TABS` array, in order."""
    start = text.index("export const TABS")
    body = text[start:]
    marks = [match.start() for match in ENTRY.finditer(body)]
    entries: list[tuple[str, str, bool]] = []
    for position, mark in enumerate(marks):
        end = marks[position + 1] if position + 1 < len(marks) else len(body)
        chunk = body[mark:end]
        found = ENTRY.search(chunk)
        if found is None:  # unreachable: `mark` came from the same pattern
            raise ValueError("a tab entry lost its id while being read")
        identifier = found.group(1)
        label = LABEL.search(chunk)
        if label is None:
            raise ValueError(f"the tab {identifier!r} has no label")
        entries.append((identifier, label.group(1), bool(UNAVAILABLE.search(chunk))))
    return entries


def documented(text: str) -> list[tuple[str, bool]]:
    """(label, opens) from the marked table in the deploy document."""
    if BEGIN not in text or END not in text:
        raise ValueError(f"the document has no {BEGIN} … {END} block")
    block = text.split(BEGIN, 1)[1].split(END, 1)[0]
    rows: list[tuple[str, bool]] = []
    for line in block.splitlines():
        line = line.strip()
        if not line.startswith("|") or set(line) <= set("|-: "):
            continue
        cells = [cell.strip() for cell in line.strip("|").split("|")]
        if len(cells) < 2 or cells[0].lower() in {"tab", "section"}:
            continue
        # `yes`/`no`, and anything else is a third answer this table does not
        # have — so it fails rather than being read as whichever is convenient.
        if cells[1] not in {"yes", "no"}:
            raise ValueError(f"the row for {cells[0]!r} says {cells[1]!r}, not yes or no")
        rows.append((cells[0], cells[1] == "yes"))
    return rows


def find_root() -> Path | None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "aerialkit" / "src").is_dir():
            return parent
    return None


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--repo-root", default=None)
    args = parser.parse_args(argv)

    root = Path(args.repo_root).resolve() if args.repo_root else find_root()
    if root is None:
        print("check-tabs: no repository root found")
        return 1

    app = root / "apps" / "configurator"
    try:
        entries = registry((app / "src" / "ui" / "tabs.tsx").read_text(encoding="utf-8"))
        rows = documented((app / "docs" / "BUILD-AND-DEPLOY.md").read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        print(f"check-tabs: could not read the registry or the document: {error}")
        return 1

    problems: list[str] = []
    for position, (identifier, label, unavailable) in enumerate(entries):
        if position >= len(rows):
            problems.append(f"the registry has {label!r}; the document stops before it")
            continue
        shown, opens = rows[position]
        if shown != label:
            problems.append(f"position {position}: registry says {label!r}, document says {shown!r}")
            continue
        if opens == unavailable:
            problems.append(
                f"{label!r}: the document says it "
                f"{'opens' if opens else 'does not open'}, and the registry says the opposite"
            )
    for shown, _ in rows[len(entries) :]:
        problems.append(f"the document lists {shown!r}; the registry has no such tab")

    if problems:
        print(f"check-tabs: the documented rail is not the registry's — {len(problems)} disagreement(s)")
        for problem in problems:
            print(f"  {problem}")
        return 1

    print(f"check-tabs: {len(entries)} tab(s) agree with docs/BUILD-AND-DEPLOY.md, in order")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
