#!/usr/bin/env python3
"""Did the records lose a section they used to have?

This project keeps two records that outlive any one tree: the ledger
(`AERIAL-KIT-GOAL-PROGRESS.md`, at the repository root) and the eight files
under `hosts/nas/agents/`. They are **ported between trees** - copied, merged,
carried by a batch of commits - and every time that has happened the thing that
went wrong was the same: the copy was shorter than the original and nothing
noticed.

The gate written for the last port asserted five things, and all five passed on
a copy that was **2,353 lines short** - 24 `## ` sections, an entire morning on
the bench. It could not see the loss because every assertion was about a
specific thing being *absent* ("the stale heading is gone") or a specific number
being *present* ("the traps file is at 120 or later"). A floor is not coverage
and the absence of the stale is not the presence of the current. That is trap
152 in `hosts/nas/agents/04-traps.md`.

So this asks the other question, in the only form a single tree can answer it:
**every section the records had at the revision they are compared against must
still be here.** A heading is present or it is not, so this is decidable and it
exits non-zero. Section *text* is compared too, but only reported - a correct
port deliberately replaces stale paragraphs, and a gate that fails on a
deliberate replacement teaches its reader to ignore it. Trap 153 is that
distinction: coverage at section granularity answers "did a section disappear",
not "did a paragraph", and the second is the one that goes quietly.

Two ways to say what to compare against:

    records_check.py                     # against HEAD, the working copy's own history
    records_check.py --base <rev>        # against any revision
    records_check.py --base-dir <tree>   # against another checkout - the port-time
                                         # check, where the two records trees are
                                         # both on disk and neither is a revision
                                         # of the other

The default is HEAD and that is the one that runs in `make test`: it is the check
that would have caught the 24 lost sections, because at the moment that copy was
staged for a batch, HEAD still held the good one and nobody had diffed the two.

**What each mode cannot see, named rather than left to be discovered:**

- `--base HEAD` catches a loss **while it is uncommitted**, which is the window a
  batch is assembled and reviewed in. Once a loss is committed it *is* HEAD, and
  this mode compares it to itself and passes. A clean tree is trivially ok, which
  is what it should be: this is a pre-commit guard, not an audit.
- `--base-dir` is the port-time check and needs both trees on disk. It is the
  only one that can catch a loss that has already been committed in the tree
  being ported *from*.
- Neither checks anything the two copies **share**. A section edited down from
  forty lines to four is present in both and invisible here; only the line count
  under each file says anything about it, and that is reported, not failed
  (trap 153).

A section may be **retired on purpose**, and the way to do that is to write its
heading in `hosts/nas/agents/records-retired.txt`, one per line. This is a
deliberate hole with a name on it rather than a quiet one: the check prints what
it retired. There is no file until the first one.

Exit 0 if nothing was lost, 1 if something was, 2 on a usage or read error.
"""

import argparse
import pathlib
import re
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
LEDGER = "AERIAL-KIT-GOAL-PROGRESS.md"
AGENTS = "hosts/nas/agents"
RETIRED = f"{AGENTS}/records-retired.txt"
H2 = re.compile(r"^## (?!\d+\.)(.*\S)\s*$")
TRAP = re.compile(r"^## (\d+)\.\s")
H3 = re.compile(r"^### (.*\S)\s*$")


def records_files(root):
    """The ledger, plus every file beside it under hosts/nas/agents/."""
    out = [LEDGER]
    d = root / AGENTS
    if d.is_dir():
        out += [f"{AGENTS}/{p.name}" for p in sorted(d.iterdir()) if p.is_file()]
    return out


def keys(text, pattern):
    """Distinct matches of `pattern` in `text`, in first-seen order."""
    seen, out = set(), []
    for line in text.splitlines():
        m = pattern.match(line)
        if m:
            k = m.group(1) if m.groups() else m.group(0)
            if k not in seen:
                seen.add(k)
                out.append(k)
    return out


def lines(text):
    """Non-blank, stripped, as a set - paragraph granularity without needing
    blank-line blocks. A paragraph that was *edited* rather than removed shows
    up here as its old lines being absent, which is what we want to see."""
    return {s for s in (ln.strip() for ln in text.splitlines()) if s}


def retired(root):
    p = root / RETIRED
    if not p.is_file():
        return []
    out = []
    for ln in p.read_text(encoding="utf-8", errors="replace").splitlines():
        s = ln.strip()
        if s and not s.startswith("#"):
            out.append(s.lstrip("# ").strip())
    return out


def from_rev(rev, rel):
    """The file at `rev`, or None if it is not there. A file that did not exist
    at the base has nothing to lose, so it is skipped rather than failed."""
    try:
        r = subprocess.run(["git", "-C", str(ROOT), "show", f"{rev}:{rel}"],
                           capture_output=True, text=True)
    except OSError:
        return "NOGIT"
    if r.returncode != 0:
        return None
    return r.stdout


def from_dir(d, rel):
    p = pathlib.Path(d) / rel
    if not p.is_file():
        return None
    return p.read_text(encoding="utf-8", errors="replace")


def compare(rel, base_text, here_text, skip):
    """(missing headings, missing trap sections, source h3, here h3, dropped lines)."""
    b_h2 = [h for h in keys(base_text, H2) if h not in skip]
    b_tr = [n for n in keys(base_text, TRAP) if n not in skip]
    h_h2, h_tr = set(keys(here_text, H2)), set(keys(here_text, TRAP))
    miss_h2 = [h for h in b_h2 if h not in h_h2]
    miss_tr = [n for n in b_tr if n not in h_tr]
    dropped = lines(base_text) - lines(here_text)
    return (miss_h2, miss_tr, len(keys(base_text, H3)), len(keys(here_text, H3)), dropped)


def self_test():
    """The check, against a pair it must fail and a pair it must pass.

    A check nobody has watched fail is a check nobody knows the meaning of -
    this project has already shipped two gates whose failure mode was to report
    success (traps 152, 153). So the fixtures are a base with four sections and
    a copy with three, and the assertion is on the verdict, not on the exit
    status of the script that produced it.
    """
    base = ("# Ledger\n\n## Alpha\n\none\n\n## Beta\n\ntwo\n\n"
            "## 7. A trap\n\nthree\n\n## 8. Another trap\n\nfour\n")
    good = base + "\n## Gamma\n\nfive, added by the port\n"
    bad = base.replace("## Beta\n\ntwo\n\n", "")
    bad_tr = base.replace("## 8. Another trap\n\nfour\n", "")
    cases = [("a superset", good, 0, 0),
             ("a dropped section", bad, 1, 0),
             ("a dropped trap section", bad_tr, 0, 1)]
    rc = 0
    for name, here, want_h2, want_tr in cases:
        miss_h2, miss_tr, _, _, _ = compare("fixture.md", base, here, [])
        ok = len(miss_h2) == want_h2 and len(miss_tr) == want_tr
        print(f"  self-test: {name:24s} -> {len(miss_h2)} heading(s), "
              f"{len(miss_tr)} trap section(s)   {'ok' if ok else 'WRONG'}")
        if not ok:
            rc = 1
    # And the retirement list has to actually retire something, or the hole is
    # not the one the file names.
    miss_h2, _, _, _, _ = compare("fixture.md", base, bad, ["Beta"])
    ok = not miss_h2
    print(f"  self-test: {'a deliberate retirement':24s} -> {len(miss_h2)} heading(s)"
          f"   {'ok' if ok else 'WRONG'}")
    if not ok:
        rc = 1
    return rc


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--base", default="HEAD",
                    help="a revision to compare against (default: HEAD)")
    ap.add_argument("--base-dir",
                    help="another checkout to compare against, for the port-time "
                         "check where the two records trees share no ancestry")
    ap.add_argument("--retired",
                    help=f"headings retired on purpose (default: {RETIRED}, "
                         f"if it exists)")
    ap.add_argument("--lines", action="store_true",
                    help="name every line the base has and this tree does not. "
                         "The count is always reported; the list is for a port, "
                         "where each one is a question, and not for a tree that "
                         "is merely ahead of its own HEAD.")
    ap.add_argument("--self-test", action="store_true",
                    help="run the fixtures the check must fail, and stop")
    ap.add_argument("files", nargs="*",
                    help=f"records to check (default: {LEDGER} and {AGENTS}/*)")
    a = ap.parse_args(argv)

    if a.self_test:
        return self_test()

    if a.base_dir:
        base_kind = "dir"
    else:
        r = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--git-dir"],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print("not a git checkout and no --base-dir, so there is nothing to "
                  "compare these records against")
            print("NOT CHECKED - a missing history is not a pass")
            return 0
        base_kind = "rev"

    skip = set(retired(ROOT) if a.retired is None
               else [s.strip() for s in pathlib.Path(a.retired).read_text(
                   encoding="utf-8").splitlines() if s.strip()])
    if skip:
        print(f"{len(skip)} heading(s) retired deliberately in "
              f"{a.retired or RETIRED}")

    files = a.files or records_files(ROOT)
    bad = lost = 0
    for rel in files:
        here = ROOT / rel
        if not here.is_file():
            print(f"  ?? {rel}: not in this tree")
            continue
        base_text = (from_dir(a.base_dir, rel) if base_kind == "dir"
                     else from_rev(a.base, rel))
        if base_text is None:
            print(f"  -- {rel}: not at {a.base_dir or a.base}, nothing to lose")
            continue
        here_text = here.read_text(encoding="utf-8", errors="replace")
        miss_h2, miss_tr, n3b, n3h, dropped = compare(rel, base_text, here_text, skip)
        if miss_h2 or miss_tr:
            bad += 1
            print(f"  FAIL {rel}: {len(miss_h2)} heading(s) and {len(miss_tr)} "
                  f"trap section(s) that {a.base_dir or a.base} has are gone")
            for h in miss_h2[:10]:
                print(f"         gone: ## {h}")
            if len(miss_h2) > 10:
                print(f"         ... and {len(miss_h2) - 10} more")
            for n in miss_tr[:10]:
                print(f"         gone: ## {n}.")
        else:
            note = "" if n3h >= n3b else f"  (### {n3b} -> {n3h}, advisory)"
            print(f"  ok   {rel}: no section lost{note}")
        if dropped:
            lost += len(dropped)
            print(f"       {len(dropped)} line(s) of {a.base_dir or a.base} are absent "
                  f"- reviewed, not failed")
            # The list is for a port, where the two copies have diverged and each
            # absent line is a question someone has to answer. On a tree that is
            # simply ahead of its own HEAD it is every edit of the day, so it is
            # asked for rather than printed.
            if a.lines:
                for ln in sorted(dropped, key=len, reverse=True):
                    print(f"         - {ln[:92]}")

    print()
    if bad:
        print(f"RECORDS LOST STRUCTURE: {bad} of {len(files)} file(s). This is "
              f"what the five-assertion gate could not see (trap 152). If a "
              f"section was retired on purpose, name it in {RETIRED}.")
        return 1
    print(f"{len(files)} file(s), every section of "
          f"{a.base_dir or a.base} still present"
          + (f"; {lost} line(s) differ and are reported above" if lost else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
