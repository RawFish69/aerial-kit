#!/usr/bin/env python3
"""Does each aggregate target reach every stage the suite declares?

The suite has one definition, in `scripts/ci.sh`: `GROUP_suite`, the assignment
`--only suite` reads. Everything else that describes the suite is describing
*that*. On 2026-09-24 two of the three descriptions were narrower than it and
nobody had noticed, because both were prose in a recipe comment and prose does
not fail:

  * `make coverage` ran `make test` and two protocol checks, and published a
    map which reported the five oracles' lines - among them the flight core's
    clock-reset branch - as lines nothing reaches. A coverage map is read as a
    work queue, so a line wrongly marked unreached is a test written twice.
  * `make sanitize` ran the same `make test`, so the same five had never been
    under the sanitizers at all, while `docs/28-build.md` said of that stage
    that "no stage is excused".

Neither was a mistake in the writing; both were the ordinary result of a list
kept in three places. So this checks the property rather than the prose: for
each aggregate target, walk the `$(MAKE)` invocations and the `python3 tools/*`
invocations its recipe actually reaches, and ask whether every stage of
`GROUP_suite` is reached by one of them.

**Reached means the check runs, not that the target is named.** `make test`
does not invoke `make boards-check`; it invokes `tools/board_resources_check.py`
directly, which is the same check. A stage counts as reached when either its
own make target is in the closure or a tool from that target's recipe is. This
is the substance of the question - the point of running a stage is the check
inside it - and it is why `boards`, `contract` and `control` are reached by
both aggregates without either one naming them.

**One-directional, deliberately.** This asks that the suite be covered. It does
not ask that coverage and sanitize run *only* the suite: both drive the
protocol checks and the fuzzer as well, which are stages of their own and
belong to a different list. A check in both directions would be a check that
the two groups are the same group, and they are not.

`--self-test` feeds it synthetic pairs, including one whose aggregate misses a
declared stage, so the check is known to be able to fail.
"""

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
CI = ROOT / "scripts" / "ci.sh"
MAKEFILE = ROOT / "Makefile"

# The targets that claim to run the suite.
AGGREGATES = ("coverage", "sanitize")

# A stage that genuinely cannot be reached has to be written here with the
# reason, which is the point: the failure mode being guarded against is a
# narrow definition nobody chose, not a wide one somebody did.
EXCUSED = {}

STAGE_LINE = re.compile(r'^stage\s+(\S+)\s+"[^"]*"\s+"[^"]*"\s+(.*)$', re.M)
TARGET_LINE = re.compile(
    r'^([A-Za-z0-9_./$()%+-]+(?:\s+[A-Za-z0-9_./$()%+-]+)*)\s*:(?!=)')
PY_TOOL = re.compile(r'python3\s+(tools/[\w.\-]+\.py)')


def group(text, name):
    """The stages of a ci.sh GROUP_ assignment."""
    m = re.search(r'^GROUP_%s="((?:[^"]|\n)*?)"' % name, text, re.M)
    return m.group(1).split() if m else None


def stage_commands(text):
    """stage name -> the make command ci.sh runs for it."""
    return {m.group(1): m.group(2).strip() for m in STAGE_LINE.finditer(text)}


def make_targets(command):
    """The make targets named in a stage's command."""
    words = command.split()
    if not words or words[0] != "make":
        return set()
    return {w for w in words[1:] if not w.startswith("-")}


def recipes(text):
    """make target -> its recipe lines."""
    out, current = {}, []
    for raw in text.splitlines():
        if raw.startswith("\t"):
            for target in current:
                out.setdefault(target, []).append(raw[1:])
            continue
        m = TARGET_LINE.match(raw)
        current = m.group(1).split() if m else []
    return out


def logical(lines):
    """Recipe lines with backslash continuations joined."""
    out, buffer = [], ""
    for line in lines:
        buffer = buffer + line if buffer else line
        if buffer.rstrip().endswith("\\"):
            buffer = buffer.rstrip()[:-1]
            continue
        out.append(buffer)
        buffer = ""
    if buffer:
        out.append(buffer)
    return out


def invoked(lines):
    """The `$(MAKE) <targets>` calls and `python3 tools/*` calls in a recipe."""
    targets, tools = set(), set()
    for line in lines:
        make = re.search(r'\$\(MAKE\)', line)
        if make:
            for token in line[make.end():].split():
                if token.startswith("-") or token.startswith("$("):
                    continue
                if "=" in token:
                    break
                targets.add(token)
        tools.update(PY_TOOL.findall(line))
    return targets, tools


def closure(recipes_, start):
    """Everything a target reaches, and every tool those recipes invoke."""
    seen, tools, frontier = set(), set(), [start]
    while frontier:
        target = frontier.pop()
        if target in seen:
            continue
        seen.add(target)
        sub, found = invoked(logical(recipes_.get(target, [])))
        tools |= found
        frontier.extend(sub - seen)
    return seen, tools


def check(ci_text, makefile_text, aggregates=AGGREGATES, excused=EXCUSED):
    """(failures, notes) - every suite stage reached by every aggregate."""
    suite = group(ci_text, "suite")
    if suite is None:
        return ["ci.sh does not declare GROUP_suite where this reads it"], []
    commands = stage_commands(ci_text)
    if not commands:
        return ["ci.sh has no `stage` directives where this reads them"], []

    recipes_ = recipes(makefile_text)
    failures, notes = [], []
    for aggregate in aggregates:
        if aggregate not in recipes_:
            failures.append("the Makefile has no `%s` target" % aggregate)
            continue
        seen, tools = closure(recipes_, aggregate)
        missed = []
        for name in suite:
            if name in excused:
                continue
            own = make_targets(commands.get(name, ""))
            if not own:
                failures.append("ci.sh runs no `make` for stage `%s`, so this "
                                "cannot tell whether %s reaches it"
                                % (name, aggregate))
                continue
            stage_tools = set()
            for target in own:
                stage_tools |= invoked(logical(recipes_.get(target, [])))[1]
            if own & seen or stage_tools & tools:
                continue
            missed.append(name)
        if missed:
            failures.append("%s does not reach: %s" % (aggregate, " ".join(missed)))
        notes.append("%s reaches %d of %d suite stages"
                     % (aggregate, len(suite) - len(missed), len(suite)))
    return failures, notes


def self_test():
    """The check on a pair it must pass, and a pair it must fail.

    The passing pair carries the branch a target-name comparison would get
    wrong: `boards` is reached through the tool its own target's recipe runs,
    which `make test` also runs, and neither aggregate names `make
    boards-check`. The failing pair drops one stage and must name that stage -
    so the check is known to be able to fail, and to say which one it was.
    """
    ci = ('GROUP_suite="host test boards timing"\n'
          'stage host "make cc" "" make host\n'
          'stage test "make cc" "" make test\n'
          'stage boards "make python3" "" make boards-check\n'
          'stage timing "make cc" "" make timing\n')
    good = ("coverage:\n\t@$(MAKE) host\n\t@$(MAKE) test\n\t@$(MAKE) timing\n"
            "\ntest:\n\t@python3 tools/board_resources_check.py\n"
            "\nboards-check:\n\t@python3 tools/board_resources_check.py\n")
    bad = ("coverage:\n\t@$(MAKE) host\n\t@$(MAKE) test\n"
           "\ntest:\n\t@true\n\nboards-check:\n\t@true\n")

    failures = []
    for label, text, want in (("reaches every stage", good, []),
                              ("misses a stage", bad, ["timing"])):
        found, notes = check(ci, text, aggregates=("coverage",))
        named = " ".join(found)
        ok = bool(found) == bool(want) and all(w in named for w in want)
        print("  self-test: %s - %s: %s"
              % ("ok" if ok else "FAIL", label, found or notes))
        if not ok:
            failures.append(label)
    return failures


def main():
    if "--self-test" in sys.argv:
        failures = self_test()
        print("suite scope check --self-test: %s"
              % ("ok" if not failures else "FAILED"))
        return 1 if failures else 0

    failures, notes = check(CI.read_text(), MAKEFILE.read_text())
    for note in notes:
        print("suite scope: %s" % note)
    for failure in failures:
        print("suite scope: FAIL - %s" % failure)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
