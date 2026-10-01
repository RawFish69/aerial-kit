#!/usr/bin/env python3
"""Two questions, both of them "does the document still say what the thing does".

The first is the console's: every command the firmware has is written down
somewhere. The firmware's command table and the documentation are two lists of
the same thing, and they drift: this project has twice found a page describing a
feature as missing after it landed, and a document that lies is worse than no
document because the next person trusts it. The check is deliberately
one-directional and cheap - ask the *firmware* what commands it has (its own
`help`, over the simulator's console) and look for each one in the documentation
- because the other direction needs a parser and this needs a substring. It runs
against the simulator rather than the source so that what it checks is the
command table the aircraft actually has, not an `if` chain somebody remembers to
keep in step.

The second is the build page's, and it exists because the first kind of check
was pointed at a *file* and read PASS. `docs/28-build.md` describes
`scripts/ci.sh`, and it makes the same claim in two places: a table of stages,
and one sentence saying what `--only suite` runs. A check that read the table
and nothing else reported 17 of 17 ok on a tree whose sentence still named the
two stages the group held before either oracle existed - the table was right,
the sentence was stale in all three trees, and the check was *named* for the
question. So the claims here are written per place the page answers, and each
one fails when it cannot find the claim at all rather than passing by finding
nothing. A page that stops saying `--only suite` is a page whose prose changed,
and either way it has to be re-read.

Both halves are one-directional for the same reason, and neither is a
spell-checker: what is checked is the part of each document that is a claim
about another file.
"""

import pathlib
import re
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent

CI = ROOT / "scripts" / "ci.sh"
BUILD_PAGE = ROOT / "docs" / "28-build.md"
PROTO_PAGE = ROOT / "docs" / "16-protocol.md"
PROTO_HEADER = ROOT / "src" / "core" / "ak_proto.h"
PROTO_SOURCE = ROOT / "src" / "core" / "ak_proto.c"

STAGE_HEADER = "| Stage | Command | Needs | What it adds over the one before |"

NUMBER_WORDS = ("zero one two three four five six seven eight nine ten "
                "eleven twelve").split()


def console_help(sim):
    """The command names, from the firmware's own `help`."""
    process = subprocess.Popen([sim, "0", "console"], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT)
    output = b""

    try:
        process.stdin.write(b"help\n")
        process.stdin.flush()
        # The prompt is the only thing the firmware prints that means "the
        # command has finished" - see docs/06-console.md.
        while output.count(b"ak> ") < 2 and len(output) < 65536:
            chunk = process.stdout.read(1)
            if not chunk:
                break
            output += chunk
    finally:
        process.kill()
        process.wait()

    text = output.decode(errors="replace")
    # Only what came back *after* the first prompt: the boot report's selftest
    # prints lines that look exactly like a command list ("  ok       ..."), and
    # the first version of this check counted "ok" as a command the firmware
    # has. And only the *first* word of each line, because the help prints the
    # command with its arguments - "calibrate accel <0-5>" is one command - and
    # a wrapped description's continuation lines start with a digit or a capital
    # letter, which is what keeps them out.
    answered = text.split("ak> ", 1)
    body = answered[1] if len(answered) > 1 else ""
    names = set()
    for line in body.splitlines():
        match = re.match(r"^  ([a-z][a-z0-9_]*)\b", line)
        if match:
            names.add(match.group(1))
    return names


def stage_list(ci_path=None, page_path=None):
    """Does the build page still describe the ci.sh it sits beside?

        (fail, lines) - fail is 0 or 1, lines are for printing.

    Six claims, and the count is the point: the page asserts its stage list in
    two places, so a check that covers one of them is narrower than the claim it
    is named for. The table is a claim about membership *and* order - the
    paragraph above it says "runs these in this order" - so an inserted row in
    the wrong place describes the wrong stage as the thing the one before it
    adds to, and passes every membership claim.

    The denominator is asserted rather than assumed: `stage` directives and
    `STAGE_NAMES` are two declarations of one list in the same file, and if they
    ever disagree then "every stage ci.sh declares" is not a question with an
    answer.
    """
    ci_path = pathlib.Path(ci_path or CI)
    page_path = pathlib.Path(page_path or BUILD_PAGE)
    ci = ci_path.read_text(errors="replace")
    doc = page_path.read_text(errors="replace")

    lines = []
    state = {"fail": 0}

    def report(ok, label, detail=""):
        if not ok:
            state["fail"] = 1
        lines.append("  %-6s %s%s" % ("ok" if ok else "FAIL", label,
                                      ("  " + detail) if detail else ""))

    def stop(label):
        # A page or script that has changed shape is not a partial answer, it is
        # a question that no longer parses. Say so and stop, rather than
        # reporting the claims below it as passing because nothing was read.
        state["fail"] = 1
        lines.append("  FAIL   %s" % label)
        return state["fail"], lines

    directives = re.findall(r"^stage ([a-z][a-z-]*) ", ci, re.M)
    match = re.search(r'^STAGE_NAMES="([^"]*)"', ci, re.M)
    if not match or not directives:
        return stop("ci.sh does not declare its stages where this reads them")
    names = match.group(1).split()
    report(directives == names, "stage directives == STAGE_NAMES",
           "(%d vs %d)" % (len(directives), len(names)))
    stages = names

    if doc.count(STAGE_HEADER) != 1:
        return stop("the build page's stage table header is not where it was")
    table = doc.split(STAGE_HEADER, 1)[1].split("\n## ", 1)[0]
    rows = re.findall(r"^\| `([a-z][a-z-]*)` \|", table, re.M)

    missing = [s for s in stages if s not in rows]
    extra = [r for r in rows if r not in stages]
    report(not missing, "every stage ci.sh declares has a row",
           "%d rows / %d stages" % (len(rows), len(stages))
           + ("  undocumented=%s" % missing if missing else ""))
    report(not extra, "no row names a stage ci.sh has not got",
           "rows-for-nothing=%s" % extra if extra else "")
    report(rows == stages, "the table is in the order the script runs them",
           "%s vs %s" % (rows, stages) if rows != stages else "")

    # The paragraph introducing the table is where the groups are described.
    # Scoped to it, and each clause cut at its own terminator: the first version
    # searched the whole page and cut at the next semicolon, but the
    # `--only protocol` clause ends at a full stop, so the cut landed three
    # sentences later and the count collected every number word in between. It
    # reported FAIL on all three trees, which looked like three findings and was
    # one instrument (04-traps.md §106, §107).
    intro = [b for b in doc[:doc.index(STAGE_HEADER)].split("\n\n") if b.strip()][-1]
    for flag in ("suite", "protocol"):
        if "`--only %s`" % flag not in intro:
            return stop("the paragraph introducing the stage table no longer "
                        "describes `--only %s`" % flag)

    def clause(flag):
        """The text between `--only <flag>` and the end of its own clause."""
        start = intro.index("`--only %s`" % flag)
        ends = [i for i in (intro.find(";", start), intro.find(".", start))
                if i > start]
        if not ends:
            return None
        return intro[start:min(ends)]

    suite = clause("suite")
    named = re.findall(r"`([a-z][a-z-]*)`", suite) if suite else []
    group = re.search(r'^GROUP_suite="((?:[^"]|\n)*?)"', ci, re.M)
    if not group:
        return stop("ci.sh does not declare GROUP_suite where this reads it")
    want = group.group(1).split()
    report(named == want, "the --only suite sentence names GROUP_suite",
           "%s vs %s" % (named, want))

    proto = clause("protocol")
    words = [w for w in re.findall(r"[a-z]+", proto or "")
             if w in NUMBER_WORDS]
    gproto = re.search(r'^GROUP_protocol="((?:[^"]|\n)*?)"', ci, re.M)
    if not gproto:
        return stop("ci.sh does not declare GROUP_protocol where this reads it")
    count = len(gproto.group(1).split())
    report(len(words) == 1 and NUMBER_WORDS.index(words[0]) == count,
           "the --only protocol clause counts GROUP_protocol",
           "%s vs %d stages" % (words, count))

    return state["fail"], lines


# One opcode is also a frame the board pushes without being asked, so it has a
# `case` and no row in the request table: `0x08`, documented under `##
# Telemetry` as the same body as STATUS sent unsolicited. Named here with its
# citation rather than skipped quietly - an exemption with a reason is a
# decision, and an exemption without one is a hole the next opcode falls
# through.
PUSHED_ONLY = {
    "AK_PROTO_CMD_TELEMETRY": "16-protocol.md, `## Telemetry`",
}

COMMAND_TABLE_HEADER = "| | Command | Payload | Reply |"


def opcode_table(header_path=None, source_path=None, page_path=None):
    """Does the protocol page list the commands the firmware dispatches?

        (fail, lines)

    Three lists of one thing, and they are written by three different hands:
    the numbered rows in `docs/16-protocol.md`, the enum in `ak_proto.h`, and
    the `case` labels in `ak_proto.c`'s dispatch. The console half of this file
    can ask a firmware what it does because it has a `help`; the protocol has no
    such thing - a board answers an opcode it does not know with `0x7F` and no
    list of the ones it does - so the source is what this reads.

    The claim that earns its place is the third one: a command with a `case` and
    no row is a command nobody can find out about, and a row with no `case` is
    documentation for something that was never built. Both have happened in
    other projects' protocols; neither is visible from inside one of the three
    lists, which is why the check reads all of them and compares.
    """
    lines = []
    state = {"fail": 0}

    def report(ok, label, detail=""):
        if not ok:
            state["fail"] = 1
        lines.append("  %-6s %s%s" % ("ok" if ok else "FAIL", label,
                                      ("  " + detail) if detail else ""))

    def stop(label):
        state["fail"] = 1
        lines.append("  FAIL   %s" % label)
        return state["fail"], lines

    header = pathlib.Path(header_path or PROTO_HEADER).read_text(errors="replace")
    source = pathlib.Path(source_path or PROTO_SOURCE).read_text(errors="replace")
    doc = pathlib.Path(page_path or PROTO_PAGE).read_text(errors="replace")

    # `name -> number`, from the enum with its explicit values. An enum member
    # added without one would take the next value silently, and the wire number
    # is the whole of what a client agrees with - so it is not read as an
    # implied one here either.
    defined = {name: int(value, 16) for name, value in
               re.findall(r"(AK_PROTO_CMD_[A-Z0-9_]+)\s*=\s*0x([0-9A-Fa-f]+)",
                          header)}
    if len(defined) < 8:
        return stop("ak_proto.h does not declare its opcodes where this reads "
                    "them (%d found)" % len(defined))

    if doc.count(COMMAND_TABLE_HEADER) != 1:
        return stop("16-protocol.md's command table header is not where it was")
    table = doc.split(COMMAND_TABLE_HEADER, 1)[1].split("\n\n", 1)[0]
    rows = re.findall(r"^\| (0x[0-9A-Fa-f]{2}) \| ([a-z][a-z0-9 ]*) \|",
                      table, re.M)
    if len(rows) < 8:
        return stop("16-protocol.md's command table does not parse (%d rows)"
                    % len(rows))

    cases = re.findall(r"case (AK_PROTO_CMD_[A-Z0-9_]+):", source)
    if len(cases) < 8:
        return stop("ak_proto.c's dispatch does not parse (%d cases)"
                    % len(cases))

    def wire_name(enum_name):
        """`AK_PROTO_CMD_PARAM_INFO` is the row `param info`."""
        return enum_name[len("AK_PROTO_CMD_"):].lower().replace("_", " ")

    by_number = {}
    duplicates = []
    for spelling, name in rows:
        number = int(spelling, 16)
        if number in by_number:
            duplicates.append(spelling)
        by_number[number] = name
    report(not duplicates, "no opcode is listed twice",
           "twice=%s" % duplicates if duplicates else "")

    # The number and the name are two fields of one claim, so they are checked
    # together: a table that renamed a row without renumbering it describes a
    # different command than the one a client would send.
    wrong = []
    for enum_name, number in sorted(defined.items(), key=lambda pair: pair[1]):
        spelled = by_number.get(number)
        if spelled is None:
            if enum_name not in PUSHED_ONLY:
                wrong.append("0x%02X %s has no row" % (number, wire_name(enum_name)))
            continue
        if spelled != wire_name(enum_name):
            wrong.append("0x%02X is `%s` in the table and `%s` in ak_proto.h"
                         % (number, spelled, wire_name(enum_name)))
    report(not wrong, "every opcode the firmware defines is a row saying the "
           "same thing", "; ".join(wrong))

    unknown = [name for name in cases if name not in defined]
    report(not unknown, "every case names an opcode the header defines",
           "undefined=%s" % unknown if unknown else "")

    # The other direction of the same claim, and it is not implied by the one
    # above: a row for a number no opcode defines is documentation for a command
    # that does not exist, and nothing else here looks at a row on its own.
    stray = ["0x%02X %s" % (number, name)
             for number, name in sorted(by_number.items())
             if number not in set(defined.values())]
    report(not stray, "every row names an opcode the header defines",
           "rows-for-nothing=%s" % stray if stray else "")

    undispatched = sorted(name for name, number in defined.items()
                          if name not in cases and number in by_number)
    report(not undispatched, "every opcode with a row has a case in the "
           "dispatch", "documented-but-not-built=%s" % undispatched
           if undispatched else "")

    # A case is accounted for when its number is in the table, or when it is the
    # pushed frame named above.
    unaccounted = [name for name in cases
                   if defined.get(name) not in by_number
                   and name not in PUSHED_ONLY]
    report(not unaccounted, "every case is either a row or a named push",
           "built-but-undocumented=%s"
           % [wire_name(n) for n in unaccounted] if unaccounted else "")

    # Counted only over the cases that are defined, because a case the header
    # does not define has no number to look up - and this line ran before the
    # claim above existed, so an undefined case used to be a KeyError here
    # rather than the finding it is. Its own self-test found that.
    covered = len([n for n in cases if n in defined and defined[n] in by_number])
    lines.append("  ----   %d opcodes: %d asked for and documented, %d pushed "
                 "(%s)" % (len(defined), covered, len(PUSHED_ONLY),
                           ", ".join(sorted(PUSHED_ONLY))))
    return state["fail"], lines


def main():
    # The simulator is **required**, and it used to default to a relative
    # `build-host/aerialkit-fw-sim`. That default is only ever reached by a hand
    # run - `make docs-check` always passes `$(HOST_OUT)/aerialkit-fw-sim` - and
    # a hand run is exactly the case where the stale copy bites: this file's
    # job is to ask a *firmware* for its command list, and a `build-host/` left
    # over from an earlier day answers with the command list of an earlier day.
    # Measured 2026-09-30: the default path held a simulator from 2026-09-29
    # 02:18 while the fresh build was from 00:15 that morning, and the two
    # disagreed by one command (26 vs 27) - a difference small enough to read as
    # noise, with exit 0 either way. So the argument is no longer optional; a
    # run that cannot name what it is checking stops instead of guessing.
    if len(sys.argv) <= 1:
        print("docs check: name the simulator to check, e.g. "
              "`python3 tools/docs_check.py $(HOST_OUT)/aerialkit-fw-sim` - "
              "this check reads the firmware's own command list, so which "
              "binary it reads is the whole of what it measures")
        return 2

    sim = sys.argv[1]
    if not pathlib.Path(sim).exists():
        print("docs check: no simulator at %s - run `make host` first" % sim)
        return 2

    commands = console_help(sim)
    if len(commands) < 10:
        print("docs check: the console answered %d commands - is it running?"
              % len(commands))
        return 2

    pages = list((ROOT / "docs").glob("*.md")) + [ROOT / "README.md"]
    text = "\n".join(page.read_text(errors="replace") for page in pages)

    missing = sorted(name for name in commands
                     if not re.search(r"\b%s\b" % re.escape(name), text))

    status = 0
    if missing:
        # The list, when there is one, is the whole point of the check.
        print("the console's commands, against the documentation")
        for name in sorted(commands):
            print("  %-14s %s" % (name, "missing" if name in missing else "ok"))
        print("undocumented: %s" % ", ".join(missing))
        status = 1
    else:
        print("%d console commands on the firmware, all of them written down "
              "(%d pages read)" % (len(commands), len(pages)))

    # And the page that documents the script that runs all of this, which is the
    # one document here whose claims are checkable exactly.
    print("\n%s, against %s" % (BUILD_PAGE.name, CI.name))
    fail, lines = stage_list()
    for line in lines:
        print(line)
    if fail:
        print("the build page does not describe the script beside it")
        status = 1
    else:
        print("the build page matches the script")

    # And the three lists of the protocol's opcodes, which are written in three
    # files by three different hands and have no way to notice each other.
    print("\n%s, against %s and %s"
          % (PROTO_PAGE.name, PROTO_HEADER.name, PROTO_SOURCE.name))
    fail, lines = opcode_table()
    for line in lines:
        print(line)
    if fail:
        print("the protocol page does not describe the firmware's dispatch")
        status = 1
    else:
        print("the protocol page, the header and the dispatch agree")
    return status


SELF_TEST_CONSOLE = """#!/bin/sh
# A console with one command nothing documents. Written by --self-test, so that
# the check's own failure mode is something this repository has seen rather than
# something its author believes in.
printf "AerialKit\\r\\nak> "
read _line
printf "\\r\\ncommands:\\r\\n"
printf "  frobnicate   does nothing at all\\r\\n"
printf "  help         this\\r\\n"
printf "  version      firmware\\r\\n"
printf "  status       state\\r\\n"
printf "  params       every parameter\\r\\n"
printf "  get <name>   one parameter\\r\\n"
printf "  set          change one\\r\\n"
printf "  defaults     built-in values\\r\\n"
printf "  save         write them\\r\\n"
printf "  load         read them\\r\\n"
printf "  clear        wipe the screen\\r\\n"
printf "\\r\\nak> "
sleep 0.2
"""


def self_test():
    """The check, against a console that has an undocumented command."""
    with tempfile.TemporaryDirectory() as tmp:
        fake = pathlib.Path(tmp) / "fakesim"
        fake.write_text(SELF_TEST_CONSOLE)
        fake.chmod(0o755)
        status = subprocess.run([sys.executable, str(pathlib.Path(__file__)),
                                 str(fake)], capture_output=True, text=True)

    # And against no simulator named at all. The argument used to be optional
    # and defaulted to a relative `build-host/` path, which is exactly what a
    # hand run picks up - and a hand run is when a day-old binary is most
    # likely to be sitting there. Measured 2026-09-30: that default held a
    # simulator from the previous day, it answered with one command fewer, and
    # it exited 0 either way. A check that cannot say which binary it read is
    # not evidence, so this asserts it now refuses instead of guessing.
    bare = subprocess.run([sys.executable, str(pathlib.Path(__file__))],
                          capture_output=True, text=True)

    print("the documentation check, against a console that has an undocumented "
          "command")
    for line in status.stdout.splitlines():
        print("  %s" % line)
    failed = 0
    if status.returncode == 1 and "frobnicate" in status.stdout:
        print("  ok       it fails, and names the command nothing writes down")
    else:
        print("  FAILED   it did not fail on a command no page mentions "
              "(exit %d)" % status.returncode)
        failed = 1

    print("and against no simulator named at all")
    if bare.returncode == 2 and "name the simulator" in bare.stdout:
        print("  ok       it refuses to guess which binary to read (exit 2)")
    else:
        print("  FAILED   it ran without being told which simulator to read "
              "(exit %d)" % bare.returncode)
        failed = 1
    return failed


def stage_self_test():
    """The stage half, against the two defects it was written for.

    The positive control comes first and is the one that matters: pointed at the
    repository's own pair of files it has to say ok. Without it a FAIL says only
    that two files disagree, not which of the two is wrong - and the first
    version of this check reported FAIL on all three trees because the
    instrument cut a clause at the wrong terminator, which read as three wrong
    pages (04-traps.md §107).

    Then the fixture, which is those same two files with the real defect put
    back: both oracle rows deleted from the table, and the `--only suite`
    sentence reverted to the two stages the group held before either oracle
    existed. That is not a hypothetical shape - it is what the landing tree, the
    records tree and the port all said on 2026-09-19, while a check named for
    this question read PASS because it parsed the table and nothing else.
    """
    print("the build page, against the ci.sh beside it")
    fail, lines = stage_list()
    for line in lines:
        print("  %s" % line)
    if fail:
        print("  FAILED   the repository's own two files do not agree, so this "
              "test cannot tell a planted defect from a real one")
        return 1
    print("  ok       the repository's own page and script agree")

    with tempfile.TemporaryDirectory() as tmp:
        tree = pathlib.Path(tmp) / "aerialkit"
        (tree / "docs").mkdir(parents=True)
        (tree / "scripts").mkdir()
        ci = tree / "scripts" / "ci.sh"
        page = tree / "docs" / "28-build.md"
        ci.write_text(CI.read_text(errors="replace"))

        text = BUILD_PAGE.read_text(errors="replace")
        removed = []
        for stage in ("estimator-oracle", "attitude-kinematics"):
            row = re.search(r"^\| `%s` \|[^\n]*\n" % stage, text, re.M)
            if not row:
                print("  FAILED   the %s row is already gone from the table, "
                      "so this test is not planting what it thinks" % stage)
                return 1
            text = text[:row.start()] + text[row.end():]
            removed.append(stage)
        stale = re.sub(r"`--only suite` runs[^;]*;",
                       "`--only suite` runs `host` and `test`;", text)
        if stale == text:
            print("  FAILED   the --only suite sentence is not the shape this "
                  "test rewrites")
            return 1
        page.write_text(stale)

        # Inside the `with`: the first version called this after the block, so
        # the fixture was already deleted and the check raised
        # FileNotFoundError - a check that cannot pass, which is worse than one
        # that cannot fail, because it gets deleted rather than read.
        fail, lines = stage_list(ci, page)
        for line in lines:
            print("  %s" % line)
        named = "".join(lines)
        if not fail:
            print("  FAILED   it passed a table missing %s and a sentence "
                  "naming two stages" % " and ".join(removed))
            return 1
        if not all(stage in named for stage in removed):
            print("  FAILED   it failed, but did not name %s"
                  % " and ".join(removed))
            return 1
    print("  ok       it fails, and names the rows the table is missing")
    return 0


def opcode_self_test():
    """The opcode half, against the two defects it was written for.

    The positive control first, for the same reason the stage half has one: a
    FAIL on the repository's own three files has to mean one of them is wrong,
    and not that the instrument reads them wrong.

    Then the two defects, each planted on its own so that the failure names
    which list is the odd one out. They are the two shapes this can actually go
    wrong in, and neither is visible from inside one file: an opcode the
    firmware dispatches with no row (nobody can find out it exists), and a row
    with no `case` (documentation for something that was never built).
    """
    print("the protocol page, against the header and the dispatch beside it")
    fail, lines = opcode_table()
    for line in lines:
        print("  %s" % line)
    if fail:
        print("  FAILED   the repository's own three lists do not agree, so "
              "this test cannot tell a planted defect from a real one")
        return 1
    print("  ok       the repository's own page, header and dispatch agree")

    header = PROTO_HEADER.read_text(errors="replace")
    source = PROTO_SOURCE.read_text(errors="replace")
    page = PROTO_PAGE.read_text(errors="replace")

    def planted(fault, expect, page_text, source_text):
        with tempfile.TemporaryDirectory() as tmp:
            tree = pathlib.Path(tmp)
            (tree / "ak_proto.h").write_text(header)
            (tree / "ak_proto.c").write_text(source_text)
            (tree / "16-protocol.md").write_text(page_text)
            fail, lines = opcode_table(tree / "ak_proto.h", tree / "ak_proto.c",
                                       tree / "16-protocol.md")
        named = "".join(lines)
        if not fail:
            print("  FAILED   it passed %s" % fault)
            return 1
        if expect not in named:
            print("  FAILED   it failed, but did not name %s" % fault)
            print(named)
            return 1
        print("  ok       it fails, and names %s" % fault)
        return 0

    row = re.search(r"^\| 0x0B \| param help \|[^\n]*\n", page, re.M)
    if not row:
        print("  FAILED   the param help row is already gone, so this test is "
              "not planting what it thinks")
        return 1
    failed = planted("an opcode with a case and no row", "param help has no row",
                     page[:row.start()] + page[row.end():], source)

    # One line changed rather than a block cut out: the regex that would find
    # the end of a `case` is the fragile part of a fixture, and the case body
    # has braces of its own at every depth.
    if "case AK_PROTO_CMD_PARAM_INFO: {" not in source:
        print("  FAILED   the param info case is not the shape this test edits")
        return 1
    failed |= planted("a row with no case",
                      "AK_PROTO_CMD_PARAM_INFO",
                      page,
                      source.replace("case AK_PROTO_CMD_PARAM_INFO: {",
                                     "case AK_PROTO_CMD_PARAM_NOPE: {", 1))
    return failed


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--self-test":
        failed = self_test() | stage_self_test() | opcode_self_test()
        sys.exit(1 if failed else 0)
    sys.exit(main())
