#!/usr/bin/env python3
"""Every command the firmware has, typed at it once.

    make command-check          # the simulator

`docs_check.py` asks the firmware for its command list and looks for each name
in the pages; this *runs* them. They are different questions, and this one is
the bench's: a command that faults, hangs, or answers nothing is a person at a
bench with a console that has stopped talking, and no check in this repository
had ever typed one that no session happens to use.

Each command is typed on its own, with no arguments, and what is checked is that
the prompt comes back. A command that needs arguments answers with its usage - 
which is an answer - and a command that does something (saving, starting a
calibration, rebooting) is left to do it: this is a smoke test, not a state
machine, and the point is the *next* prompt.

What it does not cover: whether the answers are right. That is what the tests
and the sessions are for.
"""

import re
import subprocess
import sys
import time

PROMPT = b"ak> "
# Long enough for the slowest command that answers: a calibration waits for
# samples, and the simulator only advances its clock while the firmware waits.
ANSWER_TIMEOUT_S = 8.0
# A *recorded* fault, which is the one thing an answer must not contain: the
# boot prints "fault: 2 recorded" when there is one and "fault: none recorded"
# when there is not, and "defaults" is a command - so the first draft's
# `b"fault" in answer` flagged three innocent commands and missed nothing.
FAULT_RECORDED = re.compile(rb"fault[s]?:\s+(?!none)\d", re.I)
FAULT_SAID = b"a fault is recorded"

# A second pass: the commands that take an argument have a *body* that only an
# argument reaches - and one of those bodies is `log reset`, which the bench
# checklist types and which no session does. The arguments below are the
# representative ones a person would type, and they are all legal: a smoke test
# that passed illegal arguments would be checking the firmware's refusals, which
# the protocol and parameter tests already check properly.
# Legal invocations a person types, in the order they make sense - and the
# order matters: `mission start` before a waypoint exists is refused, and the
# same command after one is the mission. One command, both of its answers, in
# the order somebody learns them. (A dict could only hold one argument per
# command, which is what this was, and it is why the refusal had never run.)
ARGUMENTS = (
    ("get", "rate_kp_roll"),
    ("set", "rate_kp_roll 0.3"),
    ("calibrate", "vbat 12.6"),
    ("mission", "start"),                     # nothing to fly yet
    ("mission", "add 52.1000000 4.9000000"),
    ("mission", "list"),
    ("mission", "start"),                     # and now there is
    ("mission", "stop"),
    ("log", "reset"),
    ("log", "long"),
    ("log", "flash"),
    ("log", "flash clear"),
    ("home", "clear"),
    # And the list filled to its limit and then refused one more: the fifth
    # waypoint is the one the parameter table's count will not take, which is
    # the only way a *valid* position is refused.
    ("mission", "add 52.2000000 4.9000000"),
    ("mission", "add 52.3000000 4.9000000"),
    ("mission", "add 52.4000000 4.9000000"),
    ("mission", "start"),
    ("mission", "stop"),
)

# And the third pass: the same console, typed at by somebody who is guessing.
# An argument out of range, a word where a number belongs, a name that does not
# exist, an extra argument, a command that is not a command. What is checked is
# the same thing as above - the prompt comes back, and no fault is recorded -
# because a console that hangs or crashes on nonsense is a console that has
# stopped talking, and that is the failure this file exists for. It is also
# where the argument parsing lives, and a parser is exactly where a board dies
# on a number nobody meant: `calibrate accel 9` and `mission add 200 500` are
# two ways a person's finger slips.
#
# Nothing here is destructive: `reboot`, `dfu`, `defaults`, `save` and
# `log reset` are deliberately absent, because a nonsense *argument* to those
# either does nothing or does the thing, and this pass is about the refusals.
NONSENSE = (
    "frobnicate",
    "get",
    "get nosuchparameter",
    "get 12345",
    "set",
    "set nosuchparameter 1",
    "set rate_kp_roll",
    "set rate_kp_roll abc",
    "set rate_kp_roll 99999",
    "set rate_kp_roll 0.3 extra",
    "calibrate what",
    "calibrate accel",
    "calibrate accel 9",
    "calibrate accel -1",
    "calibrate vbat",
    "calibrate vbat abc",
    "calibrate rc extra",
    "mission add",
    "mission add abc def",
    "mission add 200 500",     # a latitude that is not one
    "mission add 52.1 500",    # nor a longitude
    "mission what",
    "mission add 52.5000000 4.9000000",  # the list is full by now
    "log what",
    "log flash what",
    "log long what",
    "home what",
    "output what",
    "output test what",
    "spi extra",
)


def start(sim):
    return subprocess.Popen([sim, "0", "console"], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def read_prompts(process, count, deadline_s):
    """Everything up to the next `count` prompts, or b"" if they never come.

    The count matters: the *first* read is the boot, which ends at the console's
    own prompt, and every read after it is one command's answer and ends at the
    next one. A first draft of this waited for two prompts every time, which is
    a prompt that never arrives - so every command after the first sat out the
    timeout and the whole run looked like a hang.
    """
    collected = b""
    seen = 0
    end = time.time() + deadline_s
    while time.time() < end:
        byte = process.stdout.read(1)
        if not byte:
            return collected
        collected += byte
        if collected.endswith(PROMPT):
            seen += 1
        if seen >= count:
            return collected
    return b""


def command_names(process):
    read_prompts(process, 1, ANSWER_TIMEOUT_S)  # the boot, up to its prompt
    process.stdin.write(b"help\n")
    process.stdin.flush()
    body = read_prompts(process, 1, ANSWER_TIMEOUT_S)
    text = body.decode(errors="replace")
    # Everything *before* the trailing prompt, and only the first word of each
    # line. (docs_check.py splits the *other* way round, because it reads the
    # boot report whose prompt comes first; the answer to `help` ends with
    # one, and a first draft of this parsed the empty side of that split.)
    names = []
    for line in text.splitlines():
        match = re.match(r"^  ([a-z][a-z0-9_]*)\b", line)
        if match and match.group(1) not in names:
            names.append(match.group(1))
    return names


def first_words(answer, typed):
    """The firmware's own first line of an answer, for the transcript.

    The console echoes what was typed and the read can start on the previous
    answer's last newline, so the line a person wants to see is the first one
    that is neither the echo nor the prompt - and a command that answers with a
    long dump (`log flash what` prints the log) still says what it did.
    """
    for line in answer.decode(errors="replace").splitlines():
        text = line.strip()
        if not text or text == typed or text.startswith("ak>"):
            continue
        if text.startswith("alive:"):
            continue
        return text[:60]
    return "(no words)"


def type_line(process, line):
    """Type one line and read the answer, or None if it never came back.

    A recorded fault is a failure wherever it appears, so it is checked here
    rather than at each call: the three passes all mean the same thing by it.
    """
    process.stdin.write(line.encode() + b"\n")
    process.stdin.flush()
    answer = read_prompts(process, 1, ANSWER_TIMEOUT_S)
    if not answer:
        return None
    if FAULT_RECORDED.search(answer) or FAULT_SAID in answer.lower():
        print("command smoke: FAIL %r answered with a fault record" % line)
        return None
    return answer


def main():
    sim = sys.argv[1] if len(sys.argv) > 1 else "build-host/aerialkit-fw-sim"
    process = start(sim)
    failures = []
    typed = 0
    try:
        names = command_names(process)
        if len(names) < 10:
            print("command smoke: the console answered %d commands - is it "
                  "running?" % len(names))
            return 2

        for name in names:
            process.stdin.write(name.encode() + b"\n")
            process.stdin.flush()
            typed += 1
            answer = read_prompts(process, 1, ANSWER_TIMEOUT_S)
            if not answer:
                failures.append(name)
                print("command smoke: FAIL %s did not come back" % name)
            elif FAULT_RECORDED.search(answer) or FAULT_SAID in answer.lower():
                failures.append(name)
                print("command smoke: FAIL %s answered with a fault record"
                      % name)
            else:
                print("  ok       %-14s %s" % (name, first_words(answer, name)),
                      flush=True)

        for name, arguments in ARGUMENTS:
            line = ("%s %s" % (name, arguments)).strip()
            typed += 1
            answer = type_line(process, line)
            if answer is None:
                failures.append(line)
                print("command smoke: FAIL %s did not come back" % line)
            else:
                print("  ok       %-14s %s" % (line, first_words(answer, line)),
                      flush=True)

        # And the nonsense pass: every one of these has to come back with an
        # answer of its own - usually a refusal, and the transcript prints
        # which - rather than hanging or recording a fault.
        for line in NONSENSE:
            typed += 1
            answer = type_line(process, line)
            if answer is None:
                failures.append(line)
                print("command smoke: FAIL %r did not come back" % line)
            else:
                print("  ok       %-26s %s" % (line, first_words(answer, line)),
                      flush=True)
    finally:
        # End the session the way a person does - by hanging up - rather than
        # killing it: the console scenario finishes when its input closes, and
        # it is the difference between a process that wrote its coverage data
        # and one that did not (`make coverage` runs this).
        try:
            process.stdin.close()
        except OSError:
            pass
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    print("command smoke: %d commands typed, %d did not answer"
          % (typed, len(failures)), flush=True)
    print("command smoke: %s" % ("FAIL" if failures else "PASS"), flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
