#!/usr/bin/env bash
#
# The checks that fail when the app and the firmware stop agreeing.
#
# Every one of these exists because the two halves drifted and nothing noticed.
# They are run together because they are the same failure seen from three sides:
# a claim about the firmware that the firmware no longer supports, a table the
# board has outgrown, and a message definition that disagrees with the dialect
# ArduPilot and PX4 actually publish.
#
# Run from `apps/configurator`:  npm run verify:drift
#
# Exit code:
#   0  every check ran and passed
#   1  a check failed, or a check could not run
#
# A check that cannot run is a failure and not a skip. `check-dialect.py` needs
# pymavlink, which is not installed on this laptop by default, so this script is
# red for that reason alone unless the oracle is present. Reporting it as
# "passed" would be the exact class of lie the rest of this directory exists to
# prevent — so it is red, and `check-dialect.py` prints the two commands that
# fix it (pinned to a version, for the reason stated in that file):
#
#   python3 -m venv /tmp/pmv && /tmp/pmv/bin/pip install pymavlink==2.4.49
#   PATH=/tmp/pmv/bin:$PATH npm run verify:drift
#
# Measured with the oracle present, 2026-09-30: all three checks pass, and each
# was falsified individually — a wrong group in the fixture, a wrong field type
# in `dialect.json`, and a `check-dialect.py` run without `--check` (which
# rewrote its own expectation and could not fail at all until that day; the
# paragraph above the dialect step records it).

set -uo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
status=0

run() {
  local label="$1"; shift
  printf '\n== %s\n' "$label"
  if ! "$@"; then
    status=1
  fi
}

run "source citations the app makes about the firmware" \
  python3 "$here/check-citations.py"

run "the demo board's fixture table against ak_flight.c" \
  python3 "$here/check-table-drift.py"

# `--check`, not a bare run. Without the flag this script *rewrites*
# `tests/fixtures/dialect.json` from the installed pymavlink and returns 0
# whatever was there before — a step that regenerates its own expectation and
# therefore cannot fail, in a file whose whole purpose is to be the thing that
# fails. It was written that way and stayed that way until 2026-09-30, which is
# worth stating: the first two checks in this script bit when they were broken
# and this one did not, and nothing about a green run said which was which.
run "this app's MAVLink definitions against the published dialect" \
  python3 "$here/check-dialect.py" --check

printf '\n'
if [ "$status" -eq 0 ]; then
  echo "verify-drift: all checks passed"
else
  echo "verify-drift: at least one check failed or could not run (see above)"
fi
exit "$status"
