#!/usr/bin/env bash
#
# check-configurator.sh - the web configurator's own suite, run in its own directory.
#
#   scripts/check-configurator.sh                                   # the sibling app
#   CONFIGURATOR=/path/to/apps/configurator scripts/check-configurator.sh
#
# Why this exists: `apps/configurator` is self-contained by design - its own
# package.json, its own node_modules, its own vitest, its own README - and until
# this script was written, *nothing in this repository ran it*. The two gates
# whose names sound like this one are not it. `make window-test` drives the
# Python window against a fake aircraft and `make proto-test` drives the protocol
# client against the firmware over a pipe: both are the firmware's side of the
# pair, and neither one loads a line of the app's TypeScript. So the app's suite
# - the MAVLink family, the dialect table checked against ArduPilot's, the
# session and UI behaviour, 186 tests over 12 files - was run only by hand, by
# whoever remembered, which is the same shape as the stack check before it became
# a stage (trap 153) and as the records before they had a checker.
#
# Measured when it was wired in, 2026-09-20, on this machine:
#
#   npm test          exit 0, 186 passed, 10 skipped, 12 files, 5.6 s
#   npm run typecheck exit 0
#
# The 10 skips are `tests/sitl.test.ts`, which needs a live SITL and says so.
# They are not a failure and they are not hidden: the count is printed below.
#
# WHAT THIS DOES NOT ASSERT, and why. It does not assert that 186 tests passed.
# A number written into a gate goes stale exactly the way the number in
# `apps/configurator/docs/BUILD-AND-DEPLOY.md` did - that document said 171
# across 9 files, and its own paragraph admits the figure had already been wrong
# twice for the same reason ("it was 121 here after the betaflight and interface
# work landed, and 123 after the first MAVLink"). A third correction belongs
# there, not a fourth copy of the same mistake here. What it asserts instead is
# the invariant that a count cannot express: **the number of test files vitest
# reports equals the number of test files on disk**. A file that exists but is
# collected by nothing - a renamed glob, a config that stops matching, a
# `.test.tsx` the include pattern forgot - is the failure a green suite would
# otherwise hide, and it is the one this project has already hit twice in other
# instruments (traps 150 and 153). The counts are printed so that movement in
# them is visible in a log rather than only in a pass.
#
# Exit contract: 0 the suite ran and passed and the file counts agree; 1
# everything else, including the case where the app or its dependencies are not
# there at all. The *stage* treats a machine without node, npm or an installed
# app as "not run" through its needs list, so a fresh checkout reports the stage
# as not run rather than failing the pipeline; this script refusing is what
# happens when somebody asks for the check directly and it cannot be made.
set -u

here="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
app="${CONFIGURATOR:-$(cd "$here/../.." && pwd)/apps/configurator}"

say() { printf '%s\n' "$*"; }
die() { say "configurator: $*"; exit 1; }

[ -d "$app" ] || die "cannot run - no app at $app (set CONFIGURATOR= to point at it)"
[ -f "$app/package.json" ] || die "cannot run - $app has no package.json"
[ -x "$app/node_modules/.bin/vitest" ] || \
    die "cannot run - nothing installed in $app/node_modules (cd $app && npm ci)"
command -v node >/dev/null 2>&1 || die "cannot run - no node on this machine"
command -v npm  >/dev/null 2>&1 || die "cannot run - no npm on this machine"

# The app's own scripts, not a re-implementation of them: `npm test` is what its
# README and BUILD-AND-DEPLOY.md tell a person to run, and a gate that runs
# something else is a gate for a different program.
#
# Colour is off so the log greps and diffs like the rest of the pipeline's.
run_in_app() {
    ( cd "$app" && NO_COLOR=1 FORCE_COLOR=0 npm "$@" 2>&1 )
}

say "configurator: $app"
say "configurator: node $(node --version), npm $(npm --version)"
say ""

test_out="$(run_in_app test --silent)"; rc=$?
if [ "$rc" -ne 0 ]; then
    say "$test_out"
    die "npm test failed (exit $rc)"
fi
# vitest writes its summary with ANSI when it can; NO_COLOR is set above and the
# escapes are stripped here as well, because a summary line that only greps
# without them is a summary line that silently matches nothing.
plain="$(printf '%s\n' "$test_out" | sed 's/\x1b\[[0-9;]*m//g')"
files_line="$(printf '%s\n' "$plain" | grep -E '^ *Test Files ' | tail -1)"
tests_line="$(printf '%s\n' "$plain" | grep -E '^ *Tests '      | tail -1)"
[ -n "$files_line" ] || { say "$plain"; die "npm test printed no 'Test Files' summary"; }
[ -n "$tests_line" ] || { say "$plain"; die "npm test printed no 'Tests' summary"; }

say "  files  $files_line"
say "  tests  $tests_line"

# no `bc`: awk is a shell tool that is always there, bc is not
reported=$(printf '%s\n' "$files_line" | grep -oE '[0-9]+ (passed|failed|skipped)' | grep -oE '^[0-9]+' | awk '{s+=$1} END{print s+0}')
# What "on disk" means here, and why it is wider than the config's own glob. The
# app's `vite.config.ts` includes `tests/**/*.test.{ts,tsx}`. This counts every
# file under the app whose name looks like a test in the usual conventions -
# `.test.` and `.spec.`, ts and tsx - anywhere in the tree, node_modules and any
# build output excluded. So a test written next to its source in `src/` instead
# of in `tests/` counts here and is collected by nothing: that is not a false
# positive, that is the file nobody's suite is running, which is the whole
# reason this count exists. A config that narrows `include` while the files stay
# where they were is the other half of the same failure, and it lands here too.
on_disk=$(find "$app" \
              \( -name node_modules -o -name dist -o -name build-cov -o -name .git \) -prune -o \
              \( -name '*.test.ts' -o -name '*.test.tsx' \
                 -o -name '*.spec.ts' -o -name '*.spec.tsx' \) -print 2>/dev/null | wc -l)
say ""
say "  test files on disk:           $on_disk"
say "  test files vitest reported:   $reported"
if [ "$on_disk" -ne "$reported" ]; then
    say ""
    say "configurator: FAILED - $on_disk test file(s) in the app are named like"
    say "  tests, and vitest collected $reported. A file nothing collects is a"
    say "  suite that is green about less than it says it is. Check the include"
    say "  pattern in vite.config.ts and the file names."
    exit 1
fi

tsc_out="$(run_in_app run typecheck --silent)"; rc=$?
if [ "$rc" -ne 0 ]; then
    say "$tsc_out"
    die "npm run typecheck failed (exit $rc)"
fi
say ""
say "  typecheck ok (tsc --noEmit, no output)"
say ""
say "configurator: ok - $reported of $on_disk file(s) collected, all passed or skipped as documented"
