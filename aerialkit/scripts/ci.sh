#!/usr/bin/env bash
#
# ci.sh - one command that runs everything this host can run, and says what it
# could not.
#
#   scripts/ci.sh                      # everything
#   scripts/ci.sh --only test          # one stage
#   scripts/ci.sh --only profiles      # one stage, by the name --list prints
#   scripts/ci.sh --list               # what the stages are
#   scripts/ci.sh --out /tmp/report
#
# This is the entry point a pull request runs (see .github/workflows/aerialkit.yml)
# and the one a person runs before saying a change is finished. It exists because
# "the tests pass" had stopped meaning anything in particular, and that is a
# measurable complaint rather than a rhetorical one:
#
#   make -n test | grep -oE 'tools/[a-z0-9_]+\.py|aerialkit-[a-z0-9-]+' | sort -u
#
# prints six host binaries and seven Python checks, and it does not reach
# `proto-test`, either window client, `firmware-proto-check`, `sanitize`,
# `coverage` or the five ARM profiles. So a green `make test` meant whichever of
# the rest the person happened to remember, and the ones nobody remembered were
# the ones that had never been run anywhere - which is how finding 4 in
# docs/28-build.md survived in this repository.
#
# **Stages, not a script.** Each stage is one command, run in order, logged to
# its own file, and stops the run at the first failure - except the ones that
# are allowed to fail, which are named as such below and whose failure is
# recorded rather than swallowed. There are four ways a stage can end and they
# are kept apart on purpose: **ok**, **not run** (a tool is missing, or the
# check's own log says it did not check here), **failed, allowed** (it failed in
# the one way that is written down), and **failed**. The report never says a
# stage passed when it did not run.
#
# **The manifest is written first, before anything is built.** A run that fails
# in the first stage still leaves behind the revision, the toolchain and the
# source hash it failed on, which is the difference between a failure somebody
# can reproduce and a failure somebody has to guess at.
#
# **The cross toolchain is optional and its absence is not a failure of the
# tests.** Everything in the host and protocol stages is compiled by the host
# compiler; only the last stage needs `arm-none-eabi-`. A build host without it
# runs the whole of the rest and reports the profiles as not measured, rather
# than reporting a green run that measured nothing.
#
# See docs/28-build.md for the packages, the pinned versions and the container.

set -euo pipefail

here="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
root="$(cd "$here/.." && pwd)"
cd "$root"

OUT="ci-report"
CROSS="${CROSS:-arm-none-eabi-}"

# Where the web configurator lives. It is a sibling of this subtree rather than
# a part of it - its own package.json, its own node_modules, its own README - so
# a checkout that keeps it elsewhere points this at it, and the last stage
# measures the same path this names. Exported, because the `make` target under
# that stage reads the variable and a needs list that checked one path while the
# target ran another would be a gate for a different program (trap 158's shape).
export CONFIGURATOR="${CONFIGURATOR:-$(cd "$root/.." && pwd)/apps/configurator}"
ONLY=""
declare -a STAGES_RUN=() STAGES_FAILED=() STAGES_ALLOWED=() STAGES_SKIPPED=()
declare -a STAGES_NOTRUN=()

# The stage names, once, and the groups a person can ask for by name.
#
# `--only` validates against these before anything runs, because the alternative
# is what the first version did: `--only protcol` matched no stage, every stage
# was recorded as skipped, nothing ran, and the script said **ok**. A typo that
# produces a green report of a run that never happened is the worst failure this
# file can have, so an unknown name is now an error and not a selection of zero.
STAGE_NAMES="host test estimator-oracle attitude-kinematics config-policy
             config-recycle timing boards contract control proto-test window-test net-window-test
             firmware-proto sanitize coverage profiles stack configurator"
GROUP_protocol="proto-test window-test net-window-test firmware-proto"
GROUP_suite="host test estimator-oracle attitude-kinematics config-policy config-recycle timing boards contract control"

list_stages() {
    printf '%s\n' \
        "manifest   the revision, toolchain and source hashes - always first" \
        "host       the C host build" \
        "test       the whole host suite: checks, sessions, fuzzer, Python" \
        "estimator-oracle  the estimator against an independent quaternion truth" \
        "attitude-kinematics  the plant's attitude step against the same kind of truth" \
        "config-policy    every route into the configuration, and who may persist" \
        "config-recycle   the configuration ring through a power cut in its erase" \
        "timing           what the control loop does with an unexpected interval" \
        "boards           the board manifests against each other, pin by pin" \
        "contract         the cross-repository numbers, against a third implementation" \
        "control          the control core through the ABI a Python simulation uses" \
        "proto-test       the protocol client against the firmware, over a pipe" \
        "window-test      the configurator window against a fake aircraft" \
        "net-window-test  the same window over a socket" \
        "firmware-proto   the firmware-side half of the client pair" \
        "sanitize   the suite under asan/ubsan, run to the end" \
        "coverage   the suite under --coverage, and the map it leaves" \
        "profiles   every ARM profile built, measured and reported" \
        "stack      how much stack the image needs, from GCC's call graph" \
        "configurator  the web app's own suite, in the app's own directory"
    printf '%s\n' \
        "" \
        "--only takes one of those, or one of these groups:" \
        "  suite      = host test estimator-oracle attitude-kinematics" \
        "               config-policy config-recycle timing boards contract control" \
        "  protocol   = proto-test window-test net-window-test firmware-proto" \
        "" \
        "profiles and stack need the ARM cross toolchain; configurator needs" \
        "node, npm and an installed apps/configurator/node_modules. On a host" \
        "without them each stage is recorded as not run rather than as passing." \
        "docs/28-build.md says what to install."
}

# Is this stage wanted? The ONLY value is known to be valid by the time this is
# called - see the check after the argument loop.
selected() {
    local name="$1"
    [ -z "$ONLY" ] && return 0
    [ "$ONLY" = "$name" ] && return 0
    # Two statements, not one `local`: bash expands every word of a `local` line
    # before it assigns any of them, so `${!group-}` on the same line reads a
    # variable that does not exist yet and aborts with "invalid indirect
    # expansion".
    local group="GROUP_$ONLY"
    local members="${!group-}"
    case " $members " in
        *" $name "*) return 0 ;;
    esac
    return 1
}

while [ $# -gt 0 ]; do
    case "$1" in
        --out)    OUT="$2"; shift 2 ;;
        --only)   ONLY="$2"; shift 2 ;;
        --cross)  CROSS="$2"; shift 2 ;;
        --list)   list_stages; exit 0 ;;
        -h|--help) list_stages; exit 0 ;;
        *) echo "ci.sh: unknown argument $1" >&2; exit 2 ;;
    esac
done

# The name is checked *here*, before a single stage is considered, and an
# unknown one is an error. Selecting zero stages and reporting ok is not a
# hypothetical: it is what `--only protocol` did, because the group name was in
# the help text and not in the matching, and `--only protcol` would have done
# the same. See the note on STAGE_NAMES above.
if [ -n "$ONLY" ]; then
    known=0
    for candidate in $STAGE_NAMES; do
        [ "$ONLY" = "$candidate" ] && known=1
    done
    for candidate in suite protocol; do
        [ "$ONLY" = "$candidate" ] && known=1
    done
    if [ "$known" != 1 ]; then
        echo "ci.sh: --only $ONLY selects no stage, so there is nothing to run." >&2
        echo "" >&2
        list_stages >&2
        exit 2
    fi
fi

CROSS_BIN="${CROSS}gcc"
mkdir -p "$OUT"

say() { printf '%s\n' "$*"; }
rule() { printf '%s\n' "------------------------------------------------------------"; }

# --- the manifest -----------------------------------------------------------
#
# Three hashes, because they answer three different questions. The commit is
# what to check out. The tree hash is what git thinks the content is. The
# content hash is over the bytes this run actually compiled, in a fixed order,
# so it differs when a file is edited and not committed - which is the state a
# person is in when they run this before committing, and the state in which
# "which revision did that number come from" is otherwise unanswerable.

content_hash() {
    # `git ls-files`, not `find`: the index is the set of files a checkout of
    # this commit will have, and `find` also answers with build state - which
    # made the number unreproducible outside this working tree (trap 151).
    git ls-files -- src ports linker tests tools scripts 2>/dev/null |
        LC_ALL=C sort | xargs -r sha256sum 2>/dev/null | sha256sum | cut -d' ' -f1
}

tool_version() {
    if command -v "$1" >/dev/null 2>&1; then
        "$@" 2>&1 | head -1
    else
        echo "not present"
    fi
}

write_manifest() {
    local file="$OUT/manifest.txt"
    {
        echo "aerialkit CI manifest"
        echo "written:        $(date -u +'%Y-%m-%dT%H:%M:%SZ')"
        echo "host:           $(uname -srm)"
        echo "run from:       $root"
        rule
        echo "git commit:     $(git rev-parse HEAD 2>/dev/null || echo 'not a git repository')"
        echo "git describe:   $(git describe --always --dirty 2>/dev/null || echo unknown)"
        echo "git branch:     $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
        echo "git tree:       $(git rev-parse 'HEAD^{tree}' 2>/dev/null || echo unknown)"
        if [ -n "$(git status --porcelain 2>/dev/null)" ]; then
            echo "working tree:   dirty - the hashes above describe the commit, the"
            echo "                content hash below describes what was compiled"
        else
            echo "working tree:   clean"
        fi
        echo "source sha256:  $(content_hash)"
        rule
        echo "host cc:        $(tool_version cc --version)"
        echo "python3:        $(tool_version python3 --version)"
        echo "cross gcc:      $(tool_version "${CROSS_BIN}" --version)"
        echo "cross prefix:   $CROSS"
        if [ -n "${IDF_PATH:-}" ]; then
            echo "esp-idf:        $IDF_PATH"
        elif [ -f "$HOME/esp-idf/version.txt" ]; then
            echo "esp-idf:        $HOME/esp-idf ($(cat "$HOME/esp-idf/version.txt"))"
        else
            echo "esp-idf:        not present - the ESP32 profiles are not built here"
        fi
        rule
        echo "selection:      ${ONLY:-all stages}"
    } > "$file"
    say "manifest written to $file"
}

# --- stages -----------------------------------------------------------------
#
# stage NAME NEEDS ALLOWANCE COMMAND...
#
# Three separate ways for a stage not to be green, kept apart because collapsing
# any two of them is how a report starts lying:
#
#   * **NOT RUN** - a command in NEEDS (a space-separated list) is not on this
#     machine, so the stage was never invoked. Recorded as not run, with the
#     command that was missing, and it does not affect the exit code: a machine
#     without a cross toolchain is not a machine with a failing build. Unless
#     `--only` named it, which is a request that cannot be met.
#   * **FAILED, allowed** - it ran and failed, and the failure is the one
#     written down in ALLOWANCE. ALLOWANCE is "PATTERN|SENTENCE": the failure is
#     only excused if the log actually contains PATTERN. **This matters more than
#     it looks.** The first version of this excused a stage unconditionally, on
#     the theory that the reason was known - and then swallowed a completely
#     unrelated bug in a different stage, which reported itself as "no ARM cross
#     toolchain" on a machine that had one, and the run exited zero. An allowance
#     that does not check *what* it is excusing is a blanket.
#   * **FAILED** - anything else. Stops the run.
#
# An allowed failure does not fail the run - a run that is always red is a run
# nobody reads, which is how the stages got skipped in the first place. It is
# printed where it happens, counted in the summary, and named in the last line,
# so "ok" and "ok, with one thing nobody has fixed" cannot be confused. When it
# is asked for by name - `ci.sh --only sanitize` - it is the whole point of the
# run and its failure *is* the exit code; there is nothing else it could mean.
stage() {
    local name="$1"; shift
    local needs="$1"; shift
    local allowance="$1"; shift

    # --only matches this stage or a group it belongs to: `--only protocol` is
    # all four client checks, which are one idea and four invocations.
    local wanted=0
    if [ -n "$ONLY" ]; then
        if selected "$name"; then
            wanted=1
        else
            STAGES_SKIPPED+=("$name")
            return 0
        fi
    fi

    local missing="" need
    for need in $needs; do
        command -v "$need" >/dev/null 2>&1 || missing="$missing $need"
    done
    if [ -n "$missing" ]; then
        say ""
        rule
        say "== $name"
        rule
        if [ "$wanted" = 1 ]; then
            say "$name: cannot run - no${missing} on this machine."
            say "  you asked for this one by name, so this is a failure rather"
            say "  than a skip. docs/28-build.md says what provides${missing}."
            STAGES_FAILED+=("$name: no${missing}")
            finish
            exit 1
        fi
        say "$name: NOT RUN - no${missing} on this machine"
        STAGES_NOTRUN+=("$name: no${missing}")
        return 0
    fi

    local log="$OUT/$name.log"
    say ""
    rule
    say "== $name"
    rule
    local started=$SECONDS
    if "$@" > "$log" 2>&1; then
        # **Exiting zero is not the same as having checked anything.** Every
        # tool here skips with a line and a zero when its precondition is
        # missing - "the window over a network link: not checked here - no
        # qemu-system-xtensa" - and the first version of this script called that
        # `ok in 0s`, which is the exact failure mode this file exists to
        # prevent. A stage counted as ok when it printed no `ok` of its own and
        # said it did not check here is recorded as NOT RUN instead.
        #
        # Both conditions are needed. `make test` legitimately contains a check
        # that skips a section of itself (tools/reference_defaults_check.py
        # without an INAV checkout), and it prints two thousand `ok` lines
        # around it, so the stage ran; those skips are reported as a count
        # rather than as a not-run stage.
        local oks skips why
        oks=$(grep -cE '^[[:space:]]*ok[[:space:]]|PASS' "$log" || true)
        skips=$(grep -c 'not checked here' "$log" || true)
        # `grep -c` prints 0 and exits 1 for no matches, so the `|| true` above
        # is for the exit status and these two are for the other way it can come
        # back empty: an empty string compares as an integer error, and under
        # `set -e` that is a stage that dies rather than one that is judged.
        oks=${oks:-0}
        skips=${skips:-0}
        if [ "$skips" -gt 0 ] && [ "$oks" -eq 0 ]; then
            why=$(grep -m1 'not checked here' "$log" | sed 's/^[[:space:]]*//')
            say "$name: NOT RUN - the check skipped itself: $why"
            STAGES_NOTRUN+=("$name: $why")
            return 0
        fi
        say "$name: ok in $((SECONDS - started))s  (log: $log)"
        if [ "$skips" -gt 0 ]; then
            say "  note: $skips line(s) in this log say \"not checked here\" -"
            say "  part of this stage did not run on this machine:"
            grep -m5 'not checked here' "$log" | sed 's/^[[:space:]]*/    /'
        fi
        STAGES_RUN+=("$name")
        return 0
    fi
    say "$name: FAILED after $((SECONDS - started))s  (log: $log)"
    tail -30 "$log" | sed 's/^/    /'
    STAGES_RUN+=("$name")

    if [ -n "$allowance" ] && [ "$wanted" != 1 ]; then
        local pattern="${allowance%%|*}" reason="${allowance#*|}"
        if grep -qF -- "$pattern" "$log"; then
            say "$name: allowed to fail - $reason"
            STAGES_ALLOWED+=("$name: $reason")
            return 0
        fi
        say "$name: FAILED, and not in the way that is allowed."
        say "  the allowance is for a log containing: $pattern"
        say "  this log does not. Treating it as a real failure."
    fi

    STAGES_FAILED+=("$name")
    finish
    exit 1
}

# --- the profiles -----------------------------------------------------------
#
# The matrix of images this firmware ships, in one place. `targets/*/target.conf`
# in the workspace repo is the canonical description of how each of these is
# built for the NAS builder; this is the same matrix for a host that has only the
# ARM toolchain, and the two have to agree. The names are the artifact names, so
# a line of the report can be found in `dist/` without a translation.
#
# The flags are not cosmetic. `-DAK_BOARD_BARO_FITTED` and
# `-DAK_BOARD_VBAT_FITTED` are what a board with the barometer and the pack
# divider soldered on compiles; `-DAK_BOOT_STAGE` is the tell-tale image, which
# is the one a person flashes when the console never comes up. They are three
# different images because they are three different answers to "what is on this
# board", and a report that measured only the bare one would be describing a
# board nobody has.
#
# The ESP32 profiles are not here: their build system is ESP-IDF's, not this
# Makefile's, and their images are measured by `idf.py size`. scripts/esp32-proto.sh
# is what builds and boots them, and it needs IDF_PATH.

PROFILES=(
    "aerialkit-f405|AERIALKIT_F405|stm32f405|stm32f405rg|"
    "aerialkit-f405-fitted|AERIALKIT_F405|stm32f405|stm32f405rg|-DAK_BOARD_BARO_FITTED=1 -DAK_BOARD_VBAT_FITTED=1"
    "aerialkit-f405-telltale|AERIALKIT_F405|stm32f405|stm32f405rg|-DAK_BOOT_STAGE=1 -DAK_BOARD_BARO_FITTED=1 -DAK_BOARD_VBAT_FITTED=1"
    "aerialkit-ghf435|AERIALKIT_GHF435|at32f435|at32f435rg|"
    "aerialkit-ghf435-telltale|AERIALKIT_GHF435|at32f435|at32f435rg|-DAK_BOOT_STAGE=1"
)

stage_profiles() {
    # The toolchain's presence is checked by `stage` before this is called, from
    # the NEEDS list, so that a machine without it reports the stage as not run
    # rather than as a failure - and so that unbuilt profiles can never be
    # reported as zeroes.
    #
    # `mkdir -p` is not decoration: without it the redirect on the first build's
    # log fails, the loop body never runs, and the stage dies one second in with
    # "No such file or directory". It did exactly that, and the first version of
    # this script reported the corpse as "not measured - no ARM cross toolchain"
    # on a machine that had the toolchain on its PATH.
    mkdir -p "$OUT/build"

    local entries=() line name board arch part flags out
    for line in "${PROFILES[@]}"; do
        IFS='|' read -r name board arch part flags <<< "$line"
        out="$OUT/build/$name"
        rm -rf "$out"
        echo "== $name   BOARD=$board ARCH=$arch PART=$part"
        echo "   EXTRA_CFLAGS=${flags:-（none）}"
        # shellcheck disable=SC2086 # $flags is a list of flags, on purpose
        if ! make BOARD="$board" ARCH="$arch" PART="$part" PRODUCT="$name" \
                OUT="$out" EXTRA_CFLAGS="$flags" all check > "$out.build.log" 2>&1; then
            echo "FAILED: $name did not build, or did not pass its image check"
            tail -25 "$out.build.log" | sed 's/^/    /'
            return 1
        fi
        # The linker's own account of the image, kept beside it. The report is
        # compared against these lines and not against itself - it is the only
        # independent statement of the same numbers that exists.
        sed -n '/Memory region/,/^$/p' "$out.build.log" > "$out/linker-says.txt"
        entries+=("$name=$out/$name.elf")
    done

    echo ""
    echo "== the artifacts, by hash"
    # What was measured, identified by content rather than by name. The image a
    # report describes is the image whose hash is here; two builds of the same
    # revision are two images until this says otherwise, which is the mistake
    # this workspace has already made once (a NAS build and a local build).
    : > "$OUT/artifacts.txt"
    for line in "${entries[@]}"; do
        out="${line#*=}"
        # shellcheck disable=SC2086
        if [ -f "$out" ]; then
            printf '%s  %10d  %s\n' "$(sha256sum "$out" | cut -d' ' -f1)" \
                "$(stat -c%s "$out")" "${out#$OUT/build/}" >> "$OUT/artifacts.txt"
        fi
        for extra in "${out%.elf}.bin" "${out%.elf}.hex" "${out%.elf}.map"; do
            if [ -f "$extra" ]; then
                printf '%s  %10d  %s\n' "$(sha256sum "$extra" | cut -d' ' -f1)" \
                    "$(stat -c%s "$extra")" "${extra#$OUT/build/}" >> "$OUT/artifacts.txt"
            fi
        done
    done
    cat "$OUT/artifacts.txt"

    echo ""
    echo "== the report"
    # One invocation, both files: --out and --json are independent, and calling
    # the tool twice to get two views of the same measurement is two chances for
    # the two views to describe different runs.
    python3 tools/image_report.py --readelf "${CROSS}readelf" --nm "${CROSS}nm" \
        --out "$OUT/profiles.txt" --json "$OUT/profiles.json" "${entries[@]}"
    cat "$OUT/profiles.txt"

    echo "== and the report against the linker, profile by profile"
    python3 - "$OUT/profiles.json" "$OUT/build" <<'PYTHON'
import json, os, re, sys

# The report derives its numbers from the ELF's program headers instead of
# reading the linker's line off a build log, because a report has to work on an
# image somebody was handed. That derivation is checked here, against the link
# that produced each image: every region, byte for byte.
REGION = re.compile(r"^\s*(\w+):\s+(\d+) B\s")
report = json.load(open(sys.argv[1]))["profiles"]
bad = 0
for profile in report:
    said = {}
    path = os.path.join(sys.argv[2], profile["name"], "linker-says.txt")
    for line in open(path):
        found = REGION.match(line)
        if found:
            said[found.group(1)] = int(found.group(2))
    if not said:
        print("  FAILED  %s: the link printed no memory usage to compare with"
              % profile["name"])
        bad += 1
        continue
    for region in profile["regions"]:
        want = said.get(region["name"])
        if want is None:
            continue
        if want != region["used"]:
            print("  FAILED  %s %s: report says %d, the linker said %d"
                  % (profile["name"], region["name"], region["used"], want))
            bad += 1
        else:
            print("  ok      %s %s: %d B, as the linker printed"
                  % (profile["name"], region["name"], want))
print("the report against the linker: %s" % ("ok" if bad == 0 else "FAILED"))
sys.exit(1 if bad else 0)
PYTHON
}

# --- the counts -------------------------------------------------------------
#
# The numbers the report is *for*, read out of the logs rather than retyped, so
# a run cannot report a count it did not produce.
#
# That sentence was the intent and not the behaviour until this was added: the
# gates below were `[ -f "$OUT/sanitize.log" ]`, and $OUT is not cleared between
# runs, so the file is there whether or not *this* run wrote it. A `--only suite`
# run printed the previous run's coverage percentage and the previous run's
# sanitizer failure under a heading that reads "what this run produced, read out
# of its own logs", while the `skipped:` line three inches below correctly named
# both stages as skipped. Two statements in one report, contradicting each other,
# with the wrong one written as if it were measured. That is the same class of
# lie as `--only protcol` reporting a green run of nothing, which is the failure
# this file's own header says it exists to prevent - so the logs are gated on the
# record of what ran rather than on their own existence.
#
# STAGES_RUN is the right record and not STAGES_SKIPPED or STAGES_NOTRUN: it
# holds every stage that actually executed, including one that failed within its
# allowance, which is the case where the log is most worth reading. A stage that
# skipped itself ("not checked here" with no `ok` of its own) leaves a log too,
# and is in STAGES_NOTRUN, so existence alone would report that as well.
ran() {
    local want="$1" s
    for s in "${STAGES_RUN[@]:-}"; do
        [ "$s" = "$want" ] && return 0
    done
    return 1
}

# The reason a stage was excused, if it was. STAGES_ALLOWED holds
# "<name>: <reason>", and the counts block below has to say so rather than
# printing a bare FAILED: the summary already carries an `allowed:` line, so
# without this the same report says "sanitizers: FAILED" in the numbers and
# "sanitize: allowed to fail" in the ledger, three inches apart. That is the
# contradiction `ran()` was just added to remove, one level down, and it matters
# more here than there: counts.txt is the file a reader opens on its own, and
# the case that prompted it was a stage that failed *every* run - the sanitizer,
# whose bare FAILED was the normal case wearing the clothes of the alarming one.
#
# No stage is excused today. `sanitize` was the only one, and its allowance was
# retired on 2026-09-20 when the crash it excused was fixed at its cause (see
# the SAN_ENV block in the Makefile and finding 1 in docs/28-build.md). The
# mechanism stays because "an allowance is a pattern, not a licence" is a
# property of this script rather than of that stage - but the `excused` branch
# below is unexercised now, so a report that reaches it is saying something
# that has not happened since that date.
allowed_reason() {
    local want="$1" entry
    for entry in "${STAGES_ALLOWED[@]:-}"; do
        case "$entry" in
            "$want":\ *) printf '%s' "${entry#"$want": }"; return 0 ;;
        esac
    done
    return 1
}

stage_counts() {
    local file="$OUT/counts.txt" reason
    {
        echo "what this run produced, read out of its own logs"
        rule
        if ran test && [ -f "$OUT/test.log" ]; then
            grep -E '^[0-9]+ checks, [0-9]+ failed$' "$OUT/test.log" |
                sed 's/^/host checks:    /' | tail -1
            printf 'sessions:       %s passed, %s failed\n' \
                "$(grep -c '^sim: PASS' "$OUT/test.log" || true)" \
                "$(grep -c '^sim: FAIL' "$OUT/test.log" || true)"
            grep -E '^fuzz: ' "$OUT/test.log" | tail -1 | sed 's/^/fuzzer:         /'
        else
            echo "host checks:    not run this time"
        fi
        if [ -f "$OUT/boards.log" ]; then
            grep -E '^board_resources_check: ' "$OUT/boards.log" | tail -1 |
                sed 's/^board_resources_check: /boards:         /' || true
            grep -E '^board_resources: ' "$OUT/boards.log" | tail -1 |
                sed 's/^board_resources: /                /' || true
        else
            echo "boards:         not run"
        fi
        if [ -f "$OUT/contract.log" ]; then
            grep -E '^contract_check: ' "$OUT/contract.log" | tail -1 |
                sed 's/^contract_check: /contract:       /' || true
        else
            echo "contract:       not run"
        fi
        if ran coverage && [ -f "$OUT/coverage.log" ]; then
            grep -E '^coverage: ' "$OUT/coverage.log" | tail -1 |
                sed 's/^/coverage:       /'
        else
            echo "coverage:       not run this time"
        fi
        if ran sanitize && [ -f "$OUT/sanitize.log" ]; then
            if grep -q 'the suite under the sanitizers, green' "$OUT/sanitize.log"; then
                echo "sanitizers:     green"
            elif reason=$(allowed_reason sanitize); then
                echo "sanitizers:     FAILED, and excused - $reason"
            else
                echo "sanitizers:     FAILED - see sanitize.log and docs/28-build.md"
            fi
        else
            echo "sanitizers:     not run this time"
        fi
        rule
        if ran profiles && [ -f "$OUT/profiles.txt" ]; then
            sed -n '/^profile/,$p' "$OUT/profiles.txt"
        else
            # Not "no ARM cross toolchain": that is a guess about why, and the
            # first version of this file made it wrongly for a stage that died of
            # a missing directory. The reason is one of the lines above.
            echo "profiles:       not measured - see the profiles stage above"
        fi
    } > "$file"
    say ""
    cat "$file"
}

finish() {
    stage_counts
    local file="$OUT/summary.txt"
    {
        echo "aerialkit CI: $([ ${#STAGES_FAILED[@]} -eq 0 ] && echo ok || echo FAILED)"
        rule
        printf 'ran:      %s\n' "${STAGES_RUN[*]:-none}"
        printf 'skipped:  %s\n' "${STAGES_SKIPPED[*]:-none}"
        printf 'not run:  %s\n' "${STAGES_NOTRUN[*]:-none}"
        printf 'failed:   %s\n' "${STAGES_FAILED[*]:-none}"
        printf 'allowed:  %s\n' "${STAGES_ALLOWED[*]:-none}"
    } > "$file"
    say ""
    cat "$file"
    say ""
    say "everything above is in $OUT/ - manifest.txt is the revision and toolchain,"
    say "counts.txt is the numbers, profiles.txt is what each image costs, and"
    say "artifacts.txt is the sha256 of every image those numbers came from."
}

# --- the run ----------------------------------------------------------------

write_manifest

stage host      "make cc"                  ""       make host
stage test      "make cc python3"          ""       make test
# B2's acceptance test, and the reason it is a stage of its own rather than part
# of `make test`. It measures the estimator against a quaternion truth that
# shares none of the estimator's equations - which is the whole point, because
# `make test`'s 2,047 checks and 42 sessions all passed while the estimator was
# integrating body rates as Euler derivatives, and the simulator was making the
# same mistake so the two agreed. A file that shares the assumption it is
# testing measures nothing; this one does not, so it is worth a stage.
stage estimator-oracle "make cc" "" make estimator-oracle
# The same kind of stage for the *plant*, one layer down. The estimator oracle
# above checks the thing that reads the aircraft; this one checks the thing the
# aircraft is simulated by, which every session in `make test` is measured
# against. Both defects were the same mistake - body rates integrated as if
# they were Euler derivatives - and in both cases the suite agreed with itself
# because the plant and the estimator were wrong the same way. So the plant
# gets the same treatment: truth that shares none of its equations, an
# integration-order check that separates "converging" from "converging to a
# slightly wrong answer", and a control showing the old arithmetic fails.
stage attitude-kinematics "make cc" "" make attitude-kinematics
# The four client halves. `make test` drives the Python client against the
# simulator, but these four are the turns of the screw it does not: the protocol
# over a pipe, the two window clients, and the firmware-side half of the pair.
# B3's acceptance test, and a stage for the estimator oracle's reason. It
# measured the console applying a parameter while the wire did not, and both
# links persisting the board's flash with the motors live - three failures that
# `make test` could not see, because the console, the protocol and the flight
# core were each behaving as written and only the *pair* was wrong. It held
# those three as a baseline while B3 was unwritten and it PASSES now, which is
# what makes it a gate rather than a record: a route that stops asking the
# policy fails here instead of on an aircraft.
stage config-policy     "make cc"          ""       make config-policy
# B3.3's acceptance test, and a stage for the same reason again. This one is
# about the *part* rather than the routes: the F405 erases 128 KB at once, so a
# full ring has to erase its whole sector, and until the ring alternated between
# two banks a power cut inside that erase left the aircraft reading as a board
# nobody had ever saved to - which `preflight` reports as "none stored" and
# counts no problem. `make test` passed throughout, because every read and every
# write was behaving as written and the fault was in what the erase destroyed.
stage config-recycle    "make cc"          ""       make config-recycle
stage timing            "make cc"          ""       make timing
# B5's acceptance test, and the first stage here that needs no compiler at all.
# Every board header is a set of claims about hardware - this pin is the
# receiver, that one is the sensor's chip select - and nothing read two of them
# together, so a header could give one pad to two functions and every check in
# this repository passed. Two do: `ESP32S2DEV` puts the status LED and the first
# servo on GPIO 15 *in the configuration a bare devkit builds*, and `ESP32C3DEV`
# puts the receiver on the sensor's chip select once a sensor is fitted - the
# assessment's F6. The five are held as known so that a new one fails here; the
# stage runs the validator against the tree and against fixtures whose answers
# were decided before they were written, because a reader is exactly the kind of
# thing that agrees with itself.
stage boards            "make python3"      ""       make boards-check
stage contract          "make python3"      ""       make contract-check
# The control core through the ABI a Python simulation drives it by. It is a
# stage of its own rather than a line in `make test` alone because it is the
# only check here that crosses a language boundary, and the failure it is
# looking for - a struct whose layout the compiler and ctypes disagree about -
# has no compiler error and no crash, only numbers that look like numbers. It
# builds the shared library from src/core/flight and src/core/sensors, which
# are the sources the MCU image is built from.
# The experiment rides in the same stage for its syntax only. It cannot run
# here - it needs numpy and the sibling aerial-kit checkout - so it prints "not
# in this checkout" and exits zero, which means CI would not notice a syntax
# error in it until the one person who can run it tried to. Compiling it costs
# nothing and closes that gap; `make control-experiment REPO=...` is the run.
stage control           "make cc python3"   ""       make control-check python-syntax

stage proto-test        "make cc python3"  ""       make proto-test
stage window-test       "make cc python3"  ""       make window-test
stage net-window-test   "make python3"     ""       make net-window-test
stage firmware-proto    "make cc python3"  ""       make firmware-proto-check
# No longer allowed to fail, and the allowance it had is retired rather than
# left in place unused.
#
# This stage was excused by a pattern for one crash, and fixing that crash is
# what showed there had been a second failure behind it that nothing had ever
# reached - which is the argument for not leaving an allowance in place.
#
# The first: the host port test maps a modelled register block at 0xE000E000,
# which is inside AddressSanitizer's shadow gap on x86-64, so the memory check
# could not look up shadow for the address and reported a crash against a page
# the test owns. What was wrong was the word "cannot": `protect_shadow_gap=0` is
# ASan's own option for exactly this case, and `make sanitize` now sets it (the
# SAN_ENV block in the Makefile).
#
# The second: with that fixed the suite ran to the end, and the stage then failed
# at the very next thing it does - `tools/akcontrol_check.py` loads the control
# library with ctypes, and an interpreter that is not itself instrumented cannot
# load an instrumented shared object ("ASan runtime does not come first in
# initial library list"). The Makefile's CONTROL_LIB_ENV block preloads the
# runtime for that one invocation, and `tools/lsan.supp` keeps LeakSanitizer from
# reporting CPython's own shutdown allocations as if they were the library's.
#
# The number that made the first one worth fixing rather than excusing is the
# position of the abort: check 230 of 2118. The stage was not failing at the end
# of the suite, it was failing near the beginning, so 1888 checks - the mixer,
# the flight core, the estimators, the rest of everything - had never once been
# through the sanitizers. "Allowed to fail" recorded the failure and not how far
# it got, and the distance is what said how much was not being checked.
#
# The allowance mechanism stays, because "an allowance is a pattern, not a
# licence" is a property of this script and not of this stage; sanitize simply
# has no pattern now, so a failure of any kind here is a real failure.
stage sanitize  "make cc python3" "" \
                make sanitize
stage coverage  "make cc python3 gcov"     ""       make coverage
# The ARM toolchain is three programs, not one: image_report.py reads the images
# with readelf and nm, so a compiler alone would build five profiles and measure
# none of them. A host without all three records this stage as NOT RUN and still
# exits zero - a machine without a cross compiler is not a failing machine.
stage profiles  "${CROSS}gcc ${CROSS}readelf ${CROSS}nm" "" stage_profiles
# The stack requirement of the image, from the compiler's own call graph - and
# the second stage here that measures a built image rather than running a host
# program. `make test` gives the host build eight megabytes of stack; the F405
# has about forty-nine kilobytes free after its blackbox rings. So "does it fit
# at runtime" is a question the suite cannot be asked, an overflow is invisible
# on this machine, and on the bench it presents as "the board stopped saying
# anything" - which is the failure this stage exists to find first, and which is
# why `scripts/check-stack.sh` and `tools/stack_report.py` were written at all.
#
# It is a stage and not only a `make` target because an image-level gate that
# nobody runs is a gate that is green by not being asked. The Makefile says this
# about the sibling check - `image_report_check.py` "belongs here rather than in
# a target somebody has to remember" - and `profiles` needs the same three ARM
# programs and *is* a stage, with the missing-tool case already handled as a
# recorded not-run rather than a pass. What was left was the number itself:
# `make ci` could end "ok - every stage ran here and passed" on an image whose
# pessimistic reading exceeded the part's RAM, and the first person to see that
# number would be the one holding the board.
#
# Two numbers come out and the stage judges the pessimistic one: the deepest
# chain the call graph resolves, and the same chain with one largest-frame call
# charged at every site the analysis cannot follow (516 indirect calls, on this
# image). At the moment this was wired in that is 2,472 bytes resolved and
# **8,584 of the 12,288-byte limit** - so there is 3,704 bytes of headroom to
# the reading that fails, and the limit is `STACK_LIMIT`, a Makefile variable,
# rather than a number in this file. It was watched failing before it was
# trusted: the same stage with the ceiling moved below the reading exits 1 and
# the report says which number crossed it.
#
# `make stack-check` builds one image, and `profiles` builds five. That looked
# like the gap worth closing here - the fitted image adds a barometer and a pack
# divider, the tell-tale adds the boot stages, and the GHF435 is a different
# part - so all four of the others were measured the same way before this stage
# was written, with the callgraph flag and `EXTRA_CFLAGS` set as the matrix sets
# them:
#
#     profile                    functions   deepest chain   pessimistic
#     aerialkit-f405 (this one)        423         2,472         8,584
#     aerialkit-f405-fitted            426         2,472         8,584
#     aerialkit-f405-telltale          428         2,472         8,584
#     aerialkit-ghf435                 433         2,472         8,584
#
# The reading does not move, and the *function counts* are what say the flag
# reached the compiler and these are four different images - three different
# ELFs, 758,816 / 760,300 / 762,256 / 761,884 bytes. The deepest chain is the
# same five functions in every one of them (Reset_Handler, ak_firmware_main,
# preflight_run, arm_line, ak_flight_arm_check) and the largest frame is the
# same 2,120-byte `preflight_run`, so a variant adds functions that join neither.
# That is why one image is enough for this stage, and it is a measurement rather
# than an assumption: the first reading of the other four is this table.
stage stack     "${CROSS}gcc ${CROSS}objcopy ${CROSS}size" "" make stack-check

# The web configurator's own suite, and the first stage here that runs a program
# outside this subtree. `apps/configurator` is self-contained by design, and
# until this stage existed *nothing in this repository ran its tests*. The two
# gates whose names sound like it are not it: `window-test` drives the Python
# window against a fake aircraft and `proto-test` drives the protocol client
# against the firmware over a pipe, and both are the firmware's side of the
# pair. Neither loads a line of the app's TypeScript, so the MAVLink family, the
# dialect table checked against ArduPilot's and the session behaviour - 186
# tests over 12 files, 5.6 s, plus `tsc --noEmit` - were run only by whoever
# remembered, which is the same shape as the stack check before it became a
# stage and the records before they had a checker.
#
# **Last, and that is a choice rather than a leftover.** It shares nothing with
# the eighteen stages before it - a different language, no C compiler, no ARM
# toolchain, no host build - so its position is arbitrary, and appending it
# keeps the order of those eighteen exactly as every record of a pipeline run
# quotes them.
#
# **A machine without node, npm or an installed app is not a failing machine.**
# That is why `apps/configurator/node_modules/.bin/vitest` is in the needs list
# as a path and not as an assumption: node_modules is gitignored, so on a fresh
# checkout this stage reports NOT RUN and names the path it lacks, the way
# `profiles` and `stack` name the ARM programs they are missing. `make
# configurator-check` run by hand refuses instead, and says `npm ci`.
stage configurator "node npm $CONFIGURATOR/node_modules/.bin/vitest" "" \
      make configurator-check

finish

if [ ${#STAGES_FAILED[@]} -ne 0 ]; then
    say ""
    say "aerialkit CI: FAILED - ${STAGES_FAILED[*]}"
    exit 1
fi
# The last line is the one a person reads, so it names every qualification. A
# run that ends "ok" having measured five of nine stages is a run that lied by
# omission, and that is precisely how the profiles stage went unmeasured for as
# long as it did.
say ""
if [ ${#STAGES_ALLOWED[@]} -ne 0 ] || [ ${#STAGES_NOTRUN[@]} -ne 0 ]; then
    say "aerialkit CI: ok, with ${#STAGES_ALLOWED[@]} allowed failure(s) and"
    say "${#STAGES_NOTRUN[@]} stage(s) not run on this machine."
    if [ ${#STAGES_ALLOWED[@]} -ne 0 ]; then
        say "  allowed: ${STAGES_ALLOWED[*]}"
    fi
    if [ ${#STAGES_NOTRUN[@]} -ne 0 ]; then
        say "  not run: ${STAGES_NOTRUN[*]}"
    fi
    say "  ran:     ${STAGES_RUN[*]:-none}"
    exit 0
fi
say "aerialkit CI: ok - every stage ran here and passed."
