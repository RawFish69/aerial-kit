#!/usr/bin/env python3
"""The numbers we say are the references', read back out of the references.

    python3 tools/reference_defaults_check.py [--sim PATH]      # make test runs this

Three claims sit in the pages, and all three are checkable:

| our parameter | the claim | where |
| --- | --- | --- |
| `launch_throttle` = 0.70 | INAV's `nav_fw_launch_thr`, 1700 of its 1000..2000 range | `docs/24-launch.md` |
| `launch_climb_deg` = 18 | INAV's `nav_fw_launch_climb_angle` | `docs/24-launch.md` |
| `launch_timeout_s` = 5 | INAV's `nav_fw_launch_timeout`, milliseconds there | `docs/24-launch.md` |
| `arm_max_tilt_deg` = 25 | the arming gate's angle: Betaflight's `DEFAULT_SMALL_ANGLE` and INAV's `small_angle` | `docs/03-attribution.md`, `docs/04-flight-core.md` |

A claim about somebody else's firmware is the easiest kind to get wrong: a
number remembered from a configurator, a range assumed rather than read, a
setting that was renamed between releases. So nothing here is remembered. Each
value is read from the reference itself:

* **INAV's own settings table** (`upstream/inav-9.1.0/src/main/fc/settings.yaml`)
  carries the default and the `min`/`max` of every setting, which is what makes
  the throttle conversion arithmetic rather than a convention: the fraction is
  `(value - min) / (max - min)`.
* **the aircraft's own dump** (`projects/twin-wings/ghf435-inav/`'s
  `inav-postflash-default-config.txt`, a CLI `dump` taken from this board's
  controller with INAV 9.1.0 on it) says what the board actually holds - which
  is a second, independent check on the first, because a default that changed
  between releases would show up here.
* **Betaflight's header** (`src/main/flight/imu.c`'s `DEFAULT_SMALL_ANGLE`) is
  the same 25 degrees from the other reference the attribution names.
* and **our side** is the firmware's parameter table, dumped by the simulator,
  which is the same table every target builds.

What this deliberately is **not**: a comparison of loop gains. INAV's
fixed-wing PIDs are in INAV's units, inside a loop whose gyro is filtered by
bidirectional-DShot RPM telemetry that this firmware does not implement
(`docs/03-attribution.md` records the encoder's telemetry *bit*; nothing reads
the answer). The numbers our ladder starts from are the ones the simulator
liked, and `docs/04-flight-core.md` and `docs/23-first-flight.md` say so.
"""

import argparse
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
INAV = os.path.join(ROOT, "upstream", "inav-9.1.0")
BETAFLIGHT = os.path.join(ROOT, "upstream", "betaflight-2026.6.1")
CAPTURES = os.path.join(ROOT, "projects", "twin-wings", "ghf435-inav")
# The aircraft's own configuration, in the order it was written: the CLI `dump`
# of the board after the INAV flash (every default), then the two sessions that
# changed things on purpose (the DShot protocol and the differential thrust),
# then the session that ends in `save`. Later files win, which is what makes the
# last one the state the aircraft flies on.
CAPTURE_FILES = [
    "inav-postflash-default-config.txt",
    "inav-repair-after-wizard.txt",
    "inav-diffthrust-enabled.txt",
    "inav-preset-label-set.txt",
]

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def inav_setting(name):
    """`(default, min, max)` for one INAV setting, out of INAV's own table.

    The YAML is read rather than parsed: each entry is `- name: x`, then lines
    like `default_value: 1700` and `min: 1000` at a deeper indent, until the
    next `- name:` at the same level. A tiny state machine is the honest reader
    for that shape - a YAML library would be a dependency for four numbers.
    """
    table = os.path.join(INAV, "src", "main", "fc", "settings.yaml")
    if not os.path.exists(table):
        return None
    values = {}
    inside = False
    for line in open(table, encoding="utf-8", errors="replace"):
        stripped = line.strip()
        if stripped.startswith("- name:"):
            if inside:
                break
            inside = stripped.split(":", 1)[1].strip() == name
            continue
        if not inside or ":" not in stripped:
            continue
        key, _, value = stripped.partition(":")
        key, value = key.strip(), value.strip()
        if key in ("default_value", "min", "max"):
            try:
                values[key] = float(value)
            except ValueError:
                return None      # `:target` and friends: not a number here
    if "default_value" not in values:
        return None
    return values["default_value"], values.get("min"), values.get("max")


def board_dump():
    """Every `set name = value` the aircraft's own configuration carries.

    Read in the order the files were written, so the last one to mention a
    setting is the state the aircraft flies on: the defaults dump first, then
    the sessions that changed DShot, the mixer, and the save.
    """
    values = {}
    for name in CAPTURE_FILES:
        path = os.path.join(CAPTURES, name)
        if not os.path.exists(path):
            continue
        for line in open(path, encoding="utf-8", errors="replace"):
            match = re.match(r"set ([A-Za-z0-9_]+) = (.*)$", line.strip())
            if match:
                values[match.group(1)] = match.group(2).strip()
    return values


def betaflight_small_angle():
    """Betaflight's `DEFAULT_SMALL_ANGLE`, from the ordinary branch.

    It is defined **twice** in `imu.c`: 180 under `#ifdef USE_RACE_PRO`, and 25
    in the `#else`. The attribution cites the 25 - the ordinary build's - so
    this walks the directives rather than grepping for the first `#define` it
    finds, which would have returned the racing build's 180 and made a correct
    default look wrong.
    """
    source = os.path.join(BETAFLIGHT, "src", "main", "flight", "imu.c")
    if not os.path.exists(source):
        return None
    angle = None
    branch = None
    for line in open(source, encoding="utf-8", errors="replace"):
        stripped = line.strip()
        if stripped.startswith("#ifdef USE_RACE_PRO") or \
                stripped.startswith("#if defined(USE_RACE_PRO)"):
            branch = "race"
            continue
        if stripped == "#else" and branch == "race":
            branch = "plain"
            continue
        if stripped == "#endif":
            branch = None
            continue
        match = re.match(r"#define\s+DEFAULT_SMALL_ANGLE\s+(\d+)", stripped)
        if match and branch != "race":
            angle = float(match.group(1))
    return angle


def our_defaults(sim, timeout=30.0):
    """The firmware's own parameter table, as the simulator prints it.

    `set airframe 1` first, because the wing's profile is the one this is
    about, and `params` prints the table a person reads at a bench.
    """
    if not os.path.exists(sim):
        return None
    out = subprocess.run([sim, "0", "console"], input="set airframe 1\nparams\n",
                         capture_output=True, text=True, timeout=timeout)
    values = {}
    for line in out.stdout.replace("\r", "").splitlines():
        match = re.match(r"\s+([a-z_]+)\s+(-?[0-9.]+)\s", line)
        if match:
            values[match.group(1)] = float(match.group(2))
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim",
                        default=os.environ.get("AK_SIM",
                                               "build-host/aerialkit-fw-sim"))
    args = parser.parse_args()

    if not os.path.isdir(INAV) or not os.path.exists(
            os.path.join(CAPTURES, CAPTURE_FILES[0])):
        print("the defaults we borrow: not checked here - no INAV checkout or "
              "no capture (upstream/ and projects/twin-wings/)")
        return 0
    ours = our_defaults(args.sim)
    if ours is None:
        print("the defaults we borrow: not checked here - no simulator at %s "
              "(make host)" % args.sim)
        return 0

    print("the numbers the pages say are the references', read from them")
    dump = board_dump()

    # --- the three launch numbers, from INAV -----------------------------
    launch_thr = inav_setting("nav_fw_launch_thr")
    climb = inav_setting("nav_fw_launch_climb_angle")
    timeout = inav_setting("nav_fw_launch_timeout")
    expect("INAV's table gives the launch throttle a default and a range",
           launch_thr is not None and launch_thr[1] is not None and
           launch_thr[2] is not None,
           " (%s)" % (launch_thr,))
    if launch_thr is not None and launch_thr[1] is not None:
        default, low, high = launch_thr
        fraction = (default - low) / (high - low)
        expect("and this aircraft's own dump holds that same default",
               dump.get("nav_fw_launch_thr") == "%d" % default,
               " (dump says %s)" % dump.get("nav_fw_launch_thr"))
        expect("so launch_throttle is that fraction of that range",
               abs(ours.get("launch_throttle", -1) - fraction) < 0.005,
               " (%d of %d..%d -> %.2f, ours %.2f)"
               % (default, low, high, fraction, ours.get("launch_throttle", -1)))

    expect("launch_climb_deg is INAV's nav_fw_launch_climb_angle",
           climb is not None and
           ours.get("launch_climb_deg") == climb[0] and
           dump.get("nav_fw_launch_climb_angle") == "%d" % climb[0],
           " (%s, ours %s)" % (climb[0] if climb else "?", 
                               ours.get("launch_climb_deg")))

    expect("launch_timeout_s is INAV's nav_fw_launch_timeout in seconds",
           timeout is not None and
           abs(ours.get("launch_timeout_s", -1) - timeout[0] / 1000.0) < 1e-6 and
           dump.get("nav_fw_launch_timeout") == "%d" % timeout[0],
           " (%d ms -> %g s, ours %s)"
           % (timeout[0] if timeout else 0, timeout[0] / 1000.0 if timeout else 0,
              ours.get("launch_timeout_s")))

    # --- the arming angle, from both references --------------------------
    small = inav_setting("small_angle")
    betaflight = betaflight_small_angle()
    expect("the arming gate's angle is the one both references default to",
           small is not None and betaflight is not None and
           small[0] == betaflight == ours.get("arm_max_tilt_deg") and
           dump.get("small_angle") == "%d" % small[0],
           " (INAV %s, Betaflight %s, the board's dump %s, ours %s)"
           % (small[0] if small else "?", betaflight,
              dump.get("small_angle"), ours.get("arm_max_tilt_deg")))

    # --- and the two protocol choices the aircraft itself made -----------
    #
    # Not defaults: the board's INAV configuration was *changed* to DShot300
    # (from INAV's ONESHOT125) and its receiver is CRSF on a serial port. Both
    # are what this firmware's parameters default to, so both are claims about
    # this aircraft rather than about a reference - and both are read off its
    # own configuration here.
    expect("the DShot rate is the one the aircraft was configured for",
           dump.get("motor_pwm_protocol") == "DSHOT300" and
           ours.get("dshot_khz") == 300,
           " (the board says %s, ours %s kHz)"
           % (dump.get("motor_pwm_protocol"), ours.get("dshot_khz")))
    expect("and the receiver protocol is the one it is wired for",
           dump.get("serialrx_provider") == "CRSF" and
           ours.get("rc_protocol") == 0,
           " (%s, and ours 0 is CRSF - docs/08-receiver.md)"
           % dump.get("serialrx_provider"))

    # --- and where we deliberately differ, said rather than implied ------
    #
    # The cruise throttle is the one the pages record as *chosen* rather than
    # borrowed, and the difference is a design one: INAV holds altitude by
    # moving the throttle between nav_fw_min_thr and nav_fw_max_thr, while this
    # firmware holds a constant throttle and a pitch the altitude loop learns.
    # Printing both is the point - it is the reason the number is not the same
    # and the reason nothing here fails on it.
    cruise = inav_setting("nav_fw_cruise_thr")
    if cruise is not None:
        default, low, high = cruise
        print("  ----     and where we differ on purpose: INAV cruises at %.2f "
              "of %d..%d (nav_fw_cruise_thr) and clamps to %s..%s; this firmware "
              "holds rth_cruise = %.2f and learns a standing pitch instead "
              "(docs/14-navigation.md)"
              % ((default - low) / (high - low), low, high,
                 dump.get("nav_fw_min_thr"), dump.get("nav_fw_max_thr"),
                 ours.get("rth_cruise", 0)))

    print("the defaults we borrow: %s" % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
