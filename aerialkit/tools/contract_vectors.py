#!/usr/bin/env python3
"""Write contract/aerialkit-contract-v1.json - the golden vectors.

    make contract-vectors

Three of the four groups come from the firmware itself: the C program
tools/contract_vectors.c runs this tree's own encoders - ak_attitude_ddeg(),
ak_proto_telemetry_frame(), ak_log_encode_record() - and prints what they
returned. Running it is what makes the vectors evidence rather than a second
opinion about the format.

The fourth group cannot come from here. This firmware has no ENU anywhere in
it, so the NED->ENU rotation is the other repository's to state, and the
vectors for it are produced by *importing that repository's own module* rather
than by reimplementing it here. That is the point: the file records what the
two repositories say, and tools/contract_check.py - which shares no code with
either - is what decides whether they agree.

So this script is a producer and not a check. It needs the sibling checkout to
run; the check does not. Regenerating is a deliberate act, recorded in the
`provenance` block it writes.

See docs/31-contract.md.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "contract", "aerialkit-contract-v1.json")

# Where the other repository keeps the transform this contract adopts. It is
# pure standard library - no ROS, no numpy - which is what makes importing it
# from here possible at all.
FRAME_TRANSFORMS = os.path.join(
    "ros2_ws", "src", "mavlink_bridge", "mavlink_bridge", "frame_transforms.py")

# The headings the NED->ENU group covers. Level ones, because a level attitude
# is the only one the Python repository states in closed form, and pinning the
# yaw origin is what this group is for; then a handful of attitudes with roll
# and pitch in them, where the two sides can only be compared through the
# composition itself.
YAW_DEGREES = [0.0, 30.0, 90.0, 180.0, -45.0, 270.0]

# (roll, pitch, yaw) in degrees, firmware sign conventions.
ATTITUDES_DEG = [
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 90.0),
    (30.0, 0.0, 0.0),
    (0.0, 20.0, 0.0),
    (0.0, -20.0, 0.0),
    (45.0, 30.0, 60.0),
    (-45.0, -30.0, -60.0),
    (0.0, 90.0, 0.0),
    (179.0, 0.0, 0.0),
]

# North-East-Down metres, to check the position mapping alongside the attitude.
POSITIONS_NED = [
    (0.0, 0.0, 0.0),
    (1.0, 2.0, -3.0),
    (-100.0, 250.5, 12.25),
    (0.0, 0.0, -100.0),
]


def q_from_euler(roll: float, pitch: float, yaw: float):
    """A body-FRD -> NED attitude, w-first, ZYX - the firmware's own order.

    The firmware's estimator derives its Euler angles from the quaternion in
    this order (ak_estimator.c), so composing them back in the same order is
    what makes these vectors about an attitude rather than about a spelling.
    """
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    )


def q_yaw(yaw: float):
    return q_from_euler(0.0, 0.0, yaw)


def run_c_generator(binary: str) -> dict:
    proc = subprocess.run([binary], capture_output=True, text=True, cwd=ROOT)
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr)
        raise SystemExit("contract-vectors: the C generator failed; build it "
                         "with `make host`")
    return json.loads(proc.stdout)


def build_ned_enu(repo: str) -> dict:
    path = os.path.join(repo, FRAME_TRANSFORMS)
    if not os.path.isfile(path):
        raise SystemExit(
            "contract-vectors: %s is not there.\n"
            "The NED->ENU group is produced by the other repository's own "
            "module, so regenerating needs its checkout:\n"
            "    make contract-vectors REPO=/path/to/aerial-kit" % path)

    sys.path.insert(0, os.path.dirname(os.path.dirname(path)))
    # The simulator package itself, for the level-attitude closed form the
    # second half of this group compares against.
    sys.path.insert(0, os.path.abspath(repo))
    from mavlink_bridge import frame_transforms as ft  # noqa: E402
    from aerial_kit.dynamics.fixed_wing import level_attitude_quat  # noqa: E402

    # The commit the values came from, so a reader can tell whether the file
    # is describing the repository as it is now.
    try:
        commit = subprocess.run(
            ["git", "-C", repo, "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, check=True).stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        commit = "unknown"

    level = []
    for yaw_deg in YAW_DEGREES:
        yaw = math.radians(yaw_deg)
        q_ned = q_yaw(yaw)
        xyzw = ft.ned_body_to_enu_quaternion(*q_ned)
        q_enu = (xyzw[3], xyzw[0], xyzw[1], xyzw[2])

        # What the Python simulator's own level attitude is at the heading
        # that corresponds to this firmware yaw. The two repositories measure
        # heading from different origins - see docs/31-contract.md C2 - and
        # recording both makes the difference visible in the file rather than
        # only in prose.
        heading = math.pi / 2.0 - yaw
        sim = tuple(float(v) for v in level_attitude_quat(heading))
        level.append({
            "yaw_deg": yaw_deg,
            "q_ned_wxyz": list(q_ned),
            "q_ros_xyzw": list(xyzw),
            "q_enu_wxyz": list(q_enu),
            "sim_heading_rad": heading,
            "sim_level_quat_wxyz": list(sim),
        })

    attitudes = []
    for roll_deg, pitch_deg, yaw_deg in ATTITUDES_DEG:
        q_ned = q_from_euler(math.radians(roll_deg), math.radians(pitch_deg),
                             math.radians(yaw_deg))
        xyzw = ft.ned_body_to_enu_quaternion(*q_ned)
        attitudes.append({
            "roll_deg": roll_deg, "pitch_deg": pitch_deg, "yaw_deg": yaw_deg,
            "q_ned_wxyz": list(q_ned),
            "q_ros_xyzw": list(xyzw),
            "q_enu_wxyz": [xyzw[3], xyzw[0], xyzw[1], xyzw[2]],
        })

    positions = []
    for ned in POSITIONS_NED:
        enu = ft.ned_to_enu_position(*ned)
        positions.append({"ned": list(ned), "enu": list(enu),
                          "velocity_same_mapping": True})

    return {
        "source": "%s@%s %s" % (os.path.basename(os.path.abspath(repo)),
                                commit, FRAME_TRANSFORMS),
        "ned_enu_q_wxyz": list(ft._NED_ENU_Q),
        "baselink_q_wxyz": list(ft._AIRCRAFT_BASELINK_Q),
        "level_attitudes": level,
        "attitudes": attitudes,
        "positions": positions,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--binary", default=os.path.join(
        ROOT, "build-host", "aerialkit-contract-vectors"),
        help="the compiled C generator")
    ap.add_argument("--repo", default=os.environ.get(
        "AERIAL_KIT_REPO", os.path.expanduser("~/Projects/aerial-kit")),
        help="the other repository's checkout")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()

    document = run_c_generator(args.binary)
    document["ned_enu"] = build_ned_enu(args.repo)
    document["provenance"] = {
        "firmware": "tools/contract_vectors.c, run against this tree's "
                    "ak_attitude_ddeg / ak_proto_telemetry_frame / "
                    "ak_log_encode_record",
        "ned_enu": document["ned_enu"]["source"],
        "note": "Regenerate with `make contract-vectors`. The check, "
                "`make contract-check`, reads this file and shares no code "
                "with either producer.",
    }

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as handle:
        json.dump(document, handle, indent=2, sort_keys=False)
        handle.write("\n")
    print("contract-vectors: wrote %s" % os.path.relpath(args.out, ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
