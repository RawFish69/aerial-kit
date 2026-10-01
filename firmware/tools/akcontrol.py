#!/usr/bin/env python3
"""The flight core, driven from Python.

    make control-check
    AK_CONTROL_LIB=build-host/libaerialkit-control.so python3 tools/akcontrol.py

`build-host/libaerialkit-control.so` is built from `src/core/flight/*.c` and
`src/core/sensors/*.c` - the same sources the MCU image is built from - behind
the narrow ABI in `tools/ak_control.h`. This module is the Python side of that
seam, and it is the whole of it: **standard library only**, `ctypes` and no
more, for the reason `tools/contract_vectors.py` gives about itself. A binding
that needed numpy to *load a library* would put a dependency between this
repository and every experiment that wants to fly the C core, and the checks in
`make test` would stop being runnable on a machine that has only python3.

**The layout is checked, not assumed.** A struct crossing a language boundary
fails quietly: the C compiler pads, `ctypes` pads by its own reading of the same
ABI, and when the two disagree the program reads a neighbour's bytes and prints
numbers that look like numbers. So every struct here is declared a second time,
and `verify()` asks the library - which has the compiler - what the offsets
actually are, then compares. `akc_layout()` exists for that question and for no
other. A binding that skips the check is one field insertion away from being
wrong in a way nothing catches.

What this module deliberately does not have is a plant. It steps the control
core on samples you hand it and returns what the core produced; where those
samples come from is the caller's business, and keeping it that way is what
makes "estimates separately from truth" true of the interface rather than a
promise - see the note at the top of `ak_control.h`.
"""

from __future__ import annotations

import ctypes
import os
import sys

# From src/core/flight/ak_types.h. An array bound is part of the ABI: a board
# with six motors would change sizeof(akc_outputs_t), and the layout check below
# is what would say so rather than a segmentation fault later.
AK_MAX_MOTORS = 4
AK_MAX_SERVOS = 2

# From tools/ak_control.h. Kept here as literals rather than read from the
# header, because the point of the ABI version is that a *stale* binding
# discovers it is stale - a binding that parsed the number out of the header
# would always agree with it.
AKC_ABI_VERSION = 2
AKC_LAYOUT_MAX = 64
AKC_SAMPLE_MAX_AGE_MS = 250
AKC_COMMAND_MAX_AGE_MS = 500

# akc_result_t
AKC_OK = 0
AKC_NO_IMU = 1
AKC_SAMPLE_STALE = 2
AKC_COMMAND_EXPIRED = 3
AKC_NOT_CONFIGURED = 4
AKC_BAD_ARGUMENT = 5

RESULT_NAMES = {
    AKC_OK: "ok",
    AKC_NO_IMU: "no imu",
    AKC_SAMPLE_STALE: "sample stale",
    AKC_COMMAND_EXPIRED: "command expired",
    AKC_NOT_CONFIGURED: "not configured",
    AKC_BAD_ARGUMENT: "bad argument",
}


class LayoutError(RuntimeError):
    """The library and this module do not agree about a struct."""


class AbiError(RuntimeError):
    """The library is not the one this module was written against."""


class LoadError(RuntimeError):
    """The shared library could not be found or opened."""


# ---------------------------------------------------------------------------
# The structs, declared a second time.
#
# These are written from tools/ak_control.h by hand. That is the point: two
# independent statements of the same layout, compared at run time by the one
# piece of code that can see both. Generating them from the header would make
# the comparison vacuous.
# ---------------------------------------------------------------------------


class Imu(ctypes.Structure):
    _fields_ = [
        ("t_ms", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("gyro", ctypes.c_float * 3),
        ("accel", ctypes.c_float * 3),
        ("valid", ctypes.c_int),
    ]


class Baro(ctypes.Structure):
    _fields_ = [
        ("t_ms", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("pressure_pa", ctypes.c_float),
        ("temperature_c", ctypes.c_float),
        ("valid", ctypes.c_int),
    ]


class Gps(ctypes.Structure):
    _fields_ = [
        ("t_ms", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("lat_e7", ctypes.c_int32),
        ("lon_e7", ctypes.c_int32),
        ("alt_msl_mm", ctypes.c_int32),
        ("speed_mm_s", ctypes.c_int32),
        ("course_e5", ctypes.c_int32),
        ("fix_type", ctypes.c_uint8),
        ("satellites", ctypes.c_uint8),
        ("valid", ctypes.c_int),
    ]


class Samples(ctypes.Structure):
    _fields_ = [
        ("t_ms", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("imu", Imu),
        ("baro", Baro),
        ("gps", Gps),
    ]


class Command(ctypes.Structure):
    _fields_ = [
        ("t_ms", ctypes.c_uint32),
        ("roll", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("yaw", ctypes.c_float),
        ("throttle", ctypes.c_float),
        ("angle_mode", ctypes.c_int),
        ("arm_request", ctypes.c_int),
    ]


class Outputs(ctypes.Structure):
    _fields_ = [
        ("t_ms", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("motor", ctypes.c_float * AK_MAX_MOTORS),
        ("servo", ctypes.c_float * AK_MAX_SERVOS),
        ("dshot", ctypes.c_uint16 * AK_MAX_MOTORS),
        ("servo_us", ctypes.c_uint16 * AK_MAX_SERVOS),
    ]


class State(ctypes.Structure):
    _fields_ = [
        ("now_ms", ctypes.c_uint32),
        ("steps", ctypes.c_uint32),
        ("state", ctypes.c_int),
        ("link_live", ctypes.c_int),
        ("converged", ctypes.c_int),
        ("arm_block", ctypes.c_int),
        ("arm_detail", ctypes.c_float),
        ("q_wxyz", ctypes.c_float * 4),
        ("roll", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("yaw", ctypes.c_float),
        ("gyro", ctypes.c_float * 3),
        ("rate_setpoint", ctypes.c_float * 3),
        ("torque", ctypes.c_float * 3),
        ("cmd_roll", ctypes.c_float),
        ("cmd_pitch", ctypes.c_float),
        ("cmd_yaw", ctypes.c_float),
        ("cmd_throttle", ctypes.c_float),
        ("cmd_angle_mode", ctypes.c_int),
        ("cmd_arm_request", ctypes.c_int),
        ("imu_age_ms", ctypes.c_uint32),
        ("baro_age_ms", ctypes.c_uint32),
        ("gps_age_ms", ctypes.c_uint32),
        ("imu_stale", ctypes.c_int),
        ("baro_stale", ctypes.c_int),
        ("gps_stale", ctypes.c_int),
        ("command_expired", ctypes.c_int),
        ("timing_gap_steps", ctypes.c_uint32),
        ("timing_catchup_steps", ctypes.c_uint32),
        ("timing_dropped_ms", ctypes.c_uint32),
        ("timing_duplicates", ctypes.c_uint32),
        ("timing_long_loops", ctypes.c_uint32),
        ("timing_max_loop_ms", ctypes.c_uint32),
        ("config_hash", ctypes.c_uint32),
        ("mix_needed_motors", ctypes.c_uint32),
        ("mix_needed_servos", ctypes.c_uint32),
        ("board_motors", ctypes.c_uint32),
        ("board_servos", ctypes.c_uint32),
    ]


class CField(ctypes.Structure):
    _fields_ = [
        ("name", ctypes.c_char_p),
        ("offset", ctypes.c_uint32),
        ("size", ctypes.c_uint32),
    ]


class CLayout(ctypes.Structure):
    _fields_ = [
        ("name", ctypes.c_char_p),
        ("size", ctypes.c_uint32),
        ("fields", ctypes.c_uint32),
        ("field", CField * AKC_LAYOUT_MAX),
    ]


STRUCTS = {
    "imu": Imu,
    "baro": Baro,
    "gps": Gps,
    "samples": Samples,
    "command": Command,
    "outputs": Outputs,
    "state": State,
}


# ---------------------------------------------------------------------------
# The library.
# ---------------------------------------------------------------------------

DEFAULT_LIB = os.path.join("build-host", "libaerialkit-control.so")


def find_library(path=None):
    """Where the .so is. `AK_CONTROL_LIB` wins, then the argument, then the
    build directory under this repository - the order `make` uses, so a check
    run by `make` and a check run by hand look at the same file."""
    for candidate in (path, os.environ.get("AK_CONTROL_LIB")):
        if candidate:
            return candidate
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), DEFAULT_LIB)


def verify(lib, only=None):
    """Compare every struct this module declares against the library's own
    description of it. Returns a list of problem strings, empty when they
    agree.

    It compares *names in order*, not just the set: a field inserted in the
    middle moves everything below it, and a check that only asked "is `roll`
    present" would find it present at the wrong offset and pass.
    """
    problems = []

    for which, struct in sorted(STRUCTS.items()):
        if only is not None and which not in only:
            continue

        got = CLayout()
        if lib.akc_layout(which.encode(), ctypes.byref(got)) != 0:
            problems.append("%s: the library has no layout for it" % which)
            continue

        if got.size != ctypes.sizeof(struct):
            problems.append(
                "%s: size %d in C, %d in ctypes"
                % (which, got.size, ctypes.sizeof(struct))
            )

        # The count is computed by the library, so a disagreement here means
        # its own table is not internally consistent - which would make
        # everything below it a comparison against a prefix.
        rows = [got.field[i] for i in range(got.fields)]
        if got.fields == AKC_LAYOUT_MAX:
            problems.append("%s: the library's layout table overflowed" % which)

        mine = [name for name, _ in struct._fields_]
        theirs = [row.name.decode() for row in rows]

        if mine != theirs:
            missing = [n for n in mine if n not in theirs]
            extra = [n for n in theirs if n not in mine]
            if missing:
                problems.append("%s: ctypes has, C does not: %s"
                                % (which, ", ".join(missing)))
            if extra:
                problems.append("%s: C has, ctypes does not: %s"
                                % (which, ", ".join(extra)))
            if not missing and not extra:
                problems.append("%s: same fields, different order" % which)
            continue

        for name, _ in struct._fields_:
            descriptor = getattr(struct, name)
            row = rows[theirs.index(name)]
            if row.offset != descriptor.offset:
                problems.append(
                    "%s.%s: offset %d in C, %d in ctypes"
                    % (which, name, row.offset, descriptor.offset)
                )
            if row.size != descriptor.size:
                problems.append(
                    "%s.%s: size %d in C, %d in ctypes"
                    % (which, name, row.size, descriptor.size)
                )

    return problems


class Core:
    """The flight core, configured and stepped at a clock the caller owns.

        core = Core()
        core.config(open("aircraft.conf").read())
        core.set_board_outputs(4, 0)
        result, out = core.step(core.samples(...), core.command(throttle=0.5), now_ms)

    **There is one core per process.** The library holds the aircraft in file
    scope - `ak_flight_t`, the parameter table, the mixer - because that is what
    it is on a board: one aircraft, one flight core. A second `Core()` returns
    the *same* object rather than a second aircraft, and that is deliberate. Two
    objects over one `ak_flight_t` would each look independent and would silently
    overwrite each other's configuration, which is the worst of both: the
    aliasing is real either way, and only the surprising version is invisible.

    To run a second experiment, `config()` again - a config is a complete reset,
    since it re-initialises the core before applying the table - or call
    `reset()` to go back to the configuration already loaded.
    """

    _instance = None
    _path = None

    def __new__(cls, path=None, check=True):
        wanted = find_library(path)
        if cls._instance is not None:
            if os.path.abspath(wanted) != os.path.abspath(cls._path):
                raise LoadError(
                    "this process is already flying %s; a second library would "
                    "be a second aircraft in one process, which is not something "
                    "the firmware has an answer for" % cls._path
                )
            return cls._instance
        self = super().__new__(cls)
        cls._instance = self
        cls._path = wanted
        self._ready = False
        return self

    def __init__(self, path=None, check=True):
        if self._ready:
            return
        self._ready = True
        self.path = find_library(path)
        if not os.path.exists(self.path):
            raise LoadError(
                "%s is not there - `make build-host/libaerialkit-control.so` "
                "builds it, or point AK_CONTROL_LIB at one" % self.path
            )
        try:
            self.lib = ctypes.CDLL(self.path)
        except OSError as exc:
            raise LoadError("%s: %s" % (self.path, exc)) from exc

        self._declare()
        self._check_abi()
        self.problems = verify(self.lib) if check else []
        if self.problems:
            raise LayoutError(
                "%s and this binding disagree:\n  %s"
                % (self.path, "\n  ".join(self.problems))
            )

    def _declare(self):
        lib = self.lib
        lib.akc_abi_version.restype = ctypes.c_uint32
        for name in ("akc_product", "akc_board", "akc_revision", "akc_built"):
            getattr(lib, name).restype = ctypes.c_char_p

        lib.akc_config.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint]
        lib.akc_config.restype = ctypes.c_int

        lib.akc_set_board_outputs.argtypes = [ctypes.c_uint, ctypes.c_uint]
        lib.akc_set_board_outputs.restype = ctypes.c_int

        lib.akc_config_hash.restype = ctypes.c_uint32

        lib.akc_config_text.argtypes = [ctypes.c_char_p, ctypes.c_uint]
        lib.akc_config_text.restype = ctypes.c_uint

        lib.akc_reset.argtypes = []
        lib.akc_reset.restype = None

        lib.akc_step.argtypes = [
            ctypes.POINTER(Samples),
            ctypes.POINTER(Command),
            ctypes.c_uint32,
            ctypes.POINTER(Outputs),
        ]
        lib.akc_step.restype = ctypes.c_int

        lib.akc_state.argtypes = [ctypes.POINTER(State)]
        lib.akc_state.restype = ctypes.c_int

        lib.akc_layout.argtypes = [ctypes.c_char_p, ctypes.POINTER(CLayout)]
        lib.akc_layout.restype = ctypes.c_int

    def _check_abi(self):
        got = self.lib.akc_abi_version()
        if got != AKC_ABI_VERSION:
            raise AbiError(
                "the library speaks ABI %d, this binding knows %d - rebuild "
                "the library, or the binding is older than it" % (got, AKC_ABI_VERSION)
            )

    # -- identity ----------------------------------------------------------

    @property
    def abi_version(self):
        return self.lib.akc_abi_version()

    @property
    def product(self):
        return self.lib.akc_product().decode()

    @property
    def board(self):
        return self.lib.akc_board().decode()

    @property
    def revision(self):
        return self.lib.akc_revision().decode()

    @property
    def built(self):
        return self.lib.akc_built().decode()

    def identity(self):
        """What this library *is*: the four strings the firmware stamps into
        its own boot report, so an experiment result and a firmware image can
        be shown to come from the same source."""
        return {
            "abi": self.abi_version,
            "product": self.product,
            "board": self.board,
            "revision": self.revision,
            "built": self.built,
        }

    # -- configuration -----------------------------------------------------

    def config(self, text, board_outputs=None):
        """Load `name=value\\n` lines, the firmware's own saved format. Raises
        ValueError with the library's message on a bad one."""
        if isinstance(text, (list, tuple)):
            text = "".join(
                line if line.endswith("\n") else line + "\n" for line in text
            )
        msg = ctypes.create_string_buffer(256)
        rc = self.lib.akc_config(text.encode(), msg, len(msg))
        if rc != 0:
            raise ValueError(msg.value.decode() or "the configuration was refused")
        if board_outputs is not None:
            self.set_board_outputs(*board_outputs)
        return True

    def set_board_outputs(self, motors, servos):
        """How many outputs the board being stood in for has. Nothing arms
        without it, which is the same answer a board with no timers gives."""
        rc = self.lib.akc_set_board_outputs(motors, servos)
        if rc != 0:
            raise ValueError(
                "a board cannot have %d motors and %d servos; the most this "
                "build mixes is %d and %d" % (motors, servos,
                                              AK_MAX_MOTORS, AK_MAX_SERVOS)
            )

    @property
    def config_hash(self):
        """C8's identity. Zero before a successful config()."""
        return self.lib.akc_config_hash()

    def config_text(self):
        """The effective table as text - what an experiment files its result
        under, and what `config_hash` is a hash of."""
        buf = ctypes.create_string_buffer(4096)
        n = self.lib.akc_config_text(buf, len(buf))
        if n == 0:
            return ""
        return buf.value.decode()

    def reset(self):
        """Back to a just-configured core: the configuration stays and its
        identity is unchanged, while the estimator's state, the timers, the arm
        gate and the output sequence all go back to where they started. This is
        how a second identical run is set up, and it is what makes the
        reproducibility claim checkable inside one process."""
        self.lib.akc_reset()

    @property
    def configured(self):
        """Whether a config() has been accepted since the last reset that had
        nothing to restore."""
        return self.config_hash != 0

    # -- building a step ---------------------------------------------------

    @staticmethod
    def imu(t_ms=0, sequence=0, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 1.0),
            valid=True):
        """One inertial sample. gyro is rad/s in body FRD and accel is in g,
        per src/core/flight/ak_types.h; `accel = (0, 0, 1)` is the aircraft
        sitting level, which is what the estimator levels itself against."""
        return Imu(
            t_ms=int(t_ms),
            sequence=int(sequence),
            gyro=(ctypes.c_float * 3)(*[float(v) for v in gyro]),
            accel=(ctypes.c_float * 3)(*[float(v) for v in accel]),
            valid=1 if valid else 0,
        )

    @staticmethod
    def baro(t_ms=0, sequence=0, pressure_pa=101325.0, temperature_c=15.0,
             valid=False):
        return Baro(
            t_ms=int(t_ms),
            sequence=int(sequence),
            pressure_pa=float(pressure_pa),
            temperature_c=float(temperature_c),
            valid=1 if valid else 0,
        )

    @staticmethod
    def gps(t_ms=0, sequence=0, lat_e7=0, lon_e7=0, alt_msl_mm=0,
            speed_mm_s=0, course_e5=0, fix_type=0, satellites=0, valid=False):
        return Gps(
            t_ms=int(t_ms),
            sequence=int(sequence),
            lat_e7=int(lat_e7),
            lon_e7=int(lon_e7),
            alt_msl_mm=int(alt_msl_mm),
            speed_mm_s=int(speed_mm_s),
            course_e5=int(course_e5),
            fix_type=int(fix_type),
            satellites=int(satellites),
            valid=1 if valid else 0,
        )

    @classmethod
    def samples(cls, t_ms, imu, sequence=0, baro=None, gps=None):
        """One frame. The three sensors do not run at one rate, and this is
        where a consumer that wanted to pretend they did would have to."""
        return Samples(
            t_ms=int(t_ms),
            sequence=int(sequence),
            imu=imu,
            baro=baro if baro is not None else cls.baro(t_ms=t_ms),
            gps=gps if gps is not None else cls.gps(t_ms=t_ms),
        )

    @staticmethod
    def command(t_ms=0, roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0,
                angle_mode=False, arm_request=False):
        """The stick shape, per C6. `throttle` is 0..1 and the three axes are
        -1..1; what the firmware makes of them is the firmware's business."""
        return Command(
            t_ms=int(t_ms),
            roll=float(roll),
            pitch=float(pitch),
            yaw=float(yaw),
            throttle=float(throttle),
            angle_mode=1 if angle_mode else 0,
            arm_request=1 if arm_request else 0,
        )

    # -- stepping ----------------------------------------------------------

    def step(self, samples, command=None, now_ms=None, outputs=None):
        """One control step. Returns `(result, outputs)`.

        `result` is an `akc_result_t`, and anything but AKC_OK means the core
        declined this step - a stale sample, a command past its TTL, no IMU.
        The outputs come back either way, because they are the last ones the
        core produced and a caller that wanted "what did it do" should not have
        to guess whether the buffer it holds is fresh.
        """
        if now_ms is None:
            now_ms = samples.t_ms
        if outputs is None:
            outputs = Outputs()
        result = self.lib.akc_step(
            ctypes.byref(samples),
            ctypes.byref(command) if command is not None else None,
            int(now_ms),
            ctypes.byref(outputs),
        )
        return result, outputs

    def state(self):
        """What the core believes. Every number is an estimate or a state;
        the world's own attitude is not in here and has nowhere to be."""
        out = State()
        if self.lib.akc_state(ctypes.byref(out)) != 0:
            raise RuntimeError("the core has no state to report")
        return out


def describe(result):
    """`akc_result_t` as words, for a message a person reads."""
    return RESULT_NAMES.get(result, "unknown result %d" % result)


def main(argv):
    path = argv[1] if len(argv) > 1 else None
    try:
        core = Core(path)
    except (LoadError, AbiError, LayoutError) as exc:
        print("akcontrol: %s" % exc, file=sys.stderr)
        return 1

    ident = core.identity()
    print("library:  %s" % core.path)
    print("product:  %s" % ident["product"])
    print("board:    %s" % ident["board"])
    print("revision: %s" % ident["revision"])
    print("built:    %s" % ident["built"])
    print("abi:      %d" % ident["abi"])

    fields = sum(len(s._fields_) for s in STRUCTS.values())
    print(
        "layout:   %d structs, %d fields, all offsets agree"
        % (len(STRUCTS), fields)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
