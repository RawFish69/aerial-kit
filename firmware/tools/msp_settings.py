#!/usr/bin/env python3
"""The setting names a foreign firmware has, read out of the reference sources.

    python3 tools/msp_settings.py betaflight 2026.6.1    # one name per line
    python3 tools/msp_settings.py inav 9.1.0 | wc -l

Why this exists. MSP has no "list the settings" request: a board answers
`MSP2_CLI_SETTING` (0x3010) and `MSP2_CLI_SETTING_INFO` (0x3011) for *one name*
at a time and there is no frame that asks for the names - which is why every MSP
configurator ships the list itself (Betaflight's carries a JSON per release,
INAV's carries its own). This window is no different in kind; it is different in
where the list comes from: not a copy of somebody else's data pinned in this
repository, which would rot quietly, but the pinned reference checkout in
`upstream/` that this workspace already keeps for exactly this purpose. The
board says which release it is, and the list is that release's own table.

What it reads:

* **Betaflight** - `src/main/cli/settings.c`'s `valueTable`. Its entries name a
  setting either with a literal (`{ "failsafe_throttle", ... }`) or with a macro
  from `src/main/fc/parameter_names.h`
  (`{ PARAM_NAME_GYRO_HARDWARE_LPF, ... }`), so *both* are read: a reader that
  only saw the literals would produce a list that looks complete and is missing
  the entries somebody decided to name once.
* **INAV** - `src/main/fc/settings.yaml`, whose `groups:` section is the same
  table as YAML.

What it cannot know, and says in the window: which of those names the board
that is *answering* was compiled with. Betaflight's table wraps some entries in
`#if defined(USE_...)` and a target's build decides; so this is "the names this
release has", which is what the vendor's own configurator offers too. A name the
board does not have comes back as the board's own error rather than as a value,
and the window shows that sentence rather than an empty one.
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
UPSTREAM = os.path.normpath(os.path.join(HERE, "..", "..", "upstream"))

# The two firmwares whose parameters are read by name over MSP, by the variant
# string their own firmware reports.
FIRMWARES = {"BTFL": "betaflight", "INAV": "inav"}


def firmware_for(variant):
    """The reader for a variant string (`BTFL`), or None for one there is no
    reader for - which is an answer, not an error: a board that speaks MSP and
    is neither of these is a board with no name list here."""
    return FIRMWARES.get((variant or "").strip())


def checkout(firmware, version):
    """The pinned checkout of one release, or None when this machine has none.

    The names are a *release's* names, so the directory is the release:
    `upstream/betaflight-2026.6.1`. A board reporting a version this workspace
    has no checkout of gets no list (and the window says which version it
    wanted) - rather than the names of a release that is not on the other end
    of the wire.
    """
    if not firmware or not version:
        return None
    path = os.path.join(UPSTREAM, "%s-%s" % (firmware, version))
    return path if os.path.isdir(path) else None


def names(firmware, version):
    """`(names, source)` for one release, or `([], reason)`.

    `source` is a sentence for a person (`upstream/betaflight-2026.6.1, its
    settings table`): the window shows it, because a list of names with no
    provenance is a list somebody has to trust.
    """
    path = checkout(firmware, version)
    if path is None:
        return [], ("no %s-%s checkout under upstream/ (scripts/"
                    "fetch-upstreams.sh recreates them)" % (firmware, version))
    if firmware == "betaflight":
        return _betaflight(path)
    if firmware == "inav":
        return _inav(path)
    return [], "no reader for %s" % firmware


def _sentence(path, what):
    """Where a list came from, for a person: a bare path with no count reads as
    the file, and the caller that prints a count has one of its own."""
    return "upstream/%s (%s)" % (os.path.basename(path), what)


def _betaflight(root):
    table = os.path.join(root, "src", "main", "cli", "settings.c")
    macros = os.path.join(root, "src", "main", "fc", "parameter_names.h")
    if not os.path.exists(table):
        return [], "no settings table in %s" % root

    # Some entries are named by macro, and the macro's value is the name the
    # board answers to. One definition, one construction per entry - and the
    # pair is what makes the list complete: `gyro_hardware_lpf` appears in no
    # literal in settings.c.
    defined = {}
    if os.path.exists(macros):
        with open(macros, encoding="utf-8", errors="replace") as handle:
            for match in re.finditer(r'#define\s+(PARAM_NAME_\w+)\s+"([^"]*)"',
                                     handle.read()):
                defined[match.group(1)] = match.group(2)

    found = set()
    unknown = 0
    entry = re.compile(r'\{\s*(?:"([^"]+)"|(PARAM_NAME_\w+))\s*[,}]')
    with open(table, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            match = entry.search(line)
            if match is None:
                continue
            if match.group(1):
                found.add(match.group(1))
            elif match.group(2) in defined:
                found.add(defined[match.group(2)])
            else:
                unknown += 1
    if not found:
        return [], "%s has no entries this reader understands" % table
    source = _sentence(root, "its settings table")
    if unknown:
        source += " and %u macro%s this reader cannot resolve" % (
            unknown, "" if unknown == 1 else "s")
    return sorted(found), source


def _inav(root):
    table = os.path.join(root, "src", "main", "fc", "settings.yaml")
    if not os.path.exists(table):
        return [], "no settings.yaml in %s" % root

    # The settings are the `groups:` section of the YAML; `tables:` above it
    # also has `- name:` rows, and they are lookups (the values of a setting),
    # not settings - which is the mistake a one-line grep for `- name:` makes.
    found = set()
    in_groups = False
    row = re.compile(r"^\s+- name:\s*(\S+)\s*$")
    with open(table, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if line.startswith("groups:"):
                in_groups = True
                continue
            if in_groups and line and not line[0].isspace():
                break  # a new top-level key ends the section
            if in_groups:
                match = row.match(line.rstrip())
                if match:
                    found.add(match.group(1))
    if not found:
        return [], "%s has no settings this reader understands" % table
    return sorted(found), _sentence(root, "its settings.yaml groups")


def main(argv):
    if len(argv) != 3:
        print(__doc__.strip().splitlines()[2].strip(), file=sys.stderr)
        print("usage: msp_settings.py <betaflight|inav> <version>",
              file=sys.stderr)
        return 2
    firmware, version = argv[1], argv[2]
    found, source = names(firmware, version)
    if not found:
        print("msp_settings: %s" % source, file=sys.stderr)
        return 1
    for name in found:
        print(name)
    print("# %s" % source, file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
