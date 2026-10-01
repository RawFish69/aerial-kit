"""Read the firmware's parameter table out of its C source.

**This is no longer how the app learns about a board it is talking to.** Since
milestone 3 of the configurator plan, `param info` and `param help` carry the
group, the ranges, the decimals, the default and the help text off the wire, and
`AerialKitSession` reads them from there. What this file reads the source for now
is the *demo board's* starting table — a fixture, where a fixed file is the
right answer rather than a compromise.

The history matters, because this file's old docstring argued the opposite and
the argument was sound at the time. Until the metadata opcodes existed, a
configurator that wanted to show "0.000..3.000" beside a box had exactly two
options — invent it, or read it out of the firmware's own source — and inventing
it is how a configurator ends up refusing a value the board would have taken.
Reading it out of the source was the better of the two. It was still a *second
authority*, and it went sixty rows stale against a 92-parameter board without
anything failing, because the app joined it to the board's reply by name and a
missing name looks exactly like a parameter with no range.

So: the wire is the authority now, and this parser's only remaining job is the
demo. Its discipline is unchanged and still worth keeping — **it refuses to
guess**. An argument it cannot evaluate becomes `None` and the demo shows no
range rather than a wrong one, which is what keeps a fixture from teaching a
reader something the firmware never wrote.
"""

from __future__ import annotations

import pathlib
import re

# `AK_MIXER_AIRFRAMES - 1u` is a real bound in the table and a literal is not.
# Integer #defines are collected from the source and substituted, which covers
# every symbolic bound the table actually uses.
DEFINE = re.compile(r"^\s*#define\s+([A-Za-z_][A-Za-z0-9_]*)\s+(\d+)[uUlL]*\s*$", re.M)
BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.S)
LINE_COMMENT = re.compile(r"//[^\n]*")
CALL = re.compile(r"\badd_(float|u32)\s*\(\s*items\s*,\s*n\s*,", re.S)
STRING_LITERAL = re.compile(r'"((?:[^"\\]|\\.)*)"')

# `ak_param_group_t` is an enum, not a row of #defines, so the group argument is
# a symbol `_number` cannot resolve from `_defines` alone. It is parsed here
# rather than left as `None`: an unresolvable group would mean every demo
# parameter filed under "unknown", which is a worse answer than reading the four
# lines of enum that say otherwise.
GROUP_ENUM = re.compile(
    r"typedef\s+enum\s*\{(?P<body>[^}]*)\}\s*ak_param_group_t\s*;", re.S
)
GROUP_MEMBER = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*(?:=\s*(\d+))?\s*,?\s*$")

TYPE_BY_MACRO = {"float": "float", "u32": "u32"}


class TableError(Exception):
    pass


def _strip_comments(text: str) -> str:
    return LINE_COMMENT.sub("", BLOCK_COMMENT.sub(" ", text))


def _defines(root: pathlib.Path) -> dict[str, int]:
    found: dict[str, int] = {}
    for path in sorted(root.rglob("*.h")) + sorted(root.rglob("*.c")):
        try:
            text = path.read_text(errors="replace")
        except OSError:
            continue
        for name, value in DEFINE.findall(text):
            found.setdefault(name, int(value))
    return found


def _groups(root: pathlib.Path) -> dict[str, int]:
    """`AK_PARAM_GROUP_*` -> number, straight out of the header that defines it.

    Unassigned members count up from the previous value, as C does; the parser
    therefore walks the body in order rather than regex-matching values, so a
    group inserted in the middle of the enum is picked up with the numbering
    the compiler would give it.
    """
    found: dict[str, int] = {}
    for path in sorted(root.rglob("*.h")):
        try:
            text = _strip_comments(path.read_text(errors="replace"))
        except OSError:
            continue
        match = GROUP_ENUM.search(text)
        if match is None:
            continue
        next_value = 0
        for line in match.group("body").splitlines():
            member = GROUP_MEMBER.match(line)
            if member is None:
                continue
            if member.group(2) is not None:
                next_value = int(member.group(2))
            found[member.group(1)] = next_value
            next_value += 1
        break
    return found


def _split_arguments(text: str) -> list[str]:
    """Split a C argument list on top-level commas only."""
    out: list[str] = []
    depth = 0
    current: list[str] = []
    in_string = False
    escaped = False
    for char in text:
        if in_string:
            current.append(char)
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
            current.append(char)
        elif char in "([{":
            depth += 1
            current.append(char)
        elif char in ")]}":
            depth -= 1
            current.append(char)
        elif char == "," and depth == 0:
            out.append("".join(current).strip())
            current = []
        else:
            current.append(char)
    tail = "".join(current).strip()
    if tail:
        out.append(tail)
    return out


def _c_string(argument: str) -> str | None:
    """A C string, including one written as adjacent literals across lines."""
    pieces = STRING_LITERAL.findall(argument)
    if not pieces:
        return None
    # The only escapes the table uses are for its own punctuation; anything
    # else is left as written rather than half-decoded.
    return "".join(pieces).replace('\\"', '"').replace("\\\\", "\\")


def _number(argument: str, defines: dict[str, int]) -> float | int | None:
    text = argument.strip()
    text = re.sub(r"[uUlLfF]+$", "", text)
    if re.fullmatch(r"-?\d+", text):
        return int(text)
    if re.fullmatch(r"-?\d*\.\d+", text):
        return float(text)
    # `AK_MIXER_AIRFRAMES - 1u`, or a bare symbol.
    match = re.fullmatch(r"([A-Za-z_][A-Za-z0-9_]*)\s*(?:-\s*(\d+))?", text)
    if match and match.group(1) in defines:
        value = defines[match.group(1)]
        if match.group(2) is not None:
            value -= int(match.group(2))
        return value
    return None


def _function_body(text: str, name: str) -> str:
    start = text.find(name)
    if start < 0:
        raise TableError(f"{name} is not in the source any more")
    open_brace = text.find("{", start)
    if open_brace < 0:
        raise TableError(f"{name} has no body")
    depth = 0
    for at in range(open_brace, len(text)):
        if text[at] == "{":
            depth += 1
        elif text[at] == "}":
            depth -= 1
            if depth == 0:
                return text[open_brace + 1 : at]
    raise TableError(f"{name}'s body is not closed")


def extract(source: pathlib.Path, defines_root: pathlib.Path) -> list[dict]:
    """The table, in index order — which is the order the wire uses."""
    defines = {**_groups(defines_root), **_defines(defines_root)}
    body = _function_body(_strip_comments(source.read_text(errors="replace")),
                          "ak_flight_param_table")
    items: list[dict] = []
    for at, match in enumerate(CALL.finditer(body)):
        macro = match.group(1)
        # The call's arguments run to the matching close paren.
        rest = body[match.end() :]
        depth = 0
        end = len(rest)
        for index, char in enumerate(rest):
            if char == "(":
                depth += 1
            elif char == ")":
                if depth == 0:
                    end = index
                    break
                depth -= 1
        arguments = _split_arguments(rest[:end])
        if len(arguments) < 4:
            raise TableError(f"{macro} call {at} has too few arguments")

        name = _c_string(arguments[0])
        help_text = _c_string(arguments[1])
        if name is None:
            raise TableError(f"{macro} call {at} has no name")
        if help_text is None:
            raise TableError(f"{name} has no help text")

        numbers = [_number(argument, defines) for argument in arguments[3:]]
        if macro == "float":
            if len(numbers) < 4:
                raise TableError(f"{name} has too few numbers")
            decimals, low, high, group = numbers[0], numbers[1], numbers[2], numbers[3]
            kind = "float"
        else:
            if len(numbers) < 3:
                raise TableError(f"{name} has too few numbers")
            decimals, low, high, group = 0, numbers[0], numbers[1], numbers[2]
            kind = "u32"

        items.append(
            {
                "index": at,
                "name": name,
                "help": help_text,
                "type": kind,
                "decimals": decimals,
                # `None` means the source wrote a bound this parser will not
                # evaluate. The app shows no range rather than a guessed one.
                "min": low,
                "max": high,
                # The group is the *last* argument of every registration, which
                # is what makes a forgotten one a compile error in the firmware
                # rather than a parameter filed under the wrong heading. Here it
                # is read from the same place, and `None` means the enum member
                # was not one this parser could resolve.
                "group": group,
            }
        )
    if not items:
        raise TableError("the table came out empty")
    return items
