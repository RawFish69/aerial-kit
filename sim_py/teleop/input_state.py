"""Keyboard state tracking for real-time teleoperation.

This module is deliberately free of Matplotlib and simulator imports so the
input layer can be unit tested without opening a window.

Held controls are *latched*: a key stays active from its press event until its
release event (or until the window loses focus). Nothing here relies on OS key
auto-repeat, which is unavailable or throttled depending on platform.
"""

from __future__ import annotations

from dataclasses import dataclass, field

FORWARD = "forward"
BACKWARD = "backward"
LEFT = "left"
RIGHT = "right"
CLIMB = "climb"
DESCEND = "descend"
YAW_LEFT = "yaw_left"
YAW_RIGHT = "yaw_right"

HELD_ACTIONS: tuple[str, ...] = (
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    CLIMB,
    DESCEND,
    YAW_LEFT,
    YAW_RIGHT,
)

#: Physical key token -> continuous action. Both WASD and the arrow keys are
#: bound to the same actions; the documentation must not claim only one set.
HOLD_BINDINGS: dict[str, str] = {
    "w": FORWARD,
    "up": FORWARD,
    "s": BACKWARD,
    "down": BACKWARD,
    "a": LEFT,
    "left": LEFT,
    "d": RIGHT,
    "right": RIGHT,
    "space": CLIMB,
    "shift": DESCEND,
    "q": YAW_LEFT,
    "e": YAW_RIGHT,
}

QUIT_KEYS = frozenset({"escape"})
PAUSE_KEYS = frozenset({"p"})
NEUTRALIZE_KEYS = frozenset({"x"})
CAMERA_TOGGLE_KEYS = frozenset({"c"})
HELP_TOGGLE_KEYS = frozenset({"h"})
#: Follow the usual convention: minus widens the view, plus moves the camera in.
ZOOM_OUT_KEYS = frozenset({"-", "_", "subtract"})
ZOOM_IN_KEYS = frozenset({"=", "+", "add"})

MODIFIER_TOKENS = frozenset({"shift", "ctrl", "control", "alt", "meta", "super", "cmd"})

#: Long form, logged once at startup.
CONTROL_HELP_LINES = (
    "W/S or Up/Down: forward/backward     A/D or Left/Right: strafe left/right",
    "Space/Shift: climb/descend           Q/E: yaw left/right",
    "X: neutralize commands   P: pause    C: follow/world camera   -/=: zoom out/in",
    "H: hide this help        Esc: exit",
)

#: Single compact line drawn on the canvas every frame.
CONTROL_HELP_BAR = (
    "W/S A/D or arrows move | Space/Shift climb | Q/E yaw | "
    "X neutral | P pause | C cam | -/= zoom out/in | H help | Esc exit"
)


def normalize_key(key: str | None) -> tuple[str, ...]:
    """Split a Matplotlib key string into lowercase physical key tokens.

    Matplotlib reports plain keys (``"w"``, ``"up"``), the literal space
    character, and modifier combinations (``"shift+up"``). Some backends fold
    shifted letters into their uppercase form instead of emitting a combo, so an
    uppercase token also implies that shift is down.
    """
    if key is None:
        return ()
    raw = str(key)
    if raw in {" ", "\t"}:
        return ("space",)

    tokens: list[str] = []
    for part in raw.split("+"):
        part = part.strip()
        if not part:
            # A literal "+" or a trailing separator: nothing to bind.
            continue
        if part == " ":
            tokens.append("space")
            continue
        if len(part) == 1 and part.isalpha() and part.isupper():
            tokens.append("shift")
        lowered = part.lower()
        tokens.append("space" if lowered == "space" else lowered)

    # Preserve order while removing duplicates so callers can rely on the last
    # token being the non-modifier "base" key of a combination.
    seen: set[str] = set()
    unique: list[str] = []
    for token in tokens:
        if token not in seen:
            seen.add(token)
            unique.append(token)
    return tuple(unique)


@dataclass
class KeyboardState:
    """Latched keyboard state plus the session-level flags keys can toggle."""

    held_keys: set[str] = field(default_factory=set)
    paused: bool = False
    running: bool = True
    focused: bool = True
    neutralize_requests: int = 0
    focus_losses: int = 0
    camera_toggle_requests: int = 0
    help_toggle_requests: int = 0
    zoom_requests: int = 0

    def press(self, key: str | None) -> None:
        tokens = normalize_key(key)
        if not tokens:
            return
        base = tokens[-1]

        if base in QUIT_KEYS:
            self.running = False
            return
        if base in PAUSE_KEYS:
            self.paused = not self.paused
            return
        if base in NEUTRALIZE_KEYS:
            self.neutralize()
            return
        if base in CAMERA_TOGGLE_KEYS:
            self.camera_toggle_requests += 1
            return
        if base in HELP_TOGGLE_KEYS:
            self.help_toggle_requests += 1
            return
        if base in ZOOM_IN_KEYS:
            # CameraController.zoom() takes radius notches: negative shrinks the
            # follow radius, which is what "zoom in" means to a pilot.
            self.zoom_requests -= 1
            return
        if base in ZOOM_OUT_KEYS:
            self.zoom_requests += 1
            return

        for token in tokens:
            if token in HOLD_BINDINGS:
                self.held_keys.add(token)

    def release(self, key: str | None) -> None:
        """Release only the base key of the event, keeping held modifiers.

        Releasing ``d`` while shift is down arrives as ``"shift+d"``; shift is
        still physically held and stays latched until its own release event.
        """
        tokens = normalize_key(key)
        if not tokens:
            return
        base = tokens[-1]
        self.held_keys.discard(base)
        if base in MODIFIER_TOKENS:
            return
        for token in tokens[:-1]:
            if token not in MODIFIER_TOKENS:
                self.held_keys.discard(token)

    def neutralize(self) -> None:
        """Drop every held control immediately (the X panic key)."""
        self.held_keys.clear()
        self.neutralize_requests += 1

    def set_focused(self, focused: bool) -> None:
        """Track window focus, clearing held keys when focus is lost.

        Without this, a key held while alt-tabbing away never receives its
        release event and the vehicle keeps accelerating unattended.
        """
        focused = bool(focused)
        if not focused and self.focused:
            self.focus_losses += 1
            self.held_keys.clear()
        self.focused = focused

    def is_active(self, action: str) -> bool:
        return any(HOLD_BINDINGS.get(key) == action for key in self.held_keys)

    def active_actions(self) -> tuple[str, ...]:
        active = {HOLD_BINDINGS[key] for key in self.held_keys if key in HOLD_BINDINGS}
        return tuple(action for action in HELD_ACTIONS if action in active)


__all__ = [
    "BACKWARD",
    "CLIMB",
    "CONTROL_HELP_BAR",
    "CONTROL_HELP_LINES",
    "DESCEND",
    "FORWARD",
    "HELD_ACTIONS",
    "HOLD_BINDINGS",
    "KeyboardState",
    "LEFT",
    "RIGHT",
    "YAW_LEFT",
    "YAW_RIGHT",
    "normalize_key",
]
