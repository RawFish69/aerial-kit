"""Optional full MAVLink dialect support through pymavlink.

The core codec carries only the messages needed by the Aerial Kit client. This
adapter is for applications that need the complete ArduPilot/PX4 dialect; it
imports pymavlink lazily, so the base package and the small pure-Python codec
remain dependency-free.
"""

from __future__ import annotations

import importlib
from typing import Any, Mapping, cast

from . import codec


class PymavlinkDialect:
    """Encode and decode any message in an installed pymavlink dialect."""

    def __init__(self, dialect: str = "ardupilotmega", version: int = 20) -> None:
        if version not in (10, 20):
            raise ValueError("MAVLink dialect version must be 10 or 20")
        try:
            module = importlib.import_module(
                f"pymavlink.dialects.v{version}.{dialect}"
            )
        except ImportError as exc:
            raise ImportError(
                "PymavlinkDialect requires pymavlink; install aerial-kit[mavlink]"
            ) from exc
        self.module = module
        self.version = version
        self._mav = module.MAVLink(None)
        self._mav.robust_parsing = True

    @property
    def message_types(self) -> tuple[str, ...]:
        """Every message name available in the selected dialect."""
        return tuple(
            getattr(message, "msgname", "").upper()
            for message in self.module.mavlink_map.values()
        )

    def encode(
        self,
        name: str,
        fields: Mapping[str, Any],
        *,
        sequence: int = 0,
        system: int = codec.GCS_SYSTEM,
        component: int = codec.GCS_COMPONENT,
        version: int = 2,
    ) -> bytes:
        """Pack one dialect message into a complete MAVLink frame."""
        cls = getattr(self.module, f"MAVLink_{name.lower()}_message", None)
        if cls is None:
            raise KeyError(f"no {name!r} in pymavlink dialect {self.module.__name__}")
        self._mav.seq = sequence & 0xFF
        self._mav.srcSystem = system & 0xFF
        self._mav.srcComponent = component & 0xFF
        self._mav.WIRE_PROTOCOL_VERSION = "1.0" if version == 1 else "2.0"
        return cast(bytes, cls(**dict(fields)).pack(self._mav))

    def decode(self, frame: bytes) -> tuple[str, dict[str, Any], dict[str, int]]:
        """Decode one complete frame, including messages outside our core set."""
        messages = self._mav.parse_buffer(bytes(frame))
        if messages is None or len(messages) != 1:
            raise ValueError("input is not exactly one MAVLink frame")
        message = messages[0]
        fields = dict(message.to_dict())
        fields.pop("mavpackettype", None)
        return (
            message.get_type(),
            fields,
            {
                "version": 1 if message.get_msgbuf()[0] == codec.MAGIC_V1 else 2,
                "system": message.get_srcSystem(),
                "component": message.get_srcComponent(),
                "sequence": message.get_seq(),
                "compat": 0,
            },
        )
