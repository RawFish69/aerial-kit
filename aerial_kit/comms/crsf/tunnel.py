"""Segmentation and reassembly for Aerial Kit protocol frames over CRSF.

The tunnel header is deliberately self-describing enough to recover from
reordering and duplicate delivery without trusting the transport:

    A1 session seq_flags total_length chunk...

``seq_flags`` uses the low six bits for the segment sequence, bit 6 for the
first segment and bit 7 for the last. ``total_length`` is a big-endian 16-bit
length of the complete AKP frame. Every segment repeats the session and total
length so a receiver can reject one transfer mixed into another.
"""

from __future__ import annotations

from dataclasses import dataclass

from .parameters import CrsfProtocolError

AKP_TUNNEL_MAGIC = 0xA1
AKP_TUNNEL_HEADER_LEN = 4
AKP_TUNNEL_SEQUENCE_MASK = 0x3F
AKP_TUNNEL_FIRST = 0x40
AKP_TUNNEL_LAST = 0x80
AKP_TUNNEL_MAX_SEQUENCE = AKP_TUNNEL_SEQUENCE_MASK
AKP_TUNNEL_DEFAULT_CHUNK_SIZE = 4
AKP_TUNNEL_MAX_PAYLOAD = 0xFFFF


class AkpTunnelError(CrsfProtocolError):
    """Base error for AKP tunnel framing and reassembly."""


class AkpTunnelIncomplete(AkpTunnelError):
    """A reassembler was asked to finish before all segment roles arrived."""


class AkpTunnelSessionConflict(AkpTunnelError):
    """A segment belongs to a different session than the active transfer."""


class AkpTunnelMetadataConflict(AkpTunnelError):
    """Repeated session metadata or a segment size disagrees with the transfer."""


class AkpTunnelDuplicateConflict(AkpTunnelError):
    """Two different payloads claim the same sequence number."""


class AkpTunnelSequenceGap(AkpTunnelError):
    """The transfer ends without a contiguous sequence from first to last."""


class AkpTunnelLengthMismatch(AkpTunnelError):
    """The reassembled bytes disagree with the advertised total length."""


@dataclass(frozen=True)
class AkpTunnelSegment:
    """One tunnel header and its chunk."""

    session: int
    sequence: int
    first: bool
    last: bool
    total_length: int
    chunk: bytes

    def __post_init__(self) -> None:
        if not 0 <= self.session <= 0xFF:
            raise AkpTunnelError(f"session must be 0..255, got {self.session}")
        if not 0 <= self.sequence <= AKP_TUNNEL_MAX_SEQUENCE:
            raise AkpTunnelError(
                f"sequence must be 0..{AKP_TUNNEL_MAX_SEQUENCE}, "
                f"got {self.sequence}"
            )
        if not 0 <= self.total_length <= AKP_TUNNEL_MAX_PAYLOAD:
            raise AkpTunnelError(
                f"total length must be 0..{AKP_TUNNEL_MAX_PAYLOAD}, "
                f"got {self.total_length}"
            )
        if self.first and self.sequence != 0:
            raise AkpTunnelError("the first segment must carry sequence zero")
        if self.last and len(self.chunk) > self.total_length:
            raise AkpTunnelError("the final chunk exceeds the advertised total length")

    @property
    def seq_flags(self) -> int:
        flags = self.sequence & AKP_TUNNEL_SEQUENCE_MASK
        if self.first:
            flags |= AKP_TUNNEL_FIRST
        if self.last:
            flags |= AKP_TUNNEL_LAST
        return flags

    def encode(self) -> bytes:
        """Return the tunnel payload carried inside one CRSF MSP envelope."""
        return (
            bytes(
                (
                    AKP_TUNNEL_MAGIC,
                    self.session,
                    self.seq_flags,
                )
            )
            + self.total_length.to_bytes(2, "big")
            + self.chunk
        )

    @classmethod
    def decode(cls, payload: bytes | bytearray | memoryview) -> "AkpTunnelSegment":
        """Parse one complete tunnel payload."""
        data = bytes(payload)
        if len(data) < AKP_TUNNEL_HEADER_LEN:
            raise AkpTunnelError(
                f"tunnel payload needs {AKP_TUNNEL_HEADER_LEN} header bytes, "
                f"got {len(data)}"
            )
        if data[0] != AKP_TUNNEL_MAGIC:
            raise AkpTunnelError(
                f"bad AKP tunnel magic 0x{data[0]:02x}, expected "
                f"0x{AKP_TUNNEL_MAGIC:02x}"
            )
        flags = data[2]
        return cls(
            session=data[1],
            sequence=flags & AKP_TUNNEL_SEQUENCE_MASK,
            first=bool(flags & AKP_TUNNEL_FIRST),
            last=bool(flags & AKP_TUNNEL_LAST),
            total_length=int.from_bytes(data[3:5], "big"),
            chunk=data[5:],
        )


def split_akp_frame(
    payload: bytes | bytearray | memoryview,
    *,
    session: int,
    chunk_size: int = AKP_TUNNEL_DEFAULT_CHUNK_SIZE,
) -> tuple[AkpTunnelSegment, ...]:
    """Split one AKP frame into ordered tunnel segments."""
    data = bytes(payload)
    if len(data) > AKP_TUNNEL_MAX_PAYLOAD:
        raise AkpTunnelError(
            f"AKP frame is {len(data)} bytes; the tunnel limit is "
            f"{AKP_TUNNEL_MAX_PAYLOAD}"
        )
    if chunk_size <= 0:
        raise AkpTunnelError("chunk_size must be positive")
    if chunk_size > AKP_TUNNEL_MAX_PAYLOAD:
        raise AkpTunnelError(
            f"chunk_size must not exceed {AKP_TUNNEL_MAX_PAYLOAD}"
        )

    chunks = [
        data[offset:offset + chunk_size]
        for offset in range(0, len(data), chunk_size)
    ] or [b""]
    if len(chunks) > AKP_TUNNEL_MAX_SEQUENCE + 1:
        raise AkpTunnelError(
            f"AKP frame needs {len(chunks)} segments at {chunk_size} bytes "
            f"each; the sequence field permits {AKP_TUNNEL_MAX_SEQUENCE + 1}"
        )

    final = len(chunks) - 1
    return tuple(
        AkpTunnelSegment(
            session=int(session),
            sequence=index,
            first=index == 0,
            last=index == final,
            total_length=len(data),
            chunk=chunk,
        )
        for index, chunk in enumerate(chunks)
    )


class AkpTunnelReassembler:
    """Reassemble one AKP transfer from possibly reordered segments.

    Duplicate segments are ignored when byte-identical and rejected when they
    disagree. The active session and total length cannot change until the
    current transfer completes or :meth:`reset` is called.
    """

    def __init__(self, *, max_payload: int = AKP_TUNNEL_MAX_PAYLOAD) -> None:
        if not 0 <= max_payload <= AKP_TUNNEL_MAX_PAYLOAD:
            raise ValueError(
                f"max_payload must be 0..{AKP_TUNNEL_MAX_PAYLOAD}"
            )
        self.max_payload = int(max_payload)
        self.reset()

    @property
    def active(self) -> bool:
        return self._session is not None

    @property
    def session(self) -> int | None:
        return self._session

    @property
    def total_length(self) -> int | None:
        return self._total_length

    @property
    def received_sequences(self) -> tuple[int, ...]:
        return tuple(sorted(self._chunks))

    def reset(self) -> None:
        self._session: int | None = None
        self._total_length: int | None = None
        self._chunks: dict[int, bytes] = {}
        self._first_sequence: int | None = None
        self._last_sequence: int | None = None

    def feed(
        self, segment: AkpTunnelSegment | bytes | bytearray | memoryview
    ) -> bytes | None:
        """Accept one segment, returning the complete frame when available."""
        if not isinstance(segment, AkpTunnelSegment):
            segment = AkpTunnelSegment.decode(segment)
        if segment.total_length > self.max_payload:
            raise AkpTunnelMetadataConflict(
                f"advertised length {segment.total_length} exceeds receiver "
                f"limit {self.max_payload}"
            )
        if self._session is None:
            self._session = segment.session
            self._total_length = segment.total_length
        else:
            if segment.session != self._session:
                raise AkpTunnelSessionConflict(
                    f"active session is {self._session}, got "
                    f"{segment.session}"
                )
            if segment.total_length != self._total_length:
                raise AkpTunnelMetadataConflict(
                    f"active total length is {self._total_length}, got "
                    f"{segment.total_length}"
                )

        previous = self._chunks.get(segment.sequence)
        if previous is not None:
            if previous != segment.chunk:
                raise AkpTunnelDuplicateConflict(
                    f"sequence {segment.sequence} arrived twice with "
                    "different payloads"
                )
        else:
            self._chunks[segment.sequence] = segment.chunk

        if segment.first:
            self._first_sequence = 0
        if segment.last:
            if self._last_sequence is not None and \
                    self._last_sequence != segment.sequence:
                raise AkpTunnelMetadataConflict(
                    "two segments claim to be the final segment"
                )
            self._last_sequence = segment.sequence

        if self._last_sequence is not None:
            if any(index > self._last_sequence for index in self._chunks):
                raise AkpTunnelMetadataConflict(
                    f"segment follows final sequence {self._last_sequence}"
                )
            if self._first_sequence is None:
                return None
            expected = set(range(self._last_sequence + 1))
            if set(self._chunks) != expected:
                return None

            assembled = b"".join(
                self._chunks[index] for index in range(self._last_sequence + 1)
            )
            if len(assembled) != self._total_length:
                raise AkpTunnelLengthMismatch(
                    f"reassembled {len(assembled)} bytes, advertised "
                    f"{self._total_length}"
                )
            self.reset()
            return assembled
        return None

    def finish(self) -> bytes:
        """Finish an incomplete feed, raising the precise missing role."""
        if self._session is None:
            raise AkpTunnelIncomplete("no tunnel transfer has started")
        if self._first_sequence is None:
            raise AkpTunnelIncomplete("missing first segment")
        if self._last_sequence is None:
            raise AkpTunnelIncomplete("missing final segment")
        missing = sorted(
            set(range(self._last_sequence + 1)) - set(self._chunks)
        )
        if missing:
            raise AkpTunnelSequenceGap(
                "missing segment sequence(s): "
                + ", ".join(str(index) for index in missing)
            )
        return self.feed(
            AkpTunnelSegment(
                session=self._session,
                sequence=self._last_sequence,
                first=False,
                last=True,
                total_length=self._total_length or 0,
                chunk=self._chunks[self._last_sequence],
            )
        ) or b""
