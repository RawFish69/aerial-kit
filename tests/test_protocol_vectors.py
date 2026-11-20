import json
from pathlib import Path

import pytest

from protocols import check_vectors

VECTORS = Path(__file__).resolve().parents[1] / "protocols" / "vectors"


def test_shared_protocol_vectors() -> None:
    assert check_vectors.main() == 0


def test_checker_rejects_a_bad_crsf_checksum() -> None:
    document = json.loads((VECTORS / "crsf.json").read_text())
    raw = bytearray.fromhex(document["cases"][0]["raw_hex"])
    raw[-1] ^= 1
    document["cases"][0]["raw_hex"] = raw.hex()
    with pytest.raises(AssertionError):
        check_vectors.check_crsf(document)


def test_current_crsf_codec_consumes_the_shared_vectors() -> None:
    from aerial_kit.comms.crsf import CrsfParser, unpack_channels

    document = json.loads((VECTORS / "crsf.json").read_text())
    parser = CrsfParser()
    for case in document["cases"]:
        frames = parser.feed(bytes.fromhex(case["raw_hex"]))
        assert len(frames) == 1, case["name"]
        frame = frames[0]
        assert frame.frame_type == case["frame_type"]
        if frame.frame_type == 0x16:
            assert list(unpack_channels(frame.payload)) == case["decoded"]["channels"]
