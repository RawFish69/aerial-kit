import pytest

from aerial_kit.comms.crsf import (
    BarometerTelemetry,
    CrsfParser,
    decode_telemetry,
    make_barometer_frame,
)

BAROMETER_FRAME = bytes.fromhex("ea06092be2ff0688")
# Emitted by firmware/src/core/flight/ak_crsf_telemetry.c for 123.4 m and
# -2.5 m/s. The firmware addresses its telemetry from the flight controller.
AK_FIRMWARE_BAROMETER_FRAME = bytes.fromhex("c806092be2ff0688")


def test_barometer_literal_decodes_altitude_and_vario():
    decoded = decode_telemetry(CrsfParser().feed(BAROMETER_FRAME)[0])
    assert isinstance(decoded, BarometerTelemetry)
    assert decoded.altitude_m == pytest.approx(123.4)
    assert decoded.vertical_speed_mps == pytest.approx(-2.5)
    assert make_barometer_frame(decoded) == BAROMETER_FRAME


def test_ak_firmware_barometer_frame_decodes():
    decoded = decode_telemetry(CrsfParser().feed(AK_FIRMWARE_BAROMETER_FRAME)[0])
    assert isinstance(decoded, BarometerTelemetry)
    assert decoded.altitude_m == pytest.approx(123.4)
    assert decoded.vertical_speed_mps == pytest.approx(-2.5)
