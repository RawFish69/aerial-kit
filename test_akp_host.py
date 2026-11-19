from __future__ import annotations

import os
import select
import shutil
import subprocess
import time
from pathlib import Path

import pytest

from aerial_kit.comms.akp import AkClient, AkWriteRefused
from aerial_kit.comms.transport import TransportClosed, TransportError

ROOT = Path(__file__).resolve().parent
BUILDER = ROOT / "firmware" / "tools" / "host" / "build_akproto_host.py"


class ProcessTransport:
    def __init__(self, executable: Path, *args: str) -> None:
        self.process = subprocess.Popen(
            [str(executable), *args],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
        )

    def read(self, size: int = -1, timeout: float | None = None) -> bytes:
        if size == 0:
            return b""
        assert self.process.stdout is not None
        ready, _, _ = select.select(
            [self.process.stdout], [], [], 0 if timeout is None else timeout
        )
        if not ready:
            return b""
        return os.read(self.process.stdout.fileno(), 4096 if size < 0 else size)

    def read_exact(self, size: int, timeout: float | None = None) -> bytes:
        deadline = time.monotonic() + (0.0 if timeout is None else timeout)
        output = bytearray()
        while len(output) < size:
            remaining = max(0.0, deadline - time.monotonic())
            data = self.read(size - len(output), remaining)
            if not data:
                if time.monotonic() >= deadline:
                    raise TimeoutError("AKP host did not answer")
                continue
            output.extend(data)
        return bytes(output)

    def write(self, data: bytes) -> int:
        if self.process.stdin is None:
            raise TransportClosed("process input is closed")
        self.process.stdin.write(data)
        self.process.stdin.flush()
        return len(data)

    def close(self) -> None:
        self.process.terminate()
        try:
            self.process.wait(1.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(1.0)


@pytest.fixture(scope="module")
def akp_host(tmp_path_factory: pytest.TempPathFactory) -> Path:
    if shutil.which("cc") is None and shutil.which("gcc") is None:
        pytest.skip("needs a C compiler")
    output = tmp_path_factory.mktemp("akp-host") / "akproto-host"
    subprocess.run(
        [os.environ.get("PYTHON", "python3"), str(BUILDER), "--output", str(output)],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return output


def test_typed_client_against_real_ak_proto_c(akp_host: Path) -> None:
    transport = ProcessTransport(akp_host)
    client = AkClient(transport, timeout=1.0)
    try:
        hello = client.hello()
        assert hello.product == "aerialkit-akp-host"
        assert hello.parameter_count == 3
        status = client.status()
        assert status.link_live == 1
        parameter = client.get_parameter(0)
        assert parameter.name == "rate_kp"
        assert float(parameter.value) == pytest.approx(0.25)
        assert client.log_source(0).count == 2
        outputs = client.output_info()
        assert outputs.count == 2
        assert outputs.motors == 1 and outputs.servos == 1
        assert client.output_test(allow_write=True).status == 0
    finally:
        client.close()


def test_locked_board_refusal_is_typed_and_not_retried(akp_host: Path) -> None:
    transport = ProcessTransport(akp_host, "--locked")
    client = AkClient(transport, timeout=1.0)
    try:
        with pytest.raises(AkWriteRefused):
            client.set_parameter(0, "1.000", allow_write=True)
    finally:
        client.close()
