from __future__ import annotations

from aerial_kit import cli


def test_ak_dispatches_protocol_commands_to_comms(monkeypatch) -> None:
    captured: dict[str, object] = {}

    def fake_main(argv, *, prog):
        captured.update(argv=argv, prog=prog)
        return 7

    monkeypatch.setattr("aerial_kit.comms.cli.main", fake_main)
    assert cli.main(["radio", "uid", "--port", "/dev/null"]) == 7
    assert captured == {
        "argv": ["radio", "uid", "--port", "/dev/null"],
        "prog": "ak",
    }


def test_ak_sim_strips_the_family_word(monkeypatch) -> None:
    captured: list[list[str]] = []

    def fake_main(argv):
        captured.append(list(argv))

    monkeypatch.setattr("aerial_kit.sim.cli.main", fake_main)
    assert cli.main(["sim", "--example", "quadrotor", "--no-show"]) == 0
    assert captured == [["--example", "quadrotor", "--no-show"]]


def test_ak_teleop_strips_the_family_word(monkeypatch) -> None:
    captured: list[list[str]] = []

    def fake_main(argv):
        captured.append(list(argv))

    monkeypatch.setattr("aerial_kit.sim.teleop.main", fake_main)
    assert cli.main(["teleop", "--airframe", "quad"]) == 0
    assert captured == [["--airframe", "quad"]]
