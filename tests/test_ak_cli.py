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


def test_ak_propagates_a_sim_failure(monkeypatch) -> None:
    monkeypatch.setattr("aerial_kit.sim.cli.main", lambda _argv: 5)
    assert cli.main(["sim", "--no-show"]) == 5


def test_ak_teleop_strips_the_family_word(monkeypatch) -> None:
    captured: list[list[str]] = []

    def fake_main(argv):
        captured.append(list(argv))

    monkeypatch.setattr("aerial_kit.sim.teleop.main", fake_main)
    assert cli.main(["teleop", "--airframe", "quad"]) == 0
    assert captured == [["--airframe", "quad"]]


def test_ak_propagates_a_teleop_failure(monkeypatch) -> None:
    monkeypatch.setattr("aerial_kit.sim.teleop.main", lambda _argv: 6)
    assert cli.main(["teleop", "--airframe", "quad"]) == 6


def test_ak_help_lists_all_command_families(capsys) -> None:
    assert cli.main(["--help"]) == 0
    output = capsys.readouterr().out
    for command in ("radio", "fc", "mavlink", "doctor", "sim", "teleop"):
        assert command in output
    assert "arm-capable" in output


def test_ak_help_alias_and_no_argument_exit(capsys) -> None:
    assert cli.main(["help"]) == 0
    assert "usage: ak" in capsys.readouterr().out

    assert cli.main([]) == 2
    assert "usage: ak" in capsys.readouterr().err


def test_ak_doctor_dispatches_and_propagates_its_result(monkeypatch) -> None:
    captured: list[list[str]] = []

    def fake_main(argv):
        captured.append(list(argv))
        return 4

    monkeypatch.setattr("aerial_kit.doctor.main", fake_main)
    assert cli.main(["doctor", "--json", "-"]) == 4
    assert captured == [["--json", "-"]]
