from __future__ import annotations

from types import SimpleNamespace

from agent_ros import cli


def _hardware_profile(*_args):
    return SimpleNamespace(name="robot", mode="hardware")


def test_hardware_challenge_refuses_json_without_creating_a_token(tmp_path, monkeypatch, capsys):
    created: list[str] = []
    monkeypatch.setattr(cli, "load_robot_profile", _hardware_profile)
    monkeypatch.setattr(cli, "create_operator_challenge", lambda *_args: created.append("called") or "secret")

    result = cli.main(["--json", "hardware-challenge", "robot", "--runtime-dir", str(tmp_path)])

    assert result == 2
    assert created == []
    assert "secret" not in capsys.readouterr().out


def test_hardware_challenge_refuses_noninteractive_execution_without_creating_a_token(tmp_path, monkeypatch, capsys):
    created: list[str] = []
    monkeypatch.setattr(cli, "load_robot_profile", _hardware_profile)
    monkeypatch.setattr(cli, "create_operator_challenge", lambda *_args: created.append("called") or "secret")
    monkeypatch.setattr(cli, "_is_interactive_terminal", lambda: False)

    result = cli.main(["hardware-challenge", "robot", "--runtime-dir", str(tmp_path)])

    assert result == 2
    assert created == []
    assert "secret" not in capsys.readouterr().out


def test_hardware_challenge_requires_confirmation_and_writes_token_only_to_operator_terminal(tmp_path, monkeypatch, capsys):
    class OperatorTerminal:
        def __init__(self):
            self.writes: list[str] = []

        def write(self, value: str) -> None:
            self.writes.append(value)

        def flush(self) -> None:
            pass

        def readline(self) -> str:
            return "robot\n"

        def close(self) -> None:
            pass

    terminal = OperatorTerminal()
    monkeypatch.setattr(cli, "load_robot_profile", _hardware_profile)
    monkeypatch.setattr(cli, "_is_interactive_terminal", lambda: True)
    monkeypatch.setattr(cli, "_open_operator_terminal", lambda: terminal)
    monkeypatch.setattr(cli, "create_operator_challenge", lambda *_args: "secret")

    result = cli.main(["hardware-challenge", "robot", "--runtime-dir", str(tmp_path)])

    assert result == 0
    assert "secret" not in capsys.readouterr().out
    assert any("secret" in value for value in terminal.writes)
