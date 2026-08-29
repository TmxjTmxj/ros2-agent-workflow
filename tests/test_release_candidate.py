from __future__ import annotations

import hashlib
from pathlib import Path

from scripts.verify_release_candidate import verify_release_candidate


def _write_candidate(directory: Path, *, corrupt_manifest: bool = False) -> None:
    artifacts = {
        "agent_ros-0.1.0-py3-none-any.whl": b"wheel bytes",
        "agent_ros-0.1.0.tar.gz": b"source bytes",
    }
    for name, contents in artifacts.items():
        (directory / name).write_bytes(contents)
    lines = [f"{hashlib.sha256(contents).hexdigest()}  {name}" for name, contents in sorted(artifacts.items())]
    if corrupt_manifest:
        lines[0] = f"{'0' * 64}  {sorted(artifacts)[0]}"
    (directory / "SHA256SUMS.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_release_candidate_verifier_accepts_one_complete_versioned_pair(tmp_path, capsys):
    _write_candidate(tmp_path)

    assert verify_release_candidate(tmp_path) == 0
    assert '"ok": true' in capsys.readouterr().out


def test_release_candidate_verifier_rejects_manifest_hash_mismatch(tmp_path, capsys):
    _write_candidate(tmp_path, corrupt_manifest=True)

    assert verify_release_candidate(tmp_path) == 2
    assert "SHA-256 mismatch" in capsys.readouterr().out


def test_release_candidate_verifier_rejects_missing_expected_artifact(tmp_path, capsys):
    _write_candidate(tmp_path)
    (tmp_path / "agent_ros-0.1.0.tar.gz").unlink()

    assert verify_release_candidate(tmp_path) == 2
    assert "missing expected distribution artifact" in capsys.readouterr().out


def test_release_candidate_verifier_rejects_unexpected_distribution_artifact(tmp_path, capsys):
    _write_candidate(tmp_path)
    (tmp_path / "agent_ros-0.1.1-py3-none-any.whl").write_bytes(b"unexpected")

    assert verify_release_candidate(tmp_path) == 2
    assert "unexpected distribution artifact" in capsys.readouterr().out
