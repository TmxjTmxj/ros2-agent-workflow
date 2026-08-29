"""Owned subprocess lifecycle for the repository hospital demonstration."""

from __future__ import annotations

import os
import signal
import subprocess
from collections.abc import Sequence
from pathlib import Path

from agent_ros.adapters.base import AdapterError


class ManagedHospitalProcess:
    """Own one fixed hospital-lifecycle child and reap it before close returns."""

    __slots__ = ("_cwd", "_owns_process_group", "_process")

    def __init__(self, *, cwd: Path, owns_process_group: bool = False) -> None:
        self._cwd = cwd
        self._owns_process_group = owns_process_group
        self._process: subprocess.Popen[str] | None = None

    def start(self, argv: Sequence[str]) -> None:
        if self._process is not None or not argv or any(not isinstance(argument, str) for argument in argv):
            raise AdapterError("PROFILE_INVALID")
        self._process = subprocess.Popen(
            list(argv),
            cwd=self._cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            shell=False,
            start_new_session=self._owns_process_group,
        )

    def wait(self, timeout: float) -> tuple[str, str]:
        return self._require_process().communicate(timeout=max(0.0, timeout))

    def poll(self) -> int | None:
        return self._require_process().poll()

    @property
    def returncode(self) -> int | None:
        return self._require_process().returncode

    def terminate(self) -> None:
        process = self._require_process()
        if process.poll() is not None:
            return
        if self._owns_process_group:
            self._signal_exact_group(signal.SIGTERM)
        else:
            process.terminate()

    def close(self, timeout: float) -> bool:
        """Force the owned child down and wait for its exact receipt."""
        process = self._process
        if process is None:
            return True
        successful = True
        try:
            if process.poll() is None:
                if self._owns_process_group:
                    self._signal_exact_group(signal.SIGKILL)
                else:
                    process.kill()
        except ProcessLookupError:
            pass
        except (AdapterError, OSError):
            successful = False
        try:
            process.communicate(timeout=max(0.0, timeout))
        except Exception:
            successful = False
        return successful and process.poll() is not None

    def _signal_exact_group(self, sig: signal.Signals) -> None:
        process = self._require_process()
        pgid = os.getpgid(process.pid)
        if pgid != process.pid:
            raise AdapterError("UNSAFE_STATE")
        os.killpg(pgid, sig)

    def _require_process(self) -> subprocess.Popen[str]:
        process = self._process
        if process is None:
            raise AdapterError("UNSAFE_STATE")
        return process
