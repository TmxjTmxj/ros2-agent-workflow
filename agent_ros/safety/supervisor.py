"""Independent deadline supervisor for safety-critical heartbeat loss."""

from __future__ import annotations

import threading
from collections.abc import Callable


class SafetySupervisor:
    """Owns a tracked watchdog and exposes deterministic evaluation for tests/runtime loops."""

    def __init__(
        self,
        *,
        clock: Callable[[], float],
        deadline: Callable[[], float | None],
        on_expired: Callable[[], None],
        poll_interval: float = 0.05,
    ) -> None:
        self._clock = clock
        self._deadline = deadline
        self._on_expired = on_expired
        self._poll_interval = poll_interval
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    @property
    def running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def start(self) -> None:
        with self._lock:
            if self.running:
                return
            self._stop_event.clear()
            thread = threading.Thread(target=self._run, name="agent-ros-safety-watchdog", daemon=False)
            thread.start()
            self._thread = thread

    def evaluate(self) -> bool:
        deadline = self._deadline()
        if deadline is None or self._clock() <= deadline:
            return False
        self._on_expired()
        return True

    def stop(self) -> None:
        self._stop_event.set()

    def join(self, timeout: float = 1.0) -> bool:
        """Wait for owned worker teardown from outside the gateway state lock."""
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=timeout)
        return thread is None or not thread.is_alive()

    def _run(self) -> None:
        while not self._stop_event.wait(self._poll_interval):
            if self.evaluate():
                return
