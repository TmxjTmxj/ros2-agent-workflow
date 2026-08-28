from __future__ import annotations

import os
import selectors
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from types import ModuleType, SimpleNamespace

import agent_ros.adapters as adapters_package
import pytest
from agent_ros.adapters._safety import _ActivationIssuer, _EmergencyStopChannel
from agent_ros.adapters._safety import _ActivationPermit as ReExportedActivationPermit
from agent_ros.adapters.base import (
    AdapterError,
    AdapterStatus,
    HospitalAction,
    NavigationGoal,
    OdometrySample,
    TwistCommand,
    create_adapter,
)
from agent_ros.adapters.hospital import (
    HospitalCaseAdapter,
    HospitalDeliveryAdapter,
    HospitalLifecycleClient,
    HospitalSimulationRuntime,
    RclpyHospitalTransport,
)
from agent_ros.adapters.nav2 import Nav2Adapter
from agent_ros.adapters.twist import TwistAdapter
from agent_ros.profiles.models import PoseGoal, RobotProfile, TaskStage
from agent_ros.safety.outcome import EmergencyStopResult
from agent_ros.safety.sequencer import _ActivationPermit, _ActivationRejected, _SafetySequencer


def robot_profile(kind: str = "twist", *, mode: str = "simulation") -> RobotProfile:
    interfaces: dict[str, object]
    if kind == "nav2":
        interfaces = {
            "navigation": {
                "action": "/navigate_to_pose",
                "type": "nav2_msgs/action/NavigateToPose",
            }
        }
    else:
        interfaces = {
            "command": {"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"},
            "odometry": {"topic": "/odom", "type": "nav_msgs/msg/Odometry"},
        }
    return RobotProfile.from_mapping(
        {
            "name": "robot",
            "mode": mode,
            "namespace": "/robot",
            "frames": {"base": "base_link", "odom": "odom"},
            "adapter": {"kind": kind},
            "interfaces": interfaces,
            "limits": {
                "max_linear_velocity": 0.5,
                "max_angular_velocity": 1.0,
                "max_linear_acceleration": 0.5,
                "max_angular_acceleration": 1.0,
            },
            "safety": {"heartbeat_timeout": 1.0, "estop_topic": "/emergency_stop"},
            "observation_sources": ["odometry"],
        }
    )


class RecordingEmergencyChannel(_EmergencyStopChannel):
    def __init__(self, enqueue, *, hardware_verified: bool = True, available: bool = True) -> None:
        super().__init__(hardware_verified=hardware_verified)
        self._enqueue = enqueue
        self._available = available

    def _preflight(self) -> bool:
        return self._available

    def _enqueue_zero_disable(self) -> None:
        self._enqueue()


class TwistTransport:
    def __init__(self) -> None:
        self.commands: list[TwistCommand] = []
        self.odometry = OdometrySample(timestamp=0.0, x=1.0, y=2.0, yaw=0.25)
        self.estop_handler = None
        self.started_waypoints = []
        self.state = AdapterStatus("idle")
        self.generation = 0
        self.safety_channel = RecordingEmergencyChannel(self._enqueue_emergency_stop)

    def publish(self, command: TwistCommand) -> None:
        self.commands.append(command)

    def read_odometry(self) -> OdometrySample:
        return self.odometry

    def subscribe_estop(self, handler) -> None:
        self.estop_handler = handler

    def preflight_activation(self) -> bool:
        return True

    def start_waypoint(self, stage, _permit=None) -> None:
        self.started_waypoints.append(stage)
        self.state = AdapterStatus("running")

    def waypoint_status(self):
        return self.state

    def cancel_waypoint(self):
        self.state = AdapterStatus("cancelled")

    def stop_waypoint(self):
        self.commands.extend([TwistCommand.zero()] * 3)
        self.state = AdapterStatus("stopped")

    def _enqueue_emergency_stop(self):
        self.generation += 1
        self.commands.append(TwistCommand.zero())

    def emergency_channel(self):
        return self.safety_channel


def stage(*, timeout: float = 30.0) -> TaskStage:
    return TaskStage("destination", PoseGoal("odom", 2.0, 2.0, 0.0), 0.1, timeout)


def bind_permit(adapter):
    issuer = _ActivationIssuer()
    adapter._bind_runtime_safety(issuer)
    adapter._validate_runtime_safety("simulation")
    return issuer._issue()


def valid_permit(adapter, owner):
    owner(adapter)
    return bind_permit(adapter)


def test_direct_adapter_context_management_closes_bound_workers():
    transport = TwistTransport()

    with TwistAdapter(robot_profile(), transport, clock=lambda: 0.0) as adapter:
        bind_permit(adapter)
        assert adapter._safety_sequencer.worker_alive

    assert not adapter._safety_sequencer.worker_alive


def test_adapter_close_before_start_is_successful():
    adapter = TwistAdapter(robot_profile(), TwistTransport(), clock=lambda: 0.0)

    assert adapter.close(timeout=0.1)


def test_started_adapter_close_is_idempotent():
    adapter = TwistAdapter(robot_profile(), TwistTransport(), clock=lambda: 0.0)
    bind_permit(adapter)

    assert adapter.close(timeout=0.2)
    assert adapter.close(timeout=0.2)


def test_adapter_context_manager_exposes_close_failure():
    class FailingCloseAdapter(TwistAdapter):
        def close(self, timeout: float = 1.0) -> bool:
            return False

    with pytest.raises(AdapterError, match="CLEANUP_FAILED"):
        with FailingCloseAdapter(robot_profile(), TwistTransport(), clock=lambda: 0.0):
            pass


@contextmanager
def ready_subprocess(program: str, *, ready_timeout: float):
    """Yield one READY child and always reap that exact process."""
    process = subprocess.Popen(
        [sys.executable, "-c", program],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    body_error = None
    body_traceback = None
    close_errors = []
    try:
        assert process.stdout is not None
        selector = selectors.DefaultSelector()
        ready_bytes = bytearray()
        deadline = time.monotonic() + ready_timeout
        try:
            selector.register(process.stdout, selectors.EVENT_READ)
            while b"\n" not in ready_bytes:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0 or not selector.select(remaining):
                    raise TimeoutError("subprocess readiness timeout")
                chunk = os.read(process.stdout.fileno(), 4096)
                if not chunk:
                    raise RuntimeError("subprocess exited before readiness")
                ready_bytes.extend(chunk)
                if len(ready_bytes) > 4096:
                    raise RuntimeError("subprocess readiness response too large")
        finally:
            try:
                selector.close()
            except BaseException as exc:
                close_errors.append(("selector", exc))
        if close_errors:
            raise close_errors[0][1]
        ready_line, _separator, _remainder = ready_bytes.partition(b"\n")
        if ready_line.strip() != b"READY":
            raise RuntimeError("subprocess readiness rejected")
        yield process
    except BaseException as exc:
        body_error = exc
        body_traceback = exc.__traceback__
    finally:
        process_error = None
        try:
            _cleanup_subprocess(process)
        except BaseException as exc:
            process_error = exc
        for name, stream in (("stdout", process.stdout), ("stderr", process.stderr)):
            if stream is None:
                continue
            try:
                stream.close()
            except BaseException as exc:
                close_errors.append((name, exc))

        if process_error is not None:
            if body_error is not None:
                _add_error_note(process_error, "suppressed body error", body_error)
            for name, error in close_errors:
                _add_error_note(process_error, f"suppressed {name} close error", error)
            raise process_error
        if body_error is not None:
            for name, error in close_errors:
                if error is not body_error:
                    _add_error_note(body_error, f"suppressed {name} close error", error)
            raise body_error.with_traceback(body_traceback)
        if close_errors:
            _name, primary_error = close_errors[0]
            for name, error in close_errors[1:]:
                _add_error_note(primary_error, f"suppressed {name} close error", error)
            raise primary_error


def _add_error_note(primary: BaseException, label: str, error: BaseException) -> None:
    primary.add_note(f"{label}: {type(error).__name__}: {error}")


def _cleanup_subprocess(process) -> None:
    """Attempt every safe cleanup step for one exact child without short-circuiting."""
    errors: list[BaseException] = []
    reaped = False
    killed = False
    final_wait_error = None

    try:
        reaped = process.poll() is not None
    except BaseException as exc:
        errors.append(exc)

    if not reaped:
        try:
            process.terminate()
        except BaseException as exc:
            errors.append(exc)
        try:
            process.wait(timeout=0.5)
            reaped = True
        except BaseException as exc:
            errors.append(exc)

    if not reaped:
        try:
            process.kill()
            killed = True
        except BaseException as exc:
            errors.append(exc)
        try:
            process.wait(timeout=0.5)
            reaped = True
        except BaseException as exc:
            errors.append(exc)

    if not reaped and killed:
        try:
            process.wait()
            reaped = True
        except BaseException as exc:
            final_wait_error = exc
            errors.append(exc)

    if not reaped:
        cause = final_wait_error
        if cause is None:
            cause = next(
                (error for error in errors if not isinstance(error, subprocess.TimeoutExpired)),
                errors[-1] if errors else None,
            )
        cleanup_error = RuntimeError("subprocess cleanup failed")
        for error in errors:
            if error is not cause:
                _add_error_note(cleanup_error, "earlier process cleanup error", error)
        raise cleanup_error from cause
    for error in errors:
        if not isinstance(error, subprocess.TimeoutExpired):
            for other_error in errors:
                if other_error is not error:
                    _add_error_note(error, "suppressed process cleanup error", other_error)
            raise error


def test_ready_subprocess_reaps_after_kill_when_timed_wait_still_times_out(
    monkeypatch,
):
    class FakePipe:
        closed = False

        def fileno(self):
            return 123

        def close(self):
            self.closed = True

    class FakeSelector:
        def register(self, _stream, _event):
            return None

        def select(self, _timeout):
            return [(object(), selectors.EVENT_READ)]

        def close(self):
            return None

    class FakeProcess:
        def __init__(self):
            self.stdout = FakePipe()
            self.stderr = FakePipe()
            self.returncode = None
            self.calls = []

        def poll(self):
            return self.returncode

        def terminate(self):
            self.calls.append("terminate")

        def kill(self):
            self.calls.append("kill")

        def wait(self, timeout=None):
            self.calls.append(("wait", timeout))
            if timeout is not None:
                raise subprocess.TimeoutExpired("controlled-child", timeout)
            self.returncode = -9
            return self.returncode

    process = FakeProcess()
    monkeypatch.setattr(subprocess, "Popen", lambda *_args, **_kwargs: process)
    monkeypatch.setattr(selectors, "DefaultSelector", FakeSelector)
    monkeypatch.setattr(os, "read", lambda _fd, _size: b"READY\n")

    with ready_subprocess("controlled", ready_timeout=0.1) as yielded:
        assert yielded is process

    assert process.calls == [
        "terminate",
        ("wait", 0.5),
        "kill",
        ("wait", 0.5),
        ("wait", None),
    ]
    assert process.returncode == -9
    assert process.stdout.closed
    assert process.stderr.closed


def test_ready_subprocess_reaps_then_propagates_unexpected_wait_error(monkeypatch):
    class FakePipe:
        closed = False

        def fileno(self):
            return 123

        def close(self):
            self.closed = True

    class FakeSelector:
        def register(self, _stream, _event):
            return None

        def select(self, _timeout):
            return [(object(), selectors.EVENT_READ)]

        def close(self):
            return None

    class FakeProcess:
        def __init__(self):
            self.stdout = FakePipe()
            self.stderr = FakePipe()
            self.returncode = None
            self.calls = []

        def poll(self):
            return self.returncode

        def terminate(self):
            self.calls.append("terminate")

        def kill(self):
            self.calls.append("kill")

        def wait(self, timeout=None):
            self.calls.append(("wait", timeout))
            if timeout is not None:
                raise RuntimeError("controlled wait failure")
            self.returncode = -9
            return self.returncode

    process = FakeProcess()
    monkeypatch.setattr(subprocess, "Popen", lambda *_args, **_kwargs: process)
    monkeypatch.setattr(selectors, "DefaultSelector", FakeSelector)
    monkeypatch.setattr(os, "read", lambda _fd, _size: b"READY\n")

    with pytest.raises(RuntimeError, match="controlled wait failure"):
        with ready_subprocess("controlled", ready_timeout=0.1):
            pass

    assert process.calls == [
        "terminate",
        ("wait", 0.5),
        "kill",
        ("wait", 0.5),
        ("wait", None),
    ]
    assert process.returncode == -9
    assert process.stdout.closed
    assert process.stderr.closed


@pytest.mark.parametrize(
    ("failure_step", "expected_calls", "expected_error"),
    [
        (
            "poll",
            ["poll", "terminate", ("wait", 0.5)],
            "controlled poll failure",
        ),
        (
            "terminate",
            ["poll", "terminate", ("wait", 0.5)],
            "controlled terminate failure",
        ),
        (
            "kill",
            [
                "poll",
                "terminate",
                ("wait", 0.5),
                "kill",
                ("wait", 0.5),
            ],
            "controlled kill failure",
        ),
        (
            "timed_wait",
            [
                "poll",
                "terminate",
                ("wait", 0.5),
                "kill",
                ("wait", 0.5),
            ],
            "controlled timed wait failure",
        ),
        (
            "final_wait",
            [
                "poll",
                "terminate",
                ("wait", 0.5),
                "kill",
                ("wait", 0.5),
                ("wait", None),
            ],
            "subprocess cleanup failed",
        ),
        ("selector_close", ["poll"], "controlled selector close failure"),
        ("body", ["poll"], "controlled body failure"),
        ("pipe_close", ["poll"], "controlled stdout close failure"),
    ],
)
def test_ready_subprocess_cleanup_never_short_circuits_after_step_error(
    monkeypatch, failure_step, expected_calls, expected_error
):
    close_calls = []
    body_error = ValueError("controlled body failure")

    class FakePipe:
        closed = False

        def __init__(self, name):
            self._name = name

        def fileno(self):
            return 123

        def close(self):
            self.closed = True
            close_calls.append(f"{self._name}.close")
            if failure_step in {
                "final_wait",
                "selector_close",
                "body",
                "pipe_close",
            }:
                raise RuntimeError(f"controlled {self._name} close failure")

    class FakeSelector:
        def register(self, _stream, _event):
            return None

        def select(self, _timeout):
            return [(object(), selectors.EVENT_READ)]

        def close(self):
            if failure_step == "selector_close":
                raise RuntimeError("controlled selector close failure")

    class FakeProcess:
        def __init__(self):
            self.stdout = FakePipe("stdout")
            self.stderr = FakePipe("stderr")
            self.returncode = None
            self.calls = []
            self.timed_waits = 0

        def poll(self):
            self.calls.append("poll")
            if failure_step == "poll":
                raise RuntimeError("controlled poll failure")
            if failure_step in {"selector_close", "body", "pipe_close"}:
                self.returncode = 0
            return self.returncode

        def terminate(self):
            self.calls.append("terminate")
            if failure_step == "terminate":
                raise RuntimeError("controlled terminate failure")

        def kill(self):
            self.calls.append("kill")
            if failure_step == "kill":
                raise RuntimeError("controlled kill failure")

        def wait(self, timeout=None):
            self.calls.append(("wait", timeout))
            if timeout is None:
                if failure_step == "final_wait":
                    raise RuntimeError("controlled final wait failure")
                self.returncode = -9
                return self.returncode
            self.timed_waits += 1
            if failure_step in {"poll", "terminate"}:
                self.returncode = -15
                return self.returncode
            if self.timed_waits == 1:
                if failure_step == "timed_wait":
                    raise RuntimeError("controlled timed wait failure")
                raise subprocess.TimeoutExpired("controlled-child", timeout)
            if failure_step == "final_wait":
                raise subprocess.TimeoutExpired("controlled-child", timeout)
            self.returncode = -9
            return self.returncode

    process = FakeProcess()
    monkeypatch.setattr(subprocess, "Popen", lambda *_args, **_kwargs: process)
    monkeypatch.setattr(selectors, "DefaultSelector", FakeSelector)
    monkeypatch.setattr(os, "read", lambda _fd, _size: b"READY\n")

    with pytest.raises(RuntimeError, match=expected_error) as captured:
        with ready_subprocess("controlled", ready_timeout=0.1):
            if failure_step == "final_wait":
                raise body_error
            if failure_step == "body":
                raise RuntimeError("controlled body failure")

    assert process.calls == expected_calls
    assert close_calls == ["stdout.close", "stderr.close"]
    assert process.stdout.closed
    assert process.stderr.closed
    if failure_step == "final_wait":
        assert str(captured.value.__cause__) == "controlled final wait failure"
        assert captured.value.__notes__ == [
            "earlier process cleanup error: TimeoutExpired: Command 'controlled-child' timed out after 0.5 seconds",
            "earlier process cleanup error: TimeoutExpired: Command 'controlled-child' timed out after 0.5 seconds",
            "suppressed body error: ValueError: controlled body failure",
            "suppressed stdout close error: RuntimeError: controlled stdout close failure",
            "suppressed stderr close error: RuntimeError: controlled stderr close failure",
        ]
        assert process.returncode is None
    elif failure_step in {"selector_close", "body"}:
        assert captured.value.__cause__ is None
        assert captured.value.__context__ is None
        assert captured.value.__notes__ == [
            "suppressed stdout close error: RuntimeError: controlled stdout close failure",
            "suppressed stderr close error: RuntimeError: controlled stderr close failure",
        ]
    elif failure_step == "pipe_close":
        assert captured.value.__cause__ is None
        assert captured.value.__context__ is None
        assert captured.value.__notes__ == [
            "suppressed stderr close error: RuntimeError: controlled stderr close failure",
        ]
    else:
        assert process.returncode is not None


def test_subprocess_readiness_timeout_is_bounded_and_reaps_child(tmp_path):
    pid_path = tmp_path / "child.pid"
    program = f"""
import os
import signal
import time

signal.signal(signal.SIGTERM, signal.SIG_IGN)
with open({str(pid_path)!r}, "w", encoding="ascii") as stream:
    stream.write(str(os.getpid()))
time.sleep(60.0)
"""
    began = time.monotonic()

    with pytest.raises(TimeoutError, match="readiness timeout"):
        with ready_subprocess(program, ready_timeout=0.2):
            pytest.fail("a child without READY must never be yielded")

    assert time.monotonic() - began < 1.5
    child_pid = int(pid_path.read_text(encoding="ascii"))
    with pytest.raises(ChildProcessError):
        os.waitpid(child_pid, os.WNOHANG)


def test_caller_omitting_adapter_close_remains_alive_until_harness_termination():
    program = """
from tests.test_adapters import TwistAdapter, TwistTransport, bind_permit, robot_profile

adapter = TwistAdapter(robot_profile(), TwistTransport(), clock=lambda: 0.0)
bind_permit(adapter)
print("READY", flush=True)
"""
    with ready_subprocess(program, ready_timeout=2.0) as process:
        with pytest.raises(subprocess.TimeoutExpired):
            process.wait(timeout=0.1)

    assert process.returncode is not None
    assert process.stdout is not None and process.stdout.closed
    assert process.stderr is not None and process.stderr.closed


class CallbackFuture:
    def __init__(self):
        self.callback = None
        self.value = None
        self.error = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        if self.error:
            raise self.error
        return self.value

    def resolve(self, value):
        self.value = value
        if self.callback is not None:
            self.callback(self)

    def reject(self, error):
        self.error = error
        if self.callback is not None:
            self.callback(self)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeActionClient:
    def __init__(self, node, _action_type, _action_name):
        self.goal_future = CallbackFuture()
        node.action_client = self

    def send_goal_async(self, _goal):
        return self.goal_future


class FakeNode:
    def __init__(self):
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self.action_client = None

    def create_publisher(self, _message_type, _topic, _depth):
        publisher = FakePublisher()
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, _message_type, _topic, callback, _depth):
        self.subscriptions.append(callback)
        return callback

    def create_timer(self, period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object()))


def install_fake_ros(monkeypatch) -> None:
    class FakeTwist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0)
            self.angular = SimpleNamespace(z=0.0)

    class FakeNavigateToPose:
        class Goal:
            def __init__(self):
                self.pose = SimpleNamespace(
                    header=SimpleNamespace(frame_id=None, stamp=None),
                    pose=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0),
                        orientation=SimpleNamespace(z=0.0, w=0.0),
                    ),
                )

    modules = {
        "geometry_msgs.msg": {"Twist": FakeTwist},
        "nav_msgs.msg": {"Odometry": type("Odometry", (), {})},
        "std_msgs.msg": {"Bool": type("Bool", (), {})},
        "nav2_msgs.action": {"NavigateToPose": FakeNavigateToPose},
        "rclpy.action": {"ActionClient": FakeActionClient},
    }
    for dotted_name, attributes in modules.items():
        parent_name, child_name = dotted_name.split(".")
        parent = sys.modules.get(parent_name, ModuleType(parent_name))
        child = ModuleType(dotted_name)
        for name, value in attributes.items():
            setattr(child, name, value)
        setattr(parent, child_name, child)
        monkeypatch.setitem(sys.modules, parent_name, parent)
        monkeypatch.setitem(sys.modules, dotted_name, child)


def real_twist_transport(monkeypatch, *, clock=lambda: 0.0):
    from agent_ros.adapters.twist import RclpyTwistTransport

    install_fake_ros(monkeypatch)
    node = FakeNode()
    transport = RclpyTwistTransport(
        node,
        "/cmd_vel",
        "/odom",
        "/emergency_stop",
        limits=robot_profile().limits,
        control_period=0.1,
    )
    transport._clock = clock
    return transport


def real_nav2_transport(monkeypatch, *, clock=lambda: 0.0, cancel_timeout=1.0):
    from agent_ros.adapters.nav2 import RclpyNav2Transport

    install_fake_ros(monkeypatch)
    node = FakeNode()
    transport = RclpyNav2Transport(
        node,
        "/navigate_to_pose",
        "/cmd_vel",
        "/emergency_stop",
        clock=clock,
        cancel_timeout=cancel_timeout,
    )
    return transport


def test_twist_start_accepts_only_a_reviewed_stage(adapter_owner):
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    adapter.start(stage(), valid_permit(adapter, adapter_owner))

    assert transport.started_waypoints == [stage()]


def _wait_until(predicate, timeout=1.0):
    deadline = __import__("time").monotonic() + timeout
    while not predicate() and __import__("time").monotonic() < deadline:
        __import__("time").sleep(0.001)
    return predicate()


def test_twist_emergency_stop_idempotently_accepts_a_fresh_zero_enqueue(adapter_owner):
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    valid_permit(adapter, adapter_owner)

    adapter._emergency_stop()
    adapter._emergency_stop()

    assert _wait_until(lambda: len(transport.commands) == 2)
    assert transport.commands == [TwistCommand.zero()] * 2


def test_twist_stale_odometry_stops_with_a_zero_burst_and_reports_stable_code(adapter_owner):
    now = [0.0]
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: now[0], stale_after=1.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    now[0] = 1.01
    transport.waypoint_status = lambda: (_ for _ in ()).throw(AdapterError("STALE_FEEDBACK"))

    with pytest.raises(AdapterError, match="STALE_FEEDBACK"):
        adapter.status()

    assert transport.commands[-3:] == [TwistCommand.zero()] * 3


@pytest.mark.parametrize("timestamp", [-2.0, 0.2])
def test_twist_stage_refuses_stale_or_future_odometry_before_any_nonzero_motion(timestamp, adapter_owner):
    transport = TwistTransport()
    transport.odometry = OdometrySample(timestamp=timestamp, x=1.0, y=2.0, yaw=0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0, stale_after=1.0, future_skew=0.05)

    with pytest.raises(AdapterError, match="STALE_FEEDBACK"):
        adapter.start(stage(), valid_permit(adapter, adapter_owner))

    assert all(command == TwistCommand.zero() for command in transport.commands)


def test_twist_stage_delegates_feedback_control_to_transport_and_status_never_publishes(adapter_owner):
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    published = list(transport.commands)

    adapter.status()

    assert transport.started_waypoints == [stage()]
    assert transport.commands == published


def test_twist_rejects_direct_command_as_a_public_authority_bypass(adapter_owner):
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter.start(TwistCommand(0.1, 0.0), valid_permit(adapter, adapter_owner))

    assert transport.commands == []


def test_standard_adapter_start_requires_a_controller_owned_internal_permit():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)

    for forged in (None, object(), "permit"):
        with pytest.raises(AdapterError, match="PROFILE_INVALID"):
            adapter.start(stage(), forged)

    assert transport.started_waypoints == []
    assert not hasattr(adapters_package, "SafetyToken")


def test_permit_from_a_different_issuer_is_rejected(adapter_owner):
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter_owner(adapter)
    owner = _ActivationIssuer()
    foreign = _ActivationIssuer()
    adapter._bind_runtime_safety(owner)
    adapter._validate_runtime_safety("simulation")

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter.start(stage(), foreign._issue())

    assert transport.started_waypoints == []


def test_hardware_adapter_rejects_an_unverified_emergency_channel(adapter_owner):
    transport = TwistTransport()
    transport.safety_channel = RecordingEmergencyChannel(
        transport._enqueue_emergency_stop,
        hardware_verified=False,
    )
    adapter = TwistAdapter(robot_profile(mode="hardware"), transport, clock=lambda: 0.0)
    adapter_owner(adapter)
    issuer = _ActivationIssuer()
    adapter._bind_runtime_safety(issuer)

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter._validate_runtime_safety("hardware")


def test_twist_runtime_timer_limits_first_command_acceleration_from_zero(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    commands = []
    transport.publish = commands.append

    transport._control_step()

    assert commands == [TwistCommand(0.05, 0.0)]


@pytest.mark.parametrize("permit_kind", ["missing", "invalid", "foreign"])
def test_twist_timer_without_exact_owned_permit_fails_closed(monkeypatch, permit_kind, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    foreign = None
    if permit_kind == "missing":
        transport._stage_permit = None
    elif permit_kind == "invalid":
        transport._stage_permit = object()
    else:
        foreign = _ActivationIssuer()
        assert foreign.start()
        transport._stage_permit = foreign.issue()
    commands = []
    transport.publish = commands.append
    try:
        transport._control_step()

        assert commands == []
        assert transport.waypoint_status() == AdapterStatus("faulted", "UNSAFE_STATE")
    finally:
        assert adapter.close(0.2)
        if foreign is not None:
            assert foreign.close(0.2)


def test_adapter_close_latches_before_emergency_close_and_shares_one_deadline():
    close_entered = threading.Event()
    release_close = threading.Event()

    class BlockingCloseChannel(RecordingEmergencyChannel):
        def __init__(self, enqueue) -> None:
            super().__init__(enqueue)
            self.offered: list[float] = []

        def _close(self, timeout: float) -> bool:
            self.offered.append(timeout)
            close_entered.set()
            release_close.wait(timeout)
            return False

        def finish(self) -> bool:
            return super()._close(0.2)

    class RecordingSequencer(_SafetySequencer):
        def __init__(self) -> None:
            super().__init__()
            self.offered: list[float] = []

        def close(self, timeout: float) -> bool:
            self.offered.append(timeout)
            return super().close(timeout)

    transport = TwistTransport()
    channel = BlockingCloseChannel(transport._enqueue_emergency_stop)
    transport.safety_channel = channel
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    sequencer = RecordingSequencer()
    adapter._bind_runtime_safety(sequencer)
    adapter._validate_runtime_safety("simulation")
    old_permit = sequencer.issue()
    close_results = []
    began = time.monotonic()
    closer = threading.Thread(target=lambda: close_results.append(adapter.close(0.05)))
    closer.start()
    try:
        assert close_entered.wait(0.2)
        with pytest.raises(_ActivationRejected):
            sequencer.issue()
        invoked = []
        with pytest.raises(_ActivationRejected):
            sequencer.submit(old_permit, lambda: invoked.append(True), timeout=0.02)
        assert invoked == []

        closer.join(0.2)
        assert not closer.is_alive()
        assert time.monotonic() - began < 0.2
        assert close_results == [False]
        assert len(channel.offered) == 1
        assert len(sequencer.offered) == 1
        assert sum(channel.offered + sequencer.offered) <= 0.055
    finally:
        release_close.set()
        closer.join(0.2)
        assert channel.finish()


def test_twist_timer_snapshot_before_estop_cannot_publish_nonzero_after_estop_returns(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    commands = []
    transport.publish = commands.append
    snapshot = threading.Event()
    release = threading.Event()
    transport._before_publish = lambda: (snapshot.set(), release.wait(1.0))
    worker = threading.Thread(target=transport._control_step)
    worker.start()
    assert snapshot.wait(1.0)

    adapter._emergency_stop()
    release.set()
    worker.join(1.0)

    assert _wait_until(lambda: bool(commands))
    assert commands
    first_zero = commands.index(TwistCommand.zero())
    assert all(command == TwistCommand.zero() for command in commands[first_zero:])


def test_twist_emergency_enqueue_never_waits_for_a_blocked_ros_publish(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    valid_permit(adapter, adapter_owner)
    entered = threading.Event()
    release = threading.Event()
    errors = []

    def blocked_publish(_message):
        entered.set()
        release.wait()

    transport._publisher.publish = blocked_publish
    worker = threading.Thread(target=lambda: _capture(errors, adapter._emergency_stop))
    worker.start()
    worker.join(0.05)
    blocked = worker.is_alive()
    release.set()
    worker.join(1.0)

    assert not blocked
    assert not errors


def test_twist_estop_success_waits_for_blocked_nonzero_publish(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    entered = threading.Event()
    release = threading.Event()
    commands = []

    def publish(command):
        if command != TwistCommand.zero():
            entered.set()
            release.wait()
        commands.append(command)

    transport.publish = publish
    timer = threading.Thread(target=transport._control_step)
    timer.start()
    try:
        assert entered.wait(0.2)
        results = []
        stop = threading.Thread(target=lambda: results.append(adapter._emergency_stop(0.2)))
        stop.start()
        assert stop.is_alive()
        release.set()
        stop.join(0.2)
        timer.join(0.2)

        assert not stop.is_alive()
        assert not timer.is_alive()
        assert results == [EmergencyStopResult(True, True, True, "ESTOP_LATCHED")]
        assert any(command != TwistCommand.zero() for command in commands)
    finally:
        release.set()
        timer.join(0.2)
        assert adapter.close(0.2)


def test_twist_estop_degrades_when_publish_does_not_quiesce(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    entered = threading.Event()
    release = threading.Event()

    def publish(command):
        if command != TwistCommand.zero():
            entered.set()
            release.wait()

    transport.publish = publish
    timer = threading.Thread(target=transport._control_step)
    timer.start()
    try:
        assert entered.wait(0.2)
        began = time.monotonic()

        result = adapter._emergency_stop(0.02)

        assert time.monotonic() - began < 0.2
        assert result == EmergencyStopResult(True, False, True, "TRANSPORT_UNQUIESCED")
    finally:
        release.set()
        timer.join(0.2)
        assert not timer.is_alive()
        assert adapter.close(0.2)


class Nav2Transport:
    def __init__(self) -> None:
        self.requests = []
        self.cancel_count = 0
        self.zeros = 0
        self.estop_handler = None
        self.estop_cancelled = False
        self.safety_channel = RecordingEmergencyChannel(self._enqueue_emergency_stop)

    def preflight_activation(self) -> bool:
        return True

    def prepare_goal(self, request):
        return request

    def send_goal(self, goal, _permit=None) -> None:
        self.requests.append(goal)

    def track_goal(self, _future, _permit) -> None:
        return None

    def goal_status(self):
        return {"state": "running"}

    def cancel_goal(self) -> None:
        self.cancel_count += 1

    def publish_zero(self) -> None:
        self.zeros += 1

    def subscribe_estop(self, handler) -> None:
        self.estop_handler = handler

    def _enqueue_emergency_stop(self):
        self.zeros += 1
        if not self.estop_cancelled:
            self.cancel_count += 1
            self.estop_cancelled = True

    def emergency_channel(self):
        return self.safety_channel


def test_nav2_emits_fixed_action_type_with_structured_goal_values(adapter_owner):
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)

    adapter.start(NavigationGoal(frame="map", x=1.25, y=-2.5, yaw=0.75), valid_permit(adapter, adapter_owner))

    request = transport.requests[0]
    assert request.action_type == "nav2_msgs/action/NavigateToPose"
    assert request.action_name == "/navigate_to_pose"
    assert (request.frame, request.x, request.y, request.yaw) == ("map", 1.25, -2.5, 0.75)


def test_nav2_emergency_stop_publishes_independent_zero_and_initiates_cancel(adapter_owner):
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    valid_permit(adapter, adapter_owner)

    adapter._emergency_stop()

    assert _wait_until(lambda: transport.zeros == 1)
    assert transport.zeros == 1
    assert transport.cancel_count == 1

    adapter._emergency_stop()

    assert _wait_until(lambda: transport.zeros == 2)
    assert transport.zeros == 2
    assert transport.cancel_count == 1


def test_nav2_estop_success_waits_for_send_goal_boundary(adapter_owner):
    entered = threading.Event()
    release = threading.Event()

    class BlockingNav2Transport(Nav2Transport):
        def send_goal(self, goal, _permit=None):
            entered.set()
            release.wait()
            return super().send_goal(goal, _permit)

    transport = BlockingNav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    permit = valid_permit(adapter, adapter_owner)
    errors = []
    starter = threading.Thread(target=lambda: _capture(errors, adapter.start, stage(), permit))
    starter.start()
    try:
        assert entered.wait(0.2)
        results = []
        stop = threading.Thread(target=lambda: results.append(adapter._emergency_stop(0.2)))
        stop.start()
        assert stop.is_alive()
        release.set()
        stop.join(0.2)
        starter.join(0.2)

        assert not stop.is_alive()
        assert not starter.is_alive()
        assert results == [EmergencyStopResult(True, True, True, "ESTOP_LATCHED")]
        assert any(isinstance(error, AdapterError) for error in errors)
    finally:
        release.set()
        starter.join(0.2)
        assert adapter.close(0.2)


def test_nav2_estop_degrades_when_send_goal_does_not_quiesce(adapter_owner):
    entered = threading.Event()
    release = threading.Event()

    class BlockingNav2Transport(Nav2Transport):
        def send_goal(self, goal, _permit=None):
            entered.set()
            release.wait()
            return super().send_goal(goal, _permit)

    transport = BlockingNav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    permit = valid_permit(adapter, adapter_owner)
    errors = []
    starter = threading.Thread(target=lambda: _capture(errors, adapter.start, stage(), permit))
    starter.start()
    try:
        assert entered.wait(0.2)
        began = time.monotonic()

        result = adapter._emergency_stop(0.02)

        assert time.monotonic() - began < 0.2
        assert result == EmergencyStopResult(True, False, True, "TRANSPORT_UNQUIESCED")
    finally:
        release.set()
        starter.join(0.2)
        assert not starter.is_alive()
        assert adapter.close(0.2)


def test_no_stale_timer_publish_after_successful_estop(monkeypatch, adapter_owner):
    transport = real_twist_transport(monkeypatch)
    transport._sample = OdometrySample(0.0, 1.0, 2.0, 0.0)
    adapter = TwistAdapter(robot_profile(), transport, clock=lambda: 0.0)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    snapshot = threading.Event()
    release = threading.Event()
    commands = []
    transport.publish = commands.append
    transport._before_publish = lambda: (snapshot.set(), release.wait())
    timer = threading.Thread(target=transport._control_step)
    timer.start()
    try:
        assert snapshot.wait(0.2)

        result = adapter._emergency_stop(0.2)
        release.set()
        timer.join(0.2)

        assert result.successful
        assert not timer.is_alive()
        assert _wait_until(lambda: bool(commands), timeout=0.2)
        assert commands == [TwistCommand.zero()]
    finally:
        release.set()
        timer.join(0.2)
        assert adapter.close(0.2)


def test_late_nav2_goal_response_is_best_effort_cancelled(monkeypatch, adapter_owner):
    class Handle:
        accepted = True

        def __init__(self) -> None:
            self.cancel_count = 0
            self.cancel_future = CallbackFuture()
            self.result_future = CallbackFuture()

        def cancel_goal_async(self):
            self.cancel_count += 1
            return self.cancel_future

        def get_result_async(self):
            return self.result_future

    transport = real_nav2_transport(monkeypatch)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    try:
        result = adapter._emergency_stop(0.2)
        handle = Handle()

        transport._client.goal_future.resolve(handle)

        assert result.successful
        assert handle.cancel_count == 1
        assert transport.goal_status() == {"state": "cancelling"}
    finally:
        assert adapter.close(0.2)


def test_nav2_estop_between_reservation_and_goal_enqueue_rejects_late_start(adapter_owner):
    entered = threading.Event()
    release = threading.Event()
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    permit = valid_permit(adapter, adapter_owner)
    adapter._before_activation = lambda: (entered.set(), release.wait(1.0))
    errors = []
    worker = threading.Thread(
        target=lambda: _capture(errors, adapter.start, stage(), permit),
    )
    worker.start()
    assert entered.wait(1.0)

    adapter._emergency_stop()
    release.set()
    worker.join(1.0)

    assert transport.requests == []
    assert _wait_until(lambda: transport.zeros == 1)
    assert transport.zeros == 1
    assert any(isinstance(error, AdapterError) and error.code == "ESTOP_LATCHED" for error in errors)


def test_nav2_goal_callback_registration_happens_after_atomic_enqueue_lock(adapter_owner):
    class ImmediateCompletionTransport(Nav2Transport):
        def send_goal(self, goal, _permit=None):
            self.requests.append(goal)
            return object()

        def track_goal(self, _future, _permit):
            self.safety_channel._stop()

    transport = ImmediateCompletionTransport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    permit = valid_permit(adapter, adapter_owner)

    with pytest.raises(AdapterError, match="ESTOP_LATCHED"):
        adapter.start(stage(), permit)

    assert len(transport.requests) == 1
    assert _wait_until(lambda: transport.zeros == 1)
    assert transport.zeros == 1


def test_nav2_maps_a_reviewed_task_stage_to_the_fixed_goal_shape(adapter_owner):
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)

    adapter.start(stage(), valid_permit(adapter, adapter_owner))

    request = transport.requests[0]
    assert (request.frame, request.x, request.y, request.yaw) == ("odom", 2.0, 2.0, 0.0)


def test_nav2_cancellation_calls_cancel_once_then_sends_a_zero_burst():
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)

    adapter.cancel()

    assert transport.cancel_count == 1
    assert transport.zeros == 3


def test_nav2_repeated_fail_closed_stop_cancels_a_pending_goal_only_once(adapter_owner):
    transport = Nav2Transport()
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(NavigationGoal(frame="map", x=1.0, y=0.0, yaw=0.0), valid_permit(adapter, adapter_owner))

    adapter.stop()
    adapter.stop()

    assert transport.cancel_count == 1
    assert transport.zeros == 6


def test_adapter_selection_rejects_unknown_or_unimplemented_kinds():
    profile = SimpleNamespace(adapter=SimpleNamespace(kind="raw_shell"))

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        create_adapter(profile, object())


def test_adapter_physical_estop_subscription_is_wired_to_runtime_handler():
    transport = TwistTransport()
    adapter = TwistAdapter(robot_profile(), transport)
    assertions: list[bool] = []

    adapter.bind_physical_estop(assertions.append)
    transport.estop_handler(True)

    assert assertions == [True]


def test_hospital_adapter_accepts_only_fixed_actions_on_owned_simulation_runtime(adapter_owner):
    runtime = HospitalSimulationRuntime()
    adapter = HospitalDeliveryAdapter(runtime)
    assert adapter.start(HospitalAction.START, valid_permit(adapter, adapter_owner)).state == "running"
    assert runtime.commands == (HospitalAction.START,)

    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        adapter.start("start", valid_permit(adapter, adapter_owner))


def test_hospital_adapter_rejects_an_arbitrary_callable_runner_at_construction():
    with pytest.raises(AdapterError, match="PROFILE_INVALID"):
        HospitalDeliveryAdapter(lambda _action: {"state": "running"})


def test_hospital_simulation_rejects_late_start_after_permit_invalidation(adapter_owner):
    entered = threading.Event()
    release = threading.Event()
    runtime = HospitalSimulationRuntime()
    adapter = HospitalDeliveryAdapter(runtime)
    permit = valid_permit(adapter, adapter_owner)
    adapter._before_activation = lambda: (entered.set(), release.wait(1.0))
    errors = []
    worker = threading.Thread(
        target=lambda: _capture(errors, adapter.start, HospitalAction.START, permit),
    )
    worker.start()
    assert entered.wait(1.0)

    adapter._emergency_stop()
    release.set()
    worker.join(1.0)

    assert HospitalAction.START not in runtime.commands
    assert any(isinstance(error, AdapterError) and error.code == "ESTOP_LATCHED" for error in errors)


def test_hospital_start_never_waits_then_dispatches_late_behind_runtime_lock(adapter_owner):
    runtime = HospitalSimulationRuntime()
    adapter = HospitalDeliveryAdapter(runtime)
    permit = valid_permit(adapter, adapter_owner)
    errors = []
    runtime._lock.acquire()
    worker = threading.Thread(
        target=lambda: _capture(errors, adapter.start, HospitalAction.START, permit),
    )
    worker.start()
    worker.join(0.05)
    blocked = worker.is_alive()
    adapter._emergency_stop()
    runtime._lock.release()
    worker.join(1.0)

    assert not blocked
    assert HospitalAction.START not in runtime.commands
    assert any(isinstance(error, AdapterError) and error.code == "INTERNAL_ERROR" for error in errors)


def test_hospital_case_emergency_stop_uses_independent_fixed_worker_while_start_blocks(monkeypatch, adapter_owner):
    start_entered = threading.Event()
    release_start = threading.Event()
    stop_executed = threading.Event()
    mission_start_executed = threading.Event()
    stop_count = 0

    def fixed_call(self, suffix, *, timeout, generation=None):
        nonlocal stop_count
        if generation is not None and self._start_was_cancelled(generation):
            return None
        if suffix[0] == "start":
            start_entered.set()
            assert release_start.wait(1.0)
            return {"ok": True, "running": True}
        if suffix[0] == "mission-start":
            mission_start_executed.set()
            return {"ok": True, "success": True}
        if suffix[0] == "stop":
            stop_count += 1
            stop_executed.set()
            release_start.set()
            return {"ok": True, "running": False}
        raise AssertionError(suffix)

    monkeypatch.setattr(HospitalLifecycleClient, "_run_fixed", fixed_call)
    client = HospitalLifecycleClient()
    adapter = HospitalCaseAdapter(client)
    permit = valid_permit(adapter, adapter_owner)
    adapter.start(HospitalAction.START, permit)
    assert start_entered.wait(0.2)

    result = adapter._emergency_stop(timeout=0.5)

    assert result.successful
    assert stop_executed.wait(0.2)
    release_start.set()
    assert client._start_receipt.done.wait(0.5)
    assert not mission_start_executed.is_set()
    assert stop_count >= 2
    assert adapter.close(1.0)


def test_hospital_persistent_start_waits_for_fresh_diff_drive_odometry(monkeypatch):
    transport = object.__new__(RclpyHospitalTransport)
    statuses = iter(
        (
            {"state": "IDLE", "pose": None, "odom_age": None, "feedback_source": None},
            {
                "state": "IDLE",
                "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "odom_age": 0.02,
                "feedback_source": "gazebo_diff_drive_odometry",
                "sim_time": 0.0,
            },
            {
                "state": "IDLE",
                "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "odom_age": 0.02,
                "feedback_source": "gazebo_diff_drive_odometry",
                "sim_time": 0.1,
            },
        )
    )
    status_calls = []
    trigger_calls = []

    def status(_self, _timeout):
        status_calls.append(True)
        return next(statuses)

    def trigger(_self, action, timeout):
        trigger_calls.append((action, timeout))
        return {"ok": True, "success": True, "message": "STARTED"}

    monkeypatch.setattr(RclpyHospitalTransport, "status", status)
    monkeypatch.setattr(RclpyHospitalTransport, "trigger", trigger)
    monkeypatch.setattr(RclpyHospitalTransport, "close", lambda _self: True)
    monkeypatch.setattr(
        HospitalLifecycleClient,
        "_run_fixed",
        lambda _self, suffix, *, timeout, generation=None: {
            "ok": True,
            "running": suffix[0] == "start",
        },
    )
    monkeypatch.setattr("agent_ros.adapters.hospital.time.sleep", lambda _seconds: None)
    client = HospitalLifecycleClient(transport)
    try:
        assert client.dispatch(HospitalAction.START)["state"] == "starting"
        assert client._start_receipt.done.wait(0.5)
        assert client._start_receipt.error is None
        assert len(status_calls) == 3
        assert trigger_calls == [(HospitalAction.START, 10.0)]
    finally:
        assert client.close(1.0)


def test_hospital_emergency_burst_uses_typed_estop_not_process_cleanup(monkeypatch):
    transport = object.__new__(RclpyHospitalTransport)
    typed_calls = []
    process_calls = []

    def trigger(_self, action, timeout):
        typed_calls.append((action, timeout))
        return {"ok": True, "success": True, "message": "ESTOPPED"}

    def fixed_call(_self, suffix, *, timeout, generation=None):
        process_calls.append(tuple(suffix))
        return {"ok": True, "running": False}

    monkeypatch.setattr(RclpyHospitalTransport, "trigger", trigger)
    monkeypatch.setattr(RclpyHospitalTransport, "close", lambda _self: True)
    monkeypatch.setattr(HospitalLifecycleClient, "_run_fixed", fixed_call)
    client = HospitalLifecycleClient(transport)
    try:
        client._latch_stop()

        client.stop_emergency()

        assert typed_calls == [(HospitalAction.STOP, 15.0)]
        assert process_calls == []
    finally:
        assert client.close(1.0)


def test_hospital_lifecycle_read_only_polling_does_not_consume_mutating_history(
    monkeypatch,
):
    start_entered = threading.Event()
    release_start = threading.Event()

    def fixed_call(self, suffix, *, timeout, generation=None):
        if suffix[0] == "start":
            start_entered.set()
            assert release_start.wait(1.0)
            return {"ok": True, "running": True}
        if suffix[0] == "mission-start":
            return {"ok": True, "success": True}
        if suffix[0] == "stop":
            return {"ok": True, "running": False}
        raise AssertionError(suffix)

    monkeypatch.setattr(HospitalLifecycleClient, "_run_fixed", fixed_call)
    client = HospitalLifecycleClient()
    client.dispatch(HospitalAction.START)
    assert start_entered.wait(0.2)

    for _ in range(400):
        assert client.dispatch(HospitalAction.STATUS)["state"] == "starting"
        assert client.dispatch(HospitalAction.PROBE)["available"] is True

    assert client.actions == (HospitalAction.START,)
    client._latch_stop()
    release_start.set()
    assert client._start_receipt.done.wait(0.5)
    assert client.close(1.0)


def test_hospital_lifecycle_mutating_history_full_remains_fail_closed(monkeypatch):
    stop_calls = 0

    def fixed_call(self, suffix, *, timeout, generation=None):
        nonlocal stop_calls
        if suffix[0] == "stop":
            stop_calls += 1
            return {"ok": True, "running": False}
        raise AssertionError(suffix)

    monkeypatch.setattr(HospitalLifecycleClient, "_run_fixed", fixed_call)
    client = HospitalLifecycleClient()
    for _ in range(64):
        assert client.dispatch(HospitalAction.VALIDATE)["state"] == "validated"

    with pytest.raises(AdapterError, match="INTERNAL_ERROR"):
        client.dispatch(HospitalAction.STOP)

    assert client.actions == (HospitalAction.VALIDATE,) * 64
    assert stop_calls == 0
    assert client.close(1.0)


def test_hospital_emergency_stop_interrupts_and_reaps_exact_inflight_start_process(
    monkeypatch,
):
    start_entered = threading.Event()
    start_released = threading.Event()
    start_killed = threading.Event()
    processes = []

    class FixedProcess:
        def __init__(self, suffix):
            self.suffix = suffix
            self.returncode = None
            self.pid = 42420 if suffix[0] == "start" else 42421
            processes.append(self)

        def communicate(self, timeout=None):
            if self.suffix[0] == "start":
                start_entered.set()
                assert start_released.wait(1.0)
                if start_killed.is_set():
                    self.returncode = -15
                    return ("", "")
                self.returncode = 0
                return ('{"ok":true,"running":true}', "")
            self.returncode = 0
            return ('{"ok":true,"running":false}', "")

        def poll(self):
            return self.returncode

        def terminate(self):
            assert self.suffix[0] == "start"
            start_killed.set()
            start_released.set()

        def kill(self):
            self.terminate()

    def fixed_popen(argv, **kwargs):
        assert kwargs["shell"] is False
        return FixedProcess(tuple(argv[2:]))

    monkeypatch.setattr(subprocess, "Popen", fixed_popen)
    monkeypatch.setattr("agent_ros.adapters.hospital.os.getpgid", lambda pid: pid)
    monkeypatch.setattr(
        "agent_ros.adapters.hospital.os.killpg",
        lambda pid, sig: next(process for process in processes if process.pid == pid).terminate(),
    )
    client = HospitalLifecycleClient()
    try:
        client.dispatch(HospitalAction.START)
        assert start_entered.wait(0.2)

        client._latch_stop()
        client.stop_emergency()

        assert start_killed.wait(0.2)
        assert client._start_receipt.done.wait(0.2)
        assert client._start_receipt.error is None
        assert not client._worker._failed
        assert not client._emergency_worker._failed
        assert client.close(0.5)
        assert not client._worker._thread.is_alive()
        assert not client._emergency_worker._thread.is_alive()
        assert [process.suffix[0] for process in processes].count("stop") >= 1
    finally:
        start_released.set()
        client.close(1.0)


@pytest.mark.parametrize("startup_phase", ["build_reserved", "launch_recorded"])
def test_hospital_initial_start_timeout_coordinates_inner_cleanup_before_outer_kill(monkeypatch, startup_phase):
    events = []
    inner_owned = [startup_phase == "launch_recorded"]
    reservation = [startup_phase == "build_reserved"]
    processes = []

    class FixedProcess:
        def __init__(self, suffix):
            self.suffix = suffix
            self.pid = 42500 if suffix[0] == "start" else 42501
            self.returncode = None
            self.communicate_calls = 0
            processes.append(self)

        def communicate(self, timeout=None):
            self.communicate_calls += 1
            if self.suffix[0] == "start" and self.communicate_calls == 1:
                events.append("start_timeout")
                raise subprocess.TimeoutExpired(self.suffix, timeout)
            if self.suffix[0] == "stop":
                events.append("fixed_stop")
                reservation[0] = False
                inner_owned[0] = False
                self.returncode = 0
                return ('{"ok":true,"running":false}', "")
            self.returncode = -9
            return ("", "")

        def poll(self):
            return self.returncode

        def kill(self):
            events.append("non_group_kill")
            self.returncode = -9

    def fixed_popen(argv, **kwargs):
        return FixedProcess(tuple(argv[2:]))

    def fixed_killpg(pid, sig):
        events.append("outer_group_kill")
        next(process for process in processes if process.pid == pid).returncode = -9

    monkeypatch.setattr(subprocess, "Popen", fixed_popen)
    monkeypatch.setattr("agent_ros.adapters.hospital.os.getpgid", lambda pid: pid)
    monkeypatch.setattr("agent_ros.adapters.hospital.os.killpg", fixed_killpg)
    client = HospitalLifecycleClient()
    client.dispatch(HospitalAction.START)
    assert client._start_receipt.done.wait(0.5)

    assert isinstance(client._start_receipt.error, AdapterError)
    assert client._start_receipt.error.code == "TIMEOUT"
    assert events.index("fixed_stop") < events.index("outer_group_kill")
    assert "non_group_kill" not in events
    assert reservation == [False]
    assert inner_owned == [False]
    assert HospitalAction.START in client.actions
    assert HospitalAction.STATUS not in client.actions
    assert client.close(0.5) is False
    assert not client._worker._thread.is_alive()
    assert not client._emergency_worker._thread.is_alive()


def test_hospital_initial_start_timeout_surfaces_coordinated_cleanup_failure(
    monkeypatch,
):
    events = []
    processes = []

    class FixedProcess:
        def __init__(self, suffix):
            self.suffix = suffix
            self.pid = 42600 if suffix[0] == "start" else 42601
            self.returncode = None
            self.calls = 0
            processes.append(self)

        def communicate(self, timeout=None):
            self.calls += 1
            if self.suffix[0] == "start" and self.calls == 1:
                raise subprocess.TimeoutExpired(self.suffix, timeout)
            if self.suffix[0] == "stop":
                events.append("fixed_stop_failed")
                self.returncode = 1
                return ('{"ok":false}', "")
            self.returncode = -9
            return ("", "")

        def poll(self):
            return self.returncode

        def kill(self):
            self.returncode = -9

    monkeypatch.setattr(subprocess, "Popen", lambda argv, **kwargs: FixedProcess(tuple(argv[2:])))
    monkeypatch.setattr("agent_ros.adapters.hospital.os.getpgid", lambda pid: pid)
    monkeypatch.setattr(
        "agent_ros.adapters.hospital.os.killpg",
        lambda pid, sig: (
            events.append("outer_group_kill"),
            setattr(next(p for p in processes if p.pid == pid), "returncode", -9),
        ),
    )
    client = HospitalLifecycleClient()
    client.dispatch(HospitalAction.START)
    assert client._start_receipt.done.wait(0.5)

    assert isinstance(client._start_receipt.error, AdapterError)
    assert client._start_receipt.error.code == "CLEANUP_FAILED"
    assert events == ["fixed_stop_failed", "outer_group_kill"]
    assert client.close(0.5) is False
    assert not client._worker._thread.is_alive()
    assert not client._emergency_worker._thread.is_alive()


def test_hospital_case_never_starts_mission_after_estop_wins_post_start_race(monkeypatch, adapter_owner):
    cancellation_checked = threading.Event()
    release_checked_start = threading.Event()
    spawned_suffixes = []
    real_check = HospitalLifecycleClient._start_was_cancelled

    def gated_check(self, generation):
        cancelled = real_check(self, generation)
        if not cancelled:
            cancellation_checked.set()
            assert release_checked_start.wait(1.0)
        return cancelled

    class FixedProcess:
        returncode = 0

        def communicate(self, timeout=None):
            return ('{"ok":true,"running":false}', "")

        def kill(self):
            raise AssertionError("fixed process must not time out")

    def fixed_popen(argv, **kwargs):
        assert kwargs["shell"] is False
        spawned_suffixes.append(tuple(argv[2:]))
        return FixedProcess()

    monkeypatch.setattr(HospitalLifecycleClient, "_start_was_cancelled", gated_check)
    monkeypatch.setattr(subprocess, "Popen", fixed_popen)
    client = HospitalLifecycleClient()
    adapter = HospitalCaseAdapter(client)
    adapter.start(HospitalAction.START, valid_permit(adapter, adapter_owner))
    assert cancellation_checked.wait(0.2)

    results = []
    stopper = threading.Thread(target=lambda: results.append(adapter._emergency_stop(timeout=0.5)))
    stopper.start()
    assert not release_checked_start.wait(0.05)
    release_checked_start.set()
    stopper.join(0.5)
    assert not stopper.is_alive()
    result = results[0]
    assert result.successful
    assert client._start_receipt.done.wait(0.5)

    assert ("mission-start",) not in spawned_suffixes
    assert ("stop",) in spawned_suffixes
    assert adapter.close(1.0)


def test_hospital_case_odometry_observation_projects_only_closed_pose_schema(
    monkeypatch,
):
    def fixed_call(self, suffix, *, timeout, generation=None):
        assert suffix == ("mission-status",)
        return {
            "ok": True,
            "status": {
                "state": "SUCCEEDED",
                "elapsed": 137.76,
                "stage_results": [{"id": "pharmacy", "elapsed": 37.975}],
                "pose": {"x": 0.25, "y": -0.5, "yaw": 1.25},
            },
        }

    monkeypatch.setattr(HospitalLifecycleClient, "_run_fixed", fixed_call)
    adapter = HospitalCaseAdapter(HospitalLifecycleClient(), clock=lambda: 42.0)

    observation = adapter.observe("odometry")

    assert observation.source == "odometry"
    assert observation.timestamp == 42.0
    assert dict(observation.values) == {"x": 0.25, "y": -0.5, "yaw": 1.25}
    assert adapter.close(1.0)


@pytest.mark.parametrize(
    "pose",
    [None, {}, {"x": 0.0, "y": 0.0}, {"x": float("nan"), "y": 0.0, "yaw": 0.0}],
)
def test_hospital_case_rejects_malformed_odometry_observation(monkeypatch, pose):
    def fixed_call(self, suffix, *, timeout, generation=None):
        return {"ok": True, "status": {"state": "RUNNING", "pose": pose}}

    monkeypatch.setattr(HospitalLifecycleClient, "_run_fixed", fixed_call)
    adapter = HospitalCaseAdapter(HospitalLifecycleClient())

    with pytest.raises(AdapterError, match="INTERNAL_ERROR"):
        adapter.observe("odometry")

    assert adapter.close(1.0)


def _capture(errors, function, *args):
    try:
        function(*args)
    except BaseException as exc:
        errors.append(exc)


def test_nav2_late_goal_acceptance_is_cancelled_without_returning_to_running(monkeypatch, adapter_owner):
    class Handle:
        accepted = True

        def __init__(self) -> None:
            self.cancel_count = 0
            self.cancel_future = CallbackFuture()
            self.result_future = CallbackFuture()

        def cancel_goal_async(self):
            self.cancel_count += 1
            return self.cancel_future

        def get_result_async(self):
            return self.result_future

    transport = real_nav2_transport(monkeypatch)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    transport.cancel_goal()
    handle = Handle()

    transport._client.goal_future.resolve(handle)

    assert handle.cancel_count == 1
    assert transport.goal_status() == {"state": "cancelling"}
    handle.cancel_future.resolve(type("Response", (), {"goals_canceling": [object()]})())
    assert transport.goal_status() == {"state": "cancelling"}
    handle.result_future.resolve(type("Result", (), {"status": 5})())
    assert transport.goal_status() == {"state": "cancelled"}


@pytest.mark.parametrize("mode", ["rejected", "exception"])
def test_nav2_cancel_rejection_or_exception_faults_instead_of_claiming_stopped(mode, monkeypatch, adapter_owner):
    class Future:
        def add_done_callback(self, callback):
            self.callback = callback

        def result(self):
            if mode == "exception":
                raise RuntimeError("raw")
            return type("Response", (), {"goals_canceling": []})()

    class Handle:
        accepted = True

        def __init__(self):
            self.future = Future()

        def cancel_goal_async(self):
            return self.future

        def get_result_async(self):
            return type("ResultFuture", (), {"add_done_callback": lambda self, cb: None})()

    transport = real_nav2_transport(monkeypatch)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    handle = Handle()
    transport._client.goal_future.resolve(handle)
    transport.cancel_goal()
    handle.future.callback(handle.future)

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_cancel_timeout_faults_while_confirmation_is_missing(monkeypatch, adapter_owner):
    now = [0.0]
    transport = real_nav2_transport(monkeypatch, clock=lambda: now[0], cancel_timeout=0.1)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    transport.cancel_goal()
    now[0] = 0.11

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_cancel_timeout_remains_active_after_cancel_acceptance_until_terminal_result(monkeypatch, adapter_owner):
    now = [0.0]

    class Handle:
        accepted = True

        def __init__(self):
            self.cancel_future = CallbackFuture()

        def cancel_goal_async(self):
            return self.cancel_future

        def get_result_async(self):
            return CallbackFuture()

    transport = real_nav2_transport(monkeypatch, clock=lambda: now[0], cancel_timeout=0.1)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    handle = Handle()
    transport._client.goal_future.resolve(handle)
    transport.cancel_goal()
    handle.cancel_future.resolve(type("Response", (), {"goals_canceling": [object()]})())
    now[0] = 0.11

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_status_exception_is_stable_and_does_not_recurse_through_normal_stop(adapter_owner):
    transport = Nav2Transport()
    transport.goal_status = lambda: (_ for _ in ()).throw(RuntimeError("raw"))
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    valid_permit(adapter, adapter_owner)

    with pytest.raises(AdapterError, match="STALE_FEEDBACK"):
        adapter.status()

    assert _wait_until(lambda: transport.zeros == 1)
    assert transport.zeros == 1


def test_nav2_cancelled_goal_result_exception_is_faulted_not_cancelled(monkeypatch, adapter_owner):
    class Handle:
        accepted = True

        def __init__(self):
            self.cancel_future = CallbackFuture()
            self.result_future = CallbackFuture()

        def cancel_goal_async(self):
            return self.cancel_future

        def get_result_async(self):
            return self.result_future

    transport = real_nav2_transport(monkeypatch)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    handle = Handle()
    transport._client.goal_future.resolve(handle)
    transport.cancel_goal()
    handle.result_future.reject(RuntimeError("raw callback"))

    assert transport.goal_status() == {"state": "faulted"}


def test_nav2_emergency_enqueue_never_waits_for_blocked_action_cancel(monkeypatch, adapter_owner):
    entered = threading.Event()
    release = threading.Event()

    class Handle:
        accepted = True

        def cancel_goal_async(self):
            entered.set()
            release.wait()
            return CallbackFuture()

        def get_result_async(self):
            return CallbackFuture()

    transport = real_nav2_transport(monkeypatch)
    adapter = Nav2Adapter(robot_profile("nav2"), transport)
    adapter.start(stage(), valid_permit(adapter, adapter_owner))
    transport._client.goal_future.resolve(Handle())
    errors = []
    worker = threading.Thread(target=lambda: _capture(errors, adapter._emergency_stop))
    worker.start()
    worker.join(0.05)
    blocked = worker.is_alive()
    release.set()
    worker.join(1.0)

    assert not blocked
    assert not errors


def test_safety_module_preserves_activation_reexports():
    assert ReExportedActivationPermit is _ActivationPermit
