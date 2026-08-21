#!/usr/bin/env python3
"""Safe, machine-readable lifecycle CLI for the Codex ROS2 project."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any
import uuid


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PROJECT_ROOT.parents[1]
DEFAULT_RUNTIME_DIR = PROJECT_ROOT / ".runtime"
STATE_VERSION = 1
MANAGED_PROCESS_NAME = "hospital-delivery-launch"
MANAGED_LAUNCH_PREFIX = [
    "/usr/bin/python3",
    "/opt/ros/lyrical/bin/ros2",
    "launch",
    "smartcar_bringup",
    "hospital_delivery.launch.py",
]


class ProjectLifecycleError(RuntimeError):
    """Raised when a lifecycle operation cannot be completed safely."""


def _proc_identity(pid: int) -> tuple[int, list[str], int]:
    stat = (Path("/proc") / str(pid) / "stat").read_text()
    close_paren = stat.rfind(")")
    fields_after_comm = stat[close_paren + 2 :].split()
    start_time_ticks = int(fields_after_comm[19])
    raw_cmdline = (Path("/proc") / str(pid) / "cmdline").read_bytes()
    cmdline = [part.decode(errors="surrogateescape") for part in raw_cmdline.split(b"\0") if part]
    return start_time_ticks, cmdline, os.getpgid(pid)


def record_process(pid: int, name: str) -> dict[str, Any]:
    """Record enough immutable identity to reject PID reuse."""
    start_time_ticks, cmdline, pgid = _proc_identity(pid)
    return {
        "name": name,
        "pid": int(pid),
        "start_time_ticks": start_time_ticks,
        "cmdline": cmdline,
        "cwd": str((Path("/proc") / str(pid) / "cwd").resolve()),
        "pgid": pgid,
        "owns_process_group": pgid == pid,
    }


def process_matches(record: dict[str, Any]) -> bool:
    """Return true only when PID, start time, and exact argv still match."""
    try:
        pid = int(record["pid"])
        start_time_ticks, cmdline, pgid = _proc_identity(pid)
        cwd = str((Path("/proc") / str(pid) / "cwd").resolve())
    except (FileNotFoundError, ProcessLookupError, PermissionError, KeyError, ValueError):
        return False
    return (
        start_time_ticks == int(record.get("start_time_ticks", -1))
        and cmdline == list(record.get("cmdline", []))
        and cwd == record.get("cwd")
        and pgid == int(record.get("pgid", pgid))
    )


def _record_is_authorized_launch(record: dict[str, Any]) -> bool:
    cmdline = record.get("cmdline")
    return (
        record.get("name") == MANAGED_PROCESS_NAME
        and record.get("cwd") == str(PROJECT_ROOT)
        and record.get("owns_process_group") is True
        and int(record.get("pgid", -1)) == int(record.get("pid", -2))
        and isinstance(cmdline, list)
        and cmdline[: len(MANAGED_LAUNCH_PREFIX)] == MANAGED_LAUNCH_PREFIX
        and len(cmdline) == len(MANAGED_LAUNCH_PREFIX) + 1
        and cmdline[-1] in {"headless:=true", "headless:=false"}
    )


def _process_group_members(pgid: int) -> list[int]:
    members: list[int] = []
    for stat_path in Path("/proc").glob("[0-9]*/stat"):
        try:
            stat = stat_path.read_text()
            fields = stat[stat.rfind(")") + 2 :].split()
            if int(fields[2]) == pgid:
                members.append(int(stat_path.parent.name))
        except (OSError, ValueError, IndexError):
            continue
    return members


def terminate_managed_process(
    record: dict[str, Any],
    term_timeout: float = 8.0,
    *,
    require_authorized: bool = False,
) -> bool:
    """Terminate one verified process (or its verified owned process group)."""
    if not process_matches(record) or (
        require_authorized and not _record_is_authorized_launch(record)
    ):
        return False
    pid = int(record["pid"])
    owns_group = bool(record.get("owns_process_group")) and int(record.get("pgid", -1)) == pid

    def send(sig: signal.Signals) -> None:
        if owns_group:
            os.killpg(pid, sig)
        else:
            os.kill(pid, sig)

    try:
        send(signal.SIGTERM)
    except ProcessLookupError:
        return True
    deadline = time.monotonic() + term_timeout
    while time.monotonic() < deadline:
        if (owns_group and not _process_group_members(pid)) or (
            not owns_group and not process_matches(record)
        ):
            return True
        time.sleep(0.05)
    if (owns_group and _process_group_members(pid)) or (
        not owns_group and process_matches(record)
    ):
        try:
            send(signal.SIGKILL)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and (
        bool(_process_group_members(pid)) if owns_group else process_matches(record)
    ):
        time.sleep(0.05)
    return not _process_group_members(pid) if owns_group else not process_matches(record)


def terminate_spawned_launch(process: subprocess.Popen[Any], term_timeout: float = 8.0) -> bool:
    """Boundedly reap the exact fixed launch process group before identity publication."""
    try:
        pid = int(process.pid)
        if pid <= 0:
            return False
        leader_running = process.poll() is None
        members = _process_group_members(pid)
        if not leader_running and not members:
            return True
        if leader_running and os.getpgid(pid) != pid:
            return False
        os.killpg(pid, signal.SIGTERM)
        if leader_running:
            try:
                process.wait(timeout=max(0.0, term_timeout))
            except subprocess.TimeoutExpired:
                pass
        deadline = time.monotonic() + max(0.0, term_timeout)
        while _process_group_members(pid) and time.monotonic() < deadline:
            time.sleep(0.05)
        if _process_group_members(pid):
            os.killpg(pid, signal.SIGKILL)
            if process.poll() is None:
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    return False
            kill_deadline = time.monotonic() + 2.0
            while _process_group_members(pid) and time.monotonic() < kill_deadline:
                time.sleep(0.05)
        return process.poll() is not None and not _process_group_members(pid)
    except (OSError, subprocess.TimeoutExpired, ValueError, TypeError):
        return False


def load_runtime_state(state_path: str | Path) -> dict[str, Any]:
    """Load state and separate live identities from stale, untrusted records."""
    path = Path(state_path)
    if not path.exists():
        return {
            "version": STATE_VERSION,
            "processes": [],
            "stale_processes": [],
            "unresolved_process_groups": [],
            "startup_token": None,
        }
    try:
        payload = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ProjectLifecycleError(f"invalid runtime state {path}: {exc}") from exc
    startup_token = payload.get("startup_token")
    if (
        payload.get("version") != STATE_VERSION
        or not isinstance(payload.get("processes"), list)
        or (
            startup_token is not None
            and (not isinstance(startup_token, str) or not startup_token)
        )
    ):
        raise ProjectLifecycleError(f"unsupported runtime state format in {path}")
    live = []
    stale = []
    unresolved_groups = []
    for record in payload["processes"]:
        if process_matches(record):
            live.append(record)
            continue
        stale.append(record)
        if _record_is_authorized_launch(record):
            pgid = int(record["pgid"])
            members = sorted(_process_group_members(pgid))
            if members:
                unresolved_groups.append(
                    {"pgid": pgid, "members": members, "record": record}
                )
    return {
        "version": STATE_VERSION,
        "processes": live,
        "stale_processes": stale,
        "unresolved_process_groups": unresolved_groups,
        "startup_token": startup_token,
    }


def _write_state(
    path: Path,
    processes: list[dict[str, Any]],
    *,
    startup_token: str | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    state: dict[str, Any] = {"version": STATE_VERSION, "processes": processes}
    if startup_token is not None:
        state["startup_token"] = startup_token
    payload = json.dumps(state, indent=2)
    descriptor, temporary_name = tempfile.mkstemp(prefix="state-", suffix=".json", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        Path(temporary_name).replace(path)
    finally:
        Path(temporary_name).unlink(missing_ok=True)


def parse_cmd_vel_publisher_endpoints(output: str) -> list[dict[str, str]]:
    """Preserve every publisher endpoint identity, including duplicate node names."""
    result: list[dict[str, str]] = []
    current_name: str | None = None
    current_namespace = "/"
    endpoint_type: str | None = None
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("Node name:"):
            current_name = stripped.split(":", 1)[1].strip()
            current_namespace = "/"
            endpoint_type = None
        elif stripped.startswith("Node namespace:"):
            current_namespace = stripped.split(":", 1)[1].strip() or "/"
        elif stripped.startswith("Endpoint type:"):
            endpoint_type = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("GID:") and current_name and endpoint_type == "PUBLISHER":
            namespace = "/" + current_namespace.strip("/") if current_namespace.strip("/") else ""
            full_name = f"{namespace}/{current_name}".replace("//", "/")
            result.append({"node": full_name, "gid": stripped.split(":", 1)[1].strip()})
            current_name = None
    return result


def parse_cmd_vel_publishers(output: str) -> list[str]:
    result: list[str] = []
    for endpoint in parse_cmd_vel_publisher_endpoints(output):
        if endpoint["node"] not in result:
            result.append(endpoint["node"])
    return result


def _ros_environment(include_install: bool = False) -> dict[str, str]:
    setup_parts = ["source /opt/ros/lyrical/setup.bash"]
    if include_install:
        setup_parts.append(f"source {PROJECT_ROOT / 'install' / 'setup.bash'}")
    command = " && ".join(setup_parts + ["env -0"])
    result = subprocess.run(
        ["/bin/bash", "-lc", command], capture_output=True, check=True, timeout=10
    )
    environment = {}
    for item in result.stdout.split(b"\0"):
        if b"=" in item:
            key, value = item.split(b"=", 1)
            environment[key.decode()] = value.decode(errors="surrogateescape")
    environment["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        [
            str(PROJECT_ROOT / "models"),
            "/opt/ros/lyrical/share/turtlebot3_gazebo/models",
            environment.get("GZ_SIM_RESOURCE_PATH", ""),
        ]
    ).rstrip(os.pathsep)
    environment.setdefault("DISPLAY", ":99")
    return environment


def _run_ros(args: list[str], timeout: float = 10.0, include_install: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(
        args,
        cwd=PROJECT_ROOT,
        env=_ros_environment(include_install=include_install),
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def inspect_cmd_vel_publishers() -> dict[str, Any]:
    topics = _run_ros(["ros2", "topic", "list"], timeout=8, include_install=False)
    if topics.returncode != 0:
        return {"ok": False, "error": (topics.stderr or topics.stdout).strip(), "endpoints": []}
    if "/cmd_vel" not in topics.stdout.splitlines():
        return {"ok": True, "endpoints": []}
    result = _run_ros(
        ["ros2", "topic", "info", "/cmd_vel", "-v"],
        timeout=8,
        include_install=False,
    )
    if result.returncode != 0:
        return {"ok": False, "error": (result.stderr or result.stdout).strip(), "endpoints": []}
    return {"ok": True, "endpoints": parse_cmd_vel_publisher_endpoints(result.stdout)}


def _topic_publishers() -> list[str]:
    inspection = inspect_cmd_vel_publishers()
    if not inspection["ok"]:
        raise ProjectLifecycleError(f"cannot inspect /cmd_vel publishers: {inspection['error']}")
    return [endpoint["node"] for endpoint in inspection["endpoints"]]


def _require_exclusive_controller() -> dict[str, str]:
    inspection = inspect_cmd_vel_publishers()
    if not inspection["ok"]:
        raise ProjectLifecycleError(f"cannot inspect /cmd_vel publishers: {inspection['error']}")
    endpoints = inspection["endpoints"]
    if len(endpoints) != 1 or endpoints[0]["node"] != "/hospital_mission_controller":
        raise ProjectLifecycleError(
            "mission requires exactly one /cmd_vel publisher endpoint owned by "
            f"/hospital_mission_controller; observed={endpoints}"
        )
    return endpoints[0]


def _native_graph_snapshot(
    timeout: float, *, environment: dict[str, str] | None = None
) -> dict[str, Any]:
    """Collect one closed native ROS graph document through a killable helper."""
    helper_timeout = min(6.0, max(0.001, float(timeout)))
    argv = [
        str(REPOSITORY_ROOT / ".venv" / "bin" / "python"),
        "-m",
        "agent_ros.discovery.native_probe",
    ]
    environment = dict(
        _ros_environment(include_install=True)
        if environment is None
        else environment
    )
    environment["PYTHONNOUSERSITE"] = "1"
    try:
        result = subprocess.run(
            argv,
            cwd=REPOSITORY_ROOT,
            env=environment,
            capture_output=True,
            text=True,
            timeout=helper_timeout,
            shell=False,
        )
    except (OSError, subprocess.SubprocessError, ValueError):
        raise ProjectLifecycleError("native ROS graph probe failed") from None
    if result.returncode != 0:
        raise ProjectLifecycleError("native ROS graph probe failed")
    try:
        decoder = json.JSONDecoder()
        document, end = decoder.raw_decode(result.stdout.lstrip())
        if result.stdout.lstrip()[end:].strip():
            raise ValueError
    except (json.JSONDecodeError, TypeError, ValueError):
        raise ProjectLifecycleError("native ROS graph probe returned invalid evidence") from None
    required = {"nodes", "topics", "services", "actions", "topic_endpoints"}
    if not isinstance(document, dict) or set(document) != required:
        raise ProjectLifecycleError("native ROS graph probe returned invalid evidence")
    for key in ("topics", "services", "actions", "topic_endpoints"):
        if not isinstance(document[key], dict):
            raise ProjectLifecycleError("native ROS graph probe returned invalid evidence")
    if not isinstance(document["nodes"], list):
        raise ProjectLifecycleError("native ROS graph probe returned invalid evidence")
    return document


def _native_cmd_vel_publishers(document: dict[str, Any]) -> list[dict[str, str]]:
    raw = document["topic_endpoints"].get("/cmd_vel", [])
    if not isinstance(raw, list):
        raise ProjectLifecycleError("native ROS graph probe returned invalid evidence")
    publishers: list[dict[str, str]] = []
    required = {"node_name", "node_namespace", "gid", "endpoint_type"}
    for endpoint in raw:
        if (
            not isinstance(endpoint, dict)
            or set(endpoint) != required
            or not all(isinstance(endpoint[key], str) for key in required)
        ):
            raise ProjectLifecycleError("native ROS graph probe returned invalid evidence")
        if endpoint["endpoint_type"] != "publisher":
            continue
        namespace = endpoint["node_namespace"].rstrip("/")
        node = f"{namespace}/{endpoint['node_name']}" if namespace else f"/{endpoint['node_name']}"
        if not endpoint["node_name"] or not endpoint["gid"]:
            raise ProjectLifecycleError("native ROS graph probe returned invalid evidence")
        publishers.append({"node": node, "gid": endpoint["gid"]})
    return publishers


def _wait_for_graph(timeout: float) -> None:
    required_topics = {
        "/odom",
        "/scan",
        "/camera/image_raw",
        "/hospital_amr/contacts",
        "/hospital_mission/status",
    }
    required_services = {
        "/hospital_mission/start",
        "/hospital_mission/cancel",
        "/hospital_mission/estop",
        "/hospital_mission/reset",
    }
    deadline = time.monotonic() + timeout
    environment = _ros_environment(include_install=True)
    missing_topics = set(required_topics)
    missing_services = set(required_services)
    last_probe_error: ProjectLifecycleError | None = None
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            graph = _native_graph_snapshot(remaining, environment=environment)
        except ProjectLifecycleError as exc:
            last_probe_error = exc
            if time.monotonic() >= deadline:
                break
            time.sleep(min(0.5, max(0.0, deadline - time.monotonic())))
            continue
        topics = graph["topics"]
        services = graph["services"]
        missing_topics = required_topics.difference(topics)
        missing_services = required_services.difference(services)
        if not missing_topics and not missing_services:
            endpoints = _native_cmd_vel_publishers(graph)
            if not endpoints:
                time.sleep(min(0.5, max(0.0, deadline - time.monotonic())))
                continue
            if len(endpoints) != 1 or endpoints[0]["node"] != "/hospital_mission_controller":
                raise ProjectLifecycleError(
                    "mission requires exactly one /cmd_vel publisher endpoint owned by "
                    f"/hospital_mission_controller; observed={endpoints}"
                )
            return
        time.sleep(min(0.5, max(0.0, deadline - time.monotonic())))
    if last_probe_error is not None:
        raise ProjectLifecycleError("startup timed out; native ROS graph unavailable") from last_probe_error
    raise ProjectLifecycleError(
        f"startup timed out; missing topics={sorted(missing_topics)}, "
        f"missing services={sorted(missing_services)}"
    )


def _service_call(service_name: str, timeout: float = 10.0) -> dict[str, Any]:
    result = _run_ros(
        ["ros2", "service", "call", service_name, "std_srvs/srv/Trigger", "{}"],
        timeout=timeout,
    )
    combined = (result.stdout + "\n" + result.stderr).strip()
    success = "success=True" in combined or "success: true" in combined.lower()
    message = ""
    for marker in ("message=", "message:"):
        if marker in combined:
            message = combined.rsplit(marker, 1)[1].strip().strip("')\"")
            break
    return {
        "ok": result.returncode == 0 and success,
        "service": service_name,
        "success": success,
        "message": message,
        "raw": combined,
    }


def _mission_status(timeout: float = 12.0) -> dict[str, Any]:
    result = _run_ros(
        ["ros2", "topic", "echo", "/hospital_mission/status", "--once", "--field", "data"],
        timeout=timeout,
    )
    if result.returncode != 0:
        return {"ok": False, "error": (result.stderr or result.stdout).strip()}
    raw = result.stdout.strip()
    if raw.endswith("---"):
        raw = raw[:-3].rstrip()
    try:
        return {"ok": True, "status": json.loads(raw)}
    except json.JSONDecodeError:
        return {"ok": False, "error": "status topic did not contain JSON", "raw": raw}


class ProjectLifecycle:
    def __init__(self, runtime_dir: str | Path):
        self.runtime_dir = Path(runtime_dir).resolve()
        self.state_path = self.runtime_dir / "state.json"
        self.log_path = self.runtime_dir / "launch.log"
        self.lock_path = self.runtime_dir / "lifecycle.lock"
        self.startup_path = self.runtime_dir / "startup.json"
        self.startup_failure_path = self.runtime_dir / "startup-cleanup-failed.json"

    @contextmanager
    def _locked(self):
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        with self.lock_path.open("a+") as handle:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)

    def _load_startup(self) -> dict[str, Any] | None:
        if not self.startup_path.exists():
            return None
        try:
            payload = json.loads(self.startup_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise ProjectLifecycleError(f"invalid startup reservation: {exc}") from exc
        if (
            not isinstance(payload, dict)
            or not isinstance(payload.get("token"), str)
            or not payload["token"]
            or type(payload.get("cancelled")) is not bool
            or payload.get("phase") not in {"building", "handoff"}
        ):
            raise ProjectLifecycleError("invalid startup reservation")
        unresolved = payload.get("unresolved_launch")
        if unresolved is not None and (
            not isinstance(unresolved, dict)
            or isinstance(unresolved.get("pid"), bool)
            or not isinstance(unresolved.get("pid"), int)
            or unresolved["pid"] <= 0
            or (
                "record" in unresolved
                and not isinstance(unresolved["record"], dict)
            )
        ):
            raise ProjectLifecycleError("invalid startup reservation")
        return payload

    def _write_startup(self, payload: dict[str, Any]) -> None:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix="startup-", suffix=".json", dir=self.runtime_dir
        )
        try:
            with os.fdopen(descriptor, "w") as handle:
                json.dump(payload, handle, allow_nan=False)
                handle.flush()
                os.fsync(handle.fileno())
            Path(temporary_name).replace(self.startup_path)
        finally:
            Path(temporary_name).unlink(missing_ok=True)

    def _load_startup_failure(self) -> dict[str, int] | None:
        if not self.startup_failure_path.exists():
            return None
        try:
            payload = json.loads(self.startup_failure_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise ProjectLifecycleError(f"invalid startup cleanup marker: {exc}") from exc
        pid = payload.get("pid") if isinstance(payload, dict) else None
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
            raise ProjectLifecycleError("invalid startup cleanup marker")
        return {"pid": pid}

    def _write_startup_failure(self, pid: int) -> None:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix="startup-cleanup-", suffix=".json", dir=self.runtime_dir
        )
        try:
            with os.fdopen(descriptor, "w") as handle:
                json.dump({"pid": int(pid)}, handle, allow_nan=False)
                handle.flush()
                os.fsync(handle.fileno())
            Path(temporary_name).replace(self.startup_failure_path)
        finally:
            Path(temporary_name).unlink(missing_ok=True)

    def status(self) -> dict[str, Any]:
        with self._locked():
            state = load_runtime_state(self.state_path)
            startup = self._load_startup()
            startup_failure = self._load_startup_failure()
        unresolved_groups = [
            {"pgid": item["pgid"], "members": item["members"]}
            for item in state["unresolved_process_groups"]
        ]
        unresolved_startup = (
            startup.get("unresolved_launch")
            if startup is not None
            else None
        ) or startup_failure
        if unresolved_startup is not None:
            pid = int(unresolved_startup["pid"])
            unresolved_groups.append(
                {"pgid": pid, "members": sorted(_process_group_members(pid))}
            )
        return {
            "ok": True,
            "running": bool(
                state["processes"]
                or state["unresolved_process_groups"]
                or startup is not None
                or startup_failure is not None
            ),
            "managed_processes": state["processes"],
            "stale_processes": state["stale_processes"],
            "unresolved_process_groups": unresolved_groups,
        }

    def start(self, headless: bool = True, timeout: float = 45.0) -> dict[str, Any]:
        token = uuid.uuid4().hex
        preserve_startup_reservation = False
        with self._locked():
            state = load_runtime_state(self.state_path)
            if state["processes"]:
                raise ProjectLifecycleError("project is already running")
            if self._load_startup() is not None:
                raise ProjectLifecycleError("project startup is already reserved")
            if self._load_startup_failure() is not None:
                raise ProjectLifecycleError("project startup cleanup is unresolved")
            if state["unresolved_process_groups"]:
                groups = [
                    {"pgid": item["pgid"], "members": item["members"]}
                    for item in state["unresolved_process_groups"]
                ]
                raise ProjectLifecycleError(
                    f"unresolved process group remains; refusing startup: {groups}"
                )
            self._write_startup(
                {"token": token, "cancelled": False, "phase": "building"}
            )
        try:
            inspection = inspect_cmd_vel_publishers()
            if not inspection["ok"]:
                raise ProjectLifecycleError(
                    f"cannot inspect /cmd_vel publishers: {inspection['error']}"
                )
            if inspection["endpoints"]:
                raise ProjectLifecycleError(
                    "refusing startup because /cmd_vel already has publisher(s): "
                    + json.dumps(inspection["endpoints"], ensure_ascii=False)
                )
            build = subprocess.run(
                ["colcon", "build", "--packages-select", "smartcar_bringup", "--symlink-install"],
                cwd=PROJECT_ROOT,
                env=_ros_environment(include_install=False),
                capture_output=True,
                text=True,
                timeout=120,
            )
            if build.returncode != 0:
                raise ProjectLifecycleError(
                    f"colcon build failed:\n{build.stdout}\n{build.stderr}"
                )
            with self._locked():
                startup = self._load_startup()
                if (
                    startup is None
                    or startup["token"] != token
                    or startup["cancelled"] is True
                ):
                    raise ProjectLifecycleError("project startup was cancelled")
                self._write_startup(
                    {"token": token, "cancelled": False, "phase": "handoff"}
                )
                log_handle = self.log_path.open("a", buffering=1)
                command = [
                    "ros2", "launch", "smartcar_bringup", "hospital_delivery.launch.py",
                    f"headless:={'true' if headless else 'false'}",
                ]
                try:
                    process = subprocess.Popen(
                        command,
                        cwd=PROJECT_ROOT,
                        env=_ros_environment(include_install=True),
                        stdout=log_handle,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                    try:
                        record = record_process(process.pid, "hospital-delivery-launch")
                        _write_state(
                            self.state_path, [record], startup_token=token
                        )
                    except Exception as handoff_error:
                        if not terminate_spawned_launch(process):
                            preserve_startup_reservation = True
                            unresolved: dict[str, Any] = {"pid": int(process.pid)}
                            if "record" in locals():
                                unresolved["record"] = record
                            try:
                                self._write_startup_failure(int(process.pid))
                            except Exception:
                                pass
                            try:
                                self._write_startup(
                                    {
                                        "token": token,
                                        "cancelled": True,
                                        "phase": "handoff",
                                        "unresolved_launch": unresolved,
                                    }
                                )
                            except Exception:
                                pass
                            raise ProjectLifecycleError(
                                "startup handoff failed; cleanup incomplete"
                            ) from handoff_error
                        raise
                    else:
                        self.startup_path.unlink(missing_ok=True)
                finally:
                    log_handle.close()
            try:
                _wait_for_graph(timeout)
            except Exception as startup_error:
                terminated = terminate_managed_process(record, require_authorized=True)
                if (
                    not terminated
                    and not _process_group_members(int(record["pgid"]))
                ):
                    terminated = True
                with self._locked():
                    current = load_runtime_state(self.state_path)
                    current_startup = self._load_startup()
                    retained = list(current["processes"])
                    for item in current["unresolved_process_groups"]:
                        if item["record"] not in retained:
                            retained.append(item["record"])
                    if not terminated and record not in retained:
                        retained.append(record)
                    retained_token = (
                        current["startup_token"]
                        if current_startup is not None
                        and current["startup_token"] == current_startup["token"]
                        else None
                    )
                    _write_state(
                        self.state_path,
                        retained,
                        startup_token=retained_token,
                    )
                if not terminated:
                    raise ProjectLifecycleError(
                        f"startup failed ({startup_error}); cleanup incomplete and identity retained"
                    ) from startup_error
                raise
            return {
                "ok": True,
                "running": True,
                "process": record,
                "log": str(self.log_path),
            }
        finally:
            with self._locked():
                startup = self._load_startup()
                if (
                    startup is not None
                    and startup["token"] == token
                    and startup.get("unresolved_launch") is None
                    and not preserve_startup_reservation
                ):
                    self.startup_path.unlink(missing_ok=True)

    def stop(self) -> dict[str, Any]:
        with self._locked():
            startup = self._load_startup()
            startup_failure = self._load_startup_failure()
            startup_unresolved = (
                startup is not None
                and (
                    startup.get("unresolved_launch") is not None
                    or startup.get("phase") == "handoff"
                )
            ) or startup_failure is not None
            if startup is not None and not startup_unresolved:
                self._write_startup(
                    {
                        "token": startup["token"],
                        "cancelled": True,
                        "phase": startup["phase"],
                    }
                )
            state = load_runtime_state(self.state_path)
            state_authority = [
                *state["processes"],
                *state["stale_processes"],
                *(
                    item["record"]
                    for item in state["unresolved_process_groups"]
                ),
            ]
            phase_only_committed = bool(
                startup_unresolved
                and startup is not None
                and startup.get("phase") == "handoff"
                and startup.get("unresolved_launch") is None
                and startup_failure is None
                and state.get("startup_token") == startup.get("token")
                and state_authority
                and all(
                    _record_is_authorized_launch(record)
                    for record in state_authority
                )
            )

            startup_unresolved_payload: list[dict[str, Any]] = []
            if startup_unresolved:
                unresolved_launch = (
                    startup.get("unresolved_launch")
                    if startup is not None
                    else None
                ) or startup_failure
                if unresolved_launch is not None:
                    unresolved_pid = int(unresolved_launch["pid"])
                    members = sorted(_process_group_members(unresolved_pid))
                    unresolved_record = unresolved_launch.get("record")
                    if (
                        members
                        and isinstance(unresolved_record, dict)
                        and _record_is_authorized_launch(unresolved_record)
                        and process_matches(unresolved_record)
                        and terminate_managed_process(
                            unresolved_record, require_authorized=True
                        )
                    ):
                        members = sorted(_process_group_members(unresolved_pid))
                    if members:
                        startup_unresolved_payload.append(
                            {"pgid": unresolved_pid, "members": members}
                        )
                    else:
                        self.startup_path.unlink(missing_ok=True)
                        self.startup_failure_path.unlink(missing_ok=True)
                        startup_unresolved = False

            if state["processes"]:
                try:
                    _service_call("/hospital_mission/estop", timeout=4)
                except (subprocess.TimeoutExpired, OSError):
                    pass
            results = []
            for record in state["processes"]:
                results.append(
                    {
                        "pid": record["pid"],
                        "terminated": terminate_managed_process(
                            record, require_authorized=True
                        ),
                    }
                )
            remaining = [
                record
                for record, result in zip(state["processes"], results)
                if not result["terminated"]
            ]
            unresolved_groups = state["unresolved_process_groups"]
            for item in unresolved_groups:
                if item["record"] not in remaining:
                    remaining.append(item["record"])
            if (
                phase_only_committed
                and all(item["terminated"] for item in results)
                and not unresolved_groups
                and not remaining
            ):
                self.startup_path.unlink(missing_ok=True)
                self.startup_failure_path.unlink(missing_ok=True)
                startup_unresolved = False
            retained_startup_token = (
                startup["token"]
                if phase_only_committed and startup_unresolved and startup is not None
                else None
            )
            _write_state(
                self.state_path,
                remaining,
                startup_token=retained_startup_token,
            )
            if not startup_unresolved:
                self.startup_path.unlink(missing_ok=True)
                self.startup_failure_path.unlink(missing_ok=True)
            unresolved_payload = [
                {"pgid": item["pgid"], "members": item["members"]}
                for item in unresolved_groups
            ]
            unresolved_payload.extend(startup_unresolved_payload)
        return {
            "ok": (
                all(item["terminated"] for item in results)
                and not unresolved_groups
                and not startup_unresolved
            ),
            "running": bool(remaining or startup_unresolved),
            "terminated": results,
            "stale_processes": state["stale_processes"],
            "unresolved_process_groups": unresolved_payload,
        }


def _emit(payload: dict[str, Any]) -> int:
    print(json.dumps(payload, ensure_ascii=False))
    return 0 if payload.get("ok") else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-dir", default=str(DEFAULT_RUNTIME_DIR))
    subparsers = parser.add_subparsers(dest="command", required=True)
    start_parser = subparsers.add_parser("start")
    start_parser.add_argument("--gui", action="store_true")
    start_parser.add_argument("--timeout", type=float, default=45.0)
    subparsers.add_parser("status")
    subparsers.add_parser("stop")
    subparsers.add_parser("mission-start")
    subparsers.add_parser("mission-status")
    subparsers.add_parser("mission-cancel")
    subparsers.add_parser("estop")
    subparsers.add_parser("reset")
    camera_parser = subparsers.add_parser("camera")
    camera_parser.add_argument("--output")
    camera_parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args(argv)
    lifecycle = ProjectLifecycle(args.runtime_dir)
    try:
        if args.command == "status":
            return _emit(lifecycle.status())
        if args.command == "start":
            return _emit(lifecycle.start(headless=not args.gui, timeout=args.timeout))
        if args.command == "stop":
            return _emit(lifecycle.stop())
        if args.command == "mission-start":
            _require_exclusive_controller()
            return _emit(_service_call("/hospital_mission/start"))
        if args.command == "mission-status":
            return _emit(_mission_status())
        if args.command == "mission-cancel":
            return _emit(_service_call("/hospital_mission/cancel"))
        if args.command == "estop":
            return _emit(_service_call("/hospital_mission/estop"))
        if args.command == "reset":
            return _emit(_service_call("/hospital_mission/reset"))
        if args.command == "camera":
            output = args.output or str(
                PROJECT_ROOT / "logs" / f"camera-{int(time.time())}.png"
            )
            result = _run_ros(
                [
                    sys.executable,
                    str(PROJECT_ROOT / "scripts" / "capture_camera.py"),
                    "--output",
                    output,
                    "--timeout",
                    str(args.timeout),
                ],
                timeout=args.timeout + 5.0,
            )
            if result.returncode != 0:
                raise ProjectLifecycleError((result.stderr or result.stdout).strip())
            return _emit(json.loads(result.stdout))
    except (ProjectLifecycleError, subprocess.TimeoutExpired, OSError, RuntimeError) as exc:
        return _emit({"ok": False, "error": str(exc), "command": args.command})
    return _emit({"ok": False, "error": "unhandled command"})


if __name__ == "__main__":
    sys.exit(main())
