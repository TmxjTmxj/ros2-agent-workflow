"""Local operator CLI; it deliberately never exposes a challenge creation API to MCP."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from agent_ros.discovery.inference import infer_capabilities
from agent_ros.discovery.ros_graph import RosGraphProbe
from agent_ros.errors import DiscoveryError, ProfileValidationError
from agent_ros.profiles.defaults import default_profiles_root
from agent_ros.profiles.loader import load_robot_profile
from agent_ros.safety.challenge import ChallengeError, create_operator_challenge
from agent_ros.safety.gateway import SafetyError, SafetyGateway


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.command == "hardware-challenge" and args.json:
        return _emit_error(True)
    try:
        if args.command == "hardware-challenge":
            _run_hardware_challenge(args)
            print("OK")
            return 0
        result = _run(args)
    except (ChallengeError, DiscoveryError, ProfileValidationError, SafetyError, OSError, ValueError):
        return _emit_error(args.json)
    if args.json:
        print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    else:
        print("OK")
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="agent-ros")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--profiles-root", type=Path, default=default_profiles_root())
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("discover", "validate", "status", "verify-profile"):
        item = subparsers.add_parser(command)
        item.add_argument("profile")
    challenge = subparsers.add_parser("hardware-challenge")
    challenge.add_argument("profile")
    challenge.add_argument("--runtime-dir", type=Path, required=True)
    return parser


def _run(args: argparse.Namespace) -> dict[str, object]:
    profile = load_robot_profile(args.profile, args.profiles_root)
    if args.command == "verify-profile":
        return {"profile": profile.name, "verified": True}
    if args.command == "status":
        return {"profile": profile.name, "state": "NEW"}
    report = infer_capabilities(RosGraphProbe().probe())
    if args.command == "discover":
        return {
            "profile": profile.name,
            "capabilities": list(report.capability_names),
            "blocking_warnings": list(report.blocking_warnings),
        }
    gateway = SafetyGateway(profile)
    gateway.discover(report)
    gateway.validate()
    return {"profile": profile.name, "state": gateway.state.value}


def _run_hardware_challenge(args: argparse.Namespace) -> None:
    """Creation requires the controlling terminal and never returns a token to a caller."""
    profile = load_robot_profile(args.profile, args.profiles_root)
    if profile.mode != "hardware" or not _is_interactive_terminal():
        raise ValueError("operator terminal required")
    terminal = _open_operator_terminal()
    try:
        terminal.write(f"Create one challenge for {profile.name}. Type the profile name to confirm: ")
        terminal.flush()
        if terminal.readline().strip() != profile.name:
            raise ValueError("operator confirmation required")
        token = create_operator_challenge(profile.name, args.runtime_dir)
        terminal.write(f"\nHardware challenge for {profile.name}: {token}\n")
        terminal.flush()
    finally:
        terminal.close()


def _is_interactive_terminal() -> bool:
    return sys.stdin.isatty() and sys.stdout.isatty()


def _open_operator_terminal():
    return open("/dev/tty", "r+", encoding="utf-8")


def _emit_error(as_json: bool) -> int:
    if as_json:
        print('{"error":"REQUEST_REJECTED"}')
    else:
        print("ERROR: REQUEST_REJECTED")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
