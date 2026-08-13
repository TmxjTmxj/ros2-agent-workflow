"""Local operator CLI; it deliberately never exposes a challenge creation API to MCP."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from agent_ros.discovery.inference import infer_capabilities
from agent_ros.discovery.ros_graph import RosGraphProbe
from agent_ros.errors import DiscoveryError, ProfileValidationError
from agent_ros.profiles.loader import load_robot_profile
from agent_ros.safety.challenge import ChallengeError, create_operator_challenge
from agent_ros.safety.gateway import SafetyError, SafetyGateway


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
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
    parser.add_argument("--profiles-root", type=Path, default=Path("profiles"))
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
    if args.command == "hardware-challenge":
        if profile.mode != "hardware":
            raise ValueError("hardware profile required")
        return {"profile": profile.name, "challenge": create_operator_challenge(profile.name, args.runtime_dir)}
    if args.command == "status":
        return {"profile": profile.name, "state": "NEW"}
    report = infer_capabilities(RosGraphProbe().probe())
    if args.command == "discover":
        return {"profile": profile.name, "capabilities": list(report.capability_names), "blocking_warnings": list(report.blocking_warnings)}
    gateway = SafetyGateway(profile)
    gateway.discover(report)
    gateway.validate()
    return {"profile": profile.name, "state": gateway.state.value}


def _emit_error(as_json: bool) -> int:
    if as_json:
        print('{"error":"REQUEST_REJECTED"}')
    else:
        print("ERROR: REQUEST_REJECTED")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
