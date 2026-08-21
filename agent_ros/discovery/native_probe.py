"""Fixed subprocess entry point for one bounded native ROS graph participant."""

from __future__ import annotations

import json

from agent_ros.discovery.ros_graph import _probe_native, _snapshot_document


def main() -> int:
    try:
        document = _snapshot_document(_probe_native())
        output = json.dumps(
            document,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
    except Exception:
        return 1
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
