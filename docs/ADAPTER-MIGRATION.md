# Adapter Migration Guide

This project is a reference workflow for reviewed Agent-to-ROS 2 automation. Adding a robot
means adding a bounded adapter implementation, not exposing its ROS graph directly to an
LLM or MCP client.

## Preserve the workflow boundary

```text
Reviewed Profile → SafetyGateway permit → Adapter → terminal evidence
```

1. Define the robot and task in a reviewed Profile and schema.
2. Implement a named adapter with fixed lifecycle, probe, start, cancel, stop, observe, and
   close behavior.
3. Keep motion authorization in `SafetyGateway`; the adapter must reject unauthorized starts.
4. Return typed terminal evidence and map native failures to reviewed error codes.
5. Run the reusable adapter contract suite before adding simulator or hardware integration.

## Adapter responsibilities

An adapter owns only the reviewed action surface. It must use bounded timeouts, stop safely
on cancellation or emergency stop, avoid arbitrary command construction, and reap only
processes it created and can verify. It should not accept topic names, shell fragments,
paths, or arbitrary payloads supplied by a client.

Use the hospital adapter as a complete simulation reference. Its process ownership and
terminal-evidence helpers are separated so that a new adapter can reuse the workflow shape
without inheriting hospital route or mission semantics.

## Contract tests

`tests/contracts/adapter_contract.py` verifies the framework-level expectations:

- unauthorized start is rejected;
- a valid permit allows a bounded task start;
- terminal results map to typed observations/evidence;
- cancellation and stop are idempotent;
- emergency stop latches and rejects stale permits;
- `close()` is bounded and leaves no owned process running.

Run the root contract tests without ROS or a graphical display:

```bash
.venv/bin/python -m pytest tests/test_adapter_contract.py -q
```

Then add a robot-specific integration test in its native environment. The integration test
must produce independent acceptance evidence; an MCP trace alone is not sufficient.

## Non-claims

This guide does not provide a universal SDK, certification argument, or a substitute for
robot-specific hazard analysis, operational limits, or on-site safety validation. Real
robot work starts with [REAL-ROBOT.md](REAL-ROBOT.md).
