# ROS 2 Agent Workflow

[中文 README](README.md) · [Release checklist](docs/RELEASE.md) · [Adapter migration](docs/ADAPTER-MIGRATION.md)

`ros2-agent-workflow` is a safe, reproducible Agent-to-ROS 2 control-plane reference.
It turns reviewed task intent into bounded ROS actions through a profile, a fail-closed
safety gateway, and a reviewed adapter.

```text
Profile → SafetyGateway → Adapter → independent evidence
```

The hospital delivery project is the complete validation demonstration of that workflow.
It is not the project's only product boundary, and it does not imply universal robot SDK
coverage or production hardware certification.

## What is included

- Packaged robot/task profiles and schemas in the Python wheel.
- Installed `agent-ros` and `agent-ros-mcp` stdio entry points.
- Fail-closed activation, heartbeat, emergency-stop, audit, and single-writer controls.
- A reusable adapter safety/lifecycle contract suite.
- An Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Sim 10.x reference container.
- A headless hospital demonstration with independent acceptance report, screenshots, and
  a separate MCP tool trace.

## Safe quick verification

These commands exercise only the installed control plane; they do not start ROS, Gazebo,
or a motion-capable adapter.

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/agent-ros --json status hospital-amr
make check
```

For the locally verified reference environment, use Ubuntu 26.04 (Resolute), ROS 2
Lyrical, and Gazebo Sim 10.x:

```bash
make docker-build
make docker-smoke
make docker-hospital
make docker-mcp-trace
```

`docker-hospital` writes the independent acceptance evidence under
`examples/hospital_delivery/logs/`. `docker-mcp-trace` writes a separate control-plane
trace and never replaces the acceptance report.

## Architecture and boundaries

An MCP client can request fixed, task-level operations such as discovery, validation,
arming, task execution, observation, cancellation, and emergency stop. It cannot obtain
arbitrary shell access, arbitrary topic publishing, file paths, PIDs, or payloads through
this interface. The adapter maps those reviewed operations to a particular ROS graph.

The hospital adapter is deliberately closed around the hospital demonstration. To support a
new robot, keep the same profile-to-evidence workflow and implement a new reviewed adapter;
see [the migration guide](docs/ADAPTER-MIGRATION.md).

## Evidence and automation

Fast CI checks source quality and an installed wheel. A scheduled/manual GitHub workflow
runs the headless hospital case on a Linux self-hosted runner labelled `ros-gazebo`, stages
its report, screenshots, logs, and MCP trace, validates the exported bundle read-only, and
retains it for 30 days. The runner must empirically meet the existing 300-second acceptance
budget; the workflow does not weaken that budget for CPU-only hosted runners.

Before publishing a tag, follow [the release checklist](docs/RELEASE.md). It produces a
GitHub Release only after source checks and installed-wheel verification have passed.

## Demonstration

Use [the two-minute English demo script](docs/demo/english-demo-script.md) and
[the architecture one-pager](docs/demo/architecture-one-pager.md) for a concise walkthrough.
They describe the repository as a reproducible reference workflow, not as certified
production robot software.

## License

MIT. See [LICENSE](LICENSE).
