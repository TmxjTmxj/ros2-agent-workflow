# ROS 2 Agent Workflow

[中文 README](README.md) · [Release checklist](docs/RELEASE.md) · [Runner guide](docs/RUNNER.md) · [Verification baseline](docs/VERIFICATION-BASELINE.md) · [Adapter migration](docs/ADAPTER-MIGRATION.md)

`ros2-agent-workflow` is a safe, reproducible Agent-to-ROS 2 control-plane reference.
It turns reviewed task intent into bounded ROS actions through a profile, a fail-closed
safety gateway, and a reviewed adapter.

```text
Profile → SafetyGateway → Adapter → independent evidence
```

The hospital delivery project is the complete validation demonstration of that workflow.
It is not the project's only product boundary, and it does not imply universal robot SDK
coverage or production hardware certification.

## See the workflow

![Standard Agent-to-ROS 2 workflow](assets/architecture.png)

The agent can request only reviewed task-level operations. The controller and independent
evidence chain are deliberately separate: a controller cannot declare its own success.

| Mission start | Mission completion |
| --- | --- |
| <img src="examples/hospital_delivery/evidence/acceptance-initial.png" alt="Independent camera evidence at hospital mission start" width="100%"> | <img src="examples/hospital_delivery/evidence/acceptance-final.png" alt="Independent camera evidence at hospital mission completion" width="100%"> |

These are the canonical PNGs referenced by the schema-2 acceptance report, not decorative
copies. The [JSON report](examples/hospital_delivery/evidence/acceptance_report.json),
[MCP trace](examples/hospital_delivery/evidence/mcp_agent_trace.json), and the two frames
form separate, reviewable evidence surfaces.

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
make docker-control-build
make docker-smoke
make docker-hospital-preflight
make docker-hospital
make docker-mcp-trace
```

`docker-control-build` creates the ROS-free image used by `docker-smoke`.
`docker-build` retains the full ROS/Gazebo hospital runtime.  The preflight
records host/container facts but cannot prove real-time factor; `docker-hospital`
writes the independent acceptance evidence under
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

The latest local v0.1.0 control-plane verification recorded **388 root tests passed** at
81% coverage, and the ROS-free container wheel/CLI/MCP smoke recorded 8 passed. The ROS
hospital suite is intentionally separate because it requires the system ROS Python; the
full hospital acceptance run remains the authority for mission success and runner RTF.
Use [the verification baseline](docs/VERIFICATION-BASELINE.md) for citation-safe résumé
or presentation wording, correction rationale, and non-claims.

### Why there is no 500+ test total

The historical 500+/505/509 figures were not the output of one command. They manually
combined changing suites from different points in time, Python interpreters, and ROS
dependency environments, then attempted to deduplicate them. That total was neither
directly reproducible nor clear about which tests required a ROS simulation environment.

The project now reports rerunnable boundaries instead: 388 root tests from `make check`,
150 ROS hospital-reference tests from `make test-hospital`, and 8 installed control-plane
smoke checks from `make docker-smoke`. This makes résumé and presentation claims verifiable
without later numerical corrections.

Fast CI checks source quality and an installed wheel. A scheduled/manual GitHub workflow
runs the headless hospital case on a Linux self-hosted runner labelled `ros-gazebo`, stages
its report, screenshots, logs, and MCP trace, validates the exported bundle read-only, and
retains it for 30 days. The runner must empirically meet the existing 300-second acceptance
budget; the workflow does not weaken that budget for CPU-only hosted runners.

Before publishing a tag, follow [the release checklist](docs/RELEASE.md). It produces a
GitHub Release only after source checks and installed-wheel verification have passed.
Use `make release-verify RELEASE_DIST=/tmp/agent-ros-release-candidate` to validate the
wheel, sdist, package metadata, and SHA-256 manifest before tagging.

## Demonstration

Use [the two-minute English demo script](docs/demo/english-demo-script.md) and
[the architecture one-pager](docs/demo/architecture-one-pager.md) for a concise walkthrough.
They describe the repository as a reproducible reference workflow, not as certified
production robot software.

## License

MIT. See [LICENSE](LICENSE).
