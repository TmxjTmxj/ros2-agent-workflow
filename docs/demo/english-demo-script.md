# Two-Minute English Demo Script

## 0:00–0:15 — Positioning

“This repository is a reproducible Agent-to-ROS 2 workflow. The product boundary is
Profile to SafetyGateway to reviewed Adapter to independent evidence. Hospital delivery is
the complete validation example.”

## 0:15–0:35 — Installed control plane

Show `agent-ros --json status hospital-amr` and explain that it reads a packaged, reviewed
profile. State that this smoke command does not start ROS, Gazebo, or robot motion.

## 0:35–1:00 — Safety boundary

Show the MCP tool list or the source overview. Explain that clients request fixed task-level
operations; arbitrary shell commands, ROS topic payloads, paths, and PIDs are not exposed.
Mention fail-closed permits, heartbeats, emergency stop, and a single motion writer.

## 1:00–1:30 — Complete validation example

Show the headless hospital command and the resulting acceptance report with initial/final
camera screenshots. Explain that the independent monitor validates the ROS/Gazebo task;
the report is stronger evidence than a client log.

## 1:30–1:50 — Portability

Show `make docker-build`, `make docker-smoke`, and the adapter contract tests. Explain that
a new robot adds a reviewed adapter and profile while retaining the same safety/evidence
workflow.

## 1:50–2:00 — Honest close

“This is a reference workflow, not a universal SDK or a production hardware certification.
The repository provides the boundaries, tests, container, and evidence needed to evaluate a
new adapter responsibly.”
