# Architecture One-Pager

## Purpose

Provide a reusable, reviewed path from an Agent's task-level intent to ROS 2 execution with
an independent evidence trail.

```text
MCP client
    │ fixed task-level request
    ▼
Profile + schema ──► SafetyGateway ──► reviewed Adapter ──► ROS 2 / simulator
                         │                    │                     │
                         └── permits, audit, E-stop ──► terminal evidence
```

## Control-plane invariants

- Profiles, task names, adapter selection, and authority are reviewed and schema-validated.
- The safety gateway fails closed, requires permits/heartbeats, and latches emergency stop.
- Adapters expose bounded lifecycle operations and one owner for motion/process control.
- Evidence is typed and terminal; a client trace does not replace independent acceptance.

## Demonstration environment

The hospital delivery case exercises the full workflow on the locally verified reference
environment: Ubuntu 26.04, ROS 2 Lyrical, and Gazebo Sim 10.x. It produces a schema-2
acceptance report plus initial/final camera PNGs. A separate FastMCP stdio run produces the
tool trace.

## Evaluation path

1. Run `make check` and `make smoke-wheel` for the portable Python control plane.
2. Run `make docker-build` and `make docker-smoke` for the reference container.
3. Run `make docker-hospital` for the complete demonstration evidence.
4. Run `make docker-mcp-trace` for the separate production control-plane trace.

## Scope

This repository is a workflow reference and demonstration, not a universal robot SDK or a
production safety certification. Hardware deployment needs robot-specific integration,
hazard analysis, and operational validation.
