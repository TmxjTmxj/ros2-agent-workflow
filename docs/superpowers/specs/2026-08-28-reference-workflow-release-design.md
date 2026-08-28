# Reference Workflow Release Design

## Purpose

Make `ros2-agent-workflow` easy to install, verify, maintain, and present
without changing the semantics of its validated standard Agent-to-ROS 2
workflow. The project remains a complete ROS 2 Agent workflow reference
implementation, not a general-purpose robot SDK.

## Scope and Positioning

The standard workflow is the product boundary:

```text
MCP Agent -> declarative Profile -> SafetyGateway -> reviewed Adapter
          -> ROS 2 runtime -> independent acceptance evidence
```

The hospital-delivery workflow is the complete, reproducible golden example of
that standard workflow. `TwistAdapter` and `Nav2Adapter` remain supported
reference adapter paths, but do not gain a promise of full automatic hardware
integration in this release. The project supports Ubuntu 24.04 with ROS 2
Lyrical and Gazebo Sim 10 as the reproducible example environment. Other
platforms are explanatory guidance, not release compatibility claims.

## M1: Installable Control Plane

- Keep the hospital Gazebo world, model, evidence, and demo scripts
  source-owned in the repository; they are not PyPI package data.
- Package the control-plane-owned profiles and JSON schemas with the wheel.
- Expose `agent-ros` for the local operator CLI and `agent-ros-mcp` for the
  stdio MCP server.
- Let CLI and MCP server resolve packaged profiles by default while retaining
  an explicit source/repository profile root override.
- Add a `dev` optional dependency group containing development and verification
  tools.
- Repair stdio handshake tests so they execute with the active interpreter,
  never a hard-coded repository `.venv`.
- Add a wheel smoke test that builds the distribution, installs it in an empty
  virtual environment, invokes both entry points outside the repository, and
  verifies profile and MCP startup behavior without ROS/Gazebo.

## M2: Quality and Safety Boundaries

- Add Ruff, mypy, pytest-cov, pip-audit, and pre-commit as reproducible dev
  tooling.
- Gate pull requests on format/lint, type checking, unit/static tests, wheel
  smoke tests, coverage, and dependency auditing.
- Split only code whose responsibilities are already separable. Preserve public
  adapter and controller interfaces.
- Add a reusable adapter contract suite covering authorization, cancellation,
  emergency stop, fail-closed behavior, lifecycle idempotence, cleanup, and
  evidence shape. Hospital must pass it; other adapters opt in only where their
  current execution surface supports the contract.

## M3: Distribution and Demonstration

- Add a concise English README that links to the full Chinese reference
  documentation.
- Supply a Dockerfile and Dev Container for the official headless reference
  environment.
- Run the full hospital workflow in a scheduled/manual ROS/Gazebo CI job;
  archive its acceptance report, MCP trace, screenshots, and logs.
- Add release automation and release documentation for version `0.1.0`.
- Add a two-minute English demo script, architecture one-pager source, and a
  reproducible instructions page. Media recording itself remains a manual
  human action; repository deliverables provide everything needed to produce
  it.

## Error Handling and Safety

- Packaging failures must fail CI before release creation.
- Runtime smoke tests must never arm hardware or publish motion.
- Container and nightly jobs run the hospital demonstration in headless
  simulation only.
- New generalization documentation must not claim that hardware adapters are
  production-certified.
- The current fail-closed workflow and independent acceptance workflow are
  preserved as release invariants.

## Verification

- Python 3.11/3.12 fast CI verifies source and installed-wheel flows.
- Container build verifies the declared reference environment.
- Nightly/manual CI verifies the full headless hospital demonstration and publishes
  immutable evidence artifacts.
- Release workflow builds sdist/wheel, validates metadata, and uploads checked
  artifacts only from an explicit version tag.

## Non-goals

- No broad plugin marketplace or universal robot abstraction.
- No multi-distribution ROS support promise.
- No public network-hosted MCP service.
- No modifications to the verified hospital route, mission semantics, safety
  state machine, or acceptance thresholds unless separately requested.
