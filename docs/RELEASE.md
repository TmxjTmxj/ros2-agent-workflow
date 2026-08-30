# v0.1.0 Release Checklist

This checklist publishes a reproducible reference workflow release. It does not certify a
robot or expand the reviewed MCP authority boundary.

Use [the verification baseline](VERIFICATION-BASELINE.md) when updating a résumé,
presentation, or README: it records the current reproducible result wording, explains the
historical count/evidence corrections, and identifies the claims this release does not make.

## Before tagging

1. Start from a clean, reviewed commit and update the version in `pyproject.toml`.
2. Create the project virtual environment and install development dependencies.
3. Run the fast verification matrix:

   ```bash
   make check
   make smoke-wheel
   make docker-build
   make docker-control-build
   make docker-smoke
   ```

4. For a local ROS/Gazebo validation, run `make docker-hospital` and retain the generated
   `examples/hospital_delivery/logs/acceptance_report.json` and camera frames.
5. Run `make docker-mcp-trace` to collect the separate production MCP trace.
6. Build and verify release distributions in a new empty output directory:

   ```bash
   make release-verify RELEASE_DIST=/tmp/agent-ros-release-candidate
   ```

   This runs `twine check`, writes `SHA256SUMS.txt`, and verifies that exactly
   the versioned wheel and source distribution match that manifest.

## Tag and artifacts

Create and push `v0.1.0`. The `release` GitHub workflow then runs `make check`, builds the
wheel and source distribution, runs `twine check`, writes and verifies `SHA256SUMS.txt`,
uploads the verified distributions, and publishes the GitHub Release only after that
verification job completes.

The `nightly-hospital` workflow is separate: it can be manually dispatched or runs weekly.
It requires a Linux self-hosted runner labelled `ros-gazebo` that has been measured to meet
the existing 300-second acceptance budget. It builds the reference image, runs the headless
hospital demonstration and MCP trace, validates the staged evidence bundle without rewriting
it, and retains its artifact for 30 days. Do not weaken the route or acceptance thresholds to
make an underpowered hosted runner pass.
See [the runner qualification guide](RUNNER.md) for the preflight boundary and the exact
maintainer sequence.

## Two-minute recording checklist

Use [the English demo script](demo/english-demo-script.md). Show the installed CLI status,
the bounded MCP tool sequence, the independent acceptance report/screenshots, and the
architecture boundary. Do not claim universal robot support or production certification.

## Recorded verification matrix

Update this section before pushing the tag with the exact command output, current commit,
Python versions, and image digest observed for that release candidate.

### Local baseline — 2026-08-29 (not a release qualification)

- `make check` passed with Ruff, mypy, 388 root tests at 81% coverage, a clean `pip-audit`,
  and 8 installed-wheel checks.
- The Ubuntu 26.04 / ROS Lyrical / Gazebo Sim 10.x image built successfully; its installed
  wheel smoke passed all 7 checks in 102.57 seconds.
- The local Docker host is CPU-only for containers (GeForce MX350 host driver, no NVIDIA
  container runtime). The hospital run starts and passes graph readiness, but reaches only
  137.27 simulation seconds in 300.06 wall seconds (RTF about 0.46), so the independent
  acceptance monitor correctly rejects it. The same condition can miss the 1-second
  fail-closed heartbeat window during the MCP run.

This is evidence for using the labelled self-hosted `ros-gazebo` runner, not a reason to
relax the hospital route, acceptance threshold, or safety heartbeat.

| Check | Command | Expected evidence |
| --- | --- | --- |
| Quality and source | `make check` | lint, mypy, tests, coverage, audit, wheel smoke |
| Installed wheel | `make smoke-wheel` | fresh-venv CLI and stdio MCP handshake |
| Light control-plane image | `make docker-control-build && make docker-smoke` | ROS-free installed control-plane smoke |
| Reference ROS container | `make docker-build` | full hospital runtime image |
| Runner preflight | `make docker-hospital-preflight` | host/container capability JSON, not an RTF claim |
| Hospital demonstration | `make docker-hospital` | independent report and PNG screenshots |
| MCP control plane | `make docker-mcp-trace` | separate `SUCCEEDED` tool trace |
