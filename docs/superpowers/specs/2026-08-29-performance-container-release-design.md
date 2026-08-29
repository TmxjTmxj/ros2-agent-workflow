# Performance, Container, and Release-Candidate Design

## Purpose

Make the standard Agent-to-ROS 2 workflow cheaper to verify on every change,
explicit about the performance needed by the complete hospital demonstration,
and harder to release with incomplete distribution artifacts.  This extends the
existing reproducibility work without changing the reviewed workflow itself.

## Invariants

The product boundary remains:

```text
MCP Agent -> declarative Profile -> SafetyGateway -> reviewed Adapter
          -> ROS 2 runtime -> independent acceptance evidence
```

Hospital delivery remains the complete simulation demonstration of that
boundary.  Its route, 300-second acceptance budget, safety heartbeat, mission
semantics, and independent evidence checks are release invariants.  In
particular, a slow host must fail visibly; it must never be made to pass by
loosening the workflow.

## Capability Preflight

Add a read-only Python preflight command for the full hospital container.
It reports host/container facts that matter to a headless Gazebo runner:

- visible CPU count and cgroup CPU/memory limits;
- available container runtimes and optional GPU-device visibility;
- Docker daemon reachability; and
- whether the local reference container can resolve its ROS and Gazebo setup.

The command has two modes.  Its default report is diagnostic and always
machine-readable.  `--require-accelerated-runtime` is an explicit runner
policy gate for sites that have provisioned GPU-backed Docker.  It reports a
clear non-zero failure if that policy cannot be met.  Neither mode claims to
predict real-time factor (RTF): only the unchanged independent acceptance run
measures the actual workflow under load.

The nightly job runs this preflight before the expensive demonstration and
uploads the JSON result with the normal evidence bundle.  A runner guide states
that maintainers must measure a candidate machine with `make docker-hospital`
and retain a passing report before assigning it the `ros-gazebo` label.

## Two Container Targets

Refactor the Dockerfile into two named stages sharing a small Python base:

- `control-plane` contains the installed wheel, dev verification tools, default
  profiles, schemas, CLI, and stdio MCP server.  It deliberately contains no
  ROS/Gazebo packages and is used for fast installed-wheel smoke checks.
- `hospital-runtime` adds ROS 2 Lyrical, Gazebo Sim, TurtleBot assets, and the
  same installed control plane.  It remains the default image and is used by
  Dev Container and all hospital commands.

Compose exposes both services.  Existing `agent-ros` commands retain their
full-runtime behavior; `docker-smoke` moves to the lightweight control-plane
service.  This preserves a single source tree and entry-point contract while
removing the multi-gigabyte ROS layer from routine control-plane verification.

## Release Candidate Verification

Add a repository-owned verifier for a `dist/` directory.  It validates exactly
one wheel and one source distribution for the project version, checks that the
SHA-256 manifest names and hashes every distribution artifact, and rejects
unexpected package distribution files.  `twine check` remains the package
metadata authority and runs before this verifier.

The release workflow builds distributions, runs `twine check`, creates the
manifest, runs the verifier, and uploads only the verified directory.  The
workflow still publishes only after its verification job succeeds; no tag,
release, or remote runner registration is performed from local automation.

## Non-goals

- No attempt to use the current local CPU-only Docker host as a passing
  hospital runner.
- No assertion that GPU presence alone guarantees a passing RTF.
- No changes to MCP authority, adapters, profile schemas, hospital navigation,
  acceptance monitor, or safety policy.
- No GitHub Runner installation, label assignment, tag push, or GitHub Release
  creation; those require maintainer-controlled external authority.
