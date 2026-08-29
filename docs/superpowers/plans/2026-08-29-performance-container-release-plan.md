# Performance, Container, and Release-Candidate Implementation Plan

> **For agentic workers:** use `superpowers:executing-plans` and verify each
> task before moving to the next one.

**Goal:** Add an honest hospital-runner preflight, a fast control-plane image,
and release-candidate artifact verification without altering the validated
hospital workflow.

**Architecture:** A multi-stage Dockerfile shares a Python base between a
ROS-free control-plane image and the existing full ROS/Gazebo runtime image.
Preflight emits a JSON capability report and checks only explicit runner
policies; acceptance evidence remains the performance authority.  A standalone
distribution verifier checks exactly the artifacts and SHA manifest that the
release workflow publishes.

**Spec:** `docs/superpowers/specs/2026-08-29-performance-container-release-design.md`

## Global Constraints

- Never change the hospital route, independent acceptance thresholds, timeout,
  heartbeat, or MCP authority boundary.
- Keep `agent-ros` as the full runtime Compose service and default Docker image.
- Keep all safety-relevant commands fail-closed.
- Do not create external GitHub Releases, tags, or self-hosted runner state.

### Task 1: Add a deterministic release-candidate verifier

**Files:** `scripts/verify_release_candidate.py`, `tests/test_release_candidate.py`,
`pyproject.toml`, `Makefile`

- [x] Write tests for valid wheel/sdist plus `SHA256SUMS.txt`, hash mismatch,
  missing artifact, and unexpected distribution artifact.
- [x] Implement a read-only JSON verifier that accepts `--dist-dir`, extracts
  the project version from `pyproject.toml`, and exits non-zero on errors.
- [x] Add `twine` to the `dev` extra and a `release-verify` Make target that
  builds into a caller-selected directory, runs `twine check`, writes the
  manifest, then invokes the verifier.
- [x] Run focused tests and the release target in a temporary output directory.

### Task 2: Split the container into control-plane and hospital targets

**Files:** `Dockerfile`, `docker-compose.yml`, `.devcontainer/devcontainer.json`,
`Makefile`, `tests/test_distribution.py`

- [x] Extend the static container contract test with target/service/command
  expectations, then confirm it fails.
- [x] Introduce a shared non-ROS Python base and named `control-plane` and
  `hospital-runtime` stages.  Keep `hospital-runtime` as Dockerfile default.
- [x] Add a `control-plane` Compose service, route `docker-smoke` to it, and
  add explicit build targets for both images while retaining `docker-build` for
  the hospital runtime.
- [x] Pin Dev Container to `hospital-runtime` so its ROS behavior does not
  change.
- [x] Build/smoke the control-plane target and statically validate the full
  runtime configuration.

### Task 3: Add runner capability preflight

**Files:** `scripts/check_hospital_environment.py`,
`tests/test_hospital_environment.py`, `Makefile`,
`.github/workflows/nightly-hospital.yml`, `tests/test_distribution.py`

- [x] Add focused tests using injected command/filesystem probes for a
  diagnostic report, absent Docker, and failed accelerated-runtime policy.
- [x] Implement a side-effect-free JSON reporter.  It may inspect Docker and
  image commands but must not start the hospital mission or write evidence.
- [x] Add `docker-hospital-preflight` and call it in nightly before the full
  hospital job.  Stage the report with nightly artifacts even if a later step
  fails.
- [x] Extend static workflow tests and run the focused suite.

### Task 4: Document runner qualification and release hand-off

**Files:** `docs/RUNNER.md`, `docs/RELEASE.md`, `README.md`, `README.en.md`,
`tests/test_distribution.py`

- [x] Document self-hosted `ros-gazebo` qualification, the difference between
  preflight facts and measured acceptance, and exact local commands.
- [x] Document `make release-verify` and clarify that release publication still
  starts only from a reviewed tag.
- [x] Add concise README links and documentation contract assertions.

### Task 5: End-to-end verification and commit

- [x] Run formatting, lint, typing, unit/static tests, wheel smoke, audit, and
  release-candidate verification.
- [x] Build and smoke the light control-plane image.  Do not run or rewrite
  hospital evidence on the known-underpowered local host.
- [x] Review the diff for scope and preserve the user-owned `build/` directory.
- [x] Commit the implementation with a focused message.
