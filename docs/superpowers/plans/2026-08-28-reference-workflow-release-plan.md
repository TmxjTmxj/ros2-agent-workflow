# Reference Workflow Release Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a reproducible, quality-gated, containerized, and release-ready standard Agent-to-ROS 2 workflow, with hospital delivery retained as its complete validation demonstration.

**Architecture:** The Python wheel owns the portable control plane, default profiles, and schemas. The repository owns the large ROS/Gazebo hospital demonstration assets. Fast CI validates source, installed-wheel, static case, and quality boundaries; a scheduled/manual container workflow executes the headless hospital demonstration and preserves its acceptance evidence as CI artifacts.

**Tech Stack:** Python 3.11/3.12, setuptools, FastMCP, pytest, Ruff, mypy, pytest-cov, pip-audit, Docker, Dev Container, GitHub Actions, ROS 2 Lyrical, Gazebo Sim 10.

**Spec:** `docs/superpowers/specs/2026-08-28-reference-workflow-release-design.md`

## Global Constraints

- Preserve task-level MCP authority; never expose shell, arbitrary ROS topics, payloads, paths, or PIDs.
- Preserve fail-closed authorization, heartbeats, E-stop behavior, and independent evidence semantics.
- Treat Ubuntu 24.04 + ROS 2 Lyrical + Gazebo Sim 10 as the reference demonstration environment.
- Keep hospital delivery as a complete demonstration; do not claim it is the only standard workflow.
- Fast PR checks must not arm hardware, publish motion, or require a graphical display.
- Do not modify hospital route, acceptance thresholds, or mission semantics.

---

## File Structure

- `agent_ros/resources/profiles/`: packaged default robot/task profiles and schemas.
- `agent_ros/profiles/defaults.py`: deterministic packaged-profile root resolver.
- `agent_ros/cli.py`: installed CLI with explicit source override support.
- `mcp_server/ros2_mcp_server.py`: installed MCP entry point and packaged defaults.
- `tests/test_distribution.py`: wheel, entry point, and clean-interpreter smoke tests.
- `tests/contracts/`: adapter contract fixtures and reusable safety/lifecycle assertions.
- `pyproject.toml`: package data, entry points, extras, and tool configuration.
- `.pre-commit-config.yaml`, `Dockerfile`, `.devcontainer/`, `.github/workflows/`: development and automation boundaries.
- `docs/`: English entry point, adapter migration guide, release checklist, architecture one-pager, and demo script.

### Task 1: Package default workflow resources

**Files:**
- Create: `agent_ros/resources/__init__.py`
- Create: `agent_ros/resources/profiles/robots/hospital-amr.yaml`
- Create: `agent_ros/resources/profiles/tasks/hospital-delivery.yaml`
- Create: `agent_ros/resources/profiles/schema/robot-profile.schema.json`
- Create: `agent_ros/resources/profiles/schema/task-profile.schema.json`
- Create: `agent_ros/profiles/defaults.py`
- Modify: `pyproject.toml`
- Test: `tests/test_profiles.py`

**Interfaces:**
- Produces `default_profiles_root() -> pathlib.Path`, returning a readable package-owned `profiles` directory.
- `load_robot_profile` and `load_task_profile` continue to accept an explicit root.

- [ ] **Step 1: Write failing packaged-root tests**

```python
from agent_ros.profiles.defaults import default_profiles_root

def test_default_profiles_root_contains_reviewed_robot_and_task():
    root = default_profiles_root()
    assert (root / "robots" / "hospital-amr.yaml").is_file()
    assert (root / "tasks" / "hospital-delivery.yaml").is_file()
```

- [ ] **Step 2: Run the focused test and verify failure**

Run: `python -m pytest tests/test_profiles.py -k default_profiles_root -q`

Expected: import failure for `agent_ros.profiles.defaults`.

- [ ] **Step 3: Add package resources and resolver**

```python
from importlib.resources import files
from pathlib import Path

def default_profiles_root() -> Path:
    return Path(str(files("agent_ros.resources").joinpath("profiles")))
```

Copy the two reviewed YAML files and two schemas byte-for-byte into package data and configure setuptools package-data for `resources/profiles/**/*`.

- [ ] **Step 4: Verify focused profile tests**

Run: `python -m pytest tests/test_profiles.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add agent_ros/resources agent_ros/profiles/defaults.py pyproject.toml tests/test_profiles.py
git commit -m "feat: package reviewed workflow profiles"
```

### Task 2: Add installed CLI and MCP entry points

**Files:**
- Modify: `agent_ros/cli.py`
- Modify: `mcp_server/ros2_mcp_server.py`
- Modify: `pyproject.toml`
- Test: `tests/test_cli.py`
- Test: `tests/test_mcp_server.py`

**Interfaces:**
- Produces console scripts `agent-ros = agent_ros.cli:main` and `agent-ros-mcp = mcp_server.ros2_mcp_server:main`.
- CLI defaults `--profiles-root` to `default_profiles_root()`.
- MCP default `profiles_root` is `default_profiles_root()` and retains explicit injection for tests.

- [ ] **Step 1: Add failing default-root and entry-point tests**

```python
def test_cli_uses_packaged_profiles_by_default(monkeypatch, capsys):
    assert cli.main(["--json", "status", "hospital-amr"]) == 0
    assert '"profile":"hospital-amr"' in capsys.readouterr().out
```

```python
def test_module_exposes_stdio_main():
    assert callable(ros2_mcp_server.main)
```

- [ ] **Step 2: Run focused tests and verify failure**

Run: `python -m pytest tests/test_cli.py tests/test_mcp_server.py -k 'packaged or stdio_main' -q`

Expected: failures because the CLI defaults to the current directory and MCP has no `main` function.

- [ ] **Step 3: Implement default resolution and entry points**

```python
parser.add_argument("--profiles-root", type=Path, default=default_profiles_root())

def main() -> None:
    create_server().run(transport="stdio", show_banner=False)
```

Add `[project.scripts]` entries in `pyproject.toml` and keep module execution delegating to `main()`.

- [ ] **Step 4: Verify focused tests**

Run: `python -m pytest tests/test_cli.py tests/test_mcp_server.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add agent_ros/cli.py mcp_server/ros2_mcp_server.py pyproject.toml tests/test_cli.py tests/test_mcp_server.py
git commit -m "feat: expose installed control-plane commands"
```

### Task 3: Repair interpreter-independent MCP handshake tests

**Files:**
- Modify: `tests/test_mcp_handshake.py`
- Test: `tests/test_mcp_handshake.py`

**Interfaces:**
- The test launches `sys.executable`, not a repository path.
- Child environment preserves the active interpreter environment and adds only `PYTHONUNBUFFERED=1`.

- [ ] **Step 1: Write the regression assertion**

```python
assert PYTHON == Path(sys.executable)
```

- [ ] **Step 2: Run the test and verify failure**

Run: `python -m pytest tests/test_mcp_handshake.py -q`

Expected: assertion failure because `PYTHON` currently points to `.venv/bin/python`.

- [ ] **Step 3: Use the running interpreter and inherited environment**

```python
PYTHON = Path(sys.executable)
env = {**os.environ, "PYTHONUNBUFFERED": "1"}
```

- [ ] **Step 4: Verify handshake behavior**

Run: `python -m pytest tests/test_mcp_handshake.py -q`

Expected: PASS and child process reaped.

- [ ] **Step 5: Commit**

```bash
git add tests/test_mcp_handshake.py
git commit -m "fix: make MCP handshake test interpreter independent"
```

### Task 4: Build installed-wheel smoke coverage

**Files:**
- Create: `tests/test_distribution.py`
- Modify: `Makefile`
- Modify: `pyproject.toml`
- Test: `tests/test_distribution.py`

**Interfaces:**
- `make smoke-wheel` builds wheel/sdist, installs the wheel into a temporary venv, and executes installed `agent-ros --json status hospital-amr`.
- The smoke test never starts a ROS runtime or arms an adapter.

- [ ] **Step 1: Write failing subprocess smoke test**

```python
def test_built_wheel_installs_and_runs_cli(tmp_path):
    result = subprocess.run([installed_cli, "--json", "status", "hospital-amr"], capture_output=True, text=True, check=True)
    assert json.loads(result.stdout) == {"profile": "hospital-amr", "state": "NEW"}
```

- [ ] **Step 2: Run the test and verify failure**

Run: `python -m pytest tests/test_distribution.py -q`

Expected: missing build tooling or no installed console script.

- [ ] **Step 3: Implement isolated build/install helper**

Use `sys.executable -m build`, `venv.EnvBuilder(with_pip=True)`, and the wheel file discovered under a test-owned temporary directory. Install `pytest`, the built wheel, and no editable source path into the child environment.

- [ ] **Step 4: Verify wheel smoke behavior**

Run: `python -m pytest tests/test_distribution.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/test_distribution.py Makefile pyproject.toml
git commit -m "test: verify installed wheel control plane"
```

### Task 5: Establish quality toolchain and local hooks

**Files:**
- Create: `.pre-commit-config.yaml`
- Modify: `pyproject.toml`
- Modify: `Makefile`
- Modify: `CONTRIBUTING.md`
- Test: `tests/test_distribution.py`

**Interfaces:**
- Produces `make lint`, `make typecheck`, `make audit`, `make coverage`, and `make check`.
- Defines optional dependency group `dev` with `build`, `mypy`, `pip-audit`, `pre-commit`, `pytest`, `pytest-cov`, and `ruff`.

- [ ] **Step 1: Add failing configuration presence tests**

```python
def test_project_declares_dev_quality_tools():
    data = tomllib.loads(Path("pyproject.toml").read_text())
    assert {"ruff", "mypy", "pytest-cov", "pip-audit"} <= set(data["project"]["optional-dependencies"]["dev"])
```

- [ ] **Step 2: Run focused configuration test and verify failure**

Run: `python -m pytest tests/test_distribution.py -k dev_quality_tools -q`

Expected: missing `optional-dependencies`.

- [ ] **Step 3: Configure tools with scoped initial baselines**

Configure Ruff checks `E,F,I,UP,B`; configure mypy against `agent_ros` and `mcp_server`; configure coverage to report `agent_ros,mcp_server`; configure pre-commit for trailing whitespace, YAML/TOML validation, Ruff, and format checking. Use precise per-file ignores only where existing ROS dynamic imports cannot be typed without changing runtime behavior.

- [ ] **Step 4: Verify quality commands**

Run: `make lint && make typecheck && make coverage && make audit`

Expected: all commands return zero.

- [ ] **Step 5: Commit**

```bash
git add .pre-commit-config.yaml pyproject.toml Makefile CONTRIBUTING.md tests/test_distribution.py
git commit -m "chore: add reproducible quality gates"
```

### Task 5.5: Update the Chinese README for the installed standard workflow

**Files:**
- Modify: `README.md`
- Modify: `tests/test_distribution.py`

**Interfaces:**
- Documents the standard boundary as `Profile -> SafetyGateway -> reviewed Adapter -> evidence`.
- Documents hospital delivery as the complete validation demonstration, not the product boundary.
- Documents `pip install ".[dev]"`, `agent-ros`, `agent-ros-mcp`, and `make check` without claiming that either command starts ROS or motion.

- [ ] **Step 1: Add a failing README contract test**

```python
def test_readme_documents_the_installed_control_plane():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "Profile → SafetyGateway → Adapter → evidence" in readme
    assert "agent-ros --json status hospital-amr" in readme
    assert "agent-ros-mcp" in readme
    assert "make check" in readme
```

- [ ] **Step 2: Run the focused test and verify failure**

Run: `python -m pytest tests/test_distribution.py -k readme_documents -q`

Expected: assertion failure because the README lacks the installed CLI and quality-gate contract.

- [ ] **Step 3: Update positioning and safe quick verification**

Add a concise standard-workflow positioning paragraph near `项目是什么`; retain the hospital case and evidence sections unchanged. Add a quick verification block that installs `.[dev]`, invokes `agent-ros --json status hospital-amr`, identifies `agent-ros-mcp` as a stdio server, and runs `make check`. State explicitly that these commands do not start Gazebo or arm an adapter.

- [ ] **Step 4: Verify documentation contract and root tests**

Run: `python -m pytest tests/test_distribution.py -k readme_documents -q && make check`

Expected: PASS; the normal quality command still validates all root checks.

- [ ] **Step 5: Commit**

```bash
git add README.md tests/test_distribution.py
git commit -m "docs: clarify installed standard workflow"
```

### Task 6: Extract controller evidence boundary

**Files:**
- Create: `agent_ros/runtime/terminal_evidence.py`
- Modify: `agent_ros/runtime/controller.py`
- Test: `tests/test_runtime_controller.py`
- Test: `tests/test_audit.py`

**Interfaces:**
- Produces `capture_terminal_evidence(adapter, sources, terminal_status) -> Mapping[str, Observation]`.
- RuntimeController public methods and stable error codes remain unchanged.

- [ ] **Step 1: Write a focused evidence-boundary test**

```python
def test_terminal_evidence_capture_translates_adapter_failure():
    with pytest.raises(RuntimeControllerError, match="EVIDENCE_INVALID"):
        capture_terminal_evidence(FailingAdapter(), ("odometry",), AdapterStatus("SUCCEEDED"))
```

- [ ] **Step 2: Run focused test and verify failure**

Run: `python -m pytest tests/test_runtime_controller.py -k terminal_evidence_capture -q`

Expected: import failure for `terminal_evidence`.

- [ ] **Step 3: Extract the pure terminal-evidence orchestration**

Move only terminal snapshot validation and error translation from `RuntimeController` into `terminal_evidence.py`; keep state mutation, gateway transitions, and adapter invocation ordering in `RuntimeController`.

- [ ] **Step 4: Verify controller and audit regression tests**

Run: `python -m pytest tests/test_runtime_controller.py tests/test_audit.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add agent_ros/runtime/terminal_evidence.py agent_ros/runtime/controller.py tests/test_runtime_controller.py tests/test_audit.py
git commit -m "refactor: isolate terminal evidence capture"
```

### Task 7: Extract hospital process lifecycle boundary

**Files:**
- Create: `agent_ros/adapters/hospital_process.py`
- Modify: `agent_ros/adapters/hospital.py`
- Test: `tests/test_adapters.py`

**Interfaces:**
- Produces `ManagedHospitalProcess` with `start(argv, timeout)`, `terminate(timeout)`, and `close(timeout) -> bool`.
- Hospital adapter’s public classes and `RobotAdapter` contract remain unchanged.

- [ ] **Step 1: Add a failing lifecycle test**

```python
def test_managed_hospital_process_reaps_child_after_ready_process_stops(tmp_path):
    process = ManagedHospitalProcess()
    process.start([sys.executable, "-c", "print('ready')"], timeout=1.0)
    assert process.close(timeout=1.0)
```

- [ ] **Step 2: Run focused test and verify failure**

Run: `python -m pytest tests/test_adapters.py -k managed_hospital_process -q`

Expected: import failure for `hospital_process`.

- [ ] **Step 3: Extract only subprocess ownership and cleanup**

Move the existing process-group launch, readiness, terminate, kill, wait, and stderr collection helpers into `ManagedHospitalProcess`. Keep hospital mission state, ROS topic semantics, and task policy in `hospital.py`.

- [ ] **Step 4: Verify adapter regressions**

Run: `python -m pytest tests/test_adapters.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add agent_ros/adapters/hospital_process.py agent_ros/adapters/hospital.py tests/test_adapters.py
git commit -m "refactor: isolate hospital process lifecycle"
```

### Task 8: Add reusable adapter contract tests

**Files:**
- Create: `tests/contracts/__init__.py`
- Create: `tests/contracts/adapter_contract.py`
- Create: `tests/test_adapter_contract.py`
- Modify: `tests/support/runtime_owners.py`

**Interfaces:**
- Produces `assert_adapter_contract(adapter: RobotAdapter, task: object, sources: tuple[str, ...]) -> None`.
- Contract validates rejected unauthorized start, safe cancel/stop idempotence, E-stop latch, bounded close, and terminal evidence type validity using deterministic fake adapters.

- [ ] **Step 1: Write failing fake-adapter contract test**

```python
def test_contract_rejects_adapter_that_starts_without_permit():
    with pytest.raises(AssertionError, match="activation permit"):
        assert_adapter_contract(UnsafeFakeAdapter(), task=object(), sources=("odometry",))
```

- [ ] **Step 2: Run focused test and verify failure**

Run: `python -m pytest tests/test_adapter_contract.py -q`

Expected: import failure for `tests.contracts.adapter_contract`.

- [ ] **Step 3: Implement deterministic contract fixture and hospital registration**

Implement the contract with no ROS process. Add a hospital fake lifecycle adapter that exercises the same authorization and cleanup hooks used by HospitalDeliveryAdapter; do not start Gazebo.

- [ ] **Step 4: Verify contract and adapter regression tests**

Run: `python -m pytest tests/test_adapter_contract.py tests/test_adapters.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/contracts tests/test_adapter_contract.py tests/support/runtime_owners.py
git commit -m "test: add adapter safety contract suite"
```

### Task 9: Add PR quality and wheel CI workflow

**Files:**
- Modify: `.github/workflows/ci.yml`
- Create: `.github/workflows/release.yml`

**Interfaces:**
- PR workflow runs tests, wheel smoke, lint, typecheck, coverage, and audit on Python 3.11/3.12.
- Release workflow runs only on `v*` tags, builds checked distributions, attaches distributions, release notes, and verification manifest to a GitHub Release.

- [ ] **Step 1: Add CI configuration tests**

```python
def test_ci_runs_installed_wheel_smoke_and_quality_gates():
    workflow = Path(".github/workflows/ci.yml").read_text()
    assert "make smoke-wheel" in workflow
    assert "make check" in workflow
```

- [ ] **Step 2: Run focused test and verify failure**

Run: `python -m pytest tests/test_distribution.py -k ci_runs_installed -q`

Expected: assertion failure because the current workflow lacks both targets.

- [ ] **Step 3: Implement CI and release jobs**

Use pinned official GitHub Actions major versions, install `.[dev]`, execute the Make targets, upload coverage XML, and write a release artifact manifest containing SHA-256 values. Use `softprops/action-gh-release` only after build and verification jobs succeed.

- [ ] **Step 4: Validate workflow syntax and regression tests**

Run: `python -m pytest tests/test_distribution.py -q && pre-commit run --all-files`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add .github/workflows/ci.yml .github/workflows/release.yml tests/test_distribution.py
git commit -m "ci: gate releases on installed workflow verification"
```

### Task 10: Add reference container and Dev Container

**Files:**
- Create: `Dockerfile`
- Create: `.dockerignore`
- Create: `.devcontainer/devcontainer.json`
- Create: `docker-compose.yml`
- Modify: `Makefile`
- Modify: `README.md`

**Interfaces:**
- `make docker-build` builds the official Ubuntu 24.04/ROS Lyrical image.
- `make docker-smoke` executes non-motion wheel smoke tests inside the image.
- `make docker-hospital` runs the existing headless hospital demonstration and writes evidence to a bind-mounted output directory.

- [ ] **Step 1: Add failing container configuration checks**

```python
def test_container_declares_reference_ros_environment():
    dockerfile = Path("Dockerfile").read_text()
    assert "ubuntu:24.04" in dockerfile
    assert "ros-lyrical" in dockerfile
    assert "gz-sim10" in dockerfile
```

- [ ] **Step 2: Run focused test and verify failure**

Run: `python -m pytest tests/test_distribution.py -k container_declares -q`

Expected: `FileNotFoundError` for `Dockerfile`.

- [ ] **Step 3: Implement headless image and workspace mount commands**

Use a non-root `ros` user, source `/opt/ros/lyrical/setup.bash` in the shell entrypoint, install project `.[dev]`, set `PYTHONNOUSERSITE=1`, and use the existing `scripts/demo_hospital.sh --headless --verify` command. Do not expose host networking or privileged mode.

- [ ] **Step 4: Verify image build and smoke test**

Run: `make docker-build && make docker-smoke`

Expected: both commands return zero.

- [ ] **Step 5: Commit**

```bash
git add Dockerfile .dockerignore .devcontainer docker-compose.yml Makefile README.md tests/test_distribution.py
git commit -m "feat: add reference workflow container"
```

### Task 11: Add scheduled headless demonstration evidence workflow

**Files:**
- Create: `.github/workflows/nightly-hospital.yml`
- Create: `scripts/verify_release_artifacts.py`
- Modify: `examples/hospital_delivery/scripts/run_via_mcp.py`
- Test: `examples/hospital_delivery/tests/test_run_via_mcp.py`

**Interfaces:**
- Nightly/manual workflow builds the reference image, runs `make docker-hospital`, validates generated evidence, and uploads evidence, screenshots, trace, and logs.
- `verify_release_artifacts.py <directory>` validates expected artifact names and JSON report validity without rewriting artifacts.

- [ ] **Step 1: Add failing artifact verifier test**

```python
def test_release_artifact_verifier_rejects_missing_acceptance_report(tmp_path):
    assert verify_release_artifacts(tmp_path) == 2
```

- [ ] **Step 2: Run focused test and verify failure**

Run: `python -m pytest examples/hospital_delivery/tests/test_run_via_mcp.py -k release_artifact_verifier -q`

Expected: import failure for `verify_release_artifacts`.

- [ ] **Step 3: Implement read-only artifact validation and workflow**

Require `acceptance_report.json`, `mcp_agent_trace.json`, initial/final PNG evidence, and a JSON success state; use `workflow_dispatch` and weekly cron; upload artifacts with a 30-day retention policy.

- [ ] **Step 4: Verify static case tests and workflow syntax checks**

Run: `python -m pytest examples/hospital_delivery/tests/test_run_via_mcp.py examples/hospital_delivery/tests/test_acceptance_report.py -q && pre-commit run --files .github/workflows/nightly-hospital.yml scripts/verify_release_artifacts.py`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add .github/workflows/nightly-hospital.yml scripts/verify_release_artifacts.py examples/hospital_delivery/scripts/run_via_mcp.py examples/hospital_delivery/tests/test_run_via_mcp.py
git commit -m "ci: archive nightly hospital demonstration evidence"
```

### Task 12: Publish concise English and release materials

**Files:**
- Create: `README.en.md`
- Create: `docs/ADAPTER-MIGRATION.md`
- Create: `docs/RELEASE.md`
- Create: `docs/demo/english-demo-script.md`
- Create: `docs/demo/architecture-one-pager.md`
- Modify: `README.md`

**Interfaces:**
- English README gives a short positioning statement, safe quick verification, architecture overview, and links to Chinese/full documentation.
- Migration guide explains Profile -> Adapter -> safety contract -> evidence flow, without claiming hardware certification.
- Release guide defines v0.1.0 tag, artifact, and verification steps.

- [ ] **Step 1: Add documentation link checks**

```python
def test_readmes_link_to_each_other_and_release_docs():
    assert "README.en.md" in Path("README.md").read_text()
    assert "README.md" in Path("README.en.md").read_text()
    assert Path("docs/RELEASE.md").is_file()
```

- [ ] **Step 2: Run focused test and verify failure**

Run: `python -m pytest tests/test_distribution.py -k readmes_link -q`

Expected: `FileNotFoundError` for `README.en.md`.

- [ ] **Step 3: Write scoped documentation**

Keep English README under 250 lines. Make the hospital case explicitly a full validation demonstration of the standard workflow. Include precise non-claims: no universal SDK promise and no production hardware certification.

- [ ] **Step 4: Verify documentation links and Markdown lint hooks**

Run: `python -m pytest tests/test_distribution.py -q && pre-commit run --all-files`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add README.md README.en.md docs/ADAPTER-MIGRATION.md docs/RELEASE.md docs/demo tests/test_distribution.py
git commit -m "docs: package workflow migration and release materials"
```

### Task 13: Execute final release-readiness verification

**Files:**
- Modify: `docs/RELEASE.md`

**Interfaces:**
- Produces a recorded verification matrix for fast checks, wheel install, container smoke, and nightly trigger instructions.

- [ ] **Step 1: Run complete fast verification**

Run: `make check && make smoke-wheel`

Expected: all quality gates, coverage, audit, source tests, and installed-wheel smoke return zero.

- [ ] **Step 2: Run container verification**

Run: `make docker-build && make docker-smoke`

Expected: both return zero.

- [ ] **Step 3: Inspect release payload**

Run: `python -m build && twine check dist/* && sha256sum dist/*`

Expected: valid sdist/wheel metadata and deterministic hashes recorded in release instructions.

- [ ] **Step 4: Update release checklist with observed command outputs**

Document exact commands, expected GitHub workflow names, and the manual two-minute video recording checklist.

- [ ] **Step 5: Commit**

```bash
git add docs/RELEASE.md
git commit -m "docs: record v0.1.0 verification checklist"
```
