# Release Closure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a verifiable `v0.1.1` without rewriting the cancelled `v0.1.0` release attempt.

**Architecture:** Keep the reviewed control-plane and hospital semantics unchanged. Remove the CI-only concurrent-test scheduling race that can leave a non-daemon worker alive after a test assertion, and set explicit job deadlines so a future failure is observable within minutes rather than consuming GitHub's six-hour default. The tag workflow continues to build, verify, manifest, upload, and publish immutable Python artifacts.

**Tech Stack:** Python 3.11/3.12, pytest, GitHub Actions, setuptools build, Twine, GitHub Releases.

**Spec:** `docs/RELEASE.md`

## Global Constraints

- Work directly on `main`; do not create a delivery branch.
- Never rewrite or move `v0.1.0`.
- Preserve Profile, SafetyGateway, Adapter, hospital route, mission limits, and acceptance semantics.
- Release `v0.1.1` only after a green `main` CI run and a passing local quality gate.
- Do not stage the user-owned `build/` directory.

---

### Task 1: Make the queued-command test deterministic and cleanup-safe

**Files:**
- Modify: `tests/test_safety_sequencer.py:98-133`

**Interfaces:**
- Consumes: `_SafetySequencer.submit(permit, command, timeout)` and `close(timeout)`.
- Produces: A test that waits for the active command before queuing its stale command and always calls `close()` after releasing the active command.

- [ ] **Step 1: Capture the failing CI symptom**

Run: `gh run view 33259074245 --repo TmxjTmxj/ros2-agent-workflow --job 99117789877 --log`

Expected: the active command in `test_latch_rejects_every_queued_command_without_invoking_it` raises `TIMEOUT`, and the job is subsequently cancelled at GitHub's six-hour limit.

- [ ] **Step 2: Rewrite the test orchestration**

Create the queued `ThreadCall` only after `entered.wait()` proves the active receipt is in flight. Use an active submission deadline that exceeds the test's bounded orchestration window, and use nested `try/finally` cleanup so `sequencer.close()` runs even if `active.result()` raises.

- [ ] **Step 3: Run the focused regression test**

Run: `python -m pytest tests/test_safety_sequencer.py::test_latch_rejects_every_queued_command_without_invoking_it -q`

Expected: PASS; the queued command is rejected as `ESTOP_LATCHED` and its callback is never invoked.

- [ ] **Step 4: Commit with the release guard changes**

```bash
git add tests/test_safety_sequencer.py .github/workflows/ci.yml .github/workflows/release.yml pyproject.toml docs/superpowers/plans/2026-08-30-release-closure-plan.md
git commit -m "fix: bound release verification failures"
```

### Task 2: Bound GitHub Actions failures and prepare the patch release

**Files:**
- Modify: `.github/workflows/ci.yml`
- Modify: `.github/workflows/release.yml`
- Modify: `pyproject.toml`
- Test: `tests/test_distribution.py`

**Interfaces:**
- Consumes: CI `quality` job and release `verify-distributions` job.
- Produces: Explicit job time limits and package metadata version `0.1.1` matching tag `v0.1.1`.

- [ ] **Step 1: Write workflow/version assertions**

Add a test asserting `timeout-minutes:` appears in both CI and release workflows and that `pyproject.toml` reports `0.1.1`.

- [ ] **Step 2: Verify the assertion fails before the workflow edit**

Run: `python -m pytest tests/test_distribution.py -k workflow -q`

Expected: FAIL because neither workflow currently declares an explicit timeout and package metadata remains `0.1.0`.

- [ ] **Step 3: Implement the smallest release guard**

Set `timeout-minutes: 15` on Python quality jobs and `timeout-minutes: 20` on distribution verification. Set `project.version = "0.1.1"`; do not change dependencies or release artifact rules.

- [ ] **Step 4: Re-run the assertion**

Run: `python -m pytest tests/test_distribution.py -k workflow -q`

Expected: PASS.

### Task 3: Verify and publish

**Files:**
- No source changes expected.

**Interfaces:**
- Consumes: `main` at the committed release-guard revision and tag-triggered `release.yml`.
- Produces: a green `main` CI run, tag `v0.1.1`, GitHub Release with wheel, sdist, and `SHA256SUMS.txt`.

- [ ] **Step 1: Run local quality gates**

Run: `make VENV=/tmp/ros2-agent-task4.TIC8Kp/venv check && make test-hospital`

Expected: control-plane gate passes and the ROS hospital suite reports 150 passing tests.

- [ ] **Step 2: Push main and verify GitHub CI**

Run: `git push origin main`, then inspect the generated CI run.

Expected: all Python 3.11/3.12 quality and hospital-static jobs succeed.

- [ ] **Step 3: Create and push immutable patch tag**

Run:

```bash
git tag -a v0.1.1 -m "v0.1.1"
git push origin v0.1.1
```

Expected: a new tag points to the green `main` commit; `v0.1.0` remains unchanged.

- [ ] **Step 4: Verify release output**

Run: `gh release view v0.1.1 --repo TmxjTmxj/ros2-agent-workflow --json url,tagName,assets`

Expected: published GitHub Release contains `agent_ros-0.1.1-py3-none-any.whl`, `agent_ros-0.1.1.tar.gz`, and `SHA256SUMS.txt`.
