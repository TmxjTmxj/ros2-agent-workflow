# Verification Baseline and Citation Rules

This document is the citation-safe baseline for project descriptions, résumés,
slides, and README updates. It exists because older repository text mixed test
counts from different Python/ROS environments and duplicated rendered evidence
images, making otherwise accurate claims easy to misquote.

## What changed, and why

| Correction | Previous ambiguity | Current rule | Why it matters |
| --- | --- | --- | --- |
| Test count | Historical text advertised `505` “unique tests”. It was a manually deduplicated sum from changing, isolated suites. | Cite the command and its result instead of one synthetic total. | A reader can rerun the exact command; a résumé does not need later numerical correction. |
| Camera images | README displayed copies under `assets/` even though their bytes matched acceptance evidence. | README embeds `examples/hospital_delivery/evidence/acceptance-*.png`, the exact files named by the schema-2 report. | The displayed image and the independently verified artifact now have one canonical path. |
| ROS timing test | An artificial wall-time jump could let a timer run before an asynchronously delivered fresh odometry message. | The time-separation test synchronously invokes the same odometry callback at the simulated receipt time. | The test now checks ROS-time mission timeout versus wall-time feedback freshness, not DDS scheduling order. |

These are documentation, evidence-reference, and test-determinism corrections.
They do **not** change the reviewed Profile, hospital route, mission semantics,
MCP authority boundary, fail-closed behavior, 1-second heartbeat, 180-second
simulation-time mission budget, or 300-second wall-clock acceptance budget.

## Citation-safe verification results

The following commands were run on 2026-08-30 from the reviewed
`codex/release-repro-quality` branch. Quote both the scope and result.

| Scope | Command | Result | Safe wording |
| --- | --- | --- | --- |
| Python control plane | `make check` | 388 passed, 81% coverage; Ruff, mypy, dependency audit, and installed-wheel smoke passed | “388 control-plane tests passed with 81% coverage; installed CLI/MCP wheel smoke passed.” |
| ROS hospital reference suite | `make test-hospital` | 150 passed using the system ROS Python | “150 ROS hospital reference tests passed in the ROS environment.” |
| ROS-free container control plane | `make docker-smoke` | 8 passed | “The installed wheel’s CLI/MCP smoke passed in a lightweight container without starting ROS/Gazebo.” |
| Recorded hospital demonstration | schema-2 `acceptance_report.json` | `SUCCEEDED`; 137.76 seconds ROS simulation time; zero prohibited contacts | “The recorded independent hospital acceptance report is `SUCCEEDED`; see the JSON and canonical PNG evidence.” |

Do **not** quote `505 unique tests`, `509 tests`, or a combined count formed by
adding the rows above. The rows use different interpreters and runtime
dependencies, and their purpose is to establish different boundaries.

## What this does not claim

- It does not certify a physical robot or universal hardware integration.
- It does not claim that the local CPU-only Docker host passes the complete
  hospital acceptance budget.
- It does not replace the independent acceptance report with an MCP trace or a
  unit-test result.

For the release process, see [RELEASE.md](RELEASE.md). For qualifying a full
ROS/Gazebo runner, see [RUNNER.md](RUNNER.md).
