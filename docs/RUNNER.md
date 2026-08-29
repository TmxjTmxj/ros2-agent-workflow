# ROS/Gazebo Runner Qualification

`nightly-hospital` is intentionally assigned only to a Linux self-hosted runner
with the `ros-gazebo` label.  It executes the complete headless hospital
demonstration; it is not a fast unit-test job.

## Qualification boundary

`make docker-hospital-preflight` writes
`artifacts/hospital/environment-preflight.json`.  It checks Docker reachability,
container CPU/memory limits, visible GPU devices and runtimes, and that the full
reference image can resolve ROS 2 Lyrical and Gazebo commands.  It does not
start a mission and cannot predict RTF.

Only a successful unchanged independent acceptance report proves that the host
meets the 300-second wall-clock budget.  Do not qualify a runner by reducing
that budget, relaxing the route/acceptance checks, or extending the heartbeat.

## Maintainer qualification sequence

Run these commands on the candidate Linux host from a clean, reviewed commit:

```bash
make docker-build
make docker-hospital-preflight
make docker-hospital
make docker-mcp-trace
```

Retain the generated `environment-preflight.json`, the independent acceptance
report and screenshots, and the `SUCCEEDED` MCP trace.  Assign the
`self-hosted`, `linux`, and `ros-gazebo` labels only after that evidence passes.
If a deployment site requires GPU-backed Docker, make that a local explicit
policy gate:

```bash
make docker-hospital-preflight \
  HOSPITAL_PREFLIGHT_FLAGS="--require-container --require-accelerated-runtime"
```

GPU availability is a capability signal, not a substitute for measured
acceptance evidence.  The repository does not install GitHub Actions Runner,
change labels, or provision a GPU; those are maintainer-controlled operations.
