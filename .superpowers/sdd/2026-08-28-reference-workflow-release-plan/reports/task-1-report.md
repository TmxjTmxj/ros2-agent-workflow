# Task 1 report: Package default workflow resources

## Files changed

- Created `agent_ros/resources/__init__.py`.
- Created `agent_ros/resources/profiles/robots/hospital-amr.yaml`.
- Created `agent_ros/resources/profiles/tasks/hospital-delivery.yaml`.
- Created `agent_ros/resources/profiles/schema/robot-profile.schema.json`.
- Created `agent_ros/resources/profiles/schema/task-profile.schema.json`.
- Created `agent_ros/profiles/defaults.py` with `default_profiles_root() -> pathlib.Path`.
- Modified `pyproject.toml` to declare `agent_ros.resources` package data using `profiles/**/*`.
- Modified `tests/test_profiles.py` with the packaged-root test.

The four packaged profile/schema files were copied from the repository-owned `profiles/` files and verified byte-for-byte with `cmp`.

## Tests and commands

The exact brief command was attempted first:

```text
$ python -m pytest tests/test_profiles.py -k default_profiles_root -q
/bin/bash: line 1: python: command not found
exit_code=127
```

The equivalent available interpreter established the required red state:

```text
$ python3 -m pytest tests/test_profiles.py -k default_profiles_root -q
ERROR collecting tests/test_profiles.py
E   ModuleNotFoundError: No module named 'agent_ros.profiles.defaults'
!!!!!!!!!!!!!!!!!!!! Interrupted: 1 error during collection !!!!!!!!!!!!!!!!!!!!
1 error in 0.41s
exit_code=2
```

Focused profile verification:

```text
$ python3 -m pytest tests/test_profiles.py -q
...........................                                              [100%]
27 passed in 0.47s
exit_code=0
```

Packaging verification:

```text
$ python3 -m pip wheel . --no-deps --no-build-isolation --wheel-dir "$pkg_verify_dir"
Processing /home/tmxj/ros2-agent-workflow/.worktrees/release-repro-quality
  Preparing metadata (pyproject.toml): started
  Preparing metadata (pyproject.toml): finished with status 'done'
Building wheels for collected packages: agent-ros
  Building wheel for agent-ros (pyproject.toml): started
  Building wheel for agent-ros (pyproject.toml): finished with status 'done'
  Created wheel for agent-ros: filename=agent_ros-0.1.0-py3-none-any.whl size=71831 sha256=d78590d38ff470c48b8c46f00c4962542e52310d65f7399b05346affb088a752
  Stored in directory: /home/tmxj/.cache/pip/wheels/8e/6f/91/35930cb7eccbb3cfc731417a05f803e993ae92f8174b6a6eb0
Successfully built agent-ros
wheel=agent_ros-0.1.0-py3-none-any.whl
packaged resources=4
exit_code=0
```

The wheel check confirmed these four paths were present:

- `agent_ros/resources/profiles/robots/hospital-amr.yaml`
- `agent_ros/resources/profiles/tasks/hospital-delivery.yaml`
- `agent_ros/resources/profiles/schema/robot-profile.schema.json`
- `agent_ros/resources/profiles/schema/task-profile.schema.json`

`git diff --check` completed with exit code 0.

## Self-review

- The resolver uses `importlib.resources.files("agent_ros.resources")` and returns the package-owned `profiles` path as required.
- Existing explicit-root loader interfaces were not changed.
- The package marker makes the resource tree discoverable by setuptools.
- Package-data metadata is scoped to `agent_ros.resources` and the required recursive resource path.
- The focused test covers both reviewed default YAML files; the wheel check covers both YAML files and both schemas.

## Concerns

- The environment has no `python` command, so the specified commands were run with `python3`; the exact unavailable-command output is recorded above.
- `default_profiles_root()` returns a filesystem path using `Path(str(...))`, matching the brief’s requested implementation. No concerns remain for the scoped task.
