# Task 2 report: Add installed CLI and MCP entry points

## Files changed

- Modified `agent_ros/cli.py` so the CLI resolves `--profiles-root` through Task 1's `default_profiles_root()`.
- Modified `mcp_server/ros2_mcp_server.py` so the singleton runtime uses packaged profiles by default and added `main()` for the stdio server. Module execution now delegates to `main()`.
- Modified `pyproject.toml` with the `agent-ros` and `agent-ros-mcp` console scripts.
- Modified `tests/test_cli.py` with a packaged-profile test executed from an empty working directory.
- Modified `tests/test_mcp_server.py` with the stdio-main export test.

## Tests and commands

The exact brief command was attempted first:

```text
$ python -m pytest tests/test_cli.py tests/test_mcp_server.py -k 'packaged or stdio_main' -q
/bin/bash: line 1: python: command not found
exit_code=127
```

The system `python3` lacked the project's FastMCP dependency, so an isolated Python 3.11 environment was created with `uv`, `pytest`, and this project. It established the required red state:

```text
$ <isolated-python> -m pytest tests/test_cli.py tests/test_mcp_server.py -k 'packaged or stdio_main' -q
FAILED tests/test_cli.py::test_cli_uses_packaged_profiles_by_default
FAILED tests/test_mcp_server.py::test_module_exposes_stdio_main
2 failed, 23 deselected
```

Focused verification after implementation:

```text
$ <isolated-python> -m pytest tests/test_cli.py tests/test_mcp_server.py -q
.........................                                                [100%]
25 passed in 4.34s
```

Installed-command verification from an empty temporary directory:

```text
$ uv pip install --python <isolated-python> --reinstall .
... agent-ros==0.1.0 installed ...

$ agent-ros --json status hospital-amr
{"profile":"hospital-amr","state":"NEW"}
```

`agent-ros-mcp` entered its long-running stdio server loop; a five-second bounded probe intentionally stopped it with `timeout` exit code 124. `git diff --check` completed with exit code 0.

## Self-review

- The CLI and MCP import the Task 1 packaged-root resolver rather than duplicating resource lookup.
- Explicit CLI `--profiles-root` overrides remain unchanged.
- Existing injected-controller and injected-evidence-store test seams remain unchanged.
- The MCP tool registrations, schemas, metadata, and bounded authority surface were not modified.
- The console scripts target the requested module `main` functions, and direct module execution delegates to the same MCP main function.

## Concerns

- The workspace provides no `python` command and its existing shared virtual environment lacks `pip` and `pytest`; verification used a temporary isolated Python 3.11 environment instead.
- No scoped code concerns remain.
