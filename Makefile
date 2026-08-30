PYTHON ?= python3
VENV ?= .venv
ROS_SETUP ?= /opt/ros/lyrical/setup.bash
AGENT_ROS_UID ?= $(shell id -u)
AGENT_ROS_GID ?= $(shell id -g)
RELEASE_DIST ?= dist
HOSPITAL_PREFLIGHT_REPORT ?= artifacts/hospital/environment-preflight.json
HOSPITAL_PREFLIGHT_FLAGS ?= --require-container

.PHONY: venv install test smoke-wheel lint typecheck coverage audit check test-hospital verify release-verify docker-build docker-control-build docker-smoke docker-hospital-preflight docker-hospital docker-mcp-trace clean

venv:
	$(PYTHON) -m venv $(VENV)

install: venv
	$(VENV)/bin/python -m pip install --upgrade pip "setuptools>=83.0.0"
	$(VENV)/bin/python -m pip install -e ".[dev]"

test:
	$(VENV)/bin/python -m pytest tests/ -q

smoke-wheel:
	$(VENV)/bin/python -m pytest tests/test_distribution.py -q

lint:
	$(VENV)/bin/ruff check agent_ros mcp_server tests
	$(VENV)/bin/ruff format --check agent_ros mcp_server tests

typecheck:
	$(VENV)/bin/mypy

coverage:
	$(VENV)/bin/python -m pytest --cov --cov-report=term-missing tests -q

audit:
	$(VENV)/bin/python -m pip_audit

check: lint typecheck coverage audit smoke-wheel

test-hospital:
	@test -f $(ROS_SETUP) || (echo "ROS setup not found: $(ROS_SETUP)" && exit 1)
	bash -c "source $(ROS_SETUP) && /usr/bin/python3 -m pytest examples/hospital_delivery/tests/ -q"

verify: test
	@echo "Root test suite passed. Hospital ROS case should be run locally with 'make test-hospital'."

release-verify:
	@test ! -e "$(RELEASE_DIST)" || test -z "$$(find "$(RELEASE_DIST)" -mindepth 1 -maxdepth 1 -print -quit)" || (echo "Release output directory must be empty: $(RELEASE_DIST)" && exit 1)
	mkdir --parents "$(RELEASE_DIST)"
	$(VENV)/bin/python -m build --outdir "$(RELEASE_DIST)"
	$(VENV)/bin/python -m twine check "$(RELEASE_DIST)"/*
	cd "$(RELEASE_DIST)" && sha256sum agent_ros-*.tar.gz agent_ros-*.whl > SHA256SUMS.txt
	$(VENV)/bin/python scripts/verify_release_candidate.py --dist-dir "$(RELEASE_DIST)"

docker-build:
	AGENT_ROS_UID=$(AGENT_ROS_UID) AGENT_ROS_GID=$(AGENT_ROS_GID) docker compose build agent-ros

docker-control-build:
	AGENT_ROS_UID=$(AGENT_ROS_UID) AGENT_ROS_GID=$(AGENT_ROS_GID) docker compose build control-plane

docker-smoke:
	AGENT_ROS_UID=$(AGENT_ROS_UID) AGENT_ROS_GID=$(AGENT_ROS_GID) docker compose run --rm control-plane make VENV=/opt/agent-ros-venv smoke-wheel

docker-hospital-preflight:
	mkdir --parents "$(dir $(HOSPITAL_PREFLIGHT_REPORT))"
	$(PYTHON) scripts/check_hospital_environment.py --output "$(HOSPITAL_PREFLIGHT_REPORT)" $(HOSPITAL_PREFLIGHT_FLAGS)

docker-hospital:
	AGENT_ROS_UID=$(AGENT_ROS_UID) AGENT_ROS_GID=$(AGENT_ROS_GID) docker compose run --rm agent-ros bash scripts/demo_hospital.sh --headless --verify

docker-mcp-trace:
	AGENT_ROS_UID=$(AGENT_ROS_UID) AGENT_ROS_GID=$(AGENT_ROS_GID) docker compose run --rm agent-ros /opt/agent-ros-venv/bin/python examples/hospital_delivery/scripts/run_via_mcp.py

clean:
	rm -rf $(VENV) .pytest_cache
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
