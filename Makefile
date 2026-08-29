PYTHON ?= python3
VENV ?= .venv
ROS_SETUP ?= /opt/ros/lyrical/setup.bash
AGENT_ROS_UID ?= $(shell id -u)
AGENT_ROS_GID ?= $(shell id -g)

.PHONY: venv install test smoke-wheel lint typecheck coverage audit check test-hospital verify docker-build docker-smoke docker-hospital docker-mcp-trace clean

venv:
	$(PYTHON) -m venv $(VENV)

install: venv
	$(VENV)/bin/python -m pip install --upgrade pip setuptools
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

docker-build:
	AGENT_ROS_UID=$(AGENT_ROS_UID) AGENT_ROS_GID=$(AGENT_ROS_GID) docker compose build agent-ros

docker-smoke:
	AGENT_ROS_UID=$(AGENT_ROS_UID) AGENT_ROS_GID=$(AGENT_ROS_GID) docker compose run --rm agent-ros make VENV=/opt/agent-ros-venv smoke-wheel

docker-hospital:
	AGENT_ROS_UID=$(AGENT_ROS_UID) AGENT_ROS_GID=$(AGENT_ROS_GID) docker compose run --rm agent-ros bash scripts/demo_hospital.sh --headless --verify

docker-mcp-trace:
	AGENT_ROS_UID=$(AGENT_ROS_UID) AGENT_ROS_GID=$(AGENT_ROS_GID) docker compose run --rm agent-ros /opt/agent-ros-venv/bin/python examples/hospital_delivery/scripts/run_via_mcp.py

clean:
	rm -rf $(VENV) .pytest_cache
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
