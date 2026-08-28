PYTHON ?= python3
VENV ?= .venv
ROS_SETUP ?= /opt/ros/lyrical/setup.bash

.PHONY: venv install test smoke-wheel lint typecheck coverage audit check test-hospital verify clean

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

clean:
	rm -rf $(VENV) .pytest_cache
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
