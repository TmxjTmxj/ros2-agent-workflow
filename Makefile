PYTHON ?= python3
VENV ?= .venv
ROS_SETUP ?= /opt/ros/lyrical/setup.bash

.PHONY: venv install test test-hospital verify clean

venv:
	$(PYTHON) -m venv $(VENV)

install: venv
	$(VENV)/bin/python -m pip install --upgrade pip
	$(VENV)/bin/python -m pip install -e .
	$(VENV)/bin/python -m pip install pytest

test:
	$(VENV)/bin/python -m pytest tests/ -q

test-hospital:
	@test -f $(ROS_SETUP) || (echo "ROS setup not found: $(ROS_SETUP)" && exit 1)
	bash -c "source $(ROS_SETUP) && /usr/bin/python3 -m pytest examples/hospital_delivery/tests/ -q"

verify: test
	@echo "Root test suite passed. Hospital ROS case should be run locally with 'make test-hospital'."

clean:
	rm -rf $(VENV) .pytest_cache
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
