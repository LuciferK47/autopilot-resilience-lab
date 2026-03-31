PYTHON ?= python
ARDUPILOT_PATH ?= ../ardupilot

.PHONY: lint test smoke list

lint:
	$(PYTHON) -m ruff check .

test:
	$(PYTHON) -m pytest tests -q

smoke:
	$(PYTHON) resilience_prober.py --smoke --ardupilot-path $(ARDUPILOT_PATH)

list:
	$(PYTHON) resilience_prober.py --list
