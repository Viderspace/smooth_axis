.PHONY: help prepare-tests run-tests quickstart

help:
	@echo "smooth_axis - Available commands:"
	@echo ""
	@echo "  make prepare-tests  - One-time setup (install deps, compile tests)"
	@echo "  make run-tests      - Run all tests and generate plots"
	@echo "  make quickstart     - Do everything (prepare + run)"
	@echo ""

prepare-tests:
	@echo "=== Installing Python dependencies ==="
	pip install -r tests/py_scripts/requirements.txt
	@echo ""
	@echo "=== Setting up test directories ==="
	cd tests && $(MAKE) setup
	@echo ""
	@echo "=== Compiling tests ==="
	cd tests && $(MAKE) all
	@echo ""
	@echo "✓ Setup complete! Now run: make run-tests"

run-tests:
	@echo "=== Running ramp response tests ==="
	cd tests && $(MAKE) run-ramp
	@echo ""
	@echo "=== Running step response tests ==="
	cd tests && $(MAKE) run-step
	@echo ""
	@echo "=== Running API sanity tests ==="
	cd tests && $(MAKE) run-api
	@echo ""
	@echo "=== Generating visualization plots ==="
	cd tests && $(MAKE) plot
	@echo ""
	@echo "=========================================="
	@echo "📊 VISUALIZATION RESULTS"
	@echo "=========================================="
	@echo "Click links below to open graphs:"
	@echo ""
	@cd tests/data/renders && for f in *.png; do \
		echo "  file://$(CURDIR)/tests/data/renders/$$f"; \
	done
	@echo ""
	@echo "✓ All tests complete!"

quickstart: prepare-tests run-tests