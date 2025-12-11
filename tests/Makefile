# Detect project root (one level up from tests/)
ROOT_DIR := $(shell cd .. && pwd)
BUILD_DIR := $(ROOT_DIR)/build
DATA_DIR := $(ROOT_DIR)/tests/data

# Compiler settings
CC := gcc
CFLAGS := -Wall -Wextra -I$(ROOT_DIR)/src
LDFLAGS := -lm

# Source files
SRC_DIR := $(ROOT_DIR)/src
TEST_DIR := $(ROOT_DIR)/tests/c_tests

.PHONY: all setup clean run-tests plot help

help:
	@echo "smooth_axis Test Suite"
	@echo ""
	@echo "Usage:"
	@echo "  make setup      - Create data directories"
	@echo "  make all        - Compile all tests"
	@echo "  make run-tests  - Run all tests (generates CSVs)"
	@echo "  make plot       - Generate plots from test data"
	@echo "  make clean      - Remove build artifacts"
	@echo ""
	@echo "Full workflow:"
	@echo "  make setup && make all && make run-tests && make plot"

setup:
	@echo "Creating data directories..."
	mkdir -p $(DATA_DIR)/ramp_files
	mkdir -p $(DATA_DIR)/step_files
	mkdir -p $(DATA_DIR)/renders
	@echo "✓ Setup complete"

all: setup $(BUILD_DIR)/ramp_test $(BUILD_DIR)/step_test $(BUILD_DIR)/test_api

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/ramp_test: $(TEST_DIR)/ramp_response_test.c $(SRC_DIR)/smooth_axis.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "✓ Built ramp_test"

$(BUILD_DIR)/step_test: $(TEST_DIR)/step_response_test.c $(SRC_DIR)/smooth_axis.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "✓ Built step_test"

$(BUILD_DIR)/test_api: $(TEST_DIR)/test_api_sanity_enhanced.c $(SRC_DIR)/smooth_axis.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DNDEBUG -o $@ $^ $(LDFLAGS)
	@echo "✓ Built test_api"

run-ramp: $(BUILD_DIR)/ramp_test
	@cd $(ROOT_DIR) && $(BUILD_DIR)/ramp_test

run-step: $(BUILD_DIR)/step_test
	@cd $(ROOT_DIR) && $(BUILD_DIR)/step_test

run-api: $(BUILD_DIR)/test_api
	@cd $(ROOT_DIR) && $(BUILD_DIR)/test_api

run-tests: run-ramp run-step run-api
plot:
	@echo "Generating plots..."
	cd $(ROOT_DIR) && python tests/py_scripts/plot_ramp.py
	cd $(ROOT_DIR) && python tests/py_scripts/plot_step.py
	@echo "✓ Plots saved to $(DATA_DIR)/renders/"

clean:
	rm -rf $(BUILD_DIR)
	rm -f $(DATA_DIR)/ramp_files/*.csv
	rm -f $(DATA_DIR)/step_files/*.csv
	rm -f $(DATA_DIR)/renders/*.png
	@echo "✓ Cleaned build artifacts and test data"