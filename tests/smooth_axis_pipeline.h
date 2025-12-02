//
// Created by Jonatan Vider on 30/11/2025.
//
#pragma once

#include <stdint.h>
#include "smooth_axis_scenario.h"

// Find first/last reported output for a scenario
void find_start_and_final_outputs(const scenario_t *sc,
                                  uint16_t *out_start,
                                  uint16_t *out_final);

// Measure settle time between start_out and final_out
float measure_settle_time(const scenario_t *sc,
                          uint16_t start_out,
                          uint16_t final_out);

// High-level runner: prints info & compares to expected
void run_scenario(const scenario_t *sc);

// Dump full timeline to CSV
void dump_scenario_csv(const scenario_t *sc, const char *filename);