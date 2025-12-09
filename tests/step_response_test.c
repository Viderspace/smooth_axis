/**
 * @file step_response_test.c
 * @brief Step response test harness for smooth_axis settle time accuracy
 * @author Generated test harness
 * @date 2025-12-08
 *
 * Tests settle time accuracy using step input (900 → 100) and 95% threshold detection.
 * Tests under two conditions: clean and noisy+jittery.
 *
 * Outputs:
 *   - step_results_clean.csv: Summary of clean tests
 *   - step_results_noisy.csv: Summary of noisy tests
 *   - step_trace_clean_XXms.csv: Detailed traces for clean condition (8 files)
 *   - step_trace_noisy_XXms.csv: Detailed traces for noisy condition (8 files)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "smooth_axis.h"

// -----------------------------------------------------------------------------
// Output Configuration
// -----------------------------------------------------------------------------

#define OUTPUT_DIR "/Users/jonatanvider/Documents/smooth_axis_c_library/artifacts/step_test_files"

// -----------------------------------------------------------------------------
// Test Configuration
// -----------------------------------------------------------------------------

#define DT_SEC          0.0002f   // 1ms timestep (1kHz sensor simulation)
#define DURATION_SEC    1.5f     // Total test duration (0.3s before + 1.2s after step)
#define STEP_TIME_SEC   0.3f     // When step occurs

#define RAW_HIGH        900      // Initial value (top of step)
#define RAW_LOW         100      // Target value (bottom of step)
#define MAX_RAW         1023     // 10-bit ADC

// 95% settle detection
#define SETTLE_FRACTION 0.95f
#define STEP_SIZE       (RAW_HIGH - RAW_LOW)  // 800
#define THRESHOLD_95    (RAW_HIGH - (SETTLE_FRACTION * STEP_SIZE))  // 140

// Environmental conditions
#define NOISE_FRACTION  0.04f    // 4% gaussian noise
#define JITTER_FRACTION 0.08f    // 8% dt jitter

// Test matrix: settle_time values in milliseconds
static const float SETTLE_TIME_MS_VALUES[] = {
        20.0f, 50.0f, 200.0f, 500.0f, 1000.0f
};
#define NUM_SETTLE_TIMES (sizeof(SETTLE_TIME_MS_VALUES) / sizeof(SETTLE_TIME_MS_VALUES[0]))

// -----------------------------------------------------------------------------
// Test Condition Types
// -----------------------------------------------------------------------------

typedef enum {
  CONDITION_CLEAN,   // No noise, no jitter
  CONDITION_NOISY    // 4% noise, 8% jitter
}                  test_condition_t;

// -----------------------------------------------------------------------------
// Random Number Generator (from smooth_axis_scenario.c)
// -----------------------------------------------------------------------------

float rand_uniform01(unsigned int *state) {
    *state     = (*state * 1664525u + 1013904223u);
    uint32_t x = *state >> 8;
    return (float)x / (float)0xFFFFFFu;
}

// Approximate standard normal N(0,1) using Box-Muller
static float rand_normal01(unsigned int *state) {
    float u1 = rand_uniform01(state);
    if (u1 < 1e-7f) {
        u1 = 1e-7f;
    }
    float u2 = rand_uniform01(state);
    
    float r   = sqrtf(-2.0f * logf(u1));
    float phi = 2.0f * (float)M_PI * u2;
    
    return r * cosf(phi);
}

// -----------------------------------------------------------------------------
// Noise and Jitter Application
// -----------------------------------------------------------------------------

/**
 * @brief Apply gaussian noise to raw ADC value
 */
uint16_t apply_noise(uint16_t raw, unsigned int *rng) {
    float max_raw_f = (float)MAX_RAW;
    float raw_f     = (float)raw;
    
    // Gaussian noise: σ = noise_fraction / 3 (so ±3σ ≈ ±noise_fraction)
    float sigma_norm = NOISE_FRACTION / 3.0f;
    float n01        = rand_normal01(rng);
    float noise_norm = sigma_norm * n01;
    
    float noisy = raw_f + noise_norm * max_raw_f;
    
    // Clamp to valid range
    if (noisy < 0.0f) { noisy = 0.0f; }
    if (noisy > max_raw_f) { noisy = max_raw_f; }
    
    return (uint16_t)(noisy + 0.5f);
}

/**
 * @brief Apply jitter to dt value
 */
float apply_jitter(float dt_base, unsigned int *rng) {
    float u  = rand_uniform01(rng);
    float j  = (u * 2.0f - 1.0f);  // [-1, 1)
    float dt = dt_base * (1.0f + j * JITTER_FRACTION);
    
    // Ensure dt doesn't go below 10% of base
    if (dt < dt_base * 0.1f) {
        dt = dt_base * 0.1f;
    }
    return dt;
}

// -----------------------------------------------------------------------------
// Test Result Structure
// -----------------------------------------------------------------------------

typedef struct {
  float settle_time_nominal_ms;
  float settle_time_measured_ms;
  float error_pct;
  bool timed_out;
}                  test_result_t;

// -----------------------------------------------------------------------------
// Step Response Test
// -----------------------------------------------------------------------------

/**
 * @brief Run step response test for a single settle_time configuration
 *
 * @param settle_time_sec Nominal settle time in seconds
 * @param condition       Test condition (clean or noisy)
 * @param trace_file      File to write detailed trace
 * @param rng_seed        Random seed for noise/jitter
 * @return Test result with measured settle time
 */
test_result_t run_step_test(float settle_time_sec,
                            test_condition_t condition,
                            FILE *trace_file,
                            unsigned int rng_seed) {
    test_result_t result  = {0};
    result.settle_time_nominal_ms = settle_time_sec * 1000.0f;
    result.timed_out              = true;
    
    // Configure axis for LIVE_DT mode
    smooth_axis_config_t cfg;
    smooth_axis_config_live_dt(&cfg, MAX_RAW, settle_time_sec);
    cfg.sticky_zone_norm = 0.0f;
    cfg.full_off_norm    = 0.0f;
    cfg.full_on_norm     = 1.0f;
    
    smooth_axis_t axis;
    smooth_axis_init(&axis, &cfg);
    
    // Write trace header
    fprintf(trace_file,
            "time_ms,raw_input,raw_ema,crossed_95,has_new,out_u16,noise_norm,thresh_norm\n");
    
    float        t           = 0.0f;
    int          total_steps = (int)(DURATION_SEC / DT_SEC);
    bool          crossed = false;
    unsigned int rng         = rng_seed;
    uint16_t     last_out    = 0;  // Tracks last declared value (only updates on has_new)
    
    for (int i = 0; i < total_steps; i++) {
        // Generate step input: high for first 1 second, then low
        uint16_t raw_clean = (t < STEP_TIME_SEC) ? RAW_HIGH : RAW_LOW;
        
        // Apply noise if noisy condition
        uint16_t raw;
        if (condition == CONDITION_NOISY && i > 0) {
            raw = apply_noise(raw_clean, &rng);
        } else {
            raw = raw_clean;
        }
        
        // Apply jitter to dt if noisy condition
        float dt;
        if (condition == CONDITION_NOISY) {
            dt = apply_jitter(DT_SEC, &rng);
        } else {
            dt = DT_SEC;
        }
        
        // Update filter
        smooth_axis_update_live_dt(&axis, raw, dt);
        
        // Get current EMA output
        uint16_t raw_ema = smooth_axis_get_u16(&axis);
        
        // Check for has_new_value and update out_u16 accordingly
        int has_new = 0;
        if (smooth_axis_has_new_value(&axis)) {
            last_out = smooth_axis_get_u16(&axis);
            has_new  = 1;
            
            // Check for 95% threshold crossing ONLY on declared values (after step occurs)
            if (!crossed && t >= STEP_TIME_SEC && last_out <= THRESHOLD_95) {
                crossed = true;
                result.settle_time_measured_ms = (t - STEP_TIME_SEC) * 1000.0f;
                result.timed_out               = false;
            }
        }
        
        // Get introspection data
        float noise_norm  = smooth_axis_get_noise_norm(&axis);
        float thresh_norm = smooth_axis_get_effective_thresh_norm(&axis);
        
        // Write trace data
        fprintf(trace_file, "%.0f,%u,%u,%d,%d,%u,%.6f,%.6f\n",
                t * 1000.0f,  // time_ms
                raw,          // raw_input
                raw_ema,      // raw_ema (current smoothed output)
                crossed ? 1 : 0,  // crossed_95
                has_new,      // has_new
                last_out,     // out_u16 (last declared value)
                noise_norm,   // noise_norm
                thresh_norm); // thresh_norm
        
        t += dt;
    }
    
    // Calculate error percentage
    if (!result.timed_out) {
        result.error_pct = ((result.settle_time_measured_ms - result.settle_time_nominal_ms)
                            / result.settle_time_nominal_ms) * 100.0f;
    }
    
    return result;
}

// -----------------------------------------------------------------------------
// Test Suite Runner
// -----------------------------------------------------------------------------

/**
 * @brief Run all tests for a given condition
 */
void run_test_suite(test_condition_t condition, const char *condition_name) {
    printf("\n=== %s ===\n", condition_name);
    
    // Open summary results file
    char summary_filename[256];
    snprintf(summary_filename, sizeof(summary_filename),
             "%s/step_results_%s.csv",
             OUTPUT_DIR,
             (condition == CONDITION_CLEAN) ? "clean" : "noisy");
    
    FILE *results_file = fopen(summary_filename, "w");
    if (!results_file) {
        perror("Failed to open summary file");
        return;
    }
    
    fprintf(results_file, "settle_time_ms,measured_settle_ms,error_pct\n");
    
    // Run tests for each settle_time value
    for (size_t i = 0; i < NUM_SETTLE_TIMES; i++) {
        float settle_time_ms  = SETTLE_TIME_MS_VALUES[i];
        float settle_time_sec = settle_time_ms / 1000.0f;
        
        printf("Testing settle_time: %.0fms... ", settle_time_ms);
        fflush(stdout);
        
        // Open trace file for this test
        char trace_filename[256];
        snprintf(trace_filename, sizeof(trace_filename),
                 "%s/step_trace_%s_%.0fms.csv",
                 OUTPUT_DIR,
                 (condition == CONDITION_CLEAN) ? "clean" : "noisy",
                 settle_time_ms);
        
        FILE *trace_file = fopen(trace_filename, "w");
        if (!trace_file) {
            perror("Failed to open trace file");
            fprintf(results_file, "%.0f,error,N/A\n", settle_time_ms);
            continue;
        }
        
        // Run the test (use different RNG seed for each test)
        unsigned int  rng_seed = 12345u + (unsigned int)i +
                                 (condition == CONDITION_NOISY ? 1000u : 0u);
        test_result_t result   = run_step_test(settle_time_sec, condition, trace_file, rng_seed);
        
        fclose(trace_file);
        
        // Output results
        if (result.timed_out) {
            printf("TIMEOUT (did not reach 95%% in %.1fs)\n", DURATION_SEC - STEP_TIME_SEC);
            fprintf(results_file, "%.0f,timeout,N/A\n", settle_time_ms);
        } else {
            printf("measured: %.3fms (%.3f%% error)\n",
                   result.settle_time_measured_ms,
                   result.error_pct);
            fprintf(results_file, "%.0f,%.3f,%.3f\n",
                    settle_time_ms,
                    result.settle_time_measured_ms,
                    result.error_pct);
        }
    }
    
    fclose(results_file);
}

// -----------------------------------------------------------------------------
// Main Test Runner
// -----------------------------------------------------------------------------

int main(void) {
    printf("=== Step Response Test for smooth_axis ===\n");
    printf("Configuration:\n");
    printf("  Step: %u → %u (step size = %d)\n", RAW_HIGH, RAW_LOW, STEP_SIZE);
    printf("  95%% threshold: %.1f\n", THRESHOLD_95);
    printf("  dt: %.3f ms (%.0f Hz)\n", DT_SEC * 1000.0f, 1.0f / DT_SEC);
    printf("  Duration: %.1f seconds\n", DURATION_SEC);
    
    // Run clean condition tests
    run_test_suite(CONDITION_CLEAN, "CLEAN CONDITIONS");
    
    // Run noisy condition tests
    run_test_suite(CONDITION_NOISY, "NOISY CONDITIONS (4% noise, 8% jitter)");
    
    printf("\nDone. Results written to:\n");
    printf("  Directory: %s\n", OUTPUT_DIR);
    printf("  Summary files:\n");
    printf("    - step_results_clean.csv\n");
    printf("    - step_results_noisy.csv\n");
    printf("  Trace files:\n");
    printf("    - step_trace_clean_*.csv (5 files)\n");
    printf("    - step_trace_noisy_*.csv (5 files)\n");
    
    return 0;
}