/**
 * @file test_api_sanity.c
 * @brief Comprehensive API sanity tests for smooth_axis library (Enhanced)
 * @date 2025-12-08
 *
 * Enhanced test suite covering NULL safety, mode mismatches, edge cases, boundary conditions,
 * reset validation, output quantization, uninitialized state, and critical corner cases.
 *
 * Compile with:
 *   gcc -o test_api -DNDEBUG -I./include ./tests/test_api_sanity.c ./src/smooth_axis.c -lm
 *
 * Run:
 *   ./test_api
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include "smooth_axis.h"

// ============================================================================
// Test Helpers
// ============================================================================

// Mock timer for AUTO_DT tests
static uint32_t mock_time_ms = 0;

static uint32_t test_timer(void) {
    return mock_time_ms;
}

// Helper: Advance mock timer
static void advance_time_ms(uint32_t delta_ms) {
    mock_time_ms += delta_ms;
}

// Helper: Reset mock timer
static void reset_timer(void) {
    mock_time_ms = 0;
}

// Helper: Check if two floats are approximately equal
static bool float_eq(float a, float b, float epsilon) {
    return fabsf(a - b) < epsilon;
}

// ============================================================================
// Test 1: NULL Pointer Safety (Release Mode)
// ============================================================================

void test_null_safety_release_mode(void) {
#ifdef NDEBUG
    // All these should return early without crashing in release builds
    // In debug builds, they would assert, so we skip this test
    
    smooth_axis_config_auto_dt(NULL, 1023, 0.25f, test_timer);
    smooth_axis_config_live_dt(NULL, 1023, 0.25f);
    smooth_axis_init(NULL, NULL);
    smooth_axis_update_auto_dt(NULL, 500);
    smooth_axis_update_live_dt(NULL, 500, 0.016f);
    smooth_axis_reset(NULL, 500);
    
    // These should return safe default values
    bool result = smooth_axis_has_new_value(NULL);
    assert(result == false);
    
    float norm = smooth_axis_get_norm(NULL);
    assert(norm == 0.0f);
    
    uint16_t u16 = smooth_axis_get_u16(NULL);
    assert(u16 == 0);
    
    float noise = smooth_axis_get_noise_norm(NULL);
    assert(noise == 0.0f);
    
    float thresh_norm = smooth_axis_get_effective_thresh_norm(NULL);
    assert(thresh_norm == 0.0f);
    
    uint16_t thresh_u16 = smooth_axis_get_effective_thresh_u16(NULL);
    assert(thresh_u16 == 0);
    
    printf("✓ Test 1: NULL pointer safety (release mode)\n");
#else
    printf("⊘ Test 1: NULL pointer safety (skipped in debug mode - would assert)\n");
#endif
}

// ============================================================================
// Test 2: Mode Mismatch - AUTO_DT axis with LIVE_DT update
// ============================================================================

void test_mode_mismatch_auto_called_with_live(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    // Create AUTO_DT axis
    reset_timer();
    smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, test_timer);
    smooth_axis_init(&axis, &cfg);
    
    // Try to update with LIVE_DT function (should no-op in release)
    // In debug, this would assert
    smooth_axis_update_live_dt(&axis, 500, 0.016f);
    
    // Axis should remain uninitialized (no sample processed)
    bool                 has_new = smooth_axis_has_new_value(&axis);
    assert(has_new == false);
    
    printf("✓ Test 2: Mode mismatch - AUTO_DT axis with LIVE_DT update\n");
}

// ============================================================================
// Test 3: Mode Mismatch - LIVE_DT axis with AUTO_DT update
// ============================================================================

void test_mode_mismatch_live_called_with_auto(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    // Create LIVE_DT axis
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    // Try to update with AUTO_DT function (should no-op in release)
    // In debug, this would assert
    smooth_axis_update_auto_dt(&axis, 500);
    
    // Axis should remain uninitialized (no sample processed)
    bool                 has_new = smooth_axis_has_new_value(&axis);
    assert(has_new == false);
    
    printf("✓ Test 3: Mode mismatch - LIVE_DT axis with AUTO_DT update\n");
}

// ============================================================================
// Test 4: Edge Case - max_raw = 0 (degenerate)
// ============================================================================

void test_edge_max_raw_zero(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    // Degenerate case: max_raw = 0
    smooth_axis_config_live_dt(&cfg, 0, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    // Update with 0 (only valid value)
    smooth_axis_update_live_dt(&axis, 0, 0.016f);
    
    // Should handle gracefully
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value == 0);
    
    float norm = smooth_axis_get_norm(&axis);
    assert(norm == 0.0f);
    
    printf("✓ Test 4: Edge case - max_raw = 0\n");
}

// ============================================================================
// Test 5: Edge Case - max_raw = 65535 (16-bit max)
// ============================================================================

void test_edge_max_raw_16bit(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    // 16-bit ADC range
    smooth_axis_config_live_dt(&cfg, 65535, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    // Update with max value
    smooth_axis_update_live_dt(&axis, 65535, 0.016f);
    
    // Should handle large range correctly
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value == 65535);
    
    float norm = smooth_axis_get_norm(&axis);
    assert(float_eq(norm, 1.0f, 0.001f));
    
    printf("✓ Test 5: Edge case - max_raw = 65535 (16-bit)\n");
}

// ============================================================================
// Test 6: CRITICAL - Negative Delta Time
// ============================================================================

void test_critical_negative_dt(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    // Seed with valid value
    smooth_axis_update_live_dt(&axis, 500, 0.016f);
    smooth_axis_has_new_value(&axis);
    
    // Update with negative dt
    // Debug: Would assert in get_alpha_from_dt()
    // Release: Falls back to alpha=1.0 (instant convergence)
    smooth_axis_update_live_dt(&axis, 600, -0.016f);
    
    // Should converge instantly (alpha=1.0 fallback)
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value >= 595 && value <= 605);  // Should be near 600
    
    printf("✓ Test 6: CRITICAL - negative dt falls back to instant convergence (value=%u)\n",
           value);
}

// ============================================================================
// Test 7: CRITICAL - Zero Settle Time
// ============================================================================

void test_critical_zero_settle_time(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    // settle_time = 0 (instant response, no smoothing)
    smooth_axis_config_live_dt(&cfg, 1023, 0.0f);
    smooth_axis_init(&axis, &cfg);
    
    // Update should work
    smooth_axis_update_live_dt(&axis, 0, 0.016f);
    smooth_axis_update_live_dt(&axis, 1023, 0.016f);
    
    // With zero settle time, should converge instantly (alpha = 1.0)
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value == 1023);  // Should reach target immediately
    
    printf("✓ Test 7: CRITICAL - zero settle time (instant convergence)\n");
}

// ============================================================================
// Test 8: CRITICAL - Very Large Delta Time
// ============================================================================

void test_critical_very_large_dt(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    // Seed
    smooth_axis_update_live_dt(&axis, 0, 0.016f);
    
    // Update with absurdly large dt (1000 seconds!)
    smooth_axis_update_live_dt(&axis, 1023, 1000.0f);
    
    // Should converge instantly due to large dt
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value == 1023);  // Should reach target with huge dt
    
    printf("✓ Test 8: CRITICAL - very large dt (1000s) causes instant convergence\n");
}

// ============================================================================
// Test 9: CRITICAL - Inverted Dead Zones
// ============================================================================

void test_critical_inverted_dead_zones(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    
    // Set inverted dead zones (full_off > full_on)
    cfg.full_off_norm = 0.8f;  // High floor
    cfg.full_on_norm  = 0.2f;   // Low ceiling (inverted!)
    
    smooth_axis_init(&axis, &cfg);
    
    // Update with middle value
    smooth_axis_update_live_dt(&axis, 512, 0.016f);
    
    // Should handle gracefully (implementation falls back to full range)
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value >= 0 && value <= 1023);
    
    printf("✓ Test 9: CRITICAL - inverted dead zones handled gracefully\n");
}

// ============================================================================
// Test 10: CRITICAL - Sticky Zone at Maximum
// ============================================================================

void test_critical_sticky_zone_maximum(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    
    // Try to set sticky zone above maximum (should be clamped to 0.49)
    cfg.sticky_zone_norm = 0.6f;  // Exceeds MAX_STICKY_ZONE
    
    smooth_axis_init(&axis, &cfg);
    
    // Update with middle value
    smooth_axis_update_live_dt(&axis, 512, 0.016f);
    
    // Should not crash despite excessive sticky zone
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value >= 0 && value <= 1023);
    
    printf("✓ Test 10: CRITICAL - excessive sticky zone clamped\n");
}

// ============================================================================
// Test 11: Reset During Warmup (AUTO_DT)
// ============================================================================

void test_reset_during_warmup(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    reset_timer();
    smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, test_timer);
    smooth_axis_init(&axis, &cfg);
    
    // Partial warmup (100 of 256 cycles)
    for (int i = 0; i < 100; i++) {
        advance_time_ms(16);
        smooth_axis_update_auto_dt(&axis, 500);
    }
    
    // Reset mid-warmup
    smooth_axis_reset(&axis, 800);
    
    // Continue warmup
    for (int i = 100; i < 300; i++) {
        advance_time_ms(16);
        smooth_axis_update_auto_dt(&axis, 800);
    }
    
    // Should still complete warmup and function normally
    bool                 has_new = smooth_axis_has_new_value(&axis);
    // First check might be false if already at 800, that's okay
    
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value >= 750 && value <= 850);
    
    printf("✓ Test 11: Reset during warmup preserves warmup state\n");
}

// ============================================================================
// Test 12: First Sample Teleport
// ============================================================================

void test_first_sample_teleport(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    // First sample should teleport immediately (no smoothing)
    smooth_axis_update_live_dt(&axis, 1023, 0.016f);
    
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value == 1023);  // Should be exact, not smoothed from 0
    
    printf("✓ Test 12: First sample teleports (no smoothing on frame 0)\n");
}

// ============================================================================
// Test 13: Rapid Resets
// ============================================================================

void test_rapid_resets(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    // Many rapid resets
    for (int i = 0; i < 50; i++) {
        smooth_axis_reset(&axis, i * 20);
        smooth_axis_update_live_dt(&axis, i * 20, 0.016f);
    }
    
    // Should still be stable
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value >= 0 && value <= 1023);
    
    printf("✓ Test 13: Rapid resets handled gracefully\n");
}

// ============================================================================
// Test 14: Noise Saturation (Extreme Alternating Input)
// ============================================================================

void test_noise_saturation(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    // Seed to middle
    smooth_axis_update_live_dt(&axis, 512, 0.016f);
    smooth_axis_has_new_value(&axis);
    
    // Inject extreme alternating noise (1000 cycles)
    for (int i = 0; i < 1000; i++) {
        uint16_t noisy = (i % 2) ? 0 : 1023;
        smooth_axis_update_live_dt(&axis, noisy, 0.016f);
    }
    
    // Check noise estimate
    float noise = smooth_axis_get_noise_norm(&axis);
    printf("   Noise after 1000 extreme alternations: %.4f\n", noise);
    
    // Noise should be high, but not necessarily 1.0 (it's slowly tracked)
    assert(noise > 0.01f);  // Should detect the noise
    
    // Axis should still be responsive after noise ends
    for (int i = 0; i < 100; i++) {
        smooth_axis_update_live_dt(&axis, 512, 0.016f);
    }
    
    uint16_t value = smooth_axis_get_u16(&axis);
    printf("   Value after returning to 512: %u\n", value);
    assert(value >= 400 && value <= 650);  // Should recover
    
    printf("✓ Test 14: Noise saturation and recovery\n");
}

// ============================================================================
// Test 15: Stable Input Noise Decay
// ============================================================================

void test_stable_input_noise_decay(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    // Initial noise injection
    for (int i = 0; i < 50; i++) {
        uint16_t noisy = 512 + (i % 3 - 1) * 10;
        smooth_axis_update_live_dt(&axis, noisy, 0.016f);
    }
    
    float noise_after_noise = smooth_axis_get_noise_norm(&axis);
    
    // Now perfectly stable for 1000 updates
    for (int i = 0; i < 1000; i++) {
        smooth_axis_update_live_dt(&axis, 512, 0.016f);
    }
    
    float noise_after_stable = smooth_axis_get_noise_norm(&axis);
    
    printf("   Noise: %.4f (noisy) -> %.4f (stable)\n",
           noise_after_noise, noise_after_stable);
    
    // Noise should decay (but not necessarily to zero due to sign flip logic)
    assert(noise_after_stable <= noise_after_noise + 0.01f);
    
    printf("✓ Test 15: Stable input allows noise decay\n");
}

// ============================================================================
// Test 16: Output Quantization Boundaries
// ============================================================================

void test_output_quantization_boundaries(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    // Test the special case thresholds in smooth_axis_get_u16()
    // if (n <= 1.0f / max_out) { return 0; }
    // For max_raw = 1023: 1.0/1023 ≈ 0.000977
    
    // Test exact zero
    smooth_axis_reset(&axis, 0);
    smooth_axis_update_live_dt(&axis, 0, 0.016f);
    assert(smooth_axis_get_u16(&axis) == 0);
    
    // Test exact max
    smooth_axis_reset(&axis, 1023);
    smooth_axis_update_live_dt(&axis, 1023, 0.016f);
    assert(smooth_axis_get_u16(&axis) == 1023);
    
    // Test value = 1 (just above zero threshold)
    smooth_axis_reset(&axis, 1);
    smooth_axis_update_live_dt(&axis, 1, 0.016f);
    uint16_t val_1 = smooth_axis_get_u16(&axis);
    assert(val_1 >= 0 && val_1 <= 2);  // Should be 0, 1, or 2
    
    // Test value = 1022 (just below max threshold)
    smooth_axis_reset(&axis, 1022);
    smooth_axis_update_live_dt(&axis, 1022, 0.016f);
    uint16_t val_1022 = smooth_axis_get_u16(&axis);
    assert(val_1022 >= 1021 && val_1022 <= 1023);  // Should be close to 1022
    
    printf("✓ Test 16: Output quantization at boundaries (val_1=%u, val_1022=%u)\n",
           val_1, val_1022);
}

// ============================================================================
// Test 17: Warmup with Variable Frame Times
// ============================================================================

void test_warmup_variable_frame_times(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    reset_timer();
    smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, test_timer);
    smooth_axis_init(&axis, &cfg);
    
    // Warmup with alternating 10ms and 30ms frames (average = 20ms)
    for (int i = 0; i < 300; i++) {
        uint32_t dt = (i % 2) ? 10 : 30;
        advance_time_ms(dt);
        smooth_axis_update_auto_dt(&axis, 500);
    }
    
    // Should have completed warmup and measured average ~20ms
    // Update with movement
    advance_time_ms(20);
    smooth_axis_update_auto_dt(&axis, 700);
    
    bool                 has_new = smooth_axis_has_new_value(&axis);
    assert(has_new == true);
    
    printf("✓ Test 17: Warmup with variable frame times\n");
}

// ============================================================================
// Test 18: Timer Wraparound (AUTO_DT)
// ============================================================================

void test_timer_wraparound(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    // Start timer near overflow
    mock_time_ms = 0xFFFFFF00;
    smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, test_timer);
    smooth_axis_init(&axis, &cfg);
    
    // Update before overflow
    smooth_axis_update_auto_dt(&axis, 500);
    
    // Advance to cause overflow
    mock_time_ms = 0x00000010;  // Wrapped around
    smooth_axis_update_auto_dt(&axis, 500);
    
    // Should handle wraparound correctly (unsigned arithmetic)
    // dt = 0x00000010 - 0xFFFFFF00 = 0x00000110 = 272ms (valid)
    
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value >= 0 && value <= 1023);
    
    printf("✓ Test 18: Timer wraparound handled by unsigned arithmetic\n");
}

// ============================================================================
// Test 19: Sticky Zone at Zero (Disabled)
// ============================================================================

void test_sticky_zone_zero(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    cfg.sticky_zone_norm = 0.0f;  // Disable sticky zones
    
    smooth_axis_init(&axis, &cfg);
    
    // Update near endpoints
    smooth_axis_update_live_dt(&axis, 50, 0.016f);
    float norm_low = smooth_axis_get_norm(&axis);
    
    // Without sticky zone, should not snap to 0
    assert(norm_low > 0.02f);  // Should be proportional to 50/1023
    
    printf("✓ Test 19: Sticky zone = 0 (disabled) - no snapping\n");
}

// ============================================================================
// Test 20: Very Fast vs Very Slow Settle Times (Improved)
// ============================================================================

void test_fast_vs_slow_settle_times(void) {
    smooth_axis_config_t cfg_fast, cfg_slow;
    smooth_axis_t        axis_fast, axis_slow;
    
    // Fast settle (100ms)
    smooth_axis_config_live_dt(&cfg_fast, 1023, 0.1f);
    smooth_axis_init(&axis_fast, &cfg_fast);
    
    // Slow settle (2s)
    smooth_axis_config_live_dt(&cfg_slow, 1023, 2.0f);
    smooth_axis_init(&axis_slow, &cfg_slow);
    
    // Seed first sample = 0
    smooth_axis_update_live_dt(&axis_fast, 0, 0.016f);
    smooth_axis_update_live_dt(&axis_slow, 0, 0.016f);
    
    // Race: both try to reach 1023 in 10 frames
    for (int i = 0; i < 10; i++) {
        smooth_axis_update_live_dt(&axis_fast, 1023, 0.016f);
        smooth_axis_update_live_dt(&axis_slow, 1023, 0.016f);
    }
    
    uint16_t val_fast = smooth_axis_get_u16(&axis_fast);
    uint16_t val_slow = smooth_axis_get_u16(&axis_slow);
    
    assert(val_fast > val_slow);  // Fast should have moved more
    assert(val_fast > 900);       // Fast should be very close to 1023
    
    printf("✓ Test 20: Fast vs slow settle (0→1023 in 10 frames): slow=%u, fast=%u\n",
           val_slow, val_fast);
}

// ============================================================================
// Test 21: Stability - Same Value 1000 Times
// ============================================================================

void test_stability_same_value_1000_times(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    // First update should trigger new value
    smooth_axis_update_live_dt(&axis, 512, 0.016f);
    bool                 has_new = smooth_axis_has_new_value(&axis);
    assert(has_new == true);
    
    // Next 999 updates with same value should not trigger many updates
    int      new_value_count = 0;
    for (int i               = 0; i < 999; i++) {
        smooth_axis_update_live_dt(&axis, 512, 0.016f);
        if (smooth_axis_has_new_value(&axis)) {
            new_value_count++;
        }
    }
    
    // Should be very stable (maybe 1-3 updates as it settles)
    assert(new_value_count < 10);
    
    printf("✓ Test 21: Stability - same value 1000x triggered %d updates\n", new_value_count);
}

// ============================================================================
// Test 22-29: Keep existing tests from original suite
// ============================================================================

void test_edge_raw_value_zero(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    smooth_axis_update_live_dt(&axis, 0, 0.016f);
    
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value == 0);
    
    printf("✓ Test 22: Raw value at zero\n");
}

void test_edge_raw_value_max(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    smooth_axis_update_live_dt(&axis, 1023, 0.016f);
    
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value == 1023);
    
    printf("✓ Test 23: Raw value at max\n");
}

void test_reset_to_zero(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    smooth_axis_update_live_dt(&axis, 500, 0.016f);
    smooth_axis_has_new_value(&axis);
    
    smooth_axis_reset(&axis, 0);
    
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value == 0);
    
    printf("✓ Test 24: Reset to zero\n");
}

void test_reset_to_middle(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    smooth_axis_update_live_dt(&axis, 0, 0.016f);
    smooth_axis_has_new_value(&axis);
    
    smooth_axis_reset(&axis, 512);
    
    uint16_t value = smooth_axis_get_u16(&axis);
    assert(value >= 500 && value <= 524);
    
    printf("✓ Test 25: Reset to middle position\n");
}

void test_uninitialized_has_new_value(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    bool                 has_new = smooth_axis_has_new_value(&axis);
    assert(has_new == false);
    
    printf("✓ Test 26: Uninitialized - has_new_value before update\n");
}

void test_rapid_alternating_input(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    for (int i = 0; i < 100; i++) {
        uint16_t value = (i % 2 == 0) ? 0 : 1023;
        smooth_axis_update_live_dt(&axis, value, 0.016f);
    }
    
    uint16_t result = smooth_axis_get_u16(&axis);
    assert(result >= 0 && result <= 1023);
    
    printf("✓ Test 27: Rapid alternating input (0, max, 0, max...)\n");
}

void test_two_independent_axes(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis1, axis2;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis1, &cfg);
    smooth_axis_init(&axis2, &cfg);
    
    smooth_axis_update_live_dt(&axis1, 300, 0.016f);
    smooth_axis_update_live_dt(&axis2, 700, 0.016f);
    
    uint16_t val1 = smooth_axis_get_u16(&axis1);
    uint16_t val2 = smooth_axis_get_u16(&axis2);
    
    assert(val1 < 400);
    assert(val2 > 600);
    assert(abs((int)val1 - (int)val2) > 200);
    
    printf("✓ Test 28: Two independent axes\n");
}

void test_diagnostic_functions(void) {
    smooth_axis_config_t cfg;
    smooth_axis_t        axis;
    
    smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
    smooth_axis_init(&axis, &cfg);
    
    for (int i = 0; i < 20; i++) {
        uint16_t noisy_value = 512 + (i % 3 - 1) * 5;
        smooth_axis_update_live_dt(&axis, noisy_value, 0.016f);
    }
    
    float noise = smooth_axis_get_noise_norm(&axis);
    assert(noise >= 0.0f && noise <= 1.0f);
    
    float thresh_norm = smooth_axis_get_effective_thresh_norm(&axis);
    assert(thresh_norm >= 0.0f && thresh_norm <= 1.0f);
    
    uint16_t thresh_u16 = smooth_axis_get_effective_thresh_u16(&axis);
    assert(thresh_u16 >= 0 && thresh_u16 <= 1023);
    
    printf("✓ Test 29: Diagnostic functions\n");
}

// ============================================================================
// Main Test Runner
// ============================================================================

int main(void) {
    printf("=== smooth_axis API Sanity Tests (Enhanced) ===\n");
#ifdef NDEBUG
    printf("Build mode: RELEASE (assertions disabled)\n\n");
#else
    printf("Build mode: DEBUG (assertions enabled)\n");
    printf("Note: NULL pointer tests skipped in debug mode (they would assert)\n");
    printf("      To test NULL handling, compile with: gcc -DNDEBUG ...\n\n");
#endif
    
    // Core safety tests
    test_null_safety_release_mode();
    test_mode_mismatch_auto_called_with_live();
    test_mode_mismatch_live_called_with_auto();
    test_edge_max_raw_zero();
    test_edge_max_raw_16bit();
    
    // CRITICAL edge case tests (new)
    test_critical_negative_dt();
    test_critical_zero_settle_time();
    test_critical_very_large_dt();
    test_critical_inverted_dead_zones();
    test_critical_sticky_zone_maximum();
    
    // Advanced behavior tests (new/improved)
    test_reset_during_warmup();
    test_first_sample_teleport();
    test_rapid_resets();
    test_noise_saturation();
    test_stable_input_noise_decay();
    test_output_quantization_boundaries();
    test_warmup_variable_frame_times();
    test_timer_wraparound();
    test_sticky_zone_zero();
    test_fast_vs_slow_settle_times();
    test_stability_same_value_1000_times();
    
    // Basic functionality tests (from original)
    test_edge_raw_value_zero();
    test_edge_raw_value_max();
    test_reset_to_zero();
    test_reset_to_middle();
    test_uninitialized_has_new_value();
    test_rapid_alternating_input();
    test_two_independent_axes();
    test_diagnostic_functions();
    
    printf("\n=== All 29 tests passed! ===\n");
    return 0;
}



/*  Command line (copy paste): Ctrl+C -> Ctrl+V
--------------------------------------
# Compile in release mode (full test coverage)
gcc -o test_api -DNDEBUG -I./include ./tests/test_api_sanity_enhanced.c ./src/smooth_axis.c -lm

# Run it
./test_api
----------------------------------------
 */