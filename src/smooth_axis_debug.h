/**
 * @file smooth_axis_debug.h
 * @brief Debug instrumentation: assertions and optional logging
 * @author Jonatan Vider
 * @date 07/12/2024
 *
 * Two debugging aids:
 * 1. ASSERTIONS - Catch bugs during development (debug builds only)
 * 2. LOGGING - Observe behavior at runtime (optional, all builds)
 *
 * Quick start:
 * @code
 * // Assertions (active in debug builds):
 * SMOOTH_AXIS_ASSERT(ptr != NULL, "null pointer");
 *
 * // Logging (requires SMOOTH_AXIS_DEBUG_ENABLE=1):
 * SMOOTH_DEBUG("warmup complete");
 * SMOOTH_DEBUGF("alpha=%.4f dt=%.2fms", alpha, dt_ms);
 * @endcode
 */

#pragma once

#include <stddef.h>  // For NULL

// ============================================================================
// Assertions - Catch programming errors in debug builds
// ============================================================================
// Controlled by NDEBUG (standard C):
//   Debug:   Assertions halt on failure
//   Release: Compiled to nothing (zero cost)
//
// Use for: NULL checks, mode mismatches, invalid state

#ifndef NDEBUG

#include <assert.h>

#define SMOOTH_AXIS_ASSERT(condition, message) \
    assert((condition) && (message))
#else
#define SMOOTH_AXIS_ASSERT(condition, message) \
    ((void)0)
#endif

// ============================================================================
// Smart Guards - Assert in debug, early return in release
// ============================================================================
// Debug:   Crash on failure (find bugs fast)
// Release: Silent early return (graceful degradation)

#ifndef NDEBUG
#define SMOOTH_AXIS_CHECK_RETURN(condition, message) \
    SMOOTH_AXIS_ASSERT(condition, message)

#define SMOOTH_AXIS_CHECK_RETURN_VAL(condition, message, retval) \
    SMOOTH_AXIS_ASSERT(condition, message)
#else
#define SMOOTH_AXIS_CHECK_RETURN(condition, message) \
    do { if (!(condition)) { return; } } while(0)
  
#define SMOOTH_AXIS_CHECK_RETURN_VAL(condition, message, retval) \
    do { if (!(condition)) { return (retval); } } while(0)
#endif

// ============================================================================
// Debug Logging - Observe runtime behavior (optional)
// ============================================================================
// Disabled by default (zero overhead).
// Enable: Define SMOOTH_AXIS_DEBUG_ENABLE=1 in your build
//
// Use for: State transitions, computed values, performance metrics

#if defined(SMOOTH_AXIS_DEBUG_ENABLE) && SMOOTH_AXIS_DEBUG_ENABLE

// QMK: uprintf() with CONSOLE_ENABLE
#if defined(QMK_KEYBOARD_H) && defined(CONSOLE_ENABLE)
#include "print.h"
#define SMOOTH_DEBUG(msg) \
      uprintf("smooth_axis: " msg "\n")
#define SMOOTH_DEBUGF(fmt, ...) \
      uprintf("smooth_axis: " fmt "\n", ##__VA_ARGS__)
  
  // Arduino: Serial (check if ready to avoid hangs)
#elif defined(ARDUINO)
#define SMOOTH_DEBUG(msg) \
      do { \
        if (Serial) { \
          Serial.print("smooth_axis: "); \
          Serial.println(msg); \
        } \
      } while(0)
    
#define SMOOTH_DEBUGF(fmt, ...) \
      do { \
        if (Serial) { \
          Serial.print("smooth_axis: "); \
          Serial.printf(fmt, ##__VA_ARGS__); \
          Serial.println(); \
        } \
      } while(0)
  
  // Native: printf
#else
#include <stdio.h>
#define SMOOTH_DEBUG(msg) \
      printf("smooth_axis: " msg "\n")
#define SMOOTH_DEBUGF(fmt, ...) \
      printf("smooth_axis: " fmt "\n", ##__VA_ARGS__)
#endif

#else
// Logging disabled
#define SMOOTH_DEBUG(msg) \
    ((void)0)
#define SMOOTH_DEBUGF(fmt, ...) \
    ((void)0)
#endif

// ============================================================================
// Usage Examples
// ============================================================================
//
// ASSERTIONS (catch programming errors):
//   SMOOTH_AXIS_ASSERT(axis != NULL, "axis is NULL");
//   SMOOTH_AXIS_ASSERT(cfg->mode == AUTO_DT, "wrong mode");
//   SMOOTH_AXIS_ASSERT(cfg->now_ms != NULL, "AUTO mode requires now_ms");
//
// LOGGING (understand runtime behavior):
//   Simple message:
//     SMOOTH_DEBUG("warmup started");
//     SMOOTH_DEBUG("warmup complete");
//
//   With values:
//     SMOOTH_DEBUGF("warmup: cycles=%u dt_avg=%.2fms", cycles, dt_avg);
//     SMOOTH_DEBUGF("alpha=%.4f noise=%.3f", alpha, noise);
//     SMOOTH_DEBUGF("threshold: %.3f -> %.3f", old_thresh, new_thresh);
//
//   State transitions:
//     SMOOTH_DEBUG("warmup -> ready");
//     SMOOTH_DEBUGF("mode switch: %s -> %s", old_mode, new_mode);
//
//   Performance:
//     SMOOTH_DEBUGF("update took %lu us", micros_elapsed);
//
// When to use which:
//   ASSERT: Programming error (NULL, wrong mode, invalid config)
//   DEBUG:  Normal runtime event (state change, computed value)