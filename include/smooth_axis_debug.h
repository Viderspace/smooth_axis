// smooth_axis_debug.h
// Created by Jonatan Vider on 07/12/2024.
//
// Debug instrumentation for smooth_axis library.
//
// This header provides two types of debugging aids:
//
// 1. ASSERTIONS (Contract Validation)
//    - Catch programming errors during development
//    - Enabled in debug builds, compiled to nothing in release builds
//    - Use for: NULL pointers, mode mismatches, invalid state
//
// 2. DEBUG LOGGING (Observability)
//    - Understand system behavior and performance
//    - Optional feature, can be enabled even in release builds
//    - Use for: state transitions, computed values, performance metrics
//
// USAGE - ASSERTIONS:
//   SMOOTH_AXIS_ASSERT(ptr != NULL, "pointer cannot be NULL");
//   SMOOTH_AXIS_ASSERT(axis->cfg.mode == AUTO_DT, "wrong mode");
//
// USAGE - DEBUG LOGGING:
//   SMOOTH_DEBUG("warmup complete");
//   SMOOTH_DEBUGF("alpha=%.4f dt_avg=%.2fms", alpha, dt_ms);
//
// To enable debug logging, define SMOOTH_AXIS_DEBUG_ENABLE=1 in your build.
//

#pragma once

#include <stddef.h>  // For NULL

// ============================================================================
// SECTION 1: Assertions (Contract Validation)
// ============================================================================
//
// Assertions validate contracts and catch programming bugs during development.
// They are controlled by NDEBUG (standard C convention):
//   - Debug builds (NDEBUG not defined):   Assertions are active, halt on failure
//   - Release builds (NDEBUG defined):     Assertions compile to nothing (zero cost)
//
// Use assertions for conditions that indicate bugs in user code:
//   - NULL pointers passed to functions
//   - Wrong mode (calling update_auto_dt in LIVE_DT mode)
//   - Missing required configuration (now_ms function in AUTO mode)
//   - Using uninitialized state
//

#ifndef NDEBUG

#include <assert.h>

#define SMOOTH_AXIS_ASSERT(condition, message) \
      assert((condition) && (message))
#else
#define SMOOTH_AXIS_ASSERT(condition, message) \
      ((void)0)
#endif

// ============================================================================
// Smart Check Macros (Assert in Debug, Guard in Release)
// ============================================================================
//
// These macros do DIFFERENT things in debug vs release:
//
// DEBUG BUILD (NDEBUG not defined):
//   - If condition fails: Program CRASHES with assert
//   - If condition passes: Continue execution
//
// RELEASE BUILD (NDEBUG defined):
//   - If condition fails: EARLY RETURN from function
//   - If condition passes: Continue execution
//

#ifndef NDEBUG
// Debug: Just assert (will crash if fails)
#define SMOOTH_AXIS_CHECK_RETURN(condition, message) \
      SMOOTH_AXIS_ASSERT(condition, message)

#define SMOOTH_AXIS_CHECK_RETURN_VAL(condition, message, retval) \
      SMOOTH_AXIS_ASSERT(condition, message)
#else
// Release: Check and return early if fails
#define SMOOTH_AXIS_CHECK_RETURN(condition, message) \
      do { if (!(condition)) { return; } } while(0)
  
#define SMOOTH_AXIS_CHECK_RETURN_VAL(condition, message, retval) \
      do { if (!(condition)) { return (retval); } } while(0)
#endif

// ============================================================================
// SECTION 2: Debug Logging (Observability)
// ============================================================================
//
// Debug logging helps understand system behavior and performance.
// Unlike assertions, logging does NOT terminate - it reports and continues.
//
// Use debug logging for:
//   - State transitions (warmup started, warmup complete)
//   - Computed values (alpha, noise estimates, dynamic thresholds)
//   - Performance metrics (timing, sample rates)
//   - Normal runtime events that help troubleshooting
//
// Debug logging is DISABLED by default (zero overhead).
// To enable, define SMOOTH_AXIS_DEBUG_ENABLE=1 in your build system:
//
// QMK (rules.mk):
//   CONSOLE_ENABLE = yes
//   OPT_DEFS += -DSMOOTH_AXIS_DEBUG_ENABLE=1
//
// Arduino (platformio.ini):
//   build_flags = -DSMOOTH_AXIS_DEBUG_ENABLE=1
//
// Arduino IDE:
//   Add #define SMOOTH_AXIS_DEBUG_ENABLE 1 before #include "smooth_axis.h"
//

#if defined(SMOOTH_AXIS_DEBUG_ENABLE) && SMOOTH_AXIS_DEBUG_ENABLE

// ============================================================================
// QMK Platform
// ============================================================================
#if defined(QMK_KEYBOARD_H)

#ifdef CONSOLE_ENABLE
    // QMK with console enabled: use uprintf()
#include "print.h"
    
    // Simple message logging
#define SMOOTH_DEBUG(msg) \
        uprintf("smooth_axis: " msg "\n")
    
    // Formatted message logging
    // Note: QMK's uprintf supports printf-style formatting
#define SMOOTH_DEBUGF(fmt, ...) \
        uprintf("smooth_axis: " fmt "\n", ##__VA_ARGS__)
        
#else
    // QMK without console: logging disabled
#define SMOOTH_DEBUG(msg) \
        ((void)0)
#define SMOOTH_DEBUGF(fmt, ...) \
        ((void)0)
#endif

// ============================================================================
// Arduino Platform
// ============================================================================
#elif defined(ARDUINO)

  // Arduino: use Serial
  // Check if Serial is ready before logging to avoid hangs
  
#define SMOOTH_DEBUG(msg) \
      do { \
          if (Serial) { \
              Serial.print("smooth_axis: "); \
              Serial.println(msg); \
          } \
      } while(0)
  
  // Arduino supports printf-style formatting in newer cores
#define SMOOTH_DEBUGF(fmt, ...) \
      do { \
          if (Serial) { \
              Serial.print("smooth_axis: "); \
              Serial.printf(fmt, ##__VA_ARGS__); \
              Serial.println(); \
          } \
      } while(0)

// ============================================================================
// Native/Test Platform
// ============================================================================
#else

  // Native builds (unit tests, host development): use printf
#include <stdio.h>

#define SMOOTH_DEBUG(msg) \
      printf("smooth_axis: " msg "\n")
  
#define SMOOTH_DEBUGF(fmt, ...) \
      printf("smooth_axis: " fmt "\n", ##__VA_ARGS__)

#endif

// ============================================================================
// Debug Logging Disabled
// ============================================================================
#else

// No debug logging: macros compile to nothing (zero overhead)
#define SMOOTH_DEBUG(msg) \
      ((void)0)

#define SMOOTH_DEBUGF(fmt, ...) \
      ((void)0)

#endif // SMOOTH_AXIS_DEBUG_ENABLE

// ============================================================================
// Usage Examples (for reference)
// ============================================================================
//
// ASSERTIONS (catch bugs during development):
//   SMOOTH_AXIS_ASSERT(axis != NULL, "axis is NULL");
//   SMOOTH_AXIS_ASSERT(cfg->mode == AUTO_DT, "wrong mode");
//   SMOOTH_AXIS_ASSERT(cfg->now_ms != NULL, "AUTO mode requires now_ms");
//
// DEBUG LOGGING (understand system behavior):
//   Simple message:
//     SMOOTH_DEBUG("warmup started");
//     SMOOTH_DEBUG("warmup complete");
//
//   With values:
//     SMOOTH_DEBUGF("warmup complete: cycles=%u", cycles);
//     SMOOTH_DEBUGF("alpha=%.4f noise=%.3f", alpha, noise);
//     SMOOTH_DEBUGF("threshold: %.3f (base=%.3f)", dyn_thresh, base_thresh);
//
//   State transitions:
//     SMOOTH_DEBUG("entering warmup phase");
//     SMOOTH_DEBUG("warmup -> ready");
//
//   Performance monitoring:
//     SMOOTH_DEBUGF("update took %lu us", micros_elapsed);
//     SMOOTH_DEBUGF("avg dt: %.2f ms", avg_dt_ms);
//
// ============================================================================
// Quick Decision Guide: ASSERT vs DEBUG
// ============================================================================
//
// Use SMOOTH_AXIS_ASSERT when:
//   ❌ NULL pointer passed to function
//   ❌ Wrong mode (calling wrong update function)
//   ❌ Missing required config
//   ❌ Invalid state that indicates a programming bug
//   → Goal: Catch bugs in user code during development
//
// Use SMOOTH_DEBUG/DEBUGF when:
//   ℹ️ State transition happened
//   ℹ️ Computed value changed significantly
//   ℹ️ Performance metric to track
//   ℹ️ Normal event that helps troubleshooting
//   → Goal: Understand what the system is doing
//