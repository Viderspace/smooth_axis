//
// Created by Jonatan Vider on 30/11/2025.
//

// smooth_axis.h
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Overview
//
// This module turns a noisy analog axis (slider / potentiometer) into a clean,
// normalized value with:
//   - dt-aware EMA smoothing
//   - sticky ends at 0 and 1
//   - "meaningful change" detection
//
// Choose ONE of these usage patterns:
//
//   Pattern 1: fixed-rate loop (most QMK users, no dt, "good enough"):
//
//     smooth_axis_config_t cfg;
//     smooth_axis_t        axis;
//
//     smooth_axis_default_config_auto(&cfg, 1023, 0.25f); // 10-bit, ~250ms
//     smooth_axis_init(&axis, &cfg);
//
//     // in your loop:
//     uint16_t raw = read_adc();
//     smooth_axis_update_auto(&axis, raw);
//     uint16_t clean = smooth_axis_get_u16_full(&axis);
//
//   Pattern 2: variable dt / jitter-robust smoothing:
//
//     smooth_axis_config_t cfg;
//     smooth_axis_t        axis;
//
//     smooth_axis_default_config_live_dt(&cfg, 1023, 0.25f);
//     smooth_axis_init(&axis, &cfg);
//
//     // in your loop:
//     float    dt_sec = ...;        // seconds since last sample
//     uint16_t raw    = read_adc();
//     smooth_axis_update_live_dt(&axis, raw, dt_sec);
//     uint16_t clean  = smooth_axis_get_u16_full(&axis);
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Modes
// -----------------------------------------------------------------------------
typedef enum {
  // Use smooth_axis_update_live_dt()
  SMOOTH_AXIS_MODE_LIVE_DT = 0,
  
  // Use smooth_axis_update_auto(), dt inferred during warm-up
  SMOOTH_AXIS_MODE_AUTO_DT,
} smooth_axis_mode_t;

// -----------------------------------------------------------------------------
// Config (user-facing)
// -----------------------------------------------------------------------------
typedef struct {
  // --- Required: input range -----------------------------------------------
  uint16_t max_raw;          // ADC max (e.g. 1023, 4095, 65535...)
  
  // --- Optional: feel tuning (normalized 0.0 .. 1.0) -----------------------
  float full_off_norm;       // below this: "near off" (clipped)
  float full_on_norm;        // above this: "near full" (clipped)
  float sticky_zone_norm;    // symmetric sticky zone at both ends
  float movement_thresh_norm;// min nominal delta for "meaningful change"
  
  // --- Smoothing mode + main user knob -------------------------------------
  //
  // Choose ONE:
  //   - AUTO_DT:  smooth_axis_default_config_auto() + smooth_axis_update_auto()
  //   - LIVE_DT:  smooth_axis_default_config_live_dt() + smooth_axis_update_live_dt()
  //
  smooth_axis_mode_t mode;
  float              settle_time_sec; // time to ~95% settled after a step
  
  // --- Internal derived coefficient (do not touch) -------------------------
  //
  // In LIVE_DT mode we use:
  //   alpha = 1 - exp(_settle_k * dt_sec)
  //
  float _settle_k;           // internal
  
} smooth_axis_config_t;

// -----------------------------------------------------------------------------
// Runtime state
// -----------------------------------------------------------------------------
typedef struct smooth_axis_t {
  smooth_axis_config_t cfg;
  
  // Internal runtime state (underscore = not for user code)
  float    _smoothed_norm;        // EMA result in normalized 0.0 .. 1.0
  float    _dev_norm;
  float    _last_reported_norm;   // last "sent" nominal value
  bool _initialized;
  
  // AUTO_DT internal state
  bool _auto_alpha_ready;
  float    _dt_accum_sec;
  uint16_t _dt_sample_count;
  uint32_t _last_time_ms;
  float    _auto_alpha;

/* fixme - temporary internal fields for noise / movement heuristics */
  float
          _last_residual;        // previous (raw_norm - smoothed_norm) (for sign-flip gating)
    
} smooth_axis_t;

// -----------------------------------------------------------------------------
// Time hook for AUTO mode
// -----------------------------------------------------------------------------
//
// If you use SMOOTH_AXIS_MODE_AUTO_DT, you must provide this function in
// your project (e.g. thin wrapper around QMK's timer):
//
//   uint32_t smooth_axis_now_ms(void) {
//       return timer_read32();
//   }
//
uint32_t smooth_axis_now_ms(void);

// -----------------------------------------------------------------------------
// Config helpers + init
// -----------------------------------------------------------------------------

// Base defaults for the given ADC resolution.
//
// Sets sensible zones and movement behavior, then:
//   mode            = SMOOTH_AXIS_MODE_LIVE_DT
//   settle_time_sec = 0.25f
//   _settle_k       = derived from settle_time_sec
static void set_default_config(smooth_axis_config_t *cfg, uint16_t max_raw);

// Pattern 1: fixed-step AUTO (no dt, most QMK users).
//   - mode            = SMOOTH_AXIS_MODE_AUTO_DT
//   - settle_time_sec = desired settle time
void smooth_axis_default_config_auto(smooth_axis_config_t *cfg,
                                     uint16_t max_raw,
                                     float settle_time_sec);

// Pattern 2: dt-aware LIVE smoothing (for jitter robustness).
//   - mode            = SMOOTH_AXIS_MODE_LIVE_DT
//   - settle_time_sec = desired settle time
//   - _settle_k       = derived from settle_time_sec
void smooth_axis_default_config_live_dt(smooth_axis_config_t *cfg,
                                        uint16_t max_raw,
                                        float settle_time_sec);

// Initialize axis state from config (copies cfg into axis->cfg).
void smooth_axis_init(smooth_axis_t *axis, const smooth_axis_config_t *cfg);

// -----------------------------------------------------------------------------
// Core update API (call one of these each loop)
// -----------------------------------------------------------------------------

// AUTO-only update:
//   - Intended for SMOOTH_AXIS_MODE_AUTO_DT
//   - Uses a fixed alpha chosen during AUTO warm-up
//   - If cfg.mode != AUTO_DT, this does nothing.
void smooth_axis_update_auto(smooth_axis_t *axis, uint16_t raw_value);

// dt-aware update:
//   - Intended for SMOOTH_AXIS_MODE_LIVE_DT
//   - Also works for AUTO_DT as a dt-aware path (same math).
void smooth_axis_update_live_dt(smooth_axis_t *axis,
                                uint16_t raw_value,
                                float dt_sec);

// Return the current movement threshold in raw units (same scale as max_raw).
uint16_t smooth_axis_get_effective_thresh_u(const smooth_axis_t *axis);

// -----------------------------------------------------------------------------
// Output + change detection
// -----------------------------------------------------------------------------

// Nominal normalized value after sticky-end processing.
float smooth_axis_get_norm(const smooth_axis_t *axis);

// Map nominal value to [0 .. max_out] (e.g. joystick / LED range).
uint16_t smooth_axis_get_u16(const smooth_axis_t *axis, uint16_t max_out);

// Convenience: map back to cfg.max_raw.
uint16_t smooth_axis_get_u16_full(const smooth_axis_t *axis);

// "Has something meaningful changed since last time?"
// Semantics: big enough movement OR forced at edges.
bool smooth_axis_has_new_value(smooth_axis_t *axis);

// -----------------------------------------------------------------------------
// Introspection / diagnostics
// -----------------------------------------------------------------------------

// Current noise estimate in normalized units (0.0 .. 1.0).
// This is the internal _dev_norm value used for dynamic thresholding.
float smooth_axis_get_noise_norm(const smooth_axis_t *axis);

// Current effective movement threshold in normalized units (0.0 .. 1.0).
// This includes dynamic scaling based on noise and clamping to [1x .. 10x]
// of cfg.movement_thresh_norm.
float smooth_axis_get_effective_thresh_norm(const smooth_axis_t *axis);

#ifdef __cplusplus
}
#endif