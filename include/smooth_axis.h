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
// Typical usage:
//
//   smooth_axis_config_t cfg;
//   smooth_axis_t        axis;
//
//   smooth_axis_default_config_auto(&cfg, 1023, 0.25f, my_now_ms);
//   smooth_axis_init(&axis, &cfg);
//
//   for (;;) {
//       uint16_t raw = read_adc();
//       smooth_axis_update_auto(&axis, raw);
//       if (smooth_axis_has_new_value(&axis)) {
//           uint16_t new_value = smooth_axis_get_u16_full(&axis);
//           do_something_when_the_value_changed(new_value);
//           ...
//       }
//   }

// ----------------------------------------------------------------------------
// Modes
// ----------------------------------------------------------------------------
typedef enum {
  // Use smooth_axis_update_live_deltatime()
  SMOOTH_AXIS_MODE_LIVE_DT = 0,
  
  // Use smooth_axis_update_auto(), dt inferred during warm-up
  SMOOTH_AXIS_MODE_AUTO_DT,
} smooth_axis_mode_t;


// ----------------------------------------------------------------------------
// Time source type (for AUTO mode)
// ----------------------------------------------------------------------------
//
// AUTO mode needs a monotonically increasing millisecond counter.
// The user provides this function pointer when building the config.
//
// Example (QMK):
//
//   uint32_t qmk_now_ms(void) { return timer_read32(); }
//
//   smooth_axis_default_config_auto(&cfg, 1023, 0.25f, qmk_now_ms);
//
typedef uint32_t (*smooth_axis_now_ms_fn)(void);

// ----------------------------------------------------------------------------
// Config (user-facing)
// ----------------------------------------------------------------------------
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
  //   - LIVE_DT:  smooth_axis_default_config_live_deltatime() + smooth_axis_update_live_deltatime()
  //
  smooth_axis_mode_t mode;
  float settle_time_sec; // time to ~95% settled after a step
  
  
  // --- Time source for AUTO mode -------------------------------------------
  //
  // Required for SMOOTH_AXIS_MODE_AUTO_DT.
  // Ignored for SMOOTH_AXIS_MODE_LIVE_DT.
  //
  smooth_axis_now_ms_fn now_ms;
  
/*
 _ema_rate:
 Internal exponential rate constant derived from settle_time_sec.
 Used to compute alpha(dt) = 1 - exp(_ema_rate * dt).
 Larger magnitude = faster convergence. Negative value.
 */
  float _ema_rate;           // internal
  float _dyn_scale; // NEW: Pre-calculated dynamic threshold scaler
  
} smooth_axis_config_t;

// ----------------------------------------------------------------------------
// Runtime state
// ----------------------------------------------------------------------------
typedef struct smooth_axis_t {
  smooth_axis_config_t cfg;
  
  // Internal runtime state (underscore = not for user code)
  float    _smoothed_norm;        // EMA result in normalized 0.0 .. 1.0
  float _dev_norm;
  float _last_reported_norm;   // last "sent" nominal value
  bool _has_first_sample;
  
  
  // Noise / movement heuristics
  float _last_residual;        // previous (raw_norm - smoothed_norm)
  
  // AUTO_DT internal state
  float    _dt_accum_sec;
  uint16_t _warmup_cycles_done;
  uint32_t _last_time_ms;
  float    _auto_alpha;
  

} smooth_axis_t;

// ----------------------------------------------------------------------------
// Config helpers + init
// ----------------------------------------------------------------------------

// Pattern 1: fixed-step AUTO (no dt, most QMK users).
//   - mode            = SMOOTH_AXIS_MODE_AUTO_DT
//   - settle_time_sec = desired settle time
//   - now_ms          = user-provided millisecond timer
//
// now_ms MUST be non-NULL if you plan to call smooth_axis_update_auto().
void smooth_axis_default_config_auto(smooth_axis_config_t *cfg,
                                     uint16_t max_raw,
                                     float settle_time_sec,
                                     smooth_axis_now_ms_fn now_ms);

// Pattern 2: dt-aware LIVE smoothing (for jitter robustness).
//   - mode            = SMOOTH_AXIS_MODE_LIVE_DT
//   - settle_time_sec = desired settle time
//   - _ema_rate       = derived from settle_time_sec
void smooth_axis_default_config_live_deltatime(smooth_axis_config_t *cfg,
                                               uint16_t max_raw,
                                               float settle_time_sec);


// Initialize axis state from config (copies cfg into axis->cfg).
void smooth_axis_init(smooth_axis_t *axis, const smooth_axis_config_t *cfg);

// ----------------------------------------------------------------------------
// Core update API (call one of these each loop)
// ----------------------------------------------------------------------------

// AUTO-only update:
//   - Intended for SMOOTH_AXIS_MODE_AUTO_DT
//   - Uses a fixed alpha chosen during AUTO warm-up
//   - If cfg.mode != AUTO_DT, this does nothing.
//   - If cfg.now_ms == NULL, behavior is undefined (no time source).
void smooth_axis_update_auto(smooth_axis_t *axis, uint16_t raw_value);

// dt-aware update:
//   - Intended for SMOOTH_AXIS_MODE_LIVE_DT
//   - Also works for AUTO_DT as a dt-aware path (same math).
void smooth_axis_update_live_deltatime(smooth_axis_t *axis,
                                       uint16_t raw_value,
                                       float dt_sec);

// ----------------------------------------------------------------------------
// Output + change detection
// ----------------------------------------------------------------------------

// Nominal normalized value after sticky-end processing.
float smooth_axis_get_norm(const smooth_axis_t *axis);

// Map nominal value to [0 .. max_out] (e.g. joystick / LED range).
uint16_t smooth_axis_get_u16(smooth_axis_t *axis);

// Convenience: map back to cfg.max_raw.
uint16_t smooth_axis_get_u16_full(const smooth_axis_t *axis);

// "Has something meaningful changed since last time?"
// Semantics: big enough movement OR forced at edges.
bool smooth_axis_has_new_value(smooth_axis_t *axis);

// ----------------------------------------------------------------------------
// Introspection / diagnostics
// ----------------------------------------------------------------------------

// Return the current movement threshold in raw units (same scale as max_raw).
uint16_t smooth_axis_get_effective_thresh_u(const smooth_axis_t *axis);

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


// -----------------------------------------------------------------------------
// TODO (docs):
// Add note about settle-time accuracy:
//
// - With clean input (no noise), smooth_axis reaches ~95% of target in
//   settle_time_sec with typical error around ±4%.
// - Under heavy noise, timing MAPE increases (~20–25%), but monotonicity
//   and false-update protection remain perfect (0 false updates in tests).
//
// Emphasize in docs that the algorithm prioritizes stability and correctness
// over exact settle-time matching when noise is present.
// -----------------------------------------------------------------------------

// TODO (tests/docs):
// Add a clean step-response test:
//   input: 100,100,...,100, 900,900,...,900 (fixed dt)
// Use this to measure true EMA settle-time (95% of step).
// Expect even lower MAPE (<4%) vs the current 1s ramp tests,
// since the target is static after the jump and timing isn't
// distorted by the ramp shape itself.