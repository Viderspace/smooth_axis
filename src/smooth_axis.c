//
// Created by Jonatan Vider on 30/11/2025.
//


// smooth_axis.c
#include "smooth_axis.h"
#include <math.h>
#include <stdio.h>

// -----------------------------------------------------------------------------
// Default "feel" parameters
// -----------------------------------------------------------------------------
//
// If you want to change the global feel of all axes, this is the place.
//
#define SMOOTH_AXIS_RESIDUAL 0.05f  // 95% settled at settle_time_sec

static const float CANONICAL_MAX = 1023.0f;
static const float FULL_OFF_U = 10.0f;
static const float FULL_ON_U = 1013.0f;
static const float STICKY_U = 5.0f;
static const float MOVE_THRESH_U = 4.0f;
static const float BETA = 0.005f;
static const float K = 1.5;

// -----------------------------------------------------------------------------
// Config-level helpers
// -----------------------------------------------------------------------------

// Given settle_time_sec, compute K such that:
//   residual = exp(K * T)  →  K = ln(residual) / T
// and per update: alpha = 1 - exp(K * dt_sec)

static inline float _compute_settle_k(float settle_time_sec) {
  if (settle_time_sec <= 0.0f) {
    return 0.0f;
  }

  float residual = SMOOTH_AXIS_RESIDUAL;
  if (residual < 1e-4f) residual = 1e-4f;
  if (residual > 0.9999f) residual = 0.9999f;

  float ln_r = logf(residual);  // negative
  return ln_r / settle_time_sec;
}

void smooth_axis_default_config(smooth_axis_config_t *cfg, uint16_t max_raw) {
  if (!cfg) return;

  cfg->max_raw = max_raw ? max_raw : 1;

  cfg->full_off_norm = FULL_OFF_U / CANONICAL_MAX;
  cfg->full_on_norm = FULL_ON_U / CANONICAL_MAX;
  cfg->sticky_zone_norm = STICKY_U / CANONICAL_MAX;
  cfg->movement_thresh_norm = MOVE_THRESH_U / CANONICAL_MAX;

  cfg->mode = SMOOTH_AXIS_MODE_LIVE_DT;
  cfg->settle_time_sec = 0.25f;  // ~250 ms to ~95% settled
  cfg->_settle_k = _compute_settle_k(cfg->settle_time_sec);
}

void smooth_axis_default_config_auto(smooth_axis_config_t *cfg,
                                     uint16_t max_raw,
                                     float settle_time_sec) {
  if (!cfg) return;

  smooth_axis_default_config(cfg, max_raw);

  cfg->mode = SMOOTH_AXIS_MODE_AUTO_DT;
  cfg->settle_time_sec = settle_time_sec;

  // _settle_k unused in AUTO_DT
  cfg->_settle_k = 0.0f;
}

void smooth_axis_default_config_live_dt(smooth_axis_config_t *cfg,
                                        uint16_t max_raw,
                                        float settle_time_sec) {
  if (!cfg) return;

  smooth_axis_default_config(cfg, max_raw);
  cfg->mode = SMOOTH_AXIS_MODE_LIVE_DT;
  cfg->settle_time_sec = settle_time_sec;
  cfg->_settle_k = _compute_settle_k(settle_time_sec);
}

void smooth_axis_init(smooth_axis_t *axis, const smooth_axis_config_t *cfg) {
  if (!axis || !cfg) return;

  axis->cfg = *cfg;

  axis->_smoothed_norm = 0.0f;
  axis->_dev_norm = 0.0f;
  axis->_last_reported_norm = 0.0f;
  axis->_initialized = false;

  axis->_auto_alpha_ready = false;
  axis->_auto_alpha = 1.0f;  // teleport until AUTO finishes warm-up
  axis->_dt_accum_sec = 0.0f;
  axis->_dt_sample_count = 0;
  axis->_last_time_ms = 0;
}



// -----------------------------------------------------------------------------
// 1) Normalization helpers
// -----------------------------------------------------------------------------
static inline float _clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float _map_ranges(float x, float inMin, float inMax,
                                float outMin, float outMax) {
  if (inMax == inMin) {
    return outMin;  // avoid div by zero; degenerate case
  }
  float t = (x - inMin) / (inMax - inMin);
  return outMin + t * (outMax - outMin);
}

// Normalize raw input and apply full_off/full_on clipping + re-lerp to [0..1]
static float _input_norm(const smooth_axis_t *axis, uint16_t raw_value) {
  if (!axis) return 0.0f;

  uint16_t max_raw = axis->cfg.max_raw;
  if (max_raw == 0) {
    max_raw = 1;
  }

  float norm = (float)raw_value / (float)max_raw;
  norm = _clampf(norm, 0.0f, 1.0f);

  float off = axis->cfg.full_off_norm;
  float on = axis->cfg.full_on_norm;

  off = _clampf(off, 0.0f, 1.0f);
  on = _clampf(on, 0.0f, 1.0f);
  if (on <= off) {
    off = 0.0f;
    on = 1.0f;
  }

  norm = _clampf(norm, off, on);
  norm = _map_ranges(norm, off, on, 0.0f, 1.0f);
  return norm;
}


static float _get_dynamic_thresh(const smooth_axis_t *axis) {
  float base_thresh = axis->cfg.movement_thresh_norm;
  float dyn_thresh = K * axis->_dev_norm;
  float eff_thresh = _clampf(dyn_thresh, base_thresh, 10*base_thresh);
  return eff_thresh;
}

// -----------------------------------------------------------------------------
// 2) Sticky / nominal helpers
// -----------------------------------------------------------------------------

// Sticky endpoints + re-lerp the middle part:
//   - 0..sticky_zone_norm      → hard 0
//   - 1-sticky_zone_norm..1    → hard 1
//   - [sticky .. 1-sticky]     → re-lin-mapped back to [0..1]
static float _get_nominal_norm(const smooth_axis_t *axis) {
  if (!axis || !axis->_initialized) {
    return 0.0f;
  }

  float v = axis->_smoothed_norm;
  float z = axis->cfg.sticky_zone_norm;

  z = _clampf(z, 0.0f, 0.49f);

  if (v <= z) {
    return 0.0f;
  }
  if (v >= 1.0f - z) {
    return 1.0f;
  }

  float inMin = z;
  float inMax = 1.0f - z;
  return _map_ranges(v, inMin, inMax, 0.0f, 1.0f);
}

// -----------------------------------------------------------------------------
// 3) EMA + AUTO helpers
// -----------------------------------------------------------------------------
#ifndef SMOOTH_AXIS_INIT_CALIBRATION_CYCLES
#define SMOOTH_AXIS_INIT_CALIBRATION_CYCLES 4096
#endif

#define SMOOTH_AXIS_AUTO_DT_MIN_MS 0.1f
#define SMOOTH_AXIS_AUTO_DT_MAX_MS 50.0f

static inline void _apply_ema(smooth_axis_t *axis,
                              float norm,
                              float alpha) {
  if (!axis) return;

  if (!axis->_initialized) {
    axis->_smoothed_norm = norm;
    axis->_initialized = true;
    return;
  }

  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;

  axis->_smoothed_norm =
      (1.0f - alpha) * axis->_smoothed_norm + alpha * norm;
}

static float _compute_auto_alpha(float settle_time_sec,
                                 float dt_avg_sec) {
  const float residual = 0.05f;  // 95% of final at T
  if (settle_time_sec <= 0.0f || dt_avg_sec <= 0.0f) {
    return 1.0f;  // no smoothing fallback
  }

  float ln_r = logf(residual);  // negative
  float steps = settle_time_sec / dt_avg_sec;

  if (steps < 1.0f) {
    return 1.0f;
  }

  float ln_1_minus_a = ln_r / steps;
  float one_minus_a = expf(ln_1_minus_a);
  float alpha = 1.0f - one_minus_a;

  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  return alpha;
}

// AUTO warm-up: measure frame dt, accumulate average, then derive _auto_alpha.
static void _auto_warmup_step(smooth_axis_t *axis) {
  if (!axis) return;
  if (axis->_auto_alpha_ready) return;

  uint32_t now_ms = smooth_axis_now_ms();

  if (axis->_last_time_ms == 0) {
    axis->_last_time_ms = now_ms;
    return;
  }

  float dt_ms = (float)(now_ms - axis->_last_time_ms);
  axis->_last_time_ms = now_ms;

  if (dt_ms < SMOOTH_AXIS_AUTO_DT_MIN_MS) dt_ms = SMOOTH_AXIS_AUTO_DT_MIN_MS;
  if (dt_ms > SMOOTH_AXIS_AUTO_DT_MAX_MS) dt_ms = SMOOTH_AXIS_AUTO_DT_MAX_MS;

  float dt_sec = dt_ms / 1000.0f;

  axis->_dt_accum_sec += dt_sec;
  axis->_dt_sample_count++;


  if (axis->_dt_sample_count >= SMOOTH_AXIS_INIT_CALIBRATION_CYCLES) {
    float dt_avg = axis->_dt_accum_sec / (float)axis->_dt_sample_count;

    float alpha = _compute_auto_alpha(axis->cfg.settle_time_sec, dt_avg);

    axis->_auto_alpha = alpha;
    axis->_auto_alpha_ready = true;
  }
}



// -----------------------------------------------------------------------------
// 4) Core update
// -----------------------------------------------------------------------------

void smooth_axis_update_raw(smooth_axis_t *axis, uint16_t raw_value) {
  if (!axis) return;

  // AUTO-only by design
  if (axis->cfg.mode != SMOOTH_AXIS_MODE_AUTO_DT) {
    // User chose LIVE_DT but called the AUTO update path: ignore.
    return;
  }

  float norm = _input_norm(axis, raw_value);

  _auto_warmup_step(axis);

  float alpha = axis->_auto_alpha;
  _apply_ema(axis, norm, alpha);

  float error = fabsf(norm - axis->_smoothed_norm);
  axis->_dev_norm += BETA * (error - axis->_dev_norm);
}

void smooth_axis_update_raw_dt(smooth_axis_t *axis,
                               uint16_t raw_value,
                               float dt_sec) {
  if (!axis) return;

  float norm = _input_norm(axis, raw_value);

  if (!axis->_initialized) {
    axis->_smoothed_norm = norm;
    axis->_initialized = true;
    return;
  }

  float a = 0.0f;

  if (axis->cfg.mode == SMOOTH_AXIS_MODE_LIVE_DT) {
    float K = axis->cfg._settle_k;

    if (dt_sec > 0.0f && K != 0.0f) {
      float ratio = K * dt_sec;
      if (ratio < -20.0f) ratio = -20.0f;
      if (ratio > 0.0f) ratio = 0.0f;

      a = 1.0f - expf(ratio);
    } else {
      a = 1.0f;
    }
  } else if (axis->cfg.mode == SMOOTH_AXIS_MODE_AUTO_DT) {
    // For now, treat AUTO_DT dt-based updates same as LIVE_DT.
    float K = axis->cfg._settle_k;
    if (dt_sec > 0.0f && K != 0.0f) {
      float ratio = K * dt_sec;
      if (ratio < -20.0f) ratio = -20.0f;
      if (ratio > 0.0f) ratio = 0.0f;
      a = 1.0f - expf(ratio);
    } else {
      a = 1.0f;
    }
  }

  a = _clampf(a, 0.0f, 1.0f);

  axis->_smoothed_norm = (1.0f - a) * axis->_smoothed_norm + a * norm;

  float error = fabsf(norm - axis->_smoothed_norm);
  axis->_dev_norm += BETA * (error - axis->_dev_norm);
  axis->_dev_norm = _clampf(axis->_dev_norm, 0.0f, 1.0f);
}

// -----------------------------------------------------------------------------
// 5) Output + change detection
// -----------------------------------------------------------------------------

float smooth_axis_get_norm(const smooth_axis_t *axis) {
  return _get_nominal_norm(axis);
}

uint16_t smooth_axis_get_u16(const smooth_axis_t *axis, uint16_t max_out) {
  if (!axis) return 0;
  float n = _get_nominal_norm(axis);
  if (n <= 0.0f) return 0;
  if (n >= 1.0f) return max_out;
  return (uint16_t)(n * (float)max_out + 0.5f);
}

uint16_t smooth_axis_get_u16_full(const smooth_axis_t *axis) {
  if (!axis) return 0;
  return smooth_axis_get_u16(axis, axis->cfg.max_raw);
}

bool smooth_axis_has_new_value(smooth_axis_t *axis) {
  if (!axis || !axis->_initialized) {
    return false;
  }

  float current = _get_nominal_norm(axis);
  float last = axis->_last_reported_norm;
  float diff = current - last;
  if (diff < 0.0f) diff = -diff;

  // If the change is smaller than one quantization step in norm space,
  // it cannot change the integer output → ignore.
  uint16_t max_raw = axis->cfg.max_raw ? axis->cfg.max_raw : 1;
  float epsilon = 1.0f / (float)max_raw;
  if (diff <= epsilon) {
    return false;
  }

  bool force_edge =
      (current < axis->cfg.full_off_norm) || (current > axis->cfg.full_on_norm);

  float dyn_thresh  = _get_dynamic_thresh(axis);
  // printf("%f ",dyn_thresh);

  if (force_edge || diff > dyn_thresh) {
    axis->_last_reported_norm = current;
    return true;
  }

  return false;
}


// -----------------------------------------------------------------------------
// Introspection / diagnostics
// -----------------------------------------------------------------------------

float smooth_axis_get_noise_norm(const smooth_axis_t *axis) {
  return axis ? axis->_dev_norm : 0.0f;
}


float smooth_axis_get_effective_thresh_norm(const smooth_axis_t *axis) {
  return axis ? _get_dynamic_thresh (axis) : 0.0f;
}

uint16_t smooth_axis_get_effective_thresh_u(const smooth_axis_t *axis) {
  if (!axis || axis->cfg.max_raw == 0) return 0;
  float t_norm = _get_dynamic_thresh(axis);
  if (t_norm <= 0.0f) return 0;
  float val = t_norm * (float)axis->cfg.max_raw + 0.5f;
  if (val < 0.0f) val = 0.0f;
  if (val > 65535.0f) val = 65535.0f;
  return (uint16_t)val;
}