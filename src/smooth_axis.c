//
// Created by Jonatan Vider on 30/11/2025.
//


// smooth_axis.c
#include "smooth_axis.h"
#include <math.h>

// -----------------------------------------------------------------------------
// Default "feel" parameters
// -----------------------------------------------------------------------------
//
// If you want to change the global feel of all axes, this is the place.
//

// Compile-time switches for experimentation

static const float CANONICAL_MAX = 1023.0f;
static const float FULL_OFF_U    = 0.0f;
static const float FULL_ON_U     = 1023.0f;
static const float STICKY_U      = 2.0f;
static const float MOVE_THRESH_U = 3.0f;

static const float SMOOTH_AXIS_RESIDUAL = 0.05f;

static const float BETA = 0.005f; //0.005f;
static const float K    = 2.0f;




// ─────────────────────────────────────────────────────────────
// Internal function prototypes
// ─────────────────────────────────────────────────────────────

/*
 * TODO - REMOVE FWD DECLARATION AFTER TESTING
 */

static float devnorm_signflip_update(smooth_axis_t *axis, float residual);

static float apply_settle_time_scaling(const smooth_axis_t *axis,
                                       float dyn_term,
                                       float base_thresh);

// ─────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────
static inline float clamp_it(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline float clamp_it_0_1(float x) {
    return clamp_it(x, 0.0f, 1.0f);
}

static inline float map_it(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max == in_min) {
        return out_min;  // avoid div by zero; degenerate case
    }
    
    float t = (x - in_min) / (in_max - in_min);
    return out_min + t * (out_max - out_min);
}




// ─────────────────────────────────────────────────────────────
// Config-level helpers
// ─────────────────────────────────────────────────────────────

// Given settle_time_sec, compute K such that:
//   residual = exp(K * T)  →  K = ln(residual) / T
// and per update: alpha = 1 - exp(K * dt_sec)

static float compute_settle_k(float settle_time_sec) {
    if (settle_time_sec <= 0.0f) {
        return 0.0f;
    }
    
    float residual = clamp_it(SMOOTH_AXIS_RESIDUAL, 1e-4f, 0.9999f);
    float ln_r     = logf(residual);  // negative
    return ln_r / settle_time_sec;
}

static void set_default_config(smooth_axis_config_t *cfg, uint16_t max_raw) {
    if (!cfg) {
        return;
    }
    
    cfg->max_raw = max_raw ? max_raw : 1;
    
    cfg->full_off_norm        = FULL_OFF_U / CANONICAL_MAX;
    cfg->full_on_norm         = FULL_ON_U / CANONICAL_MAX;
    cfg->sticky_zone_norm     = STICKY_U / CANONICAL_MAX;
    cfg->movement_thresh_norm = MOVE_THRESH_U / CANONICAL_MAX;
    
    cfg->mode            = SMOOTH_AXIS_MODE_LIVE_DT;
    cfg->settle_time_sec = 0.25f;  // ~250 ms to ~95% settled
    cfg->_settle_k       = compute_settle_k(cfg->settle_time_sec);
}

// Normalize raw input and apply full_off/full_on clipping + re-lerp to [0..1]
static float input_norm(const smooth_axis_t *axis, uint16_t raw_value) {
    uint16_t max_raw = axis->cfg.max_raw;
    
    if (max_raw == 0) {
        max_raw = 1;
    }
    
    float norm = (float)raw_value / (float)max_raw;
    norm = clamp_it_0_1(norm);
    
    float off = clamp_it_0_1(axis->cfg.full_off_norm);
    float on  = clamp_it_0_1(axis->cfg.full_on_norm);
    
    if (on <= off) {
        off = 0.0f;
        on  = 1.0f;
    }
    
    norm = clamp_it(norm, off, on);
    norm = map_it(norm, off, on, 0.0f, 1.0f);
    return norm;
}

//static float _get_dynamic_thresh(const smooth_axis_t *axis) {
//  float base_thresh = axis->cfg.movement_thresh_norm;
//  float dyn_thresh = K * axis->_dev_norm;
//  float eff_thresh = clamp_it(dyn_thresh, base_thresh, 10*base_thresh);
//  return eff_thresh;
//}

static float get_dynamic_thresh(const smooth_axis_t *axis) {
    const float base_thresh = axis->cfg.movement_thresh_norm;
    
    // Raw dynamic term driven only by _dev_norm
    const float dyn_thresh = K * axis->_dev_norm;

//   Let the helper handle settle_time scaling + clamping
    return apply_settle_time_scaling(axis, dyn_thresh, base_thresh);
}

// ---------------------------------------------------------------------------
// 2) Sticky / nominal helpers
// ---------------------------------------------------------------------------

// Sticky endpoints + re-lerp the middle part:
//   - 0..sticky_zone_norm      → hard 0
//   - 1-sticky_zone_norm..1    → hard 1
//   - [sticky .. 1-sticky]     → re-lin-mapped back to [0..1]
static float get_nominal_norm(const smooth_axis_t *axis) {
    
    if (!axis || !axis->_initialized) {
        return 0.0f;
    }
    
    float v = axis->_smoothed_norm;
    float z = axis->cfg.sticky_zone_norm;
    
    z = clamp_it(z, 0.0f, 0.49f);
    
    if (v <= z) {
        return 0.0f;
    }
    if (v >= 1.0f - z) {
        return 1.0f;
    }
    
    float inMin = z;
    float inMax = 1.0f - z;
    return map_it(v, inMin, inMax, 0.0f, 1.0f);
}

// -----------------------------------------------------------------------------
// 3) EMA + AUTO helpers
// -----------------------------------------------------------------------------
#ifndef SMOOTH_AXIS_INIT_CALIBRATION_CYCLES
#define SMOOTH_AXIS_INIT_CALIBRATION_CYCLES 4096
#endif

#define SMOOTH_AXIS_AUTO_DT_MIN_MS 0.1f
#define SMOOTH_AXIS_AUTO_DT_MAX_MS 50.0f

static inline void apply_ema(smooth_axis_t *axis, float norm, float alpha) {
    if (!axis->_initialized) {
        axis->_smoothed_norm = norm;
        axis->_initialized   = true;
        return;
    }
    
    if (alpha < 0.0f) {
        alpha = 0.0f;
    }
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    
    axis->_smoothed_norm =
            (1.0f - alpha) * axis->_smoothed_norm + alpha * norm;
}

static float compute_auto_alpha(float settle_time_sec, float dt_avg_sec) {
    const float residual = 0.05f;  // 95% of final at T
    if (settle_time_sec <= 0.0f || dt_avg_sec <= 0.0f) { return 1.0f; }// no smoothing fallback
    
    float ln_r  = logf(residual);  // negative
    float steps = settle_time_sec / dt_avg_sec;
    
    if (steps < 1.0f) { return 1.0f; }
    
    float ln_1_minus_a = ln_r / steps;
    float one_minus_a  = expf(ln_1_minus_a);
    float alpha        = 1.0f - one_minus_a;
    
    return clamp_it_0_1(alpha);
}

// AUTO warm-up: measure frame dt, accumulate average, then derive _auto_alpha.
static void auto_warmup_step(smooth_axis_t *axis) {
    
    if (axis->_auto_alpha_ready) {
        return;
    }
    
    uint32_t now_ms = smooth_axis_now_ms();
    
    if (axis->_last_time_ms == 0) {
        axis->_last_time_ms = now_ms;
        return;
    }
    
    float dt_ms = (float)(now_ms - axis->_last_time_ms);
    axis->_last_time_ms = now_ms;
    
    if (dt_ms < SMOOTH_AXIS_AUTO_DT_MIN_MS) {
        dt_ms = SMOOTH_AXIS_AUTO_DT_MIN_MS;
    }
    if (dt_ms > SMOOTH_AXIS_AUTO_DT_MAX_MS) {
        dt_ms = SMOOTH_AXIS_AUTO_DT_MAX_MS;
    }
    
    float dt_sec = dt_ms / 1000.0f;
    
    axis->_dt_accum_sec += dt_sec;
    axis->_dt_sample_count++;
    
    if (axis->_dt_sample_count >= SMOOTH_AXIS_INIT_CALIBRATION_CYCLES) {
        float dt_avg = axis->_dt_accum_sec / (float)axis->_dt_sample_count;
        
        float alpha = compute_auto_alpha(axis->cfg.settle_time_sec, dt_avg);
        
        axis->_auto_alpha       = alpha;
        axis->_auto_alpha_ready = true;
    }
}



/*
 *  NEW HELPERS FOR CORRECTING NOISE SENSITIVITY =========================
 */


// -----------------------------------------------------------------------------
// Helper 1: scale dynamic threshold based on settle_time_sec
// -----------------------------------------------------------------------------
static float
apply_settle_time_scaling(const smooth_axis_t *axis, float dyn_term,
                          float base_thresh) {
    // Reference settle time for "normal" behavior
    const float t_ref = 0.2f; // 200 ms ~= "snappy"
    
    float ratio = axis->cfg.settle_time_sec / t_ref;
    
    // Don't *increase* sensitivity for very fast configs.
    if (ratio < 1.0f) {
        ratio = 1.0f;
    }
    
    // Softer dynamic term for long settle times:
    //   t = 0.20 → weight ~ 1.0
    //   t = 0.50 → ~0.63
    //   t = 1.00 → ~0.45
    float dynamic_weight = 1.0f / sqrtf(ratio);
    
    float scaled = dyn_term * dynamic_weight;
    
    // Clamp relative to base threshold as before
    float eff = clamp_it(scaled, base_thresh, 10.0f * base_thresh);
    return eff;
}

// -----------------------------------------------------------------------------
// Helper 2: update _dev_norm using sign-flip gating
//
// residual = raw_norm - smoothed_norm
// BETA     = smoothing factor for the noise estimate
// -----------------------------------------------------------------------------
static inline float sign_of(float residual) {
    return (residual > 0.0f) ? 1.0f : (residual < 0.0f) ? -1.0f : 0.0f;
}

static float devnorm_signflip_update(smooth_axis_t *axis, float residual) {
    // Current & previous sign of the residual
    float r_sign    = sign_of(residual);
    float last_sign = sign_of(axis->_last_residual);
    
    bool  sign_flip = (r_sign != last_sign
                       || (r_sign == 0.0f && last_sign == 0.0f) // zero noise
    );
    
    axis->_last_residual = residual;
    
    if (!sign_flip) {
        return (1.0f - BETA) * axis->_dev_norm;
    }
    
    // else: the last 2 samples had different sign (strong indication of noise)
//  axis->_last_residual = residual;
    
    float sample = fabsf(residual);
    
    // Only "trust" noise samples when sign is flipping;
    // treat monotonic residuals (ramp lag) as mostly non-noise.
    float dev = axis->_dev_norm;
    dev = (1.0f - BETA) * dev + BETA * sample;
    return clamp_it_0_1(dev);
}




// ─────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────

void smooth_axis_default_config_auto(smooth_axis_config_t *cfg, uint16_t max_raw,
                                     float settle_time_sec) {
    if (!cfg) {
        return;
    }
    
    set_default_config(cfg, max_raw);
    
    cfg->mode            = SMOOTH_AXIS_MODE_AUTO_DT;
    cfg->settle_time_sec = settle_time_sec;
    
    // _settle_k unused in AUTO_DT
    cfg->_settle_k = 0.0f;
}

void smooth_axis_default_config_live_dt(smooth_axis_config_t *cfg,
                                        uint16_t max_raw,
                                        float settle_time_sec) {
    if (!cfg) {
        return;
    }
    
    set_default_config(cfg, max_raw);
    cfg->mode            = SMOOTH_AXIS_MODE_LIVE_DT;
    cfg->settle_time_sec = settle_time_sec;
    cfg->_settle_k       = compute_settle_k(settle_time_sec);
}

void smooth_axis_init(smooth_axis_t *axis, const smooth_axis_config_t *cfg) {
    if (!axis || !cfg) {
        return;
    }
    
    axis->cfg = *cfg;
    
    axis->_smoothed_norm      = 0.0f;
    axis->_dev_norm           = 0.0f;
    axis->_last_reported_norm = 0.0f;
    axis->_initialized        = false;
    
    axis->_auto_alpha_ready = false;
    axis->_auto_alpha       = 1.0f;  // teleport until AUTO finishes warm-up
    axis->_dt_accum_sec     = 0.0f;
    axis->_dt_sample_count  = 0;
    axis->_last_time_ms     = 0;
    
    /* todo - Consider removing or keeping after testing*/
    axis->_last_residual = 0.0f;
}




// -----------------------------------------------------------------------------
// 4) Core update
// -----------------------------------------------------------------------------

void smooth_axis_update_auto(smooth_axis_t *axis, uint16_t raw_value) {
    if (!axis) {
        return;
    }
    
    // AUTO-only by design
    if (axis->cfg.mode != SMOOTH_AXIS_MODE_AUTO_DT) {
        // User chose LIVE_DT but called the AUTO update path: ignore.
        return;
    }
    
    float norm = input_norm(axis, raw_value);
    
    auto_warmup_step(axis);
    
    float alpha = axis->_auto_alpha;
    apply_ema(axis, norm, alpha);
    
    float error = fabsf(norm - axis->_smoothed_norm);
    axis->_dev_norm += BETA * (error - axis->_dev_norm);
}

inline float ema_live_dt(const smooth_axis_t *axis, const float dt) {
    float k = axis->cfg._settle_k;
    
    if (dt > 0.0f && k != 0.0f) {
        float ratio = clamp_it(k * dt, -20.0f, 0.0f);
        return 1.0f - expf(ratio);
    }
    return 1.0f;
}

void smooth_axis_update_live_dt(smooth_axis_t *axis, uint16_t raw_value, float dt_sec) {
    if (!axis) {
        return;
    }
    
    float norm = input_norm(axis, raw_value);
    
    // TODO - consider scrapping this whole if case or error handle it properly
    if (!axis->_initialized) {
        axis->_smoothed_norm = norm;
        axis->_initialized   = true;
        axis->_dev_norm      = 0.005f;  //0.0f;
        axis->_last_residual = 0.00f;
        return;
    }
    
    float a = 0.0f;
    
    if (axis->cfg.mode == SMOOTH_AXIS_MODE_LIVE_DT) {
        a = ema_live_dt(axis, dt_sec);
        
        // fixme - Is this an appropiate fallback or redundant?
    } else if (axis->cfg.mode == SMOOTH_AXIS_MODE_AUTO_DT) {
        // For now, treat AUTO_DT dt-based updates same as LIVE_DT.
        a = ema_live_dt(axis, dt_sec);
    }
    
    a = clamp_it_0_1(a);
    axis->_smoothed_norm = (1.0f - a) * axis->_smoothed_norm + a * norm;
    
    // Residual between raw and smoothed signal
    float residual = norm - axis->_smoothed_norm;
    axis->_dev_norm = devnorm_signflip_update(axis, residual);
}

// -----------------------------------------------------------------------------
// 5) Output + change detection
// -----------------------------------------------------------------------------

float smooth_axis_get_norm(const smooth_axis_t *axis) {
    return get_nominal_norm(axis);
}

uint16_t smooth_axis_get_u16(const smooth_axis_t *axis, uint16_t max_out) {
    if (!axis) {
        return 0;
    }
    float n = get_nominal_norm(axis);
    
    // clear edges are treated simple
    if (n <= 0.0f) { return 0; }
    if (n >= 1.0f) { return max_out; }
    
    return (uint16_t)(n * (float)max_out + 0.5f);
}

// fixme - why is this functino exists
uint16_t smooth_axis_get_u16_full(const smooth_axis_t *axis) {
    if (!axis) {
        return 0;
    }
    return smooth_axis_get_u16(axis, axis->cfg.max_raw);
}

bool smooth_axis_has_new_value(smooth_axis_t *axis) {
    if (!axis || !axis->_initialized) {
        return false;
    }
    
    float current = get_nominal_norm(axis);
    float last    = axis->_last_reported_norm;
    float diff    = current - last;
    
    if (diff < 0.0f) { diff = -diff; }
    
    // If the change is smaller than one quantization step in norm space,
    // it cannot change the integer output → ignore.
    uint16_t max_raw = axis->cfg.max_raw ? axis->cfg.max_raw : 1;
    float    epsilon = 1.0f / (float)max_raw;
    if (diff <= epsilon) {
        return false;
    }
    
    // todo - consider the reuse of full_off/on here conceptually
    bool force_edge = (current < axis->cfg.full_off_norm)
                      || (current > axis->cfg.full_on_norm);
    
    float dyn_thresh = get_dynamic_thresh(axis);
    
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
    return axis ? get_dynamic_thresh(axis) : 0.0f;
}

uint16_t smooth_axis_get_effective_thresh_u(const smooth_axis_t *axis) {
    if (!axis || axis->cfg.max_raw == 0) {
        return 0;
    }
    float t_norm = get_dynamic_thresh(axis);
    if (t_norm <= 0.0f) {
        return 0;
    }
    float val = t_norm * (float)axis->cfg.max_raw + 0.5f;
    val = clamp_it(val, 0.0f, 65535.0f);
    
    return (uint16_t)val;
}

