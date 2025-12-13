/**
 * @file smooth_axis.c
 * @brief Implementation of adaptive sensor smoothing
 * @author Jonatan Vider
 * @date 30/11/2025
 *
 * See smooth_axis.h for API documentation.
 */

#include "smooth_axis.h"
#include "smooth_axis_debug.h"
#include <math.h>

// ----------------------------------------------------------------------------
// Internal Constants
// ----------------------------------------------------------------------------
static const uint16_t SMOOTH_AXIS_INIT_CALIBRATION_CYCLES = 256;

// Clamp measured dt during AUTO warmup to avoid pathological cases
static const float SMOOTH_AXIS_AUTO_DT_MIN_MS = 0.1f;   // 10,000 Hz max
static const float SMOOTH_AXIS_AUTO_DT_MAX_MS = 50.0f;  // 20 Hz min
static const float FALLBACK_DELTA_TIME        = 0.016f; // 60 Hz assumption before warmup



// ----------------------------------------------------------------------------
// Default "Feel" Parameters (normalized to 1023 ADC range)
// ----------------------------------------------------------------------------

static const float CANONICAL_MAX = 1023.0f; // Reference resolution scale
static const float FULL_OFF_U    = 0.0f;    // No dead zone by default
static const float FULL_ON_U     = 1023.0f; // No dead zone by default
static const float STICKY_U      = 3.0f;    // ~0.3% magnetic zone
static const float MAX_THRESH_U  = 30.0f;    // ~2.9% upper threshold limit

// EMA convergence: 5% remaining = "settled" (Reached 95%)
static const float EMA_CONVERGENCE_THRESHOLD = 0.05f;

// Noise estimation: slow EMA for stable noise floor tracking
static const float NOISE_SMOOTHING_RATE = 0.005f;

// Dynamic threshold headroom: threshold = 3.5x noise estimate
static const float THRESHOLD_NOISE_MULTIPLIER = 3.5f;

//Prevent floor/ceiling overlap (0.5 =< would be ambiguous)
static const float MAX_STICKY_ZONE = 0.49f;


// ============================================================================
// Inline Math Utilities
// ============================================================================

static inline float clamp_f(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline float clamp_f_0_1(float x) {
    return clamp_f(x, 0.0f, 1.0f);
}

static inline float abs_f(float x) {
    return x < 0.0f ? -x : x;
}

// Exponential Moving Average: out = (1-α)·old + α·new
static inline float ema(float old, float new, float alpha) {
    return (1.0f - alpha) * old + alpha * new;
}

// Linear interpolation: map x from [in_min, in_max] to [out_min, out_max]
static inline float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max == in_min) {
        return out_min;  // Degenerate case: avoid division by zero
    }
    float t = (x - in_min) / (in_max - in_min);
    return out_min + t * (out_max - out_min);
}

// Scale the threshold inversely with settle_time (longer settle times allows lower threshold)
static inline float compute_dyn_scale(float settle_time_sec) {
    const float t_ref = 0.1f;  // Reference settle time (100ms)
    float       ratio = settle_time_sec / t_ref;
    if (ratio < 1.0f) { ratio = 1.0f; }
    return 1.0f / ratio;  // Linear inverse scaling
}

static inline float sign_of(const float residual) {
    return (residual > 0.0f) ? 1.0f : (residual < 0.0f) ? -1.0f : 0.0f;
}

// Noise detection heuristic: true movement has consistent sign, noise flips randomly
static inline bool has_sign_flipped(const float current, const float previous) {
    float r_sign    = sign_of(current);
    float last_sign = sign_of(previous);
    return r_sign != last_sign || (r_sign == 0.0f && last_sign == 0.0f);
}


// ============================================================================
// Input Pipeline Helpers
// ============================================================================


// Apply library defaults from canonical constants (normalized to user's max_raw)
static void set_default_config(smooth_axis_config_t *cfg, uint16_t max_raw) {
    cfg->max_raw          = max_raw ? max_raw : 1;
    cfg->full_off_norm    = clamp_f_0_1(FULL_OFF_U / CANONICAL_MAX);
    cfg->full_on_norm     = clamp_f_0_1(FULL_ON_U / CANONICAL_MAX);
    cfg->sticky_zone_norm = clamp_f(STICKY_U / CANONICAL_MAX, 0.0f, MAX_STICKY_ZONE);
}

// Normalize raw ADC [0..max_raw] to [0..1], with full_off/full_on dead zone clipping
static float input_norm(const smooth_axis_t *axis, uint16_t raw_value) {
    uint16_t max_raw = axis->cfg.max_raw;
    if (max_raw == 0) { max_raw = 1; }  // Safety: avoid division by zero
    
    float norm = (float)raw_value / (float)max_raw;
    norm = clamp_f_0_1(norm);
    
    float off = axis->cfg.full_off_norm;
    float on  = axis->cfg.full_on_norm;
    if (on <= off) {  // Degenerate config: treat as full range
        off = 0.0f;
        on  = 1.0f;
    }
    
    // Clip to dead zones, then re-stretch to [0..1]
    norm = clamp_f(norm, off, on);
    norm = map_f(norm, off, on, 0.0f, 1.0f);
    return norm;
}



// ============================================================================
// Output Pipeline Helpers
// ============================================================================

// True if normalized delta exceeds 1 LSB in integer output (prevents sub-quantum updates)
static bool would_change_output(const smooth_axis_t *axis, float diff) {
    float max_raw = axis->cfg.max_raw ? (float)axis->cfg.max_raw : 1.0f;
    float epsilon = 1.0f / max_raw;  // One LSB in normalized space
    return diff > epsilon;
}

// Dynamic threshold: scales with noise level, clamped to [1x .. 10x] of base threshold
static float get_dynamic_threshold(const smooth_axis_t *axis) {
    float dynamic_threshold      = THRESHOLD_NOISE_MULTIPLIER * axis->_noise_estimate_norm;
    float settle_time_attenuated = dynamic_threshold * axis->cfg._threshold_attenuation;
    return clamp_f(settle_time_attenuated, 0.0f, MAX_THRESH_U / CANONICAL_MAX);
}

// Apply sticky zones: endpoints snap to exact 0.0/1.0, middle region re-stretched to [0..1]
static float apply_sticky_margins(float axis_position, float sticky_margin_size) {
    sticky_margin_size = clamp_f(sticky_margin_size, 0.0f, MAX_STICKY_ZONE);
    
    float sticky_floor   = sticky_margin_size;
    float sticky_ceiling = 1.0f - sticky_margin_size;
    
    // Snap to endpoints if inside sticky zones
    if (axis_position <= sticky_floor) { return 0.0f; }
    if (axis_position >= sticky_ceiling) { return 1.0f; }
    
    // Re-stretch middle region to fill [0..1]
    axis_position =
            map_f(axis_position, 0.0f, 1.0f, -sticky_margin_size, 1.0f + sticky_margin_size);
    return clamp_f_0_1(axis_position);
}

// Get nominal output after smoothing + sticky zone processing
static float get_normalized(const smooth_axis_t *axis) {
    if (!axis || !axis->_has_first_sample) {
        return 0.0f;
    }
    return apply_sticky_margins(axis->_smoothed_norm, axis->cfg.sticky_zone_norm);
}


// ============================================================================
// EMA Math
// ============================================================================

// Compute decay rate k such that: after settle_time_sec, error reduces to 5%
// Formula: k = ln(0.05) / settle_time  →  alpha(dt) = 1 - exp(k·dt)
static float compute_ema_decay_rate(const float settle_time_sec) {
    if (settle_time_sec <= 0.0f) {
        return 0.0f;
    }
    // (ln(0) = NAN , ln(1) = 0 (No decay)
    float residual = clamp_f(EMA_CONVERGENCE_THRESHOLD, 1e-4f, 0.9999f);
    float ln_r     = logf(residual);  // negative
    return ln_r / settle_time_sec;
}

// Convert decay rate k and time step dt into EMA alpha.
// alpha = 1 - exp(k·dt), clamped for numerical stability
static float get_alpha_from_dt(const float k, const float dt_sec) {
    if (dt_sec > 0.0f && k != 0.0f) {
        float ratio = clamp_f(k * dt_sec, -20.0f, 0.0f);  // Prevent overflow
        return 1.0f - expf(ratio);
    }
    // Fallback: instant response (no smoothing)
    // Legitimate cases: k (settle_time) = 0 , or dt=0 (first frame)
    // Bug case: negative dt (should assert in debug)
    SMOOTH_AXIS_ASSERT(dt_sec >= 0.0f || k == 0.0f,
                       "negative dt is invalid");
    return 1.0f;
}

// ============================================================================
// Warmup (AUTO_DT Mode)
// ============================================================================

static bool is_warmup_finished(const smooth_axis_t *axis) {
    return axis->_warmup_cycles_done >= SMOOTH_AXIS_INIT_CALIBRATION_CYCLES;
}

// Measure average dt over 256 samples, then compute fixed alpha (AUTO_DT only)
static void auto_run_warmup_cycle_if_needed(smooth_axis_t *axis) {
    if (is_warmup_finished(axis)) { return; }
    
    SMOOTH_AXIS_CHECK_RETURN(axis->cfg.now_ms != NULL, "AUTO mode requires now_ms function");
    
    uint32_t now_ms = axis->cfg.now_ms();
    if (axis->_last_time_ms == 0) {
        axis->_last_time_ms = now_ms;  // First call: just record timestamp
        return;
    }
    
    // Measure dt and accumulate
    float dt_ms = (float)(now_ms - axis->_last_time_ms);
    axis->_last_time_ms = now_ms;
    dt_ms = clamp_f(dt_ms, SMOOTH_AXIS_AUTO_DT_MIN_MS, SMOOTH_AXIS_AUTO_DT_MAX_MS);
    
    float dt_sec = dt_ms / 1000.0f;
    axis->_dt_accum_sec += dt_sec;
    axis->_warmup_cycles_done++;
    
    
    // Warmup complete: compute fixed alpha from average dt
    if (is_warmup_finished(axis)) {
        float dt_avg = axis->_dt_accum_sec / (float)axis->_warmup_cycles_done;
        axis->_auto_alpha = get_alpha_from_dt(axis->cfg._ema_decay_rate, dt_avg);
        
        SMOOTH_DEBUGF("warmup complete: cycles=%u dt_avg=%.2fms alpha=%.4f",
                      axis->_warmup_cycles_done,
                      dt_avg * 1000.0f,
                      axis->_auto_alpha);
    }
}

// Set initial smoothed value from first raw sample (skip EMA on frame 0)
static bool initialize_on_first_sample(smooth_axis_t *axis, float norm) {
    if (axis->_has_first_sample) {
        return false;  // Already initialized
    }
    
    axis->_has_first_sample = true;
    axis->_smoothed_norm    = norm;
    
    SMOOTH_DEBUGF("first sample: norm=%.3f", norm);
    return true;
}


// ============================================================================
// Core Update Logic
// ============================================================================

// Track noise level via sign-flip detection: noise oscillates around signal, movement is directional
static void update_noise_estimate(smooth_axis_t *axis, const float current_residual) {
    bool is_noise_sample = has_sign_flipped(current_residual, axis->_last_residual);
    axis->_last_residual = current_residual;
    
    /* === Sign Flip Discrimination ===
    Sign flip → likely noise (update estimate). No flip → likely movement (decay estimate). */
    float new_sample = is_noise_sample ? abs_f(current_residual) : 0.0f;
    
    float old_noise = axis->_noise_estimate_norm;
    
    axis->_noise_estimate_norm = ema(axis->_noise_estimate_norm, new_sample, NOISE_SMOOTHING_RATE);
    axis->_noise_estimate_norm = clamp_f_0_1(axis->_noise_estimate_norm);
    
    
    // Debug: log significant noise changes
    float noise_change = abs_f(axis->_noise_estimate_norm - old_noise);
    if (noise_change > 0.01f) {
        SMOOTH_DEBUGF("noise: %.4f -> %.4f %s",
                      old_noise,
                      axis->_noise_estimate_norm,
                      is_noise_sample ? "(spike)" : "(settling)");
    }
}

// Apply EMA smoothing + update noise estimate
static void update_core(smooth_axis_t *axis, uint16_t raw_value, float alpha) {
    float norm = input_norm(axis, raw_value);
    if (initialize_on_first_sample(axis, norm)) { return; }
    
    float diff = norm - axis->_smoothed_norm;
    axis->_smoothed_norm += alpha * diff; // EMA: x += α·(target - x)
    
    update_noise_estimate(axis, diff);
}


// ============================================================================
// Public API - Configuration
// ============================================================================

void smooth_axis_config_auto_dt(smooth_axis_config_t *cfg,
                                uint16_t max_raw,
                                float settle_time_sec,
                                smooth_axis_now_ms_fn now_ms) {
    SMOOTH_AXIS_CHECK_RETURN(cfg != NULL, "config is NULL");
    SMOOTH_AXIS_CHECK_RETURN(now_ms != NULL, "AUTO mode requires now_ms function");
    
    set_default_config(cfg, max_raw);
    cfg->mode                   = SMOOTH_AXIS_MODE_AUTO_DT;
    cfg->settle_time_sec        = settle_time_sec;
    cfg->now_ms                 = now_ms;
    cfg->_ema_decay_rate        = compute_ema_decay_rate(settle_time_sec);
    cfg->_threshold_attenuation = compute_dyn_scale(settle_time_sec);
}

void smooth_axis_config_live_dt(smooth_axis_config_t *cfg,
                                uint16_t max_raw,
                                float settle_time_sec) {
    SMOOTH_AXIS_CHECK_RETURN(cfg != NULL, "config is NULL");
    
    set_default_config(cfg, max_raw);
    cfg->mode                   = SMOOTH_AXIS_MODE_LIVE_DT;
    cfg->settle_time_sec        = settle_time_sec;
    cfg->_ema_decay_rate        = compute_ema_decay_rate(settle_time_sec);
    cfg->_threshold_attenuation = compute_dyn_scale(settle_time_sec);
}

void smooth_axis_init(smooth_axis_t *axis, const smooth_axis_config_t *cfg) {
    SMOOTH_AXIS_CHECK_RETURN(axis != NULL, "axis is NULL");
    SMOOTH_AXIS_CHECK_RETURN(cfg != NULL, "config is NULL");
    SMOOTH_AXIS_CHECK_RETURN(cfg->mode != SMOOTH_AXIS_MODE_AUTO_DT || cfg->now_ms != NULL,
                             "AUTO mode requires now_ms function");
    
    axis->cfg                  = *cfg;
    axis->_smoothed_norm       = 0.0f;
    axis->_noise_estimate_norm = 0.01f;  // Initial noise floor estimate
    axis->_last_reported_norm  = 0.0f;
    // 60 Hz assumption until warmup
    axis->_auto_alpha          = get_alpha_from_dt(cfg->_ema_decay_rate, FALLBACK_DELTA_TIME);
    axis->_dt_accum_sec        = 0.0f;
    axis->_warmup_cycles_done  = 0;
    axis->_last_time_ms        = 0;
    axis->_last_residual       = 0.0f;
    axis->_has_first_sample    = false;
    
    SMOOTH_DEBUGF("init: mode=%s max_raw=%u settle_time=%.3fs",
                  cfg->mode == SMOOTH_AXIS_MODE_AUTO_DT ? "AUTO_DT" : "LIVE_DT",
                  cfg->max_raw,
                  cfg->settle_time_sec);
}

void smooth_axis_reset(smooth_axis_t *axis, uint16_t raw_value) {
    SMOOTH_AXIS_CHECK_RETURN(axis != NULL, "axis is NULL");
    
    float norm = raw_value ? input_norm(axis, raw_value) : 0.0f;
    
    axis->_smoothed_norm       = norm;
    axis->_noise_estimate_norm = 0.01f;
    axis->_last_reported_norm  = norm;
    axis->_last_residual       = 0.0f;
    axis->_has_first_sample    = raw_value ? true : false;
}


// ============================================================================
// Public API - Update
// ============================================================================

void smooth_axis_update_auto_dt(smooth_axis_t *axis, uint16_t raw_value) {
    SMOOTH_AXIS_CHECK_RETURN(axis != NULL, "axis is NULL");
    SMOOTH_AXIS_CHECK_RETURN(axis->cfg.mode == SMOOTH_AXIS_MODE_AUTO_DT,
                             "wrong mode: use update_live_dt() for LIVE_DT mode");
    
    auto_run_warmup_cycle_if_needed(axis);
    
    update_core(axis, raw_value, axis->_auto_alpha);  // Fixed alpha after warmup
}

void smooth_axis_update_live_dt(smooth_axis_t *axis, uint16_t raw_value, float dt_sec) {
    SMOOTH_AXIS_CHECK_RETURN(axis != NULL, "axis is NULL");
    SMOOTH_AXIS_CHECK_RETURN(axis->cfg.mode == SMOOTH_AXIS_MODE_LIVE_DT,
                             "wrong mode: use update_auto_dt() for AUTO_DT mode");
    
    float live_alpha = get_alpha_from_dt(axis->cfg._ema_decay_rate,
                                         dt_sec); // Recompute alpha each frame
    update_core(axis, raw_value, live_alpha);
}

// ============================================================================
// Public API - Output & Query
// ============================================================================

float smooth_axis_get_norm(const smooth_axis_t *axis) {
    return get_normalized(axis);
}

uint16_t smooth_axis_get_u16(const smooth_axis_t *axis) {
    if (!axis) { return 0; }
    
    float max_out = (float)axis->cfg.max_raw;
    float n       = get_normalized(axis);
    
    // Ensure exact 0 and max_raw at endpoints (prevent off-by-one from floating point rounding)
    if (n <= 1.0f / max_out) { return 0; }
    if (n >= (max_out - 1.0) / max_out) { return axis->cfg.max_raw; }
    
    return (uint16_t)lroundf(n * (float)max_out);
}

bool smooth_axis_has_new_value(smooth_axis_t *axis) {
    SMOOTH_AXIS_CHECK_RETURN_VAL(axis != NULL, "axis is NULL", false);
    if (!axis->_has_first_sample) { return false; }
    
    float current = get_normalized(axis);
    float diff    = abs_f(current - axis->_last_reported_norm);
    
    if (!would_change_output(axis, diff)) { return false; }
    
    // When approaching to the edges, we treat each movement (>= epsilon) as 'Always Important'
    float sticky_ceil   = 1 - axis->cfg.sticky_zone_norm;
    float sticky_floor  = axis->cfg.sticky_zone_norm;
    bool in_sticky_zone = (current < sticky_floor) || (current > sticky_ceil);
    
    float dynamic_threshold = get_dynamic_threshold(axis);  // Scales 1x-10x with noise
    
    if (in_sticky_zone || diff > dynamic_threshold) {
        axis->_last_reported_norm = current;
        
        SMOOTH_DEBUGF("new value: %.3f (diff=%.4f thresh=%.4f %s)",
                      current,
                      diff,
                      dynamic_threshold,
                      in_sticky_zone ? "sticky" : "normal");
        return true;
    }
    return false;
}

// ============================================================================
// Public API - Introspection / diagnostics
// ============================================================================

float smooth_axis_get_noise_norm(const smooth_axis_t *axis) {
    return axis ? axis->_noise_estimate_norm : 0.0f;
}

float smooth_axis_get_effective_thresh_norm(const smooth_axis_t *axis) {
    return axis ? get_dynamic_threshold(axis) : 0.0f;
}

uint16_t smooth_axis_get_effective_thresh_u16(const smooth_axis_t *axis) {
    if (!axis || axis->cfg.max_raw == 0) {
        return 0;
    }
    float threshold_norm = get_dynamic_threshold(axis);
    if (threshold_norm <= 0.0f) {
        return 0;
    }
    float threshold_scaled = threshold_norm * (float)axis->cfg.max_raw + 0.5f; // Round to nearest
    threshold_scaled = clamp_f(threshold_scaled, 0.0f, (float)axis->cfg.max_raw);
    return (uint16_t)threshold_scaled;
}
