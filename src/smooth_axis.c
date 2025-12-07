// smooth_axis.c
// Created by Jonatan Vider on 30/11/2025.
//
#include "smooth_axis.h"
#include "smooth_axis_debug.h"
#include <math.h>


//<editor-fold desc="Constants & #defines">
// ----------------------------------------------------------------------------
// Compile-time Configuration (Overridable)
// ----------------------------------------------------------------------------
#ifndef SMOOTH_AXIS_INIT_CALIBRATION_CYCLES
#define SMOOTH_AXIS_INIT_CALIBRATION_CYCLES 4096
#endif

// -----------------------------------------------------------------------------
// Internal Constants
// -----------------------------------------------------------------------------

// Internal clamps for AUTO warm-up
static const float SMOOTH_AXIS_AUTO_DT_MIN_MS = 0.1f;
static const float SMOOTH_AXIS_AUTO_DT_MAX_MS = 50.0f;
static const float FALLBACK_DELTA_TIME        = 0.016f; // 16 ms

// -----------------------------------------------------------------------------
// Default "feel" parameters
// -----------------------------------------------------------------------------
//
// If you want to change the global feel of all axes, this is the place.
//

// Compile-time switches for experimentation

static const float CANONICAL_MAX              = 1023.0f;
static const float FULL_OFF_U                 = 0.0f;
static const float FULL_ON_U                  = 1023.0f;
static const float STICKY_U                   = 3.0f;
static const float MOVE_THRESH_U              = 3.0f;
static const float EMA_CONVERGENCE_THRESHOLD  = 0.05f;
static const float NOISE_SMOOTHING_RATE       =
                           0.005f; // threshold sensitivity to noise fluctuations
static const float THRESHOLD_NOISE_MULTIPLIER =
                           3.5f; // Threshold headroom coefficient (1.0 = no headroom)

#define MAX_STICKY_ZONE 0.49f

//</editor-fold>

//<editor-fold desc="static-Inline tiny helpers">
// ─────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────
static inline float clamp_f(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline float clamp_f_0_1(float x) {
    return clamp_f(x, 0.0f, 1.0f);
}

static inline float abs_f(float x) { return x < 0.0f ? -x : x; }

static inline float ema(float old, float new, float alpha) {
    return (1.0f - alpha) * old + alpha * new;
}

static inline float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max == in_min) {
        return out_min;  // avoid div by zero; degenerate case
    }
    float t = (x - in_min) / (in_max - in_min);
    return out_min + t * (out_max - out_min);
}

static inline float compute_dyn_scale(float settle_time_sec) {
    const float t_ref = 0.1f;
    float       ratio = settle_time_sec / t_ref;
    if (ratio < 1.0f) { ratio = 1.0f; }
//    return 1.0f / sqrtf(ratio);
    return 1.0f / ratio;
}

static inline float sign_of(const float residual) {
    return (residual > 0.0f) ? 1.0f : (residual < 0.0f) ? -1.0f : 0.0f;
}

static inline bool has_sign_flipped(const float current, const float previous) {
    float r_sign    = sign_of(current);
    float last_sign = sign_of(previous);
    
    ///  the sign had flipped  || or completely static (zero noise edge case)
    return r_sign != last_sign || (r_sign == 0.0f && last_sign == 0.0f);
}
//</editor-fold>

//<editor-fold desc="Input pipeline helpers (input_norm, set_default_config">
// ─────────────────────────────────────────────────────────────
// Config-level helpers
// ─────────────────────────────────────────────────────────────

static void set_default_config(smooth_axis_config_t *cfg, uint16_t max_raw) {
    cfg->max_raw              = max_raw ? max_raw : 1;
    cfg->full_off_norm        = clamp_f_0_1(FULL_OFF_U / CANONICAL_MAX);
    cfg->full_on_norm         = clamp_f_0_1(FULL_ON_U / CANONICAL_MAX);
    cfg->sticky_zone_norm     = clamp_f(STICKY_U / CANONICAL_MAX, 0.0f, MAX_STICKY_ZONE);
    cfg->movement_thresh_norm = clamp_f_0_1(MOVE_THRESH_U / CANONICAL_MAX);
}

// Normalize raw input and apply full_off/full_on clipping + re-lerp to [0..1]
static float input_norm(const smooth_axis_t *axis, uint16_t raw_value) {
    uint16_t max_raw = axis->cfg.max_raw;
    if (max_raw == 0) { /// zero division protection (should never)
        max_raw = 1;
    }
    float norm = (float)raw_value / (float)max_raw;
    norm = clamp_f_0_1(norm);
    float off = axis->cfg.full_off_norm;
    float on  = axis->cfg.full_on_norm;
    if (on <= off) { /// corner case for safety
        off = 0.0f;
        on  = 1.0f;
    }
    norm      = clamp_f(norm, off, on);
    norm      = map_f(norm, off, on, 0.0f, 1.0f);
    return norm;
}
//</editor-fold>

//<editor-fold desc="Output Pipeline helpers">

/*
 * Internal helper of 'has_new_value()' API method
 */
static bool would_change_output(const smooth_axis_t *axis, float diff) {
    // If the change is smaller than one quantization step in norm space,
    // it cannot change the integer output → ignore.
    float max_raw = axis->cfg.max_raw ? (float)axis->cfg.max_raw : 1.0f;
    float epsilon = 1.0f / max_raw;
    
    return diff > epsilon;
}

static float get_dynamic_thresh(const smooth_axis_t *axis) {
    const float base_thresh = axis->cfg.movement_thresh_norm;
    
    // Raw dynamic term driven only by _noise_estimate_norm
    float dyn_thresh = THRESHOLD_NOISE_MULTIPLIER * axis->_noise_estimate_norm;
    float scaled     = dyn_thresh * axis->cfg._settle_time_scaler;
    return clamp_f(scaled, 0.0f, 10.0f * base_thresh);
}

// ---------------------------------------------------------------------------
// 2) Sticky / nominal helpers
// ---------------------------------------------------------------------------


// Sticky endpoints + re-lerp the middle part:
//   - 0..sticky_zone_norm      → hard 0
//   - 1-sticky_zone_norm..1    → hard 1
//   - [sticky .. 1-sticky]     → re-lin-mapped back to [0..1]
static float apply_sticky_margins(float axis_position, float sticky_margin_size) {
    
    /// why 0.49 ?  above >= 0.50 -> the floor and ceiling overlapping
    sticky_margin_size = clamp_f(sticky_margin_size, 0.0f, MAX_STICKY_ZONE);
    float sticky_ceiling = 1.0f - sticky_margin_size;
    float sticky_floor   = sticky_margin_size;
    if (axis_position >= sticky_ceiling) { return 1.0f; }
    if (axis_position <= sticky_floor) { return 0.0f; }
    axis_position =
            map_f(axis_position, 0.0f, 1.0f, -sticky_margin_size, 1.0f + sticky_margin_size);
    return clamp_f_0_1(axis_position);
}

static float get_normalized(const smooth_axis_t *axis) {
    if (!axis || !axis->_has_first_sample) {
        return 0.0f;
    }
    float axis_position = axis->_smoothed_norm;
//    return axis_position;
    return apply_sticky_margins(axis_position, axis->cfg.sticky_zone_norm);
}
//</editor-fold>

//<editor-fold desc="EMA Math/Logic">
// -----------------------------------------------------------------------------
// 3) EMA + AUTO helpers
// -----------------------------------------------------------------------------
/*
 * Key behaviour - This calculation only happens once (in both modes)
 *
 *  We calculate 'ema_rate' so that after settle_time_sec,
 *  the remaining distance to the target is reduced to 5% (EMA_CONVERGENCE_THRESHOLD).

 * The math behind:
 * Given a 'settle_time' which was selected by the user,
 * compute 'ema_rate' such that:
 * alpha(dt) = 1 - exp(ema_rate * dt) , s.t.  ema_rate = ln(EMA_CONVERGENCE_THRESHOLD) / settle_time
 */
static float compute_ema_decay_rate(const float settle_time_sec) {
    if (settle_time_sec <= 0.0f) {
        return 0.0f;
    }
    float residual = clamp_f(EMA_CONVERGENCE_THRESHOLD, 1e-4f, 0.9999f);
    float ln_r     = logf(residual);  // negative
    return ln_r / settle_time_sec;
}

static float ema_alpha_from_live_dt(const float k, const float dt_sec) {
    if (dt_sec > 0.0f && k != 0.0f) {
        float ratio = clamp_f(k * dt_sec, -20.0f, 0.0f);
        return 1.0f - expf(ratio);
    }
    return 1.0f; // no smoothing fallback
}
//</editor-fold>


//<editor-fold desc="Warmup (AUTO) / First sample setter">




static bool is_warmup_finished(const smooth_axis_t *axis) {
    return axis->_warmup_cycles_done >= SMOOTH_AXIS_INIT_CALIBRATION_CYCLES;
}

// AUTO warm-up: measure frame dt, accumulate average, then derive _auto_alpha.
static void auto_run_warmup_cycle_if_needed(smooth_axis_t *axis) {
    // Already calibrated - do nothing
    // Already calibrated - do nothing
    if (is_warmup_finished(axis)) { return; }
    
    SMOOTH_AXIS_CHECK_RETURN(axis->cfg.now_ms != NULL, "AUTO mode requires now_ms function");
    
    uint32_t now_ms = axis->cfg.now_ms();
    if (axis->_last_time_ms == 0) {
        axis->_last_time_ms = now_ms;
        return;
    }
    float dt_ms = (float)(now_ms - axis->_last_time_ms);
    axis->_last_time_ms = now_ms;
    dt_ms = clamp_f(dt_ms, SMOOTH_AXIS_AUTO_DT_MIN_MS, SMOOTH_AXIS_AUTO_DT_MAX_MS);
    float dt_sec = dt_ms / 1000.0f;
    axis->_dt_accum_sec += dt_sec;
    axis->_warmup_cycles_done++;
    
    // axis on auto mode should enter this block exactly once
    if (is_warmup_finished(axis)) {
        float dt_avg = axis->_dt_accum_sec / (float)axis->_warmup_cycles_done;
        axis->_auto_alpha = ema_alpha_from_live_dt(axis->cfg._ema_decay_rate, dt_avg);
        SMOOTH_DEBUGF("warmup complete: cycles=%u dt_avg=%.2fms alpha=%.4f",
                      axis->_warmup_cycles_done,
                      dt_avg * 1000.0f,
                      axis->_auto_alpha);
    }
}

static bool initialize_on_first_sample(smooth_axis_t *axis, float norm) {
    if (axis->_has_first_sample) { return false; }  // already initialized; continue with EMA
    
    axis->_has_first_sample = true;
    axis->_smoothed_norm    = norm;
    axis->t_1               = norm;
    axis->t_2               = norm;
    
    SMOOTH_DEBUGF("first sample: norm=%.3f", norm);
    return true;
}
//</editor-fold>

//<editor-fold desc="Update helpers">
static void update_noise_estimate(smooth_axis_t *axis, const float current_residual) {
    /* About 'sign flip' :
 * True movement (often) has consecutive residuals from the same sign
 * ===============
 * _____----▔▔▔▔
 * ==============
 * Noise without movement however, tends to fluctuate randomly above and below the signal
 * ==============
 * _-▔-_---_▔--_-
 * ==============
 * We only care about noise, and want to estimate it only when no true-movement
 * So if the sign flipped -> its noise -> update the noise level
 * Otherwise -> its a movement -> decrease the noise level  */
    bool is_noise_sample = has_sign_flipped(current_residual, axis->_last_residual);
    
    axis->_last_residual = current_residual;
    
    float new_sample = is_noise_sample ? abs_f(current_residual) : 0.0f;
    
    float old_noise = axis->_noise_estimate_norm;
    
    float dev = ema(axis->_noise_estimate_norm, new_sample, NOISE_SMOOTHING_RATE);
    axis->_noise_estimate_norm = clamp_f_0_1(dev);
    
    float noise_change = abs_f(axis->_noise_estimate_norm - old_noise);
    if (noise_change > 0.01f) {  // More than 1% change
        SMOOTH_DEBUGF("noise: %.4f -> %.4f %s",
                      old_noise,
                      axis->_noise_estimate_norm,
                      is_noise_sample ? "(spike)" : "(settling)");
    }
}

static void update_core(smooth_axis_t *axis, uint16_t raw_value, float alpha) {
    float norm = input_norm(axis, raw_value);
    if (initialize_on_first_sample(axis, norm)) { return; }
    
    // 1. Calculate the deviation
    float diff = norm - axis->_smoothed_norm;
    
    // 2. Update EMA
    axis->_smoothed_norm += alpha * diff;
    
    // 3. Update Noise (Exact same logic as your original)
    update_noise_estimate(axis, diff);
    
    /*
     * TODO - Note that the upper implementation change (corrected) noise logic a bit.
     * We used to calculate our diff (inside update_noise) with our smoothed_norm already updated
     * Now our diff is from smoothed_norm which is pre-updated
     *
     * original:
     * axis->_smoothed_norm = ema(axis->_smoothed_norm, norm, alpha);
     * update_deviation_norm_with_sign_flip(axis, norm);
     */
}
//</editor-fold>

//<editor-fold desc="Public API">
// ─────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────

//<editor-fold desc="Init & Configs">
void smooth_axis_default_config_auto_dt(smooth_axis_config_t *cfg,
                                        uint16_t max_raw,
                                        float settle_time_sec,
                                        smooth_axis_now_ms_fn now_ms) {
    SMOOTH_AXIS_CHECK_RETURN(cfg != NULL, "config is NULL");
    SMOOTH_AXIS_CHECK_RETURN(now_ms != NULL, "AUTO mode requires now_ms function");
    
    set_default_config(cfg, max_raw);
    cfg->mode            = SMOOTH_AXIS_MODE_AUTO_DT;
    cfg->settle_time_sec = settle_time_sec;
    cfg->now_ms          = now_ms;
    
    // NEW: AUTO also has a valid settle_k, so it can use ema_alpha_from_live_dt
    cfg->_ema_decay_rate     = compute_ema_decay_rate(settle_time_sec);
    cfg->_settle_time_scaler = compute_dyn_scale(settle_time_sec); // Calculate once
}

void smooth_axis_default_config_live_dt(smooth_axis_config_t *cfg,
                                        uint16_t max_raw,
                                        float settle_time_sec) {
    SMOOTH_AXIS_CHECK_RETURN(cfg != NULL, "config is NULL");
    
    set_default_config(cfg, max_raw);
    cfg->mode                = SMOOTH_AXIS_MODE_LIVE_DT;
    cfg->settle_time_sec     = settle_time_sec;
    cfg->_ema_decay_rate     = compute_ema_decay_rate(settle_time_sec);
    cfg->_settle_time_scaler = compute_dyn_scale(settle_time_sec); // Calculate once
}

void smooth_axis_init(smooth_axis_t *axis, const smooth_axis_config_t *cfg) {
    SMOOTH_AXIS_CHECK_RETURN(axis != NULL, "axis is NULL");
    SMOOTH_AXIS_CHECK_RETURN(cfg != NULL, "config is NULL");
    SMOOTH_AXIS_CHECK_RETURN(cfg->mode != SMOOTH_AXIS_MODE_AUTO_DT || cfg->now_ms != NULL,
                             "AUTO mode requires now_ms function");
    
    axis->cfg                  = *cfg;
    axis->_smoothed_norm       = 0.0f;
    axis->_noise_estimate_norm = 0.01f;
    axis->_last_reported_norm  = 0.0f;
    /// temporary alpha (default based on a FALLBACK_DELTA_TIME fixed update until warmup's done)
    axis->_auto_alpha          = ema_alpha_from_live_dt(cfg->_ema_decay_rate, FALLBACK_DELTA_TIME);
    axis->_dt_accum_sec        = 0.0f;
    axis->_warmup_cycles_done  = 0;
    axis->_last_time_ms        = 0;
    axis->_last_residual       = 0.0f;
    /// first loop iteration will flag this true
    axis->_has_first_sample    = false;
    
    SMOOTH_DEBUGF("init: mode=%s max_raw=%u settle_time=%.3fs",
                  cfg->mode == SMOOTH_AXIS_MODE_AUTO_DT ? "AUTO_DT" : "LIVE_DT",
                  cfg->max_raw,
                  cfg->settle_time_sec);
}
//</editor-fold>

//<editor-fold desc="Update APIs">
// -----------------------------------------------------------------------------
// 4) Core update
// -----------------------------------------------------------------------------
// TODO(jonatan): assert in debug if mode mismatch (AUTO vs LIVE_DT)

void smooth_axis_update_auto_dt(smooth_axis_t *axis, uint16_t raw_value) {
    SMOOTH_AXIS_CHECK_RETURN(axis != NULL, "axis is NULL");
    SMOOTH_AXIS_CHECK_RETURN(axis->cfg.mode == SMOOTH_AXIS_MODE_AUTO_DT,
                             "wrong mode: use update_live_dt() for LIVE_DT mode");
    
    auto_run_warmup_cycle_if_needed(axis);
    
    /// In auto mode, once the alpha was calculated once - we use it as a constant
    update_core(axis, raw_value, axis->_auto_alpha);
}

void smooth_axis_update_live_dt(smooth_axis_t *axis, uint16_t raw_value, float dt_sec) {
    SMOOTH_AXIS_CHECK_RETURN(axis != NULL, "axis is NULL");
    SMOOTH_AXIS_CHECK_RETURN(axis->cfg.mode == SMOOTH_AXIS_MODE_LIVE_DT,
                             "wrong mode: use update_auto_dt() for AUTO_DT mode");
    
    /// In live dt mode, we calibrate alpha according to the new dt in each update
    float live_alpha = ema_alpha_from_live_dt(axis->cfg._ema_decay_rate, dt_sec);
    update_core(axis, raw_value, live_alpha);
}
//</editor-fold>

//<editor-fold desc="Query methods">
// -----------------------------------------------------------------------------
// 5) Output + change detection
// -----------------------------------------------------------------------------

float smooth_axis_get_norm(const smooth_axis_t *axis) {
    return get_normalized(axis);
}

uint16_t smooth_axis_get_u16(const smooth_axis_t *axis) {
    if (!axis) { return 0; }
    float max_out = (float)axis->cfg.max_raw;
    float n       = get_normalized(axis);
    
    
    // Read about 'Zeno's Paradox' to understand the following 2 lines below
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
    
    float dyn_thresh = get_dynamic_thresh(axis);
    if (in_sticky_zone || diff > dyn_thresh) {
        axis->_last_reported_norm = current;
        SMOOTH_DEBUGF("new value: %.3f (diff=%.4f thresh=%.4f %s)",
                      current,
                      diff,
                      dyn_thresh,
                      in_sticky_zone ? "sticky" : "normal");
        return true;
    }
    return false;
}


//<editor-fold desc="Noise / Threshold getters">
// -----------------------------------------------------------------------------
// Introspection / diagnostics
// -----------------------------------------------------------------------------

float smooth_axis_get_noise_norm(const smooth_axis_t *axis) {
    return axis ? axis->_noise_estimate_norm : 0.0f;
}

float smooth_axis_get_effective_thresh_norm(const smooth_axis_t *axis) {
    return axis ? get_dynamic_thresh(axis) : 0.0f;
}

uint16_t smooth_axis_get_effective_thresh_u16(const smooth_axis_t *axis) {
    if (!axis || axis->cfg.max_raw == 0) {
        return 0;
    }
    float t_norm = get_dynamic_thresh(axis);
    if (t_norm <= 0.0f) {
        return 0;
    }
    float val = t_norm * (float)axis->cfg.max_raw + 0.5f;
    val = clamp_f(val, 0.0f, (float)axis->cfg.max_raw);
    return (uint16_t)val;
}
//</editor-fold>

//</editor-fold>

//</editor-fold>

