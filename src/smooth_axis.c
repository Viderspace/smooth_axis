// smooth_axis.c
// Created by Jonatan Vider on 30/11/2025.
//
#include "smooth_axis.h"
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
static const float SMOOTH_AXIS_RESIDUAL       = 0.05f;
static const float BETA                       =
                           0.005f; // threshold sensitivity to noise fluctuations
static const float K                          =
                           3.5f; // Threshold headroom coefficient (1.0 = no headroom)
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

static inline bool sign_has_flipped(const float current, const float previous) {
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
    cfg->sticky_zone_norm     = clamp_f(STICKY_U / CANONICAL_MAX, 0.0f, 0.49f);
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
    
    norm = clamp_f(norm, off, on);
    norm = map_f(norm, off, on, 0.0f, 1.0f);
    return norm;
}
//</editor-fold>

//<editor-fold desc="Output Pipeline helpers">

static float get_dynamic_thresh(const smooth_axis_t *axis) {
    const float base_thresh = axis->cfg.movement_thresh_norm;
    
    // Raw dynamic term driven only by _dev_norm
    float       dyn_thresh  = K * axis->_dev_norm;
    float       scaled      = dyn_thresh * axis->cfg._dyn_scale;
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
    sticky_margin_size = clamp_f(sticky_margin_size, 0.0f, 0.49f);
    
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
 *  the remaining distance to the target is reduced to 5% (SMOOTH_AXIS_RESIDUAL).

 * The math behind:
 * Given a 'settle_time' which was selected by the user,
 * compute 'ema_rate' such that:
 * alpha(dt) = 1 - exp(ema_rate * dt) , s.t.  ema_rate = ln(SMOOTH_AXIS_RESIDUAL) / settle_time
 */
static float compute_ema_rate(const float settle_time_sec) {
    if (settle_time_sec <= 0.0f) {
        return 0.0f;
    }
    float residual = clamp_f(SMOOTH_AXIS_RESIDUAL, 1e-4f, 0.9999f);
    float ln_r     = logf(residual);  // negative
    return ln_r / settle_time_sec;
}

static float ema_alpha_from_dt(const float k, const float dt_sec) {
    if (dt_sec > 0.0f && k != 0.0f) {
        float ratio = clamp_f(k * dt_sec, -20.0f, 0.0f);
        return 1.0f - expf(ratio);
    }
    return 1.0f; // no smoothing fallback
}
//</editor-fold>

//<editor-fold desc="Error Handling |     todo - Apply across the board">
#ifndef NDEBUG

#include <stdio.h>
#include <assert.h>

static void handle_missing_time_source(void) {
    fprintf(stderr, "smooth_axis_default_config_auto: now_ms is NULL\n");
    assert(0 && "AUTO mode requires non-NULL now_ms");
}

#endif
//</editor-fold>

//<editor-fold desc="Warmup (AUTO) / First sample setter">




static bool auto_warmup_has_finished(const smooth_axis_t *axis) {
    return axis->_warmup_cycles_done >= SMOOTH_AXIS_INIT_CALIBRATION_CYCLES;
}

// AUTO warm-up: measure frame dt, accumulate average, then derive _auto_alpha.
static void auto_run_warmup_cycle_if_needed(smooth_axis_t *axis) {
    // Already calibrated - do nothing
    if (auto_warmup_has_finished(axis)) { return; }
    
    if (!axis->cfg.now_ms) { // Misconfiguration error: no time source.
#ifndef NDEBUG
        handle_missing_time_source();
#endif
        return;
    }
    
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
    
    if (auto_warmup_has_finished(axis)) {
        float dt_avg = axis->_dt_accum_sec / (float)axis->_warmup_cycles_done;
        
        // NEW: use the same EMA math as LIVE_DT
        axis->_auto_alpha = ema_alpha_from_dt(axis->cfg._ema_rate, dt_avg);
    }
}

static bool register_first_sample(smooth_axis_t *axis, float norm) {
    if (!axis->_has_first_sample) {
        axis->_smoothed_norm    = norm;
        axis->_has_first_sample = true;
        return true; // handled; caller should return
    }
    return false;    // already initialized; continue with EMA
}
//</editor-fold>

//<editor-fold desc="Update helpers">
static void update_deviation(smooth_axis_t *axis, const float current_residual) {
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
    
    bool zero_crossed = sign_has_flipped(current_residual, axis->_last_residual);
    axis->_last_residual = current_residual;
    
    float new_sample = zero_crossed ? abs_f(current_residual) : 0.0f;
    float dev        = ema(axis->_dev_norm, new_sample, BETA);
    axis->_dev_norm = clamp_f_0_1(dev);
}

static void update_core(smooth_axis_t *axis, uint16_t raw_value, float alpha) {
    float norm = input_norm(axis, raw_value);
    if (register_first_sample(axis, norm)) { return; }
    
    // 1. Calculate the deviation
    float diff = norm - axis->_smoothed_norm;
    
    // 2. Update EMA
    axis->_smoothed_norm += alpha * diff;
    
    // 3. Update Noise (Exact same logic as your original)
    update_deviation(axis, diff);
    
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
void smooth_axis_default_config_auto(smooth_axis_config_t *cfg,
                                     uint16_t max_raw,
                                     float settle_time_sec,
                                     smooth_axis_now_ms_fn now_ms) {
    if (!cfg) { return; }

#ifndef NDEBUG // Misconfiguration error: no time source.
    if (!now_ms) {
        handle_missing_time_source();
    }
#endif
    set_default_config(cfg, max_raw);
    cfg->mode            = SMOOTH_AXIS_MODE_AUTO_DT;
    cfg->settle_time_sec = settle_time_sec;
    cfg->now_ms          = now_ms;
    
    // NEW: AUTO also has a valid settle_k, so it can use ema_alpha_from_dt
    cfg->_ema_rate  = compute_ema_rate(settle_time_sec);
    cfg->_dyn_scale = compute_dyn_scale(settle_time_sec); // Calculate once
}

void smooth_axis_default_config_live_deltatime(smooth_axis_config_t *cfg,
                                               uint16_t max_raw,
                                               float settle_time_sec) {
    if (!cfg) { return; }
    
    set_default_config(cfg, max_raw);
    cfg->mode            = SMOOTH_AXIS_MODE_LIVE_DT;
    cfg->settle_time_sec = settle_time_sec;
    cfg->_ema_rate       = compute_ema_rate(settle_time_sec);
    cfg->_dyn_scale      = compute_dyn_scale(settle_time_sec); // Calculate once
}

void smooth_axis_init(smooth_axis_t *axis, const smooth_axis_config_t *cfg) {
    if (!axis || !cfg) { return; }
    axis->cfg                 = *cfg;
    axis->_smoothed_norm      = 0.0f;
    axis->_dev_norm           = 0.01f;
    axis->_last_reported_norm = 0.0f;
    /// temporary alpha (default based on a FALLBACK_DELTA_TIME fixed update until warmup's done)
    axis->_auto_alpha         = ema_alpha_from_dt(cfg->_ema_rate, FALLBACK_DELTA_TIME);
    axis->_dt_accum_sec       = 0.0f;
    axis->_warmup_cycles_done = 0;
    axis->_last_time_ms       = 0;
    axis->_last_residual      = 0.0f;
    /// first loop iteration will flag this true
    axis->_has_first_sample   = false;
    
}
//</editor-fold>

//<editor-fold desc="Update APIs">
// -----------------------------------------------------------------------------
// 4) Core update
// -----------------------------------------------------------------------------
// TODO(jonatan): assert in debug if mode mismatch (AUTO vs LIVE_DT)

void smooth_axis_update_auto(smooth_axis_t *axis, uint16_t raw_value) {
    if (!axis) { return; }
    if (axis->cfg.mode != SMOOTH_AXIS_MODE_AUTO_DT) { return; }
    
    auto_run_warmup_cycle_if_needed(axis);
    
    /// In auto mode, once the alpha was calculated once - we use it as a constant
    update_core(axis, raw_value, axis->_auto_alpha);
}

void smooth_axis_update_live_deltatime(smooth_axis_t *axis, uint16_t raw_value, float dt_sec) {
    if (!axis) { return; }
    if (axis->cfg.mode != SMOOTH_AXIS_MODE_LIVE_DT) { return; }
    
    /// In live dt mode, we calibrate alpha according to the new dt in each update
    float live_alpha = ema_alpha_from_dt(axis->cfg._ema_rate, dt_sec);
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
    if (!axis || !axis->_has_first_sample) { /// Protecting misuse
        return false;
    }
    
    float current = get_normalized(axis);
    float last    = axis->_last_reported_norm;
    float diff    = abs_f(current - last);
    
    // If the change is smaller than one quantization step in norm space,
    // it cannot change the integer output → ignore.
    
    uint16_t max_raw = axis->cfg.max_raw ? axis->cfg.max_raw : 1;
    float    epsilon = 1.0f / (float)max_raw;
    if (diff <= epsilon) {
        return false;
    }
    
    // When approaching to the edges, we treat each movement (>= epsilon) as 'Always Important'
    float sticky_ceil            = 1-axis->cfg.sticky_zone_norm;
    float sticky_floor           = axis->cfg.sticky_zone_norm;
    bool inside_the_sticky_edges = (current < sticky_floor) || (current > sticky_ceil);

    float dyn_thresh = get_dynamic_thresh(axis);
    
    if (inside_the_sticky_edges || diff > dyn_thresh) {
        axis->_last_reported_norm = current;
        return true;
    }
    
    return false;
}


//<editor-fold desc="Noise / Threshold getters">
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
    val = clamp_f(val, 0.0f, (float)axis->cfg.max_raw);
    return (uint16_t)val;
}
//</editor-fold>

//</editor-fold>

//</editor-fold>

