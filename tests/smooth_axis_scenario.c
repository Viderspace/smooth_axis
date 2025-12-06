//
// Created by Jonatan Vider on 30/11/2025.
//

#include <math.h>
#include "smooth_axis.h"
#include "smooth_axis_scenario.h"

// -----------------------------------------------------------------------------
// Random helpers (shared for jitter + noise)
// -----------------------------------------------------------------------------

float rand_uniform01(unsigned int *state) {
    *state     = (*state * 1664525u + 1013904223u);
    uint32_t x = *state >> 8;
    return (float)x / (float)0xFFFFFFu;
}

// Approximate standard normal N(0,1) using Box-Muller.
// Good enough for test harness / visualization.
static float rand_normal01(unsigned int *state) {
    // Avoid exact 0.0 for logf
    float u1 = rand_uniform01(state);
    if (u1 < 1e-7f) {
        u1 = 1e-7f;
    }
    float u2 = rand_uniform01(state);
    
    float r   = sqrtf(-2.0f * logf(u1));
    float phi = 2.0f * (float)M_PI * u2;
    
    return r * cosf(phi);   // N(0,1)
}

// -----------------------------------------------------------------------------
// Environment helpers (dt jitter + input noise)
// -----------------------------------------------------------------------------

float step_dt_for_scenario(const scenario_t *sc, unsigned int *rng) {
    if (!sc) { return 0.0f; }
    
    const env_conditions_t *env = &sc->env;
    float                  base = env->dt_sec;
    
    if (env->jitter_fraction <= 0.0f) {
        return base;
    }
    
    float u  = rand_uniform01(rng);   // [0,1)
    float j  = (u * 2.0f - 1.0f);      // [-1,1)
    float dt = base * (1.0f + j * env->jitter_fraction);
    
    if (dt < base * 0.1f) {
        dt = base * 0.1f;
    }
    return dt;
}

uint16_t apply_noise_to_raw(const scenario_t *sc,
                            uint16_t raw,
                            unsigned int *rng) {
    if (!sc) { return raw; }
    
    const env_conditions_t *env  = &sc->env;
    const user_config_t    *user = &sc->user;
    
    if (env->noise_fraction <= 0.0f) {
        return raw;
    }
    
    float max_raw_f = (float)user->max_raw;
    float raw_f     = (float)raw;
    
    /*
     * FIXME - OLD UNIFORM-RANDOM NOISE APPROACH
     */
//  float amp = env->noise_fraction * max_raw_f;
//  float u   = rand_uniform01(rng);              // [0,1)
//  float j   = (u * 2.0f - 1.0f);                // [-1,1)
//  float noisy = raw_f + j * amp;
//
//  if (noisy < 0.0f) noisy = 0.0f;
//  if (noisy > max_raw_f) noisy = max_raw_f;
//
//  return (uint16_t)(noisy + 0.5f);


/*
 * FIXME - NEW GAUSSIAN NOISE (REALISTIC NOISE) APPROACH*/
    
    // Interpret noise_fraction as an approximate PEAK fraction of full-scale:
    //   "±noise_fraction ≈ ±3σ"  → σ = noise_fraction / 3
    //
    // Example: noise_fraction = 0.10 ( "10% noise" ) →
    //   σ_norm ≈ 0.033 → ~99.7% of samples in ±10% FS.
    
    float sigma_norm = env->noise_fraction / 3.0f;     // tunable choice
    float n01        = rand_normal01(rng);             // N(0,1)
    float noise_norm = sigma_norm * n01;    // N(0, σ²) in normalized units
    
    float noisy = raw_f + noise_norm * max_raw_f;
    
    if (noisy < 0.0f) { noisy = 0.0f; }
    if (noisy > max_raw_f) { noisy = max_raw_f; }
    
    return (uint16_t)(noisy + 0.5f);
    
}


// -----------------------------------------------------------------------------
// Movement helper: compute the clean base raw value at time t
// -----------------------------------------------------------------------------

uint16_t compute_base_raw(const scenario_t *sc, float t, int step_index) {
    if (!sc) { return 0; }
    
    const slide_movement_t *mov = &sc->move;
    
    uint16_t init   = mov->init_raw;
    uint16_t target = mov->target_raw;
    
    switch (mov->type) {
        case SYNTHETIC_STEP:
            // Old behaviour: first sample at init, then instant jump to target
            if (step_index == 0) {
                return init;
            } else {
                return target;
            }
        
        case HUMAN_RAMP: {
            float start = mov->move_start_sec;
            float dur   = mov->move_duration_sec;
            
            // Guard against degenerate duration
            if (dur <= 0.0f) {
                // If duration is invalid, just treat as instant step at move_start_sec
                return (t < start) ? init : target;
            }
            
            if (t <= start) {
                return init;
            }
            if (t >= start + dur) {
                return target;
            }
            
            // Linear interpolation between init and target
            float u = (t - start) / dur;  // 0..1
            if (u < 0.0f) { u = 0.0f; }
            if (u > 1.0f) { u = 1.0f; }
            
            float raw_f = (1.0f - u) * (float)init + u * (float)target;
            if (raw_f < 0.0f) { raw_f = 0.0f; }
            // max_raw clip will happen later via noise application, but clamp anyway
            if (raw_f > (float)sc->user.max_raw) {
                raw_f = (float)sc->user
                        .max_raw;
            }
            return (uint16_t)(raw_f + 0.5f);
        }
        
        default:return init;
    }
}


// -----------------------------------------------------------------------------
// Config & update helpers
// -----------------------------------------------------------------------------



void make_config_for_scenario(const scenario_t *sc,
                              smooth_axis_config_t *cfg) {
    if (!cfg || !sc) { return; }
    const user_config_t *user = &sc->user;
    
    // We test the new "live-dt + settle_time" API only.
    // max_raw comes from the scenario; settle_time_sec is the user knob.
    smooth_axis_default_config_live_deltatime(cfg,
                                              user->max_raw,
                                              user->settle_time_sec);
//    cfg->full_off_norm = 0.0f;
//    cfg->full_on_norm  = 1.0f;
}

void update_axis_for_scenario_dt(const scenario_t *sc,
                                 smooth_axis_t *axis,
                                 uint16_t raw,
                                 float dt_step) {
    (void)sc;
    if (!axis) { return; }
    
    // Test the dt-aware path only.
    smooth_axis_update_live_deltatime(axis, raw, dt_step);
}