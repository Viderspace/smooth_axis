//
// Created by Jonatan Vider on 30/11/2025.
//

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "smooth_axis.h"

// Movement type
typedef enum {
    SYNTHETIC_STEP = 0,
    HUMAN_RAMP     = 1,
} movement_type_t;

// Env conditions: dt, jitter, noise, RNG
typedef struct {
    float    dt_sec;
    float    duration_sec;
    float    jitter_fraction;
    float    noise_fraction;
    unsigned rng_state;
} env_conditions_t;

// User-facing config knobs for tests
typedef struct {
    uint16_t   max_raw;
    uint16_t   max_out;
    float      settle_fraction;
    float      settle_time_sec;   // new API knob
    // (old mode/alpha/tau fields are gone in the new world)
} user_config_t;

// Movement profile
typedef struct {
    movement_type_t type;
    float           move_start_sec;
    float           move_duration_sec;
    uint16_t        init_raw;
    uint16_t        target_raw;
} slide_movement_t;

// Full test scenario: env + user + movement + name buffer
typedef struct {
    const char        *name;
    char               name_buf[128];

    env_conditions_t   env;
    user_config_t      user;
    slide_movement_t   move;
} scenario_t;

float    rand_uniform01(unsigned int *state);

float    step_dt_for_scenario(const scenario_t *sc, unsigned int *rng);

uint16_t apply_noise_to_raw(const scenario_t *sc, uint16_t raw, unsigned int *rng);

uint16_t compute_base_raw(const scenario_t *sc, float t, int step_index);

void     make_config_for_scenario(const scenario_t *sc, smooth_axis_config_t *cfg);

void     update_axis_for_scenario_dt(const scenario_t *sc, smooth_axis_t *axis, uint16_t raw, float dt_step);