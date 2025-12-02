//
// Created by Jonatan Vider on 30/11/2025.
//

// smooth_axis_presets.h
#pragma once
#include <stdlib.h>

// Global timing constants (all tests share these)
//#define STATIC_START_SEC   0.5f   // hold at init for 0.5 s
//#define MOVE_DURATION_SEC  1.0f   // 1.0 s ramp
//#define SETTLE_TAIL_SEC    1.0f   // 1.0 s settle after ramp
//
//#define TOTAL_DURATION_SEC (STATIC_START_SEC + MOVE_DURATION_SEC + SETTLE_TAIL_SEC)
//

// This matches how make_10bit_ramp_scenario uses env_profile_t:
//   env_prof->jitter_fraction
//   env_prof->noise_fraction
//   env_prof->name (via build_scenario_name)
typedef struct {
    const char *name;            // short id: "pure", "good", ...
    float       noise_fraction;  // e.g. 0.005f for 0.5%
    float       jitter_fraction; // e.g. 0.010f for 1.0%
} env_profile_t;

// Env presets (slightly "stricter than the description", per your design)
//
// Percentages converted to fractions:
//   0.5%  -> 0.005f
//   1.5%  -> 0.015f
//   4.0%  -> 0.040f
//   10.0% -> 0.100f
//   1.0%  -> 0.010f
//   2.0%  -> 0.020f
//   5.0%  -> 0.050f
//   25.0% -> 0.250f
static const env_profile_t ENV_PROFILES[] = {
    {
        .name            = "pure",
        .noise_fraction  = 0.000f,
        .jitter_fraction = 0.000f,
    },
    {
        .name            = "good",
        .noise_fraction  = 0.005f, // 0.5%
        .jitter_fraction = 0.010f, // 1.0%
    },
    {
        .name            = "common",
        .noise_fraction  = 0.015f, // 1.5%
        .jitter_fraction = 0.020f, // 2.0%
    },
    {
        .name            = "noisy",
        .noise_fraction  = 0.040f, // 4.0%
        .jitter_fraction = 0.050f, // 5.0%
    },
    {
        .name            = "torture",
        .noise_fraction  = 0.100f, // 10.0%
        .jitter_fraction = 0.250f, // 25.0%
    },
};

static const size_t ENV_PROFILES_COUNT = sizeof(ENV_PROFILES) / sizeof(ENV_PROFILES[0]);

// -----------------------------------------------------------------------------
// Profiles + helpers
// -----------------------------------------------------------------------------

typedef struct {
    uint16_t    max_raw;
    uint16_t    max_out;
    const char *label;
} res_profile_t;

typedef struct {
    float       settle_time_sec;
    const char *label;
} settle_profile_t;

static const float SETTLE_TIME_SEC_VALUES[] = {
    0.05f, // ultra responsive
    0.10f, // snappy
    0.20f, // clean & quick (default)
    0.50f, // smooth
    1.00f, // cinematic
};

static const settle_profile_t SETTLE_PROFILES[] = {
    {0.05f, "settle_ultra"},
    {0.01f, "settle_snappy"},
    {0.20f, "settle_small"},
    {0.50f, "settle_medium"},
    {1.00f, "settle_heavy"},
};

#define NUM_SETTLE_PROFILES (sizeof(SETTLE_PROFILES) / sizeof(SETTLE_PROFILES[0]))

// 4 resolution profiles: 8, 10, 12-in/10-out, 16-in/10-out
static const res_profile_t RES_PROFILES[] = {
    {255, 255, "8bit_in_out"},
    {1023, 1023, "10bit_in_out"},
    {4095, 4095, "12in_out"},
    {65535, 65535, "16in_out"},
};
