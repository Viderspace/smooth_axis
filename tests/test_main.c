//
// Created by Jonatan Vider on 30/11/2025.
//

// tests/test_main.c
#include <stdio.h>
#include "smooth_axis_presets.h"
#include "smooth_axis_scenario.h"
#include "smooth_axis_pipeline.h"

#define OUTPUT_ARTIFACTS_DIR "/Users/jonatanvider/Documents/smooth_axis_c_library/artifacts" // todo

// -----------------------------------------------------------------------------
// Scenario name helper
// -----------------------------------------------------------------------------
static scenario_t make_10bit_ramp_scenario(const env_profile_t *env_prof, float settle_time_sec, unsigned int rng_seed) {
    scenario_t sc;

    sc.name                   = NULL; // will be filled by build_scenario_name

    // --- Env: 0.2s static, 1.0s ramp, 1.3s settle => 2.5s total ---
    sc.env.dt_sec             = 0.001f; // 1 ms
    sc.env.duration_sec       = 3.0f;   // 0.2 + 1.0 + 1.3

    sc.env.jitter_fraction    = env_prof->jitter_fraction;
    sc.env.noise_fraction     = env_prof->noise_fraction;
    sc.env.rng_state          = rng_seed;

    // --- User config: 10-bit  ---
    sc.user.max_raw           = 1023;
    sc.user.max_out           = 1023;
    sc.user.settle_fraction   = 0.95f;
    sc.user.settle_time_sec   = settle_time_sec;

    // --- Movement: HUMAN_RAMP 10% -> 90% ---
    sc.move.type              = HUMAN_RAMP;
    sc.move.move_start_sec    = 0.2f; // 0.2s static
    sc.move.move_duration_sec = 0.8f; // 1.0s ramp

    // Raw endpoints based on 10% and 90%
    float max_raw_f           = (float)sc.user.max_raw;
    sc.move.init_raw          = (uint16_t)(0.10f * max_raw_f + 0.5f);
    sc.move.target_raw        = (uint16_t)(0.90f * max_raw_f + 0.5f);

    return sc;
}

void build_scenario_name(scenario_t *sc) {
    if (!sc) return;

    const char *mode_str = "settle_time";
    const char *move_str = (sc->move.type == SYNTHETIC_STEP) ? "step" : "ramp";

    snprintf(sc->name_buf, sizeof(sc->name_buf), "%ubit_%s_%.4f_dt=%.4f_jit=%.4f_noise=%.4f_%s_%u_to_%u", sc->user.max_raw, mode_str, sc->user.settle_time_sec, sc->env.dt_sec, sc->env.jitter_fraction, sc->env.noise_fraction, move_str, sc->move.init_raw, sc->move.target_raw);

    sc->name = sc->name_buf;
}

// Helper: build name, run scenario, and dump CSV into SMOOTH_AXIS_LOG_DIR.
// Assumes the directory already exists.
static void run_and_dump_scenario(scenario_t *sc) {
    if (!sc) return;

    build_scenario_name(sc);

    // Optional: keep the timing/settle prints
    run_scenario(sc);

    // Build CSV filename: smooth_axis_logs/smooth_axis_<name>.csv
    char filename[256];
    snprintf(filename, sizeof(filename), OUTPUT_ARTIFACTS_DIR "/smooth_axis_%s.csv", sc->name);

    dump_scenario_csv(sc, filename);
}

// Small helper to run one scenario (build name, print, etc.)
static void run_scenario_with_name(scenario_t *sc) {
    build_scenario_name(sc);
    run_scenario(sc);
}

int main(void) {
    for (size_t ei = 0; ei < 5; ++ei) {
        const env_profile_t *env = &ENV_PROFILES[ei];

        for (size_t ti = 0; ti < 5; ++ti) {
            float        settle_time_sec = SETTLE_TIME_SEC_VALUES[ti];

            unsigned int seed            = (unsigned int)(1000u + ei * 100u + ti * 7u);

            scenario_t   sc              = make_10bit_ramp_scenario(env, settle_time_sec, seed);
            run_and_dump_scenario(&sc);
        }
    }

    return 0;
}