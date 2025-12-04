//
// Created by Jonatan Vider on 30/11/2025.
//

// smooth_axis_pipeline_test.c
// Full-pipeline tests for smooth_axis module

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>


#include "smooth_axis.h"
#include "smooth_axis_scenario.h"
#include "smooth_axis_pipeline.h"
#include "smooth_axis_presets.h"



// -----------------------------------------------------------------------------
// First pass: find start_out (first reported value after step)
//             and final_out (last reported value at end).
// -----------------------------------------------------------------------------

void find_start_and_final_outputs(const scenario_t *sc,
                                  uint16_t *out_start,
                                  uint16_t *out_final) {
  if (!sc || !out_start || !out_final) return;

  const env_conditions_t  *env  = &sc->env;
  const user_config_t     *user = &sc->user;
  const slide_movement_t  *mov  = &sc->move;

  smooth_axis_config_t cfg;
  make_config_for_scenario(sc, &cfg);

  smooth_axis_t axis;
  smooth_axis_init(&axis, &cfg);

  int steps = (int)(env->duration_sec / env->dt_sec);

  bool     have_first_out = false;
  uint16_t first_out      = 0;
  uint16_t last_out       = 0;

  unsigned int rng = env->rng_state;

  float t = 0.0f;

  for (int i = 0; i < steps; ++i) {
    float dt_step = step_dt_for_scenario(sc, &rng);  // includes jitter if configured

    uint16_t base_raw = compute_base_raw(sc, t, i);

    // First sample: seed filter with clean base_raw
    uint16_t raw = (i == 0)
                   ? base_raw
                   : apply_noise_to_raw(sc, base_raw, &rng);

    update_axis_for_scenario_dt(sc, &axis, raw, dt_step);

    if (smooth_axis_has_new_value(&axis)) {
      uint16_t nominal = smooth_axis_get_u16(&axis);
      if (!have_first_out) {
        first_out      = nominal;
        have_first_out = true;
      }
      last_out = nominal;
    }

    t += dt_step;
  }



  if (!have_first_out) {
    first_out = 0;
    last_out  = 0;
  }

  *out_start = first_out;
  *out_final = last_out;
}

// -----------------------------------------------------------------------------
// Second pass: measure settle time when output crosses threshold.
// -----------------------------------------------------------------------------

float measure_settle_time(const scenario_t *sc,
                          uint16_t start_out,
                          uint16_t final_out) {
  if (!sc) return -1.0f;

  const env_conditions_t  *env  = &sc->env;
  const user_config_t     *user = &sc->user;
  const slide_movement_t  *mov  = &sc->move;

  smooth_axis_config_t cfg;
  make_config_for_scenario(sc, &cfg);

  smooth_axis_t axis;
  smooth_axis_init(&axis, &cfg);

  int   steps = (int)(env->duration_sec / env->dt_sec);
  float t     = 0.0f;

  float s = (float)start_out;
  float f = (float)final_out;
  float d = f - s;

  if (d == 0.0f) {
    return 0.0f; // no movement
  }

  float threshold_out = s + user->settle_fraction * d;



  unsigned int rng    = env->rng_state;
  for (int i = 0; i < steps; ++i) {
    float dt_step = step_dt_for_scenario(sc, &rng);

    uint16_t base_raw = compute_base_raw(sc, t, i);

    uint16_t raw = (i == 0)
                   ? base_raw
                   : apply_noise_to_raw(sc, base_raw, &rng);

    update_axis_for_scenario_dt(sc, &axis, raw, dt_step);

    if (smooth_axis_has_new_value(&axis)) {
      uint16_t nominal = smooth_axis_get_u16(&axis);
      float out_f = (float)nominal;

      if (d > 0.0f) {
        if (out_f >= threshold_out) {
          return t;
        }
      } else {
        if (out_f <= threshold_out) {
          return t;
        }
      }
    }

    t += dt_step;
  }



  return -1.0f; // never reached threshold
}

// -----------------------------------------------------------------------------
// High-level runner
// -----------------------------------------------------------------------------

void run_scenario(const scenario_t *sc) {
  if (!sc) return;

  const env_conditions_t  *env  = &sc->env;
  const user_config_t     *user = &sc->user;
  const slide_movement_t  *mov  = &sc->move;

  printf("=== Scenario: %s ===\n", sc->name);
  printf("max_raw=%u, max_out=%u, dt=%.4fs, duration=%.3fs\n",
         user->max_raw, user->max_out, env->dt_sec, env->duration_sec);
  printf("init_raw=%u, target_raw=%u, settle_fraction=%.2f, target_settle=%.4fs\n",
         mov->init_raw, mov->target_raw,
         user->settle_fraction,
         user->settle_time_sec);

  uint16_t start_out = 0;
  uint16_t final_out = 0;
  find_start_and_final_outputs(sc, &start_out, &final_out);

  printf("start_out=%u, final_out=%u\n", start_out, final_out);

  float measured = measure_settle_time(sc, start_out, final_out);

  if (measured < 0.0f) {
    printf("Measured settle: did not reach %.2f of final in %.3f sec\n\n",
           user->settle_fraction, env->duration_sec);
    return;
  }

  if (mov->type == SYNTHETIC_STEP) {
    // For synthetic step, theory = config: settle_time_sec is what we asked for.
    float expected = user->settle_time_sec;
    printf("Expected settle (config): %.4f sec | Measured: %.4f sec\n\n",
           expected, measured);
  } else {
    // HUMAN_RAMP etc – can ignore for now or keep old logic if you want.
    printf("Measured settle (HUMAN_RAMP): %.4f sec (ramp handling TBD)\n\n",
           measured);
  }
}


void dump_scenario_csv(const scenario_t *sc, const char *filename) {
  if (!sc || !filename) return;

  const env_conditions_t  *env  = &sc->env;
  const user_config_t     *user = &sc->user;

  FILE *f = fopen(filename, "w");
  if (!f) {
    perror("fopen");
    return;
  }

  // Header row
// Header row
  fprintf(f,
          "t_sec,dt_sec,raw_base,raw_noisy,has_new,out_u16,"
          "noise_norm,thresh_norm,acceleration\n");

  // Set up filter
  smooth_axis_config_t cfg;
  make_config_for_scenario(sc, &cfg);

  smooth_axis_t axis;
  smooth_axis_init(&axis, &cfg);

  unsigned int rng = env->rng_state;
  float t = 0.0f;
  int steps = (int)(env->duration_sec / env->dt_sec);

  uint16_t last_out = 0;
  uint16_t current_smoothed = 0;

  for (int i = 0; i < steps; ++i) {
    float dt_step = step_dt_for_scenario(sc, &rng);

    uint16_t base_raw  = compute_base_raw(sc, t, i);

    uint16_t noisy_raw = (i == 0)
                         ? base_raw
                         : apply_noise_to_raw(sc, base_raw, &rng);

    update_axis_for_scenario_dt(sc, &axis, noisy_raw, dt_step);

    int has_new = 0;
//      current_smoothed = smooth_axis_get_u16(&axis, user->max_out);
    if (smooth_axis_has_new_value(&axis)) {
      last_out = smooth_axis_get_u16(&axis);
      has_new  = 1;
    }

    // New: introspection per step
    float noise_norm   = smooth_axis_get_noise_norm(&axis);
    float thresh_norm  = smooth_axis_get_effective_thresh_norm(&axis);

    fprintf(f, "%.6f,%.6f,%u,%u,%d,%u,%.6f,%.6f\n",
            t,
            dt_step,
            (unsigned)base_raw,
            (unsigned)noisy_raw,
            has_new,
            last_out,
            noise_norm,
            thresh_norm);

    t += dt_step;
  }

  fclose(f);
}