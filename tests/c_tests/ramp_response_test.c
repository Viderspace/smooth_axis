#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <sys/stat.h>

#include "../src/smooth_axis.h"


// -----------------------------------------------------------------------------
// Configuration & Constants
// -----------------------------------------------------------------------------
#define OUTPUT_DIR "tests/data/ramp_files"

// Fixed Test Parameters (migrated from make_10bit_ramp_scenario)
#define BASE_DT_SEC         0.001f
#define TOTAL_DURATION_SEC  3.0f
#define RAMP_START_SEC      0.2f
#define RAMP_DURATION_SEC   0.8f
#define MAX_RAW             1023

typedef struct {
  const char *name;
  float       noise_frac;
  float       jitter_frac;
} env_profile_t;

static const env_profile_t ENV_PROFILES[] = {
        { "pure",    0.000f, 0.000f },
        { "good",    0.005f, 0.010f }, // 0.5% noise, 1.0% jitter
        { "common",  0.015f, 0.020f }, // 1.5% noise, 2.0% jitter
        { "noisy",   0.040f, 0.050f }, // 4.0% noise, 5.0% jitter
        { "torture", 0.100f, 0.250f }, // 10.0% noise, 25.0% jitter
};

static const float SETTLE_TIMES[] = { 0.05f, 0.10f, 0.20f, 0.50f, 1.00f };



// Check if running from project root
static bool validate_working_directory(void) {
    struct stat st;
    // Check if we can see the tests/ directory
    if (stat("tests", &st) != 0 || !S_ISDIR(st.st_mode)) {
        fprintf(stderr, "\nERROR: Must run from project root directory!\n");
        fprintf(stderr, "Current directory doesn't contain 'tests/' folder.\n");
        fprintf(stderr, "\nUsage:\n");
        fprintf(stderr, "  cd /path/to/smooth_axis\n");
        fprintf(stderr, "  ./build/ramp_test\n\n");
        return false;
    }
    return true;
}

// Create output directory if it doesn't exist
static bool ensure_output_dir(const char *path) {
    struct stat st;
    if (stat(path, &st) == 0 && S_ISDIR(st.st_mode)) {
        return true;  // Already exists
    }
    
    // Try to create it (assumes parent dirs exist)
    if (mkdir(path, 0755) == 0) {
        return true;
    }
    
    fprintf(stderr, "ERROR: Cannot create output directory: %s\n", path);
    fprintf(stderr, "Please run: mkdir -p %s\n", path);
    return false;
}


// -----------------------------------------------------------------------------
// Math Helpers (Random & Signal Generation)
// -----------------------------------------------------------------------------

static float rand_uniform01(uint32_t *state) {
    *state = (*state * 1664525u + 1013904223u);
    return (float)(*state >> 8) / (float)0xFFFFFFu;
}

static float rand_normal01(uint32_t *state) {
    float u1 = rand_uniform01(state);
    if (u1 < 1e-7f) u1 = 1e-7f;
    float u2 = rand_uniform01(state);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

// Calculates the "perfect" raw value at time t based on the ramp profile
static uint16_t get_clean_raw(float t) {
    float start = RAMP_START_SEC;
    float dur   = RAMP_DURATION_SEC;
    
    // 10% to 90% of MAX_RAW
    float min_val = 0.10f * MAX_RAW;
    float max_val = 0.90f * MAX_RAW;
    
    if (t <= start) return (uint16_t)(min_val + 0.5f);
    if (t >= start + dur) return (uint16_t)(max_val + 0.5f);
    
    float u = (t - start) / dur; // 0.0 to 1.0
    float val = (1.0f - u) * min_val + u * max_val;
    return (uint16_t)(val + 0.5f);
}

// -----------------------------------------------------------------------------
// Core Logic: Run & Dump
// -----------------------------------------------------------------------------

void run_test_and_dump(const env_profile_t *env, float settle_time, uint32_t seed) {
    char filename[256];
    snprintf(filename, sizeof(filename),
             "%s/smooth_axis_%ubit_settle_time_%.4f_dt=%.4f_jit=%.4f_noise=%.4f_ramp_102_to_921.csv",
             OUTPUT_DIR, MAX_RAW, settle_time, BASE_DT_SEC, env->jitter_frac, env->noise_frac);
    
    FILE *f = fopen(filename, "w");
    if (!f) { perror("Failed to open file"); return; }
    
    smooth_axis_config_t cfg;
    smooth_axis_config_live_dt(&cfg, MAX_RAW, settle_time);
    
    smooth_axis_t axis;
    smooth_axis_init(&axis, &cfg);
    
    fprintf(f, "t_sec,dt_sec,raw_base,raw_noisy,has_new,out_u16,noise_norm,thresh_norm\n");
    
    float t = 0.0f;
    uint16_t last_out = 0;
    int steps = (int)(TOTAL_DURATION_SEC / BASE_DT_SEC);
    
    for (int i = 0; i < steps; ++i) {
        float dt = BASE_DT_SEC;
        if (env->jitter_frac > 0.0f) {
            float u = rand_uniform01(&seed);     // [0,1)
            float j = (u * 2.0f - 1.0f);         // [-1,1)
            dt = BASE_DT_SEC * (1.0f + j * env->jitter_frac);
            if (dt < BASE_DT_SEC * 0.1f) dt = BASE_DT_SEC * 0.1f;
        }
        
        uint16_t clean_raw = get_clean_raw(t);
        uint16_t noisy_raw = clean_raw;
        
        if (env->noise_frac > 0.0f && i > 0) { // Keep first sample clean to seed filter
            float sigma = (env->noise_frac / 3.0f) * MAX_RAW; // ~99.7% within noise_frac
            float noise = sigma * rand_normal01(&seed);
            float val   = (float)clean_raw + noise;
            
            if (val < 0.0f) val = 0.0f;
            if (val > MAX_RAW) val = MAX_RAW;
            noisy_raw = (uint16_t)(val + 0.5f);
        }
        
        smooth_axis_update_live_dt(&axis, noisy_raw, dt);
        
        int has_new = 0;
        if (smooth_axis_has_new_value(&axis)) {
            last_out = smooth_axis_get_u16(&axis);
            has_new = 1;
        }
        
        fprintf(f, "%.6f,%.6f,%u,%u,%d,%u,%.6f,%.6f\n",
                t, dt, clean_raw, noisy_raw, has_new, last_out,
                smooth_axis_get_noise_norm(&axis),
                smooth_axis_get_effective_thresh_norm(&axis));
        
        t += dt;
    }
    
    fclose(f);
}

// -----------------------------------------------------------------------------
// Entry Point
// -----------------------------------------------------------------------------

int main(void) {
    // Validate we're in the right place
    if (!validate_working_directory()) {
        return 1;
    }
    
    // Ensure output directory exists
    if (!ensure_output_dir(OUTPUT_DIR)) {
        return 1;
    }
    
    clock_t start_time = clock();
    size_t env_count = sizeof(ENV_PROFILES) / sizeof(ENV_PROFILES[0]);
    size_t st_count  = sizeof(SETTLE_TIMES) / sizeof(SETTLE_TIMES[0]);
    
    printf("Running ramp response tests...\n");
    
    for (size_t ei = 0; ei < env_count; ++ei) {
        for (size_t ti = 0; ti < st_count; ++ti) {
            // Deterministic seed for reproducibility
            uint32_t seed = (uint32_t)(1000u + ei * 100u + ti * 7u);
            
            run_test_and_dump(&ENV_PROFILES[ei], SETTLE_TIMES[ti], seed);
        }
    }
    
    double cpu_time = ((double)(clock() - start_time)) / CLOCKS_PER_SEC;
    printf("Done. CPU time: %.3f seconds\n", cpu_time);
    return 0;
}