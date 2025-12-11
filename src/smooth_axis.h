/**
 * @file smooth_axis.h
 * @brief Adaptive analog-signal smoothing
 *
 * @author Jonatan Vider
 * @date 30/11/2025
 *
 * Single-axis EMA filter with:
 * - Settle-time based tuning (primary control knob)
 * - Auto or manual delta time handling
 * - Dynamic noise-adaptive thresholding
 * - Sticky zones for precise endpoint control
 * - Dead zone compensation for unreliable sensor edges
 *
 * Typical usage:
 * @code
 * smooth_axis_config_t cfg;
 * smooth_axis_t axis;
 *
 * smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, my_timer_fn);
 * smooth_axis_init(&axis, &cfg);
 *
 * while (1) {
 *     uint16_t raw = read_adc();
 *     smooth_axis_update_auto_dt(&axis, raw);
 *
 *     if (smooth_axis_has_new_value(&axis)) {
 *         uint16_t value = smooth_axis_get_u16(&axis);
 *         handle_change(value);
 *     }
 * }
 * @endcode
 */


#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Delta time source for axis smoothing
 *
 * @note Usage:
 *       AUTO_DT: smooth_axis_config_auto_dt() + smooth_axis_update_auto_dt()
 *       LIVE_DT: smooth_axis_config_live_dt() + smooth_axis_update_live_dt()
 */
typedef enum {
  /**
   * Detect average loop duration during warmup, then use it as constant dt.
   * No need to handle delta time in user code.
   * Best for: Fixed/stable update rate (most QMK/Arduino loops).
   * More efficient - once warm, no timer() or alpha calculations needed.
   */
  SMOOTH_AXIS_MODE_AUTO_DT = 0,
  
  /**
   * Scale slider movement via live delta time.
   * Requires passing dt_sec to update function.
   * Best for: Accuracy, jitter-free. Most stable behavior. No warmup time.
   */
  SMOOTH_AXIS_MODE_LIVE_DT,
} smooth_axis_mode_t;

/**
 * @brief Monotonic millisecond timer function (for AUTO_DT mode)
 *
 * @return Current time in milliseconds (must be monotonically increasing)
 *
 * @note Required for SMOOTH_AXIS_MODE_AUTO_DT, ignored for LIVE_DT
 *
 * Example (QMK):
 * @code
 * uint32_t qmk_now_ms(void) { return timer_read32(); }
 * smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, qmk_now_ms);
 * @endcode
 *
 * Example (Arduino):
 * @code
 * uint32_t arduino_now_ms(void) { return millis(); }
 * smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, arduino_now_ms);
 * @endcode
 */
typedef uint32_t (*smooth_axis_now_ms_fn)(void);

/**
 * @brief Configuration for axis smoothing behavior
 *
 * Build using helper functions:
 * - smooth_axis_config_auto_dt() for AUTO_DT mode
 * - smooth_axis_config_live_dt() for LIVE_DT mode
 *
 * All normalized parameters use range [0.0 .. 1.0] relative to max_raw.
 */
typedef struct {
  // --- Required: input range ---
  /** ADC maximum value (e.g. 1023 for 10-bit, 4095 for 12-bit, 65535 for 16-bit) */
  uint16_t max_raw;
  
  // --- Optional: feel tuning (normalized 0.0 .. 1.0) ---
  /**
   * Dead zone at low end. Clips insensitive/noisy edge region to zero.
   * Use 0.0 for full range (default). Increase if low values are unreliable.
   */
  float full_off_norm;
  
  /**
   * Dead zone at high end. Clips insensitive/noisy edge region to max.
   * Use 1.0 for full range (default). Decrease if high values are unreliable.
   */
  float full_on_norm;
  
  /**
   * Magnetic zone at endpoints (0 and 1). Creates hysteresis for confident edge detection.
   * Helps reach exact 0/1 outputs and prevents edge dithering.
   */
  float sticky_zone_norm;
  
  // --- Smoothing mode + main user knob ---
  /** Operating mode: AUTO_DT or LIVE_DT */
  smooth_axis_mode_t mode;
  
  /**
   * PRIMARY TUNING KNOB: Time to reach ~95% of target after step change (seconds).
   *
   * Controls smoothing strength and responsiveness trade-off:
   * - Lower (0.05-0.15s): Responsive, tracks fast movements, less noise filtering
   * - Medium (0.2-0.4s): Balanced feel for most applications
   * - Higher (0.5-1.0s): Heavily smoothed, slow/cinematic movement
   */
  float settle_time_sec;
  
  // --- Time source for AUTO mode ---
  /**
   * Millisecond timer function (required for AUTO_DT, ignored for LIVE_DT).
   * Must return monotonically increasing value.
   */
  smooth_axis_now_ms_fn now_ms;
  
  // --- Internal (do not modify directly) ---
  /** @internal EMA decay rate constant derived from settle_time_sec */
  float _ema_decay_rate;
  
  /** @internal Pre-calculated scalar for dynamic threshold based on settle_time_sec */
  float _threshold_attenuation;
} smooth_axis_config_t;

/**
 * @brief Runtime state for a single axis
 *
 * Opaque structure - do not access fields directly.
 * Use provided API functions to query state.
 *
 * Initialize with smooth_axis_init() after building config.
 */
typedef struct smooth_axis_t {
  smooth_axis_config_t cfg;
  
  // Internal runtime state (do not access directly)
  float _smoothed_norm;
  float _noise_estimate_norm;
  float _last_reported_norm;
  bool _has_first_sample;
  float _last_residual;
  
  // AUTO_DT internal state
  float    _dt_accum_sec;
  uint16_t _warmup_cycles_done;
  uint32_t _last_time_ms;
  float    _auto_alpha;
} smooth_axis_t;

// ----------------------------------------------------------------------------
// Config helpers + init
// ----------------------------------------------------------------------------
/**
 * @brief Build config for AUTO_DT mode (auto-calibrated fixed deltatime)
 *
 * Best for stable update rates (typical QMK/Arduino loops).
 * Library measures average dt during warmup, then uses constant alpha.
 *
 * @param[out] cfg            Config struct to populate
 * @param[in]  max_raw        ADC maximum (e.g. 1023, 4095, 65535)
 * @param[in]  settle_time_sec Time to ~95% settled after step (seconds)
 * @param[in]  now_ms         Monotonic millisecond timer function (required, non-NULL)
 *
 * @note Warmup takes 256 cycles to calibrate dt. Use fallback alpha until complete.
 *
 * Example (QMK):
 * @code
 * smooth_axis_config_t cfg;
 * smooth_axis_t axis;
 *
 * uint32_t qmk_now_ms(void) { return timer_read32(); }
 *
 * smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, qmk_now_ms);
 * smooth_axis_init(&axis, &cfg);
 * @endcode
 */
void smooth_axis_config_auto_dt(smooth_axis_config_t *cfg,
                                uint16_t max_raw,
                                float settle_time_sec,
                                smooth_axis_now_ms_fn now_ms);

/**
 * @brief Build config for LIVE_DT mode (deltatime passed each update)
 *
 * Best for variable update rates or jitter-sensitive applications.
 * No warmup needed - accurate from first frame.
 *
 * @param[out] cfg            Config struct to populate
 * @param[in]  max_raw        ADC maximum (e.g. 1023, 4095, 65535)
 * @param[in]  settle_time_sec Time to ~95% settled after step (seconds)
 *
 * @note No timer function needed. You provide dt_sec to smooth_axis_update_live_dt().
 *
 * Example (Arduino with variable loop time):
 * @code
 * smooth_axis_config_t cfg;
 * smooth_axis_t axis;
 *
 * smooth_axis_config_live_dt(&cfg, 1023, 0.25f);
 * smooth_axis_init(&axis, &cfg);
 *
 * unsigned long last_us = micros();
 * while (1) {
 *     unsigned long now_us = micros();
 *     float dt_sec = (now_us - last_us) / 1000000.0f;
 *     last_us = now_us;
 *
 *     uint16_t raw = analogRead(A0);
 *     smooth_axis_update_live_dt(&axis, raw, dt_sec);
 * }
 * @endcode
 */
void smooth_axis_config_live_dt(smooth_axis_config_t *cfg,
                                uint16_t max_raw,
                                float settle_time_sec);

/**
 * @brief Initialize axis state from config
 *
 * Call after building config with default_config_auto_dt() or default_config_live_dt().
 * Copies config into axis and resets all runtime state.
 *
 * @param[out] axis Axis state to initialize
 * @param[in]  cfg  Config to copy (must remain valid only during this call)
 *
 * @note After init, axis is ready for first update call.
 * @note AUTO_DT mode: First 256 cycles perform warmup calibration.
 *
 * @code
 * smooth_axis_config_t cfg;
 * smooth_axis_t axis;
 *
 * smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, my_timer_fn);
 * smooth_axis_init(&axis, &cfg);  // Ready to use
 * @endcode
 */
void smooth_axis_init(smooth_axis_t *axis, const smooth_axis_config_t *cfg);

/**
 * @brief Reset axis state to initial conditions
 *
 * Clears smoothing history and optionally teleports to new position.
 * Useful for layer switches, sleep wake, or mode changes.
 *
 * @param[in,out] axis      Axis to reset
 * @param[in]     raw_value Initial position (0 = start at zero)
 *
 * @note In AUTO_DT mode, does NOT restart warmup (keeps calibrated alpha)
 * @note If raw_value is 0, starts at zero. Pass current sensor value to avoid jump.
 */
void smooth_axis_reset(smooth_axis_t *axis, uint16_t raw_value);

// ----------------------------------------------------------------------------
// Core update API (call one of these each loop)
// ----------------------------------------------------------------------------

/**
 * @brief Update axis with new raw sample (AUTO_DT mode)
 *
 * Call once per loop with latest ADC reading.
 * Uses auto-calibrated fixed alpha after warmup completes.
 *
 * @param[in,out] axis      Axis state (mode must be AUTO_DT)
 * @param[in]     raw_value Current ADC reading [0 .. max_raw]
 *
 * @note First 256 calls perform warmup to measure average dt.
 * @note Wrong mode: Calling this in LIVE_DT mode does nothing (checked by assertion).
 *
 * @code
 * while (1) {
 *     uint16_t raw = read_adc();
 *     smooth_axis_update_auto_dt(&axis, raw);
 *
 *     if (smooth_axis_has_new_value(&axis)) {
 *         // Handle meaningful change
 *     }
 * }
 * @endcode
 */
void smooth_axis_update_auto_dt(smooth_axis_t *axis, uint16_t raw_value);

/**
 * @brief Update axis with new raw sample and delta time (LIVE_DT mode)
 *
 * Call once per loop with latest ADC reading and elapsed time.
 * Computes alpha dynamically based on actual dt for jitter-free smoothing.
 *
 * @param[in,out] axis      Axis state (mode must be LIVE_DT)
 * @param[in]     raw_value Current ADC reading [0 .. max_raw]
 * @param[in]     dt_sec    Time elapsed since last update (seconds)
 *
 * @note No warmup needed - accurate from first call.
 * @note Wrong mode: Calling this in AUTO_DT mode does nothing (checked by assertion).
 *
 * @code
 * float last_time_sec = get_time_sec();
 * while (1) {
 *     float now_sec = get_time_sec();
 *     float dt_sec = now_sec - last_time_sec;
 *     last_time_sec = now_sec;
 *
 *     uint16_t raw = read_adc();
 *     smooth_axis_update_live_dt(&axis, raw, dt_sec);
 *
 *     if (smooth_axis_has_new_value(&axis)) {
 *         // Handle meaningful change
 *     }
 * }
 * @endcode
 */
void smooth_axis_update_live_dt(smooth_axis_t *axis,
                                uint16_t raw_value,
                                float dt_sec);

// ----------------------------------------------------------------------------
// Output + change detection
// ----------------------------------------------------------------------------

/**
 * @brief Get current normalized position [0.0 .. 1.0]
 *
 * Returns smoothed position after sticky zone processing.
 *
 * @param[in] axis Axis state
 * @return Normalized position (0.0 = minimum, 1.0 = maximum)
 *
 * @note Returns 0.0 if axis uninitialized or no samples received.
 * @note Value snaps to exact 0.0 or 1.0 within sticky zones.
 */
float smooth_axis_get_norm(const smooth_axis_t *axis);

/**
 * @brief Get current position as integer [0 .. max_raw]
 *
 * Maps normalized position to configured output range.
 *
 * @param[in] axis Axis state
 * @return Integer position [0 .. max_raw]
 *
 * @note Returns 0 if axis is NULL or uninitialized.
 * @note Rounds to nearest integer (uses lroundf() internally).
 * @note Guarantees exact 0 and max_raw at endpoints (no off-by-one).
 */
uint16_t smooth_axis_get_u16(const smooth_axis_t *axis);



/**
 * @brief Check if axis value has meaningfully changed since last check
 *
 * Intelligent change detection that filters noise while catching real movement.
 * Combines noise-adaptive thresholding with sticky zone logic.
 *
 * @param[in,out] axis Axis state (updates internal _last_reported_norm on true return)
 * @return true if position changed enough to warrant application action
 * @return false if change is below threshold, sub-LSB, or no samples received
 *
 * **Detection Logic:**
 * - Returns true if inside sticky zone AND movement >= 1 LSB
 * - Returns true if outside sticky zone AND movement > dynamic_threshold
 * - Dynamic threshold scales 1x-10x with noise level automatically
 * - Ignores sub-LSB changes that cannot affect integer output
 *
 * @note Safe to poll every frame - only returns true once per significant change.
 * @note Threshold adapts in real-time: noisy input → higher threshold → fewer false updates.
 * @note Sticky zones treat any movement >= 1 LSB as "always important" to allow gradual exit.
 *
 * @code
 * while (1) {
 *     uint16_t raw = read_adc();
 *     smooth_axis_update_auto_dt(&axis, raw);
 *
 *     if (smooth_axis_has_new_value(&axis)) {
 *         uint16_t value = smooth_axis_get_u16(&axis);
 *         handle_position_change(value);  // Only called when meaningful
 *     }
 * }
 * @endcode
 */
bool smooth_axis_has_new_value(smooth_axis_t *axis);

// ----------------------------------------------------------------------------
// Introspection / diagnostics
// ----------------------------------------------------------------------------

/**
 * @brief Get current noise estimate in normalized units
 *
 * Returns the library's real-time estimate of input noise level.
 * Used internally for dynamic threshold scaling.
 *
 * @param[in] axis Axis state
 * @return Noise estimate [0.0 .. 1.0], or 0.0 if axis is NULL
 *
 * @note Useful for diagnostics and tuning. Higher values indicate noisier input.
 */
float smooth_axis_get_noise_norm(const smooth_axis_t *axis);

/**
 * @brief Get current effective movement threshold in normalized units
 *
 * Returns the active threshold after noise-adaptive scaling.
 * This is the value used by smooth_axis_has_new_value() for change detection.
 *
 * @param[in] axis Axis state
 * @return Effective threshold [0.0 .. 1.0], or 0.0 if axis is NULL
 *
 * @note Scales dynamically between 1x-10x of cfg.movement_thresh_norm based on noise.
 * @note Higher noise → higher threshold → fewer updates (stability over responsiveness).
 */
float smooth_axis_get_effective_thresh_norm(const smooth_axis_t *axis);

/**
 * @brief Get current effective movement threshold in raw units
 *
 * Same as smooth_axis_get_effective_thresh_norm() but scaled to [0 .. max_raw].
 *
 * @param[in] axis Axis state
 * @return Effective threshold in raw ADC units, or 0 if axis is NULL
 *
 * @note Useful for debugging: "How many ADC counts must change to trigger update?"
 */
uint16_t smooth_axis_get_effective_thresh_u16(const smooth_axis_t *axis);

#ifdef __cplusplus
}
#endif