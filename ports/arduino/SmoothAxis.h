//
// Created by Jonatan Vider on 09/12/2025.
//
/**
 * @file SmoothAxis.h
 * @brief Arduino wrapper for smooth_axis sensor smoothing library
 * @author Jonatan Vider
 *
 * Adaptive sensor smoothing with noise-aware change detection.
 * Wraps the smooth_axis C library for Arduino-friendly usage.
 */

#pragma once

#include <Arduino.h>
#include "smooth_axis.h"

class SmoothAxis {
        public:
        /**
         * @brief Operating mode for the filter
         */
        enum Mode {
            AUTO         = SMOOTH_AXIS_MODE_AUTO_DT,  ///< Automatic timing using millis()
                    LIVE = SMOOTH_AXIS_MODE_LIVE_DT   ///< Manual timing (you provide delta-time)
        };
        
        private:
        smooth_axis_t _axis;
        
        // Internal millis() wrapper for AUTO mode
        static uint32_t _millis_wrapper() {
            return millis();
        }
        
        public:
        /**
         * @brief Construct a new SmoothAxis filter
         *
         * @param maxRaw        ADC resolution (1023 for 10-bit, 4095 for 12-bit, 65535 for 16-bit)
         * @param settleTimeSec Smoothing time constant - time to reach ~95% of target after step change
         *                      - Snappy: 0.1-0.2s (fast movements, less smoothing)
         *                      - Balanced: 0.25-0.4s (good for most applications)
         *                      - Smooth: 0.5-1.0s (slow/cinematic movement)
         * @param mode          AUTO (default, uses millis() internally) or LIVE (you provide dt_sec)
         *
         * @note The filter is fully initialized after construction - no begin() needed
         *
         * Example:
         * @code
         * SmoothAxis joystick(1023, 0.25f);  // 10-bit ADC, balanced feel
         * @endcode
         */
        SmoothAxis(uint16_t maxRaw,
        float settleTimeSec,
        Mode mode = AUTO)
        {
            smooth_axis_config_t cfg;
            
            if (mode == AUTO) {
                smooth_axis_config_auto_dt(&cfg, maxRaw, settleTimeSec, _millis_wrapper);
            } else {
                smooth_axis_config_live_dt(&cfg, maxRaw, settleTimeSec);
            }
            
            smooth_axis_init(&_axis, &cfg);
        }
        
        // --- Update Methods ---
        
        /**
         * @brief Update filter with new sensor reading (AUTO mode)
         *
         * Call this once per loop with the latest ADC reading.
         * Uses millis() internally for timing.
         *
         * @param rawValue Current sensor reading [0..maxRaw]
         *
         * @note Only for AUTO mode. Calling in LIVE mode will trigger assertion.
         *
         * Example:
         * @code
         * void loop() {
         *     uint16_t raw = analogRead(A0);
         *     filter.update(raw);
         *
         *     if (filter.hasChanged()) {
         *         // Handle the change
         *     }
         * }
         * @endcode
         */
        void update(uint16_t rawValue) {
            smooth_axis_update_auto_dt(&_axis, rawValue);
        }
        
        /**
         * @brief Update filter with new sensor reading and delta time (LIVE mode)
         *
         * Call this once per loop with the latest ADC reading and time elapsed.
         * More accurate than AUTO mode for variable loop rates.
         *
         * @param rawValue Current sensor reading [0..maxRaw]
         * @param dt_sec   Time elapsed since last update (seconds)
         *
         * @note Only for LIVE mode. Calling in AUTO mode will trigger assertion.
         *
         * Example:
         * @code
         * unsigned long lastMicros = 0;
         *
         * void loop() {
         *     unsigned long now = micros();
         *     float dt = (now - lastMicros) / 1000000.0f;
         *     lastMicros = now;
         *
         *     uint16_t raw = analogRead(A0);
         *     filter.update(raw, dt);
         * }
         * @endcode
         */
        void update(uint16_t rawValue, float dt_sec) {
            smooth_axis_update_live_dt(&_axis, rawValue, dt_sec);
        }
        
        // --- Read Values ---
        
        /**
         * @brief Get current smoothed value as integer
         *
         * @return Smoothed sensor value [0..maxRaw]
         *
         * Example:
         * @code
         * uint16_t position = filter.read();
         * analogWrite(LED_PIN, map(position, 0, 1023, 0, 255));
         * @endcode
         */
        uint16_t read() const {
            return smooth_axis_get_u16(&_axis);
        }
        
        /**
         * @brief Get current smoothed value as float
         *
         * @return Normalized smoothed value [0.0..1.0]
         *
         * Example:
         * @code
         * float normalized = filter.readFloat();
         * servo.write(normalized * 180);  // Map to servo range
         * @endcode
         */
        float readFloat() const {
            return smooth_axis_get_norm(&_axis);
        }
        
        /**
         * @brief Check if value changed meaningfully since last check
         *
         * Intelligent change detection that filters noise while catching real movement.
         * Only returns true once per significant change.
         *
         * @return true if position changed enough to warrant action
         * @return false if change is below threshold or sub-LSB
         *
         * @note Safe to call every loop - only returns true once per change
         * @note Threshold adapts dynamically based on noise level
         *
         * Example:
         * @code
         * if (filter.hasChanged()) {
         *     uint16_t newValue = filter.read();
         *     Serial.println(newValue);  // Only prints on meaningful changes
         * }
         * @endcode
         */
        bool hasChanged() {
            return smooth_axis_has_new_value(&_axis);
        }
        
        // --- Utility ---
        
        /**
         * @brief Reset filter state
         *
         * Clears smoothing history and optionally teleports to new position.
         * Useful for mode switches, sleep/wake, or layer changes.
         *
         * @param rawValue Initial position (0 = start at zero, or pass current sensor value)
         *
         * @note In AUTO mode, does NOT restart warmup (keeps calibrated timing)
         *
         * Example:
         * @code
         * // Reset to zero
         * filter.reset();
         *
         * // Reset to current position (avoid jump)
         * uint16_t current = analogRead(A0);
         * filter.reset(current);
         * @endcode
         */
        void reset(uint16_t rawValue = 0) {
            smooth_axis_reset(&_axis, rawValue);
        }
        
        /**
         * @brief Optional: Fine-tune filter behavior
         *
         * Call in setup() before first update if experiencing issues.
         * Default values work for 90% of cases.
         *
         * @param stickyZone Magnetic snap strength at endpoints [0.0-0.5]
         *                   Higher = stronger snap to exact 0/max (default 0.003)
         *                   Use when: Can't reach exact 0 or max value
         *
         * @param fullOff    Dead zone at low end [0.0-1.0]
         *                   Clips unreliable low values to zero (default 0.0 = no dead zone)
         *                   Use when: Sensor is noisy/unreliable at low values
         *
         * @param fullOn     Dead zone at high end [0.0-1.0]
         *                   Clips unreliable high values to max (default 1.0 = no dead zone)
         *                   Use when: Sensor is noisy/unreliable at high values
         *
         * @param moveThresh Base movement threshold [0.0-1.0]
         *                   Higher = less sensitive, fewer updates (default 0.003)
         *                   Use when: Getting too many false updates from noise
         *
         * Example:
         * @code
         * SmoothAxis filter(1023, 0.25f);
         *
         * void setup() {
         *     // Potentiometer has noisy edges - add 5% dead zones
         *     filter.fineTune(0.003f,  // Keep default sticky
         *                     0.05f,   // 5% dead zone at bottom
         *                     0.95f);  // 5% dead zone at top
         * }
         * @endcode
         */
        void fineTune(
        float stickyZone = 0.003f,
        float fullOff = 0.0f,
        float fullOn = 1.0f,
        float moveThresh = 0.003f)
        {
            _axis.cfg.sticky_zone_norm     = stickyZone;
            _axis.cfg.full_off_norm        = fullOff;
            _axis.cfg.full_on_norm         = fullOn;
            _axis.cfg.movement_thresh_norm = moveThresh;
        }
        
        // --- Diagnostics ---
        
        /**
         * @brief Get current noise estimate
         *
         * Returns the filter's real-time estimate of input noise level.
         * Higher values indicate noisier input.
         *
         * @return Noise estimate [0.0..1.0]
         *
         * Example:
         * @code
         * float noise = filter.getNoiseLevel();
         * Serial.print("Noise: ");
         * Serial.println(noise, 4);
         * @endcode
         */
        float getNoiseLevel() const {
            return smooth_axis_get_noise_norm(&_axis);
        }
        
        /**
         * @brief Get effective change detection threshold
         *
         * Returns the current threshold used by hasChanged().
         * Scales dynamically (1x-10x) based on noise level.
         *
         * @return Active threshold [0.0..1.0]
         *
         * Example:
         * @code
         * float threshold = filter.getThreshold();
         * Serial.print("Current threshold: ");
         * Serial.println(threshold, 4);
         * @endcode
         */
        float getThreshold() const {
            return smooth_axis_get_effective_thresh_norm(&_axis);
        }
};