# smooth_axis

**Adaptive sensor smoothing with noise-aware change detection for embedded systems.**

A single-axis filter that lets you specify response time in seconds instead of opaque filter coefficients. Designed for potentiometers, sliders, and analog sensors on resource-constrained platforms.

---

## Features

- **Settle-time tuning** — specify responsiveness in seconds, not alpha values
- **Noise-adaptive thresholding** — automatically distinguishes noise from real movement
- **Resolution agnostic** — works with any ADC resolution up to 16-bit
- **Frame-rate independent** — same behavior at 60Hz or 1000Hz
- **Zero false updates** — tested across 10,000+ events under extreme noise
- **100% monotonic output** — signal never reverses direction during transitions
- **Tiny footprint** — no heap allocation, ~40 bytes RAM per axis

---

## Quick Start

```c
#include "smooth_axis.h"

uint32_t my_millis(void) { return millis(); }  // or timer_read32() for QMK

smooth_axis_config_t cfg;
smooth_axis_t axis;

smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, my_millis);  // 10-bit ADC, 250ms settle
smooth_axis_init(&axis, &cfg);

void loop(void) {
    uint16_t raw = read_adc();
    smooth_axis_update_auto_dt(&axis, raw);

    if (smooth_axis_has_new_value(&axis)) {
        uint16_t value = smooth_axis_get_u16(&axis);
        // Only called when movement is real, not noise
    }
}
```

---

## The Problem

Raw ADC readings from analog sensors are noisy. The standard solution is an Exponential Moving Average (EMA):

```c
smoothed = alpha * raw + (1 - alpha) * smoothed;
```

This creates two problems:

1. **Alpha is meaningless.** What does `alpha = 0.1` feel like? How does it change if your loop runs faster? You end up tuning by trial and error.

2. **EMA doesn't solve the update problem.** A smoothed signal still fluctuates. When do you act on a change? A fixed threshold either triggers on noise (too sensitive) or misses real movement (too sluggish). The optimal threshold depends on current noise conditions — which change.

---

## How smooth_axis Solves It

### Settle-Time Semantics

Instead of alpha, you specify **settle time** — how long the filter takes to reach 95% of its target after a step change:

```c
smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, timer_fn);  // 250ms settle time
```

The library computes the correct alpha internally, accounting for your actual frame rate. The same settle time produces the same physical response whether you run at 100Hz or 1000Hz.

| Settle Time | Character |
|-------------|-----------|
| 50–100ms | Responsive, tracks fast movement |
| 200–300ms | Balanced feel for most applications |
| 500ms–1s | Heavily smoothed, cinematic |

### Noise-Aware Thresholding

The library maintains a real-time estimate of input noise and scales the change-detection threshold accordingly:

- **Low noise** → small threshold → responsive to subtle movement
- **High noise** → large threshold → stable output, no false triggers

The threshold scales between 1× and 10× of the base value, adapting automatically as conditions change.

### Sign-Flip Discrimination

The core insight: **noise oscillates, movement is directional.**

When the input is noisy but stationary, the EMA residual (difference between raw and smoothed) flips sign frequently — the noise bounces above and below the true value. During real movement, the residual maintains consistent sign as the filter "catches up" in one direction.

The library tracks residual sign flips and uses this to distinguish noise from signal:

- Frequent sign flips → input is noise → raise threshold
- Consistent sign → real movement → lower threshold

This allows aggressive noise rejection during static periods without sacrificing response speed during actual movement.

### Sticky Zones

Analog sensors often have unreliable behavior at their extremes. Sticky zones create hysteresis at the endpoints:

- Values near 0 snap to exactly 0
- Values near max snap to exactly max
- Small movements within the sticky zone are absorbed
- Larger movements escape the zone normally

This prevents endpoint dithering and guarantees clean 0% and 100% output when the physical control is at its limits.

---

## Performance

Tested across a matrix of conditions: 5 noise levels × 5 settle times × multiple runs.

### Settle-Time Accuracy

The filter delivers what you ask for. Measured time to reach 95% of target vs. requested settle time:

![Settle-time accuracy chart](docs/step_accuracy.png)

| Condition | MAPE |
|-----------|------|
| Clean input | 1.07% |
| Noisy input (8% jitter, 4% gaussian) | 2.76% |

### Noise Rejection

Stress-tested from pristine to extreme conditions (up to 25% timing jitter + 10% Gaussian noise):

![Environment matrix](docs/settle_time_behavior_matrix.png)

| Metric | Result |
|--------|--------|
| Monotonic accuracy | **100%** |
| False updates | **0 / 10,271** |

The output never reverses direction during transitions, and no spurious updates occur even under torture-test noise levels.

---

## API Reference

### Configuration

```c
// AUTO_DT mode: library measures your loop timing during warmup
void smooth_axis_config_auto_dt(
    smooth_axis_config_t *cfg,
    uint16_t max_raw,           // ADC max: 1023, 4095, 65535, etc.
    float settle_time_sec,      // Time to 95% settled
    smooth_axis_now_ms_fn now_ms // Monotonic millisecond timer
);

// LIVE_DT mode: you provide delta time each frame
void smooth_axis_config_live_dt(
    smooth_axis_config_t *cfg,
    uint16_t max_raw,
    float settle_time_sec
);

// Initialize axis state from config
void smooth_axis_init(smooth_axis_t *axis, const smooth_axis_config_t *cfg);

// Reset state (e.g., after sleep wake or mode change)
void smooth_axis_reset(smooth_axis_t *axis, uint16_t raw_value);
```

### Update Loop

```c
// AUTO_DT: call once per loop
void smooth_axis_update_auto_dt(smooth_axis_t *axis, uint16_t raw_value);

// LIVE_DT: call once per loop with elapsed time
void smooth_axis_update_live_dt(smooth_axis_t *axis, uint16_t raw_value, float dt_sec);
```

### Output

```c
// Check if value changed meaningfully since last check
bool smooth_axis_has_new_value(smooth_axis_t *axis);

// Get current position
float    smooth_axis_get_norm(const smooth_axis_t *axis);  // [0.0 .. 1.0]
uint16_t smooth_axis_get_u16(const smooth_axis_t *axis);   // [0 .. max_raw]
```

### Diagnostics

```c
// Current noise estimate [0.0 .. 1.0]
float smooth_axis_get_noise_norm(const smooth_axis_t *axis);

// Current effective threshold (after noise scaling)
float    smooth_axis_get_effective_thresh_norm(const smooth_axis_t *axis);
uint16_t smooth_axis_get_effective_thresh_u16(const smooth_axis_t *axis);
```

---

## Configuration Guide

### Choosing a Mode

| Mode | Best For | Trade-off |
|------|----------|-----------|
| `AUTO_DT` | Stable loop rates (most QMK/Arduino) | 256-cycle warmup period |
| `LIVE_DT` | Variable timing, maximum precision | You manage delta time |

### Tuning Feel

The `smooth_axis_config_t` struct exposes additional parameters after initialization:

```c
smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, timer_fn);

// Optional: adjust feel parameters (normalized 0.0 – 1.0)
cfg.full_off_norm = 0.02f;        // Clip bottom 2% to zero
cfg.full_on_norm = 0.98f;         // Clip top 2% to max
cfg.sticky_zone_norm = 0.01f;     // 1% hysteresis at endpoints
cfg.movement_thresh_norm = 0.003f; // Base threshold (before noise scaling)

smooth_axis_init(&axis, &cfg);
```

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `full_off_norm` | 0.0 | Dead zone at low end (noisy/unreliable region) |
| `full_on_norm` | 1.0 | Dead zone at high end |
| `sticky_zone_norm` | ~0.3% | Endpoint hysteresis |
| `movement_thresh_norm` | ~0.3% | Base change threshold |

---

## Platform Notes

### QMK

```c
uint32_t qmk_timer(void) { return timer_read32(); }

smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, qmk_timer);
```

### Arduino

For a cleaner Arduino-style API with a wrapper class, see the [SmoothAxis](https://github.com/Viderspace/Smooth-Axis-Arduino) library available through the Arduino Library Manager.

Direct usage:

```c
uint32_t arduino_timer(void) { return millis(); }

smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, arduino_timer);
```

### Memory Usage

- `smooth_axis_config_t`: ~36 bytes
- `smooth_axis_t`: ~56 bytes (includes embedded config)
- No dynamic allocation
- Stack-safe for constrained environments

---

## License

MIT

---

## Author

Jonatan Vider
