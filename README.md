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
- **Tiny footprint** — no heap allocation, ~88 bytes RAM per axis

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

This has two issues:

1. **Alpha is unintuitive.** What does `alpha = 0.1` feel like? How should it change if your loop runs faster? You end up tuning by trial and error.

2. **EMA doesn't solve the update problem.** A smoothed signal still fluctuates. When do you act on a change? A fixed threshold either fires on noise (too sensitive) or misses real movement (too sluggish). The right threshold depends on current noise level — which varies.

---

## How smooth_axis Solves It

### Settle-Time Semantics

Instead of alpha, you specify **settle time** — how long the filter takes to reach 95% of its target after a step change:

```c
smooth_axis_config_auto_dt(&cfg, 1023, 0.25f, timer_fn);  // 250ms settle time
```

The library computes the appropriate alpha internally based on your actual frame rate.

| Settle Time | Character |
|-------------|-----------|
| 50–100ms | Responsive, tracks fast movement |
| 200–300ms | Balanced feel for most applications |
| 500ms–1s | Heavily smoothed, slow/cinematic |

### Noise-Adaptive Change Detection

The real work happens in deciding *when to report a change*. The library maintains a real-time noise estimate and scales the threshold dynamically:

- **Low noise** → small threshold → responsive to subtle movement
- **High noise** → large threshold → stable, no false triggers

The threshold scales between 1× and 10× of the base value, adapting as conditions change.

#### Sign-Flip Discrimination

The core insight: **noise oscillates, movement is directional.**

When input is noisy but stationary, the EMA residual (raw − smoothed) flips sign frequently as noise bounces above and below the true value. During real movement, the residual maintains consistent sign — the filter is always "catching up" in one direction.

The library uses this property:

- Frequent sign flips → noise → raise the threshold
- Consistent sign → real movement → lower the threshold

This allows aggressive noise rejection while stationary without sacrificing response speed during movement.

#### In Practice

The chart below shows the algorithm under stress: 5 noise levels × 5 settle times, from pristine (top row) to torture-test conditions (bottom row, 25% timing jitter + 10% Gaussian noise).

![Environment matrix](docs/settle_time_behavior_matrix.png)

Watch the **dotted yellow line** (noise activation threshold) in each cell:

- During the **static segments** (before and after the ramp), sign flips are frequent. The threshold rises to suppress false updates.
- During the **ramp**, movement is directional — sign flips stop. The threshold drops rapidly, allowing the filter to track without lag.

Even in the bottom-left cell (torture noise + 50ms settle time), where the pink input signal is chaotic, the blue output rises cleanly and monotonically.

| Metric | Result |
|--------|--------|
| Monotonic accuracy | **100%** |
| False updates | **0 / 10,271** |

#### Graceful Degradation

Compare the 50ms column top-to-bottom. With clean input (top-left), the algorithm produces ~350 updates over the transition. Under torture-test noise (bottom-left), it produces only 27.

This is deliberate. When noise threatens to overwhelm the signal, the algorithm trades granularity for stability. The threshold rises until only unambiguous movement gets through. The output becomes coarser — effectively bit-crushed — but remains *correct*: monotonic, no false updates, still tracking the actual movement.

27 clean updates are more useful than 350 jittery ones.

That said, the torture row represents extreme stress conditions unlikely in practice. In real-world use — even with mediocre hardware — most applications fall somewhere in the top three rows, where degradation is negligible and the full resolution of your sensor comes through.

### Settle-Time Accuracy

The settle-time parameter means what it says. Measured time to 95% vs. requested:

![Settle-time accuracy chart](docs/step_accuracy.png)

| Condition | MAPE |
|-----------|------|
| Clean input | 1.07% |
| Noisy input (8% jitter, 4% gaussian) | 2.76% |

### Sticky Zones

Analog sensors often behave unreliably at their extremes. Sticky zones create hysteresis at the endpoints:

- Values near 0 snap to exactly 0
- Values near max snap to exactly max
- Small movements within the zone are absorbed
- Larger movements escape normally

This prevents endpoint dithering and guarantees clean 0% / 100% output when the control is at its physical limits.

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

- `smooth_axis_t`: ~88 bytes (includes embedded config, system-dependent due to padding)
- No dynamic allocation
- Stack-safe for constrained environments

---

## License

MIT

---

## Author

Jonatan Vider
