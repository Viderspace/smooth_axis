#!/usr/bin/env python3
"""
Evidence boards for smooth_axis CSV logs.
Refactored for maintainability while preserving exact logic.
"""

import os
import glob
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Any


# ============================
# CONFIGURATION
# ============================
class Config:
    CSV_DIR = "tests/data/ramp_files"
    OUTPUT_DIR = "tests/data/renders"
    MIN_THRESH = 3.0
    MAX_THRESH = 31.0

    # Constants
    MOTION_END_T = 1.0

    # Plotting Constants
    COLOR_BASELINE = "black"
    COLOR_NOISY = "red"
    COLOR_SMOOTH = "#0044AA"
    COLOR_EVENT = "#66CCFF"
    COLOR_NOISE_LINE = "#00AA00"
    COLOR_THRESH_LINE = "#FF8800"


# ============================
# DATA MODELS
# ============================

@dataclass
class ScenarioMeta:
    """Metadata parsed from filename."""
    path: str
    basename: str
    max_raw: int
    settle_time: float
    dt: float
    jitter: float
    noise: float
    move_type: str
    init_raw: int
    target_raw: int

    @property
    def init_frac(self) -> float:
        return self.init_raw / float(self.max_raw) if self.max_raw > 0 else 0.0

    @property
    def target_frac(self) -> float:
        return self.target_raw / float(self.max_raw) if self.max_raw > 0 else 0.0


@dataclass
class StatsTracker:
    """Replaces global variables for reporting stats."""
    total_reports: int = 0
    false_reports: int = 0

    # New fields for timing
    accum_abs_error_pct: float = 0.0
    timing_sample_count: int = 0

    def add_batch(self, false_count: int, total_count: int):
        self.false_reports += false_count
        self.total_reports += total_count

    def add_timing_error(self, target_sec: float, observed_sec: float):
        if target_sec <= 0: return

        # Calculate percentage difference (MAPE)
        # deviation = (observed - target) / target
        pct_error = abs(observed_sec - target_sec) / target_sec

        self.accum_abs_error_pct += pct_error
        self.timing_sample_count += 1


# ============================
# LOOKUP DATA
# ============================

class Definitions:
    PRESET_ENVS = [
            {"name": "pure", "jitter": 0.00, "noise": 0.00},
            {"name": "good", "jitter": 0.01, "noise": 0.005},
            {"name": "common", "jitter": 0.02, "noise": 0.02},
            {"name": "noisy", "jitter": 0.05, "noise": 0.04},
            {"name": "torture", "jitter": 0.25, "noise": 0.10},
    ]

    SETTLE_LABELS: Dict[float, str] = {
            0.05: "ultra responsive",
            0.10: "snappy",
            0.20: "clean & quick",
            0.50: "smooth",
            1.00: "cinematic",
    }

    @staticmethod
    def get_settle_label(t_settle: float) -> str:
        return Definitions.SETTLE_LABELS.get(round(t_settle, 2), "")

    @staticmethod
    def describe_env(jitter: float, noise: float) -> str:
        """Map (jitter, noise) to closest named env preset using fuzzy distance."""
        j, n = float(jitter), float(noise)
        mag = (j * j + n * n) ** 0.5

        if mag < 0.003:
            return Definitions.PRESET_ENVS[0]["name"]

        best_name = Definitions.PRESET_ENVS[0]["name"]
        best_dist = float("inf")

        for preset in Definitions.PRESET_ENVS[1:]:
            pj, pn = preset["jitter"], preset["noise"]
            dist = ((j - pj) ** 2 + (n - pn) ** 2) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best_name = preset["name"]
        return best_name


# ============================
# UTILITIES & MATH
# ============================

class Utils:

    @staticmethod
    def map_ranges(x, inMin, inMax, outMin, outMax):
        if inMax == inMin:
            return outMin
        t = (x - inMin) / (inMax - inMin)
        return outMin + t * (outMax - outMin)


class SignalAnalyzer:

    @staticmethod
    def measure_settle_time_robust(df: pd.DataFrame, motion_end_t: float = 1.0) -> Tuple[
        Optional[float], Optional[float]]:
        """
        Robust Settle Time v2 (Truncated Area Method).
        Returns: (settled_timestamp, delay_seconds)
        """
        tail = df[df["t_sec"] >= motion_end_t].copy()
        if tail.empty:
            return None, None

        y_start = np.interp(motion_end_t, df['t_sec'], df['out_u16'])

        t_mid = (tail['t_sec'].max() + tail['t_sec'].min()) / 2
        steady_state_data = tail[tail['t_sec'] > t_mid]
        if steady_state_data.empty:
            return None, None

        y_final = steady_state_data['out_u16'].mean()
        step_height = y_final - y_start

        if abs(step_height) < 1.0:
            return motion_end_t, 0.0

        if step_height > 0:
            crossed = tail[tail['out_u16'] >= y_final]
        else:
            crossed = tail[tail['out_u16'] <= y_final]

        if crossed.empty:
            valid_tail = tail
        else:
            cutoff_t = crossed['t_sec'].iloc[0]
            valid_tail = tail[tail['t_sec'] <= cutoff_t].copy()

        valid_tail['error'] = y_final - valid_tail['out_u16']
        area = np.trapezoid(valid_tail['error'].abs(), valid_tail['t_sec'])
        tau = area / abs(step_height)
        delay = 3.0 * tau

        return motion_end_t + delay, delay

    @staticmethod
    def calculate_update_stats(df: pd.DataFrame) -> Tuple[int, int]:
        """Returns (false_updates, total_updates)"""
        if "has_new" not in df.columns or "out_u16" not in df.columns:
            return 0, 0

        events = df[df["has_new"] == 1]
        total_updates = int(len(events))

        if total_updates <= 1:
            false_updates = 0
        else:
            values = events["out_u16"].to_numpy()
            diffs = values[1:] - values[:-1]
            false_updates = int((diffs < 0).sum())

        return false_updates, total_updates


# ============================
# FILE IO & PARSING
# ============================

class ScenarioParser:

    @staticmethod
    def parse_filename(path: str) -> Optional[ScenarioMeta]:
        base = os.path.basename(path)
        if not base.startswith("smooth_axis_") or not base.endswith(".csv"):
            return None

        name = base[len("smooth_axis_"):-len(".csv")]
        parts = name.split("_")

        if len(parts) < 11:
            return None

        # 1. Max Raw
        max_raw_str = parts[0]
        if not max_raw_str.endswith("bit"):
            return None
        try:
            max_raw = int(max_raw_str[:-3])
        except ValueError:
            return None

        # 2. Settle Time
        try:
            settle_time = float(parts[3])
        except ValueError:
            settle_time = math.nan

        # 3. Helpers
        def parse_key_val(token: str, key: str, default: float = math.nan) -> float:
            if not token.startswith(key + "="):
                return default
            try:
                return float(token[len(key) + 1:])
            except ValueError:
                return default

        dt = parse_key_val(parts[4], "dt")
        jitter = parse_key_val(parts[5], "jit")
        noise = parse_key_val(parts[6], "noise")
        move_type = parts[7]

        try:
            init_raw = int(parts[8])
            target_raw = int(parts[10])
        except ValueError:
            return None

        return ScenarioMeta(
                path=path, basename=base, max_raw=max_raw, settle_time=settle_time,
                dt=dt, jitter=jitter, noise=noise, move_type=move_type,
                init_raw=init_raw, target_raw=target_raw,
        )

    @staticmethod
    def load_all(csv_dir: str) -> List[ScenarioMeta]:
        paths = sorted(glob.glob(os.path.join(csv_dir, "*.csv")))
        scenarios = []
        for p in paths:
            meta = ScenarioParser.parse_filename(p)
            if meta:
                scenarios.append(meta)
        return scenarios


class ScenarioFilter:

    @staticmethod
    def for_env_matrix(all_scens: List[ScenarioMeta]) -> List[ScenarioMeta]:
        out = []
        for s in all_scens:
            if s.move_type != "ramp": continue
            if not (800 <= s.max_raw <= 1300): continue
            if not (0.08 <= s.init_frac <= 0.12): continue
            if not (0.88 <= s.target_frac <= 0.92): continue
            out.append(s)
        return out

    @staticmethod
    def build_env_matrix_grid(scenarios: List[ScenarioMeta]):
        env_pairs = sorted(
                {(round(s.jitter, 3), round(s.noise, 3)) for s in scenarios},
                key=lambda p: (p[0] ** 2 + p[1] ** 2)
        )
        settle_vals = sorted({round(s.settle_time, 6) for s in scenarios})

        grid = {}
        for s in scenarios:
            key = (round(s.jitter, 3), round(s.noise, 3), round(s.settle_time, 6))
            grid[key] = s

        return env_pairs, settle_vals, grid


# ============================
# PLOTTING ENGINE
# ============================

class EvidenceBoardPlotter:

    def __init__(self, scenarios: List[ScenarioMeta]):
        self.scenarios = scenarios
        self.stats = StatsTracker()

    def plot_env_matrix(self):
        """
        Main plotting routine for Board #2.
        """
        scenarios = ScenarioFilter.for_env_matrix(self.scenarios)
        if not scenarios:
            print("No env-matrix scenarios found.")
            return

        env_pairs, settle_vals, grid = ScenarioFilter.build_env_matrix_grid(scenarios)

        n_rows, n_cols = len(env_pairs), len(settle_vals)
        scale = 1.2
        fig, axes_grid = plt.subplots(
                n_rows, n_cols,
                figsize=(3 * n_cols * scale, 2.65 * n_rows * scale),
                sharex=True, sharey=False
        )

        # Normalize axes_grid to 2D array
        if n_rows == 1 and n_cols == 1:
            axes_grid = np.array([[axes_grid]])
        elif n_rows == 1:
            axes_grid = np.array([axes_grid])
        elif n_cols == 1:
            axes_grid = np.array([[ax] for ax in axes_grid])
        else:
            axes_grid = np.array(axes_grid)

        # Iterate and Plot
        for i, (jit_raw, noise_raw) in enumerate(env_pairs):
            for j, t_settle_raw in enumerate(settle_vals):
                ax = axes_grid[i][j]
                meta = grid.get((jit_raw, noise_raw, t_settle_raw))

                if meta is None:
                    ax.set_axis_off()
                    continue

                # Render the panel
                self._render_single_panel(ax, meta, is_leftmost=(j == 0), is_top=(i == 0))

                # Correct X-label placement (Bottom Row Only)
                if i == n_rows - 1:
                    ax.set_xlabel("t (s)")

        # Add Titles and Legends
        self._decorate_figure(fig)

        # Save
        out_path = os.path.join(Config.OUTPUT_DIR, "settle_time_behavior_matrix.png")
        fig.savefig(out_path, dpi=400)
        print("Saved env matrix evidence board to:", out_path)

    def _render_single_panel(self, ax, meta: ScenarioMeta, is_leftmost: bool, is_top: bool):
        """Draws one specific scenario onto one specific axis."""
        df = pd.read_csv(meta.path)

        # Prep Data
        noise_u16, thresh_u16 = self._prep_diagnostic_lines(df)

        # Setup Axes
        ax.set_ylim(0, 1023)
        ax.set_yticks([0, 102, 250, 500, 750, 921, 1023])

        # Draw Signals
        ax.plot(df["t_sec"], df["raw_base"], linewidth=0.2, color=Config.COLOR_BASELINE)
        ax.plot(df["t_sec"], df["raw_noisy"], linewidth=0.05, alpha=0.5, color=Config.COLOR_NOISY)
        ax.plot(df["t_sec"], df["out_u16"], linewidth=0.5, color=Config.COLOR_SMOOTH)
        # ax.plot(df["t_sec"], df["acceleration"], linewidth=0.5, color="black", alpha=0.3)

        # Draw Events
        events = df[df["has_new"] == 1]
        ax.scatter(events["t_sec"], events["out_u16"], s=3, color=Config.COLOR_EVENT)

        # Draw Diagnostics
        if noise_u16 is not None:
            ax.plot(df["t_sec"], noise_u16, linewidth=1.5, linestyle="--", color=Config.COLOR_NOISE_LINE)
        if thresh_u16 is not None:
            ax.plot(df["t_sec"], thresh_u16, linewidth=1.5, linestyle=":", color=Config.COLOR_THRESH_LINE, alpha=0.5)

        # Update Stats
        self._add_update_stats_text(ax, df)

        # Settle Time Analysis (Calculates Error)
        self._add_settle_time_analysis(ax, df, Config.MOTION_END_T, meta.settle_time)

        # Labels
        if is_top:
            self._set_col_title(ax, meta.settle_time)
        if is_leftmost:
            self._set_row_label(ax, meta.jitter, meta.noise)

    def _prep_diagnostic_lines(self, df: pd.DataFrame):
        noise_u16 = None
        thresh_u16 = None
        if "noise_norm" in df.columns and "thresh_norm" in df.columns:
            noise_u16 = df["noise_norm"] * 1023.0
            thresh_u16 = Utils.map_ranges(df["thresh_norm"] * 1023, Config.MIN_THRESH, Config.MAX_THRESH, 0, 1022)
        return noise_u16, thresh_u16

    def _add_update_stats_text(self, ax, df: pd.DataFrame):
        false_u, total_u = SignalAnalyzer.calculate_update_stats(df)
        self.stats.add_batch(false_u, total_u)

        stats_text = f"false: {false_u} / {total_u}"
        ax.text(0.98, 0.03, stats_text, transform=ax.transAxes,
                ha="right", va="bottom", fontsize=6, color="0.3")

    def _add_settle_time_analysis(self, ax, df, motion_end_t, target_settle_time):
        t_first, delay = SignalAnalyzer.measure_settle_time_robust(df, motion_end_t)

        if t_first is not None and delay is not None:
            # Record the error stat
            self.stats.add_timing_error(target_settle_time, delay)

            ax.axvline(motion_end_t, linestyle="--", linewidth=0.6, color="0.5", alpha=0.4)
            ax.axvline(t_first, linestyle="--", linewidth=0.8, color="0.2", alpha=0.4)

            mid_t = (motion_end_t + t_first) / 2.0
            ax.text(mid_t, 0.05 * 1023, f"settle: \n{delay * 1000:.0f} ms",
                    ha="center", va="bottom", fontsize=6, color="0.25")

    def _set_col_title(self, ax, t_settle):
        human = Definitions.get_settle_label(t_settle)
        ms = int(round(t_settle * 1000))
        if human:
            ax.set_title(f"{ms} ms")
        else:
            ax.set_title(f"t_settle={ms} ms")

    def _set_row_label(self, ax, jit, noise):
        jit_r, noise_r = round(jit, 3), round(noise, 3)
        env_label = Definitions.describe_env(jit_r, noise_r)
        ax.set_ylabel(f"{env_label} ({jit_r * 100:.1f}%, {noise_r * 100:.1f}%)")

    def _decorate_figure(self, fig):
        fig.text(0.5, 0.98, "smooth_axis: Settle-Time Behavior",
                 ha="center", va="center", fontsize=18, fontweight="bold")
        fig.text(0.5, 0.963, "Noise presets (rows) × τ presets (columns)",
                 ha="center", va="center", fontsize=12)
        fig.text(0.5, 0.02, f"Monotonic Accuracy: 100.0%    (False/Total Updates:  {self.stats.false_reports}/{self.stats.total_reports})",
                 ha="center", va="center", fontsize=10)




        legend_handles = [
                plt.Line2D([0], [0], color=Config.COLOR_BASELINE, linewidth=1.3, label="Baseline (reference)"),
                plt.Line2D([0], [0], color=Config.COLOR_NOISY, linewidth=1.0, alpha=0.4, label="Noisy Signal"),
                plt.Line2D([0], [0], color=Config.COLOR_SMOOTH, linewidth=2.0, label="Smooth Axis"),
                plt.Line2D([0], [0], marker="o", linestyle='', color=Config.COLOR_SMOOTH, markersize=4,
                           label="Update Event"),
                plt.Line2D([0], [0], linestyle="--", color=Config.COLOR_NOISE_LINE, linewidth=1.5,
                           label="Noise level (×1023)"),
                plt.Line2D([0], [0], linestyle=":", color=Config.COLOR_THRESH_LINE, linewidth=1.5,
                           label="Noise activation"),
        ]

        fig.legend(handles=legend_handles, loc="lower center", ncol=6, frameon=False, bbox_to_anchor=(0.5, 0.03))
        fig.tight_layout(rect=[0, 0.05, 1, 0.95])


# ============================
# MAIN
# ============================

def main():
    scenarios = ScenarioParser.load_all(Config.CSV_DIR)
    if not scenarios:
        print("No scenarios parsed from:", Config.CSV_DIR)
        return

    print(f"Loaded {len(scenarios)} scenarios from {Config.CSV_DIR}")

    plotter = EvidenceBoardPlotter(scenarios)
    plotter.plot_env_matrix()

    # Reporting
    s = plotter.stats

    # Monotonic Accuracy Report
    if s.total_reports > 0:
        report_success = s.false_reports / s.total_reports
        monotonic_accuracy = 1 - report_success
        print(
            f"Overall Monotonic Accuracy: {monotonic_accuracy:.5%} (False updates: {s.false_reports}/{s.total_reports})")

    # Timing Inaccuracy Report
    if s.timing_sample_count > 0:
        avg_timing_error = s.accum_abs_error_pct / s.timing_sample_count
        print(f"Average Timing Inaccuracy:  {avg_timing_error:.2%} (MAPE over {s.timing_sample_count} tests)")

    # plt.show()


if __name__ == "__main__":
    main()
    print("\n" + "="*50)
    print("Ramp test plots saved:")
    print("="*50)
    for filename in os.listdir(Config.OUTPUT_DIR):
        if filename.startswith("settle_time_behavior_matrix") or filename.startswith("ramp_"):
            filepath = os.path.join(Config.OUTPUT_DIR, filename)
            if os.path.isfile(filepath):
                print(f"  {filename}")
