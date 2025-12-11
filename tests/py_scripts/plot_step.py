#!/usr/bin/env python3
"""
Step Response Accuracy Visualization for smooth_axis library.
Creates a 2×8 grid showing settle time accuracy under clean and noisy conditions.
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from typing import Dict, Tuple, Optional


# ============================
# CONFIGURATION
# ============================
class Config:
    # Base directory for all CSV files and output
    BASE_DIR = "tests/data/step_files"
    OUT_DIR = "tests/data/renders"
    SUMMARY_CLEAN = "step_results_clean.csv"
    SUMMARY_NOISY = "step_results_noisy.csv"

    # Trace file pattern: step_trace_{condition}_{settle_ms}ms.csv
    TRACE_PATTERN = "step_trace_{condition}_{settle_ms}ms.csv"

    # Test parameters
    STEP_TIME_MS = 1000  # When the step occurs
    STEP_TIME_SEC = 1.0
    THRESHOLD_VALUE = 140  # 95% threshold
    SETTLE_TIMES_MS = [20, 50, 200, 500, 1000]

    # Plot settings
    TIME_RANGE = [0, 1.5]  # seconds
    VALUE_RANGE = [0, 1023]

    # Colors
    COLOR_INPUT = "#FF4444"  # Faint red for raw input (noisy signal)
    COLOR_OUTPUT = "#0044AA"  # Dark blue for smooth axis output (matching ramp test)
    COLOR_EVENT = "#66CCFF"  # Light blue for update events (has_new)
    COLOR_BASELINE = "black"  # Gray for ideal baseline reference
    COLOR_STEP_START = "black"  # Black for step start marker
    COLOR_OBSERVED = "black"  # Black for observed settle time
    COLOR_THRESHOLD = "gray"  # Gray for threshold line

    # Output
    OUTPUT_PNG = os.path.join(OUT_DIR, "step_response_accuracy.png")
    OUTPUT_TXT = os.path.join(BASE_DIR, "step_test_summary.txt")
    DPI = 300


# ============================
# DATA LOADING
# ============================
class DataLoader:

    @staticmethod
    def load_summary(filepath: str) -> pd.DataFrame:
        """Load summary CSV with settle time results."""
        return pd.read_csv(filepath)

    @staticmethod
    def load_trace(condition: str, settle_ms: int, base_dir: str = ".") -> Optional[pd.DataFrame]:
        """Load a single trace CSV file."""
        filename = Config.TRACE_PATTERN.format(condition=condition, settle_ms=settle_ms)
        filepath = os.path.join(base_dir, filename)

        if not os.path.exists(filepath):
            print(f"Warning: Trace file not found: {filepath}")
            return None

        df = pd.read_csv(filepath)
        # Convert time to seconds
        df['time_sec'] = df['time_ms'] / 1000.0
        return df

    @staticmethod
    def load_all_data(base_dir: str = None) -> Tuple[pd.DataFrame, pd.DataFrame, Dict]:
        """Load all summary and trace data."""
        if base_dir is None:
            base_dir = Config.BASE_DIR

        # Load summaries
        summary_clean = DataLoader.load_summary(os.path.join(base_dir, Config.SUMMARY_CLEAN))
        summary_noisy = DataLoader.load_summary(os.path.join(base_dir, Config.SUMMARY_NOISY))

        # Load all trace files
        traces = {}
        for condition in ['clean', 'noisy']:
            traces[condition] = {}
            for settle_ms in Config.SETTLE_TIMES_MS:
                df = DataLoader.load_trace(condition, settle_ms, base_dir)
                if df is not None:
                    traces[condition][settle_ms] = df

        return summary_clean, summary_noisy, traces


# ============================
# PLOTTING
# ============================
class StepResponsePlotter:

    def __init__(self, summary_clean: pd.DataFrame, summary_noisy: pd.DataFrame, traces: Dict):
        self.summary_clean = summary_clean
        self.summary_noisy = summary_noisy
        self.traces = traces

    def plot_grid(self, output_path: str):
        """Create the main 2×8 grid plot."""
        n_rows = 2
        n_cols = len(Config.SETTLE_TIMES_MS)

        # Create figure with appropriate size (adjusted for 5 columns)
        fig_width = 3.5 * n_cols
        fig_height = 2.5 * n_rows + 1.0  # Extra space for titles/labels

        fig, axes = plt.subplots(
                n_rows, n_cols,
                figsize=(fig_width, fig_height),
                sharex=True,
                sharey=True
        )

        # Plot each condition (row) and settle time (column)
        conditions = ['clean', 'noisy']
        for row_idx, condition in enumerate(conditions):
            for col_idx, settle_ms in enumerate(Config.SETTLE_TIMES_MS):
                ax = axes[row_idx, col_idx]
                self._plot_single_panel(ax, condition, settle_ms,
                                        is_leftmost=(col_idx == 0),
                                        is_bottom=(row_idx == 1),
                                        is_top=(row_idx == 0),
                                        is_rightmost=(col_idx == n_cols - 1))

        # Add overall title and labels
        self._add_figure_labels(fig)

        # Adjust layout - extended header and footer for proper spacing
        fig.tight_layout(rect=[0.02, 0.11, 0.98, 0.84])

        # Save
        fig.savefig(output_path, dpi=Config.DPI, bbox_inches='tight')
        print(f"Saved plot to: {output_path}")

        return fig

    def _plot_single_panel(self, ax, condition: str, settle_ms: int,
                           is_leftmost: bool, is_bottom: bool, is_top: bool, is_rightmost: bool = False):
        """Plot a single subplot for one condition and settle time."""
        # Get trace data
        df = self.traces[condition].get(settle_ms)
        if df is None:
            ax.set_axis_off()
            ax.text(0.5, 0.5, "Data\nNot Found",
                    ha='center', va='center', transform=ax.transAxes,
                    fontsize=10, color='red')
            return

        # Get summary data for this settle time
        summary = self.summary_clean if condition == 'clean' else self.summary_noisy
        row = summary[summary['settle_time_ms'] == settle_ms]
        if len(row) > 0:
            measured_ms = row.iloc[0]['measured_settle_ms']
            error_pct = row.iloc[0]['error_pct']
        else:
            measured_ms = settle_ms
            error_pct = 0.0

        # Find step start time (when raw_input changes from 900 to 100)
        step_start_time = None
        for i in range(1, len(df)):
            if df.iloc[i - 1]['raw_input'] >= 800 and df.iloc[i]['raw_input'] <= 200:
                step_start_time = df.iloc[i]['time_sec']
                break

        # Create ideal baseline curve (same for all tests)
        # 900 from 0 to step_start_time, then 100 afterwards
        if step_start_time is not None:
            time_axis = df['time_sec'].values
            baseline = np.where(time_axis < step_start_time, 900, 100)
            ax.plot(time_axis, baseline,
                    linewidth=0.8, color=Config.COLOR_BASELINE, alpha=0.5,
                    label='Ideal Baseline' if is_leftmost and is_top else '')

        # Plot raw input (faint red)
        ax.plot(df['time_sec'], df['raw_input'],
                linewidth=0.5, alpha=0.3, color=Config.COLOR_INPUT,
                label='Raw Input' if is_leftmost and is_top else '')

        # Plot output (dark blue - matching ramp test)
        ax.plot(df['time_sec'], df['out_u16'],
                linewidth=1.5, color=Config.COLOR_OUTPUT,
                label='Smooth Axis' if is_leftmost and is_top else '')

        # Plot update events (has_new) as light blue dots (s=3)
        if 'has_new' in df.columns:
            events = df[df['has_new'] == 1]
            ax.scatter(events['time_sec'], events['out_u16'],
                       s=3, color=Config.COLOR_EVENT, zorder=5,
                       label='Update Event' if is_leftmost and is_top else '')

        # Threshold line (gray dashed horizontal)
        ax.axhline(Config.THRESHOLD_VALUE,
                   linestyle='--', linewidth=1.0,
                   color=Config.COLOR_THRESHOLD, alpha=0.5,
                   label='95% Threshold' if is_leftmost and is_top else '')

        # Find exact crossing point (where has_new=1 AND crossed_95=1)
        crossing_point = None
        if 'has_new' in df.columns and 'crossed_95' in df.columns:
            crossed = df[(df['has_new'] == 1) & (df['crossed_95'] == 1)]
            if not crossed.empty:
                crossing_point = crossed.iloc[0]

        # Plot marker at exact crossing point (smaller and thinner)
        if crossing_point is not None:
            ax.plot(crossing_point['time_sec'], crossing_point['out_u16'],
                    marker='o', markersize=5, color='black',
                    markerfacecolor='none', markeredgewidth=1, zorder=10,
                    label='Crossing Point' if is_leftmost and is_top else '')

            # Add annotation near the crossing point (same size as false update label)
            annotation_text = f"{measured_ms:.1f}ms ({error_pct:.1f}%)"

            # Special positioning for rightmost column to keep label inside frame
            if is_rightmost:
                # Place directly above the marker for rightmost column
                xytext_offset = (0, 10)
                h_align = 'center'
            else:
                # Normal positioning: to the right and above
                xytext_offset = (7, 8)
                h_align = 'left'

            ax.annotate(annotation_text,
                        xy=(crossing_point['time_sec'], crossing_point['out_u16']),
                        xytext=xytext_offset, textcoords='offset points',
                        fontsize=6,
                        bbox=dict(boxstyle='round,pad=0.4', facecolor='white',
                                  alpha=0.8, edgecolor='none'),
                        ha=h_align, va='bottom')

        # Calculate and display false update statistics (lower left corner)
        if 'has_new' in df.columns and 'out_u16' in df.columns:
            events = df[df['has_new'] == 1]
            total_updates = int(len(events))

            if total_updates <= 1:
                false_updates = 0
            else:
                values = events['out_u16'].to_numpy()
                diffs = values[1:] - values[:-1]
                # For falling step (900→100), false updates are when value increases
                false_updates = int((diffs > 0).sum())

            stats_text = f"false: {false_updates} / {total_updates}"
            ax.text(0.98, 0.03, stats_text, transform=ax.transAxes,
                    ha="right", va="bottom", fontsize=6, color="0.3")

        # Set axis limits
        ax.set_xlim(Config.TIME_RANGE)
        ax.set_ylim(Config.VALUE_RANGE)

        # Y-axis ticks (with 1023 instead of 1000)
        if is_leftmost:
            ax.set_yticks([0, 200, 400, 600, 800, 1023])

        # X-axis: make step_start_time appear as t=0.0, with custom tick positions
        if step_start_time is not None:
            # Define exact relative tick positions: [0.0, 0.5, 1.0]
            relative_ticks = [0.0, 0.5, 1.0]

            # Convert to actual time values
            x_ticks_actual = [step_start_time + rel_t for rel_t in relative_ticks]

            # Create labels
            x_ticks_labels = [f"{rel_t:.1f}" for rel_t in relative_ticks]

            ax.set_xticks(x_ticks_actual)
            if is_bottom:
                ax.set_xticklabels(x_ticks_labels)

        # Labels
        if is_bottom:
            ax.set_xlabel("t(s)", fontsize=7)
        if is_leftmost:
            # Set row label with environment parameters (matching ramp plotter)
            if condition == 'clean':
                ax.set_ylabel("pure (0.0%, 0.0%)")
            else:  # noisy
                ax.set_ylabel("noisy (8.0%, 4.0%)")

        # Column title (only on top row)
        if is_top:
            ax.set_title(f"{settle_ms} ms", fontsize=10, fontweight='bold')

        # Grid
        ax.grid(True, alpha=0.2, linestyle=':')

        # Tick parameters
        ax.tick_params(labelsize=8)

    def _add_figure_labels(self, fig):
        """Add main title, subtitle, row labels, and aggregate metrics."""
        # Calculate MAPE for both conditions
        clean_mape = self.summary_clean['error_pct'].abs().mean()
        noisy_mape = self.summary_noisy['error_pct'].abs().mean()

        # Main title (adjusted for extended header space)
        fig.text(0.5, 0.98, "smooth_axis: Settle-Time Accuracy",
                 ha='center', va='top', fontsize=18, fontweight='bold')

        # Subtitle
        fig.text(0.5, 0.925, "Clean/Noisy (rows) × τ presets (columns)",
                 ha='center', va='top', fontsize=12)

        # Aggregate metrics (MAPE)
        metrics_text = f"MAPE (Avg):    pure: {clean_mape:.2f}%,    noisy: {noisy_mape:.2f}%"

        fig.text(0.5, 0.88, metrics_text,
                 ha='center', va='top', fontsize=8,  color='0.4')

        # Add legend at the bottom - matching ramp test style
        legend_handles = [
                plt.Line2D([0], [0], color=Config.COLOR_BASELINE, linewidth=1.3, alpha=0.5, label="Baseline (reference)"),
                plt.Line2D([0], [0], color=Config.COLOR_INPUT, linewidth=1.0, alpha=0.4, label="Noisy Signal"),
                plt.Line2D([0], [0], color=Config.COLOR_OUTPUT, linewidth=2.0, label="Smooth Axis"),
                plt.Line2D([0], [0], marker="o", linestyle='', color=Config.COLOR_EVENT, markersize=4,
                           label="Update Event"),
                plt.Line2D([0], [0], color=Config.COLOR_THRESHOLD, linewidth=1.5, linestyle='--',
                           label="95% to Target"),
                plt.Line2D([0], [0], marker='o', linestyle='', markerfacecolor='none',
                           markeredgecolor='black', markersize=5, markeredgewidth=1, label="Crossing Point"),
        ]
        fig.legend(handles=legend_handles, loc='lower center', ncol=6,
                   frameon=False, fontsize=9, bbox_to_anchor=(0.5, 0.03))


# ============================
# SUMMARY REPORT
# ============================
class SummaryReporter:

    @staticmethod
    def generate_summary(summary_clean: pd.DataFrame, summary_noisy: pd.DataFrame,
                         output_path: str):
        """Generate text summary of test results."""
        # Calculate MAPE (Mean Absolute Percentage Error)
        clean_mape = summary_clean['error_pct'].abs().mean()
        noisy_mape = summary_noisy['error_pct'].abs().mean()

        # Find max errors
        clean_max_idx = summary_clean['error_pct'].abs().idxmax()
        noisy_max_idx = summary_noisy['error_pct'].abs().idxmax()

        clean_max_error = summary_clean.loc[clean_max_idx, 'error_pct']
        clean_max_settle = summary_clean.loc[clean_max_idx, 'settle_time_ms']

        noisy_max_error = summary_noisy.loc[noisy_max_idx, 'error_pct']
        noisy_max_settle = summary_noisy.loc[noisy_max_idx, 'settle_time_ms']

        # Build report
        lines = [
                "=" * 70,
                "Step Response Test Summary",
                "=" * 70,
                "",
                f"Clean Conditions:",
                f"  MAPE: {clean_mape:.2f}%",
                f"  Max error: {abs(clean_max_error):.2f}% (at {int(clean_max_settle)}ms)",
                "",
                f"Noisy Conditions (4% noise, 8% jitter):",
                f"  MAPE: {noisy_mape:.2f}%",
                f"  Max error: {abs(noisy_max_error):.2f}% (at {int(noisy_max_settle)}ms)",
                "",
                "Individual Results:",
                "-" * 70,
        ]

        # Add individual results
        for _, clean_row in summary_clean.iterrows():
            settle_ms = int(clean_row['settle_time_ms'])
            clean_meas = clean_row['measured_settle_ms']
            clean_err = clean_row['error_pct']

            # Find corresponding noisy result
            noisy_row = summary_noisy[summary_noisy['settle_time_ms'] == settle_ms]
            if len(noisy_row) > 0:
                noisy_meas = noisy_row.iloc[0]['measured_settle_ms']
                noisy_err = noisy_row.iloc[0]['error_pct']
            else:
                noisy_meas = settle_ms
                noisy_err = 0.0

            lines.append(
                    f"  {settle_ms:4d}ms: "
                    f"clean={clean_meas:6.1f}ms ({clean_err:5.1f}%), "
                    f"noisy={noisy_meas:6.1f}ms ({noisy_err:5.1f}%)"
            )

        lines.append("=" * 70)

        # Write to file
        report_text = "\n".join(lines)
        with open(output_path, 'w') as f:
            f.write(report_text)

        print(f"\nSaved summary to: {output_path}")
        print("\n" + report_text)


# ============================
# MAIN
# ============================
def main():
    """Main entry point."""
    print("Loading step response test data...")

    # Load all data
    summary_clean, summary_noisy, traces = DataLoader.load_all_data()

    print(f"Loaded {len(summary_clean)} clean results")
    print(f"Loaded {len(summary_noisy)} noisy results")
    print(f"Loaded {sum(len(t) for t in traces.values())} trace files")

    # Create plotter
    plotter = StepResponsePlotter(summary_clean, summary_noisy, traces)

    # Generate plot
    print("\nGenerating step response accuracy plot...")
    plotter.plot_grid(Config.OUTPUT_PNG)

    # Generate summary report
    print("\nGenerating summary report...")
    SummaryReporter.generate_summary(summary_clean, summary_noisy, Config.OUTPUT_TXT)

    print("\n✓ Done! All outputs generated successfully.")
    # plt.show()


if __name__ == "__main__":
    main()

    print("\n" + "="*50)
    print("Step test plots saved:")
    print("="*50)
    for filename in os.listdir(Config.OUT_DIR):
        if filename.startswith("step_"):
            filepath = os.path.join(Config.OUT_DIR, filename)
            if os.path.isfile(filepath):
                print(f"  {filename}")