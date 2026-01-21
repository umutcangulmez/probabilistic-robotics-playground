#!/usr/bin/env python3
"""
visualize_results.py

Visualize experiment results from CSV data.

Usage:
    python3 visualize_results.py --data ~/ros2_ws/experiment_data/experiment_20240120_153000
    python3 visualize_results.py --data ./exp1 ./exp2 ./exp3 --compare
    python3 visualize_results.py --data ./exp1 --landmarks landmarks.json
"""

import os
import sys
import argparse
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Circle
from pathlib import Path
from typing import List, Optional, Tuple


# =============================================================================
# Plot Style
# =============================================================================

def setup_style():
    """Publication-ready plot style."""
    plt.rcParams.update({
        'font.size': 11,
        'font.family': 'serif',
        'axes.labelsize': 12,
        'axes.titlesize': 13,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10,
        'figure.figsize': (10, 7),
        'figure.dpi': 120,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
        'lines.linewidth': 1.5,
        'axes.grid': True,
        'grid.alpha': 0.3,
    })


# =============================================================================
# Data Loading
# =============================================================================

class ExperimentData:
    """Container for one experiment's data."""

    def __init__(self, path: str):
        self.path = path
        self.name = os.path.basename(path)

        # Load CSVs
        self.ekf = self._load_csv('ekf_trajectory.csv')
        self.gt = self._load_csv('ground_truth.csv')
        self.metrics = self._load_csv('metrics.csv')
        self.visual = self._load_csv('visual_measurements.csv')

        # Load summary
        summary_path = os.path.join(path, 'summary.json')
        if os.path.exists(summary_path):
            with open(summary_path) as f:
                self.summary = json.load(f)
        else:
            self.summary = {}

        # Extract name from summary or path
        self.label = self.summary.get('experiment_name', self.name)

    def _load_csv(self, filename: str) -> Optional[pd.DataFrame]:
        filepath = os.path.join(self.path, filename)
        if os.path.exists(filepath):
            return pd.read_csv(filepath)
        return None

    def has_data(self) -> bool:
        return self.ekf is not None and len(self.ekf) > 0


def load_landmarks(json_path: str) -> List[Tuple[float, float, int, str]]:
    """Load landmarks from JSON file. Returns (x, y, id, name)."""
    with open(json_path) as f:
        data = json.load(f)
    return [(lm['x'], lm['y'], lm['id'], lm.get('name', f"LM{lm['id']}"))
            for lm in data['landmarks']]


# =============================================================================
# Single Experiment Plots
# =============================================================================

def plot_trajectory(data: ExperimentData, landmarks: List = None,
                    output_path: str = None, show: bool = True):
    """Plot EKF trajectory vs ground truth."""
    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot landmarks
    if landmarks:
        for x, y, lm_id, name in landmarks:
            circle = Circle((x, y), 0.3, fill=True, color='orange',
                            alpha=0.7, zorder=1)
            ax.add_patch(circle)
            ax.annotate(f"ID{lm_id}: {name}", (x, y), textcoords="offset points",
                        xytext=(5, 5), fontsize=8, alpha=0.8)

    # Plot ground truth - convert to numpy arrays
    if data.gt is not None and len(data.gt) > 0:
        gt_x = data.gt['x'].to_numpy()
        gt_y = data.gt['y'].to_numpy()
        ax.plot(gt_x, gt_y, 'k-', linewidth=2.5, label='Ground Truth', zorder=2)

    # Plot EKF estimate - convert to numpy arrays
    if data.ekf is not None and len(data.ekf) > 0:
        ekf_x = data.ekf['x'].to_numpy()
        ekf_y = data.ekf['y'].to_numpy()
        ax.plot(ekf_x, ekf_y, 'b-', linewidth=1.5, label='EKF Estimate', alpha=0.8, zorder=3)

        # Mark start and end
        ax.scatter(ekf_x[0], ekf_y[0], color='green', s=150, marker='o', zorder=5, label='Start')
        ax.scatter(ekf_x[-1], ekf_y[-1], color='red', s=150, marker='s', zorder=5, label='End')

        # Plot uncertainty ellipses every N points
        N = len(data.ekf) // 20
        if N > 0 and 'cov_xx' in data.ekf.columns:
            for i in range(0, len(data.ekf), N):
                row = data.ekf.iloc[i]
                if row['cov_xx'] > 0:
                    plot_covariance_ellipse(
                        ax, row['x'], row['y'],
                        row['cov_xx'], row['cov_yy'], row.get('cov_xy', 0),
                        n_std=2, alpha=0.2, color='blue'
                    )

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title(f'Trajectory: {data.label}')
    ax.legend(loc='best')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path)
        print(f"Saved: {output_path}")
    if show:
        plt.show()
    else:
        plt.close()


def plot_covariance_ellipse(ax, x, y, cov_xx, cov_yy, cov_xy,
                            n_std=2, **kwargs):
    """Plot 2D covariance ellipse."""
    cov = np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]])

    # Eigenvalue decomposition
    vals, vecs = np.linalg.eigh(cov)
    vals = np.maximum(vals, 1e-6)  # Ensure positive

    # Compute ellipse parameters
    width, height = 2 * n_std * np.sqrt(vals)
    angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))

    ellipse = Ellipse(xy=(x, y), width=width, height=height,
                      angle=angle, **kwargs)
    ax.add_patch(ellipse)


def plot_metrics(data: ExperimentData, output_path: str = None, show: bool = True):
    """Plot RMSE, NEES, and error over time."""
    if data.metrics is None or len(data.metrics) == 0:
        print("No metrics data available")
        return

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    t = data.metrics['time'].to_numpy()
    rmse = data.metrics['rmse'].to_numpy()
    nees = data.metrics['nees'].to_numpy()
    error = data.metrics['error'].to_numpy()

    # RMSE
    ax = axes[0]
    ax.plot(t, rmse, 'b-', linewidth=1.5)
    ax.set_ylabel('RMSE (m)')
    ax.set_title(f'Metrics Over Time: {data.label}')
    final_rmse = rmse[-1]
    ax.axhline(y=final_rmse, color='r', linestyle='--', alpha=0.5,
               label=f'Final: {final_rmse:.4f}m')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    # NEES
    ax = axes[1]
    nees_clipped = np.clip(nees, 0, 50)  # Clip for visualization
    ax.plot(t, nees_clipped, 'g-', linewidth=1.5)
    ax.axhline(y=2.0, color='k', linestyle='--', label='Expected (2.0)')
    ax.axhline(y=5.99, color='r', linestyle=':', label='95% bound')
    ax.fill_between(t, 0.05, 5.99, alpha=0.1, color='green', label='Consistent region')
    ax.set_ylabel('NEES')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 20)

    # Instantaneous error
    ax = axes[2]
    ax.plot(t, error, 'r-', linewidth=1.5)
    ax.set_ylabel('Position Error (m)')
    ax.set_xlabel('Time (s)')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path)
        print(f"Saved: {output_path}")
    if show:
        plt.show()
    else:
        plt.close()


def plot_visual_updates(data: ExperimentData, output_path: str = None, show: bool = True):
    """Plot visual measurement statistics."""
    if data.visual is None or len(data.visual) == 0:
        print("No visual measurement data available")
        return

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    t = data.visual['time'].to_numpy()
    range_vals = data.visual['range'].to_numpy()
    bearing_vals = data.visual['bearing'].to_numpy()
    landmark_ids = data.visual['landmark_id'].to_numpy()

    # Range measurements over time
    ax = axes[0, 0]
    scatter = ax.scatter(t, range_vals, c=landmark_ids, cmap='tab10', s=10, alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Range (m)')
    ax.set_title('Range Measurements')
    ax.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax, label='Landmark ID')

    # Bearing measurements over time
    ax = axes[0, 1]
    scatter = ax.scatter(t, np.degrees(bearing_vals), c=landmark_ids, cmap='tab10', s=10, alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Bearing (deg)')
    ax.set_title('Bearing Measurements')
    ax.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax, label='Landmark ID')

    # Landmark detection histogram
    ax = axes[1, 0]
    unique, counts = np.unique(landmark_ids, return_counts=True)
    ax.bar(unique, counts, color='steelblue')
    ax.set_xlabel('Landmark ID')
    ax.set_ylabel('Detection Count')
    ax.set_title('Detections per Landmark')
    ax.grid(True, alpha=0.3)

    # Range histogram
    ax = axes[1, 1]
    ax.hist(range_vals, bins=30, color='steelblue', alpha=0.7)
    ax.set_xlabel('Range (m)')
    ax.set_ylabel('Count')
    ax.set_title('Range Distribution')
    ax.grid(True, alpha=0.3)

    plt.suptitle(f'Visual Measurements: {data.label}', fontsize=14)
    plt.tight_layout()

    if output_path:
        plt.savefig(output_path)
        print(f"Saved: {output_path}")
    if show:
        plt.show()
    else:
        plt.close()


def plot_covariance_evolution(data: ExperimentData, output_path: str = None, show: bool = True):
    """Plot position uncertainty over time."""
    if data.ekf is None or 'cov_xx' not in data.ekf.columns:
        print("No covariance data available")
        return

    fig, ax = plt.subplots(figsize=(12, 5))

    t = data.ekf['time'].to_numpy()
    cov_xx = data.ekf['cov_xx'].to_numpy()
    cov_yy = data.ekf['cov_yy'].to_numpy()

    sigma_x = 3 * np.sqrt(cov_xx)
    sigma_y = 3 * np.sqrt(cov_yy)

    ax.plot(t, sigma_x, 'b-', label='3σ_x', linewidth=1.5)
    ax.plot(t, sigma_y, 'r-', label='3σ_y', linewidth=1.5)

    # Overlay actual error if we have synchronized data
    if data.metrics is not None and len(data.metrics) > 0:
        t_metrics = data.metrics['time'].to_numpy()
        error = data.metrics['error'].to_numpy()
        ax.plot(t_metrics, error, 'k--', label='Actual Error', linewidth=1, alpha=0.7)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Uncertainty (m)')
    ax.set_title(f'Covariance Evolution: {data.label}')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path)
        print(f"Saved: {output_path}")
    if show:
        plt.show()
    else:
        plt.close()


# =============================================================================
# Comparison Plots
# =============================================================================

def plot_trajectory_comparison(data_list: List[ExperimentData],
                               landmarks: List = None,
                               output_path: str = None, show: bool = True):
    """Compare trajectories from multiple experiments."""
    fig, ax = plt.subplots(figsize=(12, 10))

    colors = plt.cm.tab10(np.linspace(0, 1, len(data_list)))

    # Plot landmarks
    if landmarks:
        for x, y, lm_id, name in landmarks:
            circle = Circle((x, y), 0.3, fill=True, color='orange',
                            alpha=0.7, zorder=1)
            ax.add_patch(circle)
            ax.annotate(f"ID{lm_id}: {name}", (x, y), textcoords="offset points",
                        xytext=(5, 5), fontsize=8, alpha=0.8)

    # Plot ground truth from first experiment
    if data_list[0].gt is not None and len(data_list[0].gt) > 0:
        gt_x = data_list[0].gt['x'].to_numpy()
        gt_y = data_list[0].gt['y'].to_numpy()
        ax.plot(gt_x, gt_y, 'k-', linewidth=3, label='Ground Truth', zorder=2)

    # Plot each experiment
    for i, data in enumerate(data_list):
        if data.ekf is not None and len(data.ekf) > 0:
            ekf_x = data.ekf['x'].to_numpy()
            ekf_y = data.ekf['y'].to_numpy()
            ax.plot(ekf_x, ekf_y, color=colors[i], linewidth=1.5,
                    label=data.label, alpha=0.8, zorder=3)

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Trajectory Comparison')
    ax.legend(loc='best')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path)
        print(f"Saved: {output_path}")
    if show:
        plt.show()
    else:
        plt.close()


def plot_rmse_comparison(data_list: List[ExperimentData],
                         output_path: str = None, show: bool = True):
    """Compare RMSE from multiple experiments."""
    fig, ax = plt.subplots(figsize=(12, 6))

    colors = plt.cm.tab10(np.linspace(0, 1, len(data_list)))

    for i, data in enumerate(data_list):
        if data.metrics is not None and len(data.metrics) > 0:
            t = data.metrics['time'].to_numpy()
            rmse = data.metrics['rmse'].to_numpy()
            final_rmse = rmse[-1]
            ax.plot(t, rmse, color=colors[i], linewidth=1.5,
                    label=f"{data.label} (final: {final_rmse:.3f}m)")

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('RMSE (m)')
    ax.set_title('RMSE Comparison')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path)
        print(f"Saved: {output_path}")
    if show:
        plt.show()
    else:
        plt.close()


def plot_nees_comparison(data_list: List[ExperimentData],
                         output_path: str = None, show: bool = True):
    """Compare NEES from multiple experiments."""
    fig, ax = plt.subplots(figsize=(12, 6))

    colors = plt.cm.tab10(np.linspace(0, 1, len(data_list)))

    max_t = 0
    for i, data in enumerate(data_list):
        if data.metrics is not None and len(data.metrics) > 0:
            t = data.metrics['time'].to_numpy()
            nees = data.metrics['nees'].to_numpy()
            nees_clipped = np.clip(nees, 0, 50)
            avg_nees = np.mean(nees)
            ax.plot(t, nees_clipped, color=colors[i], linewidth=1.5,
                    label=f"{data.label} (avg: {avg_nees:.2f})")
            max_t = max(max_t, t[-1])

    # Consistency bounds
    ax.axhline(y=2.0, color='k', linestyle='--', linewidth=2, label='Expected (2.0)')
    ax.axhline(y=5.99, color='r', linestyle=':', linewidth=1.5)
    ax.axhline(y=0.05, color='r', linestyle=':', linewidth=1.5)
    ax.fill_between([0, max_t], 0.05, 5.99, alpha=0.1, color='green')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('NEES')
    ax.set_title('NEES Comparison (Filter Consistency)')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 20)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path)
        print(f"Saved: {output_path}")
    if show:
        plt.show()
    else:
        plt.close()


def create_summary_table(data_list: List[ExperimentData], output_path: str = None):
    """Create summary table of all experiments."""
    rows = []

    for data in data_list:
        if data.metrics is not None and len(data.metrics) > 0:
            final_rmse = data.metrics['rmse'].iloc[-1]
            avg_nees = data.metrics['nees'].mean()
            max_error = data.metrics['error'].max()
        else:
            final_rmse = avg_nees = max_error = float('nan')

        visual_count = len(data.visual) if data.visual is not None else 0
        duration = data.ekf['time'].iloc[-1] if data.ekf is not None and len(data.ekf) > 0 else 0
        consistent = "Yes" if 0.05 < avg_nees < 5.99 else "No"

        rows.append({
            'Experiment': data.label,
            'Duration (s)': f"{duration:.1f}",
            'RMSE (m)': f"{final_rmse:.4f}",
            'Avg NEES': f"{avg_nees:.2f}",
            'Max Error (m)': f"{max_error:.4f}",
            'Visual Updates': visual_count,
            'Consistent': consistent,
        })

    df = pd.DataFrame(rows)

    # Print to console
    print("\n" + "=" * 80)
    print("EXPERIMENT RESULTS SUMMARY")
    print("=" * 80)
    print(df.to_string(index=False))
    print("=" * 80 + "\n")

    # Save to file
    if output_path:
        df.to_csv(output_path, index=False)
        print(f"Saved: {output_path}")

        # Also save LaTeX version
        latex_path = output_path.replace('.csv', '.tex')
        with open(latex_path, 'w') as f:
            f.write(df.to_latex(index=False))
        print(f"Saved: {latex_path}")

    return df


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='Visualize ES-EKF experiment results')
    parser.add_argument('--data', '-d', nargs='+', required=True,
                        help='Path(s) to experiment data directories')
    parser.add_argument('--landmarks', '-l', default=None,
                        help='Path to landmarks JSON file')
    parser.add_argument('--output', '-o', default=None,
                        help='Output directory for figures (default: show only)')
    parser.add_argument('--compare', '-c', action='store_true',
                        help='Generate comparison plots for multiple experiments')
    parser.add_argument('--no-show', action='store_true',
                        help='Do not display plots (only save)')
    args = parser.parse_args()

    setup_style()

    # Load data
    data_list = []
    for path in args.data:
        print(f"Loading: {path}")
        data = ExperimentData(path)
        if data.has_data():
            data_list.append(data)
            print(f"  Loaded {len(data.ekf)} EKF samples, {len(data.visual) if data.visual is not None else 0} visual measurements")
        else:
            print(f"  Warning: No data found in {path}")

    if not data_list:
        print("No valid data found!")
        return

    # Load landmarks
    landmarks = None
    if args.landmarks:
        landmarks = load_landmarks(args.landmarks)
        print(f"Loaded {len(landmarks)} landmarks")

    # Create output directory
    show = not args.no_show
    if args.output:
        os.makedirs(args.output, exist_ok=True)

    # Generate plots
    if len(data_list) == 1 or not args.compare:
        # Single experiment plots
        for data in data_list:
            safe_name = data.label.replace(' ', '_').replace('/', '_')

            plot_trajectory(
                data, landmarks,
                output_path=os.path.join(args.output, f'trajectory_{safe_name}.png') if args.output else None,
                show=show
            )

            plot_metrics(
                data,
                output_path=os.path.join(args.output, f'metrics_{safe_name}.png') if args.output else None,
                show=show
            )

            plot_visual_updates(
                data,
                output_path=os.path.join(args.output, f'visual_{safe_name}.png') if args.output else None,
                show=show
            )

            plot_covariance_evolution(
                data,
                output_path=os.path.join(args.output, f'covariance_{safe_name}.png') if args.output else None,
                show=show
            )

    if args.compare and len(data_list) > 1:
        # Comparison plots
        plot_trajectory_comparison(
            data_list, landmarks,
            output_path=os.path.join(args.output, 'comparison_trajectory.png') if args.output else None,
            show=show
        )

        plot_rmse_comparison(
            data_list,
            output_path=os.path.join(args.output, 'comparison_rmse.png') if args.output else None,
            show=show
        )

        plot_nees_comparison(
            data_list,
            output_path=os.path.join(args.output, 'comparison_nees.png') if args.output else None,
            show=show
        )

    # Summary table
    create_summary_table(
        data_list,
        output_path=os.path.join(args.output, 'summary.csv') if args.output else None
    )


if __name__ == '__main__':
    main()