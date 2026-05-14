"""
Plot vehicle state from state.txt and reference path from trajectory.txt:
  - x-y: reference vs actual position
  - velocity vs time
  - heading vs time

Usage:
    python plot_state.py
    python plot_state.py path/to/state.txt
    python plot_state.py path/to/state.txt path/to/trajectory.txt
"""

import sys
from pathlib import Path
from typing import Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np


def load_state_csv(path: Path) -> Tuple[np.ndarray, ...]:
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    heading = data[:, 3]
    velocity = data[:, 4]
    return t, x, y, heading, velocity


def load_trajectory_csv(path: Path) -> Optional[Tuple[np.ndarray, ...]]:
    if not path.is_file():
        return None
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    ncol = data.shape[1]
    t = data[:, 0]
    x_ref = data[:, 1]
    y_ref = data[:, 2]
    heading_ref = data[:, 3]
    if ncol >= 6:
        v_ref = data[:, 4]
        steering_ref = data[:, 5]
    else:
        v_ref = np.full_like(t, np.nan, dtype=float)
        steering_ref = data[:, 4]
    return t, x_ref, y_ref, heading_ref, v_ref, steering_ref


def main() -> None:
    base = Path(__file__).resolve().parent
    state_path = Path(sys.argv[1]) if len(sys.argv) > 1 else base / "state.txt"
    traj_path = Path(sys.argv[2]) if len(sys.argv) > 2 else base / "trajectory.txt"

    if not state_path.is_file():
        print(f"State file not found: {state_path}", file=sys.stderr)
        sys.exit(1)

    t, x, y, heading, velocity = load_state_csv(state_path)
    traj = load_trajectory_csv(traj_path)
    if traj is None:
        print(f"Note: no trajectory file at {traj_path}; x–y plot shows actual only.", file=sys.stderr)

    fig, axes = plt.subplots(3, 1, figsize=(8, 10), height_ratios=[1.15, 1.0, 1.0])

    ax_xy = axes[0]
    if traj is not None:
        _, xr, yr, _, _, _ = traj
        ax_xy.plot(
            xr,
            yr,
            color="tab:red",
            linewidth=2.0,
            linestyle="--",
            label="reference (trajectory.txt)",
            alpha=0.9,
        )
    ax_xy.plot(x, y, color="tab:blue", linewidth=1.5, label="actual (state.txt)")
    ax_xy.scatter(x[0], y[0], color="green", s=40, zorder=5, label="start (actual)")
    ax_xy.scatter(x[-1], y[-1], color="red", s=40, zorder=5, label="end (actual)")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Position: reference vs actual")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best")

    ax_v = axes[1]
    ax_v.plot(t, velocity, color="tab:orange", linewidth=1.5, label="actual")
    if traj is not None:
        _, _, _, _, vref, _ = traj
        if len(vref) == len(t) and np.all(np.isfinite(vref)):
            ax_v.plot(t, vref, color="tab:red", linewidth=1.5, linestyle="--", alpha=0.9, label="v_ref")
    ax_v.legend(loc="best")
    ax_v.set_xlabel("time [s]")
    ax_v.set_ylabel("velocity [m/s]")
    ax_v.set_title("Velocity vs time (actual)")
    ax_v.grid(True, alpha=0.3)

    ax_h = axes[2]
    ax_h.plot(t, np.rad2deg(heading), color="tab:green", linewidth=1.5, label="actual")
    if traj is not None:
        _, _, _, href, _, _ = traj
        if len(href) == len(t):
            ax_h.plot(
                t,
                np.rad2deg(href),
                color="tab:red",
                linewidth=1.5,
                linestyle="--",
                alpha=0.9,
                label="reference",
            )
        else:
            tt = traj[0]
            ax_h.plot(
                tt,
                np.rad2deg(href),
                color="tab:red",
                linewidth=1.5,
                linestyle="--",
                alpha=0.9,
                label="reference (own time base)",
            )
    ax_h.set_xlabel("time [s]")
    ax_h.set_ylabel("heading [deg]")
    ax_h.set_title("Heading vs time")
    ax_h.grid(True, alpha=0.3)
    ax_h.legend(loc="best")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
