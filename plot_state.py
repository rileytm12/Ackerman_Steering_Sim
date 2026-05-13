"""
Plot vehicle state from state.txt produced by the Ackerman sim:
  - trajectory (x, y)
  - velocity vs time
  - heading vs time

Usage (from this directory):
    python plot_state.py
    python plot_state.py path/to/custom_state.txt
"""

import sys
from pathlib import Path
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np


def load_state_csv(path: Path) -> Tuple[np.ndarray, ...]:
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    # columns: time, x, y, heading, velocity
    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    heading = data[:, 3]
    velocity = data[:, 4]
    return t, x, y, heading, velocity


def main() -> None:
    default = Path(__file__).resolve().parent / "state.txt"
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else default

    if not path.is_file():
        print(f"File not found: {path}", file=sys.stderr)
        sys.exit(1)

    t, x, y, heading, velocity = load_state_csv(path)

    fig, axes = plt.subplots(3, 1, figsize=(8, 10), height_ratios=[1.15, 1.0, 1.0])

    ax_xy = axes[0]
    ax_xy.plot(x, y, color="tab:blue", linewidth=1.5, label="trajectory")
    ax_xy.scatter(x[0], y[0], color="green", s=40, zorder=5, label="start")
    ax_xy.scatter(x[-1], y[-1], color="red", s=40, zorder=5, label="end")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Position (x, y)")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best")

    ax_v = axes[1]
    ax_v.plot(t, velocity, color="tab:orange", linewidth=1.5)
    ax_v.set_xlabel("time [s]")
    ax_v.set_ylabel("velocity [m/s]")
    ax_v.set_title("Velocity vs time")
    ax_v.grid(True, alpha=0.3)

    ax_h = axes[2]
    # dynamics_fun stores heading in radians; plot in degrees for readability
    ax_h.plot(t, np.rad2deg(heading), color="tab:green", linewidth=1.5)
    ax_h.set_xlabel("time [s]")
    ax_h.set_ylabel("heading [deg]")
    ax_h.set_title("Heading vs time")
    ax_h.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
