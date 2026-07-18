"""Trajectory input: position-only setpoints -> time-sampled X(t), Y(t) with derivatives.

Real setpoints arrive as position only, so velocity/acceleration are reconstructed by
Savitzky-Golay smoothing+differentiation (double-differentiating raw position is noisy,
which is why the setpoint log should be sampled as fast as the controller allows).

Also provides a synthetic trajectory generator (a stop-at-corner square) so the whole
pipeline runs with no external data file.
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d


@dataclass
class Trajectory:
    t: np.ndarray          # (N,)
    x: np.ndarray          # (N,) commanded machine X position (m)
    y: np.ndarray          # (N,) commanded machine Y position (m)

    def interpolators(self):
        """Return (fx, fy) callables t->position, clamped outside the range."""
        fx = interp1d(self.t, self.x, kind="linear", bounds_error=False,
                      fill_value=(self.x[0], self.x[-1]))
        fy = interp1d(self.t, self.y, kind="linear", bounds_error=False,
                      fill_value=(self.y[0], self.y[-1]))
        return fx, fy

    def derivatives(self):
        """Return (vx, vy, ax, ay) reconstructed from position via Savitzky-Golay."""
        dt = float(np.mean(np.diff(self.t)))
        win = max(5, int(0.02 / dt) | 1)   # ~20 ms window, odd
        win = min(win, (len(self.t) // 2) * 2 - 1) if len(self.t) > 6 else 5
        win = max(win, 5)
        poly = 3 if win > 3 else 2
        vx = savgol_filter(self.x, win, poly, deriv=1, delta=dt)
        vy = savgol_filter(self.y, win, poly, deriv=1, delta=dt)
        ax = savgol_filter(self.x, win, poly, deriv=2, delta=dt)
        ay = savgol_filter(self.y, win, poly, deriv=2, delta=dt)
        return vx, vy, ax, ay


def load_csv(path: str) -> Trajectory:
    """Load a CSV with header columns: t,x,y  (seconds, metres, metres)."""
    t, x, y = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            x.append(float(row["x"]))
            y.append(float(row["y"]))
    return Trajectory(np.array(t), np.array(x), np.array(y))


def save_csv(traj: Trajectory, path: str):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y"])
        for i in range(len(traj.t)):
            w.writerow([traj.t[i], traj.x[i], traj.y[i]])


def synth_square(waypoints: List[Tuple[float, float]] = None,
                 feed: float = 0.5, accel: float = 8.0,
                 fs: float = 1000.0) -> Trajectory:
    """Stop-at-corner trapezoidal motion through waypoints -> position-only samples.

    feed  = cruise speed (m/s), accel = accel limit (m/s^2), fs = sample rate (Hz).
    High feed + high accel + sharp corners is exactly the regime that provokes lost steps,
    so the default is deliberately aggressive to make the deviation visible in the demo.
    """
    if waypoints is None:
        waypoints = [(0.1, 0.1), (1.1, 0.1), (1.1, 0.8), (0.1, 0.8), (0.1, 0.1)]
    dt = 1.0 / fs
    ts, xs, ys = [0.0], [waypoints[0][0]], [waypoints[0][1]]
    t = 0.0
    for (x0, y0), (x1, y1) in zip(waypoints[:-1], waypoints[1:]):
        seg = np.array([x1 - x0, y1 - y0])
        L = float(np.linalg.norm(seg))
        if L < 1e-9:
            continue
        u = seg / L
        # trapezoidal profile along the segment (start & end at rest)
        t_acc = feed / accel
        d_acc = 0.5 * accel * t_acc ** 2
        if 2 * d_acc >= L:                    # triangular (never reaches cruise)
            t_acc = math.sqrt(L / accel)
            t_flat = 0.0
            vpk = accel * t_acc
        else:
            t_flat = (L - 2 * d_acc) / feed
            vpk = feed
        T = 2 * t_acc + t_flat
        n = max(1, int(round(T / dt)))
        for i in range(1, n + 1):
            tau = i * dt
            if tau < t_acc:
                s = 0.5 * accel * tau ** 2
            elif tau < t_acc + t_flat:
                s = d_acc + vpk * (tau - t_acc)
            else:
                td = tau - t_acc - t_flat
                s = d_acc + vpk * t_flat + vpk * td - 0.5 * accel * td ** 2
            s = min(s, L)
            p = np.array([x0, y0]) + u * s
            t += dt
            ts.append(t); xs.append(float(p[0])); ys.append(float(p[1]))
    return Trajectory(np.array(ts), np.array(xs), np.array(ys))
