"""Integrate both axes over a commanded trajectory and log commanded vs actual.

Both axes are dynamically independent (confirmed: independent Cartesian), so they are
integrated together only for convenience as one 8-state system sharing the time axis:
    state = [ x-axis: theta_m, omega_m, x_c, v_c,  y-axis: theta_m, omega_m, x_c, v_c ]
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.integrate import solve_ivp

from .model import axis_derivatives, initial_state, permanent_offset
from .params import AxisParams
from .trajectory import Trajectory

# Stepper pull-out is at 90 electrical degrees: |Nr*(theta_cmd - theta_m)| > pi/2 means the
# motor can no longer produce the torque the motion demands -> it starts losing steps. Past
# this point the open-loop trajectory is not trustworthy, so we flag the trajectory as
# infeasible at that instant rather than integrating through the (numerically wild) slip.
PULLOUT_ELEC = math.pi / 2


@dataclass
class SimResult:
    t: np.ndarray
    x_cmd: np.ndarray
    y_cmd: np.ndarray
    x_act: np.ndarray      # actual carriage position, machine X (m)
    y_act: np.ndarray      # actual carriage position, machine Y (m)
    perm_offset_x: float   # permanent lost-step offset at end (m); NaN if infeasible
    perm_offset_y: float
    max_err_x: float       # peak |commanded - actual| during motion (m)
    max_err_y: float
    feasible: bool = True
    slip_time: Optional[float] = None    # first pull-out time (s)
    slip_axis: Optional[str] = None      # 'x' or 'y'

    def summary(self) -> str:
        if not self.feasible:
            return (
                f"INFEASIBLE: axis '{self.slip_axis.upper()}' reaches pull-out (loses steps) "
                f"at t={self.slip_time:.3f}s. The trajectory shown is truncated at that point; "
                f"reduce feed/accel or increase motor current to make it executable."
            )
        return (
            f"FEASIBLE (no pull-out).\n"
            f"X: peak tracking err {self.max_err_x*1000:7.3f} mm | "
            f"permanent offset {self.perm_offset_x*1000:7.3f} mm\n"
            f"Y: peak tracking err {self.max_err_y*1000:7.3f} mm | "
            f"permanent offset {self.perm_offset_y*1000:7.3f} mm"
        )


def simulate(traj: Trajectory, px: AxisParams, py: AxisParams,
             log_hz: float = 1000.0, max_step: float = 2e-4) -> SimResult:
    fx, fy = traj.interpolators()

    def rhs(t, s):
        xcmd = float(fx(t)); ycmd = float(fy(t))
        dx = axis_derivatives(s[0:4], xcmd / px.r, px)
        dy = axis_derivatives(s[4:8], ycmd / py.r, py)
        return (*dx, *dy)

    # terminal events: stop integrating the instant either axis crosses pull-out, so we
    # never rely on the unreliable post-slip path.
    def make_event(idx, p, f):
        def ev(t, s):
            return PULLOUT_ELEC - abs(p.Nr * (float(f(t)) / p.r - s[idx]))
        ev.terminal = True
        ev.direction = -1
        return ev
    ev_x = make_event(0, px, fx)
    ev_y = make_event(4, py, fy)

    x0, y0 = float(traj.x[0]), float(traj.y[0])
    s0 = (*initial_state(x0, 0.0, px), *initial_state(y0, 0.0, py))

    t0, t1 = float(traj.t[0]), float(traj.t[-1])
    t_eval = np.arange(t0, t1, 1.0 / log_hz)

    sol = solve_ivp(rhs, (t0, t1), s0, method="LSODA", events=[ev_x, ev_y],
                    t_eval=t_eval, max_step=max_step, rtol=1e-6, atol=1e-8)
    if not sol.success:
        raise RuntimeError(f"integration failed: {sol.message}")

    t = sol.t
    x_act = sol.y[2]
    y_act = sol.y[6]
    theta_m_x = sol.y[0]
    theta_m_y = sol.y[4]
    x_cmd = fx(t)
    y_cmd = fy(t)

    # did a pull-out event fire?
    feasible, slip_time, slip_axis = True, None, None
    tx = sol.t_events[0]
    ty = sol.t_events[1]
    cands = []
    if tx.size:
        cands.append((float(tx[0]), "x"))
    if ty.size:
        cands.append((float(ty[0]), "y"))
    if cands:
        feasible = False
        slip_time, slip_axis = min(cands, key=lambda c: c[0])

    if feasible:
        perm_x = permanent_offset(theta_m_x[-1], x_cmd[-1], px)
        perm_y = permanent_offset(theta_m_y[-1], y_cmd[-1], py)
    else:
        perm_x = perm_y = float("nan")

    return SimResult(
        t=t, x_cmd=x_cmd, y_cmd=y_cmd, x_act=x_act, y_act=y_act,
        perm_offset_x=perm_x, perm_offset_y=perm_y,
        max_err_x=float(np.max(np.abs(x_cmd - x_act))) if t.size else 0.0,
        max_err_y=float(np.max(np.abs(y_cmd - y_act))) if t.size else 0.0,
        feasible=feasible, slip_time=slip_time, slip_axis=slip_axis,
    )


def save_log(res: SimResult, path: str):
    """Write the commanded-vs-actual log consumed by overlay_plot and rviz_player."""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x_cmd", "y_cmd", "x_act", "y_act"])
        for i in range(len(res.t)):
            w.writerow([f"{res.t[i]:.6f}", f"{res.x_cmd[i]:.6f}", f"{res.y_cmd[i]:.6f}",
                        f"{res.x_act[i]:.6f}", f"{res.y_act[i]:.6f}"])


def load_log(path: str) -> SimResult:
    t, xc, yc, xa, ya = [], [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"])); xc.append(float(row["x_cmd"]))
            yc.append(float(row["y_cmd"])); xa.append(float(row["x_act"]))
            ya.append(float(row["y_act"]))
    t = np.array(t)
    return SimResult(t, np.array(xc), np.array(yc), np.array(xa), np.array(ya),
                     float("nan"), float("nan"), float("nan"), float("nan"))
