"""Pure coordinate-scaling helpers (no ROS dependency, so they are unit-testable).

Linear map from a machine/workspace coordinate range onto a URDF joint-value range:

    joint = joint_min + (value - machine_min)/(machine_max - machine_min)
                        * (joint_max - joint_min)
"""
from __future__ import annotations

from typing import Tuple


def scale(value: float, machine_min: float, machine_max: float,
          joint_min: float, joint_max: float, clamp: bool = True) -> float:
    span = (machine_max - machine_min)
    if span == 0:
        j = joint_min
    else:
        f = (value - machine_min) / span
        j = joint_min + f * (joint_max - joint_min)
    if clamp:
        lo, hi = (joint_min, joint_max) if joint_min <= joint_max else (joint_max, joint_min)
        j = max(lo, min(hi, j))
    return j


def in_range(value: float, machine_min: float, machine_max: float) -> bool:
    lo, hi = (machine_min, machine_max) if machine_min <= machine_max else (machine_max, machine_min)
    return lo <= value <= hi


def to_metres(value: float, units: str) -> float:
    return value / 1000.0 if str(units).lower() == "mm" else value


def units_factor(units: str) -> float:
    return 0.001 if str(units).lower() == "mm" else 1.0
