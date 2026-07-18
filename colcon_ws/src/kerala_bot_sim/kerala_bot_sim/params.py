"""Parameter loading with defaults.

Reads docs/params_template.yaml (the sheet you fill at the bench) and fills any blank
field with a clearly-flagged PLACEHOLDER default so the whole pipeline runs before the
real numbers exist. Every field that falls back to a default is reported, so you always
know which results are still resting on guesses.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


# ---- PLACEHOLDER defaults (SI units). Replace via the YAML as you measure. ----
# These are order-of-magnitude guesses chosen so the demo shows realistic behaviour
# (Y, carrying the whole gantry, lags and slips more than X). They are NOT your machine.
_DEFAULTS = {
    "x": dict(
        moving_mass_m=3.0,            # kg   tool head + carriage
        pulley_pitch_radius_r=None,   # m    (derived from belt block if None)
        friction_coulomb_Fc=5.0,      # N
        friction_viscous_bvisc=5.0,   # N/(m/s)
        belt_stiffness_k=5.0e4,       # N/m
        belt_damping_c=20.0,          # N/(m/s)
        motor_bearing_damping_bm=1e-4,# N*m/(rad/s)
        torque_peak_holding=1.9,      # N*m  holding torque
        torque_speed_curve=None,      # list[(rad/s, N*m)] or None
        omega_corner=100.0,           # rad/s  half-torque speed for default rolloff
        travel_limit_m=1.2,
    ),
    "y": dict(
        moving_mass_m=12.0,           # kg   X gantry + tool head + Y carriage (heavy!)
        pulley_pitch_radius_r=None,
        friction_coulomb_Fc=15.0,
        friction_viscous_bvisc=8.0,
        belt_stiffness_k=5.0e4,
        belt_damping_c=25.0,
        motor_bearing_damping_bm=1e-4,
        torque_peak_holding=1.9,
        torque_speed_curve=None,
        omega_corner=100.0,
        travel_limit_m=0.94,
    ),
}
_DEFAULT_COMMON = dict(
    gravity=9.81,
    full_steps_per_rev=200,
    pulses_per_rev=25000,
    rotor_teeth_Nr=50,
    rotor_inertia_Jm=3.0e-4,   # kg*m^2  PLACEHOLDER
    belt_pitch_mm=2.0,
    belt_pulley_teeth=20,
)


@dataclass
class AxisParams:
    name: str
    m: float
    r: float
    Fc: float
    bvisc: float
    k: float
    c: float
    bm: float
    Jm: float
    Nr: int
    T_hold: float
    omega_corner: float
    torque_speed_curve: Optional[List[Tuple[float, float]]]
    travel_limit_m: float
    pulses_per_rev: int
    used_defaults: List[str] = field(default_factory=list)

    def T_pk(self, omega: float) -> float:
        """Available (pull-out) torque at motor speed |omega| (rad/s)."""
        w = abs(omega)
        if self.torque_speed_curve:
            xs = [p[0] for p in self.torque_speed_curve]
            ys = [p[1] for p in self.torque_speed_curve]
            if w <= xs[0]:
                return ys[0]
            if w >= xs[-1]:
                return ys[-1]
            # linear interp between the two bracketing points
            for i in range(1, len(xs)):
                if w <= xs[i]:
                    f = (w - xs[i - 1]) / (xs[i] - xs[i - 1])
                    return ys[i - 1] + f * (ys[i] - ys[i - 1])
            return ys[-1]
        # default smooth rolloff: half torque at omega_corner
        return self.T_hold / (1.0 + w / self.omega_corner)


def _pick(axis_cfg, common_cfg, key, default, axis_name, used):
    val = None
    if axis_cfg is not None:
        val = axis_cfg.get(key, None)
    if val is None or val == "":
        used.append(f"{axis_name}.{key}")
        return default
    return val


def _build_axis(name, axis_cfg, common_cfg, defaults_common) -> AxisParams:
    d = _DEFAULTS[name]
    used: List[str] = []

    Jm = common_cfg.get("stepper", {}).get("rotor_inertia_Jm") if common_cfg else None
    if Jm in (None, ""):
        used.append("common.stepper.rotor_inertia_Jm")
        Jm = defaults_common["rotor_inertia_Jm"]

    Nr = (common_cfg.get("stepper", {}).get("rotor_teeth_Nr") if common_cfg else None) \
        or defaults_common["rotor_teeth_Nr"]
    pulses = (common_cfg.get("stepper", {}).get("pulses_per_rev") if common_cfg else None) \
        or defaults_common["pulses_per_rev"]

    # pulley pitch radius: axis override, else derive from common.belt, else default calc
    r = None
    if axis_cfg is not None:
        r = axis_cfg.get("pulley_pitch_radius_r", None)
    if r in (None, ""):
        belt = common_cfg.get("belt", {}) if common_cfg else {}
        teeth = belt.get("pulley_teeth") or defaults_common["belt_pulley_teeth"]
        pitch_mm = belt.get("pitch_mm") or defaults_common["belt_pitch_mm"]
        r = (teeth * pitch_mm / 1000.0) / (2 * math.pi)
        used.append(f"{name}.pulley_pitch_radius_r(derived from belt block)")

    return AxisParams(
        name=name,
        m=_pick(axis_cfg, common_cfg, "moving_mass_m", d["moving_mass_m"], name, used),
        r=r,
        Fc=_pick(axis_cfg, common_cfg, "friction_coulomb_Fc", d["friction_coulomb_Fc"], name, used),
        bvisc=_pick(axis_cfg, common_cfg, "friction_viscous_bvisc", d["friction_viscous_bvisc"], name, used),
        k=_pick(axis_cfg, common_cfg, "belt_stiffness_k", d["belt_stiffness_k"], name, used),
        c=_pick(axis_cfg, common_cfg, "belt_damping_c", d["belt_damping_c"], name, used),
        bm=_pick(axis_cfg, common_cfg, "motor_bearing_damping_bm", d["motor_bearing_damping_bm"], name, used),
        Jm=Jm,
        Nr=int(Nr),
        T_hold=_pick(axis_cfg, common_cfg, "torque_peak_holding", d["torque_peak_holding"], name, used),
        omega_corner=d["omega_corner"],
        torque_speed_curve=(axis_cfg.get("torque_speed_curve") if axis_cfg else None) or None,
        travel_limit_m=_pick(axis_cfg, common_cfg, "travel_limit_m", d["travel_limit_m"], name, used),
        pulses_per_rev=int(pulses),
        used_defaults=used,
    )


def load_params(yaml_path: Optional[str] = None, verbose: bool = True):
    """Return {'x': AxisParams, 'y': AxisParams}. Missing file/fields -> defaults."""
    cfg = {}
    if yaml_path and yaml and os.path.exists(yaml_path):
        with open(yaml_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    elif verbose:
        print(f"[params] no YAML at {yaml_path!r}; using ALL placeholder defaults.")

    common = cfg.get("common", {})
    axes = {
        "x": _build_axis("x", cfg.get("axis_x"), common, _DEFAULT_COMMON),
        "y": _build_axis("y", cfg.get("axis_y"), common, _DEFAULT_COMMON),
    }
    if verbose:
        for nm, ap in axes.items():
            if ap.used_defaults:
                print(f"[params] axis '{nm}' using PLACEHOLDER defaults for: "
                      + ", ".join(ap.used_defaults))
    return axes
