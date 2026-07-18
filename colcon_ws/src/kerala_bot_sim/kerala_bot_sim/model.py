"""Per-axis 2-mass dynamics: motor rotor <-belt spring/damper-> carriage.

State per axis (4): [theta_m, omega_m, x_c, v_c]
  theta_m : motor rotor angle (rad)
  omega_m : rotor angular velocity (rad/s)
  x_c     : carriage position (m)
  v_c     : carriage velocity (m/s)

The controller is open-loop: it commands the motor field to the angle that WOULD place
the carriage at x_cmd assuming a rigid belt, i.e. theta_cmd = x_cmd / r. The rotor follows
via the stepper torque-angle curve:

    tau_motor = T_pk(omega_m) * sin( Nr * (theta_cmd - theta_m) )

When the load demands more torque than T_pk, the electrical lead angle Nr*(theta_cmd-theta_m)
is driven past pi/2, torque collapses, and the rotor slips a pole -> LOST STEPS. Because
nothing pulls the rotor back (no encoder), that error is permanent and accumulates. This
is the mechanism that produces the offset stacking you see on the real machine.
"""
from __future__ import annotations

import math

from .params import AxisParams


def _smooth_sign(v: float, eps: float = 1e-3) -> float:
    """tanh-smoothed sign to keep Coulomb friction from chattering the ODE at v=0."""
    return math.tanh(v / eps)


def axis_derivatives(state, theta_cmd: float, p: AxisParams):
    """Return d/dt of [theta_m, omega_m, x_c, v_c] for one axis."""
    theta_m, omega_m, x_c, v_c = state

    # belt acts as a spring+damper between motor-pulley surface (r*theta_m) and carriage
    belt_stretch = p.r * theta_m - x_c
    belt_rate = p.r * omega_m - v_c
    F_belt = p.k * belt_stretch + p.c * belt_rate

    # stepper torque-angle (pole-slip emerges from the sin term)
    lead = p.Nr * (theta_cmd - theta_m)
    tau_motor = p.T_pk(omega_m) * math.sin(lead)

    # motor: driven by stepper torque, loaded by belt reaction + bearing drag
    domega_m = (tau_motor - p.r * F_belt - p.bm * omega_m) / p.Jm
    dtheta_m = omega_m

    # carriage: driven by belt, opposed by guide friction (Coulomb + viscous)
    F_friction = p.Fc * _smooth_sign(v_c) + p.bvisc * v_c
    dv_c = (F_belt - F_friction) / p.m
    dx_c = v_c

    return (dtheta_m, domega_m, dx_c, dv_c)


def initial_state(x0: float, v0: float, p: AxisParams):
    """Start tracking cleanly: rotor placed so belt is unstretched at the start pose."""
    theta_m0 = x0 / p.r
    omega_m0 = v0 / p.r
    return (theta_m0, omega_m0, x0, v0)


def permanent_offset(theta_m_final: float, x_cmd_final: float, p: AxisParams) -> float:
    """Offset (m) that survives after motion stops and the belt relaxes.

    Once still and unstretched the carriage sits at r*theta_m, so the permanent error vs
    the commanded end position is x_cmd_final - r*theta_m_final = accumulated lost steps.
    """
    return x_cmd_final - p.r * theta_m_final
