from __future__ import annotations

import math


def idm_acceleration(
    v: float,         # current speed (m/s)
    v0: float,        # target speed (m/s)
    s: float | None,  # actual gap to leader (m), None = no leader
    dv: float = 0.0,  # speed difference (ego - leader, m/s)
    *,
    a: float = 2.0,     # max acceleration (m/s²)
    b: float = 1.5,     # comfortable deceleration (m/s²)
    s0: float = 2.0,    # minimum gap (m)
    T: float = 1.5,     # desired time headway (s)
    delta: float = 4.0, # acceleration exponent
) -> float:
    """IDM (Intelligent Driver Model) acceleration.

    Returns acceleration in m/s².  Positive = accelerate, negative = decelerate.

    Reference: Treiber, Hennecke, Helbing (2000)
    """
    v = max(0.0, v)
    v0 = max(0.1, v0)

    # Base: free-road acceleration
    accel = a * (1.0 - (v / v0) ** delta)

    # Car-following term (skip if no leader or gap is None)
    if s is not None and s > 0.0:
        # Desired gap: s* = s0 + max(0, v*T + v*dv/(2*sqrt(a*b)))
        s_star = s0 + max(0.0, v * T + v * dv / (2.0 * math.sqrt(max(a * b, 0.01))))
        # s*/s ratio squared
        accel -= a * (s_star / s) ** 2

    return accel
