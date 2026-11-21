"""The lumped thermal model (see ThermalParams for the equations): continuous-time matrices, a step, a simulator,
steady state and time-to-limit. Standard library only.

State x = [T_w, T_h, T_b, T_a] in °C: winding, housing, board, ambient (ambient has no dynamics here; the observer
lets it drift). Given the current, the model is linear in x — copper's resistance rise α·T_w·I² is a term in A — so
the observer is an exact linear Kalman filter, no linearisation.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from robot_thermal import _linalg as la
from robot_thermal.thermal_params import ThermalParams

W, H, B, A = 0, 1, 2, 3  # state indices
NODES = ("winding", "housing", "board", "ambient")


def matrices(p: ThermalParams, current_a: float) -> tuple[la.Mat, la.Vec]:
    """(A, b) with dx/dt = A·x + b at a constant current (A: phase-current amplitude)."""
    i2 = current_a * current_a
    cw, ch, cb = p.winding_heat_capacity_j_per_k, p.housing_heat_capacity_j_per_k, p.board_heat_capacity_j_per_k
    gwh, gha = p.winding_to_housing_w_per_k, p.housing_to_ambient_w_per_k
    gba, gbh = p.board_to_ambient_w_per_k, p.board_to_housing_w_per_k
    k_cu = 1.5 * p.phase_resistance_ohm * i2  # W at T₀
    a = [
        [(-gwh + k_cu * p.copper_alpha_per_k) / cw, gwh / cw, 0.0, 0.0],
        [gwh / ch, -(gwh + gha + gbh) / ch, gbh / ch, gha / ch],
        [0.0, gbh / cb, -(gba + gbh) / cb, gba / cb],
        [0.0, 0.0, 0.0, 0.0],
    ]
    b = [
        k_cu * (1.0 - p.copper_alpha_per_k * p.resistance_ref_c) / cw,
        0.0,
        (p.board_loss_w_per_a2 * i2 + p.board_idle_w) / cb,
        0.0,
    ]
    return a, b


def copper_loss_w(p: ThermalParams, winding_c: float, current_a: float) -> float:
    return 1.5 * p.phase_resistance_ohm * (1.0 + p.copper_alpha_per_k * (winding_c - p.resistance_ref_c)) * current_a**2


def max_substep_s(p: ThermalParams) -> float:
    """Euler is accurate and stable well below the fastest node's time constant; a tenth of it, at most 1 s."""
    fastest = min(
        p.winding_heat_capacity_j_per_k / p.winding_to_housing_w_per_k,
        p.housing_heat_capacity_j_per_k
        / (p.winding_to_housing_w_per_k + p.housing_to_ambient_w_per_k + p.board_to_housing_w_per_k),
        p.board_time_constant_s,
    )
    return min(1.0, 0.1 * fastest)


def step(p: ThermalParams, x: Sequence[float], current_a: float, dt: float) -> list[float]:
    """x after `dt` seconds at a constant current (forward Euler in substeps of at most max_substep_s)."""
    x = list(x)
    if dt <= 0:
        return x
    a, b = matrices(p, current_a)
    n = max(1, math.ceil(dt / max_substep_s(p)))
    h = dt / n
    for _ in range(n):
        dx = [sum(r[j] * x[j] for j in range(4)) + b[i] for i, r in enumerate(a)]
        x = [x[i] + h * dx[i] for i in range(4)]
    return x


def simulate(
    p: ThermalParams, x0: Sequence[float], times: Sequence[float], currents: Sequence[float]
) -> list[list[float]]:
    """States at each time; the current between two times is the later sample's (zero-order hold)."""
    out, x = [list(x0)], list(x0)
    for k in range(1, len(times)):
        x = step(p, x, currents[k], times[k] - times[k - 1])
        out.append(x)
    return out


def steady_state(p: ThermalParams, current_a: float, ambient_c: float) -> list[float] | None:
    """Where the temperatures settle at a constant current; None past thermal runaway (copper's α·I²·R ≥ G_wh)."""
    a, b = matrices(p, current_a)
    sub = [row[:3] for row in a[:3]]
    rhs = [-(b[i] + a[i][A] * ambient_c) for i in range(3)]
    try:
        x = la.solve(sub, rhs)
    except ZeroDivisionError:
        return None
    if any(v < ambient_c - 1e-6 for v in x):
        return None
    return [*x, ambient_c]


def time_to_limit_s(
    p: ThermalParams, x: Sequence[float], current_a: float, horizon_s: float = 3600.0, resolution_s: float = 0.25
) -> tuple[float, str]:
    """(seconds until the winding or board limit at a constant current, which one); (inf, "") within the horizon.

    Exact for the linear model: the transition over a stride is e^{M·stride} of the augmented [A b; 0 0], strides
    double up to the horizon and then halve back down to `resolution_s` (binary lifting, about 2·log2 steps).
    WHY first-crossing by lifting is safe here: from any state at a constant current every node moves monotonically
    toward its steady state after at most one turn, and the turn sits at the start (a hot winding over a cold
    housing), so the first crossing is found; the test checks it against a fine Euler march.
    """
    def over(v: Sequence[float]) -> str:
        if v[W] >= p.winding_limit_c:
            return "winding"
        if v[B] >= p.board_limit_c:
            return "board"
        return ""

    if over(x):
        return 0.0, over(x)
    a, b = matrices(p, current_a)
    aug = [row[:] + [b[i]] for i, row in enumerate(a)] + [[0.0] * 5]
    n_levels = max(1, math.ceil(math.log2(max(2.0, horizon_s / resolution_s))) + 1)
    phi = la.expm(la.scale(aug, resolution_s))
    mats = []  # mats[k] = transition over resolution_s · 2^k
    for _ in range(n_levels):
        mats.append(phi)
        phi = la.matmul(phi, phi)
    z = [*x, 1.0]
    t = 0.0
    # climb: biggest stride first that does not cross, from the current point
    for k in range(len(mats) - 1, -1, -1):
        stride = resolution_s * (2**k)
        if t + stride > horizon_s:
            continue
        cand = la.matvec(mats[k], z)
        if not over(cand):
            # WHY check the midpoint too: guards a crossing that turns back within the stride (a burst's winding)
            mid = la.matvec(mats[k - 1], z) if k > 0 else cand
            if not over(mid):
                z, t = cand, t + stride
    nxt = la.matvec(mats[0], z)
    if t + resolution_s <= horizon_s and over(nxt):
        return t + resolution_s, over(nxt)
    return math.inf, ""
