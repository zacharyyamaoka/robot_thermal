"""The v2 thermal observer: a Kalman filter over [winding, housing, board, ambient], standard library only.

    s = start(params, t_s, board_c, current_a)
    every tick:   s = update(params, hot, s, t_s, current_a, board_c=..., winding_c=None, housing_c=None)
    when needed:  e = estimate(params, hot, s)      # e.winding_c ± e.winding_sd_c, e.winding_hi_c, e.time_to_limit_s

`hot = params.hot(config.hot_k_sd)` is the pessimistic network. The filter fuses the model with whatever is
measured: the board temperature (every moteus reports it), a resistance-derived winding temperature
(`winding_from_resistance`), a housing thermistor. Losses come from the logged current: copper I²R with R rising
with the winding's own estimated temperature, and the board's conduction loss.

Why two models. With only the board measured, the winding branch is not observable (it reaches the board only
through the housing, if at all), so a filter alone would report the winding with a confidence it has not earned.
The hot bound carries the winding forward with every capacity and conductance on the pessimistic side of its prior;
protection decisions (`winding_hi_c`, `time_to_limit_s`) read it, and a direct winding or housing measurement resets
it to the filter's mean + 2 sd. The prior's spread is therefore the price of not measuring the winding, stated in °C.
"""

from __future__ import annotations

import math

from robot_thermal import thermal as th
from robot_thermal.observer_config import ObserverConfig
from robot_thermal.observer_state import ObserverState
from robot_thermal.thermal_estimate import ThermalEstimate
from robot_thermal.thermal_params import ThermalParams

__all__ = ["ObserverConfig", "ObserverState", "estimate", "start", "update", "winding_from_resistance"]

DEFAULT = ObserverConfig()


def winding_from_resistance(p: ThermalParams, resistance_ohm: float) -> float:
    """°C from a measured phase resistance: R = R₀·(1 + α·(T − T₀))."""
    return p.resistance_ref_c + (resistance_ohm / p.phase_resistance_ohm - 1.0) / p.copper_alpha_per_k


def start(
    p: ThermalParams,
    t_s: float,
    board_c: float,
    current_a: float = 0.0,
    config: ObserverConfig = DEFAULT,
    ambient_c: float | None = None,
) -> ObserverState:
    """Assume the drive has sat at `current_a` long enough to settle: ambient follows from the board reading."""
    rise = th.steady_state(p, current_a, 0.0) or [0.0, 0.0, 0.0, 0.0]
    amb = board_c - rise[th.B] if ambient_c is None else ambient_c
    mean = (rise[0] + amb, rise[1] + amb, board_c, amb)
    sd = config.initial_sd_c
    cov = tuple(tuple(sd[i] ** 2 if i == j else 0.0 for j in range(4)) for i in range(4))
    hot = p.hot(config.hot_k_sd)
    hot_rise = th.steady_state(hot, current_a, 0.0) or rise
    return ObserverState(t_s=float(t_s), mean_c=mean, cov=cov, hot_c=(hot_rise[0] + amb, hot_rise[1] + amb),
                         current_a=abs(current_a))


def _predict(p: ThermalParams, hot: ThermalParams, s: ObserverState, t_s: float, current_a: float,
             config: ObserverConfig, extra_heat_w: float = 0.0) -> tuple[list[float], list[list[float]], list[float]]:
    dt = max(0.0, float(t_s) - s.t_s)
    x = list(s.mean_c)
    cov = [list(r) for r in s.cov]
    hot_x = [s.hot_c[0], s.hot_c[1], x[th.B], x[th.A]]
    if dt == 0.0:
        return x, cov, hot_x
    # WHY the mean of the two samples' I²: the heat delivered between them, for an estimator (the v1 bucket takes
    # the larger, being a protection rule)
    i_eff = math.sqrt(0.5 * (s.current_a**2 + current_a**2))
    a, _ = th.matrices(p, i_eff)
    n = max(1, math.ceil(dt / th.max_substep_s(p)))
    h = dt / n
    p_cu = th.copper_loss_w(p, x[th.W], i_eff) + extra_heat_w
    p_board = p.board_loss_w_per_a2 * i_eff**2 + p.board_idle_w
    q = (
        config.winding_noise_k2_per_s
        + (config.loss_rel_sd * p_cu / p.winding_heat_capacity_j_per_k) ** 2 * config.loss_correlation_s,
        config.housing_noise_k2_per_s,
        config.board_noise_k2_per_s
        + (config.loss_rel_sd * p_board / p.board_heat_capacity_j_per_k) ** 2 * config.loss_correlation_s,
        config.ambient_noise_k2_per_s,
    )
    f = [[(1.0 if i == j else 0.0) + h * a[i][j] for j in range(4)] for i in range(4)]
    for _ in range(n):
        fc = [[sum(f[i][k] * cov[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
        cov = [[sum(fc[i][k] * f[j][k] for k in range(4)) + (q[i] * h if i == j else 0.0) for j in range(4)]
               for i in range(4)]
    x = th.step(p, x, i_eff, dt, extra_heat_w)
    hot_x = th.step(hot, hot_x, i_eff, dt, extra_heat_w)
    return x, cov, hot_x


def _correct(x: list[float], cov: list[list[float]], node: int, value: float, sd: float) -> None:
    """Scalar Kalman update of one measured node, in place."""
    s = cov[node][node] + sd * sd
    k = [cov[i][node] / s for i in range(4)]
    innov = value - x[node]
    for i in range(4):
        x[i] += k[i] * innov
    row = list(cov[node])
    for i in range(4):
        for j in range(4):
            cov[i][j] -= k[i] * row[j]
    for i in range(4):  # keep it symmetric against rounding
        for j in range(i + 1, 4):
            m = 0.5 * (cov[i][j] + cov[j][i])
            cov[i][j] = cov[j][i] = m


def update(
    p: ThermalParams,
    hot: ThermalParams,
    s: ObserverState,
    t_s: float,
    current_a: float,
    board_c: float | None = None,
    winding_c: float | None = None,
    housing_c: float | None = None,
    config: ObserverConfig = DEFAULT,
    extra_heat_w: float = 0.0,
) -> ObserverState:
    """Predict to `t_s` on the current since the last update, then fuse whichever measurements are given.

    `current_a` is the phase-current magnitude √(i_q² + i_d²) at `t_s`. `extra_heat_w` is winding heat the current
    does not show: flux braking on a drive whose d current is not logged. `burst.flux_brake_heat_w(τ, ω)` estimates
    it for this drive; `burst.bus_regen_w` over every drive bounds it.
    """
    current_a = abs(float(current_a))
    x, cov, hot_x = _predict(p, hot, s, t_s, current_a, config, extra_heat_w)
    if board_c is not None and math.isfinite(board_c):
        _correct(x, cov, th.B, float(board_c), config.board_sd_c)
    direct = False
    if winding_c is not None and math.isfinite(winding_c):
        _correct(x, cov, th.W, float(winding_c), config.winding_sd_c)
        direct = True
    if housing_c is not None and math.isfinite(housing_c):
        _correct(x, cov, th.H, float(housing_c), config.housing_sd_c)
        direct = True
    hot_c = (hot_x[0], hot_x[1])
    if direct:
        hot_c = (x[th.W] + 2.0 * math.sqrt(max(cov[th.W][th.W], 0.0)),
                 x[th.H] + 2.0 * math.sqrt(max(cov[th.H][th.H], 0.0)))
    return ObserverState(
        t_s=max(float(t_s), s.t_s),
        mean_c=(x[0], x[1], x[2], x[3]),
        cov=tuple(tuple(r) for r in cov),
        hot_c=hot_c,
        current_a=current_a,
        updates=s.updates + 1,
    )


def estimate(
    p: ThermalParams, hot: ThermalParams, s: ObserverState, current_a: float | None = None,
    horizon_s: float = 3600.0,
) -> ThermalEstimate:
    """The estimate now, and the time to a limit if the drive holds `current_a` (default: the last current)."""
    x, cov = s.mean_c, s.cov
    sd = [math.sqrt(max(cov[i][i], 0.0)) for i in range(4)]
    i = s.current_a if current_a is None else abs(current_a)
    hot_state = [s.hot_c[0], s.hot_c[1], x[th.B] + 2.0 * sd[th.B], x[th.A]]
    ttl, which = th.time_to_limit_s(hot, hot_state, i, horizon_s=horizon_s)
    return ThermalEstimate(
        winding_c=x[th.W],
        winding_sd_c=sd[th.W],
        winding_hi_c=max(x[th.W] + 2.0 * sd[th.W], s.hot_c[0]),
        housing_c=x[th.H],
        housing_sd_c=sd[th.H],
        board_c=x[th.B],
        board_sd_c=sd[th.B],
        ambient_c=x[th.A],
        copper_loss_w=th.copper_loss_w(p, x[th.W], i),
        time_to_limit_s=ttl,
        limit=which,
    )
