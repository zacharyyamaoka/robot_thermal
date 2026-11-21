"""Identify a drive's thermal parameters from a log: maximum a posteriori least squares, standard library only.

    result = fit(prior, times, currents, board_c, winding=[(t, °C), ...], free=(...))

The model is simulated open loop from rest-equilibrium at the log's first sample (ambient is fitted with it) and
compared with the board readings and any winding measurements (resistance-derived or a thermistor). Every free
parameter works in log space with its prior as a penalty, so a parameter the experiment cannot excite stays at its
prior and its posterior spread says so (practical identifiability): the fit never invents a winding time constant
from board data alone. Levenberg–Marquardt with a forward-difference Jacobian; scipy is its oracle in the tests.

WHY open loop and not the filter: the filter would explain a wrong parameter away as process noise; open loop puts
the whole misfit on the parameters, which is what identification needs.
"""

from __future__ import annotations

import math
from bisect import bisect_left
from collections.abc import Sequence
from dataclasses import replace

from robot_thermal import _linalg as la
from robot_thermal import thermal as th
from robot_thermal.fit_result import FitResult
from robot_thermal.thermal_params import ThermalParams

AMBIENT_PRIOR_SD_C = 10.0


def _with(prior: ThermalParams, free: Sequence[str], theta: Sequence[float]) -> ThermalParams:
    return replace(prior, **{name: math.exp(v) for name, v in zip(free, theta)})


def predict_board_and_winding(
    p: ThermalParams, ambient_c: float, times: Sequence[float], currents: Sequence[float]
) -> list[list[float]]:
    """The open-loop states at `times`, from rest-equilibrium at the first sample's current."""
    rise = th.steady_state(p, currents[0], 0.0) or [0.0, 0.0, 0.0, 0.0]
    x0 = [rise[0] + ambient_c, rise[1] + ambient_c, rise[2] + ambient_c, ambient_c]
    return th.simulate(p, x0, times, currents)


def fit(
    prior: ThermalParams,
    times: Sequence[float],
    currents: Sequence[float],
    board_c: Sequence[float],
    winding: Sequence[tuple[float, float]] = (),
    free: Sequence[str] = (),
    board_sd_c: float = 0.6,
    winding_sd_c: float = 2.5,
    ambient_prior_c: float | None = None,
    max_iterations: int = 40,
) -> FitResult:
    free = tuple(free)
    for name in free:
        if prior.log_sd(name) <= 0:
            raise ValueError(f"{name} has no prior spread in {prior.name}; give it one to fit it")
    amb0 = board_c[0] if ambient_prior_c is None else ambient_prior_c
    theta0 = [math.log(getattr(prior, n)) for n in free] + [amb0]
    sds = [prior.log_sd(n) for n in free] + [AMBIENT_PRIOR_SD_C]
    w_idx = [min(len(times) - 1, bisect_left(times, t)) for t, _ in winding]

    def residuals(theta: Sequence[float]) -> list[float]:
        p = _with(prior, free, theta[:-1])
        xs = predict_board_and_winding(p, theta[-1], times, currents)
        r = [(xs[k][th.B] - board_c[k]) / board_sd_c for k in range(len(times))]
        r += [(xs[k][th.W] - meas) / winding_sd_c for k, (_, meas) in zip(w_idx, winding)]
        r += [(theta[i] - theta0[i]) / sds[i] for i in range(len(theta))]
        return r

    def cost(r: Sequence[float]) -> float:
        return 0.5 * sum(v * v for v in r)

    theta = list(theta0)
    r = residuals(theta)
    c = cost(r)
    lam, it = 1e-2, 0
    jac: list[list[float]] = []
    for it in range(1, max_iterations + 1):
        jac = _jacobian(residuals, theta, r)
        jt = la.transpose(jac)
        jtj = la.matmul(jt, jac)
        g = la.matvec(jt, r)
        improved = False
        for _ in range(12):
            a = [[jtj[i][j] + (lam * jtj[i][i] if i == j else 0.0) for j in range(len(theta))]
                 for i in range(len(theta))]
            try:
                delta = la.solve(a, [-v for v in g])
            except ZeroDivisionError:
                lam *= 10
                continue
            cand = [t + d for t, d in zip(theta, delta)]
            rc = residuals(cand)
            cc = cost(rc)
            if cc < c:
                improved = True
                rel = (c - cc) / max(c, 1e-12)
                theta, r, c = cand, rc, cc
                lam = max(lam / 3, 1e-9)
                break
            lam *= 4
        if not improved or rel < 1e-10:
            break
    jac = _jacobian(residuals, theta, r)
    jtj = la.matmul(la.transpose(jac), jac)
    try:
        post = la.inverse(jtj)
        post_sd = [math.sqrt(max(post[i][i], 0.0)) for i in range(len(theta))]
    except ZeroDivisionError:
        post_sd = list(sds)
    p = _with(prior, free, theta[:-1])
    xs = predict_board_and_winding(p, theta[-1], times, currents)
    board_rms = math.sqrt(sum((xs[k][th.B] - board_c[k]) ** 2 for k in range(len(times))) / len(times))
    w_rms = (math.sqrt(sum((xs[k][th.W] - m) ** 2 for k, (_, m) in zip(w_idx, winding)) / len(winding))
             if winding else None)
    return FitResult(
        params=replace(p, name=prior.name + " (fit)",
                       prior_log_sd=tuple((n, post_sd[i]) if n in free else (n, prior.log_sd(n))
                                          for i, n in enumerate(free)) +
                       tuple((n, sd) for n, sd in prior.prior_log_sd if n not in free)),
        ambient_c=theta[-1],
        spread=tuple((n, sds[i], post_sd[i]) for i, n in enumerate(free)),
        board_rms_c=board_rms,
        winding_rms_c=w_rms,
        iterations=it,
        cost=c,
    )


def _jacobian(fun, theta: list[float], r0: list[float]) -> list[list[float]]:
    cols = []
    for i in range(len(theta)):
        h = 1e-5 if i < len(theta) - 1 else 1e-3
        t = list(theta)
        t[i] += h
        ri = fun(t)
        cols.append([(a - b) / h for a, b in zip(ri, r0)])
    return la.transpose(cols)
