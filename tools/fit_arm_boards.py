"""Fit each BAM FB board's thermal node from tonight's logged JTC runs (read-only on the rig's run folders).

Tonight's logs are 4-42 s runs at ~60 Hz with the board temperature in whole degrees, and no current logged between
runs. Over seconds a board with a ~2 min time constant moves 0-5 °C, so the most this data can identify is the
board node, as three combinations:

    dT_b/dt = a·I² − (T_b − T_eq)/τ_b        I² the RMS current over each second

    τ_b            s       the board's time constant
    rise_per_a2    K/A²    a·τ_b: the steady rise per A² of phase current (×I² = the rise at that current)
    T_eq           °C      where the board settles at zero current: ambient + its idle draw + any warm housing under it

The winding is not in this data at all. Each run starts from its own first reading (± a free offset, the
whole-degree step), and the runs from 02:11 UTC on are held out and predicted, against a baseline that just holds
the first reading.

    uv run --extra test python tools/fit_arm_boards.py  > reports/media/arm_boards_fit.json
"""

from __future__ import annotations

import glob
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = "/home/bam/.local/state/rack/submissions"
DRIVES = {"J1": 11, "J2": 12, "C1": 13, "W1": 14}
HELD_OUT_FROM = "021137"  # run_HHMMSS (UTC) at or after this is held out


def runs() -> list[dict]:
    out = []
    for d in sorted(glob.glob(f"{ROOT}/1120-*/runs/real_*/run_*")):
        if not os.path.isdir(d) or not os.path.exists(f"{d}/jtc_ticks.npy"):
            continue
        cols = json.loads(Path(f"{d}/jtc_tick_columns.json").read_text())
        a = np.load(f"{d}/jtc_ticks.npy")
        c = {k: a[:, i] for i, k in enumerate(cols)}
        out.append(dict(path=d, run=os.path.basename(d).removeprefix("run_"), cols=c))
    return out


def per_second(t: np.ndarray, i: np.ndarray, temp: np.ndarray):
    """1 s bins: (bin end time from run start, RMS current over the bin, last reading in the bin)."""
    t = t - t[0]
    edges = np.arange(0.0, t[-1] + 1e-9, 1.0)
    times, irms, temps = [0.0], [abs(i[0])], [temp[0]]
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (t > lo) & (t <= hi)
        if m.any():
            times.append(hi)
            irms.append(float(np.sqrt(np.mean(i[m] ** 2))))
            temps.append(float(temp[m][-1]))
    return np.array(times), np.array(irms), np.array(temps)


def simulate(theta, series):
    """theta = [log a, log tau, T_eq, offset_run_0, ...]; returns the predicted board temperature per run."""
    a, tau, teq = math.exp(theta[0]), math.exp(theta[1]), theta[2]
    preds = []
    for r, (t, i, temp) in enumerate(series):
        x = temp[0] + theta[3 + r]
        out = [x]
        for k in range(1, len(t)):
            dt = t[k] - t[k - 1]
            # exact for a constant current over the bin
            target = teq + a * tau * i[k] ** 2
            x = target + (x - target) * math.exp(-dt / tau)
            out.append(x)
        preds.append(np.array(out))
    return preds


def fit_drive(series, theta0):
    def resid(theta):
        preds = simulate(theta, series)
        r = np.concatenate([(p - s[2]) / 0.6 for p, s in zip(preds, series)])
        # priors: offsets within the half-degree step; tau around the bare board's 127 s (mjbots), loosely
        prior = [(theta[3 + k]) / 0.5 for k in range(len(series))] + [(theta[1] - math.log(127.0)) / 1.0]
        return np.concatenate([r, prior])

    res = least_squares(resid, theta0, method="lm")
    j = res.jac
    dof = max(1, len(res.fun) - len(theta0))
    s2 = float(np.sum(res.fun**2) / dof)
    cov = np.linalg.pinv(j.T @ j) * max(s2, 1.0)
    return res.x, cov


def main() -> None:
    all_runs = runs()
    train = [r for r in all_runs if r["run"] < HELD_OUT_FROM]
    test = [r for r in all_runs if r["run"] >= HELD_OUT_FROM]
    report = dict(runs=[r["run"] for r in all_runs], held_out=[r["run"] for r in test], drives={})
    for name, can in DRIVES.items():
        def series_of(rs):
            return [per_second(r["cols"]["t_s"], r["cols"][f"can{can}_q_current_a"], r["cols"][f"can{can}_temperature_c"])
                    for r in rs]

        tr, te = series_of(train), series_of(test)
        mean_t = float(np.mean([s[2][0] for s in tr]))
        theta0 = np.array([math.log(0.002), math.log(127.0), mean_t - 10.0] + [0.0] * len(tr))
        theta, cov = fit_drive(tr, theta0)
        sd = np.sqrt(np.maximum(np.diag(cov), 0.0))
        a, tau, teq = math.exp(theta[0]), math.exp(theta[1]), theta[2]
        # held out: each run from its own first reading, no offset
        th_test = np.concatenate([theta[:3], np.zeros(len(te))])
        preds = simulate(th_test, te)
        err = np.concatenate([p - s[2] for p, s in zip(preds, te)])
        base = np.concatenate([s[2][0] - s[2] for s in te])
        train_preds = simulate(theta, tr)
        report["drives"][name] = dict(
            can=can,
            tau_s=tau, tau_rel_sd=float(sd[1]),
            rise_per_a2_k=a * tau, rise_per_a2_rel_sd=float(math.hypot(sd[0], sd[1])),
            t_eq_c=teq, t_eq_sd_c=float(sd[2]),
            train_rms_c=float(np.sqrt(np.mean(np.concatenate([p - s[2] for p, s in zip(train_preds, tr)]) ** 2))),
            held_out_rms_c=float(np.sqrt(np.mean(err**2))),
            held_out_baseline_rms_c=float(np.sqrt(np.mean(base**2))),
            temp_range_c=[float(min(s[2].min() for s in tr + te)), float(max(s[2].max() for s in tr + te))],
            series=[dict(run=r["run"], t=s[0].tolist(), i_rms=s[1].tolist(), board=s[2].tolist(), fit=p.tolist())
                    for r, s, p in zip(train + test, tr + te, train_preds + preds)],
        )
    json.dump(report, sys.stdout)


if __name__ == "__main__":
    main()
