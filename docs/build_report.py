"""Build the robot_thermal report: every number and figure recomputed from the tested code and tonight's logs.

    env -u VIRTUAL_ENV uv run --directory /home/bam/robot_thermal --extra test --extra report python docs/build_report.py

Writes /home/bam/robot_thermal/reports/media/thermally-limited-servoing-2026-11-20.html (self-contained; its inline
figures are over 256 KB, so it lives in the gitignored reports/media/ and is rebuilt from this script).
"""

from __future__ import annotations

import base64
import html
import io
import json
import math
import statistics
import subprocess
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "tools"))

import test_hidden_truth as H  # noqa: E402

from robot_thermal import burst as bu  # noqa: E402
from robot_thermal import observer as ob  # noqa: E402
from robot_thermal import thermal as th  # noqa: E402
from robot_thermal.drives import arm  # noqa: E402
from robot_thermal.drives.gim4305_10 import rule as GIM_RULE  # noqa: E402
from robot_thermal.drives.gim4305_10_thermal import thermal as GIM_T  # noqa: E402
from robot_thermal.drives.jmc_60asm400_ad60 import rule as JMC_RULE  # noqa: E402
from robot_thermal.drives.jmc_60asm400_ad60_thermal import thermal as JMC_T  # noqa: E402

OUT = ROOT / "reports" / "media" / "thermally-limited-servoing-2026-11-20.html"
FIT_JSON = ROOT / "reports" / "media" / "arm_boards_fit.json"
INK, MUTED, GRID = "#1f2328", "#6b7280", "#e5e7eb"
C_TRUE, C_EST, C_HOT, C_MEAS, C_BOARD = "#111827", "#2563eb", "#dc2626", "#9ca3af", "#d97706"

plt.rcParams.update({"font.size": 9.5, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
                     "figure.dpi": 110, "savefig.bbox": "tight"})


def png(fig) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110)
    plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()


def git_sha() -> str:
    return subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True,
                          text=True).stdout.strip()


# --- 1. the v1 burst budget in action --------------------------------------------------------------------------------

def fig_burst():
    """A greedy controller asks for 28.1 A for 8 s every 20 s on a JMC drive; the rule folds it back."""
    rule, p = arm.RULES["J2"], JMC_T
    s, x = bu.update(rule, bu.BurstState(), 0.0, 0.0, 30.0), [25.0, 25.0, 30.0, 22.0]
    dt, ts, want, got, left, board = 0.05, [], [], [], [], []
    t = 0.0
    while t < 300.0:
        demand = rule.peak_current_a if (t % 60.0) < 8.0 else 10.0
        lim = bu.verdict(rule, s, 10.0).current_limit_a
        i = min(demand, lim)
        t += dt
        x = th.step(p, x, i, dt)
        s = bu.update(rule, s, t, i, x[th.B])
        ts.append(t); want.append(demand); got.append(i); left.append(bu.verdict(rule, s).budget_left_pct)
        board.append(x[th.B])
    fig, ax = plt.subplots(2, 1, figsize=(8.6, 4.2), sharex=True, height_ratios=[1.3, 1])
    ax[0].plot(ts, want, color=C_MEAS, lw=1.0, label="asked for")
    ax[0].plot(ts, got, color=C_EST, lw=1.4, label="allowed (clamped to the verdict)")
    ax[0].axhline(rule.continuous_current_a, color=MUTED, ls="--", lw=0.8)
    ax[0].text(301, rule.continuous_current_a, f" I_cont {rule.continuous_current_a:.1f} A", va="center", color=MUTED,
               fontsize=8)
    ax[0].set_ylabel("q current (A)")
    ax[0].legend(loc="upper right", ncol=2, fontsize=8)
    ax[1].plot(ts, left, color=C_HOT, lw=1.3)
    ax[1].set_ylabel("budget left (%)")
    ax[1].set_xlabel("time (s) — J2-like drive holding 10 A between bursts")
    ax[1].set_ylim(-5, 105)
    fig.suptitle(f"v1 burst budget, J2: 8 s at {rule.peak_current_a:.1f} A asked every minute, 5 s given, refilled over the minute",
                 fontsize=10, x=0.01, ha="left")
    return png(fig)


def replay_rows():
    sys.path.insert(0, str(ROOT / "tools"))
    import numpy as np
    from replay_v1 import CAN, load

    run = Path("/home/bam/.local/state/rack/submissions/1120-175952-740f/runs/real_20261121_010013/run_010030")
    d = load(run)
    rows = []
    for joint, can in CAN.items():
        rule = arm.RULES[joint]
        cur, temp = d[f"can{can}_q_current_a"], d[f"can{can}_temperature_c"]
        s, worst = bu.BurstState(), 100.0
        for k in range(len(d["t_s"])):
            s = bu.update(rule, s, float(d["t_s"][k]), float(cur[k]), float(temp[k]))
            worst = min(worst, bu.verdict(rule, s).budget_left_pct)
        rows.append((joint, float(np.max(np.abs(cur))), float(np.sqrt(np.mean(cur**2))), rule.continuous_current_a,
                     rule.peak_current_a, worst, float(np.max(temp))))
    return rows


# --- 2. the hidden-truth proof ---------------------------------------------------------------------------------------

def fig_e1(e1, fitted):
    t = [v / 60 for v in e1["times"]]
    truth = e1["truth"]
    model = __import__("robot_thermal.fit", fromlist=["x"]).predict_board_and_winding(
        fitted.params, fitted.ambient_c, e1["times"], e1["currents"])
    fig, ax = plt.subplots(2, 1, figsize=(8.6, 4.6), sharex=True, height_ratios=[2.2, 1])
    ax[0].plot(t, [x[th.W] for x in truth], color=C_TRUE, lw=1.6, label="hidden truth: winding")
    ax[0].plot(t, [x[th.W] for x in model], color=C_EST, lw=1.1, ls="--", label="fitted model: winding")
    ax[0].plot(t, [x[th.B] for x in truth], color=C_BOARD, lw=1.4, label="hidden truth: board")
    ax[0].step(t, e1["board"], where="post", color=C_MEAS, lw=0.8, label="board reading (whole °C)")
    ax[0].plot([w[0] / 60 for w in e1["winding"]], [w[1] for w in e1["winding"]], "o", ms=3.2, color=C_HOT,
               label="resistance check (±2.5 °C)")
    ax[0].set_ylabel("°C")
    ax[0].legend(loc="upper right", fontsize=7.5, ncol=2)
    ax[1].plot(t, e1["currents"], color=INK, lw=0.9)
    ax[1].set_ylabel("d current (A)")
    ax[1].set_xlabel(f"minutes — E1: idle, 10 min at {H.E1_HEAT_A:.0f} A, cool, 5 × (5 s at 28.1 A), cool")
    fig.suptitle("E1 on the hidden plant: what the fit sees, and the model it recovers", fontsize=10, x=0.01,
                 ha="left")
    return png(fig)


def fig_heldout(runs):
    fig, axes = plt.subplots(len(runs), 1, figsize=(8.6, 2.35 * len(runs)), sharex=True)
    for ax, (title, r) in zip(axes, runs):
        tr = r["trace"]
        t = [v / 60 for v in tr["t"]]
        lo = [m - 2 * s for m, s in zip(tr["est_w"], tr["sd_w"])]
        hi = [m + 2 * s for m, s in zip(tr["est_w"], tr["sd_w"])]
        ax.fill_between(t, lo, hi, color=C_EST, alpha=0.15, lw=0, label="estimate ± 2 sd")
        ax.plot(t, tr["true_w"], color=C_TRUE, lw=1.5, label="hidden truth: winding")
        ax.plot(t, tr["est_w"], color=C_EST, lw=1.0, label="estimate")
        ax.plot(t, tr["hi_w"], color=C_HOT, lw=1.0, ls="--", label="bound a protection reads")
        ax.set_ylabel("winding °C")
        ax.set_title(f"{title} — RMS {r['rms']:.1f} °C, max {r['max']:.1f} °C, "
                     f"bound above truth {100 * r['bound']:.0f} % of the time", fontsize=8.5, loc="left")
    axes[0].legend(loc="lower right", fontsize=7.5, ncol=4)
    axes[-1].set_xlabel("minutes — held-out arm duty: 10 A hold, moves to 18 A, 3 s at 28.1 A every 40 s")
    return png(fig)


# --- 3. tonight's real boards -------------------------------------------------------------------------------------

def fig_real(fit):
    names = ["J2", "C1", "J1"]
    fig, axes = plt.subplots(len(names), 1, figsize=(8.6, 2.3 * len(names)))
    held = set(fit["held_out"])
    for ax, name in zip(axes, names):
        d = fit["drives"][name]
        x0 = 0.0
        for k, s in enumerate(d["series"]):
            t = [x0 + v for v in s["t"]]
            col = C_HOT if s["run"] in held else C_EST
            ax.step(t, s["board"], where="post", color=C_MEAS, lw=0.9, label="board reading" if k == 0 else None)
            ax.plot(t, s["fit"], color=col, lw=1.3,
                    label=("held-out prediction" if s["run"] in held else "fit") if s["run"] in
                    (d["series"][0]["run"], sorted(held)[0]) else None)
            x0 = t[-1] + 8.0
        ax.set_ylabel(f"{name} board °C")
        ax.set_title(f"{name}: τ_b {d['tau_s']:.0f} s ± {100 * d['tau_rel_sd']:.0f} %, held-out RMS "
                     f"{d['held_out_rms_c']:.2f} °C vs {d['held_out_baseline_rms_c']:.2f} °C for 'no change'",
                     fontsize=8.5, loc="left")
        ax.set_xticks([])
    axes[0].legend(loc="upper right", fontsize=7.5, ncol=3)
    axes[-1].set_xlabel("tonight's 18 JTC runs side by side (gaps between runs removed)")
    return png(fig)


# --- 4. what the model predicts --------------------------------------------------------------------------------------

def fig_ttl():
    fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.2))
    for ax, (p, rule, cur, title) in zip(axes, [(JMC_T, JMC_RULE, [16, 18, 20, 22, 24, 26, 28.1, 32, 36, 40],
                                                 "JMC shoulder (board cap 28.1 A)"),
                                                (GIM_T, GIM_RULE, [2.1, 2.5, 3, 4, 5, 6, 7.5],
                                                 "GIM wrist (cap 2.0 A today)")]):
        for params, col, lab in ((p, C_EST, "model, prior"), (p.hot(1.0), C_HOT, "hot bound")):
            ys, which = [], []
            for i in cur:
                x0 = [25.0, 25.0, 25.0, 25.0]
                ttl, w = th.time_to_limit_s(params, x0, i, horizon_s=3600.0)
                ys.append(min(ttl, 3600.0))
                which.append(w)
            ax.plot(cur, ys, "-", color=col, lw=1.2, label=lab)
            for xv, yv, w in zip(cur, ys, which):
                ax.plot([xv], [yv], "^" if w == "board" else "o", color=col, ms=4.5)
        ax.plot(cur, [min(bu.burst_seconds_at(rule, bu.BurstState(), i), 3600.0) for i in cur], "s--", ms=3,
                color=MUTED, lw=1.0, label="v1 rule: burst from a full budget")
        ax.set_yscale("log")
        ax.set_xlabel("held current (A), from 25 °C")
        ax.set_title(title, fontsize=9, loc="left")
    axes[0].set_ylabel("seconds until a limit (capped at 1 h)")
    axes[0].legend(fontsize=7.5)
    return png(fig)


# --- 5. timing ----------------------------------------------------------------------------------------------------

def timing():
    def bench(fn, n):
        out = []
        for _ in range(n):
            t0 = time.perf_counter()
            fn()
            out.append(time.perf_counter() - t0)
        out.sort()
        return statistics.median(out), out[int(0.95 * len(out))]

    s = bu.update(JMC_RULE, bu.BurstState(), 0.0, 10.0, 40.0)
    tick = [0.0]

    def burst_tick():
        tick[0] += 0.017
        bu.verdict(JMC_RULE, bu.update(JMC_RULE, s, tick[0], 20.0, 40.0))

    hot = JMC_T.hot(1.0)
    st = ob.start(JMC_T, 0.0, 40.0, 10.0)

    def obs_tick():
        ob.update(JMC_T, hot, st, 0.017, 12.0, board_c=40.0)

    return dict(burst=bench(burst_tick, 5000), observer=bench(obs_tick, 3000),
                estimate=bench(lambda: ob.estimate(JMC_T, hot, st, 28.1), 300))


# --- page --------------------------------------------------------------------------------------------------------

def main() -> None:
    e1 = H.make_e1()
    fitted = H.make_fit(e1)
    blind = __import__("robot_thermal.fit", fromlist=["x"]).fit(H.PRIOR, e1["times"], e1["currents"], e1["board"],
                                                                 winding=(), free=H.FREE)
    with_r = H.run_observer(fitted.params, H.ARM, 1200.0, with_resistance=True, trace=True)
    board_only = H.run_observer(fitted.params, H.ARM, 1200.0, with_resistance=False, trace=True)
    prior_only = H.run_observer(H.PRIOR, H.ARM, 1200.0, with_resistance=False, trace=True)
    fit = json.loads(FIT_JSON.read_text())
    tm = timing()
    rows = replay_rows()
    sha = git_sha()

    param_rows = []
    for name, prior_sd, post in fitted.spread:
        true, pri, got = getattr(H.TRUTH, name), getattr(H.PRIOR, name), getattr(fitted.params, name)
        b_post = blind.rel_sd(name)
        param_rows.append(f"<tr><td><code>{name}</code></td><td>{true:.4g}</td><td>{pri:.4g}</td><td>{got:.4g}</td>"
                          f"<td class='{'ok' if abs(got / true - 1) <= (0.25 if name in H.CORE else 0.5) else 'bad'}'>"
                          f"{100 * (got / true - 1):+.0f} %</td><td>{prior_sd:.2f} → <b>{post:.2f}</b></td>"
                          f"<td>{b_post:.2f}</td></tr>")
    real_rows = []
    for name, d in fit["drives"].items():
        ident = d["tau_rel_sd"] < 0.5
        real_rows.append(
            f"<tr><td>{name} (CAN {d['can']})</td><td>{d['temp_range_c'][0]:.0f}–{d['temp_range_c'][1]:.0f}</td>"
            f"<td>{d['tau_s']:.0f} s ± {100 * d['tau_rel_sd']:.0f} %</td>"
            f"<td>{d['rise_per_a2_k']:.2f} K/A² ± {100 * min(d['rise_per_a2_rel_sd'], 9.99):.0f} %</td>"
            f"<td>{d['t_eq_c']:.0f} ± {d['t_eq_sd_c']:.0f} °C</td>"
            f"<td>{d['held_out_rms_c']:.2f} vs {d['held_out_baseline_rms_c']:.2f}</td>"
            f"<td class='{'ok' if ident else 'warn'}'>{'τ identified' if ident else 'not identified'}</td></tr>")
    rep_rows = "".join(
        f"<tr><td>{j}</td><td>{pk:.2f}</td><td>{rms:.2f}</td><td>{ic:.1f} / {ip:.1f}</td><td>{w:.0f} %</td>"
        f"<td>{bt:.0f}</td></tr>" for j, pk, rms, ic, ip, w, bt in rows)

    def us(v):
        return f"{1e6 * v:.0f} µs"

    spread = {n: (pr, po) for n, pr, po in fitted.spread}
    wb = [spread[n][1] / spread[n][0] for n in ("winding_heat_capacity_j_per_k", "winding_to_housing_w_per_k",
                                                 "housing_heat_capacity_j_per_k", "housing_to_ambient_w_per_k")]
    bd = [spread[n][1] / spread[n][0] for n in ("board_heat_capacity_j_per_k", "board_to_ambient_w_per_k")]
    bd_err = max(abs(getattr(fitted.params, n) / getattr(H.TRUTH, n) - 1)
                 for n in ("board_heat_capacity_j_per_k", "board_to_ambient_w_per_k"))

    def ratio(name, res):
        return f"{res.rel_sd(name) / H.PRIOR.log_sd(name):.2f}"

    from dataclasses import replace as _replace
    x25 = [25.0] * 4
    no_board = _replace(JMC_T, board_limit_c=1e9)
    tests = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", str(ROOT / "tests")],
                           capture_output=True, text=True, cwd=ROOT).stdout.strip().splitlines()[-1]
    n_pass = int(tests.split(" passed")[0].split()[-1])
    n_xfail = int(tests.split(" xfailed")[0].split()[-1]) if "xfailed" in tests else 0

    page = TEMPLATE.format(
        n_pass=n_pass, n_xfail=n_xfail,
        sh_ip="{:.1f}-{:.1f} A".format(min(arm.RULES[j].peak_current_a for j in ("J1", "J2", "C1")),
                                       max(arm.RULES[j].peak_current_a for j in ("J1", "J2", "C1"))),
        wr_ic="{:.2f}-{:.2f} A".format(min(arm.RULES[j].continuous_current_a for j in ("W1", "W2", "W3")),
                                       max(arm.RULES[j].continuous_current_a for j in ("W1", "W2", "W3"))),
        wr_ip="{:.2f}-{:.2f} A".format(min(arm.RULES[j].peak_current_a for j in ("W1", "W2", "W3")),
                                       max(arm.RULES[j].peak_current_a for j in ("W1", "W2", "W3"))),
        jmc_ic=JMC_RULE.continuous_current_a, jmc_ip=JMC_RULE.peak_current_a, jmc_refill=JMC_RULE.refill_at_rest_s,
        gim_ic=GIM_RULE.continuous_current_a, gim_ip=GIM_RULE.peak_current_a, gim_refill=GIM_RULE.refill_at_rest_s,
        gim_at_cap=bu.burst_seconds_at(GIM_RULE, bu.BurstState(), 2.0),
        jmc_ttl_board=th.time_to_limit_s(JMC_T, x25, 28.1)[0],
        jmc_ttl_wind=th.time_to_limit_s(no_board, x25, 28.1)[0],
        jmc_ttl_wind_hot=th.time_to_limit_s(_replace(JMC_T.hot(1.0), board_limit_c=1e9), x25, 28.1)[0],
        gim_ttl_45=th.time_to_limit_s(GIM_T, x25, 4.2)[0],
        gim_ttl_45_hot=th.time_to_limit_s(GIM_T.hot(1.0), x25, 4.2)[0],
        jmc_wlim=JMC_T.winding_limit_c, gim_wlim=GIM_T.winding_limit_c,
        wb_ratio=f"{min(wb):.2f}-{max(wb):.2f}", board_ratio=f"{min(bd):.2f}-{max(bd):.2f}",
        board_err=f"{100 * bd_err:.0f} %",
        blind_cw=ratio("winding_heat_capacity_j_per_k", blind), blind_gwh=ratio("winding_to_housing_w_per_k", blind),
        fit_cw=ratio("winding_heat_capacity_j_per_k", fitted), fit_gwh=ratio("winding_to_housing_w_per_k", fitted),
        sha=sha,
        burst_png=fig_burst(), e1_png=fig_e1(e1, fitted), heldout_png=fig_heldout(
            [("With a resistance check every 60 s (after E1's fit)", with_r),
             ("Board readings only (after E1's fit)", board_only),
             ("Board readings only, datasheet priors — tonight, before any experiment", prior_only)]),
        real_png=fig_real(fit), ttl_png=fig_ttl(),
        param_rows="".join(param_rows), real_rows="".join(real_rows), rep_rows=rep_rows,
        wr_rms=with_r["rms"], wr_max=with_r["max"], wr_cov=100 * with_r["coverage"],
        bo_rms=board_only["rms"], bo_max=board_only["max"], bo_bound=100 * board_only["bound"],
        po_rms=prior_only["rms"], po_max=prior_only["max"], po_bound=100 * prior_only["bound"],
        t_burst=us(tm["burst"][0]), t_burst95=us(tm["burst"][1]), t_obs=us(tm["observer"][0]),
        t_obs95=us(tm["observer"][1]), t_est=f"{1e3 * tm['estimate'][0]:.1f} ms",
        t_est95=f"{1e3 * tm['estimate'][1]:.1f} ms",
        board_rms=fitted.board_rms_c, wind_rms=fitted.winding_rms_c,
        j2_tau=fit["drives"]["J2"]["tau_s"], j2_tau_sd=100 * fit["drives"]["J2"]["tau_rel_sd"],
        c1_tau=fit["drives"]["C1"]["tau_s"], c1_tau_sd=100 * fit["drives"]["C1"]["tau_rel_sd"],
    )
    OUT.write_text(page)
    print(OUT, f"{OUT.stat().st_size / 1024:.0f} KB")


TEMPLATE = (Path(__file__).with_name("report_template.html")).read_text()

if __name__ == "__main__":
    main()
