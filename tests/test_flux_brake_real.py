"""Flux braking against a real measurement: J2 on belt_u v2 (job 1120-205047-29d5), the first run with d_A logged.

It runs where the rig's run folder exists (this machine) and skips elsewhere; the run data stay out of git.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

from robot_thermal.burst import BurstState, bus_regen_w, flux_brake_heat_w, update  # noqa: E402
from robot_thermal.drives.arm import PHASE_RESISTANCE_OHM, RULES  # noqa: E402

RUN = Path("/home/bam/.local/state/rack/submissions/1120-205047-29d5/runs/real_20261121_035113/run_035127")
CANS = {"J1": 11, "J2": 12, "C1": 13, "W1": 14, "W2": 15, "W3": 16, "CLAW": 17}

pytestmark = pytest.mark.skipif(not (RUN / "jtc_ticks.npy").exists(), reason="the rig's run folder is not here")


@pytest.fixture(scope="module")
def run():
    cols = json.loads((RUN / "jtc_tick_columns.json").read_text())
    a = np.load(RUN / "jtc_ticks.npy")
    return {k: a[:, i] for i, k in enumerate(cols)}


def test_the_bus_bounds_j2s_measured_flux_braking_heat_and_its_own_regen_only_estimates_it(run):
    t = run["t_s"]
    dt = np.diff(t, prepend=t[0])
    measured = float(np.sum(1.5 * PHASE_RESISTANCE_OHM["J2"] * run["can12_d_current_a"] ** 2 * dt))
    own = float(np.sum([flux_brake_heat_w(a, b) for a, b in zip(run["can12_torque_nm"], run["can12_velocity_rad_s"])] * dt))
    cans = [c for c in CANS.values() if f"can{c}_torque_nm" in run]
    bus = float(sum(bus_regen_w([run[f"can{c}_torque_nm"][k] for c in cans],
                                [run[f"can{c}_velocity_rad_s"][k] for c in cans]) * dt[k] for k in range(len(t))))
    assert measured > 1.0  # it really flux-braked
    assert measured <= bus, (measured, bus)
    assert abs(own / measured - 1) < 0.10, (own, measured)  # close, but not a bound: 2.39 J vs 2.45 J


def test_q_plus_the_bus_bound_never_under_counts_the_logged_q_and_d(run):
    rule = RULES["J2"]
    cans = [c for c in CANS.values() if f"can{c}_torque_nm" in run]
    with_d, bounded = BurstState(), BurstState()
    for k in range(len(run["t_s"])):
        t, temp = float(run["t_s"][k]), float(run["can12_temperature_c"][k])
        iq, i_d = float(run["can12_q_current_a"][k]), float(run["can12_d_current_a"][k])
        extra = bus_regen_w([run[f"can{c}_torque_nm"][k] for c in cans], [run[f"can{c}_velocity_rad_s"][k] for c in cans])
        with_d = update(rule, with_d, t, float(np.hypot(iq, i_d)), temp)
        bounded = update(rule, bounded, t, iq, temp, extra_heat_w=extra)
    # both start full; compare the I²·t each one counted (heat clamps at 0, so compare the running sums directly)
    t = run["t_s"]
    dt = np.diff(t, prepend=t[0])
    i2_d = np.sum((run["can12_q_current_a"] ** 2 + run["can12_d_current_a"] ** 2) * dt)
    bus = np.array([bus_regen_w([run[f"can{c}_torque_nm"][k] for c in cans], [run[f"can{c}_velocity_rad_s"][k] for c in cans])
                    for k in range(len(t))])
    i2_b = np.sum((run["can12_q_current_a"] ** 2 + bus / (1.5 * rule.phase_resistance_ohm)) * dt)
    assert i2_b >= i2_d, (i2_b, i2_d)
    assert bounded.heat_a2s >= with_d.heat_a2s
