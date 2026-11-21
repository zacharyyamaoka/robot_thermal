"""The model's maths against independent oracles: scipy/numpy for the linear algebra, a fine Euler march for
time-to-limit, a long simulation for steady state, and an energy balance."""

from __future__ import annotations

import math

import numpy as np
import pytest
import scipy.linalg

from robot_thermal import _linalg as la
from robot_thermal import thermal as th
from robot_thermal.drives.gim4305_10_thermal import thermal as GIM
from robot_thermal.drives.jmc_60asm400_ad60_thermal import thermal as JMC


@pytest.mark.parametrize("current", [0.0, 15.0, 28.1])
def test_expm_and_solve_match_scipy(current):
    a, b = th.matrices(JMC, current)
    aug = [row[:] + [b[i]] for i, row in enumerate(a)] + [[0.0] * 5]
    for t in (0.25, 7.0, 600.0):
        ours = np.array(la.expm(la.scale(aug, t)))
        ref = scipy.linalg.expm(np.array(aug) * t)
        assert np.allclose(ours, ref, rtol=1e-9, atol=1e-9)
    m = np.random.default_rng(1).normal(size=(9, 9)) + 9 * np.eye(9)
    v = np.arange(9.0)
    assert np.allclose(la.solve(m.tolist(), v.tolist()), np.linalg.solve(m, v))
    assert np.allclose(la.inverse(m.tolist()), np.linalg.inv(m))


@pytest.mark.parametrize("p,current", [(JMC, 10.0), (JMC, 28.1), (GIM, 2.0), (GIM, 4.0)])
def test_steady_state_matches_a_long_simulation(p, current):
    ss = th.steady_state(p, current, 22.0)
    x = th.step(p, [22.0] * 4, current, 40 * p.housing_time_constant_s)
    assert ss is not None
    assert np.allclose(x, ss, atol=0.05)


def test_peak_current_held_on_the_gim_is_thermal_runaway():
    """At 7.5 A the copper's resistance rise adds α·1.5·R₀·I² ≈ 0.21 W/K of heating per kelvin, more than the
    winding-to-ambient path sheds (≈ 0.09 W/K): there is no steady state, so the GIM's peak can only be a burst."""
    i = 7.5
    feedback = GIM.copper_alpha_per_k * 1.5 * GIM.phase_resistance_ohm * i * i
    series = 1 / (1 / GIM.winding_to_housing_w_per_k + 1 / GIM.housing_to_ambient_w_per_k)
    assert feedback > series
    assert th.steady_state(GIM, i, 22.0) is None


def test_energy_balance_at_steady_state():
    p, i = JMC, 20.0
    ss = th.steady_state(p, i, 22.0)
    loss = th.copper_loss_w(p, ss[th.W], i) + p.board_loss_w_per_a2 * i * i + p.board_idle_w
    out = p.housing_to_ambient_w_per_k * (ss[th.H] - 22.0) + p.board_to_ambient_w_per_k * (ss[th.B] - 22.0)
    assert out == pytest.approx(loss, rel=1e-9)


@pytest.mark.parametrize("p,current,start", [(JMC, 28.1, 25.0), (JMC, 40.0, 25.0), (GIM, 7.5, 30.0),
                                             (GIM, 4.0, 30.0), (JMC, 5.0, 25.0)])
def test_time_to_limit_matches_a_fine_euler_march(p, current, start):
    x0 = [start, start, start, start]
    got, which = th.time_to_limit_s(p, x0, current, horizon_s=3600.0)
    x, t, ref, ref_which = list(x0), 0.0, math.inf, ""
    while t < 3600.0:
        x = th.step(p, x, current, 0.05)
        t += 0.05
        if x[th.W] >= p.winding_limit_c or x[th.B] >= p.board_limit_c:
            ref, ref_which = t, "winding" if x[th.W] >= p.winding_limit_c else "board"
            break
    if math.isinf(ref):
        assert math.isinf(got)
    else:
        assert abs(got - ref) <= 0.5 and which == ref_which, (got, ref, which, ref_which)


def test_hot_bound_is_pessimistic():
    hot = JMC.hot(2.0)
    assert hot.winding_heat_capacity_j_per_k < JMC.winding_heat_capacity_j_per_k
    assert th.steady_state(hot, 15.0, 22.0)[th.W] > th.steady_state(JMC, 15.0, 22.0)[th.W]
    t_nom, _ = th.time_to_limit_s(JMC, [25.0] * 4, 28.1)
    t_hot, _ = th.time_to_limit_s(hot, [25.0] * 4, 28.1)
    assert t_hot < t_nom
