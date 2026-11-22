"""The v1 burst budget's acceptance test. Every number here is the rule, stated before the code ran."""

from __future__ import annotations

import math
import random
from dataclasses import FrozenInstanceError

import pytest

from robot_thermal.burst import (
    BurstRule,
    BurstState,
    burst_seconds_at,
    seconds_until_burst,
    update,
    update_all,
    verdict,
)
from robot_thermal.drives import arm, gim4305_10, jmc_60asm400_ad60

JMC = jmc_60asm400_ad60.rule
GIM = gim4305_10.rule
DT = 0.001  # s: a 1 kHz stream


def run(rule, state, t0, seconds, current_a, board_c=40.0, dt=DT):
    """Feed a constant current for `seconds`; return (state, time)."""
    n = round(seconds / dt)
    t = t0
    for _ in range(n):
        t += dt
        state = update(rule, state, t, current_a, board_c)
    return state, t


def test_full_strength_burst_lasts_burst_s():
    state = update(JMC, BurstState(), 0.0, 0.0, 40.0)
    state, t = run(JMC, state, 0.0, 4.99, JMC.peak_current_a)
    assert verdict(JMC, state).may_burst
    state, t = run(JMC, state, t, 0.02, JMC.peak_current_a)
    v = verdict(JMC, state)
    assert not v.may_burst and v.reason == "budget_spent"
    assert v.current_limit_a == JMC.continuous_current_a


def test_weaker_burst_lasts_longer_and_continuous_is_free():
    fresh = BurstState()
    assert burst_seconds_at(JMC, fresh, JMC.peak_current_a) == pytest.approx(JMC.burst_s)
    currents = [14.0, 17.0, 20.0, JMC.peak_current_a]
    seconds = [burst_seconds_at(JMC, fresh, i) for i in currents]
    assert all(a > b for a, b in zip(seconds, seconds[1:]))
    assert burst_seconds_at(JMC, fresh, JMC.continuous_current_a) == math.inf
    # anything below continuous costs nothing, for as long as it lasts
    state, _ = run(JMC, update(JMC, BurstState(), 0.0, 0.0), 0.0, 600.0, 0.95 * JMC.continuous_current_a)
    assert state.heat_a2s == 0.0


# a rule whose peak is beyond sqrt(period/burst) = 3.46x continuous: there the copper's cooling, not the minute, binds
STEEP = BurstRule(name="steep", continuous_current_a=1.5, peak_current_a=7.5)


def test_refill_follows_the_rule_below_3_46x_and_the_physics_above():
    # JMC 22.8/12 = 1.9x continuous: Zach's 60 s period binds, 55 s to refill at rest
    assert JMC.refill_at_rest_s == pytest.approx(55.0)
    # GIM 6.61/1.90 = 3.47x and STEEP 5x continuous: the copper's I_cont² cooling binds
    for rule in (GIM, STEEP):
        assert rule.peak_to_continuous > math.sqrt(rule.period_s / rule.burst_s)
        assert rule.refill_at_rest_s == pytest.approx(rule.capacity_a2s / rule.continuous_current_a**2)
    for rule in (JMC, GIM, STEEP):
        state = update(rule, BurstState(), 0.0, 0.0)
        state, t = run(rule, state, 0.0, rule.burst_s, rule.peak_current_a)
        assert seconds_until_burst(rule, state, rule.burst_s, rule.peak_current_a) == pytest.approx(
            rule.refill_at_rest_s, rel=1e-3
        )
        state, t = run(rule, state, t, rule.refill_at_rest_s - 0.5, 0.0)
        assert seconds_until_burst(rule, state, rule.burst_s, rule.peak_current_a) > 0
        state, t = run(rule, state, t, 0.6, 0.0)
        assert seconds_until_burst(rule, state, rule.burst_s, rule.peak_current_a) == 0.0


def test_five_seconds_every_minute_runs_forever_on_the_jmc():
    state, t = update(JMC, BurstState(), 0.0, 0.0), 0.0
    for _ in range(30):  # 30 minutes
        assert verdict(JMC, state).may_burst
        state, t = run(JMC, state, t, 4.95, JMC.peak_current_a, dt=0.01)
        state, t = run(JMC, state, t, 55.05, 0.0, dt=0.01)
    assert state.heat_a2s == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("rule", [JMC, GIM, STEEP], ids=["jmc", "gim", "steep"])
def test_a_greedy_controller_that_obeys_the_limit_stays_at_or_below_continuous_rms(rule):
    """The safety property: ask for the peak whenever it is allowed, clamp to the verdict's limit, for 30 min of
    random demand. The I²·t delivered never exceeds what the continuous rating allows plus the heat still in the
    bucket: ∫I²dt ≤ I_cont²·T + heat(T). (A first draft asserted RMS ≤ 1.01·I_cont over the run; that is false by
    up to capacity/T when the run ends with a spent bucket, which the GIM case showed: 1.525 vs 1.5 A. This exact
    inequality is the property the rule actually guarantees, and it is tighter, not looser.)"""
    rng = random.Random(7)
    state, t, dt = update(rule, BurstState(), 0.0, 0.0), 0.0, 0.01
    sum_i2, n = 0.0, 0
    while t < 1800.0:
        demand = rule.peak_current_a if rng.random() < 0.8 else rng.uniform(0.0, rule.peak_current_a)
        current = min(demand, verdict(rule, state).current_limit_a)
        t += dt
        state = update(rule, state, t, current, 40.0)
        sum_i2 += current * current
        n += 1
    delivered = sum_i2 * dt
    allowed = rule.continuous_current_a**2 * t + state.heat_a2s
    assert delivered <= allowed * (1 + 1e-9), (delivered, allowed)
    # so the RMS over any long run tends to the continuous rating: capacity/T is the most it can add
    rms = math.sqrt(sum_i2 / n)
    assert rms**2 <= rule.continuous_current_a**2 + rule.capacity_a2s / t
    # and it did burst: the budget is used, not hoarded
    assert rms > 0.9 * rule.continuous_current_a


def test_hot_board_stops_bursts_with_hysteresis():
    state = update(JMC, BurstState(), 0.0, 0.0, 59.9)
    assert verdict(JMC, state).may_burst
    state = update(JMC, state, 0.1, 0.0, 60.0)
    v = verdict(JMC, state)
    assert (v.may_burst, v.reason, v.current_limit_a) == (False, "board_hot", JMC.continuous_current_a)
    state = update(JMC, state, 0.2, 0.0, 57.0)  # cooling, still above the release point
    assert not verdict(JMC, state).may_burst
    state = update(JMC, state, 0.3, 0.0, None)  # no reading this tick: keep the latch
    assert not verdict(JMC, state).may_burst
    state = update(JMC, state, 0.4, 0.0, 54.9)
    assert verdict(JMC, state).may_burst


def test_ignoring_the_limit_overdraws_and_the_wait_counts_it():
    state = update(JMC, BurstState(), 0.0, 0.0)
    state, t = run(JMC, state, 0.0, 10.0, JMC.peak_current_a)  # twice the allowed burst
    v = verdict(JMC, state)
    assert v.budget_left_pct == pytest.approx(-100.0, abs=0.5)
    assert v.next_full_burst_in_s == pytest.approx(2 * JMC.refill_at_rest_s, rel=1e-2)


def test_the_hold_current_slows_the_refill_only_where_physics_says_so():
    state = update(STEEP, BurstState(), 0.0, 0.0)
    state, _ = run(STEEP, state, 0.0, STEEP.burst_s, STEEP.peak_current_a)
    at_rest = verdict(STEEP, state, rest_current_a=0.0).next_full_burst_in_s
    holding = verdict(STEEP, state, rest_current_a=0.55).next_full_burst_in_s
    assert holding > at_rest
    assert holding == pytest.approx(STEEP.capacity_a2s / (STEEP.continuous_current_a**2 - 0.55**2), rel=1e-3)
    # below 3.46x the minute binds, and a light hold does not slow it
    state = update(JMC, BurstState(), 0.0, 0.0)
    state, _ = run(JMC, state, 0.0, JMC.burst_s, JMC.peak_current_a)
    assert verdict(JMC, state, rest_current_a=5.0).next_full_burst_in_s == pytest.approx(55.0, rel=1e-3)


def test_time_going_backwards_passes_no_time():
    state = update(JMC, BurstState(), 10.0, 20.0)
    later = update(JMC, state, 9.0, 28.1)
    assert later.t_s == 10.0 and later.heat_a2s == 0.0


def test_states_are_frozen_and_update_is_pure():
    state = BurstState()
    after = update(JMC, state, 0.0, 28.1)
    assert state == BurstState()
    with pytest.raises(FrozenInstanceError):
        after.heat_a2s = 1.0  # type: ignore[misc]


def test_rule_validation():
    with pytest.raises(ValueError):
        BurstRule(name="x", continuous_current_a=10.0, peak_current_a=5.0)
    with pytest.raises(ValueError):
        BurstRule(name="x", continuous_current_a=1.0, peak_current_a=2.0, burst_s=60.0, period_s=60.0)


def test_update_all_tracks_every_arm_drive():
    states = {}
    states = update_all(arm.RULES, states, 0.0, {"J2": 10.0, "W1": 0.5}, {"J2": 42.0})
    states = update_all(arm.RULES, states, 1.0, {"J2": 28.1, "W1": 0.5}, {"J2": 42.0})
    assert set(states) == set(arm.RULES)
    assert states["J2"].heat_a2s == pytest.approx(28.1**2 - 12.0**2)
    assert arm.RULES["J2"].peak_current_a == pytest.approx(18.0 / 0.7396, rel=1e-6)
    assert states["J1"] == BurstState()


def test_flux_braking_heat_counts_as_the_current_that_would_make_it():
    from robot_thermal.burst import bus_regen_w, flux_brake_heat_w

    assert bus_regen_w([10.0, -5.0, 2.0], [-1.0, 2.0, 3.0]) == pytest.approx(10.0 + 10.0)
    # EDITED 2026-11-22 (lane J; the dispatcher's regen decision, step 1b flashes servo.max_regen_power_W = 0 on J1 / C1 / W1): was
    # "J2 in, J1 not in"; J1 / J2 / C1 / W1 now burn their own regen, W2 / W3 / the claw still return it to the bus
    assert arm.FLUX_BRAKING == frozenset({"J1", "J2", "C1", "W1"})
    assert not {"W2", "W3", "CLAW"} & arm.FLUX_BRAKING

    j2 = arm.RULES["J2"]
    assert flux_brake_heat_w(10.0, 1.0) == 0.0  # motoring: nothing comes back
    assert flux_brake_heat_w(10.0, -1.0) == pytest.approx(10.0)  # lowering against gravity: 10 W back
    a = update(j2, update(j2, BurstState(), 0.0, 13.0), 1.0, 13.0)
    b = update(j2, update(j2, BurstState(), 0.0, 13.0), 1.0, 13.0, extra_heat_w=20.0)
    assert b.heat_a2s - a.heat_a2s == pytest.approx(20.0 / (1.5 * j2.phase_resistance_ohm), rel=1e-9)
    with pytest.raises(ValueError):
        update(JMC, BurstState(), 0.0, 1.0, extra_heat_w=5.0)  # the type rule carries no resistance
