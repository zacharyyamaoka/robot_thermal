"""v1 must be safe by v2's own hot bound: a full burst, from the worst state v1 allows before it, keeps the
pessimistic model's winding under its limit.

The worst pre-burst state v1 allows is a long hold at exactly I_cont (the bucket neither fills nor drains there); the
room is taken at 30 °C. The board is left out: it is measured, and v1 cuts bursts at 55 °C on the reading itself.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from robot_thermal import thermal as th
from robot_thermal.drives import gim4305_10, gim4305_10_thermal, jmc_60asm400_ad60, jmc_60asm400_ad60_thermal
from robot_thermal.observer import DEFAULT

ROOM_C = 30.0
PAIRS = [(jmc_60asm400_ad60.rule, jmc_60asm400_ad60_thermal.thermal),
         (gim4305_10.rule, gim4305_10_thermal.thermal)]


@pytest.mark.parametrize("rule,params", PAIRS, ids=["jmc", "gim"])
def test_a_full_burst_from_a_hold_at_continuous_keeps_the_hot_winding_under_its_limit(rule, params):
    hot = replace(params.hot(DEFAULT.hot_k_sd), board_limit_c=1e9)
    x0 = th.steady_state(hot, rule.continuous_current_a, ROOM_C)
    assert x0 is not None and x0[th.W] < hot.winding_limit_c, x0
    ttl, which = th.time_to_limit_s(hot, x0, rule.peak_current_a)
    assert ttl >= rule.burst_s, (rule.name, ttl, which)
