"""v1 against v2's own hot bound: a full burst, from the worst state v1 allows before it, should keep the
pessimistic model's winding under its limit.

The worst pre-burst state v1 allows is a long hold at exactly I_cont (the bucket neither fills nor drains there); the
room is taken at 30 °C. The board is left out: it is measured, and v1 cuts bursts at moteus's own 60 °C on the
reading itself.

The shipped rules are the datasheet's (Zach's standing rule: no agent-picked caps). They fail this check, and that
failure is kept as a strict xfail: it is the evidence for `drives/proposed_margins.py`, which passes it and waits on
Zach. The model's winding parameters are physics priors, not measurements, so E1 is what would settle it.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from robot_thermal import thermal as th
from robot_thermal.drives import gim4305_10, gim4305_10_thermal, jmc_60asm400_ad60, jmc_60asm400_ad60_thermal
from robot_thermal.drives.proposed_margins import PROPOSED
from robot_thermal.observer import DEFAULT

ROOM_C = 30.0
THERMAL = {"jmc-60asm400+ad60-10": jmc_60asm400_ad60_thermal.thermal, "gim4305-10": gim4305_10_thermal.thermal}


def passes(rule, params) -> tuple[bool, str]:
    hot = replace(params.hot(DEFAULT.hot_k_sd), board_limit_c=1e9)
    x0 = th.steady_state(hot, rule.continuous_current_a, ROOM_C)
    if x0 is None or x0[th.W] >= hot.winding_limit_c:
        return False, f"hold at {rule.continuous_current_a} A settles at {x0 and round(x0[th.W])} °C"
    ttl, _ = th.time_to_limit_s(hot, x0, rule.peak_current_a)
    return ttl >= rule.burst_s, f"burst at {rule.peak_current_a} A lasts {ttl:.1f} s"


@pytest.mark.parametrize("name", sorted(PROPOSED))
def test_the_proposed_margins_pass(name):
    ok, why = passes(PROPOSED[name]["rule"], THERMAL[name])
    assert ok, why


@pytest.mark.xfail(strict=True, reason="FINDING, Zach's call: the datasheet rules fail v2's hot bound (see "
                                       "drives/proposed_margins.py for the evidence and the cost)")
@pytest.mark.parametrize("rule", [jmc_60asm400_ad60.rule, gim4305_10.rule], ids=["jmc", "gim"])
def test_the_datasheet_rules_pass(rule):
    ok, why = passes(rule, THERMAL[rule.name])
    assert ok, why
