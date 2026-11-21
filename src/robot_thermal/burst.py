"""The v1 burst budget: pure functions over (BurstRule, BurstState). Standard library only.

    state = BurstState()
    every tick:  state = update(rule, state, t_s, abs(q_current_a), board_temp_c)
                 v = verdict(rule, state, rest_current_a=hold_current_a)
                 clamp |q current| to v.current_limit_a   (|torque| to v.current_limit_a · kt)

The bucket, in A²·s of heat stored above the continuous rating:

    I > I_cont:  heat += (I² − I_cont²)·dt                        a burst fills it; a stronger burst fills it faster
    I ≤ I_cont:  heat −= min(capacity/(period − burst), I_cont² − I²)·dt     rests empty it, never below 0

`capacity = (I_peak² − I_cont²)·burst_s`, so a full-strength burst from an empty bucket lasts `burst_s`. The
refill is the slower of Zach's rule (a spent budget takes `period − burst` to refill) and the physics (a winding
sheds heat at about I_cont²·R at its limit), so over any stretch that starts and ends with an empty bucket the RMS
current is at most I_cont, whatever the duty cycle. When the bucket is full, or the measured board temperature
crosses `board_burst_off_c`, the limit folds back to I_cont until the bucket empties (and the board cools below
`board_burst_on_c`).
"""

from __future__ import annotations

import math
from collections.abc import Mapping

from robot_thermal.burst_rule import BurstRule
from robot_thermal.burst_state import BurstState
from robot_thermal.burst_verdict import BurstVerdict

__all__ = [
    "flux_brake_heat_w",
    "BurstRule",
    "BurstState",
    "BurstVerdict",
    "burst_seconds_at",
    "heat_rate_a2_per_s",
    "seconds_until_burst",
    "update",
    "update_all",
    "verdict",
]


def flux_brake_heat_w(torque_nm: float, velocity_rad_s: float) -> float:
    """W of regenerated power, max(0, −τ·ω): an upper bound on what flux braking burns in the winding.

    WHY: with `servo.max_regen_power_W` 0 (J2's setting), moteus cannot push regenerated energy onto the bus, so it
    burns it as d-axis current in the winding (limiter code 104, mjbots' "limited torque" table). The logs carry q
    current but not d, so a bucket fed |q| alone misses that heat. The regenerated electrical power cannot exceed the
    mechanical power flowing back in, so −τ·ω from the logged torque and velocity bounds it.
    """
    return max(0.0, -float(torque_nm) * float(velocity_rad_s))


def heat_rate_a2_per_s(rule: BurstRule, current_a: float) -> float:
    """A²/s the bucket fills (+) or empties (−) at while the drive carries `current_a`."""
    i2 = current_a * current_a
    ic2 = rule.continuous_current_a**2
    if i2 > ic2:
        return i2 - ic2
    return -min(rule.refill_cap_a2_per_s, ic2 - i2)


def update(rule: BurstRule, state: BurstState, t_s: float, current_a: float, board_c: float | None = None,
           extra_heat_w: float = 0.0) -> BurstState:
    """The state after one sample. `current_a` is the magnitude; `board_c` None keeps the last board reading.

    `extra_heat_w` is winding heat the current does not show (flux braking: `flux_brake_heat_w(τ, ω)`); it counts as
    the current that would make it, I² = W / (1.5·R), so it needs `rule.phase_resistance_ohm`.

    WHY the larger of the two samples' I² over the interval: between two samples the current is unknown, and the
    larger one is the conservative guess for a protection rule (it over-counts heat and under-counts cooling).
    WHY a backwards or repeated time is not an error: this runs inside a control loop, where raising would stop the
    arm; the sample's current and temperature are kept and no time passes.
    """
    current_a = abs(float(current_a))
    if not math.isfinite(current_a):
        raise ValueError(f"{rule.name}: current must be finite, got {current_a}")
    if extra_heat_w > 0.0:
        if not rule.phase_resistance_ohm:
            raise ValueError(f"{rule.name}: extra_heat_w needs rule.phase_resistance_ohm")
        current_a = math.sqrt(current_a * current_a + extra_heat_w / (1.5 * rule.phase_resistance_ohm))
    dt = 0.0 if state.t_s is None else max(0.0, float(t_s) - state.t_s)
    t_new = float(t_s) if state.t_s is None else max(float(t_s), state.t_s)
    worst = max(current_a, state.current_a) if state.t_s is not None else current_a
    heat = max(0.0, state.heat_a2s + heat_rate_a2_per_s(rule, worst) * dt)
    board_hot = state.board_hot
    if board_c is not None and math.isfinite(board_c):
        board_hot = board_c >= rule.board_burst_off_c or (state.board_hot and board_c > rule.board_burst_on_c)
    else:
        board_c = state.board_c
    return BurstState(t_s=t_new, heat_a2s=heat, current_a=current_a, board_c=board_c, board_hot=board_hot)


def burst_seconds_at(rule: BurstRule, state: BurstState, current_a: float) -> float:
    """s a burst at `current_a` may last from this state (inf at or below continuous, 0 when spent)."""
    excess = current_a * current_a - rule.continuous_current_a**2
    if excess <= 0:
        return math.inf
    return max(0.0, rule.capacity_a2s - state.heat_a2s) / excess


def seconds_until_burst(
    rule: BurstRule, state: BurstState, burst_s: float, current_a: float, rest_current_a: float = 0.0
) -> float:
    """s of resting at `rest_current_a` before a `burst_s` burst at `current_a` fits (0 now, inf never).

    The board temperature is not forecast here: a hot board says so in `verdict`.
    """
    excess = current_a * current_a - rule.continuous_current_a**2
    need = max(0.0, excess) * burst_s
    if need > rule.capacity_a2s:
        return math.inf
    surplus = state.heat_a2s - (rule.capacity_a2s - need)
    if surplus <= 0:
        return 0.0
    drain = -heat_rate_a2_per_s(rule, rest_current_a)
    return surplus / drain if drain > 0 else math.inf


def verdict(rule: BurstRule, state: BurstState, rest_current_a: float = 0.0) -> BurstVerdict:
    """What the controller should do now. `rest_current_a` is what the drive draws between bursts (J2's gravity
    hold is about 10 A), used only for `next_full_burst_in_s`."""
    left = rule.capacity_a2s - state.heat_a2s
    if state.board_hot:
        may, reason = False, "board_hot"
    elif left <= 0:
        may, reason = False, "budget_spent"
    else:
        may, reason = True, "ok"
    return BurstVerdict(
        name=rule.name,
        may_burst=may,
        reason=reason,
        current_limit_a=rule.peak_current_a if may else rule.continuous_current_a,
        burst_s_left=burst_seconds_at(rule, state, rule.peak_current_a),
        budget_left_pct=100.0 * left / rule.capacity_a2s,
        next_full_burst_in_s=seconds_until_burst(rule, state, rule.burst_s, rule.peak_current_a, rest_current_a),
        board_c=state.board_c,
    )


def update_all(
    rules: Mapping[str, BurstRule],
    states: Mapping[str, BurstState],
    t_s: float,
    current_a: Mapping[str, float],
    board_c: Mapping[str, float] | None = None,
) -> dict[str, BurstState]:
    """`update` for every drive named in `rules`; a drive missing from `current_a` keeps its state."""
    board_c = board_c or {}
    out = {}
    for name, rule in rules.items():
        state = states.get(name, BurstState())
        out[name] = update(rule, state, t_s, current_a[name], board_c.get(name)) if name in current_a else state
    return out
