"""BurstState: where one drive's burst budget stands after the last sample."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, kw_only=True)
class BurstState:
    """Frozen: `burst.update` returns a new state and never edits this one."""

    t_s: float | None = None  # s: the last sample's time; None before the first sample
    heat_a2s: float = 0.0  # A²·s stored above continuous; 0 = full budget, rule.capacity_a2s = spent
    current_a: float = 0.0  # A: the last sample's current magnitude
    board_c: float | None = None  # °C: the last measured board temperature, None if never measured
    board_hot: bool = False  # latched at board_burst_off_c, released below board_burst_on_c
