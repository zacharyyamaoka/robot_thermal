"""BurstVerdict: the answer a controller reads each tick — may this drive burst, for how long, and what to clamp to."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

Reason = Literal["ok", "budget_spent", "board_hot"]


@dataclass(frozen=True, kw_only=True)
class BurstVerdict:
    name: str
    may_burst: bool  # True: the drive may go above continuous_current_a now
    reason: Reason  # why not, when it may not
    current_limit_a: float  # A: clamp |current| (or |torque|/kt) to this now: peak when it may burst, else continuous
    burst_s_left: float  # s: how long a burst at peak_current_a may still last from here
    budget_left_pct: float  # % of a full budget left (negative = overdrawn by a controller that ignored the limit)
    next_full_burst_in_s: float  # s: until a full burst is allowed again, if the drive rests at rest_current_a
    board_c: float | None  # °C: the board temperature the verdict was taken at
