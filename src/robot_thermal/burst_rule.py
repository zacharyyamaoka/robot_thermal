"""BurstRule: how far above its continuous rating one drive may go, and for how long.

The v1 rule (Zach, 2026-11-20): "we are only allow to burst for like 5s every 1 minute or something. And of course
we directly measure the board temperature, and if that gets too hot we already know to lower that."

It is an I²t bucket, the protection industrial servo drives ship (Kollmorgen "foldback", Siemens "I2t motor
model", IEC 60947-4-1 thermal-memory relays): copper loss is I²·R, so I² above the continuous rating is heat the
winding stores, and I² below it is heat the winding can shed. A burst at `peak_current_a` from a full budget lasts
`burst_s`; a weaker burst lasts longer, because it stores heat more slowly; below `continuous_current_a` the drive
never spends budget at all.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, kw_only=True)
class BurstRule:
    """One drive's burst budget. Currents are moteus `q_A` units: the phase-current AMPLITUDE in A.

    Pass the current magnitude, √(i_q² + i_d²) when i_d is logged; the sign does not matter (I² heats either way).
    """

    name: str
    continuous_current_a: float  # A: what the winding may carry forever (the rated current, derated)
    peak_current_a: float  # A: the full-strength burst current (usually the board's servo.max_current_A)
    burst_s: float = 5.0  # s: how long a burst at peak_current_a may last from a full budget
    period_s: float = 60.0  # s: at most one full burst per period; the budget refills no faster than this
    board_burst_off_c: float = 55.0  # °C: measured board temperature at which bursts stop
    board_burst_on_c: float = 50.0  # °C: bursts resume once the board has cooled below this (hysteresis)
    source: str = ""  # where the numbers come from

    def __post_init__(self) -> None:
        if not (self.continuous_current_a > 0 and math.isfinite(self.continuous_current_a)):
            raise ValueError(f"{self.name}: continuous_current_a must be a positive number")
        if not self.peak_current_a > self.continuous_current_a:
            raise ValueError(f"{self.name}: peak_current_a must exceed continuous_current_a")
        if not 0 < self.burst_s < self.period_s:
            raise ValueError(f"{self.name}: need 0 < burst_s < period_s")
        if not self.board_burst_on_c < self.board_burst_off_c:
            raise ValueError(f"{self.name}: board_burst_on_c must be below board_burst_off_c")

    @property
    def capacity_a2s(self) -> float:
        """A²·s: the heat a full budget holds, (I_peak² − I_cont²)·burst_s."""
        return (self.peak_current_a**2 - self.continuous_current_a**2) * self.burst_s

    @property
    def refill_cap_a2_per_s(self) -> float:
        """A²/s: the fastest the budget may refill, so a spent budget takes at least period_s − burst_s at rest."""
        return self.capacity_a2s / (self.period_s - self.burst_s)

    @property
    def peak_to_continuous(self) -> float:
        """How many times the continuous current a full-strength burst is."""
        return self.peak_current_a / self.continuous_current_a

    @property
    def refill_at_rest_s(self) -> float:
        """s: a spent budget refills in this long at zero current.

        WHY the max: the refill rate is min(rule cap, I_cont² − I²). The rule cap is Zach's "every minute"; the
        I_cont² − I² term is the physics (a winding at its limit sheds heat at about I_cont²·R), so the budget never
        refills faster than the copper can actually cool. That second term is what makes the long-run RMS current
        provably ≤ continuous_current_a whatever the duty cycle.
        """
        return max(self.period_s - self.burst_s, self.capacity_a2s / self.continuous_current_a**2)
