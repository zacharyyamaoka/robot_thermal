"""FitResult: a parameter fit, with how well each parameter was pinned down by the data."""

from __future__ import annotations

from dataclasses import dataclass

from robot_thermal.thermal_params import ThermalParams


@dataclass(frozen=True, kw_only=True)
class FitResult:
    params: ThermalParams
    ambient_c: float
    # per free name: (prior log sd, posterior log sd); posterior/prior < 0.5 means the data, not the prior, set it
    spread: tuple[tuple[str, float, float], ...]
    board_rms_c: float  # °C, the board residual at the fit
    winding_rms_c: float | None  # °C, the winding-measurement residual (None if none were given)
    iterations: int
    cost: float

    def identified(self, ratio: float = 0.5) -> dict[str, bool]:
        return {name: post < ratio * prior for name, prior, post in self.spread}

    def rel_sd(self, name: str) -> float:
        """The posterior's relative standard deviation of one parameter (≈ log sd)."""
        return next(post for n, _, post in self.spread if n == name)
