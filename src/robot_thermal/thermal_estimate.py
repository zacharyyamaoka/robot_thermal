"""ThermalEstimate: what the observer tells a controller and a report about one drive right now."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, kw_only=True)
class ThermalEstimate:
    winding_c: float  # °C, the filter's best estimate
    winding_sd_c: float  # °C, its standard deviation
    winding_hi_c: float  # °C, the bound a protection reads: max(best + 2 sd, the hot model)
    housing_c: float
    housing_sd_c: float
    board_c: float
    board_sd_c: float
    ambient_c: float
    copper_loss_w: float  # W at the present current and estimated winding temperature
    time_to_limit_s: float  # s at the present current until the hot bound reaches a limit (inf: never)
    limit: str  # "winding" | "board" | "" — which limit it reaches first
