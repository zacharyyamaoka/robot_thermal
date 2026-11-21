"""ObserverConfig: the observer's noise model — how much it trusts each sensor and its own model."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, kw_only=True)
class ObserverConfig:
    board_sd_c: float = 0.6  # °C: moteus reports whole degrees (0.29 °C quantisation) plus the sensor's own lag
    winding_sd_c: float = 2.5  # °C: a resistance-derived winding temperature, about 1 % of R
    housing_sd_c: float = 1.0  # °C: a thermistor on the housing or end turns
    # process noise, K²/s: what the model leaves out per node (unmodelled paths, air movement, the gearbox)
    winding_noise_k2_per_s: float = 0.002
    housing_noise_k2_per_s: float = 0.0005
    board_noise_k2_per_s: float = 0.002
    ambient_noise_k2_per_s: float = 0.0004  # the room drifts: about 1.2 K in an hour
    # WHY a loss term: a wrong resistance or board-loss coefficient shows up as heating the model cannot explain,
    # in proportion to the heating; (rel_sd · heating rate)² · correlation time is its white-noise equivalent
    loss_rel_sd: float = 0.1
    loss_correlation_s: float = 30.0
    initial_sd_c: tuple[float, float, float, float] = (8.0, 5.0, 1.0, 3.0)  # winding, housing, board, ambient
    hot_k_sd: float = 2.0  # the hot bound's parameters sit this many prior sds on the pessimistic side
