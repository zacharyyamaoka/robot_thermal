"""ObserverState: the Kalman filter's mean and covariance, plus the hot bound, after the last update."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, kw_only=True)
class ObserverState:
    t_s: float
    mean_c: tuple[float, float, float, float]  # winding, housing, board, ambient
    cov: tuple[tuple[float, ...], ...]  # 4×4, K²
    # the pessimistic model's winding and housing, carried open-loop between direct winding/housing measurements
    hot_c: tuple[float, float]
    current_a: float = 0.0
    updates: int = 0
