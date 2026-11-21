"""Current profiles and sensor models for simulating a drive: the E1 rack protocol, an arm duty cycle, and moteus's
whole-degree board reading. Used by the hidden-ground-truth test and the report. Standard library only."""

from __future__ import annotations

import random
from collections.abc import Callable


def e1_current(heat_a: float, burst_a: float) -> tuple[Callable[[float], float], float, list[float]]:
    """docs/EXPERIMENTS.md E1: (current at t, duration, resistance-check times).

    2 min idle, 10 min at heat_a, 15 min cooling, 5 × (5 s at burst_a, 55 s off), 5 min cooling.
    """
    idle, heat, cool, bursts, tail = 120.0, 600.0, 900.0, 300.0, 300.0
    t_heat, t_cool, t_burst, t_tail = idle, idle + heat, idle + heat + cool, idle + heat + cool + bursts
    total = t_tail + tail

    def current(t: float) -> float:
        if t_heat <= t < t_cool:
            return heat_a
        if t_burst <= t < t_tail and (t - t_burst) % 60.0 < 5.0:
            return burst_a
        return 0.0

    checks = [0.0, 120.0] + [t_heat + 60.0 * k for k in range(1, 11)] + [t_cool + 60.0 * k for k in range(1, 16)]
    checks += [t_burst + 60.0 * k + 6.0 for k in range(5)] + [t_tail + 60.0 * k for k in range(1, 6)]
    return current, total, sorted(c for c in checks if c <= total)


def arm_duty(hold_a: float, burst_a: float, move_a: float, seed: int = 3) -> Callable[[float], float]:
    """An arm-like load: a gravity hold, moves at move_a, and a burst at burst_a for 3 s every 40 s (the first at
    t = 20 s, so a run can start from rest)."""
    rng = random.Random(seed)
    moves = [(rng.uniform(0, 1), rng.uniform(0.4, 1.0)) for _ in range(4000)]

    def current(t: float) -> float:
        if 20.0 <= t % 40.0 < 23.0:
            return burst_a
        k = int(t // 2.0)
        phase, level = moves[k % len(moves)]
        return move_a * level if (t % 2.0) / 2.0 < phase * 0.6 else hold_a

    return current


def board_reading(true_c: float, rng: random.Random, noise_sd_c: float = 0.3) -> float:
    """What a moteus reports: whole degrees, after a little sensor noise."""
    return float(round(true_c + rng.gauss(0.0, noise_sd_c)))
