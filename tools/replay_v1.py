"""Replay one logged rig run (jtc_ticks.npy) through the v1 burst budget. Read-only on the run folder.

    python tools/replay_v1.py <run_dir>
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

from robot_thermal.burst import BurstState, update, verdict
from robot_thermal.drives.arm import HOLD_CURRENT_A, RULES

CAN = {"J1": 11, "J2": 12, "C1": 13, "W1": 14, "W2": 15, "W3": 16, "CLAW": 17}


def load(run_dir: Path) -> dict[str, np.ndarray]:
    cols = json.loads((run_dir / "jtc_tick_columns.json").read_text())
    ticks = np.load(run_dir / "jtc_ticks.npy")
    return {c: ticks[:, i] for i, c in enumerate(cols)}


def main(run_dir: str) -> None:
    d = load(Path(run_dir))
    t = d["t_s"]
    for joint, can in CAN.items():
        rule = RULES[joint]
        cur, temp = d[f"can{can}_q_current_a"], d[f"can{can}_temperature_c"]
        state, worst_pct, blocked = BurstState(), 100.0, 0
        for k in range(len(t)):
            state = update(rule, state, float(t[k]), float(cur[k]), float(temp[k]))
            v = verdict(rule, state, HOLD_CURRENT_A[joint])
            worst_pct = min(worst_pct, v.budget_left_pct)
            blocked += not v.may_burst
        rms = float(np.sqrt(np.mean(cur**2)))
        print(f"{joint:4s} peak {np.max(np.abs(cur)):6.2f} A  rms {rms:5.2f} A  vs cont {rule.continuous_current_a:5.2f} A"
              f"  budget low-water {worst_pct:6.1f} %  ticks blocked {blocked}  board max {np.max(temp):.0f} C")


if __name__ == "__main__":
    main(sys.argv[1])
