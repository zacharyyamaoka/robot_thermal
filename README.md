# robot_thermal — thermally limited servoing

Run servo motors above their continuous rating in short bursts, safely, by tracking how hot the windings and the
boards are. Standard-library runtime core; numpy/scipy only as test oracles.

- **v1 — burst budget** (`robot_thermal.burst`): an I²t bucket per drive plus a hard cut on measured board
  temperature. The rule the arm enforces: `docs/V1_RULE.md`.
- **v2 — thermal model + observer** (`robot_thermal.thermal`): a lumped thermal network per drive, losses from
  logged q-axis current, a Kalman filter fusing it with measured board temperature (and resistance, where measured).

```bash
env -u VIRTUAL_ENV uv run --directory /home/bam/robot_thermal --extra test pytest -q
```
