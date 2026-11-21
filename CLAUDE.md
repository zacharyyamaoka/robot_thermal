# robot_thermal — working notes for agents

Thermally limited servoing: burst budgets, lumped thermal models and observers for servo drives. Vault hub:
`~/zach_brain/PROJECT - Thermally Limited Servoing.md`.

- **Every cap cites its source beside the constant:** a datasheet (document), a measured limit (run and file), a board
  config, or Zach's dated words (`~/zach_brain/Settings/Memory/feedback_no_artificial_guards.md`). Never add a margin to
  a shipped rule. A margin you believe in goes in `src/robot_thermal/drives/proposed_margins.py` with its evidence
  and cost, for Zach to decide.
- **The runtime core is standard-library only** (`burst`, `thermal`, `observer`, `fit`, `sim`, `drives`), so a rig's
  loop imports it with nothing installed. `tests/test_boundary.py` proves it. numpy and scipy are test oracles only.
- **Hand-authored types are frozen dataclasses, one per file;** drive records are one `rule = ...` / `thermal = ...`
  per file.
- **An estimator is proven against a hidden plant in simulation before it touches hardware**
  (`tests/test_hidden_truth.py`). Bounds are stated before the run; a failing ruler becomes a strict xfail with the
  finding, never a looser bound.
- **This repo never touches a rig.** Rig time and logs go through the Trajectory Tracking Agent. E1 runs only on the
  1-DOF rack's hardstop stations (CAN 13, CAN 1), never the pendulum or BAM FB.
- Machine rules while the arm is powered: pin to `taskset -c 2-5,16,17 nice -n 19`, one worker.

```bash
env -u VIRTUAL_ENV uv run --directory /home/bam/robot_thermal --frozen --extra test pytest -q
```

```bash
env -u VIRTUAL_ENV uv run --directory /home/bam/robot_thermal --frozen --extra test --extra report python docs/build_report.py
```
