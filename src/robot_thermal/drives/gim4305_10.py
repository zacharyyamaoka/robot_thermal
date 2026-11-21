"""SteadyWin GIM4305-10 (10:1 planetary gimbal actuator) on a moteus c1: the wrist W1-W3 and the claw (CAN 14-17).
Numbers: /home/bam/robotparts/user/parts/actuators/servo/steadywin-gim4305-10/ and the boards' config backups of
2026-11-14 (/archive/datasets/bam_rig/rack_2026-11-13/config_backups/can14-20261114T010744Z-before.cfg)."""

from robot_thermal.burst_rule import BurstRule

rule = BurstRule(
    name="gim4305-10",
    # WHY 1.5 A: rated 1.05 N·m at the output over the rig's 0.5087 N·m/A is 2.06 A of phase amplitude, derated
    # ×0.75 like the JMC (no maker thermal data; a 140 g motor). Today's board cap, 2.0 A, is already about the
    # rated current, so the wrists only enter the burst regime if that cap is raised.
    continuous_current_a=1.5,
    # WHY 4.5 A (2.29 N·m), not the maker's 7.5 A peak (3.82 N·m): v2's hot bound (tests/test_v1_v2.py) needs a full
    # 5 s burst from a long hold at I_cont in a 30 °C room to keep this 140 g motor's winding under 85 °C. At 7.5 A
    # it got 1.8 s, at 4.5 A 5.8 s. 3x continuous, so Zach's minute sets the refill.
    peak_current_a=4.5,
    burst_s=5.0,
    period_s=60.0,
    board_burst_off_c=55.0,
    board_burst_on_c=50.0,
    source="SteadyWin rated 1.05 N.m / kt 0.5087 x0.75 margin; peak 4.5 A by v2 hot bound; moteus derate from 60 C",
)
