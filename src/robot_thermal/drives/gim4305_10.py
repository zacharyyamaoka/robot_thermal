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
    # WHY 7.5 A: the maker's peak torque, 3.82 N·m, over 0.5087 N·m/A. It is 5x continuous, beyond the 3.46x at
    # which "5 s every 60 s" stays thermally sustainable, so the physics term sets this refill: about 2 min at rest.
    peak_current_a=7.5,
    burst_s=5.0,
    period_s=60.0,
    board_burst_off_c=55.0,
    board_burst_on_c=50.0,
    source="SteadyWin rated 1.05 N.m / kt 0.5087 x0.75 margin; peak 3.82 N.m; moteus derate from 60 C",
)
