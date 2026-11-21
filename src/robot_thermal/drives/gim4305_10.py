"""SteadyWin GIM4305-10 (10:1 planetary gimbal actuator) on a moteus c1: the wrist W1-W3 and the claw (CAN 14-17).
Every number cites its source (Zach's standing rule, 2026-11-20). Margins robot_thermal recommends on top are in
`proposed_margins.py`, for Zach to decide, never applied here.

Sources: SteadyWin's product page as recorded in /home/bam/robotparts/user/parts/actuators/servo/steadywin-gim4305-10/
(rated 1.05 N·m, peak 3.82 N·m); the rig's kt 0.5087 N·m/A (bam_pid_tunning plants/bam_fb_v2.py GIM_KT_NM_PER_A);
the boards' config backup /archive/datasets/bam_rig/rack_2026-11-13/config_backups/can14-20261114T010744Z-before.cfg.
"""

from robot_thermal.burst_rule import BurstRule

rule = BurstRule(
    name="gim4305-10",
    # SOURCE: SteadyWin's rated torque 1.05 N·m over the rig's kt 0.5087 N·m/A = 2.06 A of phase amplitude. Today's
    # board cap, servo.max_current_A 2.0 A, is the rig's setting and binds first.
    continuous_current_a=2.06,
    # SOURCE: SteadyWin's peak torque 3.82 N·m over 0.5087 N·m/A = 7.5 A. 3.6x continuous, past the 3.46x at which
    # "5 s every 60 s" is thermally sustainable, so the copper's I_cont² cooling sets the refill (61 s at rest).
    peak_current_a=7.5,
    burst_s=5.0,  # SOURCE: Zach, 2026-11-20
    period_s=60.0,  # SOURCE: Zach, 2026-11-20
    board_burst_off_c=60.0,  # SOURCE: moteus derates from fault_temperature 80 − temperature_margin 20 = 60 °C
    board_burst_on_c=55.0,  # not a cap: hysteresis on re-arming
    source="SteadyWin rated 1.05 / peak 3.82 N.m over kt 0.5087; Zach 5 s/60 s; moteus derate 60 C",
)
