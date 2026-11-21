"""JMC 60ASM400 through a Stepperonline AD60 10:1 planetary, on a moteus r4.11: the five-bar shoulder's J1, J2, C1
(CAN 11-13). Every number cites its source (Zach's standing rule, 2026-11-20: no artificial guards; a cap is a
datasheet value, a measured limit, or his words). Margins robot_thermal recommends on top are in
`proposed_margins.py`, for Zach to decide, never applied here.

Sources: JMC's drawing for this model, /home/bam/robotparts/user/parts/actuators/servo/bldc/jmc-60asm400-5-1024c/
(sources/jmc_drawing.py); the boards' config backups of 2026-11-19
(/archive/datasets/bam_rig/rack_2026-11-19/config_backups/can12-20261119T130929Z-before.cfg).
"""

from robot_thermal.burst_rule import BurstRule

rule = BurstRule(
    name="jmc-60asm400+ad60-10",
    # SOURCE: JMC's drawing rates 14.5 A RMS phase current (the seller's list says 16 A); moteus q_A is the
    # phase-current amplitude, so 14.5·√2 = 20.5 A. The maker's own document for this exact model wins over the list.
    continuous_current_a=20.5,
    # SOURCE: the boards' servo.max_current_A, 28.1 A since 2026-11-19, which is 18 N·m at the gearbox output (the
    # AD60's maximum permissible torque, the limit Zach kept on 2026-11-20) at torque_caps.py's 0.64 N·m/A.
    peak_current_a=28.1,
    burst_s=5.0,  # SOURCE: Zach, 2026-11-20, "only allow to burst for like 5s every 1 minute"
    period_s=60.0,  # SOURCE: the same words
    # SOURCE: the board's own config: moteus derates from fault_temperature − temperature_margin = 80 − 20 = 60 °C.
    board_burst_off_c=60.0,
    # not a cap: 5 °C of hysteresis only delays re-arming bursts, so the verdict does not chatter at 60 °C
    board_burst_on_c=55.0,
    source="JMC drawing 14.5 A RMS; board max_current_A 28.1 (= AD60 18 N.m); Zach 5 s/60 s; moteus derate 60 C",
)
