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
    # SOURCE, the smaller of two: the moteus r4.11's own "Continuous phase current: 12A / 32A (w/o and w/ thermal
    # management)" (mjbots shop page, robotparts mjbots-moteus-r4-11), 12 A uncooled, binds before the motor's JMC
    # drawing rating of 14.5 A RMS = 20.5 A amplitude (moteus q_A is the amplitude).
    continuous_current_a=12.0,
    # SOURCE: the AD60's 18 N·m maximum permissible output torque (the limit Zach kept, 2026-11-20) over this drive's
    # kt. kt differs per drive (arm.py sets each); this type default is the lowest current of the three, the Kv 104.607 motor's (C1 until
    # 2026-11-22, J2 since the CAN 12 / 13 swap):
    # 18 / 0.7906 N·m/A (10 × 8.270 / Kv 104.607, fivebar-bench deployment.json) = 22.8 A.
    peak_current_a=22.77,
    burst_s=5.0,  # SOURCE: Zach, 2026-11-20, "only allow to burst for like 5s every 1 minute"
    period_s=60.0,  # SOURCE: the same words
    # SOURCE: the board's own config: moteus derates from fault_temperature − temperature_margin = 80 − 20 = 60 °C.
    board_burst_off_c=60.0,
    # not a cap: 5 °C of hysteresis only delays re-arming bursts, so the verdict does not chatter at 60 °C
    board_burst_on_c=55.0,
    source="r4.11 12 A continuous uncooled (< JMC 20.5 A); AD60 18 N.m / kt per drive; Zach 5 s/60 s; moteus derate 60 C",
)
