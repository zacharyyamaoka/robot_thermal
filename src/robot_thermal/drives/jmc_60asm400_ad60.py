"""JMC 60ASM400 through a Stepperonline AD60 10:1 planetary, on a moteus r4.11: the five-bar shoulder's J1, J2, C1
(CAN 11-13). Numbers: /home/bam/robotparts/user/parts/actuators/servo/bldc/jmc-60asm400-5-1024c/ and the boards'
config backups of 2026-11-19 (/archive/datasets/bam_rig/rack_2026-11-19/config_backups/can12-*-before.cfg)."""

from robot_thermal.burst_rule import BurstRule

rule = BurstRule(
    name="jmc-60asm400+ad60-10",
    # WHY 15 A: JMC's own drawing rates the motor at 14.5 A RMS phase current (its seller's list says 16 A). moteus
    # reports the phase-current amplitude, so that is 14.5·√2 = 20.5 A. Derated ×0.75 because the rating is for a
    # motor bolted to a test heat-sink plate, ours sits in an arm on a gearbox, and the two JMC sources disagree.
    # J2's gravity hold (about 10 A, all day, boards steady near 40 °C) sits well inside it.
    continuous_current_a=15.0,
    # WHY 28.1 A: the boards' servo.max_current_A since 2026-11-19, which is 18 N·m (the gearbox limit Zach kept) at
    # torque_caps.py's 0.64 N·m/A. It is only 1.9x continuous, so on these drives the bucket mostly bounds RMS.
    peak_current_a=28.1,
    burst_s=5.0,
    period_s=60.0,
    # WHY 55/50 °C: the boards start derating themselves at fault_temperature − temperature_margin = 80 − 20 =
    # 60 °C; stopping bursts 5 °C earlier keeps a burst from ending inside the board's own derate.
    board_burst_off_c=55.0,
    board_burst_on_c=50.0,
    source="JMC drawing 14.5 A RMS rated x0.75 margin; board max_current_A 28.1; moteus derate from 60 C",
)
