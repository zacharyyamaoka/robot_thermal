"""SteadyWin GIM4305-10 (10:1 planetary gimbal actuator) on a moteus c1: the wrist W1-W3 and the claw (CAN 14-17).
Every number cites its source (Zach's standing rule, 2026-11-20). Margins robot_thermal recommends on top are in
`proposed_margins.py`, for Zach to decide, never applied here.

Sources: SteadyWin's product page (https://www.steadywin.cn/en/h-pd-9.html, recorded in /home/bam/robotparts/user/
parts/actuators/servo/steadywin-gim4305-10/); the arm boards' calibrated Kv per drive, as worked out in the wrist
datasheet check (zach_brain reports/media/audits/2026-11-20-wrist-datasheet-check.md). The rig code's 0.5087 N·m/A is
the one-dof rack's CAN 14, a different motor.
"""

from robot_thermal.burst_rule import BurstRule

rule = BurstRule(
    name="gim4305-10",
    # SOURCE: SteadyWin's GDS34 column (the variant is unproven, and GDS34 is the one both columns satisfy: wrist
    # datasheet check, zach_brain reports/media/audits/2026-11-20-wrist-datasheet-check.md): nominal 1.0 N·m over
    # this drive's kt. kt differs per drive (arm.py sets each); this type default is W1's, 0.5252 N·m/A
    # (10 × 8.270 / Kv 157.474), giving 1.90 A.
    continuous_current_a=1.904,
    # SOURCE: GDS34's stall torque 3.47 N·m (SteadyWin gives no duration) over W1's kt = 6.61 A. It is 3.47x
    # continuous, a hair past the 3.46x at which "5 s every 60 s" is sustainable, so the copper's cooling sets the refill.
    peak_current_a=6.607,
    burst_s=5.0,  # SOURCE: Zach, 2026-11-20
    period_s=60.0,  # SOURCE: Zach, 2026-11-20
    board_burst_off_c=60.0,  # SOURCE: moteus derates from fault_temperature 80 − temperature_margin 20 = 60 °C
    board_burst_on_c=55.0,  # not a cap: hysteresis on re-arming
    source="SteadyWin GDS34 nominal 1.0 / stall 3.47 N.m over kt per drive; Zach 5 s/60 s; moteus derate 60 C",
)
