"""Which rule each drive of the five-bar arm (bam_fb_v2) runs, by the joint names the rig logs use."""

from dataclasses import replace

from robot_thermal.drives import gim4305_10, jmc_60asm400_ad60

RULES = {
    "J1": replace(jmc_60asm400_ad60.rule, name="J1"),  # CAN 11
    "J2": replace(jmc_60asm400_ad60.rule, name="J2"),  # CAN 12, holds about 10 A against gravity
    "C1": replace(jmc_60asm400_ad60.rule, name="C1"),  # CAN 13
    "W1": replace(gim4305_10.rule, name="W1"),  # CAN 14
    "W2": replace(gim4305_10.rule, name="W2"),  # CAN 15
    "W3": replace(gim4305_10.rule, name="W3"),  # CAN 16
    "CLAW": replace(gim4305_10.rule, name="CLAW"),  # CAN 17
}

# A: what each drive draws between bursts, for the verdict's refill forecast (HEADROOM_1120.md, dev000 at 1.0).
HOLD_CURRENT_A = {"J1": 0.0, "J2": 10.0, "C1": 5.0, "W1": 0.55, "W2": 0.1, "W3": 0.2, "CLAW": 0.0}
