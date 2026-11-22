"""Which rule each drive of the five-bar arm (bam_fb_v2) runs, by the joint names the rig logs use.

Every current is a torque over THAT drive's own kt, so the caps are per drive (the rig owner's reconciliation,
2026-11-20). kt at the output = 10 × 8.270 / Kv (moteus's torque factor; Kv the board's own calibration).
"""

from dataclasses import replace

from robot_thermal.drives import gim4305_10, jmc_60asm400_ad60

GEARBOX_MAX_NM = 18.0  # SOURCE: the AD60 10:1's maximum permissible output torque, the limit Zach kept (2026-11-20)
GIM_NOMINAL_NM = 1.0  # SOURCE: SteadyWin GIM4305-10, GDS34 column, nominal torque
GIM_STALL_NM = 3.47  # SOURCE: the same column's stall torque (no duration published)

# N·m/A at the output. SOURCE: J1/J2/C1/CLAW from /home/bam/bam_ws-armhold/src/dev/bam_rack_twin/records/
# fivebar-bench/deployment.json (Kv 113.834 / 104.607 / 111.824 / 160.251); W1-W3 from the wrist datasheet check
# (Kv 157.474 / 163.385 / 160.055).
# WHY J2 0.7906 / C1 0.7396 (2026-11-22, Zach: "can you please swtich CAn 12 and 13 I put them on the wrong way"): the reassembly put
# each five-bar actuator (moteus board bolted on its motor) on the other joint; the ids were swapped back to the joints (plan A), so
# J2 is now the Kv 104.607 motor and C1 the Kv 111.824 one. kt and phase R are the MOTOR's and moved with it.
KT_OUT_NM_PER_A = {"J1": 0.7265, "J2": 0.7906, "C1": 0.7396, "W1": 0.5252, "W2": 0.5062, "W3": 0.5167,
                   "CLAW": 0.5161}


# Ω per phase, for counting flux-braking heat. SOURCE: J1/J2/C1/CLAW the boards' motor.resistance_ohm in the
# fivebar-bench deployment.json; W1 0.6305 and, for W2/W3, the low end (0.625) of the arm wrists' calibrated
# 0.625-0.642 Ω (wrist datasheet check): a lower R counts a watt as more I², the cautious side of a measured range.
PHASE_RESISTANCE_OHM = {"J1": 0.0871016, "J2": 0.086851, "C1": 0.080294, "W1": 0.6305, "W2": 0.625, "W3": 0.625,
                        "CLAW": 0.669335}


def _shoulder(name: str):
    return replace(jmc_60asm400_ad60.rule, name=name, peak_current_a=GEARBOX_MAX_NM / KT_OUT_NM_PER_A[name],
                   phase_resistance_ohm=PHASE_RESISTANCE_OHM[name])


def _wrist(name: str):
    kt = KT_OUT_NM_PER_A[name]
    return replace(gim4305_10.rule, name=name, continuous_current_a=GIM_NOMINAL_NM / kt,
                   peak_current_a=GIM_STALL_NM / kt, phase_resistance_ohm=PHASE_RESISTANCE_OHM[name])


RULES = {
    "J1": _shoulder("J1"),  # CAN 11: 24.8 A peak
    "J2": _shoulder("J2"),  # CAN 12: 22.8 A peak (the Kv 104.607 motor since 2026-11-22); holds about 10 A against gravity
    "C1": _shoulder("C1"),  # CAN 13: 24.3 A peak (the Kv 111.824 motor since 2026-11-22)
    "W1": _wrist("W1"),  # CAN 14: 1.90 / 6.61 A
    "W2": _wrist("W2"),  # CAN 15: 1.98 / 6.86 A
    "W3": _wrist("W3"),  # CAN 16: 1.94 / 6.72 A
    "CLAW": _wrist("CLAW"),  # CAN 17: 1.94 / 6.72 A
}

# Drives that burn regenerated energy in their own winding (flux braking): servo.max_regen_power_W 0. Without logged d current these
# drives take `burst.bus_regen_w` as extra heat. SOURCE (CHANGED 2026-11-22): J2 (CAN 12) has been 0 in flash since before 11-19 (its
# post-power-on dump, rack_2026-11-19 config_backups); J1 / C1 / W1 (CAN 11 / 13 / 14, nan before) are flashed to 0 in the post-service
# re-home's step 1b (arm_motion_bench scripts/bam_fb_real_step1b_flash.py), the dispatcher's decision from the W3 Goal Agent's electrical twin
# (a fault during J1's hardest stop with J2 / C1 not drawing would put the bus at 50-55 V over the boards' 44 V max_voltage). W2 / W3 / the
# claw still return their regen to the bus.
FLUX_BRAKING = frozenset({"J1", "J2", "C1", "W1"})

# A: what each drive draws between bursts, for the verdict's refill forecast (HEADROOM_1120.md, dev000 at 1.0).
HOLD_CURRENT_A = {"J1": 0.0, "J2": 10.0, "C1": 5.0, "W1": 0.55, "W2": 0.1, "W3": 0.2, "CLAW": 0.0}
