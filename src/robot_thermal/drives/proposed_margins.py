"""Margins robot_thermal recommends on top of the datasheet rules: Zach's call, never applied by default.

Zach's standing rule (2026-11-20, Settings/Memory/feedback_no_artificial_guards.md): "If you believe one is needed,
state it to Zach with the evidence and the cost in capability, and run at the datasheet until he answers." Each entry
is the rule it would replace, the evidence, and the cost. The evidence is v2's pessimistic model (tests/test_v1_v2.py):
a full burst from a long hold at I_cont in a 30 °C room must keep the winding under its limit. That model's winding
parameters are physics priors, not measurements (no maker publishes them), so E1 on the rack's hardstop stations is
what would turn this evidence into a measurement.
"""

from dataclasses import replace

from robot_thermal.drives import gim4305_10, jmc_60asm400_ad60

PROPOSED = {
    "gim4305-10": dict(
        rule=replace(gim4305_10.rule, continuous_current_a=1.5, peak_current_a=4.2),
        evidence=("Held at the datasheet's 2.06 A in a 30 °C room, the winding settles at 78 °C on the prior model and "
                  "103 °C on the hot bound, against SteadyWin's only stated temperature, 80 °C. Held at 1.5 A it is "
                  "54 / 65 °C, and 4.2 A is then the largest burst current that lasts 5 s on the hot bound. At 7.5 A "
                  "held, the prior model has no steady state (thermal runaway)."),
        cost=("continuous 0.76 N·m instead of 1.05 N·m (−27 %), burst 2.14 N·m instead of 3.82 N·m (−44 %); moot "
              "while the wrist boards are capped at 2.0 A"),
    ),
    "jmc-60asm400+ad60-10": dict(
        rule=replace(jmc_60asm400_ad60.rule, continuous_current_a=17.0),
        evidence=("Held at the rated 20.5 A in a 30 °C room, the winding settles at 130 °C on the prior model and "
                  "204 °C on the hot bound, against IEC 60034-1's 145 °C class-F limit. Held at 17 A it is 92 / "
                  "130 °C, and a full 28.1 A burst then has 9.8 s on the hot bound."),
        cost="continuous 17 A (12.1 N·m at the output) instead of 20.5 A (14.6 N·m); J2's gravity hold is ~10 A",
    ),
}
