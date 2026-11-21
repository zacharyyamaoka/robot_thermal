"""Margins robot_thermal recommends on top of the datasheet rules: Zach's call, never applied by default.

Zach's standing rule (2026-11-20, Settings/Memory/feedback_no_artificial_guards.md): "If you believe one is needed,
state it to Zach with the evidence and the cost in capability, and run at the datasheet until he answers." Each entry
is the rule it would replace, the evidence, and the cost. (A JMC entry, I_cont 17 A, was dropped on 2026-11-20 once
the r4.11's own 12 A continuous rating was cited: the datasheet rule now sits below it and passes the check.) The evidence is v2's pessimistic model (tests/test_v1_v2.py):
a full burst from a long hold at I_cont in a 30 °C room must keep the winding under its limit. That model's winding
parameters are physics priors, not measurements (no maker publishes them), so E1 on the rack's hardstop stations is
what would turn this evidence into a measurement.
"""

from dataclasses import replace

from robot_thermal.drives import gim4305_10

PROPOSED = {
    "gim4305-10": dict(
        rule=replace(gim4305_10.rule, continuous_current_a=1.5, peak_current_a=4.2),
        evidence=("Held at the datasheet's 1.9 A (W1) in a 30 °C room, the winding settles above SteadyWin's only "
                  "stated temperature, 80 °C, on the hot bound. Held at 1.5 A it is 54 / 65 °C (prior / hot), and "
                  "4.2 A is then the largest burst current that lasts 5 s on the hot bound. At the stall current, "
                  "held, the prior model has no steady state (thermal runaway)."),
        cost=("on W1, continuous 0.79 N·m instead of 1.0 N·m (−21 %), burst 2.21 N·m instead of 3.47 N·m (−36 %); "
              "moot while the wrist boards are capped at 2.0 A"),
    ),
}
