# The v1 thermal rule the arm's controller enforces (2026-11-20)

Zach, 2026-11-20: *"for v1. we can just have a super simple model saying like we are only allow to burst for like
5s every 1 minute or someting. And of course we directly measure the board tempreature, and if that gets to hot we
already know to lower that."*

**Per drive, every tick, keep one I²t bucket in A²·s, fed with |q current| (moteus `q_A`, the phase-current
amplitude): when |I| > I_cont add (I² − I_cont²)·dt; when |I| ≤ I_cont subtract min(C / 55 s, I_cont² − I²)·dt,
never below 0; C = (I_peak² − I_cont²) · 5 s. While the bucket is below C and the board is below 60 °C the drive may
command up to I_peak; once the bucket reaches C, or the drive's measured board temperature reaches 60 °C (re-armed
below 55 °C), clamp that drive to I_cont (torque I_cont·kt) until the bucket is empty again.** Shoulders J1/J2/C1:
I_cont 20.5 A, I_peak 28.1 A, C 1,847 A²·s: 5 s at 28.1 A, refilled in 55 s at rest, repeatable forever. Wrists and
claw: I_cont 2.06 A, I_peak 7.5 A, C 260 A²·s: 5 s at 7.5 A, refilled in 61 s at rest. At 3.6× continuous the
copper's cooling, not the minute, sets that refill. Today's 2.0 A wrist board cap sits below I_cont, so the wrists
cannot spend budget until that cap is raised. Tonight's 1.0 run (job `1120-175952-740f`) never touches a bucket.

## Where every number comes from

Zach's standing rule (2026-11-20, `Settings/Memory/feedback_no_artificial_guards.md`): a cap cites a datasheet, a
measured limit, or his words. Nothing in the shipped rule is an agent-picked margin.

| constant | value | source |
|---|---|---|
| burst length, period | 5 s per 60 s | Zach, 2026-11-20 (quoted above) |
| JMC I_cont | 20.5 A | JMC's drawing for the 60ASM400-5-1024C: rated 14.5 A RMS phase current, × √2 for moteus's amplitude. The seller's list says 16 A RMS; the maker's own drawing wins |
| JMC I_peak | 28.1 A | the boards' `servo.max_current_A` since 2026-11-19 = 18 N·m at the AD60 output (its maximum permissible torque, the limit Zach kept) at 0.64 N·m/A |
| GIM I_cont | 2.06 A | SteadyWin's rated torque 1.05 N·m ÷ the rig's kt 0.5087 N·m/A |
| GIM I_peak | 7.5 A | SteadyWin's peak torque 3.82 N·m ÷ 0.5087 N·m/A |
| board cut | 60 °C | the boards' own config: moteus derates from `fault_temperature` 80 − `temperature_margin` 20 = 60 °C, reaching 0 A near 72 °C on the JMC boards (`derate_current_A` −20) |
| re-arm | 55 °C | not a cap: hysteresis, so the verdict does not chatter at 60 °C |
| refill rate | min(C/55 s, I_cont² − I²) | Zach's minute, and the physics of I²R: a winding at its rating sheds about I_cont²·R |

## What robot_thermal recommends on top: Zach's call, not applied

These live in `src/robot_thermal/drives/proposed_margins.py`. The evidence comes from v2's pessimistic thermal model
(`tests/test_v1_v2.py`): after a long hold at I_cont in a 30 °C room, a full burst should keep the winding under its
limit. The datasheet rules fail that check, and the failure is a strict xfail in the suite. Neither maker publishes
thermal resistance or time constants, so the model's winding parameters are physics priors. This is evidence, not a
measurement; E1 on the rack's hardstop stations is what would settle it.

| drive | proposed | evidence | cost |
|---|---|---|---|
| GIM4305-10 | I_cont 1.5 A, I_peak 4.2 A | Held at 2.06 A in a 30 °C room, the winding settles at 78 °C (prior) / 103 °C (hot bound), against SteadyWin's only stated temperature, 80 °C. At 1.5 A it settles at 54 / 65 °C, and 4.2 A is the largest burst that then lasts 5 s. 7.5 A held is thermal runaway on the prior | continuous −27 % (0.76 vs 1.05 N·m), burst −44 % (2.14 vs 3.82 N·m); moot while the wrist boards are capped at 2.0 A |
| JMC 60ASM400 + AD60 | I_cont 17 A | Held at 20.5 A in a 30 °C room, the winding settles at 130 °C (prior) / 204 °C (hot bound), against IEC 60034-1's 145 °C class-F average-winding limit. At 17 A it settles at 92 / 130 °C, and a 28.1 A burst then has 9.8 s | continuous 12.1 vs 14.6 N·m at the output; J2's gravity hold is ~10 A, so no effect at tonight's loads |

The winding limits themselves:
- **JMC 145 °C** is IEC 60034-1's class-F limit for the average winding temperature (105 K over a 40 °C ambient).
  Class F comes from JMC's drawing of the sibling 60ASM400-5-17BCH-DB15, because this model's drawing states no class.
- **GIM 80 °C** is weak. SteadyWin states no insulation class; its −20 to 80 °C is an ambient working range (the rig's
  wrist datasheet check, 2026-11-20).
- These limits feed v2's estimates and the proposal, never the shipped v1 rule.

**On the shoulders the board binds before the winding at 28.1 A.** Tonight's board fits (C1, J2) extrapolate to
2.3-2.9 K/s at 28.1 A. From J2's ~40 °C hold the board would then reach 55 °C in about 6 s and moteus's 60 °C in about
8 s. That is extrapolated from 8-10 A, so it is a prediction for the first real burst to check.

## In code

```python
from robot_thermal.burst import BurstState, update, verdict
from robot_thermal.drives.arm import HOLD_CURRENT_A, RULES   # J1 J2 C1 W1 W2 W3 CLAW

states = {name: BurstState() for name in RULES}
# every tick, per drive:
states[j] = update(RULES[j], states[j], t_s, abs(q_current_a), board_temperature_c)
v = verdict(RULES[j], states[j], rest_current_a=HOLD_CURRENT_A[j])
limit_nm = v.current_limit_a * kt          # clamp |torque command| to this
# v.may_burst, v.reason ("ok" | "budget_spent" | "board_hot"), v.burst_s_left, v.next_full_burst_in_s
```

`robot_thermal.burst` is standard-library only (`tests/test_boundary.py`), so it can be imported or vendored into
the rig's loop as is. Replay a logged run: `python tools/replay_v1.py <run_dir>`.

## History

- **7ea2406:** the first version, with my own ×0.75 margin on I_cont (15 A shoulders, 1.5 A wrists) and a 55 °C board
  cut.
- **013a5e8:** tightened by v2's check (shoulders 14 A, wrist peak 4.5 A).
- **This version:** back to cited datasheet numbers under Zach's no-artificial-guards rule, after the rig owner asked
  where the winding limits came from. The margins moved to `proposed_margins.py` as a recommendation.

## Why this shape

- **It is the industrial servo drive's I²t foldback**, not a new idea. Heat stored above the continuous rating fills
  the bucket, rests below it empty it, and a full bucket folds the limit back to continuous.
- **It degrades gracefully.** A burst at I lasts (C − heat)/(I² − I_cont²), so a weaker burst lasts longer, and
  anything at or below I_cont never spends budget.
- **The refill is the slower of Zach's minute and the copper's cooling.** So over any run the I²·t delivered is at
  most I_cont²·T plus the heat still in the bucket (`tests/test_burst.py`, a greedy controller for 30 min).
- **An industrial servo's own overload curve allows more.** Delta's ECM-B3 400 W (60 mm class) fits
  t·(L² − 1) ≈ 23 s from cold: 300 % for 2.8 s, 200 % for 8 s (see the prior-art note). This bucket holds
  C / I_cont² = 4.4 s in the same units.

## What would change it

- E1 (a resistance-checked heating run on the rack's hardstop stations) or E2 (a thermistor on one motor) turns the
  winding from a datasheet prior into a measurement; v2's observer can then replace the bucket.
- `servo.max_power_W` (72 W on J1 and J2) binds before 28.1 A at speed. That is the rig's setting, not this rule.
