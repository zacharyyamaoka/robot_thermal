# The v1 thermal rule the arm's controller enforces (2026-11-20)

Zach, 2026-11-20: *"for v1. we can just have a super simple model saying like we are only allow to burst for like
5s every 1 minute or someting. And of course we directly measure the board tempreature, and if that gets to hot we
already know to lower that."*

**Per drive, every tick, keep one I²t bucket in A²·s, fed with |q current| (moteus `q_A`, the phase-current
amplitude): when |I| > I_cont add (I² − I_cont²)·dt; when |I| ≤ I_cont subtract min(C / 55 s, I_cont² − I²)·dt,
never below 0; C = (I_peak² − I_cont²) · 5 s. While the bucket is below C and the board is below 60 °C the drive may
command up to I_peak; once the bucket reaches C, or the drive's measured board temperature reaches 60 °C (re-armed
below 55 °C), clamp that drive to I_cont (torque I_cont·kt) until the bucket is empty again.** Every current is a
torque over that drive's own kt, so the numbers are per drive (`robot_thermal.drives.arm.RULES`):

| drive | I_cont (A) | I_peak (A) | C (A²·s) | refill at rest (s) |
|---|---|---|---|---|
| J1 (CAN 11) | 12.00 | 24.78 | 2,349 | 55 |
| J2 (CAN 12) | 12.00 | 24.34 | 2,242 | 55 |
| C1 (CAN 13) | 12.00 | 22.77 | 1,872 | 55 |
| W1 (CAN 14) | 1.90 | 6.61 | 200 | 55 |
| W2 (CAN 15) | 1.98 | 6.85 | 215 | 55 |
| W3 (CAN 16) | 1.94 | 6.72 | 207 | 55 |
| CLAW (CAN 17) | 1.94 | 6.72 | 207 | 55 |

A full burst lasts 5 s from an empty bucket on every drive, and 5 s every minute can repeat forever. At today's 2.0 A
wrist board cap a wrist spends budget slowly: W1 could hold 2.0 A for about 9 min before folding back to 1.90 A. Lane J
uses min(board cap, rule), so the boards' own `servo.max_current_A` still binds wherever it is lower. Tonight's 1.0 run
(job `1120-175952-740f`) never touches a bucket, but J2's 11.6 A peak sits just under its 12 A I_cont, so any faster
run starts spending J2's budget.

**Flux braking counts as heat.** J2 runs `servo.max_regen_power_W` 0, so moteus burns regenerated energy as d-axis
current in the winding (limiter code 104, on 62 % of J2's replies at 1.0). The logs carry q current but not d, so the
rule adds the regenerated power max(0, −τ·ω), its upper bound, as the current that would make that heat:
I² = W / (1.5·R_phase). At 1.0 it is 5 J against 181 J of q copper loss on J2. It grows with speed, so it counts.
Logging d current (moteus reports it) would replace the bound with the real figure.

## Where every number comes from

Zach's standing rule (2026-11-20, `Settings/Memory/feedback_no_artificial_guards.md`): a cap cites a datasheet, a
measured limit, or his words. Nothing in the shipped rule is an agent-picked margin.

| constant | value | source |
|---|---|---|
| burst length, period | 5 s per 60 s | Zach, 2026-11-20 (quoted above) |
| shoulder I_cont | 12 A | **the smaller of two cited limits.** The moteus r4.11: "Continuous phase current: 12A / 32A (w/o and w/ thermal management)" (mjbots shop page, robotparts `mjbots-moteus-r4-11`), with no cooling fitted. The JMC 60ASM400's drawing: 14.5 A RMS = 20.5 A amplitude |
| shoulder I_peak | 18 N·m ÷ kt | the AD60 10:1's maximum permissible output torque (the limit Zach kept) over each drive's kt. kt = 10 × 8.270 / Kv, with Kv 113.834 / 111.824 / 104.607 for J1 / J2 / C1 (fivebar-bench `deployment.json`). The boards' earlier 28.1 A was 20.4-22.2 N·m by those kt (rig owner's astra check), not 18 |
| wrist I_cont | 1.0 N·m ÷ kt | SteadyWin GIM4305-10, GDS34 column: nominal torque. The fitted variant is unproven; GDS34 is the column both variants satisfy (wrist datasheet check, `zach_brain/reports/media/audits/2026-11-20-wrist-datasheet-check.md`) |
| wrist I_peak | 3.47 N·m ÷ kt | the same column's stall torque. SteadyWin gives no duration |
| wrist kt | 0.5252 / 0.5062 / 0.5167 (W1-W3), 0.5161 (claw) | 10 × 8.270 / the arm boards' own Kv (157.474 / 163.385 / 160.055; claw 160.251). The rig code's 0.5087 is the one-dof rack's CAN 14, a different motor |
| board cut | 60 °C | the boards' own config: moteus derates from `fault_temperature` 80 − `temperature_margin` 20 = 60 °C |
| re-arm | 55 °C | not a cap: hysteresis, so the verdict does not chatter at 60 °C |
| refill rate | min(C/55 s, I_cont² − I²) | Zach's minute, and the physics of I²R: a winding at its rating sheds about I_cont²·R |

## What robot_thermal recommends on top: Zach's call, not applied

This is in `src/robot_thermal/drives/proposed_margins.py`. The evidence comes from v2's pessimistic thermal model
(`tests/test_v1_v2.py`): after a long hold at I_cont in a 30 °C room, a full burst should keep the winding under its
limit. The model's winding parameters are physics priors, because neither maker publishes thermal resistance or time
constants. So this is evidence, not a measurement; E1 on the rack's hardstop stations is what would settle it.

- **Shoulders: no proposal.** With the r4.11's 12 A binding, the cited rule passes the check.
- **Wrists:** the cited rule fails it (a strict xfail in the suite). The proposal is I_cont 1.5 A and I_peak 4.2 A.
  - Evidence: held at W1's 1.90 A in a 30 °C room, the hot-bound winding settles above SteadyWin's only stated
    temperature, 80 °C. Held at 1.5 A it is 54 / 65 °C (prior / hot), and 4.2 A is then the largest burst that lasts
    5 s. Held at the stall current, the prior model has no steady state (thermal runaway).
  - Cost on W1: −21 % continuous torque, −36 % burst torque. Moot while the wrist boards cap at 2.0 A.
  - Recommendation: wait for E1.

The winding limits behind it:
- **GIM 80 °C** is weak. SteadyWin states no insulation class; its −20 to 80 °C is an ambient working range.
- **JMC 145 °C** is IEC 60034-1's class-F limit for the average winding temperature (105 K over a 40 °C ambient). The
  class comes from JMC's drawing of the sibling 60ASM400-5-17BCH-DB15, because this model's drawing leaves it blank.
- These limits feed v2 and the proposal, never the shipped v1 rule.

**On the shoulders the board binds before the winding.** Tonight's board fits (C1, J2) extrapolate to 2.3-2.9 K/s at
28.1 A, or about 1.9-2.2 K/s at the new 22.8-24.8 A peaks. From J2's ~40 °C hold that reaches 60 °C in about
9-10 s, after the 5 s bucket. This is extrapolated from 8-10 A, so it is a prediction to check.

## In code

```python
from robot_thermal.burst import BurstState, flux_brake_heat_w, update, verdict
from robot_thermal.drives.arm import HOLD_CURRENT_A, RULES   # J1 J2 C1 W1 W2 W3 CLAW

states = {name: BurstState() for name in RULES}
# every tick, per drive:
states[j] = update(RULES[j], states[j], t_s, abs(q_current_a), board_temperature_c,
                   extra_heat_w=flux_brake_heat_w(torque_nm, velocity_rad_s))   # J2: max_regen_power_W 0
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
- **0299d62:** back to cited datasheet numbers under Zach's no-artificial-guards rule, after the rig owner asked where
  the winding limits came from. The margins moved to `proposed_margins.py` as a recommendation.
- **This version:** per drive, reconciled with the rig owner's cited checks. The shoulder peak is 18 N·m over each
  board's own kt, not 28.1 A. The shoulder continuous current is the r4.11's own 12 A, not the motor's 20.5 A. The
  wrists use SteadyWin's GDS34 column over the arm's own kt (the rack's 0.5087 was a different motor).

## Why this shape

- **It is the industrial servo drive's I²t foldback**, not a new idea. Heat stored above the continuous rating fills
  the bucket, rests below it empty it, and a full bucket folds the limit back to continuous.
- **It degrades gracefully.** A burst at I lasts (C − heat)/(I² − I_cont²), so a weaker burst lasts longer, and
  anything at or below I_cont never spends budget.
- **The refill is the slower of Zach's minute and the copper's cooling.** So over any run the I²·t delivered is at
  most I_cont²·T plus the heat still in the bucket (`tests/test_burst.py`, a greedy controller for 30 min).
- **An industrial servo's own overload curve allows more.** Delta's ECM-B3 400 W (60 mm class) fits
  t·(L² − 1) ≈ 23 s from cold: 300 % for 2.8 s, 200 % for 8 s (see the prior-art note). This bucket holds
  C / I_cont² = 13-16 s in the same units on the shoulders.

## What would change it

- E1 (a resistance-checked heating run on the rack's hardstop stations) or E2 (a thermistor on one motor) turns the
  winding from a datasheet prior into a measurement; v2's observer can then replace the bucket.
- `servo.max_power_W` (72 W on J1 and J2) binds before 28.1 A at speed. That is the rig's setting, not this rule.
