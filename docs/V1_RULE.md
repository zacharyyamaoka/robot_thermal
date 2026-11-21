# The v1 thermal rule the arm's controller enforces (2026-11-20)

Zach, 2026-11-20: *"for v1. we can just have a super simple model saying like we are only allow to burst for like
5s every 1 minute or someting. And of course we directly measure the board tempreature, and if that gets to hot we
already know to lower that."*

**Per drive, every tick, keep one I²t bucket in A²·s, fed with |q current| (moteus `q_A`, the phase-current
amplitude): when |I| > I_cont add (I² − I_cont²)·dt; when |I| ≤ I_cont subtract min(C / 55 s, I_cont² − I²)·dt,
never below 0; C = (I_peak² − I_cont²) · 5 s. While the bucket is below C and the board is cool the drive may command
up to I_peak; once the bucket reaches C, or the drive's measured board temperature reaches 55 °C (released below
50 °C), clamp that drive to I_cont (torque I_cont·kt) until the bucket is empty again.** Shoulders J1/J2/C1 (JMC
60ASM400 + AD60 10:1): I_cont 14 A (JMC's drawing rates 14.5 A RMS = 20.5 A amplitude, taken ×0.68), I_peak 28.1 A
(the 18 N·m gearbox cap, the boards' `servo.max_current_A`), C 2,968 A²·s: 5 s at 28.1 A, refilled in 55 s at rest,
and a full-strength 5 s burst every 60 s can run forever. Wrists and claw (GIM4305-10): I_cont 1.5 A (rated
1.05 N·m / 0.5087 N·m/A = 2.06 A, ×0.75), I_peak 4.5 A (2.29 N·m), C 90 A²·s: 5 s at 4.5 A, refilled in 55 s; at
today's 2.0 A board cap a wrist may sit at 2.0 A for 51 s before it folds back to 1.5 A. The boards derate themselves
from 60 °C (`fault_temperature` 80 − `temperature_margin` 20, reaching 0 A near 72 °C on the JMC boards), so the
55 °C cut ends bursts before the board throttles them. Tonight's 1.0 run (job `1120-175952-740f`) never touches a
bucket: J2 peaks at 11.6 A, every drive keeps 100 % of its budget. No winding temperature is measured
(`servo.enable_motor_temperature` 0), so these are datasheet numbers with a margin, checked against v2's pessimistic
thermal model, not a measurement.

**Changed the same night (the commit after cbda40e that adds `tests/test_v1_v2.py`):** the first version said I_cont 15 A on the shoulders
and I_peak 7.5 A (the maker's peak torque) on the wrists. v2's hot bound showed a full burst from a long hold at
I_cont in a 30 °C room would take a GIM winding to its 85 °C limit in 1.8 s at 7.5 A, and a JMC to 110 °C in 4.5 s
from a 15 A hold; at 4.5 A and 14 A they get 5.8 s and 12 s (`tests/test_v1_v2.py`).

**On the shoulders the board binds before the winding at 28.1 A.** Tonight's board fits (C1, J2) extrapolate to
2.3-2.9 K/s at 28.1 A, so from J2's ~40 °C hold the 55 °C cut ends a burst after about 6 s, close to the bucket's 5 s.
That is an extrapolation from 8-10 A; the first real burst will show it.

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

## Why this shape

- **It is the industrial servo drive's I²t foldback**, not a new idea: heat stored above the continuous rating fills
  the bucket, rests below it empty it, and a full bucket folds the limit back to continuous.
- **It degrades gracefully:** a burst at I lasts (C − heat)/(I² − I_cont²), so a weaker burst lasts longer and
  anything at or below I_cont never spends budget.
- **The refill is the slower of Zach's minute and the copper's cooling,** so over any run the I²·t delivered is at
  most I_cont²·T plus the heat still in the bucket (`tests/test_burst.py`, a greedy controller for 30 min).
- **On the shoulders 28.1 A is only 2× continuous,** so a 5 s burst cannot cook a JMC winding (v2's prior gives it
  68 s from a 14 A hold); there the rule bounds RMS over minutes, and the board is the nearer limit. The wrists are
  where bursts above continuous are a real thermal question.
- **v1 is checked against v2:** a full burst from the worst state v1 allows before it (a long hold at I_cont, 30 °C
  room) keeps v2's pessimistic winding under its limit (`tests/test_v1_v2.py`).

## What would change it

- A thermistor on one motor or a resistance check (see the project's experiment plan) turns I_cont and the margin
  from a datasheet guess into a measurement; v2's observer then replaces the bucket.
- `servo.max_power_W` (72 W on J1 and J2) binds before 28.1 A at speed; that is the rig's setting, not this rule.
