# Experiments that turn the winding estimate into a measurement (2026-11-20)

No winding temperature is measured on any BAM drive today (`servo.enable_motor_temperature` 0, no thermistor), and
the boards' own temperature is logged at 1 °C steps. From that alone the winding branch of the thermal model is not
identifiable: the model can carry a datasheet prior forward, but it cannot learn the winding's thermal resistance or
mass. These experiments fix that, cheapest first. robot_thermal never touches a rig: the rig's owner runs them and
hands over the log.

## E1 — winding heating and cooling with resistance checks, 1-DOF rack (about 40 min, no hardware change)

> [!important] Where E1 may run (Zach, 2026-11-20, relayed by the Trajectory Tracking Agent)
> *"The thermal session should just work in Sim, and it can actually work on the motors on the 1 DOF rack, there
> are motors that have the hardstops its can work with ... but it 100% should not be working on the pendulum or
> bam fb that should be on our w2 goals"*
> - **Yes:** the One Dof Test Rack's hardstop stations on fdcanusb 272813EF: **CAN 13**, a JMC (60ASM200 inferred)
>   + AD60 10:1 on a balanced stop plate (the arm shoulder's twin), and **CAN 1**, a GIM4305-10 with a pin
>   hardstop (the wrist's twin). Motor, `servo.max_current_A` and `motor.resistance_ohm` come from the rig's
>   `records/one-dof-test-rack/deployment.json` and the boards' own config, not the robotparts BOM.
> - **Never:** CAN 11 (the pendulum), or any BAM FB board (the arm's fdcanusb 0607623A: J1, J2, C1, wrist, claw).
> - The bus is shared with the pendulum: E1 runs in a gap the W2 goal's pendulum work leaves, booked through the
>   rack's bus lock and gated tools; the gap is the W2 Goal Agent's to give. The rig's owner runs it; this project
>   never opens a device.
> - Sim first: `tests/test_hidden_truth.py` runs E1 on a hidden plant and shows what the fit recovers from it.

**What it measures:** the winding temperature itself, through copper's resistance, R(T) = R₀·(1 + 0.00393·(T − T₀)),
so 1 % of resistance is 2.5 °C. Together with the logged current and board temperature it identifies the winding →
housing → ambient network (both time constants, both thermal resistances) and the board's own heating, which is
the observer's v2 parameter set.

**Why d-axis current:** a positive d-axis current (`make_current(d_A=I, q_A=0)`) puts I²R into the copper with no
torque, so the pendulum stays where it hangs and no gravity pose or hard stop is needed. Positive d (field
strengthening) carries no demagnetisation risk. The rotor must already be at rest.

**The rack's supply has no regen sink** (it dropped out at 04:32Z on 2026-11-21 when a pendulum climb pumped its 24 V
bus to 27-31 V; the rack is down until Zach power-cycles it). E1 regenerates nothing mechanical: every step holds the
rotor still, so there is no −τ·ω to return, and a d current switched off returns only its inductive energy
(½·L·I², about 0.1 J at 20 A on a JMC). Still log the bus voltage and abort above 28 V. `servo.max_regen_power_W` 0
for the run is harmless if the rig's owner wants it (it only acts on regeneration).

**Log, at 10 Hz or faster, for the whole run:** t (s, monotonic), mode, fault, position, velocity, q_A, d_A, board
temperature (°C), bus voltage, and every resistance check's voltages and currents (below).

**Resistance check (about 1.5 s, at rest, zero current before it):** voltage-FOC mode at one fixed electrical angle
(`make_vfoc(theta=θ₀, voltage=V)`), three voltage levels for 0.4 s each, recording the mean current magnitude
√(d_A² + q_A²) over the last 0.2 s of each level. R_phase = slope of V against I (the slope removes the inverter's
dead-time offset). Use the same θ₀ every time: the first check may turn the rotor up to half an electrical period to
align it (on a JMC with 5 pole pairs behind 10:1, at most 3.6° at the output); later checks do not move it. Levels
giving roughly 0.25, 0.5 and 0.75 × the heating current (JMC, R ≈ 0.08 Ω: V ≈ 0.4, 0.8, 1.2 V at a 20 A heat
current; GIM4305, R ≈ 0.63 Ω: V ≈ 0.3, 0.6, 0.9 V at 2 A).

**Sequence:**

| step | what | duration | resistance checks |
|---|---|---|---|
| 0 | idle, zero current (the baseline: ambient, board at rest) | 2 min | at 0 and 2 min (R₀ cold, twice for repeatability) |
| 1 | heat: d-axis current I_heat, constant | 10 min | every 60 s (pause the d current ~1.5 s for each) |
| 2 | cool: zero current | 15 min | every 60 s |
| 3 | bursts: d-axis 5 s at I_burst, 55 s at zero, ×5 | 5 min | after each burst |
| 4 | cool: zero current | 5 min | every 60 s |

**Currents:** I_heat = the motor's continuous current (JMC 60ASM400: 20 A amplitude; a 60ASM200: its rated RMS
current × √2; GIM4305-10: 2.0 A, today's cap), I_burst = the board's `servo.max_current_A`. Do not raise any
board limit for this. **The board heats with I² too:** in simulation a 60ASM400-class plant at 20 A takes its board to
77 °C, past the abort, while 16 A holds it at 57 °C. So start the heat step at 0.8 × the continuous current, and lower
it if the board passes 55 °C in the first 5 min. Log whatever current was used: the fit only needs it logged, not
any particular value. With the link resting on its hardstop the d-axis heating still applies no torque; a q-axis
press into the stop at the same current is an equivalent alternative if d-axis current mode is unavailable.

**Optional step 3b, for the board's own parameters (+12 min):** 3 × (2 min at I_heat, 2 min at zero). The
simulation shows plain E1 narrows the board's heat capacity and board-to-ambient conductance only to ~0.67 of
their prior; a square wave near the board's ~2 min time constant separates it from the housing's ~13 min.

**Abort (stop current, log why):** bus ≥ 28 V (the rack's 24 V supply has no regen sink), board ≥ 60 °C (where moteus starts its own derate), winding by resistance
≥ 90 °C (R ≥ 1.25 × R₀ from 25 °C), any fault, any motion over 2° at the output during a d-axis step.

**Hand-over:** the log as one `.npy` + columns `.json` (like `jtc_ticks.npy`), or CSV, plus which drive and motor ran
and its `servo.max_current_A`, `motor.resistance_ohm`. robot_thermal fits it with `tools/fit_rack.py` and the
observer's winding estimate becomes a measurement for that motor type.

**What it cannot tell:** the five-bar's own mounting (a JMC on the arm sits on a gearbox in an arm, not on the rack's
plate). The rack fit gives the motor's internal winding → housing step; the arm's housing → ambient step still comes
from the arm's own board-and-current logs.

## E2 — a thermistor on one motor (the permanent fix, a small hardware change; Zach's call)

moteus already reads a 10 kΩ NTC (β 4050) on its motor-thermistor pad: set `servo.enable_motor_temperature 1`,
`servo.motor_thermistor_ohm 10000`, and a finite `servo.motor_fault_temperature` (the board then derates from
`motor_fault_temperature − motor_temperature_margin`). Glue one NTC to the end turns of one JMC (J2, the hottest:
it holds the arm against gravity all day). The observer takes it as a direct measurement of that node (`correct`
with the thermistor's σ) and the fitted parameters carry to the other two JMCs. Zach's own notes on this pad:
`Sources/Notion Import/Notes/BAM Robotics/Sept 18 - Motor Thermal Testing Notes.md` (2025-09-18).

## E3 — the arm's own logs, continuously (no rig time)

The arm-hold service keeps J2 and C1 near 40 °C all day. A continuous 1-10 Hz log of q_A and board temperature for
every drive, through a working session, identifies each board's heating per A² and its time constant on the arm as
mounted (tonight's runs are 4-42 s snapshots; J2's board went 55 → 36 °C between them, but with no current logged in
between, so the cooling curve cannot be fitted).
