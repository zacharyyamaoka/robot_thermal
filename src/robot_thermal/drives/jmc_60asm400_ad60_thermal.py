"""The thermal prior for a JMC 60ASM400 on a Stepperonline AD60 10:1, driven by a moteus r4.11 (J1, J2, C1).

JMC and Stepperonline publish no thermal resistance or time constant, so the winding and housing numbers are
physics estimates with wide priors (log sd 0.5 ≈ ×/÷1.65); the board's are mjbots' own measurement of a bare r4
(10.0 K/W, 12.7 J/K, docs/mpat.html). Experiment E1 (docs/EXPERIMENTS.md) replaces them with a fit.
"""

from robot_thermal.thermal_params import ThermalParams

thermal = ThermalParams(
    name="jmc-60asm400+ad60-10 / moteus r4.11",
    # WHY 80 J/K: about 0.2 kg of copper (385 J/kg·K) in a 1.3 kg, 400 W, 60 mm motor
    winding_heat_capacity_j_per_k=80.0,
    # WHY 800 J/K: the stator iron and aluminium case (~1 kg) plus the gearbox bolted to it (~0.7 kg)
    housing_heat_capacity_j_per_k=800.0,
    board_heat_capacity_j_per_k=12.7,
    # WHY 2.5 W/K: 0.4 K/W winding-to-stator, typical of a 60 mm servo; τ_w = 32 s
    winding_to_housing_w_per_k=2.5,
    # WHY 1.0 W/K: natural convection and radiation from ~0.05 m² of motor and gearbox plus conduction into the arm;
    # with it, the rated 20.5 A puts the winding near 120 °C at 25 °C ambient, as a continuous rating should
    housing_to_ambient_w_per_k=1.0,
    board_to_ambient_w_per_k=0.10,
    # WHY 0.05 W/K and a wide prior: where the boards sit relative to the motors is not recorded here
    board_to_housing_w_per_k=0.05,
    # WHY 0.015 W/A²: J2's board sits ~15 K above J1's while holding ~10 A against gravity; 1.5 W over 10 K/W
    board_loss_w_per_a2=0.015,
    board_idle_w=0.5,
    phase_resistance_ohm=0.080294,  # CAN 12's motor.resistance_ohm, 2026-11-19 backup
    winding_limit_c=110.0,  # class F (155 °C) less margin for the magnets, the encoder and our uncertainty
    board_limit_c=60.0,
    prior_log_sd=(
        ("winding_heat_capacity_j_per_k", 0.5),
        ("housing_heat_capacity_j_per_k", 0.5),
        ("board_heat_capacity_j_per_k", 0.3),
        # WHY 0.35, tighter than the capacities: the maker's rated current pins the total winding-to-ambient
        # resistance (at rated current the winding sits near its class limit); nothing pins the time constants
        ("winding_to_housing_w_per_k", 0.35),
        ("housing_to_ambient_w_per_k", 0.35),
        ("board_to_ambient_w_per_k", 0.3),
        ("board_to_housing_w_per_k", 1.0),
        ("board_loss_w_per_a2", 0.7),
        ("board_idle_w", 0.7),
    ),
    source="physics estimates (JMC publishes none); board: mjbots docs/mpat.html bare r4",
)
