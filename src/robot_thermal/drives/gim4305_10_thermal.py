"""The thermal prior for a SteadyWin GIM4305-10 driven by a moteus c1 (W1-W3, CLAW).

SteadyWin publishes no thermal data; the winding and housing are physics estimates (log sd 0.5), the board is
mjbots' measurement of a bare c1 (11.3 K/W, 10.7 J/K, docs/mpat.html).
"""

from robot_thermal.thermal_params import ThermalParams

thermal = ThermalParams(
    name="gim4305-10 / moteus c1",
    winding_heat_capacity_j_per_k=8.0,  # ~20 g of copper in a 140 g actuator
    housing_heat_capacity_j_per_k=60.0,  # stator, case and planetary, ~100 g
    board_heat_capacity_j_per_k=10.7,
    winding_to_housing_w_per_k=0.4,  # τ_w = 20 s
    # WHY 0.12 W/K: with it the rated 2.06 A settles the winding near 77 °C at 25 °C, inside the maker's 80 °C
    housing_to_ambient_w_per_k=0.12,
    board_to_ambient_w_per_k=0.0885,
    board_to_housing_w_per_k=0.02,
    board_loss_w_per_a2=0.02,
    board_idle_w=0.3,
    phase_resistance_ohm=0.630497,  # CAN 14's motor.resistance_ohm, 2026-11-14 backup
    winding_limit_c=85.0,  # the maker's working range ends at 80 °C; a small gimbal motor's magnets are the worry
    board_limit_c=60.0,
    prior_log_sd=(
        ("winding_heat_capacity_j_per_k", 0.5),
        ("housing_heat_capacity_j_per_k", 0.5),
        ("board_heat_capacity_j_per_k", 0.3),
        ("winding_to_housing_w_per_k", 0.5),
        ("housing_to_ambient_w_per_k", 0.5),
        ("board_to_ambient_w_per_k", 0.3),
        ("board_to_housing_w_per_k", 1.0),
        ("board_loss_w_per_a2", 0.7),
        ("board_idle_w", 0.7),
    ),
    source="physics estimates (SteadyWin publishes none); board: mjbots docs/mpat.html bare c1",
)
