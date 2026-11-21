"""ThermalParams: one drive's lumped thermal network — winding, housing, board — and its losses.

    C_w·dT_w/dt = P_cu − G_wh·(T_w − T_h)
    C_h·dT_h/dt = G_wh·(T_w − T_h) − G_ha·(T_h − T_a) + G_bh·(T_b − T_h)
    C_b·dT_b/dt = P_board − G_ba·(T_b − T_a) − G_bh·(T_b − T_h)
    P_cu    = 1.5·R₀·(1 + α·(T_w − T₀))·I²        copper loss; I the phase-current amplitude (moteus q_A, d_A)
    P_board = k_b·I² + P_idle                     the board's conduction loss and its standing draw

The low-order lumped-parameter thermal network (LPTN) of the PMSM literature (Wallscheid & Böcker): a fast winding
node on a slow housing node, plus the moteus board as its own node that may sit on the housing (G_bh > 0) or apart
(G_bh = 0). The 1.5 is the amplitude-invariant three-phase loss, 1.5·R_phase·(i_d² + i_q²).
"""

from __future__ import annotations

from dataclasses import dataclass, fields, replace

# the names a fit may move, in the order the fit vector holds them; all are positive, so the fit works in logs
POSITIVE = (
    "winding_heat_capacity_j_per_k",
    "housing_heat_capacity_j_per_k",
    "board_heat_capacity_j_per_k",
    "winding_to_housing_w_per_k",
    "housing_to_ambient_w_per_k",
    "board_to_ambient_w_per_k",
    "board_to_housing_w_per_k",
    "board_loss_w_per_a2",
    "board_idle_w",
)


@dataclass(frozen=True, kw_only=True)
class ThermalParams:
    name: str
    winding_heat_capacity_j_per_k: float  # C_w
    housing_heat_capacity_j_per_k: float  # C_h: stator iron, case, and the gearbox it is bolted to
    board_heat_capacity_j_per_k: float  # C_b
    winding_to_housing_w_per_k: float  # G_wh
    housing_to_ambient_w_per_k: float  # G_ha
    board_to_ambient_w_per_k: float  # G_ba
    board_to_housing_w_per_k: float  # G_bh: 0 when the board is not on the motor
    board_loss_w_per_a2: float  # k_b
    board_idle_w: float  # P_idle
    phase_resistance_ohm: float  # R₀ at resistance_ref_c (moteus motor.resistance_ohm)
    resistance_ref_c: float = 25.0  # T₀
    copper_alpha_per_k: float = 0.00393  # α of annealed copper
    winding_limit_c: float = 110.0  # °C the winding must stay under
    board_limit_c: float = 60.0  # °C: where moteus starts its own derate (fault_temperature − temperature_margin)
    # how far each POSITIVE name may be off, as a log-normal sd (0.3 ≈ ±30 %): the fit's prior and the hot bound
    prior_log_sd: tuple[tuple[str, float], ...] = ()
    source: str = ""

    def __post_init__(self) -> None:
        for f in POSITIVE:
            v = getattr(self, f)
            if f == "board_to_housing_w_per_k" or f == "board_idle_w" or f == "board_loss_w_per_a2":
                if v < 0:
                    raise ValueError(f"{self.name}: {f} must be ≥ 0")
            elif not v > 0:
                raise ValueError(f"{self.name}: {f} must be > 0")

    def log_sd(self, name: str) -> float:
        return dict(self.prior_log_sd).get(name, 0.0)

    @property
    def winding_time_constant_s(self) -> float:
        """C_w / G_wh: how fast the winding runs ahead of the housing."""
        return self.winding_heat_capacity_j_per_k / self.winding_to_housing_w_per_k

    @property
    def housing_time_constant_s(self) -> float:
        return self.housing_heat_capacity_j_per_k / self.housing_to_ambient_w_per_k

    @property
    def board_time_constant_s(self) -> float:
        return self.board_heat_capacity_j_per_k / (self.board_to_ambient_w_per_k + self.board_to_housing_w_per_k)

    def hot(self, k_sd: float = 1.0) -> ThermalParams:
        """The pessimistic network: every heat capacity and conductance k_sd prior sds low, the board loss high.

        WHY one-sided: it is the bound a protection decision reads (time to limit, the winding's upper estimate);
        lower capacity heats faster and lower conductance holds the heat in.
        """
        import math

        changes = {}
        for f in POSITIVE:
            sd = self.log_sd(f)
            if not sd:
                continue
            up = f in ("board_loss_w_per_a2", "board_idle_w")
            # WHY G_bh is not lowered: a board on the housing reads it, so less coupling hides heat; the hot bound
            # keeps the prior coupling rather than guessing which way it errs
            if f == "board_to_housing_w_per_k":
                continue
            changes[f] = getattr(self, f) * math.exp((k_sd if up else -k_sd) * sd)
        return replace(self, name=self.name + " (hot bound)", **changes)

    def as_dict(self) -> dict[str, float | str]:
        return {f.name: getattr(self, f.name) for f in fields(self) if f.name != "prior_log_sd"}
