"""Per-leg VMC length-loop gains derived from the identified leg plants.

Port of the 2026-09-21 derivation in rl-serial-balance
``control/virtual_leg.py`` (``derive_per_leg_gains`` / ``IdentifiedLegPlant``
/ ``IDENTIFIED_PER_LEG_GAINS_2026_09_21``), which retunes the two legs'
length PID against their separately identified plants -- a deliberate
departure from firmware symmetry on the user's 2026-09-21 instruction, the
controller-side counterpart of the 2026-09-20 asymmetric identification
(``leg_friction_damping_2026-09-21_asymmetric.json``).

Method (all numbers reproducible offline; rl-serial-balance
``tests/test_virtual_leg_per_leg_pid_2026_09_21.py``):

1. Each side's identified single-leg plant is reduced to the VMC virtual
   coordinates ``x = [leg length, leg swing]`` with the firmware virtual-work
   Jacobian (``M_v = V^T M V``), about the stand pose, rigid loop closure, no
   ground contact / gravity (small signal).
2. Both channels are modelled ``J x'' + b x' = tau`` under
   ``tau = kp*e - kd*x'`` with ``kp = m*omega^2``,
   ``kd = 2*zeta*m*omega - b``, so both legs land on the SAME closed-loop
   target ``(omega, zeta)``.
3. The length-channel mass is the leg-local effective mass PLUS the chassis
   mass the static support already carries (``support_mass_kg``), so the
   length gains follow the supported mass while the swing gains do not.
4. Coulomb friction (identified per motor) is nonlinear and nearly equal on
   both legs: it is never compensated -- the length loop keeps the firmware's
   bounded anti-windup integral and the swing channel has no integrator.

Anchor: the table is solved against
``tuning.IDENTIFIED_SUPPORT_MASS_KG`` (13.592626 kg, the 2026-09-21
physical ruling), which reproduces rl-serial-balance's LOCKED table
(``IDENTIFIED_PER_LEG_GAINS_2026_09_21``, cloud recipe
``vmc_per_leg_gains: True``) digit-for-digit -- kp 2039.2593 /
1960.7407 N/m, kd 15.9257 / 28.0743 N s/m on the length channel and
12.4129 / 11.4862 Nm/rad, 1.3405 / 1.8888 Nm s/rad on the swing
channel; ``tests/test_sysid.py`` pins the whole table.  Feedforward
at that mass: 0.5 * 13.592626 * 9.81 = 66.6718 N per leg.

Only the length channel feeds :class:`TraditionalLQR`: in this platform the
swinging ``Tp`` is the firmware six-state LQR output plus the theta-sync
channel, not a policy-referenced theta PD, so rl-serial's per-leg
``theta_torque_kp/kd`` has no consumer here (the theta fields are still
derived so tests can pin the cross-repo table).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Historical shared length-loop gains the per-leg split is anchored to.
# 450 N/m was rejected on hardware (see tuning.LENGTH_KP); 2000/22 is the
# only length tuning validated on the real machine and in the 60 s runs, so
# the split preserves its mean stiffness and mean damping ratio exactly and
# only redistributes between the legs.
LENGTH_PID_KP_ANCHOR_N_PER_M = 2000.0
LENGTH_PID_KD_ANCHOR_NS_M = 22.0


@dataclass(frozen=True)
class IdentifiedLegPlant:
    """Identified single-leg plant at the stand pose (virtual coordinates)."""

    side: str
    length_effective_mass_kg: float
    length_effective_damping_ns_m: float
    theta_effective_inertia_kg_m2: float
    theta_effective_damping_nms_rad: float
    theta_coulomb_friction_nm: float
    length_coulomb_friction_n: float
    length_effective_damping_with_passive_ns_m: float


IDENTIFIED_LEG_PLANTS: dict[str, IdentifiedLegPlant] = {
    "l": IdentifiedLegPlant(
        side="l",
        length_effective_mass_kg=2.0272744116687234,
        length_effective_damping_ns_m=15.646148160584485,
        theta_effective_inertia_kg_m2=0.1939510726896869,
        theta_effective_damping_nms_rad=0.8317277282500174,
        theta_coulomb_friction_nm=0.9399123902641874,
        length_coulomb_friction_n=4.076622562054109,
        length_effective_damping_with_passive_ns_m=21.819575356994108,
    ),
    "r": IdentifiedLegPlant(
        side="r",
        length_effective_mass_kg=1.4258533304142134,
        length_effective_damping_ns_m=2.281824897283841,
        theta_effective_inertia_kg_m2=0.17947209395354452,
        theta_effective_damping_nms_rad=0.12129866787702391,
        theta_coulomb_friction_nm=0.9920563010593797,
        length_coulomb_friction_n=4.302783171666369,
        length_effective_damping_with_passive_ns_m=75.20205654241676,
    ),
}
"""2026-09-20 tensioned identification, resolved per side (rl-serial-balance
``leg_friction_damping_2026-09-21_asymmetric.json`` -> per-leg reduction).
Motor-only damping is used for the recommended gains: the passive chain's own
damping is not something this controller commands and the right-leg passive
value (0.302 N m s/rad, 12x the left) is the least constrained equivalent.
The sensitivity is reported by rl-serial (with passive damping included the
length damping ratio would be 0.1057 left / 0.3009 right instead of 0.0884).
"""

# Common closed-loop target both legs are solved onto.
OMEGA_THETA_RAD_S = 8.0  # historical common-mode swing bandwidth (sqrt(12/J))
ZETA_THETA = 0.70        # above the feasibility floor: left identified swing
                         # damping alone is 0.268, a lower zeta would need a
                         # negative (anti-damping) kd on the left leg


def anchored_length_target(
    support_mass_kg: float,
) -> tuple[float, float]:
    """``(omega_length, zeta_length)`` reproducing the historical design point.

    ``omega = sqrt(2000 / mean(support + m_leg))`` and
    ``zeta = (22 + mean(b_leg)) / (2 * mean_m * omega)``, so any supported
    mass keeps the mean kp at the anchor 2000 N/m and the mean closed-loop
    damping ratio unchanged.
    """
    plants = IDENTIFIED_LEG_PLANTS
    masses = [support_mass_kg + plant.length_effective_mass_kg for plant in plants.values()]
    dampings = [plant.length_effective_damping_ns_m for plant in plants.values()]
    m_ref = sum(masses) / len(masses)
    b_ref = sum(dampings) / len(dampings)
    omega = math.sqrt(LENGTH_PID_KP_ANCHOR_N_PER_M / m_ref)
    zeta = (LENGTH_PID_KD_ANCHOR_NS_M + b_ref) / (2.0 * m_ref * omega)
    return omega, zeta


def derive_per_leg_gains(
    support_mass_kg: float,
    length_pid_ki: float = 100.0,
) -> dict[str, dict[str, float]]:
    """Per-leg gains ``kp = m*omega^2``, ``kd = 2*zeta*m*omega - b``.

    Returns ``{side: {"length_kp", "length_ki", "length_kd", "theta_kp",
    "theta_kd"}}``.  Raises ``ValueError`` when the target damping would fall
    below the plant's own identified damping on a channel (that leg would
    need a negative kd = anti-damping).
    """
    omega_length, zeta_length = anchored_length_target(support_mass_kg)
    result: dict[str, dict[str, float]] = {}
    for side in ("l", "r"):
        plant = IDENTIFIED_LEG_PLANTS[side]
        theta_kp = plant.theta_effective_inertia_kg_m2 * OMEGA_THETA_RAD_S ** 2
        theta_kd = (
            2.0 * ZETA_THETA * plant.theta_effective_inertia_kg_m2 * OMEGA_THETA_RAD_S
            - plant.theta_effective_damping_nms_rad
        )
        mass = support_mass_kg + plant.length_effective_mass_kg
        length_kp = mass * omega_length ** 2
        length_kd = (
            2.0 * zeta_length * mass * omega_length
            - plant.length_effective_damping_ns_m
        )
        if theta_kd < 0.0:
            raise ValueError(
                f"target zeta_theta={ZETA_THETA} is below the identified {side} "
                f"swing damping; that leg would need a negative kd"
            )
        if length_kd < 0.0:
            raise ValueError(
                f"target zeta_length={zeta_length:.6f} is below the identified "
                f"{side} length damping; that leg would need a negative kd"
            )
        result[side] = {
            "length_kp": length_kp,
            "length_ki": float(length_pid_ki),
            "length_kd": length_kd,
            "theta_kp": theta_kp,
            "theta_kd": theta_kd,
        }
    return result


__all__ = [
    "IDENTIFIED_LEG_PLANTS",
    "IdentifiedLegPlant",
    "LENGTH_PID_KD_ANCHOR_NS_M",
    "LENGTH_PID_KP_ANCHOR_N_PER_M",
    "OMEGA_THETA_RAD_S",
    "ZETA_THETA",
    "anchored_length_target",
    "derive_per_leg_gains",
]
