"""Traditional-control line: firmware LQR/VMC controller split by concern."""

from .kinematics import RecoverySwingAxis, SideIds, VmcKinematics, unit
from .controller import TraditionalLQR
from .vmc_gains import (
    IDENTIFIED_LEG_PLANTS,
    IdentifiedLegPlant,
    anchored_length_target,
    derive_per_leg_gains,
)

__all__ = [
    "IDENTIFIED_LEG_PLANTS",
    "IdentifiedLegPlant",
    "RecoverySwingAxis",
    "SideIds",
    "TraditionalLQR",
    "VmcKinematics",
    "anchored_length_target",
    "derive_per_leg_gains",
    "unit",
]
