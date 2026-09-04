"""System-identification integration: profiles, loading, and application.

Public surface:
- ``load_profile``: load a bundled or explicit identified-parameter YAML.
- ``apply_model_parameters`` / ``apply_mechanics_fit``: plant-side overlay.
- ``MechanicsFit``: coupled-mechanics fit container (real2sim output).
Controller-side application lives on ``TraditionalLQR.apply_identified``.
"""

from .apply import (
    BUNDLED_PROFILE,
    NOMINAL_HIP_JOINT_DAMPING,
    NOMINAL_WHEEL_JOINT_DAMPING,
    MechanicsFit,
    apply_mechanics_fit,
    apply_model_parameters,
    load_profile,
)
from .parameters import ModelParameters, load_model_parameters

__all__ = [
    "BUNDLED_PROFILE",
    "NOMINAL_HIP_JOINT_DAMPING",
    "NOMINAL_WHEEL_JOINT_DAMPING",
    "MechanicsFit",
    "ModelParameters",
    "apply_mechanics_fit",
    "apply_model_parameters",
    "load_model_parameters",
    "load_profile",
]
