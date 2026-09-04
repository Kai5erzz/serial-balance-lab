"""Plant construction: MuJoCo models with an optional identified overlay.

``load_model`` is the single place that turns a model variant plus a plant
profile into a ready-to-step :class:`mujoco.MjModel`.  The nominal plant
(model XML as committed) is what the regression battery validates; any
identified profile must be requested explicitly.
"""

from __future__ import annotations

from pathlib import Path

import mujoco

from ..paths import MODEL_FAST, MODEL_FAST_PLATFORM, MODEL_FULL
from ..sysid.apply import (
    MechanicsFit,
    apply_mechanics_fit,
    apply_model_parameters,
)
from ..sysid.parameters import ModelParameters, load_model_parameters

VARIANTS = {
    "fast": MODEL_FAST,
    "full": MODEL_FULL,
    "platform": MODEL_FAST_PLATFORM,
}

PlantSpec = str | Path | ModelParameters | tuple[ModelParameters, MechanicsFit | None]


def load_model(
    variant: str = "fast",
    *,
    plant: PlantSpec = "nominal",
) -> tuple[mujoco.MjModel, dict[str, float]]:
    """Load a model variant and optionally overlay an identified profile.

    variant: ``"fast"`` (primitive collisions), ``"full"`` (mesh collisions),
        or ``"platform"`` (fast model + 600 mm stair terrain).
    plant: ``"nominal"`` (default, no overlay), a profile name/path, a
        :class:`ModelParameters`, or a ``(ModelParameters, MechanicsFit)``
        tuple for the coupled-mechanics overlay.

    Returns ``(model, applied)`` where ``applied`` records every value
    written; empty for the nominal plant.
    """
    if variant not in VARIANTS:
        raise ValueError(f"unknown model variant {variant!r}; expected {sorted(VARIANTS)}")
    model = mujoco.MjModel.from_xml_path(str(VARIANTS[variant]))

    if isinstance(plant, str) and plant == "nominal":
        return model, {}

    mechanics: MechanicsFit | None = None
    if isinstance(plant, tuple):
        params, mechanics = plant
    elif isinstance(plant, (str, Path)):
        params = load_model_parameters(plant)
    else:
        params = plant
    applied = apply_model_parameters(model, params)
    if mechanics is not None:
        applied.update(apply_mechanics_fit(model, mechanics))
    return model, applied


__all__ = ["PlantSpec", "VARIANTS", "load_model"]
