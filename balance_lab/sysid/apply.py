"""Apply identified parameters to the MuJoCo plant and the controller.

Two application layers exist, mirroring where each parameter physically
acts (see docs/SYSID.md for the full table):

``apply_model_parameters(model, params)``
    Plant-side: actuator torque strength, joint viscous damping, and
    contact sliding friction.  These mutate the :class:`mujoco.MjModel`
    in place and are idempotent (values are *set*, not accumulated).

``apply_mechanics_fit(model, fit)``
    Optional coupled-mechanics overlay from the PRBS real2sim fitter
    (rl-serial-balance ``scripts/identify_real2sim.py``): per-chain
    damping scale, Coulomb friction, and reflected rotor inertia.
    Applied *after* ``apply_model_parameters`` because it overwrites
    ``dof_damping``; call it only with an accepted real2sim result.

``TraditionalLQR.apply_identified(params)``
    Controller-side: gas-spring force and the theta-compensation fit.

Nothing in this module mutates controller tuning constants or the XML
files: the nominal plant stays bit-identical unless a profile is applied
explicitly, which is what keeps the regression battery meaningful.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco

from .parameters import ModelParameters, load_model_parameters

PROFILES_DIR = Path(__file__).resolve().parent / "profiles"
BUNDLED_PROFILE = "identified_2026-09-04"

# Chain-wide viscous damping the MJCF files ship with (hip chain hinges and
# wheel hinges).  The real2sim PRBS fitter expresses its damping result as a
# *scale* on exactly these nominal values.
NOMINAL_HIP_JOINT_DAMPING = 0.35
NOMINAL_WHEEL_JOINT_DAMPING = 0.08

HIP_ACTUATOR_NAMES = ("l_j1_motor", "l_j2_motor", "r_j1_motor", "r_j2_motor")
WHEEL_ACTUATOR_NAMES = ("l_wheel_motor", "r_wheel_motor")
# The platform scene replaces the flat plane with pyramid layers; every
# driving surface these wheels can touch is listed here.
GROUND_GEOM_NAMES = (
    "ground",
    "ground_outer",
    "platform_top",
    "platform_step1",
    "platform_step2",
    "platform_step3",
)


def _leg_dof_ids(model: mujoco.MjModel) -> list[int]:
    """Dof addresses of every five-bar chain hinge (actuated + closure).

    Same selection rule as the viewer and the real2sim fitter: leg joints
    start with ``l_``/``r_`` and are not wheels.
    """
    dofs = []
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint) or ""
        if name.startswith(("l_", "r_")) and not name.endswith("_wheel"):
            dofs.append(int(model.jnt_dofadr[joint]))
    return dofs


def _chain_dof_ids(model: mujoco.MjModel) -> list[int]:
    """Dofs of the leg hinges carrying the MJCF's nominal viscous damping.

    These are the physical motor/bearing hinges of the closed chain
    (``l_j1``/``l_j2``/links/``l_leg``); the four ``*_loop*`` closure hinges
    per side are pure constraints with zero nominal damping and stay
    untouched by the viscous-damping parameters.
    """
    dofs = []
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint) or ""
        if name.startswith(("l_", "r_")) and not name.endswith("_wheel") and "loop" not in name:
            dofs.append(int(model.jnt_dofadr[joint]))
    return dofs


def _wheel_dof_ids(model: mujoco.MjModel) -> list[int]:
    dofs = []
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint) or ""
        if name.endswith("_wheel"):
            dofs.append(int(model.jnt_dofadr[joint]))
    return dofs


def _actuated_leg_dof_ids(model: mujoco.MjModel) -> set[int]:
    """Dofs of actuated leg joints (rotor inertia reflects onto these only)."""
    leg_dofs = set(_leg_dof_ids(model))
    dofs = set()
    for actuator in range(model.nu):
        joint = int(model.actuator_trnid[actuator, 0])
        dof = int(model.jnt_dofadr[joint])
        if dof in leg_dofs:
            dofs.add(dof)
    return dofs


def _body_collision_geoms(model: mujoco.MjModel, body_name: str) -> list[int]:
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body < 0:
        raise ValueError(f"model has no body named {body_name!r}")
    return [
        geom
        for geom in range(model.ngeom)
        if int(model.geom_bodyid[geom]) == body and int(model.geom_contype[geom]) != 0
    ]


def apply_model_parameters(
    model: mujoco.MjModel,
    params: ModelParameters,
) -> dict[str, float]:
    """Write plant-side identified parameters onto ``model`` (in place).

    Returns a human-readable record of every field written, suitable for
    logs and for tests.  Raises if the model is missing any expected
    actuator/geom, so a renamed MJCF cannot silently skip identification.
    """
    applied: dict[str, float] = {}
    actuators = params.section("actuators")

    for names, key in ((HIP_ACTUATOR_NAMES, "hip"), (WHEEL_ACTUATOR_NAMES, "wheel")):
        strength = float(actuators[key]["strength_scale"])
        if not 0.0 < strength <= 1.5:
            raise ValueError(f"actuators.{key}.strength_scale out of range: {strength}")
        for name in names:
            actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if actuator < 0:
                raise ValueError(f"model has no actuator named {name!r}")
            # A MuJoCo motor applies gainprm[0] * ctrl, so the measured
            # strength ratio becomes the gain.  Torque *limits* stay under
            # controller/ctrlrange authority (the jump window needs 40 Nm).
            model.actuator_gainprm[actuator, 0] = strength
            applied[f"gainprm[{name}]"] = strength
        viscous = float(actuators[key]["viscous_friction_nms_rad"])
        dofs = _chain_dof_ids(model) if key == "hip" else _wheel_dof_ids(model)
        for dof in dofs:
            model.dof_damping[dof] = viscous
        applied[f"damping[{key}_chain]"] = viscous

    contacts = params.section("contacts")
    wheel_sliding = float(contacts["wheel_dynamic_friction"])
    ground_sliding = float(contacts["ground_static_friction"])
    # MuJoCo has one sliding coefficient per geom (pairwise max at contact);
    # a static/dynamic split is not representable, so the dynamic value is
    # the honest choice for the rolling contact.
    for body_name in ("l_wheel", "r_wheel"):
        for geom in _body_collision_geoms(model, body_name):
            model.geom_friction[geom, 0] = wheel_sliding
    applied["friction[wheel]"] = wheel_sliding
    for name in GROUND_GEOM_NAMES:
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom < 0:
            if name == "ground":
                raise ValueError("model has no ground geom")
            continue  # platform layers only exist in the platform scene
        model.geom_friction[geom, 0] = ground_sliding
    applied["friction[ground]"] = ground_sliding
    # contacts.restitution is nominal 0, and MuJoCo encodes restitution via a
    # negative-solref parametrization; applying it is deferred until a
    # non-zero value is actually identified.
    return applied


@dataclass(frozen=True)
class MechanicsFit:
    """Coupled-mechanics result of the PRBS real2sim fitter.

    Field names match ``identify_real2sim.py``; ``source`` records which
    report/run the numbers came from.  Populate this only from an accepted
    fit -- the preliminary rl-serial-balance report is not an accepted
    result yet (docs/SYSID.md).
    """

    hip_damping_scale: float
    hip_frictionloss_nm: float
    hip_armature_kg_m2: float
    wheel_damping_scale: float
    wheel_frictionloss_nm: float
    source: str = "unspecified"


def apply_mechanics_fit(model: mujoco.MjModel, fit: MechanicsFit) -> dict[str, float]:
    """Overlay the coupled-mechanics fit onto chain damping/friction/inertia.

    Damping is written as an absolute ``scale x nominal`` value, so the
    result is the same regardless of whether
    :func:`apply_model_parameters` ran before.  Coulomb friction applies to
    every leg hinge (closure hinges included: their bearings exist on the
    real machine too); reflected rotor inertia applies to the actuated
    hinges only, matching ``identify_real2sim.py``.
    """
    applied: dict[str, float] = {}
    chain_dofs = _chain_dof_ids(model)
    actuated_leg_dofs = _actuated_leg_dof_ids(model)
    for dof in chain_dofs:
        model.dof_damping[dof] = fit.hip_damping_scale * NOMINAL_HIP_JOINT_DAMPING
    for dof in _leg_dof_ids(model):
        model.dof_frictionloss[dof] = fit.hip_frictionloss_nm
    for dof in actuated_leg_dofs:
        model.dof_armature[dof] = fit.hip_armature_kg_m2
    for dof in _wheel_dof_ids(model):
        model.dof_damping[dof] = fit.wheel_damping_scale * NOMINAL_WHEEL_JOINT_DAMPING
        model.dof_frictionloss[dof] = fit.wheel_frictionloss_nm
    applied.update(
        hip_damping=fit.hip_damping_scale * NOMINAL_HIP_JOINT_DAMPING,
        hip_frictionloss_nm=fit.hip_frictionloss_nm,
        hip_armature_kg_m2=fit.hip_armature_kg_m2,
        wheel_damping=fit.wheel_damping_scale * NOMINAL_WHEEL_JOINT_DAMPING,
        wheel_frictionloss_nm=fit.wheel_frictionloss_nm,
        source=fit.source,
    )
    return applied


def load_profile(name: str | Path = BUNDLED_PROFILE) -> ModelParameters:
    """Load a profile from ``sysid/profiles/<name>.yaml`` or an explicit path."""
    path = Path(name)
    if not path.suffix:
        path = PROFILES_DIR / f"{path}.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"unknown sysid profile: {name}")
    return load_model_parameters(path)
