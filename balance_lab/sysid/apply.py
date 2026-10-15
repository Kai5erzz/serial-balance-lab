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

import math
from dataclasses import dataclass
from pathlib import Path
from weakref import WeakKeyDictionary

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

# Baseline mass/inertia snapshot per model, taken before the first ``legs``
# overlay is written.  Applying the multiplicative leg scale from the
# pristine baseline (instead of the current values) is what makes the
# overlay idempotent, matching the set-semantics rule of every other field.
_MASS_BASELINE: WeakKeyDictionary = WeakKeyDictionary()

LEGS_SECTION_KEYS = {"source", "mass_inertia_scale", "motors", "passive_joints"}
MOTOR_LEG_JOINTS = ("l_j1", "l_j2", "r_j1", "r_j2")
MOTOR_FIELDS = (
    "strength_scale",
    "viscous_friction_nms_rad",
    "coulomb_friction_nm",
    "armature_kg_m2",
)


def _finite(section: str, key: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"model parameter {section}.{key} must be a number, got {value!r}")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"model parameter {section}.{key} must be finite, got {value!r}")
    return number


def _apply_legs_section(model: mujoco.MjModel, legs: dict) -> dict[str, float]:
    """Apply the asymmetric per-side leg overlay (optional ``legs`` section).

    Three physically distinct pieces, all written with set semantics from the
    model's pristine baseline (idempotent; see ``_MASS_BASELINE``):

    ``mass_inertia_scale`` {l, r}
        Multiplies ``body_mass`` / ``body_inertia`` of every body whose name
        starts with ``l_`` / ``r_`` -- the same body set (wheels included) the
        source profile defines as the leg subtree.  Solver inverse-weight
        hints stay at their compile-time values (refreshing them needs a
        recompile); their role is constraint warm-start weighting only.
    ``motors`` {l_j1, l_j2, r_j1, r_j2}
        Per-motor gain, viscous damping, Coulomb friction and reflected
        rotor armature -- overrides the scalar ``actuators.hip`` values for
        these four hinges.
    ``passive_joints`` {l, r}
        Viscous/Coulomb values for the remaining passive leg hinges of that
        side (links, ``*_leg`` knee, closure hinges); motor hinges excluded.
    """
    if not isinstance(legs, dict):
        raise ValueError(f"legs section must be a mapping, got {legs!r}")
    unknown = set(legs) - LEGS_SECTION_KEYS
    if unknown:
        raise ValueError(f"legs section has unknown keys: {sorted(unknown)}; "
                         f"expected {sorted(LEGS_SECTION_KEYS)}")
    applied: dict[str, float] = {}
    if "source" in legs:
        if not isinstance(legs["source"], str) or not legs["source"].strip():
            raise ValueError("legs.source must be a non-empty string")
        applied["legs_source"] = 1.0  # presence marker (string kept in the profile)

    scale = legs.get("mass_inertia_scale")
    if scale is not None:
        if not isinstance(scale, dict) or set(scale) != {"l", "r"}:
            raise ValueError("legs.mass_inertia_scale must map exactly {'l', 'r'}")
        factors = {side: _finite("legs.mass_inertia_scale", side, scale[side])
                   for side in ("l", "r")}
        for side, factor in factors.items():
            if not 0.5 <= factor <= 3.0:
                raise ValueError(f"legs.mass_inertia_scale.{side} out of range: {factor}")
        baseline = _MASS_BASELINE.get(model)
        if baseline is None:
            baseline = (model.body_mass.copy(), model.body_inertia.copy())
            _MASS_BASELINE[model] = baseline
        base_mass, base_inertia = baseline
        for side, factor in factors.items():
            body_ids = [
                body for body in range(model.nbody)
                if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or "")
                .startswith(f"{side}_")
            ]
            if not body_ids:
                raise ValueError(f"model has no bodies named {side}_*")
            for body in body_ids:
                model.body_mass[body] = base_mass[body] * factor
                model.body_inertia[body] = base_inertia[body] * factor
            applied[f"mass_inertia_scale[{side}]"] = factor
        applied["plant_total_mass_kg"] = float(model.body_mass.sum())

    motors = legs.get("motors")
    if motors is not None:
        if not isinstance(motors, dict):
            raise ValueError("legs.motors must be a mapping of joint -> fields")
        unknown = set(motors) - set(MOTOR_LEG_JOINTS)
        if unknown:
            raise ValueError(f"legs.motors names unknown joints: {sorted(unknown)}")
        for joint_name, fields in motors.items():
            if not isinstance(fields, dict) or not fields:
                raise ValueError(f"legs.motors.{joint_name} must be a non-empty mapping")
            unknown = set(fields) - set(MOTOR_FIELDS)
            if unknown:
                raise ValueError(
                    f"legs.motors.{joint_name} has unknown fields {sorted(unknown)}; "
                    f"expected {sorted(MOTOR_FIELDS)}"
                )
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint < 0:
                raise ValueError(f"model has no joint named {joint_name!r}")
            dof = int(model.jnt_dofadr[joint])
            actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                         f"{joint_name}_motor")
            if actuator < 0:
                raise ValueError(f"model has no actuator named {joint_name}_motor")
            for field, value in fields.items():
                number = _finite(f"legs.motors.{joint_name}", field, value)
                if field == "strength_scale":
                    if not 0.0 < number <= 1.5:
                        raise ValueError(
                            f"legs.motors.{joint_name}.strength_scale out of range: {number}"
                        )
                    model.actuator_gainprm[actuator, 0] = number
                elif field == "viscous_friction_nms_rad":
                    if number < 0.0:
                        raise ValueError(f"{joint_name}.viscous_friction_nms_rad < 0")
                    model.dof_damping[dof] = number
                elif field == "coulomb_friction_nm":
                    if number < 0.0:
                        raise ValueError(f"{joint_name}.coulomb_friction_nm < 0")
                    model.dof_frictionloss[dof] = number
                else:  # armature_kg_m2
                    if not 0.0 <= number <= 0.08:
                        raise ValueError(
                            f"{joint_name}.armature_kg_m2 outside the validated "
                            f"fixture bound [0, 0.08]: {number}"
                        )
                    model.dof_armature[dof] = number
                applied[f"{field}[{joint_name}]"] = number

    passive = legs.get("passive_joints")
    if passive is not None:
        if not isinstance(passive, dict) or set(passive) != {"l", "r"}:
            raise ValueError("legs.passive_joints must map exactly {'l', 'r'}")
        actuated = _actuated_leg_dof_ids(model)
        for side, fields in passive.items():
            if not isinstance(fields, dict):
                raise ValueError(f"legs.passive_joints.{side} must be a mapping")
            unknown = set(fields) - {"viscous_friction_nms_rad", "coulomb_friction_nm"}
            if unknown:
                raise ValueError(
                    f"legs.passive_joints.{side} has unknown fields {sorted(unknown)}"
                )
            values = {field: _finite(f"legs.passive_joints.{side}", field, value)
                      for field, value in fields.items()}
            for field, number in values.items():
                if number < 0.0:
                    raise ValueError(f"legs.passive_joints.{side}.{field} must be >= 0")
            dof_ids = [
                dof for name, dof in _named_leg_dofs(model)
                if name.startswith(f"{side}_") and dof not in actuated
            ]
            if not dof_ids:
                raise ValueError(f"model has no passive leg dofs on side {side!r}")
            for dof in dof_ids:
                if "viscous_friction_nms_rad" in values:
                    model.dof_damping[dof] = values["viscous_friction_nms_rad"]
                if "coulomb_friction_nm" in values:
                    model.dof_frictionloss[dof] = values["coulomb_friction_nm"]
            for field, number in values.items():
                applied[f"{field}[passive_{side}]"] = number
    return applied


def _named_leg_dofs(model: mujoco.MjModel) -> list[tuple[str, int]]:
    """``(joint name, dof)`` of every five-bar chain + closure hinge."""
    dofs = []
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint) or ""
        if name.startswith(("l_", "r_")) and not name.endswith("_wheel"):
            dofs.append((name, int(model.jnt_dofadr[joint])))
    return dofs


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
    *, ground_geom_names: tuple[str, ...] | None = None,
) -> dict[str, float]:
    """Write plant-side identified parameters onto ``model`` (in place).

    Returns a human-readable record of every field written, suitable for
    logs and for tests.  Raises if the model is missing any expected
    actuator/geom, so a renamed MJCF cannot silently skip identification.
    ``ground_geom_names`` selects explicit terrain surfaces; an empty tuple
    preserves an external terrain's own materials instead of overriding them.
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
    if ground_geom_names is None and not any(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) >= 0
               for name in ("ground", "ground_outer")):
        raise ValueError("model has no ground or ground_outer geom")
    names = GROUND_GEOM_NAMES if ground_geom_names is None else ground_geom_names
    for name in names:
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom < 0:
            continue  # platform layers only exist in the platform scene
        model.geom_friction[geom, 0] = ground_sliding
    if names:
        applied["friction[ground]"] = ground_sliding
    # contacts.restitution is nominal 0, and MuJoCo encodes restitution via a
    # negative-solref parametrization; applying it is deferred until a
    # non-zero value is actually identified.

    # Optional asymmetric per-side leg overlay (2026-09-21 profile): runs
    # after the scalar path so its per-motor / per-side values win.
    legs = params.data.get("legs")
    if legs is not None:
        applied.update(_apply_legs_section(model, legs))
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
