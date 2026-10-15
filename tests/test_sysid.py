"""Regression tests for the sysid integration (balance_lab.sysid).

Locks the identified-parameter pipeline end to end:
- the bundled profile loads, validates, and keeps its cross-repository
  fingerprint (identical to serial_balance_isaac in rl-serial-balance);
- plant-side application writes exactly the documented fields, is
  idempotent, and leaves the nominal plant untouched;
- the coupled-mechanics overlay follows identify_real2sim.py's rules;
- controller-side application (gas spring, theta compensation) survives
  reset and rejects unrepresentable profiles;
- the identified plant still balances under the firmware LQR.
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import balance_lab as bl
from balance_lab.plant import load_model
from balance_lab.sysid import (
    NOMINAL_HIP_JOINT_DAMPING,
    NOMINAL_WHEEL_JOINT_DAMPING,
    MechanicsFit,
    apply_mechanics_fit,
    apply_model_parameters,
    load_model_parameters,
    load_profile,
)
from balance_lab.sysid.apply import (
    HIP_ACTUATOR_NAMES,
    WHEEL_ACTUATOR_NAMES,
    _chain_dof_ids,
    _leg_dof_ids,
    _wheel_dof_ids,
)

FINGERPRINT = "495989a22f4a6201"
FINGERPRINT_2026_09_21 = "b3d77732f0b5d7ee"
ASYMMETRIC = load_profile("identified_2026-09-21")

PARAMS = load_profile()


def test_profile_fingerprint_is_stable() -> None:
    """The bundled profile keeps its provenance fingerprint (cross-repo key)."""
    assert PARAMS.fingerprint == FINGERPRINT, (
        "identified_2026-09-04.yaml changed; re-pin FINGERPRINT and document "
        "the new identification run in docs/SYSID.md"
    )
    reloaded = load_model_parameters(
        bl.REPO_ROOT / "balance_lab" / "sysid" / "profiles" / "identified_2026-09-04.yaml"
    )
    assert reloaded.fingerprint == FINGERPRINT


def _chain_and_loop_dofs(model: mujoco.MjModel) -> tuple[list[int], list[int], list[int]]:
    """(motor-chain, closure-constraint, wheel) dof addresses of the legs."""
    chain = _chain_dof_ids(model)
    loops = [dof for dof in _leg_dof_ids(model) if dof not in set(chain)]
    wheels = _wheel_dof_ids(model)
    return chain, loops, wheels


def test_nominal_damping_constants_match_every_model_variant() -> None:
    """The scale semantics of the real2sim fitter need the shipped damping."""
    for variant in ("fast", "full", "platform"):
        model = mujoco.MjModel.from_xml_path(str(bl.REPO_ROOT / "model" / {
            "fast": "whole_body_lqr_fast.xml",
            "full": "whole_body_lqr.xml",
            "platform": "whole_body_lqr_fast_platform.xml",
        }[variant]))
        chain, loops, wheels = _chain_and_loop_dofs(model)
        assert chain and loops and wheels
        for dof in chain:
            assert model.dof_damping[dof] == NOMINAL_HIP_JOINT_DAMPING
        for dof in loops:
            assert model.dof_damping[dof] == 0.0  # closure constraints
        for dof in chain + loops:
            assert model.dof_frictionloss[dof] == 0.0
            assert model.dof_armature[dof] == 0.0
        for dof in wheels:
            assert model.dof_damping[dof] == NOMINAL_WHEEL_JOINT_DAMPING


def test_apply_model_parameters_writes_documented_fields() -> None:
    model, applied = load_model("fast", plant=PARAMS)
    hip = PARAMS.section("actuators")["hip"]
    wheel = PARAMS.section("actuators")["wheel"]
    for name in HIP_ACTUATOR_NAMES:
        actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        assert model.actuator_gainprm[actuator, 0] == hip["strength_scale"]
    for name in WHEEL_ACTUATOR_NAMES:
        actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        assert model.actuator_gainprm[actuator, 0] == wheel["strength_scale"]
    chain, loops, wheels = _chain_and_loop_dofs(model)
    assert all(model.dof_damping[dof] == hip["viscous_friction_nms_rad"] for dof in chain)
    assert all(model.dof_damping[dof] == 0.0 for dof in loops)  # constraints untouched
    assert all(model.dof_damping[dof] == wheel["viscous_friction_nms_rad"] for dof in wheels)
    assert applied["friction[wheel]"] == PARAMS.section("contacts")["wheel_dynamic_friction"]
    assert applied["friction[ground]"] == PARAMS.section("contacts")["ground_static_friction"]


def test_application_is_idempotent() -> None:
    model, applied = load_model("fast", plant=PARAMS)
    again = apply_model_parameters(model, PARAMS)
    assert applied == again


def test_external_terrain_can_retain_its_own_contact_materials() -> None:
    model, _ = load_model("fast", plant="nominal")
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    model.geom_friction[ground, :] = (.73, .014, .002)
    original = model.geom_friction[ground].copy()
    applied = apply_model_parameters(model, PARAMS, ground_geom_names=())
    np.testing.assert_array_equal(model.geom_friction[ground], original)
    assert "friction[ground]" not in applied
    assert applied["friction[wheel]"] == PARAMS.number("contacts", "wheel_dynamic_friction")


def test_identified_platform_applies_every_driving_surface() -> None:
    model, applied = load_model("platform", plant=ASYMMETRIC)
    friction = ASYMMETRIC.number("contacts", "ground_static_friction")
    for name in ("ground_outer", "platform_top", "platform_step1", "platform_step2", "platform_step3"):
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        assert geom >= 0
        assert model.geom_friction[geom, 0] == friction
    assert applied["friction[ground]"] == friction


def test_nominal_plant_is_untouched() -> None:
    model, applied = load_model("fast")
    assert applied == {}
    assert all(model.actuator_gainprm[a, 0] == 1.0 for a in range(model.nu))
    chain, loops, wheels = _chain_and_loop_dofs(model)
    assert all(model.dof_damping[dof] == NOMINAL_HIP_JOINT_DAMPING for dof in chain)
    assert all(model.dof_frictionloss[dof] == 0.0 for dof in chain + loops + wheels)


def test_mechanics_fit_overlay_rules() -> None:
    fit = MechanicsFit(
        hip_damping_scale=0.6,
        hip_frictionloss_nm=0.3,
        hip_armature_kg_m2=0.0098,
        wheel_damping_scale=1.5,
        wheel_frictionloss_nm=0.05,
        source="test-fixture",
    )
    model, applied = load_model("fast", plant=(PARAMS, fit))
    chain, loops, wheels = _chain_and_loop_dofs(model)
    assert all(model.dof_damping[dof] == 0.6 * NOMINAL_HIP_JOINT_DAMPING for dof in chain)
    assert all(model.dof_damping[dof] == 0.0 for dof in loops)
    assert all(model.dof_frictionloss[dof] == 0.3 for dof in chain + loops)
    assert all(model.dof_damping[dof] == 1.5 * NOMINAL_WHEEL_JOINT_DAMPING for dof in wheels)
    assert all(model.dof_frictionloss[dof] == 0.05 for dof in wheels)
    # rotor inertia reflects onto the four actuated hip joints only
    actuated = {
        int(model.jnt_dofadr[int(model.actuator_trnid[a, 0])])
        for a in range(model.nu)
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) in HIP_ACTUATOR_NAMES
    }
    assert actuated and all(model.dof_armature[dof] == 0.0098 for dof in actuated)
    assert all(model.dof_armature[dof] == 0.0 for dof in set(chain + loops) - actuated)
    assert applied["source"] == "test-fixture"
    # overlay without a profile is also fine (only mechanics fields written)
    model2 = mujoco.MjModel.from_xml_path(str(bl.MODEL_FAST))
    record = apply_mechanics_fit(model2, fit)
    assert record["hip_armature_kg_m2"] == 0.0098


def test_controller_apply_identified_and_reset_stability() -> None:
    model, _ = load_model("fast", plant=PARAMS)
    controller = bl.TraditionalLQR(model)
    data = mujoco.MjData(model)
    bl.apply_saved_pose(model, data, bl.CALIBRATED_STAND_POSE_PATH)
    controller.apply_identified(PARAMS)
    gas = PARAMS.section("gas_springs")
    assert controller.gas_spring_force == gas["force_n"] * gas["scale"]
    poly = PARAMS.numbers("theta_compensation", "polynomial_rad")
    assert controller.theta_comp_poly == tuple(poly)
    assert controller.theta_dx_offset_m == (
        PARAMS.section("theta_compensation")["dx_offset_left_m"]
    )
    # gas-spring conversion: virtual support scales linearly with force
    data2 = mujoco.MjData(model)
    bl.apply_saved_pose(model, data2, bl.CALIBRATED_STAND_POSE_PATH)
    controller.reset(data2)
    expected = _nominal_virtual_support(0.15) * (
        controller.gas_spring_force / bl.GAS_SPRING_FORCE
    )
    assert abs(controller._gas_virtual_support(0.15) - expected) < 1e-9
    # reset() must not clobber identified overrides
    controller.reset(data2)
    assert controller.gas_spring_force == gas["force_n"] * gas["scale"]


def _nominal_virtual_support(leg_length: float) -> float:
    """Reference conversion at the nominal 300 N gas-spring force."""
    force = bl.GAS_SPRING_FORCE
    denominator = 2.0 * bl.VMC_L1 * bl.VMC_L2
    cosine = np.clip(
        (bl.VMC_L1**2 + bl.VMC_L2**2 - leg_length**2) / denominator, -1.0, 1.0
    )
    theta3 = float(np.arccos(cosine))
    sine3 = max(float(np.sin(theta3)), 1e-5)
    ls = float(np.sqrt(
        bl.GAS_L6**2 + bl.GAS_LS2**2
        - 2.0 * bl.GAS_L6 * bl.GAS_LS2 * np.cos(theta3 - bl.GAS_ALPHA)
    ))
    return (
        force * leg_length * bl.GAS_L6 * bl.GAS_LS2
        * np.sin(theta3 - bl.GAS_ALPHA) / (ls * bl.VMC_L1 * bl.VMC_L2 * sine3)
    )


def test_controller_rejects_asymmetric_theta_offsets() -> None:
    import copy

    from balance_lab.sysid.parameters import ModelParameters

    broken = copy.deepcopy(PARAMS.data)
    broken["theta_compensation"]["dx_offset_right_m"] = (
        broken["theta_compensation"]["dx_offset_left_m"] + 0.004
    )
    controller = bl.TraditionalLQR(mujoco.MjModel.from_xml_path(str(bl.MODEL_FAST)))
    try:
        controller.apply_identified(ModelParameters(broken))
    except ValueError as error:
        assert "asymmetric" in str(error)
    else:
        raise AssertionError("asymmetric dx offsets must be rejected")


def test_identified_plant_balances_under_firmware_lqr() -> None:
    """Smoke acceptance: the identified plant is a usable validation plant."""
    model, _ = load_model("fast", plant=PARAMS)
    data = mujoco.MjData(model)
    controller = bl.TraditionalLQR(model)
    bl.apply_saved_pose(model, data, bl.CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    controller.apply_identified(PARAMS)
    for _ in range(int(3.0 / model.opt.timestep)):
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    assert abs(float(np.degrees(controller.body_pitch(data)))) < 5.0
    assert controller.is_properly_standing(data)


def test_identified_2026_09_21_profile_fingerprint() -> None:
    """The adopted asymmetric profile keeps its cross-repo provenance key."""
    assert ASYMMETRIC.fingerprint == FINGERPRINT_2026_09_21, (
        "identified_2026-09-21.yaml changed; re-pin FINGERPRINT_2026_09_21 "
        "and re-document the source hashes in its header"
    )
    source = bl.REPO_ROOT / "balance_lab" / "sysid" / "profiles" / "identified_2026-09-21.yaml"
    assert load_model_parameters(source).fingerprint == FINGERPRINT_2026_09_21
    # provenance: geometry carries the unified Z_new zero, not the legacy one
    zeros = ASYMMETRIC.section("geometry")["encoder_zero_rad"]
    assert zeros["left_j1"] == bl.VMC_ENCODER_ZERO_CAD_Q["l_j1"]
    assert zeros["right_j2"] == bl.VMC_ENCODER_ZERO_CAD_Q["r_j2"]


def test_legs_overlay_writes_documented_fields_idempotently() -> None:
    """Per-motor/per-side/mass overlay: exact values, set semantics, no drift."""
    from balance_lab.sysid.apply import _actuated_leg_dof_ids

    model = mujoco.MjModel.from_xml_path(str(bl.MODEL_FAST))
    nominal_mass = float(model.body_mass.sum())
    applied = apply_model_parameters(model, ASYMMETRIC)
    legs = ASYMMETRIC.section("legs")

    # per-motor gain / damping / coulomb / armature
    for joint_name, fields in legs["motors"].items():
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        dof = int(model.jnt_dofadr[joint])
        actuator = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{joint_name}_motor"
        )
        assert model.actuator_gainprm[actuator, 0] == fields["strength_scale"]
        assert model.dof_damping[dof] == fields["viscous_friction_nms_rad"]
        assert model.dof_frictionloss[dof] == fields["coulomb_friction_nm"]
        assert model.dof_armature[dof] == fields["armature_kg_m2"]
        assert applied[f"strength_scale[{joint_name}]"] == fields["strength_scale"]

    # passive side values on links/knee/closure, never on the motor hinges
    actuated = _actuated_leg_dof_ids(model)
    for name, dof in _named(model):
        side = name[0]
        if dof in actuated:
            continue
        assert model.dof_damping[dof] == legs["passive_joints"][side][
            "viscous_friction_nms_rad"
        ], name
        assert model.dof_frictionloss[dof] == legs["passive_joints"][side][
            "coulomb_friction_nm"
        ], name

    # mass scale: same body set the source profile defines, wheels included
    scale = legs["mass_inertia_scale"]
    pristine = mujoco.MjModel.from_xml_path(str(bl.MODEL_FAST))
    for body in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
        factor = scale["l"] if name.startswith("l_") else (
            scale["r"] if name.startswith("r_") else 1.0
        )
        np.testing.assert_allclose(model.body_mass[body], pristine.body_mass[body] * factor,
                                   rtol=0.0, atol=1e-15)
        np.testing.assert_allclose(model.body_inertia[body],
                                   pristine.body_inertia[body] * factor,
                                   rtol=0.0, atol=1e-15)
    total = float(model.body_mass.sum())
    np.testing.assert_allclose(
        total, ASYMMETRIC.number("rigid_bodies", "total_mass_kg"), atol=1e-9
    )
    assert applied["plant_total_mass_kg"] == total
    assert total > nominal_mass

    # idempotence: same model, second application changes nothing
    again = apply_model_parameters(model, ASYMMETRIC)
    assert again == applied
    assert float(model.body_mass.sum()) == total


def _named(model: mujoco.MjModel) -> list[tuple[str, int]]:
    from balance_lab.sysid.apply import _named_leg_dofs
    return _named_leg_dofs(model)


def test_per_leg_gain_table_matches_rl_serial_lock() -> None:
    """THE locked 2026-09-21 table (rl-serial-balance, cloud recipe on).

    rl-serial-balance ``virtual_leg.IDENTIFIED_PER_LEG_GAINS_2026_09_21`` /
    ``tests/test_virtual_leg_per_leg_pid_2026_09_21.py`` (20 pins) and the
    cloud recipe with ``vmc_per_leg_gains: True`` run these exact values at
    ``support_mass_kg = 13.592626``; rl-lab must reproduce them digit for
    digit or the two platforms are no longer validating the same controller.
    """
    from balance_lab.control.vmc_gains import derive_per_leg_gains

    gains = derive_per_leg_gains(bl.IDENTIFIED_SUPPORT_MASS_KG)
    expected = {
        #            kp           kd
        "l": {"length_kp": 2039.2593, "length_kd": 15.9257,
              "theta_kp": 12.4129, "theta_kd": 1.3405},
        "r": {"length_kp": 1960.7407, "length_kd": 28.0743,
              "theta_kp": 11.4862, "theta_kd": 1.8888},
    }
    for side in ("l", "r"):
        for channel, value in expected[side].items():
            np.testing.assert_allclose(
                gains[side][channel], value, rtol=0.0, atol=5e-4,
                err_msg=f"{side}.{channel} diverged from the rl-serial lock",
            )
        np.testing.assert_allclose(gains[side]["length_ki"], 100.0, rtol=0.0, atol=0.0)
    # mean stiffness stays the anchored design point for any support mass
    np.testing.assert_allclose(
        0.5 * (gains["l"]["length_kp"] + gains["r"]["length_kp"]), 2000.0, atol=1e-9
    )
    # design targets the table is solved onto
    from balance_lab.control.vmc_gains import OMEGA_THETA_RAD_S, ZETA_THETA
    np.testing.assert_allclose(OMEGA_THETA_RAD_S, 8.0, atol=0.0)
    np.testing.assert_allclose(ZETA_THETA, 0.70, atol=0.0)
    # static support at the ruling mass: 66.6718 N per leg
    np.testing.assert_allclose(
        0.5 * bl.IDENTIFIED_SUPPORT_MASS_KG * bl.GRAVITY, 66.6718, atol=5e-4
    )


def test_support_mass_defaults_to_ruling_and_profile_field_is_explicit() -> None:
    """The controller anchors on the 2026-09-21 ruling out of the box.

    Default (no profile): support_mass = 13.592626 -> the locked table and
    the 66.6718 N/leg feedforward.  ``apply_identified`` only moves it when
    a profile carries an explicit ``rigid_bodies.support_mass_kg``; the
    bundled legacy profile has no such field, so loading it cannot silently
    re-anchor the table (or drop it back to the stale firmware constant).
    """
    controller = bl.TraditionalLQR(mujoco.MjModel.from_xml_path(str(bl.MODEL_FAST)))
    assert controller.support_mass == 13.592626
    np.testing.assert_allclose(
        controller.length_gains["l"]["length_kp"], 2039.2593, atol=5e-4
    )
    np.testing.assert_allclose(
        controller.length_gains["r"]["length_kd"], 28.0743, atol=5e-4
    )

    # bundled legacy profile: no support_mass_kg field -> unchanged
    legacy_record = controller.apply_identified(PARAMS)
    assert "support_mass_kg" not in legacy_record
    assert controller.support_mass == 13.592626

    # adopted profile: explicit field, recorded, table re-derived identically
    record = controller.apply_identified(ASYMMETRIC)
    assert controller.support_mass == 13.592626
    assert record["support_mass_kg"] == 13.592626
    # total_mass_kg stays the ASSET number and must not leak into the anchor
    np.testing.assert_allclose(
        ASYMMETRIC.number("rigid_bodies", "total_mass_kg"), 13.836367294868388,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        controller.length_gains["l"]["length_kp"], 2039.2593, atol=5e-4
    )
    np.testing.assert_allclose(
        controller.length_gains["r"]["length_kp"], 1960.7407, atol=5e-4
    )


def test_identified_2026_09_21_plant_balances_under_firmware_lqr() -> None:
    """Smoke acceptance: the asymmetric identified plant is a usable plant."""
    model, applied = load_model("fast", plant="identified_2026-09-21")
    data = mujoco.MjData(model)
    controller = bl.TraditionalLQR(model)
    controller.apply_identified(ASYMMETRIC)
    bl.apply_saved_pose(model, data, bl.CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    for _ in range(int(3.0 / model.opt.timestep)):
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    assert abs(float(np.degrees(controller.body_pitch(data)))) < 5.0
    assert controller.is_properly_standing(data), (
        f"asymmetric plant failed to hold the stand; pitch="
        f"{np.degrees(controller.body_pitch(data)):.3f} deg"
    )


if __name__ == "__main__":
    for name, test in sorted(globals().items()):
        if name.startswith("test_") and callable(test):
            test()
            print(f"PASS {name}")
    print("sysid regression: all tests passed")
