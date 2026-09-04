"""Empirically map the real prone-pose recovery control directions.

This is deliberately headless.  It starts from CAD q=0, lays the complete
chassis on the ground, then applies physically bounded bilateral VMC/wheel
commands.  The printed table is used to choose the torque polarity and the
first recovery stage; no position teleporting occurs after the initial pose.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from itertools import product
from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import TraditionalLQR, apply_code_zero_pose


MODEL = REPO_ROOT / "model" / "whole_body_lqr.xml"


def rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    skew = np.array(((0.0, -axis[2], axis[1]), (axis[2], 0.0, -axis[0]), (-axis[1], axis[0], 0.0)))
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def make_prone(model: mujoco.MjModel, sign: float) -> tuple[mujoco.MjData, TraditionalLQR]:
    data = mujoco.MjData(model)
    apply_code_zero_pose(model, data)
    controller = TraditionalLQR(model)
    controller.reset(data, calibrate_frame=True)
    _, _, lateral = controller._frame(data)
    base_rotation = data.xmat[controller.base].reshape(3, 3)
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, (rotation(lateral, sign * np.pi / 2.0) @ base_rotation).ravel())
    data.qpos[3:7] = quaternion
    # Both orientations are lowered to their actual first collision, rather
    # than being held above the floor.  The different values reflect the
    # asymmetric chassis exterior.
    # 2 mm contact preload, calibrated from the actual mesh contact distances.
    # The belly-down orientation has several chassis contacts, so it must not
    # start 2.6 mm inside the floor or the constraint impulse folds a leg.
    data.qpos[2] = 0.3605 if sign > 0.0 else 0.2365
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    controller.reset(data)  # retain the world longitudinal calibration
    return data, controller


def run_pulse(model: mujoco.MjModel, sign: float, force: float, tp: float, wheel: float, seconds: float = 0.60) -> tuple[float, float, int]:
    data, controller = make_prone(model, sign)
    pitch0 = float(np.degrees(controller.body_pitch(data)))
    for _ in range(round(seconds / model.opt.timestep)):
        controller._apply_gas_springs(data)
        data.ctrl[:] = 0.0
        controller._apply_gimbal_lock(data)
        for side, ids in controller.sides.items():
            controller.set_vmc_motor_torques(data, side, force, tp)
            data.ctrl[ids.wheel_motor] = controller.wheel_sign[side] * wheel
        mujoco.mj_step(model, data)
    return float(np.degrees(controller.body_pitch(data))) - pitch0, float(data.xpos[controller.base, 2]), data.ncon


def trace_pulse(model: mujoco.MjModel, sign: float, force: float, tp: float, wheel: float, seconds: float = 3.0) -> None:
    """Print a sparse pose trace for one candidate swing-up pulse."""
    data, controller = make_prone(model, sign)
    print("time pitch z pitch_rate contacts q(l1,l2,r1,r2)")
    every = max(1, round(0.05 / model.opt.timestep))
    for step in range(round(seconds / model.opt.timestep)):
        controller._apply_gas_springs(data)
        data.ctrl[:] = 0.0
        controller._apply_gimbal_lock(data)
        for side, ids in controller.sides.items():
            controller.set_vmc_motor_torques(data, side, force, tp)
            data.ctrl[ids.wheel_motor] = controller.wheel_sign[side] * wheel
        mujoco.mj_step(model, data)
        if step % every == 0:
            _, _, lateral = controller._frame(data)
            q = [data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]] for name in ("l_j1", "l_j2", "r_j1", "r_j2")]
            rate = -float(np.dot(data.qvel[3:6], lateral))
            print(f"{data.time:4.2f} {np.degrees(controller.body_pitch(data)):7.1f} {data.xpos[controller.base,2]:.3f} {np.degrees(rate):8.1f} {data.ncon:2d} " + " ".join(f"{x:+.2f}" for x in q))


def run_phi_swing(model: mujoco.MjModel, sign: float, direction: float, force: float, seconds: float = 6.0) -> tuple[float, float, float, int]:
    """Port the firmware inverted-turn phi-rate loop for one direction."""
    data, controller = make_prone(model, sign)
    rate = direction * np.radians(105.0)
    refs = {side: controller.vmc_kinematics(data, side).phi0 for side in ("l", "r")}
    actual = refs.copy()
    previous_wrapped = refs.copy()
    best_abs_pitch = abs(float(np.degrees(controller.body_pitch(data))))
    for _ in range(round(seconds / model.opt.timestep)):
        commands = {}
        for side in ("l", "r"):
            kin = controller.vmc_kinematics(data, side)
            delta_phi = controller._wrap_pi(kin.phi0 - previous_wrapped[side])
            actual[side] += delta_phi
            previous_wrapped[side] = kin.phi0
            refs[side] += rate * model.opt.timestep
            phi_rate = delta_phi / model.opt.timestep
            # Same physical gains and +4 Nm travel bias as the firmware's
            # inverted turn; VMC output sign is deliberately empirical.
            commands[side] = float(np.clip(36.0 * (refs[side] - actual[side]) + 2.6 * (rate - phi_rate) + direction * 4.0, -9.5, 9.5))
        controller._apply_gas_springs(data)
        data.ctrl[:] = 0.0
        controller._apply_gimbal_lock(data)
        for side, ids in controller.sides.items():
            controller.set_vmc_motor_torques(data, side, force, commands[side])
        mujoco.mj_step(model, data)
        best_abs_pitch = min(best_abs_pitch, abs(float(np.degrees(controller.body_pitch(data)))))
    return (float(np.degrees(controller.body_pitch(data))), best_abs_pitch,
            float(data.xpos[controller.base, 2]), data.ncon)


def run_flip_handoff(model: mujoco.MjModel, sign: float, force: float, tp: float, wheel: float, seconds: float = 12.0, handoff_pitch: float = 35.0, contact_min: int = 0) -> tuple[bool, float, float, str, int]:
    """Use one physical swing pulse, then hand to the CAD stance controller."""
    data, controller = make_prone(model, sign)
    handoff = False
    for _ in range(round(seconds / model.opt.timestep)):
        pitch = float(np.degrees(controller.body_pitch(data)))
        if not handoff and data.time > 1.50 and abs(pitch) < handoff_pitch and data.ncon >= contact_min:
            handoff = True
            controller.reset(data)
            controller.begin_recovery(data)
        if handoff:
            controller.step(data, model.opt.timestep)
        else:
            controller._apply_gas_springs(data)
            data.ctrl[:] = 0.0
            controller._apply_gimbal_lock(data)
            for side, ids in controller.sides.items():
                controller.set_vmc_motor_torques(data, side, force, tp)
                data.ctrl[ids.wheel_motor] = controller.wheel_sign[side] * wheel
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all():
            return handoff, np.nan, np.nan, "nonfinite", data.ncon
    return (handoff, float(np.degrees(controller.body_pitch(data))),
            float(data.xpos[controller.base, 2]), controller.recovery_state, data.ncon)


def run_flip_joint_trim(model: mujoco.MjModel, sign: float, seconds: float = 18.0) -> tuple[bool, float, float, dict[str, float], dict[str, float]]:
    """Test a torque-PD re-entry into the measured normal trim branch."""
    data, controller = make_prone(model, sign)
    refs = {"l_j1": -0.516686, "l_j2": -0.515576, "r_j1": 0.539810, "r_j2": 0.528595}
    actuator = {name: controller._actuator(f"{name}_motor") for name in refs}
    joint = {name: controller._joint(name) for name in refs}
    stage = "swing"
    trim_elapsed = 0.0
    handoff = False
    for _ in range(round(seconds / model.opt.timestep)):
        pitch = float(np.degrees(controller.body_pitch(data)))
        if stage == "swing" and data.time > 1.5 and abs(pitch) < 20.0 and data.ncon >= 2:
            stage = "trim"
            trim_elapsed = 0.0
        if stage == "swing":
            controller._apply_gas_springs(data)
            data.ctrl[:] = 0.0
            controller._apply_gimbal_lock(data)
            for side, ids in controller.sides.items():
                controller.set_vmc_motor_torques(data, side, -80.0, -12.0)
                data.ctrl[ids.wheel_motor] = controller.wheel_sign[side] * -3.69
        elif stage == "trim":
            trim_elapsed += model.opt.timestep
            controller._apply_gas_springs(data)
            data.ctrl[:] = 0.0
            controller._apply_gimbal_lock(data)
            for name, target in refs.items():
                qadr = model.jnt_qposadr[joint[name]]
                dadr = model.jnt_dofadr[joint[name]]
                err = controller._wrap_pi(target - data.qpos[qadr])
                data.ctrl[actuator[name]] = np.clip(30.0 * err - 1.5 * data.qvel[dadr], -22.0, 22.0)
            _, forward, lateral = controller._frame(data)
            wheel_state = np.array((controller.body_pitch(data), -np.dot(data.qvel[3:6], lateral), np.dot(data.xpos[controller.base] - controller.base_position0, forward), np.dot(data.qvel[:3], forward)))
            wheel_torque = float(np.clip(-np.array((-5.63055, .14467, -1.66558, -.44693)) @ wheel_state, -3.69, 3.69))
            for side, ids in controller.sides.items():
                data.ctrl[ids.wheel_motor] = controller.wheel_sign[side] * wheel_torque
            max_err = max(abs(controller._wrap_pi(target - data.qpos[model.jnt_qposadr[joint[name]]])) for name, target in refs.items())
            if trim_elapsed > 1.5 and max_err < 0.07 and abs(pitch) < 10.0:
                stage = "cad"
                handoff = True
                controller.reset(data)
                controller.begin_recovery(data)
        else:
            controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    q = {name: float(data.qpos[model.jnt_qposadr[joint[name]]]) for name in refs}
    kin = {side: (controller.vmc_kinematics(data, side).l0, np.degrees(controller.vmc_kinematics(data, side).phi0)) for side in ("l", "r")}
    return handoff, float(np.degrees(controller.body_pitch(data))), float(data.xpos[controller.base, 2]), q, kin


def run_flip_vmc_retrim(model: mujoco.MjModel, sign: float, seconds: float = 18.0) -> tuple[bool, float, float, dict[str, float]]:
    """Test VMC-force shortening back into the normal CAD trim branch."""
    data, controller = make_prone(model, sign)
    targets = {"l": (0.2431703, np.radians(119.569)), "r": (0.2413014, np.radians(120.603))}
    last = {side: controller.vmc_kinematics(data, side) for side in ("l", "r")}
    stage = "swing"
    trim_elapsed = 0.0
    handoff = False
    phase_actual: dict[str, float] = {}
    phase_previous: dict[str, float] = {}
    phase_reference: dict[str, float] = {}
    phase_target: dict[str, float] = {}
    for _ in range(round(seconds / model.opt.timestep)):
        pitch = float(np.degrees(controller.body_pitch(data)))
        if stage == "swing" and data.time > 1.5 and abs(pitch) < 20.0 and data.ncon >= 2:
            stage = "retrim"
            trim_elapsed = 0.0
        if stage == "swing":
            controller._apply_gas_springs(data)
            data.ctrl[:] = 0.0
            controller._apply_gimbal_lock(data)
            for side, ids in controller.sides.items():
                controller.set_vmc_motor_torques(data, side, -80.0, -12.0)
                data.ctrl[ids.wheel_motor] = controller.wheel_sign[side] * -3.69
        elif stage in {"retrim", "phase"}:
            trim_elapsed += model.opt.timestep
            controller._apply_gas_springs(data)
            data.ctrl[:] = 0.0
            controller._apply_gimbal_lock(data)
            _, forward, lateral = controller._frame(data)
            wheel_state = np.array((controller.body_pitch(data), -np.dot(data.qvel[3:6], lateral), np.dot(data.xpos[controller.base] - controller.base_position0, forward), np.dot(data.qvel[:3], forward)))
            wheel_torque = float(np.clip(-np.array((-5.63055, .14467, -1.66558, -.44693)) @ wheel_state, -3.69, 3.69))
            errors = []
            for side, ids in controller.sides.items():
                kin = controller.vmc_kinematics(data, side)
                dlen = (kin.l0 - last[side].l0) / model.opt.timestep
                dphi = controller._wrap_pi(kin.phi0 - last[side].phi0) / model.opt.timestep
                lref, phiref = targets[side]
                force = float(np.clip(1000.0 * (kin.l0 - lref) + 3.0 * dlen, -20.0, 100.0))
                if stage == "retrim":
                    tp = float(np.clip(20.0 * controller._wrap_pi(phiref - kin.phi0) - 1.5 * dphi, -10.0, 10.0))
                else:
                    delta = controller._wrap_pi(kin.phi0 - phase_previous[side])
                    phase_actual[side] += delta
                    phase_previous[side] = kin.phi0
                    phase_reference[side] = max(phase_target[side], phase_reference[side] - np.radians(85.0) * model.opt.timestep)
                    tp = float(np.clip(26.0 * (phase_reference[side] - phase_actual[side]) - 1.5 * (dphi + np.radians(85.0)), -10.0, 10.0))
                controller.set_vmc_motor_torques(data, side, force, tp)
                data.ctrl[ids.wheel_motor] = controller.wheel_sign[side] * wheel_torque
                errors.extend((abs(kin.l0-lref),abs(controller._wrap_pi(phiref-kin.phi0))))
                last[side] = kin
            length_error = max(errors[0::2])
            if stage == "retrim" and trim_elapsed > 1.0 and length_error < 0.015 and abs(pitch) < 10.0:
                stage = "phase"
                for side in ("l", "r"):
                    phi = controller.vmc_kinematics(data, side).phi0
                    phase_actual[side] = phi
                    phase_previous[side] = phi
                    phase_reference[side] = phi
                    phase_target[side] = phi - np.pi
            if stage == "phase" and all(phase_actual[side] <= phase_target[side] + 0.08 for side in ("l", "r")) and abs(pitch) < 10.0:
                stage = "cad"
                handoff = True
                controller.reset(data)
                controller.begin_recovery(data)
        else:
            controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    kin = {
        side: (controller.vmc_kinematics(data, side).l0,
               float(np.degrees(controller.vmc_kinematics(data, side).phi0)))
        for side in ("l", "r")
    }
    return handoff, float(np.degrees(controller.body_pitch(data))), float(data.xpos[controller.base, 2]), kin


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    for sign in (-1.0, 1.0):
        print(f"\nprone rotation sign={sign:+.0f}")
        rows = []
        for force, tp, wheel in product((-80.0, 0.0, 80.0), (-12.0, 0.0, 12.0), (-3.69, 0.0, 3.69)):
            delta, height, contacts = run_pulse(model, sign, force, tp, wheel)
            rows.append((abs(delta), delta, force, tp, wheel, height, contacts))
        for _, delta, force, tp, wheel, height, contacts in sorted(rows, reverse=True)[:12]:
            print(f"dPitch={delta:+7.1f} F={force:+5.0f} Tp={tp:+5.1f} Wh={wheel:+5.2f} z={height:.3f} c={contacts}")


if __name__ == "__main__":
    main()
