"""Torque-driven compact-leg recovery using the firmware VMC motor map."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import (
    CONTROL_PERIOD_S,
    TraditionalLQR,
    apply_code_zero_pose,
    apply_saved_pose,
)


ROOT = Path(__file__).resolve().parent
MODEL = ROOT / "model" / "whole_body_lqr.xml"
CROUCH_POSE = ROOT / "recovery_crouch_pose.json"
RUN_SECONDS = 4.0
RECOVERY_LENGTH_KP = 700.0
RECOVERY_LENGTH_KD = 28.0
RECOVERY_FORCE_MIN = -100.0
RECOVERY_FORCE_MAX = 35.0
RECOVERY_TP_KP = 32.0
RECOVERY_TP_KD = 5.5


def wrap(value: float) -> float:
    return float((value + np.pi) % (2.0 * np.pi) - np.pi)


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)

    apply_code_zero_pose(model, data)
    target = {side: controller.vmc_kinematics(data, side) for side in ("l", "r")}
    apply_saved_pose(model, data, CROUCH_POSE)
    controller.reset(data)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    last_length = {side: controller.vmc_kinematics(data, side).l0 for side in ("l", "r")}
    elapsed = 0.0

    while data.time < RUN_SECONDS:
        controller._apply_gas_springs(data)
        elapsed += model.opt.timestep
        if elapsed + 1e-12 >= CONTROL_PERIOD_S:
            dt = elapsed
            elapsed = 0.0
            for side in ("l", "r"):
                kin = controller.vmc_kinematics(data, side)
                d_l0 = (kin.l0 - last_length[side]) / dt
                d_phi = 0.0
                force = np.clip(
                    RECOVERY_LENGTH_KP * (target[side].l0 - kin.l0) - RECOVERY_LENGTH_KD * d_l0,
                    RECOVERY_FORCE_MIN, RECOVERY_FORCE_MAX,
                )
                tp = np.clip(
                    RECOVERY_TP_KP * wrap(target[side].phi0 - kin.phi0) - RECOVERY_TP_KD * d_phi,
                    -10.0, 10.0,
                )
                controller.set_vmc_motor_torques(data, side, float(force), float(tp))
                last_length[side] = kin.l0
        mujoco.mj_step(model, data)

    kin = {side: controller.vmc_kinematics(data, side) for side in ("l", "r")}
    print(f"finite={np.isfinite(data.qpos).all()} time={data.time:.3f} contacts={data.ncon}")
    print(f"base_z={data.xpos[base, 2]:.4f}")
    print(f"l0={kin['l'].l0:.4f}/{kin['r'].l0:.4f} target={target['l'].l0:.4f}/{target['r'].l0:.4f}")
    print(f"phi0={kin['l'].phi0:.4f}/{kin['r'].phi0:.4f} target={target['l'].phi0:.4f}/{target['r'].phi0:.4f}")
    print(f"motor_tau_l={controller.last_motor_torque['l']} motor_tau_r={controller.last_motor_torque['r']}")


if __name__ == "__main__":
    main()
