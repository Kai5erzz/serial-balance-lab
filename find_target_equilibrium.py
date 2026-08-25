"""Fit a static actuator bias for the closed-chain 90-degree stand pose."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import apply_saved_pose


ROOT = Path(__file__).resolve().parent


def evaluate(model: mujoco.MjModel, data: mujoco.MjData, pose: np.ndarray, ctrl: np.ndarray) -> np.ndarray:
    data.qpos[:] = pose
    data.qvel[:] = 0.0
    data.ctrl[:] = ctrl
    mujoco.mj_forward(model, data)
    return data.qacc.copy()


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "model" / "whole_body_lqr.xml"))
    data = mujoco.MjData(model)
    apply_saved_pose(model, data, ROOT / "vmc_low_stand_pose.json")
    # Ensure both levelled wheels are in finite contact before linearising.
    data.qpos[2] -= .002
    pose = data.qpos.copy()
    zero = np.zeros(model.nu)
    acceleration = evaluate(model, data, pose, zero)
    columns = []
    for actuator in range(model.nu):
        trial = zero.copy()
        trial[actuator] = 1.0
        columns.append(evaluate(model, data, pose, trial) - acceleration)
    jacobian = np.column_stack(columns)
    # Base and all non-free joint accelerations should vanish at a valid
    # static configuration. A small Tikhonov term avoids extreme torques.
    control = np.linalg.solve(jacobian.T @ jacobian + .02 * np.eye(model.nu), -jacobian.T @ acceleration)
    control = np.clip(control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
    final = evaluate(model, data, pose, control)
    print("contacts", data.ncon)
    print("zero_qacc_norm", float(np.linalg.norm(acceleration)), "fit_qacc_norm", float(np.linalg.norm(final)))
    print("actuators", [(model.actuator(i).name, float(control[i])) for i in range(model.nu)])
    print("base_qacc", final[:6])
    print("joint_qacc", [(model.joint(model.dof_jntid[dof]).name, float(final[dof])) for dof in range(6, model.nv)])


if __name__ == "__main__":
    main()
