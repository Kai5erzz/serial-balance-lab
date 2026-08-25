"""Solve all passive loop coordinates from the user-calibrated code zero."""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np


PROJECT = Path(__file__).resolve().parent
MODEL_PATH = PROJECT / "model" / "whole_body_lqr.xml"
OUTPUT = PROJECT / "code_zero_pose.json"

# User calibration shown in the MuJoCo information panel.  These values may
# be represented modulo 2*pi by the visualizer; retain the measured form.
CODE_ZERO_DRIVE = {
    "r_j1": 5.349,
    "r_j2": 4.615,
    "l_j1": -5.310,
    "l_j2": -4.593,
}


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    fixed = {
        model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]: value
        for name, value in CODE_ZERO_DRIVE.items()
    }
    excluded = set(fixed)
    for name in ("l_wheel", "r_wheel", "gimbal_yaw"):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        excluded.add(int(model.jnt_qposadr[joint]))
    variable_addresses = [
        int(model.jnt_qposadr[joint])
        for joint in range(1, model.njnt)
        if int(model.jnt_qposadr[joint]) not in excluded
    ]
    baseline = data.qpos.copy()

    def residual(values: np.ndarray) -> np.ndarray:
        data.qpos[:] = baseline
        for address, value in fixed.items():
            data.qpos[address] = value
        data.qpos[variable_addresses] = values
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        return data.efc_pos[: 3 * model.neq].copy()

    values = baseline[variable_addresses].copy()
    damping = 1e-5
    converged = False
    for iteration in range(300):
        error = residual(values)
        max_error = float(np.max(np.abs(error)))
        if max_error < 2e-6:
            converged = True
            break
        jacobian = np.empty((error.size, values.size))
        epsilon = 1e-6
        for column in range(values.size):
            perturbed = values.copy()
            perturbed[column] += epsilon
            jacobian[:, column] = (residual(perturbed) - error) / epsilon
        normal = jacobian.T @ jacobian + damping * np.eye(values.size)
        delta = np.linalg.solve(normal, -jacobian.T @ error)
        baseline_cost = float(error @ error)
        accepted = False
        for scale in (1.0, 0.5, 0.25, 0.125, 0.0625):
            candidate = values + scale * delta
            candidate_error = residual(candidate)
            if float(candidate_error @ candidate_error) < baseline_cost:
                values = candidate
                damping = max(damping * 0.3, 1e-10)
                accepted = True
                break
        if not accepted:
            damping = min(damping * 10.0, 1e6)
    error = residual(values)
    if not converged and float(np.max(np.abs(error))) > 2e-5:
        raise RuntimeError(
            "Unable to close the code-zero loop: "
            f"max error={np.max(np.abs(error)):.6g} m"
        )

    pose = {"base_free": baseline[:7].tolist()}
    for joint in range(1, model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        pose[name] = float(data.qpos[model.jnt_qposadr[joint]])
    OUTPUT.write_text(json.dumps(pose, indent=2), encoding="utf-8")
    print(f"Wrote {OUTPUT}")
    print(f"passive coordinates solved={len(variable_addresses)}; max loop error={np.max(np.abs(error)):.3e} m")
    for name in CODE_ZERO_DRIVE:
        print(f"{name}={pose[name]:+.6f} rad")


if __name__ == "__main__":
    main()
