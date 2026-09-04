"""Find closed-loop crouch poses around calibrated code zero for recovery."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import json
from pathlib import Path

import mujoco
import numpy as np


ROOT = REPO_ROOT
MODEL_PATH = ROOT / "model" / "whole_body_lqr.xml"
POSE_PATH = ROOT / "code_zero_pose.json"
DRIVE = ("r_j1", "r_j2", "l_j1", "l_j2")
FORWARD_SIGN = {"r_j1": 1.0, "r_j2": 1.0, "l_j1": -1.0, "l_j2": -1.0}


def solve_pose(model: mujoco.MjModel, data: mujoco.MjData, baseline: np.ndarray, drive: dict[str, float]) -> np.ndarray | None:
    fixed = {
        int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]): value
        for name, value in drive.items()
    }
    excluded = set(fixed)
    for name in ("l_wheel", "r_wheel", "gimbal_yaw"):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        excluded.add(int(model.jnt_qposadr[joint]))
    variables = [
        int(model.jnt_qposadr[joint]) for joint in range(1, model.njnt)
        if int(model.jnt_qposadr[joint]) not in excluded
    ]

    def residual(values: np.ndarray) -> np.ndarray:
        data.qpos[:] = baseline
        for address, value in fixed.items():
            data.qpos[address] = value
        data.qpos[variables] = values
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        return data.efc_pos[: 3 * model.neq].copy()

    values = baseline[variables].copy()
    damping = 1e-5
    for _ in range(100):
        error = residual(values)
        if np.max(np.abs(error)) < 2e-6:
            return data.qpos.copy()
        jacobian = np.empty((error.size, len(variables)))
        for col in range(len(variables)):
            perturbed = values.copy()
            perturbed[col] += 1e-6
            jacobian[:, col] = (residual(perturbed) - error) / 1e-6
        delta = np.linalg.solve(jacobian.T @ jacobian + damping * np.eye(len(variables)), -jacobian.T @ error)
        cost = float(error @ error)
        accepted = False
        for scale in (1.0, .5, .25, .125, .0625):
            candidate = values + scale * delta
            trial = residual(candidate)
            if float(trial @ trial) < cost:
                values = candidate
                damping = max(damping * .3, 1e-10)
                accepted = True
                break
        if not accepted:
            damping = min(damping * 10.0, 1e6)
    return None


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    pose = json.loads(POSE_PATH.read_text(encoding="utf-8"))
    baseline = data.qpos.copy()
    baseline[:7] = pose["base_free"]
    for joint in range(1, model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if name in pose:
            baseline[model.jnt_qposadr[joint]] = pose[name]

    j1 = {side: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_j1") for side in ("l", "r")}
    wheel = {side: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel") for side in ("l", "r")}
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    for swing in (-.7, -.35, 0.0, .35, .7):
        for fold in (-.7, -.35, 0.0, .35, .7):
            candidate = {name: pose[name] + FORWARD_SIGN[name] * (swing + (fold if name.endswith("j1") else -fold)) for name in DRIVE}
            result = solve_pose(model, data, baseline, candidate)
            if result is None:
                print(f"swing={swing:+.2f} fold={fold:+.2f} NO_SOLUTION")
                continue
            data.qpos[:] = result
            mujoco.mj_forward(model, data)
            lengths = [float(np.linalg.norm(data.xpos[wheel[side]] - data.xanchor[j1[side]])) for side in ("l", "r")]
            wheel_heights = [float(data.xpos[wheel[side], 2] - .058) for side in ("l", "r")]
            print(f"swing={swing:+.2f} fold={fold:+.2f} len={lengths[0]:.3f}/{lengths[1]:.3f} base_z_if_ground={data.xpos[base, 2] - min(wheel_heights):.3f}")


if __name__ == "__main__":
    main()
