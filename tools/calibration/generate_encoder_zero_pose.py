"""Generate the loop-closed MuJoCo pose for the calibrated encoder zero."""

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

from explore_recovery_poses import solve_pose
from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    VMC_ENCODER_ZERO_CAD_Q,
    VMC_ENCODER_ZERO_L0,
    VMC_ENCODER_ZERO_PHI0,
)


ROOT = REPO_ROOT
MODEL_PATH = ROOT / "model" / "whole_body_lqr.xml"
OUTPUT_PATH = ROOT / "encoder_zero_pose.json"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    source = json.loads(CALIBRATED_STAND_POSE_PATH.read_text(encoding="utf-8"))
    baseline = model.qpos0.copy()
    baseline[:7] = source["base_free"]
    for joint_id in range(1, model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name in source:
            baseline[model.jnt_qposadr[joint_id]] = source[name]

    drive_addresses = {
        name: int(model.jnt_qposadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name
        )])
        for name in VMC_ENCODER_ZERO_CAD_Q
    }
    start_drive = {
        name: float(baseline[address])
        for name, address in drive_addresses.items()
    }
    # Follow the physical motion from the known 0.150 m standing assembly.
    # The endpoint is across the four-bar dead point.  The passive-joint
    # limits in the model include this calibrated branch and the standing
    # branch, so recovery can cross the mechanism without disabling limits.
    result = baseline.copy()
    for step in range(1, 81):
        alpha = step / 80.0
        drive = {
            name: start_drive[name] + alpha * (
                VMC_ENCODER_ZERO_CAD_Q[name] - start_drive[name]
            )
            for name in VMC_ENCODER_ZERO_CAD_Q
        }
        result = solve_pose(model, data, result, drive)
        if result is None:
            raise RuntimeError(
                f"Unable to close encoder-zero continuation at {alpha:.3f}"
            )
    data.qpos[:] = result
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    controller = TraditionalLQR(model)
    controller.reset(data, calibrate_frame=True)
    for side in ("l", "r"):
        kin = controller.vmc_kinematics(data, side)
        if not np.isclose(kin.l0, VMC_ENCODER_ZERO_L0, atol=2e-9):
            raise RuntimeError(f"{side} encoder-zero l0 mismatch: {kin.l0}")
        if not np.isclose(kin.phi0, VMC_ENCODER_ZERO_PHI0, atol=2e-9):
            raise RuntimeError(f"{side} encoder-zero phi0 mismatch: {kin.phi0}")

    # The calibrated left/right linkages are not perfectly mirror-symmetric.
    # Level the complete closed mechanism about vehicle-forward so both wheel
    # centres have the same height.  This is a rigid root rotation: it changes
    # neither the encoder coordinates nor the VMC leg length/angle.
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    wheels = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel")
        for side in ("l", "r")
    ]
    free_joint = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "base_free"
    )
    qadr = int(model.jnt_qposadr[free_joint])
    q_base = result[qadr + 3:qadr + 7].copy()
    forward = data.xmat[base].reshape(3, 3) @ controller.forward_body

    def height_difference(angle: float) -> float:
        rotation = np.empty(4)
        composed = np.empty(4)
        mujoco.mju_axisAngle2Quat(rotation, forward, angle)
        mujoco.mju_mulQuat(composed, rotation, q_base)
        data.qpos[:] = result
        data.qpos[qadr + 3:qadr + 7] = composed
        mujoco.mj_forward(model, data)
        return float(data.xpos[wheels[0], 2] - data.xpos[wheels[1], 2])

    difference = height_difference(0.0)
    slope = (height_difference(.01) - difference) / .01
    if abs(slope) < 1e-7:
        raise RuntimeError("Could not level encoder-zero wheel contacts")
    roll = float(np.clip(-difference / slope, -.10, .10))
    rotation = np.empty(4)
    composed = np.empty(4)
    mujoco.mju_axisAngle2Quat(rotation, forward, roll)
    mujoco.mju_mulQuat(composed, rotation, q_base)
    result[qadr + 3:qadr + 7] = composed
    data.qpos[:] = result
    mujoco.mj_forward(model, data)

    passive_branch = {}
    for name in ("l_leg", "r_leg"):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        passive_branch[name] = float(data.qpos[model.jnt_qposadr[joint_id]])

    pose = {"base_free": result[:7].tolist()}
    for joint_id in range(1, model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        pose[name] = float(result[model.jnt_qposadr[joint_id]])
    OUTPUT_PATH.write_text(json.dumps(pose, indent=2), encoding="utf-8")
    loop_error = float(np.max(np.abs(data.efc_pos[: 3 * model.neq])))
    print(f"Wrote {OUTPUT_PATH}")
    print(
        f"l0={VMC_ENCODER_ZERO_L0:.9f} m, "
        f"phi0={np.degrees(VMC_ENCODER_ZERO_PHI0):.6f} deg, "
        f"max loop error={loop_error:.3e} m, "
        f"wheel-level roll={np.degrees(roll):+.3f} deg"
    )
    print(
        "dead-point branch: "
        + ", ".join(f"{name}={value:+.6f}" for name, value in passive_branch.items())
    )


if __name__ == "__main__":
    main()
