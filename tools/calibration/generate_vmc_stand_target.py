"""Save the verified CAD-zero normal-standing pose for MuJoCo."""

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

from traditional_lqr import TraditionalLQR, apply_code_zero_pose


ROOT = REPO_ROOT
MODEL_PATH = ROOT / "model" / "whole_body_lqr.xml"
OUTPUT = ROOT / "vmc_low_stand_pose.json"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    apply_code_zero_pose(model, data)
    baseline = data.qpos.copy()
    # Preserve the calibrated vehicle-forward direction while levelling the
    # two tyre contacts.  Leg qpos stays exactly at the verified CAD-zero
    # normal-standing pose.
    data.qpos[:] = baseline
    mujoco.mj_forward(model, data)
    frame = TraditionalLQR(model)
    frame.reset(data)
    forward_body = frame.forward_body.copy()

    result = baseline.copy()
    data.qpos[:] = result
    mujoco.mj_forward(model, data)

    # A sub-degree CAD asymmetry leaves the wheels about 4 mm apart in height
    # at this pose.  Roll the complete closed mechanism about vehicle-forward
    # so both physical tyre contacts start on the same ground plane.  This is
    # a rigid global rotation and therefore does not change any loop closure
    # or motor/VMC coordinate.
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    wheels = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel") for side in ("l", "r")]
    qadr = int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")])
    q_base = result[qadr + 3:qadr + 7].copy()
    forward = data.xmat[base].reshape(3, 3) @ forward_body

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
        raise RuntimeError("Could not level wheel contacts: roll derivative is zero")
    roll = float(np.clip(-difference / slope, -.10, .10))
    rotation = np.empty(4)
    composed = np.empty(4)
    mujoco.mju_axisAngle2Quat(rotation, forward, roll)
    mujoco.mju_mulQuat(composed, rotation, q_base)
    result[qadr + 3:qadr + 7] = composed
    data.qpos[:] = result
    mujoco.mj_forward(model, data)

    pose = {"base_free": result[:7].tolist()}
    for joint in range(1, model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        pose[name] = float(result[model.jnt_qposadr[joint]])
    OUTPUT.write_text(json.dumps(pose, indent=2), encoding="utf-8")
    l0 = [frame.vmc_kinematics(data, side).l0 for side in ("l", "r")]
    phi = [np.degrees(frame.vmc_kinematics(data, side).phi0) for side in ("l", "r")]
    print(f"Wrote {OUTPUT}; CAD q=0 standing l0={l0[0]:.5f}/{l0[1]:.5f} m, phi0={phi[0]:.2f}/{phi[1]:.2f} deg; wheel-level roll={np.degrees(roll):+.3f} deg")


if __name__ == "__main__":
    main()
