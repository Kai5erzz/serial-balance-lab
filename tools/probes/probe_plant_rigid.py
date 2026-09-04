"""Plant direction with hips position-locked (rigid cart), wheels torque fixed."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import mujoco
import numpy as np

MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast.xml"
HIP_KP = 80.0
HIP_KD = 8.0


def run(torque: float) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    from traditional_lqr import CALIBRATED_STAND_POSE_PATH, TraditionalLQR, apply_saved_pose
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    free = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    qadr = model.jnt_qposadr[free]
    wl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "l_wheel_motor")
    wr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_wheel_motor")
    hip_joints = ["r_j1", "r_j2", "l_j1", "l_j2"]
    hips = {
        name: (
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_motor"),
        )
        for name in hip_joints
    }
    qadrs = {name: model.jnt_qposadr[jid] for name, (jid, _) in hips.items()}
    dadrs = {name: model.jnt_dofadr[jid] for name, (jid, _) in hips.items()}

    # stand with the LQR, then capture the pose
    end = data.time + 1.0
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    targets = {name: float(data.qpos[qadrs[name]]) for name in hips}
    start = data.xpos[base].copy()
    nose = controller.forward_world
    # hips locked by PD, wheels fixed torque
    data.ctrl[wl] = -torque
    data.ctrl[wr] = +torque
    last = 0.0
    while data.time < 2.0:
        for name in hips:
            error = targets[name] - data.qpos[qadrs[name]]
            data.ctrl[hips[name][1]] = float(np.clip(
                HIP_KP * error - HIP_KD * data.qvel[dadrs[name]],
                -24.0, 24.0,
            ))
        mujoco.mj_step(model, data)
        if data.time - last >= 0.4:
            last = data.time
            disp = data.xpos[base] - start
            print(
                f"    t={data.time:.2f} disp=({disp[0]:+.3f},{disp[1]:+.3f}) "
                f"nose_dist={np.dot(disp, nose):+.3f} "
                f"wq_l/r={data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'l_wheel')]]:+.2f}/"
                f"{data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'r_wheel')]]:+.2f}"
            )


if __name__ == "__main__":
    for t in (1.0, -1.0, 2.0):
        print(f"=== rigid cart, fixed wheel torque {t:+.1f} Nm (l=-T, r=+T) ===")
        run(t)
