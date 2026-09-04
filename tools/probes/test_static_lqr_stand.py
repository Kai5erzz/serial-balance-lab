"""Check that the corrected VMC-state LQR can hold the 90-degree stand pose."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import os
from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import TraditionalLQR, apply_code_zero_pose, apply_saved_pose


ROOT = REPO_ROOT


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "model" / "whole_body_lqr.xml"))
    if os.environ.get("DISABLE_BASE_GROUND_DIAGNOSTIC", "0") == "1":
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        mask = model.geom_bodyid == base_id
        model.geom_contype[mask] = 0
        model.geom_conaffinity[mask] = 0
    data = mujoco.MjData(model)
    ctrl = TraditionalLQR(model)
    apply_code_zero_pose(model, data)
    ctrl.reset(data, calibrate_frame=True)
    apply_saved_pose(model, data, ROOT / "vmc_low_stand_pose.json")
    # MuJoCo creates a contact only after a finite overlap.  Preload the two
    # tyres by 2 mm, matching the runtime reset convention.
    data.qpos[2] -= .002
    mujoco.mj_forward(model, data)
    ctrl.reset(data)
    ctrl.lqr_leg_level = "low"
    wheel_sign = float(os.environ.get("LQR_WHEEL_GLOBAL_SIGN", "1"))
    ctrl.wheel_sign = {"l": -wheel_sign, "r": wheel_sign}
    ctrl.vmc_tp_sign = float(os.environ.get("LQR_TP_SIGN", "-1"))
    ctrl.enabled = True
    ctrl.lqr_gain = 1.0
    ctrl.stance_enabled = True
    ctrl.gas_enabled = False
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    peak_pitch = 0.0
    for _ in range(5000):
        ctrl.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        peak_pitch = max(peak_pitch, abs(np.degrees(ctrl.body_pitch(data))))
    print("sign", wheel_sign, ctrl.vmc_tp_sign, "finite", np.isfinite(data.qpos).all(), "contacts", data.ncon)
    print("base_z", data.xpos[base, 2], "pitch", np.degrees(ctrl.body_pitch(data)), "peak", peak_pitch)
    print("vmc", [(s, ctrl.vmc_kinematics(data, s).l0, np.degrees(ctrl.vmc_kinematics(data, s).phi0)) for s in "lr"])
    print("wheel", ctrl.last_output, "hip", ctrl.last_motor_torque)


if __name__ == "__main__":
    main()
