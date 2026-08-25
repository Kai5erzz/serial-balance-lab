"""Diagnostic: can the actual closed-chain model reach calibrated stand pose?"""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import TraditionalLQR, apply_code_zero_pose


ROOT = Path(__file__).resolve().parent


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "model" / "whole_body_lqr.xml"))
    data = mujoco.MjData(model)
    apply_code_zero_pose(model, data)
    target = json.loads((ROOT / "vmc_low_stand_pose.json").read_text(encoding="utf-8"))
    ctrl = TraditionalLQR(model)
    ctrl.reset(data)
    ctrl.gas_enabled = False
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    for _ in range(6000):
        data.ctrl[:] = 0.0
        for side in "lr":
            ids = ctrl.sides[side]
            for joint, actuator, key in ((ids.j1, ids.hip_motors[0], f"{side}_j1"),
                                         (ids.j2, ids.hip_motors[1], f"{side}_j2")):
                qadr = model.jnt_qposadr[joint]
                dadr = model.jnt_dofadr[joint]
                tau = 100.0 * (target[key] - data.qpos[qadr]) - 4.0 * data.qvel[dadr]
                data.ctrl[actuator] = np.clip(tau, -24.0, 24.0)
        mujoco.mj_step(model, data)

    kin_l = ctrl.vmc_kinematics(data, "l")
    kin_r = ctrl.vmc_kinematics(data, "r")
    print("finite", np.isfinite(data.qpos).all(), "contacts", data.ncon)
    print("base_z", data.xpos[base, 2], "pitch", np.degrees(ctrl.body_pitch(data)))
    print("l0", kin_l.l0, kin_r.l0, "phi", np.degrees(kin_l.phi0), np.degrees(kin_r.phi0))
    for side in "lr":
        ids = ctrl.sides[side]
        print(side, "joints", data.qpos[model.jnt_qposadr[ids.j1]], data.qpos[model.jnt_qposadr[ids.j2]],
              "target", target[f"{side}_j1"], target[f"{side}_j2"])


if __name__ == "__main__":
    main()
