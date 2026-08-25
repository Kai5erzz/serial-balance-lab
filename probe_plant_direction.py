"""Pure plant probe: fixed common-mode wheel torque -> chassis direction."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast.xml"


def run(torque: float) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    free = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    qadr = model.jnt_qposadr[free]
    wl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "l_wheel_motor")
    wr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_wheel_motor")
    # stand on wheels at rest, all outputs zero
    data.ctrl[:] = 0.0
    for _ in range(500):
        mujoco.mj_step(model, data)
    start = data.xpos[base].copy()
    q0 = data.qpos.copy()
    # left wheel gets -T, right wheel gets +T (the LQR common-mode pattern)
    data.ctrl[wl] = -torque
    data.ctrl[wr] = +torque
    last = data.time
    while data.time < 2.0:
        mujoco.mj_step(model, data)
        if data.time - last >= 0.5:
            last = data.time
            disp = data.xpos[base] - start
            print(
                f"  t={data.time:.2f} disp=({disp[0]:+.3f},{disp[1]:+.3f},{disp[2]:+.3f}) "
                f"wheel_q_l/r={data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'l_wheel')]]:+.3f}/"
                f"{data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'r_wheel')]]:+.3f}"
            )


if __name__ == "__main__":
    for t in (1.0, -1.0):
        print(f"common-mode torque {t:+.1f} Nm (left -T, right +T):")
        run(t)
