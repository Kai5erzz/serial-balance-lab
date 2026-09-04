"""Definitive plant-direction test: short common-mode wheel-torque pulse,
measure the very first chassis acceleration (before tipping matters)."""

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


def pulse(torque: float, pulse_steps: int = 12) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    free = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    dadr = model.jnt_dofadr[free]
    wl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "l_wheel_motor")
    wr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_wheel_motor")
    data.ctrl[:] = 0.0
    for _ in range(1000):  # settle on the ground
        mujoco.mj_step(model, data)
    start_pos = data.xpos[base].copy()
    start_vel = data.qvel[dadr:dadr + 6].copy()
    data.ctrl[wl] = -torque
    data.ctrl[wr] = +torque
    for _ in range(pulse_steps):
        mujoco.mj_step(model, data)
    data.ctrl[wl] = 0.0
    data.ctrl[wr] = 0.0
    vel = data.qvel[dadr:dadr + 6].copy()
    dv = vel - start_vel
    # continue coasting for 0.3 s and measure the displacement
    for _ in range(60):
        mujoco.mj_step(model, data)
    disp = data.xpos[base] - start_pos
    print(
        f"torque={torque:+.2f} Nm: dv=({dv[0]:+.4f},{dv[1]:+.4f},{dv[2]:+.4f}) "
        f"disp=({disp[0]:+.4f},{disp[1]:+.4f},{disp[2]:+.4f}) "
        f"wheel_q_l/r={data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'l_wheel')]]:+.3f}/"
        f"{data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'r_wheel')]]:+.3f}"
    )


if __name__ == "__main__":
    for t in (2.0, -2.0, 1.0, -1.0):
        pulse(t)
