"""Decompose the LQR wheel output into per-state contributions after the
recovery handoff, to locate what drives the steady backward cruise."""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from traditional_lqr import TraditionalLQR, apply_encoder_zero_pose

MODEL = ROOT / "model" / "whole_body_lqr_fast.xml"

model = mujoco.MjModel.from_xml_path(str(MODEL))
data = mujoco.MjData(model)
controller = TraditionalLQR(model)
mujoco.mj_resetData(model, data)
controller.reset(data, calibrate_frame=True)
apply_encoder_zero_pose(model, data)
controller.begin_encoder_zero_recovery(data)
controller.enabled = True

last = 0.0
while data.time < 13.0:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    if data.time - last >= 1.0:
        last = data.time
        if controller.recovery_state != "fw_stand":
            continue
        state, length = controller._state(data, "l", model.opt.timestep)
        k = controller.gain(length)[0]
        contributions = {
            "theta_obs": float(k[0] * state[0]),
            "theta_dot": float(k[1] * state[1]),
            "x": float(k[2] * state[2]),
            "x_dot": float(k[3] * state[3]),
            "-pitch": float(k[4] * state[4]),
            "-pitch_dot": float(k[5] * state[5]),
        }
        print(
            f"t={data.time:.1f} vx={controller.chassis_vx:+.3f} "
            f"x={state[2]:+.3f} xd={state[3]:+.3f} "
            f"theta_obs={np.degrees(state[0]):+.2f}deg "
            f"pitch={np.degrees(state[4]):+.2f}deg "
            f"l0={length:.3f} phi0={np.degrees(controller.vmc_kinematics(data, 'l').phi0):+.1f}"
        )
        print(
            "   K[0]="
            + " ".join(f"{v:+.2f}" for v in k)
            + "  contrib="
            + " ".join(f"{n}={v:+.3f}" for n, v in contributions.items())
            + f"  out={controller.last_output['l'][0]:+.3f}"
        )
