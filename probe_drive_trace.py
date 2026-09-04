"""Trace drive command response: vx/x/out during forward command."""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)

MODEL = ROOT / "model" / "whole_body_lqr_fast.xml"

model = mujoco.MjModel.from_xml_path(str(MODEL))
data = mujoco.MjData(model)
controller = TraditionalLQR(model)
apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
controller.reset(data, calibrate_frame=True)
controller.enable_calibrated_lqr(data)

end = data.time + 1.0
while data.time < end:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)

start = data.xpos[controller.base].copy()
controller.set_drive_command(0.40, 0.0)
last = 0.0
while data.time < end + 2.0:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    if data.time - last >= 0.25:
        last = data.time
        state, length = controller._state(data, "l", model.opt.timestep)
        disp = data.xpos[controller.base] - start
        fwd = -float(np.dot(disp, controller.forward_world))
        print(
            f"t={data.time:.2f} fwd_disp={fwd:+.3f} vx={controller.chassis_vx:+.3f} "
            f"x={state[2]:+.3f} xd={state[3]:+.3f} theta_obs={np.degrees(state[0]):+.2f} "
            f"l0={length:.3f} out={controller.last_output['l'][0]:+.3f}"
        )
