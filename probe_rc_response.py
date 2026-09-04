"""Verify RC drive response: forward 1.2 m/s and yaw 0.3 rad/s references."""
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

end = data.time + 3.0
while data.time < end:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)

start = data.xpos[controller.base].copy()
_, yaw_start = controller._heading(data)
controller.set_drive_command(0.40, 0.0)
last = 0.0
max_pitch = 0.0
while data.time < end + 2.0:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    max_pitch = max(max_pitch, abs(float(np.degrees(controller.body_pitch(data)))))
    if data.time - last >= 0.5:
        last = data.time
        disp = data.xpos[controller.base] - start
        fwd = -float(np.dot(disp, controller.forward_world))
        print(
            f"[0.40] t={data.time:.1f} fwd={fwd:+.3f} vx={controller.chassis_vx:+.3f} "
            f"pitch={np.degrees(controller.body_pitch(data)):+.2f}"
        )
print(f"[0.40] forward 2s: {fwd:+.3f} m (drive test expects +0.118)")

controller.stop_drive()
end2 = data.time + 1.0
while data.time < end2:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)

start = data.xpos[controller.base].copy()
controller.set_drive_command(1.40, 0.0)
last = 0.0
max_pitch = 0.0
while data.time < end2 + 4.0:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    max_pitch = max(max_pitch, abs(float(np.degrees(controller.body_pitch(data)))))
    if data.time - last >= 0.5:
        last = data.time
        disp = data.xpos[controller.base] - start
        fwd = -float(np.dot(disp, controller.forward_world))
        print(
            f"[1.20] t={data.time:.1f} fwd={fwd:+.3f} vx={controller.chassis_vx:+.3f} "
            f"pitch={np.degrees(controller.body_pitch(data)):+.2f}"
        )
print(f"[1.20] forward 4s: {fwd:+.3f} m, max_pitch={max_pitch:.2f} deg")

controller.stop_drive()
end2 = data.time + 1.0
while data.time < end2:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)

controller.set_drive_command(0.0, 0.30)
last = 0.0
max_pitch = 0.0
while data.time < end2 + 2.0:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    max_pitch = max(max_pitch, abs(float(np.degrees(controller.body_pitch(data)))))
_, yaw_end = controller._heading(data)
yaw_delta = controller._wrap_pi(yaw_end - yaw_start)
print(f"yaw 2s: delta={np.degrees(yaw_delta):+.1f} deg, max_pitch={max_pitch:.2f} deg")
