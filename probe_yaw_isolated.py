"""Isolated yaw test: settle, then pure yaw 0.30 for 2 s, check direction."""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, r"D:\User\balance_infantry\mujoco_lqr_whole_body")
from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)

MODEL = Path(r"D:\User\balance_infantry\mujoco_lqr_whole_body\model\whole_body_lqr_fast.xml")
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

_, yaw_start = controller._heading(data)
print(f"start vx={controller.chassis_vx:+.3f} yaw_rate={controller.yaw_rate:+.3f}")
controller.set_drive_command(0.0, 0.30)
last = 0.0
while data.time < end + 2.0:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    if data.time - last >= 0.5:
        last = data.time
        print(f"t={data.time:.1f} yaw_rate={controller.yaw_rate:+.3f} "
              f"yaw_out={controller.yaw_output:+.3f} vx={controller.chassis_vx:+.3f} "
              f"ctrl_l={data.ctrl[controller.sides['l'].wheel_motor]:+.3f} "
              f"ctrl_r={data.ctrl[controller.sides['r'].wheel_motor]:+.3f} "
              f"out_l={controller.last_output['l'][0]:+.3f} "
              f"out_r={controller.last_output['r'][0]:+.3f}")
_, yaw_end = controller._heading(data)
yaw_delta = controller._wrap_pi(yaw_end - yaw_start)
print(f"yaw_delta 2s = {np.degrees(yaw_delta):+.1f} deg (positive = turn left)")
print(f"expected for +0.30 rad/s ref over 2 s: about +34 deg CCW")