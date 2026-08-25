"""Print one-second telemetry snapshots for the calibrated LQR mode."""

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)


model = mujoco.MjModel.from_xml_path(
    str(Path(__file__).resolve().parent / "model" / "whole_body_lqr.xml")
)
data = mujoco.MjData(model)
controller = TraditionalLQR(model)
apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
controller.reset(data, calibrate_frame=True)
controller.enable_calibrated_lqr(data)

print("t pitch roll phiL phiR lL lR thL thR x vx Tw TpL TpR F0L F0R")
for index in range(15_000):
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    if (index + 1) % 1_000:
        continue
    left = controller.vmc_kinematics(data, "l")
    right = controller.vmc_kinematics(data, "r")
    state_l = controller.last_state["l"]
    state_r = controller.last_state["r"]
    output_l = controller.last_output["l"]
    output_r = controller.last_output["r"]
    print(
        f"{data.time:.0f} "
        f"{np.degrees(controller.body_pitch(data)):+.3f} "
        f"{np.degrees(controller.body_roll(data)):+.3f} "
        f"{np.degrees(left.phi0):+.2f} {np.degrees(right.phi0):+.2f} "
        f"{left.l0:.4f} {right.l0:.4f} "
        f"{np.degrees(state_l[0]):+.2f} {np.degrees(state_r[0]):+.2f} "
        f"{state_l[2]:+.3f} {state_l[3]:+.3f} "
        f"{output_l[0]:+.2f} {output_l[1]:+.2f} {output_r[1]:+.2f} "
        f"{controller.last_support['l']:+.1f} "
        f"{controller.last_support['r']:+.1f}"
    )
