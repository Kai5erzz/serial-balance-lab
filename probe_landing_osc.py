"""Trace post-landing pitch: moving jump and double jump."""
from __future__ import annotations

import os
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


def trace(label, settle=3.0, forward=0.0, double=False):
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    controller.set_drive_command(forward, 0.0)
    end = data.time + settle
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)

    print(f"\n===== {label} (forward={forward}) =====")
    accepted, reason = controller.begin_jump(data)
    print(f"begin_jump={accepted} ({reason})")
    last = 0.0
    jumps = 0
    punch_times = []
    while data.time < data.time:
        break
    # run up to 12 s, allow double jump at ~7s
    total_end = data.time + 12.0
    second_jump_armed = False
    while data.time < total_end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 0.05:
            last = data.time
            pitch = np.degrees(controller.body_pitch(data))
            print(
                f"t={data.time:.2f} state={controller.jump_state:8s} "
                f"buf={controller.landing_buffer_phase:7s} "
                f"vx={controller.chassis_vx:+.2f} pitch={pitch:+.2f} "
                f"l0={controller.vmc_kinematics(data, 'l').l0:.3f}"
            )
        if double and not second_jump_armed and controller.jump_state == "idle" \
                and data.time > 6.5:
            second_jump_armed = True
            print(f"--- second jump at t={data.time:.2f} ---")
            controller.begin_jump(data)


trace("stationary", forward=0.0)
trace("moving 0.2", forward=0.20)
trace("double jump", forward=0.0, double=True)