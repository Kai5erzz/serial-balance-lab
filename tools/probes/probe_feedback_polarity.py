"""Determine velocity-feedback polarity: disturb the standing robot and log
chassis_vx vs wheel torque; then same during drive command."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import CALIBRATED_STAND_POSE_PATH, TraditionalLQR, apply_saved_pose

MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast.xml"


def log_line(controller, data, tag: str) -> str:
    vx = controller.chassis_vx
    xd = controller.last_state["l"][3]
    out = controller.last_output["l"][0]
    return (
        f"{tag} t={data.time:.2f} chassis_vx={vx:+.3f} x_dot_state={xd:+.3f} "
        f"lqr_out_l={out:+.3f} pitch={np.degrees(controller.body_pitch(data)):+.2f}"
    )


def main() -> None:
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

    # Disturbance: impulse forward along the nose (world frame).
    nose = controller.forward_world
    base = controller.base
    for _ in range(20):
        mujoco.mj_applyFT(
            model, data, 30.0 * nose, np.zeros(3), data.xpos[base], base,
            data.qfrc_applied,
        )
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    print("--- balance recovery after forward push ---")
    last = 0.0
    while data.time < 3.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 0.25:
            last = data.time
            print(log_line(controller, data, "BAL"))

    # Now drive command.
    print("--- drive +0.20 ---")
    controller.set_drive_command(0.20, 0.0)
    last = 0.0
    while data.time < 3.5:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 0.25:
            last = data.time
            print(log_line(controller, data, "DRV"))
    controller.stop_drive()


if __name__ == "__main__":
    main()
