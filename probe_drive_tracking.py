"""Measure forward-velocity tracking of the drive command (headless)."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import CALIBRATED_STAND_POSE_PATH, TraditionalLQR, apply_saved_pose

MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast.xml"


def measure(target: float, duration: float = 4.0) -> None:
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
    controller.set_drive_command(target, 0.0)
    samples = []
    end = data.time + duration
    last_t = data.time
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last_t >= 0.5:
            last_t = data.time
            samples.append((controller.chassis_vx, np.degrees(controller.body_pitch(data))))
    dist = -float(np.dot(data.xpos[controller.base] - start, controller.forward_world))
    controller.stop_drive()
    end = data.time + 1.0
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    final_vx = controller.chassis_vx
    print(
        f"target={target:+.2f} m/s  (vx, pitch_deg) samples(0.5s): "
        + " ".join(f"({v:+.3f},{p:+.2f})" for v, p in samples)
        + f"  distance={dist:+.3f} m  vx_after_stop={final_vx:+.3f} m/s"
    )


if __name__ == "__main__":
    for target in (-0.20, 0.20, 0.30, 0.40):
        measure(target)
