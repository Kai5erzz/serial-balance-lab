"""Base code: does a NEGATIVE drive command move the robot FORWARD (nose+)?"""

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


def run(target: float) -> None:
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
    nose = controller.forward_world
    start = data.xpos[controller.base].copy()
    controller.set_drive_command(target, 0.0)
    last = 0.0
    while data.time < 3.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 0.5:
            last = data.time
            disp = data.xpos[controller.base] - start
            print(
                f"  t={data.time:.2f} nose_dist={np.dot(disp, nose):+.3f} "
                f"vx={controller.chassis_vx:+.3f} out={controller.last_output['l'][0]:+.3f} "
                f"phi0={np.degrees(controller.vmc_kinematics(data, 'l').phi0):+.1f} "
                f"pitch={np.degrees(controller.body_pitch(data)):+.2f}"
            )


if __name__ == "__main__":
    for t in (-0.30, 0.30):
        print(f"=== drive target {t:+.2f} ===")
        run(t)
