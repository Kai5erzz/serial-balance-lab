"""Run the real recovery chain (encoder zero -> recovery -> LQR handoff),
then measure drift / drive behavior exactly as the user sees it."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import TraditionalLQR, apply_encoder_zero_pose

MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    mujoco.mj_resetData(model, data)
    controller.reset(data, calibrate_frame=True)
    apply_encoder_zero_pose(model, data)
    controller.begin_encoder_zero_recovery(data)
    controller.enabled = True
    last = 0.0
    nose = controller.forward_world
    base = controller.base
    start = data.xpos[base].copy()
    while data.time < 15.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 1.0:
            last = data.time
            disp = data.xpos[base] - start
            print(
                f"t={data.time:.2f} recovery={controller.recovery_state} gain={controller.lqr_gain:.2f} "
                f"nose_dist={np.dot(disp, nose):+.3f} vx={controller.chassis_vx:+.3f} "
                f"x={controller.lqr_virtual_x:+.3f} phi0={np.degrees(controller.vmc_kinematics(data, 'l').phi0):+.1f} "
                f"l0={controller.vmc_kinematics(data, 'l').l0:.3f} "
                f"pitch={np.degrees(controller.body_pitch(data)):+.2f} "
                f"out={controller.last_output['l'][0]:+.3f}"
            )


if __name__ == "__main__":
    main()
