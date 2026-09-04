"""Plant direction with legs held by the controller, wheel torque overridden."""

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


def run(torque: float) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    wl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "l_wheel_motor")
    wr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_wheel_motor")
    base = controller.base
    # settle with LQR (legs held at 90 deg)
    end = data.time + 1.0
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    start = data.xpos[base].copy()
    nose = controller.forward_world
    print(f"  nose={nose}")
    last = 0.0
    while data.time < 2.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        data.ctrl[wl] = -torque  # override wheel torques
        data.ctrl[wr] = +torque
        mujoco.mj_step(model, data)
        if data.time - last >= 0.5:
            last = data.time
            disp = data.xpos[base] - start
            print(
                f"    t={data.time:.2f} disp=({disp[0]:+.3f},{disp[1]:+.3f}) "
                f"nose_dist={np.dot(disp, nose):+.3f} "
                f"vx={controller.chassis_vx:+.3f} phi0={np.degrees(controller.vmc_kinematics(data, 'l').phi0):+.1f} "
                f"l0={controller.vmc_kinematics(data, 'l').l0:.3f}"
            )


if __name__ == "__main__":
    for t in (1.0, -1.0):
        print(f"=== fixed wheel torque {t:+.1f} Nm (l=-T, r=+T) with legs held ===")
        run(t)
