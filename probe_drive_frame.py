"""Raw-frame probe: where does the robot actually go for a forward command?"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import CALIBRATED_STAND_POSE_PATH, TraditionalLQR, apply_saved_pose

MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast.xml"


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
    print(f"forward_body={controller.forward_body}")
    print(f"forward_world={controller.forward_world}")
    print(f"lateral_body={controller.lateral_body}")
    start = data.xpos[controller.base].copy()
    h0, y0 = controller._heading(data)
    print(f"heading0={h0} yaw0={np.degrees(y0):+.2f} deg")
    controller.set_drive_command(0.20, 0.0)
    last_t = data.time
    wheel_motor = {
        side: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, side + "_wheel_motor")
        for side in ("l", "r")
    }
    while data.time < 4.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last_t >= 0.5:
            last_t = data.time
            h, yaw = controller._heading(data)
            disp = data.xpos[controller.base] - start
            state_l = controller.last_state["l"]
            print(
                f"t={data.time:.2f} disp=({disp[0]:+.3f},{disp[1]:+.3f}) "
                f"dot(disp,nose)={np.dot(disp,h):+.3f} yaw={np.degrees(yaw):+.2f} "
                f"chassis_vx={controller.chassis_vx:+.3f} "
                f"state_l=({state_l[0]:+.3f},{state_l[1]:+.3f},{state_l[2]:+.3f},"
                f"{state_l[3]:+.3f},{state_l[4]:+.3f},{state_l[5]:+.3f}) "
                f"wheel_cmd_l/r={data.ctrl[wheel_motor['l']]:+.3f}/{data.ctrl[wheel_motor['r']]:+.3f} Nm "
                f"lqr_out_l={controller.last_output['l'][0]:+.3f} "
                f"pitch={np.degrees(controller.body_pitch(data)):+.2f}"
            )


if __name__ == "__main__":
    main()
