"""Detailed telemetry for the flip_both variant during drive +0.30."""

from __future__ import annotations

import types
from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    VMC_RATE_LPF_RETAIN,
    TraditionalLQR,
    apply_saved_pose,
)

MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast.xml"


def make_state(flip_v: bool, flip_x: bool):
    def state(self, data, side: str, dt: float):
        kin = self.vmc_kinematics(data, side)
        length = kin.l0
        pitch = self.body_pitch(data)
        theta = np.pi / 2.0 - kin.phi0 + pitch
        theta_observed = theta - self._theta_compensation(data, length)
        theta_dot_raw = self._wrap_pi(theta - self.previous[f"theta_{side}"]) / dt
        self.theta_rate_lpf[side] = (
            VMC_RATE_LPF_RETAIN * self.theta_rate_lpf[side]
            + (1.0 - VMC_RATE_LPF_RETAIN) * theta_dot_raw
        )
        theta_dot = self.theta_rate_lpf[side]
        x = self.lqr_virtual_x
        firmware_vx_reference = -self.drive_forward_target
        if flip_v:
            x_dot = firmware_vx_reference - self.chassis_vx
        else:
            x_dot = self.chassis_vx - firmware_vx_reference
        pitch_dot = (pitch - self.previous["pitch"]) / dt
        if flip_x:
            x = -x
        return np.array((theta_observed, theta_dot, x, x_dot, -pitch, -pitch_dot)), length

    return state


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    controller._state = types.MethodType(make_state(True, True), controller)
    end = data.time + 1.0
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    nose = controller.forward_world
    start = data.xpos[controller.base].copy()
    controller.set_drive_command(0.30, 0.0)
    last = 0.0
    while data.time < 4.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 0.25:
            last = data.time
            disp = data.xpos[controller.base] - start
            st = controller.last_state["l"]
            kin = controller.vmc_kinematics(data, "l")
            print(
                f"t={data.time:.2f} nose_dist={np.dot(disp, nose):+.3f} "
                f"vx={controller.chassis_vx:+.3f} "
                f"out={controller.last_output['l'][0]:+.3f} "
                f"state=(t{np.degrees(st[0]):+.2f}, td{st[1]:+.3f}, x{st[2]:+.3f}, "
                f"xd{st[3]:+.3f}, p{np.degrees(st[4]):+.2f}, pd{st[5]:+.2f}) "
                f"phi0={np.degrees(kin.phi0):+.1f} pitch={np.degrees(controller.body_pitch(data)):+.2f} "
                f"yaw={np.degrees(controller._heading(data)[1]):+.2f}"
            )


if __name__ == "__main__":
    main()
