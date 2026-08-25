"""Recovery chain with x_dot state flipped (negative velocity feedback)."""

from __future__ import annotations

import types
from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import (
    VMC_RATE_LPF_RETAIN,
    TraditionalLQR,
    apply_encoder_zero_pose,
)

MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast.xml"


def make_state(flip_xd: bool, flip_x: bool):
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
        x_dot = self.chassis_vx - firmware_vx_reference
        if flip_xd:
            x_dot = -x_dot
        if flip_x:
            x = -x
        pitch_dot = (pitch - self.previous["pitch"]) / dt
        return np.array((theta_observed, theta_dot, x, x_dot, -pitch, -pitch_dot)), length

    return state


def run(flip_xd: bool, flip_x: bool, duration: float = 14.0, drive: float = 0.0) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    mujoco.mj_resetData(model, data)
    controller.reset(data, calibrate_frame=True)
    apply_encoder_zero_pose(model, data)
    controller.begin_encoder_zero_recovery(data)
    controller.enabled = True
    controller._state = types.MethodType(make_state(flip_xd, flip_x), controller)
    nose = controller.forward_world
    base = controller.base
    start = data.xpos[base].copy()
    last = 0.0
    while data.time < duration:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 2.0:
            last = data.time
            disp = data.xpos[base] - start
            print(
                f"t={data.time:.2f} rec={controller.recovery_state} gain={controller.lqr_gain:.2f} "
                f"nd={np.dot(disp, nose):+.2f} vx={controller.chassis_vx:+.3f} "
                f"x={controller.lqr_virtual_x:+.2f} phi0={np.degrees(controller.vmc_kinematics(data, 'l').phi0):+.1f} "
                f"l0={controller.vmc_kinematics(data, 'l').l0:.3f} "
                f"pitch={np.degrees(controller.body_pitch(data)):+.2f} out={controller.last_output['l'][0]:+.2f}"
            )


if __name__ == "__main__":
    for fxd, fx in ((False, False), (True, False), (False, True), (True, True)):
        print(f"=== flip_xd={fxd} flip_x={fx} ===")
        run(fxd, fx)
