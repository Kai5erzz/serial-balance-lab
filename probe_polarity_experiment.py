"""Polarity experiment: compare x/x_dot state sign variants.

Variants:
  base : current code (x_dot = vx - ref)
  flip_v : x_dot = ref - vx
  flip_both : x = -x_obs, x_dot = ref - vx

Metrics per variant:
  drive: command +0.30 m/s for 4 s -> nose-distance and final speed.
  balance: 3 N s forward push -> chassis drift after 3 s and max pitch.
"""

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
        state = np.array((theta_observed, theta_dot, x, x_dot, -pitch, -pitch_dot))
        return state, length

    return state


def new_controller(flip_v: bool, flip_x: bool) -> tuple[mujoco.MjModel, mujoco.MjData, TraditionalLQR]:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    controller._state = types.MethodType(make_state(flip_v, flip_x), controller)
    return model, data, controller


def settle(model, data, controller, seconds: float) -> None:
    end = data.time + seconds
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)


def run_drive(model, data, controller) -> None:
    settle(model, data, controller, 1.0)
    nose = controller.forward_world
    start = data.xpos[controller.base].copy()
    controller.set_drive_command(0.30, 0.0)
    last = 0.0
    speeds = []
    while data.time < 5.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 0.5:
            last = data.time
            speeds.append(controller.chassis_vx)
    disp = data.xpos[controller.base] - start
    nose_dist = float(np.dot(disp, nose))
    controller.stop_drive()
    settle(model, data, controller, 1.0)
    print(f"  drive +0.30: nose_dist={nose_dist:+.3f} m  vx={speeds}")


def run_balance(model, data, controller) -> None:
    settle(model, data, controller, 1.0)
    start = data.xpos[controller.base].copy()
    nose = controller.forward_world
    base = controller.base
    for _ in range(20):
        mujoco.mj_applyFT(
            model, data, 30.0 * nose, np.zeros(3), data.xpos[base], base,
            data.qfrc_applied,
        )
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    max_pitch = 0.0
    last = 0.0
    while data.time < 3.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        max_pitch = max(max_pitch, abs(float(np.degrees(controller.body_pitch(data)))))
    disp = data.xpos[controller.base] - start
    nose_dist = float(np.dot(disp, nose))
    print(f"  balance push: drift={nose_dist:+.3f} m  max_pitch={max_pitch:.2f} deg  vx={controller.chassis_vx:+.3f}")


if __name__ == "__main__":
    for name, fv, fx in (("base", False, False), ("flip_v", True, False), ("flip_both", True, True)):
        print(f"--- {name} (flip_v={fv}, flip_x={fx}) ---")
        model, data, controller = new_controller(fv, fx)
        run_drive(model, data, controller)
        model, data, controller = new_controller(fv, fx)
        run_balance(model, data, controller)
