"""Detailed pose telemetry for t=1 x=0 xd=0 (theta flipped): stand, drive, stop."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

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

MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast.xml"


def make_state():
    def state(self, data, side: str, dt: float):
        kin = self.vmc_kinematics(data, side)
        length = kin.l0
        pitch = self.body_pitch(data)
        theta = np.pi / 2.0 - kin.phi0 + pitch
        theta_observed = -(theta - self._theta_compensation(data, length))
        theta_dot_raw = self._wrap_pi(theta - self.previous[f"theta_{side}"]) / dt
        self.theta_rate_lpf[side] = (
            VMC_RATE_LPF_RETAIN * self.theta_rate_lpf[side]
            + (1.0 - VMC_RATE_LPF_RETAIN) * theta_dot_raw
        )
        theta_dot = -self.theta_rate_lpf[side]
        x = self.lqr_virtual_x
        firmware_vx_reference = -self.drive_forward_target
        x_dot = self.chassis_vx - firmware_vx_reference
        pitch_dot = (pitch - self.previous["pitch"]) / dt
        return np.array((theta_observed, theta_dot, x, x_dot, -pitch, -pitch_dot)), length

    return state


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    controller._state = types.MethodType(make_state(), controller)
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    base = controller.base
    wheel_body = {side: ids.wheel_body for side, ids in controller.sides.items()}

    def report(tag: str) -> None:
        kin_l = controller.vmc_kinematics(data, "l")
        contact = set()
        for i in range(data.ncon):
            c = data.contact[i]
            if ground not in (c.geom1, c.geom2):
                continue
            other = c.geom2 if c.geom1 == ground else c.geom1
            body = int(model.geom_bodyid[other])
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or f"#{body}"
            contact.add(name)
        st = controller.last_state["l"]
        print(
            f"{tag} t={data.time:.2f} pitch={np.degrees(controller.body_pitch(data)):+.2f} "
            f"phi0={np.degrees(kin_l.phi0):+.2f} l0={kin_l.l0:.3f} "
            f"theta_obs={np.degrees(st[0]):+.2f} xd={st[3]:+.3f} out={controller.last_output['l'][0]:+.3f} "
            f"contact={sorted(contact)}"
        )

    end = data.time + 2.0
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    report("hold")
    controller.set_drive_command(0.30, 0.0)
    end = data.time + 3.0
    last = 0.0
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 0.75:
            last = data.time
            report("drive")
    controller.stop_drive()
    end = data.time + 2.0
    last = 0.0
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 0.75:
            last = data.time
            report("stop")


if __name__ == "__main__":
    main()
