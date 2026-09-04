"""Brute-force search over wheel-state sign configurations.

Each config flips the sign of the wheel-row state entries {theta, x, x_dot}.
Acceptance:
  drive +0.30, 4 s : nose_dist > +0.5 m, |vx| < 0.7 (forward, bounded)
  balance push 4 s : drift < 0.25 m, |vx| < 0.15
  rest hold 4 s    : drift < 0.1 m
"""

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


def make_state(flip_theta: bool, flip_pitch: bool, flip_x: bool, flip_xd: bool):
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
        pitch_dot = (pitch - self.previous["pitch"]) / dt
        if flip_theta:
            theta_observed = -theta_observed
            theta_dot = -theta_dot
        if flip_pitch:
            pitch = -pitch
            pitch_dot = -pitch_dot
        if flip_x:
            x = -x
        if flip_xd:
            x_dot = -x_dot
        return np.array((theta_observed, theta_dot, x, x_dot, -pitch, -pitch_dot)), length

    return state


def new_controller(ft: bool, fp: bool, fx: bool, fxd: bool):
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    controller._state = types.MethodType(make_state(ft, fp, fx, fxd), controller)
    return model, data, controller


def settle(model, data, controller, seconds: float) -> None:
    end = data.time + seconds
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)


def nose_disp(controller, start, data) -> float:
    return float(np.dot(data.xpos[controller.base] - start, controller.forward_world))


def test_drive(model, data, controller) -> tuple[float, float, float, float]:
    settle(model, data, controller, 1.0)
    start = data.xpos[controller.base].copy()
    controller.set_drive_command(0.30, 0.0)
    last = 0.0
    vx_end = 0.0
    while data.time < 4.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 1.0:
            last = data.time
            vx_end = controller.chassis_vx
    phi0 = float(np.degrees(controller.vmc_kinematics(data, "l").phi0))
    return nose_disp(controller, start, data), vx_end, float(np.degrees(controller.body_pitch(data))), phi0


def test_balance(model, data, controller) -> tuple[float, float, float]:
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
    while data.time < 4.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    phi0 = float(np.degrees(controller.vmc_kinematics(data, "l").phi0))
    return nose_disp(controller, start, data), controller.chassis_vx, phi0


def test_hold(model, data, controller) -> tuple[float, float, float]:
    settle(model, data, controller, 1.0)
    start = data.xpos[controller.base].copy()
    while data.time < 4.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    phi0 = float(np.degrees(controller.vmc_kinematics(data, "l").phi0))
    return nose_disp(controller, start, data), float(np.degrees(controller.body_pitch(data))), phi0


if __name__ == "__main__":
    print(f"{'config':<28} {'drive':<52} {'push':<34} {'hold':<28} verdict")
    for ft in (False, True):
        for fp in (False, True):
            for fx in (False, True):
                for fxd in (False, True):
                    m, d, c = new_controller(ft, fp, fx, fxd)
                    nd, vx, pt, ph = test_drive(m, d, c)
                    m, d, c = new_controller(ft, fp, fx, fxd)
                    pd, pvx, pph = test_balance(m, d, c)
                    m, d, c = new_controller(ft, fp, fx, fxd)
                    hd, hp, hph = test_hold(m, d, c)
                    ok_drive = nd > 0.5 and abs(vx) < 0.7
                    ok_bal = abs(pd) < 0.25 and abs(pvx) < 0.15
                    ok_hold = abs(hd) < 0.10 and abs(hp) < 2.0
                    verdict = "PASS" if (ok_drive and ok_bal and ok_hold) else ""
                    print(
                        f"t={int(ft)} p={int(fp)} x={int(fx)} xd={int(fxd)}   "
                        f"nd={nd:+.2f} vx={vx:+.2f} pt={pt:+.1f} ph={ph:+.1f}  "
                        f"pd={pd:+.2f} pvx={pvx:+.2f} pph={pph:+.1f}  "
                        f"hd={hd:+.3f} hp={hp:+.1f} hph={hph:+.1f}   {verdict}"
                    )
