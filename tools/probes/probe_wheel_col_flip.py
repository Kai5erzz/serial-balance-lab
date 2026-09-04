"""Flip only the WHEEL-row x/xd columns of the LQR gain (hip row untouched).
Run the recovery chain and measure health (stand, drift, x saturation)."""

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

from traditional_lqr import TraditionalLQR, apply_encoder_zero_pose

MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast.xml"


def patch_gain(controller: TraditionalLQR, fx: bool, fxd: bool) -> None:
    original_gain = controller.gain

    def gain(self, leg_length: float) -> np.ndarray:
        k = original_gain(leg_length).copy()
        if fx:
            k[0, 2] = -k[0, 2]
        if fxd:
            k[0, 3] = -k[0, 3]
        return k

    controller.gain = types.MethodType(gain, controller)


def run(fx: bool, fxd: bool, duration: float = 14.0) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    mujoco.mj_resetData(model, data)
    controller.reset(data, calibrate_frame=True)
    apply_encoder_zero_pose(model, data)
    controller.begin_encoder_zero_recovery(data)
    controller.enabled = True
    patch_gain(controller, fx, fxd)
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
                f"t={data.time:.2f} rec={controller.recovery_state} "
                f"nd={np.dot(disp, nose):+.2f} vx={controller.chassis_vx:+.3f} "
                f"x={controller.lqr_virtual_x:+.2f} phi0={np.degrees(controller.vmc_kinematics(data, 'l').phi0):+.1f} "
                f"l0={controller.vmc_kinematics(data, 'l').l0:.3f} "
                f"pitch={np.degrees(controller.body_pitch(data)):+.2f} out={controller.last_output['l'][0]:+.2f}"
            )


if __name__ == "__main__":
    for fx, fxd in ((False, False), (False, True), (True, False), (True, True)):
        print(f"=== wheel-row flip x={int(fx)} xd={int(fxd)} (hip row untouched) ===")
        run(fx, fxd)
