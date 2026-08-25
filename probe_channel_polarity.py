"""Channel-isolation polarity test.

Patch gain() to keep only one feedback column, then push the standing robot
along the nose and observe whether that channel brakes (negative feedback) or
accelerates (positive feedback).

channels:
  theta   : col 0 (K=-20.29)
  theta_d : col 1 (K=-4.98)
  x       : col 2 (K=-0.54)
  x_dot   : col 3 (K=-1.66)
  pitch   : col 4 (K=+7.38)
  pitch_d : col 5 (K=+1.34)
"""

from __future__ import annotations

import types
from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import CALIBRATED_STAND_POSE_PATH, TraditionalLQR, apply_saved_pose

MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast.xml"

BASE_K = np.array([
    [-20.293545, -4.977628, -0.539547, -1.661860, 7.379106, 1.341167],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
])


def patch_gain(controller: TraditionalLQR, keep: int | None) -> None:
    def gain(self, leg_length: float) -> np.ndarray:
        k = BASE_K.copy()
        if keep is not None:
            mask = np.zeros(6, dtype=bool)
            mask[keep] = True
            k[0, ~mask] = 0.0
        return k

    controller.gain = types.MethodType(gain, controller)


def run_channel(keep: int | None) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    patch_gain(controller, keep)
    # settle
    end = data.time + 1.0
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    nose = controller.forward_world
    start = data.xpos[controller.base].copy()
    base = controller.base
    for _ in range(20):
        mujoco.mj_applyFT(
            model, data, 30.0 * nose, np.zeros(3), data.xpos[base], base,
            data.qfrc_applied,
        )
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    last = 0.0
    rows = []
    while data.time < 2.5:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 0.5:
            last = data.time
            disp = data.xpos[controller.base] - start
            rows.append(
                f"t={data.time:.2f} nose_dist={np.dot(disp, nose):+.3f} "
                f"vx={controller.chassis_vx:+.3f} out={controller.last_output['l'][0]:+.3f}"
            )
    print(f"--- channel {keep} ---")
    for r in rows:
        print("  " + r)


if __name__ == "__main__":
    for ch in (0, 1, 2, 3, 4, 5, None):
        run_channel(ch)
