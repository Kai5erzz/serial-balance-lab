"""Determine the MuJoCo wheel-torque sign in the calibrated vehicle frame."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import TraditionalLQR, apply_code_zero_pose


MODEL = REPO_ROOT / "model" / "whole_body_lqr.xml"
PULSE_TORQUE_NM = 0.15
PULSE_SECONDS = 0.25


def displacement_for_pattern(left: float, right: float) -> float:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    apply_code_zero_pose(model, data)
    controller = TraditionalLQR(model)
    controller.reset(data)
    controller.enabled = False
    start = data.xpos[controller.base].copy()
    forward = controller._frame(data)[1].copy()
    steps = round(PULSE_SECONDS / model.opt.timestep)
    for _ in range(steps):
        controller.step(data, model.opt.timestep)
        data.ctrl[controller.sides["l"].wheel_motor] = left
        data.ctrl[controller.sides["r"].wheel_motor] = right
        mujoco.mj_step(model, data)
    return float(np.dot(data.xpos[controller.base] - start, forward))


def main() -> None:
    source_pattern = displacement_for_pattern(-PULSE_TORQUE_NM, PULSE_TORQUE_NM)
    inverse_pattern = displacement_for_pattern(PULSE_TORQUE_NM, -PULSE_TORQUE_NM)
    chosen = "left=-T, right=+T" if source_pattern > inverse_pattern else "left=+T, right=-T"
    print(f"source firmware pattern displacement: {source_pattern:+.6f} m")
    print(f"inverse pattern displacement:         {inverse_pattern:+.6f} m")
    print(f"simulation forward-driving pattern: {chosen}")


if __name__ == "__main__":
    main()
