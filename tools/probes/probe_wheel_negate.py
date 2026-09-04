"""Verify the root-cause fix: negate the LQR wheel-row output.

Metrics: drive +0.30 (should move forward and converge), balance push
(should return to rest), rest hold (no drift).
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

from traditional_lqr import CALIBRATED_STAND_POSE_PATH, TraditionalLQR, apply_saved_pose

MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast.xml"


def patch_wheel_negate(controller: TraditionalLQR) -> None:
    """Negate the wheel row of the LQR output (root-cause polarity fix)."""

    def step(self, data, dt: float) -> None:
        raise NotImplementedError

    # Instead of monkey-patching the big step(), patch the gain matrix used by
    # the wheel output by wrapping `gain` and negating row 0.
    original_gain = controller.gain

    def gain(self, leg_length: float) -> np.ndarray:
        k = original_gain(leg_length).copy()
        k[0] = -k[0]
        return k

    controller.gain = types.MethodType(gain, controller)


def new_controller(negate: bool):
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    if negate:
        patch_wheel_negate(controller)
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
    rows = []
    while data.time < 5.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if data.time - last >= 0.5:
            last = data.time
            disp = data.xpos[controller.base] - start
            rows.append(
                f"t={data.time:.2f} nose_dist={np.dot(disp, nose):+.3f} "
                f"vx={controller.chassis_vx:+.3f} pitch={np.degrees(controller.body_pitch(data)):+.2f}"
            )
    controller.stop_drive()
    settle(model, data, controller, 1.5)
    disp = data.xpos[controller.base] - start
    rows.append(f"after stop: nose_dist={np.dot(disp, nose):+.3f} vx={controller.chassis_vx:+.3f}")
    for r in rows:
        print("  " + r)


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
    while data.time < 4.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        max_pitch = max(max_pitch, abs(float(np.degrees(controller.body_pitch(data)))))
    disp = data.xpos[controller.base] - start
    print(
        f"  push: drift={np.dot(disp, nose):+.3f} m  max_pitch={max_pitch:.2f} deg  "
        f"vx={controller.chassis_vx:+.3f}"
    )


if __name__ == "__main__":
    for name, neg in (("base (current)", False), ("wheel-row negated", True)):
        print(f"--- {name} ---")
        model, data, controller = new_controller(neg)
        run_drive(model, data, controller)
        model, data, controller = new_controller(neg)
        run_balance(model, data, controller)
