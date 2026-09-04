"""Headless acceptance test: prone CAD-zero -> torque recovery -> stable LQR."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import TraditionalLQR, apply_prone_zero_pose


MODEL = REPO_ROOT / "model" / "whole_body_lqr.xml"


def run(direction: float) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_prone_zero_pose(model, data, controller, direction)
    initial_pitch = float(np.degrees(controller.body_pitch(data)))
    controller.begin_recovery(data)
    pulse_peak = 0.0
    base = controller.base

    for _ in range(16_000):
        controller.step(data, model.opt.timestep)
        if 11.0 <= data.time < 11.1:
            _, _, lateral = controller._frame(data)
            mujoco.mj_applyFT(
                model,
                data,
                np.zeros(3),
                2.0 * lateral,
                data.xpos[base],
                base,
                data.qfrc_applied,
            )
        mujoco.mj_step(model, data)
        if data.time >= 11.0:
            pulse_peak = max(pulse_peak, abs(float(np.degrees(controller.body_pitch(data)))))

    pitch = float(np.degrees(controller.body_pitch(data)))
    lengths = {side: controller.vmc_kinematics(data, side).l0 for side in ("l", "r")}
    print(
        f"direction={direction:+.0f} initial={initial_pitch:+.1f} deg "
        f"finite={np.isfinite(data.qpos).all()} state={controller.recovery_state} "
        f"pitch={pitch:+.3f} deg pulse_peak={pulse_peak:.3f} deg contacts={data.ncon} "
        f"l0={lengths['l']:.6f}/{lengths['r']:.6f} m"
    )
    if not np.isfinite(data.qpos).all():
        raise SystemExit("non-finite physics state")
    if controller.recovery_state != "stand" or not controller.enabled:
        raise SystemExit("recovery did not hand off to LQR standing state")
    if abs(pitch) > 0.5 or pulse_peak > 6.0 or data.ncon < 2:
        raise SystemExit("prone recovery did not finish in a stable standing trim")


def main() -> None:
    # The chassis exterior is asymmetric; both physical face-down starts must
    # use the contact gate successfully rather than relying on one screenshot.
    for direction in (-1.0, 1.0):
        run(direction)


if __name__ == "__main__":
    main()
