"""Verify the firmware VMC torque map moves both CAD legs in the expected sense."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import TraditionalLQR, apply_code_zero_pose


MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr.xml"
TEST_FORCE_N = 50.0
TEST_SECONDS = 0.20


def run(force: float) -> tuple[dict[str, float], dict[str, float], dict[str, np.ndarray]]:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    apply_code_zero_pose(model, data)
    controller = TraditionalLQR(model)
    controller.reset(data)
    initial = {side: controller.vmc_kinematics(data, side).l0 for side in ("l", "r")}
    while data.time < TEST_SECONDS:
        data.ctrl[:] = 0.0
        data.qfrc_applied[:] = 0.0
        for side in ("l", "r"):
            controller.set_vmc_motor_torques(data, side, force, 0.0)
        mujoco.mj_step(model, data)
    final = {side: controller.vmc_kinematics(data, side).l0 for side in ("l", "r")}
    return initial, final, controller.last_motor_torque


def main() -> None:
    for force in (TEST_FORCE_N, -TEST_FORCE_N):
        initial, final, torque = run(force)
        delta = {side: final[side] - initial[side] for side in ("l", "r")}
        print(f"F0={force:+.1f} N  delta_l0={delta['l']:+.5f}/{delta['r']:+.5f} m")
        print(f"motor_tau_left={np.array2string(torque['l'], precision=3)}  motor_tau_right={np.array2string(torque['r'], precision=3)}")


if __name__ == "__main__":
    main()
