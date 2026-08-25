"""Headless pitch-impulse recovery check for the calibrated LQR model."""

from __future__ import annotations

from pathlib import Path
import sys

import mujoco
import numpy as np

from traditional_lqr import (
    RECOVERY_CROUCH_POSE_PATH,
    TraditionalLQR,
    apply_code_zero_pose,
    apply_saved_pose,
)


MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr.xml"
SETTLE_SECONDS = 1.0
PULSE_START_S = 1.5
PULSE_DURATION_S = 0.10
PULSE_TORQUE_NM = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
END_SECONDS = 4.5


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    apply_code_zero_pose(model, data)
    controller = TraditionalLQR(model)
    controller.reset(data)
    controller.capture_recovery_target(data)
    apply_saved_pose(model, data, RECOVERY_CROUCH_POSE_PATH)
    controller.reset(data)
    controller.begin_recovery(data)

    timeline: list[tuple[float, float, float, float]] = []
    max_wheel_torque = 0.0
    while data.time < END_SECONDS:
        controller.step(data, model.opt.timestep)
        if PULSE_START_S <= data.time < PULSE_START_S + PULSE_DURATION_S:
            _, _, lateral = controller._frame(data)
            mujoco.mj_applyFT(
                model, data, np.zeros(3), PULSE_TORQUE_NM * lateral,
                data.xpos[controller.base], controller.base, data.qfrc_applied,
            )
        mujoco.mj_step(model, data)
        pitch = float(np.degrees(controller.body_pitch(data)))
        max_wheel_torque = max(max_wheel_torque, *(abs(value) for value in data.ctrl[:2]))
        timeline.append((data.time, pitch, float(controller.last_output["l"][0]), float(controller.last_output["r"][0])))

    samples = np.asarray(timeline)
    disturbed = samples[(samples[:, 0] >= PULSE_START_S) & (samples[:, 0] <= PULSE_START_S + 1.0)]
    final = samples[-200:]
    print(f"peak_pitch_after_pulse_deg={np.max(np.abs(disturbed[:, 1])):.4f}")
    print(f"final_pitch_mean_deg={np.mean(final[:, 1]):+.4f}")
    print(f"final_pitch_peak_deg={np.max(np.abs(final[:, 1])):.4f}")
    print(f"max_wheel_torque_Nm={max_wheel_torque:.4f}")
    print(f"base_height_m={data.xpos[controller.base, 2]:.4f}; contacts={data.ncon}")
    print(f"recovery_state={controller.recovery_state}; lqr_gain={controller.lqr_gain:.3f}")


if __name__ == "__main__":
    main()
