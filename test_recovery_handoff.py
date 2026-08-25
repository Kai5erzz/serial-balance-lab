"""Headless verification of recovery settle, LQR handoff, and stand hold."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import (
    RECOVERY_CROUCH_POSE_PATH,
    TraditionalLQR,
    apply_code_zero_pose,
    apply_saved_pose,
)


MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr.xml"
RUN_SECONDS = 5.0


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

    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    start_height = float(data.xpos[base, 2])
    peak_pitch = 0.0
    max_hip_torque = 0.0
    while data.time < RUN_SECONDS:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        peak_pitch = max(peak_pitch, abs(float(np.degrees(controller.body_pitch(data)))))
        max_hip_torque = max(max_hip_torque, *(abs(value) for value in controller.last_motor_torque["l"]), *(abs(value) for value in controller.last_motor_torque["r"]))

    pitch = float(np.degrees(controller.body_pitch(data)))
    finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    print(f"finite={finite}; recovery_state={controller.recovery_state}; lqr_gain={controller.lqr_gain:.3f}")
    print(f"pitch_deg={pitch:+.3f}; peak_pitch_deg={peak_pitch:.3f}")
    print(f"base_height_m={data.xpos[base, 2]:.4f}; rise_m={data.xpos[base, 2] - start_height:.4f}; contacts={data.ncon}")
    print(f"max_hip_torque_Nm={max_hip_torque:.3f}")
    if not finite or controller.recovery_state != "stand" or abs(pitch) > 1.0 or peak_pitch > 5.0 or data.ncon < 2 or data.xpos[base, 2] - start_height < .04 or max_hip_torque < .5:
        raise SystemExit("Recovery handoff did not reach a valid standing hold")


if __name__ == "__main__":
    main()
