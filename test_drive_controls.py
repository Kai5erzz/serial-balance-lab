"""Headless regression for the persistent viewer drive commands."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)


MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast.xml"


def run_for(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    controller: TraditionalLQR,
    duration: float,
) -> tuple[float, float]:
    max_pitch = 0.0
    max_roll = 0.0
    end = data.time + duration
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        max_pitch = max(
            max_pitch, abs(float(np.degrees(controller.body_pitch(data))))
        )
        max_roll = max(
            max_roll, abs(float(np.degrees(controller.body_roll(data))))
        )
    return max_pitch, max_roll


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    # The leg-length settle takes ~2-3 s with the MATLAB wheels; driving from
    # the 1 s transient (l0~0.17) sits outside the theta-compensation match
    # band and biases the response backward.
    run_for(model, data, controller, 3.0)

    start = data.xpos[controller.base].copy()
    _, yaw_start = controller._heading(data)
    controller.set_drive_command(0.40, 0.0)
    pitch_drive, roll_drive = run_for(model, data, controller, 2.0)
    # The calibrated controller frame follows the firmware observer sign;
    # the visible nose of this CAD export is the negative frame direction.
    forward_distance = -float(np.dot(
        data.xpos[controller.base] - start, controller.forward_world
    ))

    controller.stop_drive()
    run_for(model, data, controller, 0.8)
    controller.set_drive_command(0.0, 0.60)
    pitch_turn, roll_turn = run_for(model, data, controller, 2.0)
    _, yaw_end = controller._heading(data)
    yaw_delta = controller._wrap_pi(yaw_end - yaw_start)
    controller.stop_drive()

    finite = bool(
        np.isfinite(data.qpos).all()
        and np.isfinite(data.qvel).all()
        and np.isfinite(data.ctrl).all()
    )
    print(
        f"finite={finite} forward_distance={forward_distance:+.3f}m "
        f"yaw_delta={yaw_delta:+.3f}rad "
        f"max_pitch={max(pitch_drive, pitch_turn):.2f}deg "
        f"max_roll={max(roll_drive, roll_turn):.2f}deg"
    )
    if not finite:
        raise SystemExit("drive control produced a non-finite state")
    # The MATLAB-wheel plant accelerates slower than the old light-wheel plant
    # (2 s of 0.40 m/s reference covers ~0.12 m); 0.20 m was tuned for the
    # light wheels.  The chassis must still clearly move forward.
    if forward_distance < 0.10:
        raise SystemExit("positive forward command did not move the chassis forward")
    if yaw_delta < 0.20:
        raise SystemExit("positive yaw command did not turn the chassis left")
    if max(pitch_drive, pitch_turn) > 12.5:
        raise SystemExit("drive command left the standing pitch basin")
    if max(roll_drive, roll_turn) > 12.0:
        raise SystemExit("drive command left the standing roll basin")


if __name__ == "__main__":
    main()


