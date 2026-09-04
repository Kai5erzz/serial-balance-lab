"""Regression test for a complete, attitude-safe platform descent.

The controller is deliberately driven with one persistent command.  The
descent guard must detect the box-terrain edge, brake before the drop, absorb
the landing, and leave the vehicle upright on the surrounding ground plane.
The test records peak attitude over every physics step, not only the final
pose.
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import sys
import os
from pathlib import Path

import mujoco
import numpy as np

ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT))
from traditional_lqr import (  # noqa: E402
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)


MODEL = ROOT / "model" / "whole_body_lqr_fast_platform.xml"
TARGET_SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 22.0
LEG_LENGTH_TARGET = os.environ.get("LEG_LENGTH_TARGET")
SAFE_PITCH_DEG = 20.0
SAFE_ROLL_DEG = 12.0
GROUND_Z_MAX = 0.25
MIN_TRAVEL = 2.0


def main() -> int:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    # The first downward tread has its top at z=0.6.
    data.qpos[2] += 0.6
    mujoco.mj_forward(model, data)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    if LEG_LENGTH_TARGET is not None:
        controller.set_leg_length(float(LEG_LENGTH_TARGET))

    for _ in range(int(np.ceil(2.0 / model.opt.timestep))):
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)

    origin = data.xpos[controller.base].copy()
    heading = controller.forward_world.copy()
    controller.set_drive_command(TARGET_SPEED, 0.0)

    max_pitch = 0.0
    max_roll = 0.0
    max_speed = 0.0
    chassis_contact: str | None = None
    nonfinite = False
    last_report = data.time
    end_time = data.time + DURATION

    while data.time < end_time:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)

        pitch = float(np.degrees(controller.body_pitch(data)))
        roll = float(np.degrees(controller.body_roll(data)))
        ui_speed = -float(controller.chassis_vx)
        max_pitch = max(max_pitch, abs(pitch))
        max_roll = max(max_roll, abs(roll))
        max_speed = max(max_speed, abs(ui_speed))
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            nonfinite = True

        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            names = [
                mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
                )
                for geom_id in (contact.geom1, contact.geom2)
            ]
            for name in ("fast_collision_base_link", "recovery_skid"):
                if name in names and chassis_contact is None:
                    chassis_contact = f"{name}@t={data.time:.2f}"

        if data.time - last_report >= 1.0:
            last_report = data.time
            print(
                f"t={data.time:.2f} z={data.xpos[controller.base, 2]:.3f} "
                f"travel={np.dot(data.xpos[controller.base] - origin, heading):+.2f} "
                f"ui_vx={ui_speed:+.2f} pitch={pitch:+.2f} roll={roll:+.2f} "
                f"guard={int(controller.descent_guard_active)} "
                f"off={int(controller.off_ground)} gbuf={controller.gbuf_phase}"
            )

    controller.stop_drive()
    travel = float(np.dot(data.xpos[controller.base] - origin, heading))
    final_z = float(data.xpos[controller.base, 2])
    safe = (
        not nonfinite
        and chassis_contact is None
        and max_pitch <= SAFE_PITCH_DEG
        and max_roll <= SAFE_ROLL_DEG
        and travel >= MIN_TRAVEL
        and final_z <= GROUND_Z_MAX
    )

    print("\n=== safe-descent summary ===")
    print(
        f"final z={final_z:.3f} travel={travel:+.2f} m "
        f"max_pitch={max_pitch:.2f} deg max_roll={max_roll:.2f} deg "
        f"max_ui_vx={max_speed:.2f} m/s"
    )
    print(f"chassis_contact={chassis_contact or 'none'} finite={not nonfinite}")
    print("RESULT:", "SAFE DESCENT" if safe else "UNSAFE / CRASHED")
    return 0 if safe else 1


if __name__ == "__main__":
    raise SystemExit(main())
