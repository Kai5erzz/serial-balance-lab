"""Operator-style platform descent: segment the drive like a human driver.

Phase A: drive to ~0.8 m from the platform center (0.2 m before the edge),
release and settle.
Phase B: creep forward 0.25 m/s across the edge until the wheels lose
support; the off-ground module takes over, lands on the next step and
settles.  Repeat per step until the ground floor.
"""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, r"D:\User\balance_infantry\mujoco_lqr_whole_body")
from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)

MODEL = Path(r"D:\User\balance_infantry\mujoco_lqr_whole_body\model\whole_body_lqr_fast_platform.xml")

model = mujoco.MjModel.from_xml_path(str(MODEL))
data = mujoco.MjData(model)
controller = TraditionalLQR(model)
apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
data.qpos[2] += 0.6
# The platform is built with its edges perpendicular to the chassis forward
# frame, so both wheels cross each edge simultaneously.
mujoco.mj_forward(model, data)
controller.reset(data, calibrate_frame=True)
controller.enable_calibrated_lqr(data)

origin = data.xpos[controller.base].copy()
heading, _ = controller._heading(data)


def travel() -> float:
    """Signed forward travel of the base along -heading (UI forward)."""
    disp = data.xpos[controller.base] - origin
    return -float(np.dot(disp, heading))


def run(duration: float) -> None:
    end = data.time + duration
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)


def settle(duration: float) -> None:
    controller.stop_drive()
    run(duration)


def report(tag: str) -> None:
    fn = controller.offground_fn_filtered
    print(
        f"[{tag}] t={data.time:.2f} z={data.xpos[controller.base,2]:.3f} "
        f"trav={travel():+.2f} vx={controller.chassis_vx:+.2f} "
        f"pitch={np.degrees(controller.body_pitch(data)):+.2f} "
        f"off={int(controller.off_ground)} gbuf={controller.gbuf_phase} "
        f"fn=({fn['l']:.0f}/{fn['r']:.0f})"
    )


print("=== platform descent (coast-over-edge, driver-style) ===")
report("start")
run(2.0)
report("settled-top")

# Phase A: accelerate to ~0.85 m (0.15 m before the edge), then release and
# let the chassis coast over the edge with minimal speed (driver lifts the
# throttle when approaching a drop).
controller.set_drive_command(0.40, 0.0)
while travel() < 0.85 and data.time < 15.0:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
controller.stop_drive()
print(f"--- throttle released at trav={travel():+.2f} vx={controller.chassis_vx:+.2f} ---")
end = data.time + 6.0
while data.time < end:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    z = data.xpos[controller.base, 2]
    if controller.gbuf_phase in ("hold", "return") and z < 0.70:
        break
controller.stop_drive()
run(1.5)
report("after-step1")

print("\n=== continue to next steps ===")
for step in range(2, 4):
    controller.set_drive_command(0.30, 0.0)
    end = data.time + 5.0
    while data.time < end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        z = data.xpos[controller.base, 2]
        if z < 0.32 * step and data.time > 5.0:
            break
    controller.stop_drive()
    run(1.0)
    report(f"after-step{step}")

print("\n=== summary ===")
print(f"final z={data.xpos[controller.base,2]:.3f}  travelled={travel():+.2f} m")
print(f"final pitch={np.degrees(controller.body_pitch(data)):+.2f} "
      f"roll={np.degrees(controller.body_roll(data)):+.2f}")
print(f"off_ground={controller.off_ground} gbuf={controller.gbuf_phase}")
ok = (abs(np.degrees(controller.body_pitch(data))) < 8.0
      and abs(np.degrees(controller.body_roll(data))) < 8.0
      and data.xpos[controller.base, 2] < 0.35)
print("RESULT:", "SAFE DESCENT" if ok else "UNSAFE / CRASHED")