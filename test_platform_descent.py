"""Platform descent test: start on the 600 mm platform center, drive forward
and try to walk down the three 200 mm steps to the ground.

Reports contact/hover states and final rest pose; a crash is recorded when
pitch or roll exceeds the stand basin or the chassis touches non-wheel geoms.
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
TARGET_SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 0.60
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

model = mujoco.MjModel.from_xml_path(str(MODEL))
data = mujoco.MjData(model)
controller = TraditionalLQR(model)
apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
# put the chassis on the platform top (top face at z=0.6)
data.qpos[2] += 0.6
mujoco.mj_forward(model, data)
controller.reset(data, calibrate_frame=True)
controller.enable_calibrated_lqr(data)

platform_names = {
    "platform_top", "platform_step1", "platform_step2", "platform_step3",
}
print(f"start: z={data.xpos[controller.base,2]:.3f} on platform top center")

# settle on the top
end = data.time + 2.0
while data.time < end:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
print(f"settled: z={data.xpos[controller.base,2]:.3f} vx={controller.chassis_vx:+.3f}")

controller.set_drive_command(TARGET_SPEED, 0.0)
last = 0.0
max_pitch = 0.0
max_roll = 0.0
hover_start = None
events = []
ground_time = None
start_pos = data.xpos[controller.base].copy()
crash = None
while data.time < end + DURATION:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    pitch = abs(float(np.degrees(controller.body_pitch(data))))
    roll = abs(float(np.degrees(controller.body_roll(data))))
    max_pitch = max(max_pitch, pitch)
    max_roll = max(max_roll, roll)
    contacts = set()
    wheel_contact = False
    for i in range(data.ncon):
        c = data.contact[i]
        for g in (c.geom1, c.geom2):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
            if name == "fast_collision_l_wheel" or name == "fast_collision_r_wheel":
                wheel_contact = True
            elif name in platform_names:
                contacts.add(name)
            elif name in ("fast_collision_base_link", "recovery_skid"):
                if crash is None:
                    crash = f"chassis grounded at t={data.time:.2f} ({name})"
    if not wheel_contact and hover_start is None and data.time > 2.5:
        hover_start = data.time
    if wheel_contact and hover_start is not None:
        events.append((hover_start, data.time, sorted(contacts)))
        hover_start = None
    if wheel_contact and contacts == {"platform_top"} and data.time > 3.0:
        if ground_time is None:
            ground_time = data.time
    if data.time - last >= 0.5:
        last = data.time
        z = data.xpos[controller.base, 2]
        fn = getattr(controller, "offground_fn_filtered", None)
        fn_s = (
            f" fn_l={fn['l']:.0f} fn_r={fn['r']:.0f}"
            if fn else ""
        )
        print(
            f"t={data.time:.2f} z={z:.3f} vx={controller.chassis_vx:+.3f} "
            f"pitch={np.degrees(controller.body_pitch(data)):+.2f} "
            f"off={int(getattr(controller, 'off_ground', False))} "
            f"gbuf={getattr(controller, 'gbuf_phase', '?')}"
            f"{fn_s} wheels={int(wheel_contact)} on={sorted(contacts)}"
        )
    if pitch > 30.0 or roll > 30.0:
        crash = crash or f"overturned at t={data.time:.2f} (pitch={pitch:.0f})"
        break

controller.stop_drive()
disp = data.xpos[controller.base] - start_pos
fwd = -float(np.dot(disp, controller.forward_world))
print("\n=== summary ===")
print(f"final z={data.xpos[controller.base,2]:.3f}  fwd_travel={fwd:+.2f} m")
print(f"max_pitch={max_pitch:.2f} deg  max_roll={max_roll:.2f} deg")
print(f"hover episodes: {[(f'{a:.2f}-{b:.2f}', c) for a, b, c in events]}")
print(f"on platform_top until: {ground_time}")
print(f"CRASH: {crash}" if crash else "OK: no crash")