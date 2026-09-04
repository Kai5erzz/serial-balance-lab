"""Landing handoff timing + post-landing position: does the robot settle back
at the launch spot? Measures buffer phase duration and base drift after idle.
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT))
from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)

MODEL = ROOT / "model" / "whole_body_lqr_fast.xml"

model = mujoco.MjModel.from_xml_path(str(MODEL))
data = mujoco.MjData(model)
controller = TraditionalLQR(model)
apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
controller.reset(data, calibrate_frame=True)
controller.enable_calibrated_lqr(data)

while data.time < 3.0:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)

launch_pos = data.xpos[controller.base].copy()
print(f"launch: vx={controller.chassis_vx:+.3f} x={controller.lqr_virtual_x:+.3f} l0={controller.vmc_kinematics(data, 'l').l0:.3f}")
# nose direction for forward displacement
_, heading = controller._heading(data)
accepted, reason = controller.begin_jump(data)
print(f"begin_jump={accepted} ({reason})  launch_z={launch_pos[2]:.3f}")

touch_time = None
inactive_time = None
idle_time = None
last_phase = controller.landing_buffer_phase
phase_changes = []
end = data.time + 5.0
while data.time < end:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    if controller.jump_off_ground and touch_time is None:
        takeoff = data.time
    if (not controller.jump_off_ground) and touch_time is None and data.time > 3.0 and controller.jump_state == "landing":
        touch_time = data.time
        print(f"touchdown at t={touch_time:.3f} z={data.xpos[controller.base, 2]:.3f}")
    if controller.landing_buffer_phase != last_phase:
        phase_changes.append((data.time, last_phase, controller.landing_buffer_phase))
        last_phase = controller.landing_buffer_phase
    if controller.landing_buffer_phase == "inactive" and inactive_time is None and data.time > 3.0:
        inactive_time = data.time
    if controller.jump_state == "idle" and idle_time is None and data.time > 3.0:
        idle_time = data.time

disp = data.xpos[controller.base] - launch_pos
forward_disp = float(np.dot(disp, controller.forward_world))
print(f"phase changes: {[(f'{t:.3f}', a, b) for t, a, b in phase_changes]}")
print(f"touch={touch_time:.3f}" if touch_time else "touch=never",
      f"idle={idle_time:.3f}" if idle_time else "idle=never",
      f"buffer_inactive={inactive_time:.3f}" if inactive_time else "inactive=never")
if touch_time and inactive_time:
    print(f"handoff delay (touch->inactive): {inactive_time - touch_time:.3f} s")
kin = controller.vmc_kinematics(data, "l")
theta = np.pi / 2.0 - kin.phi0 + controller.body_pitch(data)
comp = controller._theta_compensation(data, kin.l0)
state, _ = controller._state(data, "l", model.opt.timestep)
print(f"final: z={data.xpos[controller.base,2]:.3f} "
      f"drift={np.linalg.norm(disp[:2]):.4f} m forward={forward_disp:+.4f} m "
      f"vx={controller.chassis_vx:+.3f} x={controller.lqr_virtual_x:+.3f} "
      f"pitch={np.degrees(controller.body_pitch(data)):+.2f}deg")
print(f"pose: l0={kin.l0:.4f} phi0={np.degrees(kin.phi0):+.2f}deg "
      f"theta={np.degrees(theta):+.2f}deg comp={np.degrees(comp):+.2f}deg "
      f"theta_obs={np.degrees(state[0]):+.2f}deg gain={controller.lqr_gain}")
print(f"recovery_state={controller.recovery_state} jump_state={controller.jump_state}")
