"""Render the 1.0 m/s moving jump (the most dynamic passing cell) to MP4.
Fixed world camera; EGL offscreen; streaming encode.
"""
from __future__ import annotations

import os
os.environ.setdefault("MUJOCO_GL", "egl")

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import mujoco
import numpy as np
import imageio.v2 as imageio

import balance_lab.control.jump as _jm
from balance_lab.plant import load_model
from balance_lab.paths import CALIBRATED_STAND_POSE_PATH
from balance_lab.plant.poses import apply_saved_pose
from balance_lab.control.controller import TraditionalLQR

SPEED = 1.0
TRIGGER = 2.0
SPAWN = 5.6
OUT = REPO_ROOT / "out" / "moving_jump_1.0.mp4"

model, _ = load_model("fast")
h_ = model.vis.headlight
h_.ambient[:] = [0.7, 0.7, 0.7]
h_.diffuse[:] = [1.1, 1.1, 1.1]
_jm.JUMP_PRELOAD_LENGTH = 0.117
_jm.JUMP_THRUST_THETA_LQR_GAIN = 0.2

data = mujoco.MjData(model)
c = TraditionalLQR(model)
apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
mujoco.mj_forward(model, data)
c.reset(data, calibrate_frame=True)
hh, _ = c._heading(data)
data.qpos[0] += float(hh[0]) * SPAWN
data.qpos[1] += float(hh[1]) * SPAWN
mujoco.mj_forward(model, data)
c.enable_calibrated_lqr(data)
c.disable_descent_guard = True
c.disable_danger_check = True
c.set_leg_length(0.24)

cam = mujoco.MjvCamera()
cam.elevation = -10.0
cam.azimuth = 135.0
writer = imageio.get_writer(str(OUT), fps=50, codec="libx264",
                            quality=8, pixelformat="yuv420p")
renderer = mujoco.Renderer(model, height=480, width=640)
DT = model.opt.timestep
n = 0


def snap():
    cam.lookat[:] = data.xpos[c.base] + np.array([0, 0, 0.1])
    renderer.update_scene(data, camera=cam)
    writer.append_data(renderer.render().copy())


while data.time < 2.5:
    c.step(data, DT)
    mujoco.mj_step(model, data)
    n += 1
    if n % 40 == 0:
        snap()
c.set_drive_command(SPEED, 0.0)
while data.time < 14.0:
    c.step(data, DT)
    mujoco.mj_step(model, data)
    n += 1
    if float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1])) <= TRIGGER:
        break
    if n % 40 == 0:
        snap()
ok, reason = c.begin_jump(data)
print(f"begin_jump {ok} {reason}")
while c.jump_state != "idle" and data.time < 22.0:
    if c.jump_state in ("preload", "thrust"):
        c.set_drive_command(SPEED, 0.0)
    c.step(data, DT)
    mujoco.mj_step(model, data)
    n += 1
    if n % 10 == 0:            # denser sampling through the flight
        snap()
for _ in range(1500):
    c.step(data, DT)
    mujoco.mj_step(model, data)
    n += 1
    if n % 20 == 0:
        snap()
writer.close()
try:
    renderer.close()
except Exception:
    pass
print(f"wrote {OUT}")
