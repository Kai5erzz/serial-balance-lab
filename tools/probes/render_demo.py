"""Render two MP4 demos headless via EGL offscreen (small GPU context).

Uses imageio streaming so frames are encoded as produced (constant low
RAM).  Intended to run alongside the RL training: single short process,
modest resolution, one EGL context reused across frames.

Run (in rl-lab env, MUJOCO_GL=egl):
    python tools/probes/render_demo.py stand    # 5 s calibrated-LQR stand
    python tools/probes/render_demo.py jump     # one firmware standing jump
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

from balance_lab.plant import load_model
from balance_lab.paths import CALIBRATED_STAND_POSE_PATH
from balance_lab.plant.poses import apply_saved_pose
from balance_lab.control.controller import TraditionalLQR

WH, WD = 480, 640
DT = model_timestep = None


def brighten(model):
    """The plant defines no lights; lift the built-in headlight so the
    offscreen render is not near-black."""
    h = model.vis.headlight
    h.ambient[:] = [0.7, 0.7, 0.7]
    h.diffuse[:] = [1.1, 1.1, 1.1]
    h.specular[:] = [0.35, 0.35, 0.35]


def make_camera():
    cam = mujoco.MjvCamera()
    cam.elevation = -12.0
    cam.azimuth = 135.0
    return cam


def arm_stand(model, data, c):
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    c.enable_calibrated_lqr(data)
    c.disable_descent_guard = True
    c.disable_danger_check = True


def track(cam, c, data, dist, zlift=0.12):
    cam.lookat[:] = data.xpos[c.base] + np.array([0, 0, zlift])
    cam.distance = dist


def render_stand(out_path: Path):
    model, _ = load_model("fast")
    brighten(model)
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)
    arm_stand(model, data, c)
    cam = make_camera()
    DT = model.opt.timestep
    writer = imageio.get_writer(str(out_path), fps=50, codec="libx264",
                                quality=8, pixelformat="yuv420p")
    renderer = mujoco.Renderer(model, height=WH, width=WD)
    t = 0.0
    n = 0
    while t < 5.5:                      # 0.5 settle + 5.0 balance
        c.step(data, DT)
        mujoco.mj_step(model, data)
        t += DT
        n += 1
        if t >= 0.5 and n % 20 == 0:    # every 20 ms
            track(cam, c, data, dist=1.9)
            renderer.update_scene(data, camera=cam)
            writer.append_data(renderer.render().copy())
    writer.close()
    try:
        renderer.close()
    except Exception:
        pass
    print(f"wrote {out_path}")


def render_jump(out_path: Path):
    model, _ = load_model("fast")
    brighten(model)
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)
    arm_stand(model, data, c)
    cam = make_camera()
    DT = model.opt.timestep
    writer = imageio.get_writer(str(out_path), fps=50, codec="libx264",
                                quality=8, pixelformat="yuv420p")
    renderer = mujoco.Renderer(model, height=WH, width=WD)

    def snap():
        # fixed world camera so the hop is visible on screen
        renderer.update_scene(data, camera=cam)
        writer.append_data(renderer.render().copy())

    t = 0.0
    n = 0
    apex = 0.0
    while t < 1.2:                      # settle into a clean stand
        c.step(data, DT)
        mujoco.mj_step(model, data)
        t += DT
        n += 1
        if n == 1:
            cam.lookat[:] = data.xpos[c.base] + np.array([0, 0, 0.2])
            cam.distance = 2.8
        if n % 20 == 0:
            snap()
    ok, reason = c.begin_jump(data)
    print(f"begin_jump {ok} {reason}")
    guard = 0
    while c.jump_state != "idle" and guard < 4000:
        c.step(data, DT)
        mujoco.mj_step(model, data)
        apex = max(apex, float(data.xpos[c.base][2]))
        guard += 1
        if guard % 20 == 0:
            snap()
    tail = 0
    while tail < 800:                   # land + settle a moment
        c.step(data, DT)
        mujoco.mj_step(model, data)
        tail += 1
        if tail % 20 == 0:
            snap()
    writer.close()
    try:
        renderer.close()
    except Exception:
        pass
    print(f"apex base z = {apex:.3f} m; wrote {out_path}")


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "stand"
    out_dir = REPO_ROOT / "out"
    out_dir.mkdir(exist_ok=True)
    if which == "stand":
        render_stand(out_dir / "lqr_stand_5s.mp4")
    elif which == "jump":
        render_jump(out_dir / "jump.mp4")
    else:
        print("usage: render_demo.py [stand|jump]")


if __name__ == "__main__":
    main()
