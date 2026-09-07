"""Render current-mass jump attempts for visual debugging (side view).

Two clips, both at full 12.8 kg:
  p270   moving jump onto the 270 mm platform (face collision + crush)
  stair  moving jump at the 200 mm stair tier-1 (the near-success perch)

Side-on camera (perpendicular to the approach diagonal) so the face
strike and pitch are visible.  EGL offscreen, streaming encode.
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

OUT = REPO_ROOT / "out"


def brighten(model):
    h = model.vis.headlight
    h.ambient[:] = [0.7, 0.7, 0.7]
    h.diffuse[:] = [1.1, 1.1, 1.1]
    h.specular[:] = [0.35, 0.35, 0.35]


def render_attempt(tag: str, model_path: Path, speed: float, trigger: float,
                   spawn_d: float, lookat, cam_dist: float,
                   lever: bool) -> None:
    lever_pkg = lever
    model = mujoco.MjModel.from_xml_path(str(model_path))
    brighten(model)
    model.vis.global_.offwidth = 720
    model.vis.global_.offheight = 480
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)
    if lever_pkg:
        _jm.JUMP_THRUST_LENGTH = 0.33
        _jm.JUMP_PRELOAD_LENGTH = 0.117
        _jm.JUMP_THRUST_THETA_LQR_GAIN = 0.2
    else:
        _jm.JUMP_THRUST_LENGTH = 0.33
        _jm.JUMP_PRELOAD_LENGTH = 0.117
        _jm.JUMP_THRUST_THETA_LQR_GAIN = 0.2
        _jm.JUMP_THRUST_BOOST_FORCE = 100.0
        _jm.JUMP_LENGTH_LIMIT_PRELOAD = 110.0
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    data.qpos[0] += float(h[0]) * spawn_d
    data.qpos[1] += float(h[1]) * spawn_d
    mujoco.mj_forward(model, data)
    c.enable_calibrated_lqr(data)
    c.disable_descent_guard = True
    c.disable_danger_check = True
    c.set_leg_length(0.24)
    c.jump_air_landing_length = 0.27
    c.jump_air_deploy_pitch_bias = np.radians(-4.0)
    c.jump_thrust_pitch_rate_target = -2.0

    cam = mujoco.MjvCamera()
    cam.elevation = -8.0
    cam.azimuth = 225.0          # side-on to the approach diagonal
    cam.lookat[:] = lookat
    cam.distance = cam_dist
    writer = imageio.get_writer(str(OUT / f"{tag}.mp4"), fps=50,
                                codec="libx264", quality=8,
                                pixelformat="yuv420p")
    renderer = mujoco.Renderer(model, height=480, width=720)
    DT = model.opt.timestep
    n = 0

    def snap():
        renderer.update_scene(data, camera=cam)
        writer.append_data(renderer.render().copy())

    while data.time < 2.5:
        c.step(data, DT)
        mujoco.mj_step(model, data)
        n += 1
        if n % 40 == 0:
            snap()
    c.set_drive_command(speed, 0.0)

    def radial():
        return float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))

    while data.time < 14.0:
        c.step(data, DT)
        mujoco.mj_step(model, data)
        n += 1
        if radial() <= trigger:
            break
        if n % 30 == 0:
            snap()
    ok, reason = c.begin_jump(data)
    print(f"[{tag}] begin_jump {ok} {reason}", flush=True)
    guard = 0
    while c.jump_state != "idle" and guard < 6000:
        if c.jump_state in ("preload", "thrust"):
            c.set_drive_command(speed, 0.0)
        c.step(data, DT)
        mujoco.mj_step(model, data)
        guard += 1
        if guard % 8 == 0:       # dense frames through the strike
            snap()
    for _ in range(1500):
        c.step(data, DT)
        mujoco.mj_step(model, data)
        if guard % 8 == 0 and guard < 3000:
            pass
        guard += 1
        if guard % 20 == 0:
            snap()
    writer.close()
    try:
        renderer.close()
    except Exception:
        pass
    print(f"[{tag}] wrote {OUT / (tag + '.mp4')}", flush=True)


def main() -> None:
    which = sys.argv[1] if len(sys.argv) > 1 else "both"
    if which in ("p270", "both"):
        render_attempt("p270_attempt_current_mass",
                       REPO_ROOT / "model" / "whole_body_lqr_fast_p270.xml",
                       speed=1.26, trigger=2.05, spawn_d=5.0,
                       lookat=np.array([-0.75, 0.75, 0.18]),
                       cam_dist=2.0, lever=False)
    if which in ("stair", "both"):
        render_attempt("stair_tier1_attempt_current_mass",
                       REPO_ROOT / "model" / "whole_body_lqr_fast_platform.xml",
                       speed=1.5, trigger=2.40, spawn_d=5.6,
                       lookat=np.array([-1.0, 1.0, 0.22]),
                       cam_dist=2.3, lever=True)


if __name__ == "__main__":
    main()
