"""Render a climb_matrix cell run to a GIF for visual verification.

Uses the same run_cell sequence but records MuJoCo offscreen frames and
writes out/climb_<tag>.gif.  Headless (EGL/OSMesa), no interactive
viewer.

Run:  python tools/probes/render_climb.py --trig 2.42 [cell params...]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import imageio
import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from balance_lab.plant import load_model  # noqa: E402
from balance_lab.paths import CALIBRATED_STAND_POSE_PATH  # noqa: E402
from balance_lab.plant.poses import apply_saved_pose  # noqa: E402
from balance_lab.control.controller import TraditionalLQR  # noqa: E402
import balance_lab.control.jump as _jm  # noqa: E402

TIER_TOP = {"ground_outer": 0.0, "platform_step3": 0.0,
            "platform_step2": 0.2, "platform_step1": 0.4,
            "platform_top": 0.6}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trig", type=float, default=2.40)
    ap.add_argument("--approach", type=float, default=1.5)
    ap.add_argument("--redrive", type=float, default=1.5)
    ap.add_argument("--rt", type=float, default=-2.0)
    ap.add_argument("--bias", type=float, default=-4.0)
    ap.add_argument("--spawn", type=float, default=3.6)
    ap.add_argument("--out", default="climb_mount.gif")
    args = ap.parse_args()

    model, _ = load_model("platform")
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    data.qpos[0] += float(h[0]) * args.spawn
    data.qpos[1] += float(h[1]) * args.spawn
    mujoco.mj_forward(model, data)
    c.enable_calibrated_lqr(data)
    c.disable_descent_guard = True
    c.disable_danger_check = True
    c.set_leg_length(0.24)
    c.jump_air_landing_length = 0.27
    c.jump_air_deploy_pitch_bias = np.radians(args.bias)

    renderer = mujoco.Renderer(model, height=480, width=640)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [1.2, -1.2, 0.35]
    cam.distance = 4.0
    cam.elevation = -12.0
    cam.azimuth = 115.0
    frames = []

    def snap():
        renderer.update_scene(data, camera=cam)
        frames.append(renderer.render().copy())

    def suppress():
        if c.descent_guard_active:
            c.descent_guard_active = False
            c.descent_guard_drop_seen = False
            c.descent_guard_preedge_elapsed = 0.0
            c.descent_edge_profile_active = False

    while data.time < 2.5:
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        suppress()
    c.set_drive_command(args.approach, 0.0)
    d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
    while data.time < 14.0:
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        suppress()
        d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
        if d <= args.trig:
            break
    c.jump_thrust_pitch_rate_target = args.rt
    ok, reason = c.begin_jump(data)
    print(f"begin_jump {ok} {reason}")
    while data.time < 20.0 and c.jump_state != "idle":
        if c.jump_state in ("preload", "thrust"):
            c.set_drive_command(args.redrive, 0.0)
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        suppress()
        if c.jump_state in ("thrust", "flight"):
            snap()
        elif data.time % 0.1 < 0.002:
            snap()
    c.stop_drive()
    creep_end = data.time + 1.4
    while data.time < creep_end:
        c.set_drive_command(0.18, 0.0)
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        suppress()
        if data.time % 0.05 < 0.002:
            snap()
    c.stop_drive()
    settle_end = data.time + 1.5
    while data.time < settle_end:
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        suppress()
        if data.time % 0.1 < 0.002:
            snap()

    lv = {"l": 0.0, "r": 0.0}
    for i in range(data.ncon):
        con = data.contact[i]
        for g in (int(con.geom1), int(con.geom2)):
            nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
            top = TIER_TOP.get(nm)
            if top is None:
                continue
            for side in ("l", "r"):
                bid = c.sides[side].wheel_body
                if int(model.geom_bodyid[g]) == bid and \
                        abs(con.pos[2] - top) <= 0.035:
                    lv[side] = max(lv[side], top)
    pit = np.degrees(c.body_pitch(data))
    rol = np.degrees(c.body_roll(data))
    print(f"final lvl={lv} pitch={pit:+.2f} roll={rol:+.2f} "
          f"z={data.xpos[c.base][2]:.3f} standing={c.is_properly_standing(data)}")

    out = REPO_ROOT / "out" / args.out
    imageio.mimsave(out, frames, fps=50, loop=0)
    print(f"wrote {out} ({len(frames)} frames)")


if __name__ == "__main__":
    main()
