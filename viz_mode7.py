"""Render the mode-7 stair-climb attempt into an animated GIF.

Follows the same scenario as mode7_stair_climb_demo.py but captures frames
with MuJoCo's offscreen renderer (EGL) while a tracking camera follows the
chassis.  Outputs:
    out/mode7.gif          - 25 fps animation of the whole attempt
    out/mode7_sheet.png    - 4x3 key-frame contact sheet
"""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageEnhance

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from traditional_lqr import (  # noqa: E402
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)

MODEL = ROOT / "model" / "whole_body_lqr_fast_platform.xml"
OUT = ROOT / "out"
LEG_LENGTH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.24
W, H, FPS = 480, 360, 25


def main() -> None:
    OUT.mkdir(exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)
    apply_saved_pose(model, data, Path(CALIBRATED_STAND_POSE_PATH))

    # spawn near ground outside the lowest tier, like the headless demo
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    data.qpos[0] -= float(h[0]) * 2.2
    data.qpos[1] -= float(h[1]) * 2.2
    data.qpos[2] = max(data.qpos[2], 0.02)
    mujoco.mj_forward(model, data)
    c.enable_calibrated_lqr(data)
    if abs(LEG_LENGTH - 0.150) > 1e-6:
        try:
            c.set_leg_length(LEG_LENGTH)
        except ValueError as exc:
            raise SystemExit(str(exc))

    renderer = mujoco.Renderer(model, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = 3.0
    cam.elevation = -12.0
    cam.azimuth = 100.0

    frames: list[Image.Image] = []

    def snap() -> None:
        p = data.xpos[c.base]
        cam.lookat[:] = (float(p[0]), float(p[1]), float(p[2]) + 0.10)
        renderer.update_scene(data, camera=cam)
        img = Image.fromarray(renderer.render())
        frames.append(ImageEnhance.Brightness(img).enhance(1.6))

    next_frame_t = data.time

    end = data.time + 3.0
    while data.time < end:                    # pre-jump stand
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)

    hops = 0
    t_stop = data.time + 18.0
    while hops < 4 and data.time < t_stop:
        ok, reason = c.begin_jump(data)
        if not ok:
            print(f"[hop {hops}] rejected: {reason}")
            break
        prev_state = c.jump_state
        while data.time < t_stop:
            c.step(data, model.opt.timestep)
            mujoco.mj_step(model, data)
            while data.time >= next_frame_t:
                snap()
                next_frame_t += 1.0 / FPS
            pit = np.degrees(c.body_pitch(data))
            if abs(pit) > 29 or not np.isfinite(data.qpos).all():
                break
            if c.jump_state == "idle":
                break
        print(f"[hop {hops}] finished at z={data.xpos[c.base, 2]:.3f}")
        hops += 1
        hold_end = data.time + 1.5
        while data.time < hold_end:
            c.step(data, model.opt.timestep)
            mujoco.mj_step(model, data)
            while data.time >= next_frame_t:
                snap()
                next_frame_t += 1.0 / FPS

    renderer.close()

    if not frames:
        raise SystemExit("no frames captured")
    gif_path = OUT / "mode7.gif"
    frames[0].save(
        gif_path, save_all=True, append_images=frames[1:],
        duration=int(1000 / FPS), loop=0,
    )
    print(f"wrote {gif_path} ({len(frames)} frames)")

    idx = np.linspace(0, len(frames) - 1, 12).astype(int)
    chosen = [frames[i] for i in dict.fromkeys(idx)]
    cols, rows = 4, 3
    sheet = Image.new("RGB", (cols * W // 2, rows * H // 2))
    for k, im in enumerate(chosen[: cols * rows]):
        sheet.paste(im.resize((W // 2, H // 2)), (
            (k % cols) * W // 2, (k // cols) * H // 2))
    sheet_path = OUT / "mode7_sheet.png"
    sheet.save(sheet_path)
    print(f"wrote {sheet_path}")


if __name__ == "__main__":
    main()
