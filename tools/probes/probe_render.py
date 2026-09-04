"""Render the robot from a fixed world camera and save PNGs for visual check."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import mujoco
import mujoco.glfw
import numpy as np

MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast.xml"
OUT = REPO_ROOT / "probe_render.png"


def main() -> None:
    from mujoco.glfw import glfw
    glfw.init()
    glfw.window_hint(glfw.VISIBLE, 0)
    window = glfw.create_window(800, 600, "offscreen", None, None)
    glfw.make_context_current(window)
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    # put the robot on the ground
    for _ in range(200):
        mujoco.mj_step(model, data)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")]
    # top-down: x right, y up on screen; world +x = right, +y = up
    cam.azimuth = 0.0
    cam.elevation = -89.5
    cam.distance = 1.5
    opt = mujoco.MjvOption()
    scn = mujoco.MjvScene(model, 1000)
    con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)
    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
    viewport = mujoco.MjrRect(0, 0, 800, 600)
    mujoco.mjr_render(viewport, scn, con)
    rgb = np.zeros((600, 800, 3), dtype=np.uint8)
    mujoco.mjr_readPixels(rgb, None, viewport, con)
    rgb = np.flipud(rgb)
    from PIL import Image
    Image.fromarray(rgb).save(OUT)
    print(f"saved {OUT}")
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    print("base xpos:", data.xpos[base])
    print("world +x = screen right; world +y = screen up (top-down view)")


if __name__ == "__main__":
    main()
