"""Live MuJoCo window that plays the mode-7 stair-climb attempt.

Pops up a real viewer on the desktop, runs the same scenario as
mode7_stair_climb_demo.py, auto-exits after ~60 s (or when closed).
Run:  python viz_mode7_viewer.py [leg_length]
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT))
from traditional_lqr import (  # noqa: E402
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)

MODEL = ROOT / "model" / "whole_body_lqr_fast_platform.xml"
LEG_LENGTH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.24


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)
    apply_saved_pose(model, data, Path(CALIBRATED_STAND_POSE_PATH))
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    data.qpos[0] -= float(h[0]) * 2.2
    data.qpos[1] -= float(h[1]) * 2.2
    mujoco.mj_forward(model, data)
    c.enable_calibrated_lqr(data)
    if abs(LEG_LENGTH - 0.150) > 1e-6:
        try:
            c.set_leg_length(LEG_LENGTH)
        except ValueError as exc:
            raise SystemExit(str(exc))

    with mujoco.viewer.launch_passive(model, data) as v:
        # frame the action: look at the spawn point, slight elevation
        v.cam.lookat[:] = (float(data.qpos[0]), float(data.qpos[1]), 0.3)
        v.cam.distance = 4.0
        v.cam.elevation = -12.0
        v.cam.azimuth = 100.0

        deadline = time.time() + 300.0
        end = data.time + 3.0
        while data.time < end and time.time() < deadline:
            c.step(data, model.opt.timestep)
            mujoco.mj_step(model, data)
            v.sync()

        hops = 0
        while time.time() < deadline:
            ok, reason = c.begin_jump(data)
            if not ok:
                print(f"[hop {hops}] rejected: {reason}")
                # wait for cooldown to expire, then try again
                time.sleep(2.5)
                continue
            print(f"[hop {hops}] jump! z={data.xpos[c.base, 2]:.3f}")
            prev_state = c.jump_state
            while time.time() < deadline:
                c.step(data, model.opt.timestep)
                mujoco.mj_step(model, data)
                v.sync()
                if c.jump_state != prev_state:
                    prev_state = c.jump_state
                    print(f"    -> {c.jump_state}")
                pit = np.degrees(c.body_pitch(data))
                if abs(pit) > 29 or c.jump_state == "idle":
                    break
            hops += 1
            hold = time.time() + 1.5
            while time.time() < hold:
                c.step(data, model.opt.timestep)
                mujoco.mj_step(model, data)
                v.sync()
        print("viewer session ended")


if __name__ == "__main__":
    main()
