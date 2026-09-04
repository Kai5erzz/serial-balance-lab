"""Chained jump-climb: stall at each riser, hop, mount, repeat x3 tiers.

Headless:  python climb_chain.py
Viewer:    python climb_chain.py --viewer
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
import traditional_lqr as TL  # noqa: E402
from traditional_lqr import (  # noqa: E402
    CALIBRATED_STAND_POSE_PATH,
    apply_saved_pose,
)

MODEL = ROOT / "model" / "whole_body_lqr_fast_platform.xml"
HEADLESS = "--viewer" not in sys.argv
TIER_H = (0.2, 0.4, 0.6)
APPROACH = float(sys.argv[-2]) if len(sys.argv) > 2 else 0.30
TRIGGER = float(sys.argv[-1]) if len(sys.argv) > 2 else 2.05


def support_levels(model, c, data):
    lvl = {"l": 0.0, "r": 0.0}
    for i in range(data.ncon):
        con = data.contact[i]
        pair = [int(con.geom1), int(con.geom2)]
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
            for g in pair
        ]
        for side in ("l", "r"):
            if int(model.geom_bodyid[pair[0]]) == c.sides[side].wheel_body or \
               int(model.geom_bodyid[pair[1]]) == c.sides[side].wheel_body:
                for nm in names:
                    if nm == "platform_step2":
                        lvl[side] = max(lvl[side], 0.2)
                    elif nm == "platform_step1":
                        lvl[side] = max(lvl[side], 0.4)
                    elif nm == "platform_top":
                        lvl[side] = max(lvl[side], 0.6)
    return lvl


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    c = TL.TraditionalLQR(model)
    apply_saved_pose(model, data, Path(CALIBRATED_STAND_POSE_PATH))
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    data.qpos[0] += float(h[0]) * 2.2
    data.qpos[1] += float(h[1]) * 2.2
    mujoco.mj_forward(model, data)
    c.enable_calibrated_lqr(data)
    c.disable_descent_guard = True
    c.disable_danger_check = True

    viewer = None
    if not HEADLESS:
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.cam.lookat[:] = (1.2, -1.2, 0.3)
        viewer.cam.distance = 4.2
        viewer.cam.elevation = -14.0
        viewer.cam.azimuth = 100.0
    deadline = time.time() + 120.0

    def tick():
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if viewer is not None:
            viewer.sync()

    for tier in range(3):
        target = TIER_H[tier]
        trig = TRIGGER - 0.30 * tier
        print(f"=== TIER {tier + 1} (target {target:.2f} m, trigger {trig:.2f}) ===")
        # approach until the trigger distance, then jump with run-up
        c.set_drive_command(APPROACH, 0.0)
        jumped = False
        while time.time() < deadline:
            tick()
            lvl = support_levels(model, c, data)
            if min(lvl.values()) >= target - 1e-3:
                print(f"  already mounted: {lvl}")
                break
            d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
            stalled = d <= trig
            if stalled:
                ok, reason = c.begin_jump(data)
                if ok:
                    jumped = True
                    print(f"  jump accepted (vx={c.chassis_vx:+.2f})")
                    break
                if "cooldown" in reason:
                    time.sleep(0.3)
                    continue
                print(f"  rejected: {reason}")
                time.sleep(0.3)
        if not jumped:
            print("  no jump; stopping")
            break
        # carry: re-drive through preload/thrust, stop in flight
        while time.time() < deadline and c.jump_state != "idle":
            if c.jump_state in ("preload", "thrust"):
                c.set_drive_command(0.30, 0.0)
            else:
                c.stop_drive()
            tick()
        c.stop_drive()
        # creep: roll the straddled wheel up onto the tread
        creep_end = data.time + 1.5
        while data.time < creep_end:
            c.set_drive_command(0.18, 0.0)
            tick()
        c.stop_drive()
        # settle
        end = data.time + 1.5
        while data.time < end:
            tick()
        lvl = support_levels(model, c, data)
        pit = np.degrees(c.body_pitch(data))
        z = float(data.xpos[c.base][2])
        print(f"  landed: support={lvl} z={z:.3f} pitch={pit:+.2f}")

    time.sleep(1.0)
    lvl = support_levels(model, c, data)
    pit = np.degrees(c.body_pitch(data))
    z = float(data.xpos[c.base][2])
    ok_stand = c.is_properly_standing(data)
    print(f"FINAL: z={z:.3f} pitch={pit:+.2f} support={lvl} "
          f"properly_standing={ok_stand}")
    if viewer is not None:
        viewer.close()


if __name__ == "__main__":
    main()
