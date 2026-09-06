"""Stair-climbing pipeline (Mode 7): Ground -> Tier 1 -> Tier 2 -> Tier 3 (Platform Top).

Usage:
    python tools/probes/climb_stair_pipeline.py [--tier 1|2|3] [--viewer]
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import argparse
import time
import mujoco
import numpy as np

from balance_lab.plant import load_model
from balance_lab.paths import CALIBRATED_STAND_POSE_PATH
from balance_lab.plant.poses import apply_saved_pose
from balance_lab.control.controller import TraditionalLQR

DIAG = np.array([-1.0 / np.sqrt(2), 1.0 / np.sqrt(2)])

def get_wheel_support(model: mujoco.MjModel, data: mujoco.MjData, c: TraditionalLQR) -> dict[str, set[str]]:
    """Return the set of contact geoms touching each wheel."""
    support = {"l": set(), "r": set()}
    wheel_bids = {side: c.sides[side].wheel_body for side in ("l", "r")}
    for i in range(data.ncon):
        con = data.contact[i]
        g1, g2 = int(con.geom1), int(con.geom2)
        b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
        for side in ("l", "r"):
            bid = wheel_bids[side]
            if b1 == bid or b2 == bid:
                other_geom = g2 if b1 == bid else g1
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other_geom)
                if name:
                    support[side].add(name)
    return support

def get_platform_level(support: dict[str, set[str]]) -> dict[str, float]:
    """Convert support geom sets to numerical elevation levels."""
    levels = {"l": 0.0, "r": 0.0}
    for side, geoms in support.items():
        if "platform_top" in geoms:
            levels[side] = max(levels[side], 0.6)
        elif "platform_step1" in geoms:
            levels[side] = max(levels[side], 0.4)
        elif "platform_step2" in geoms:
            levels[side] = max(levels[side], 0.2)
        elif "platform_step3" in geoms or "ground_outer" in geoms:
            levels[side] = max(levels[side], 0.0)
    return levels

def spawn_on_diagonal(model: mujoco.MjModel, data: mujoco.MjData, c: TraditionalLQR, dist_from_center: float, z_offset: float = 0.0):
    """Place the robot on the diagonal looking directly towards the center of the pyramid."""
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    data.qpos[2] += z_offset
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    # The nose faces -heading. Moving along +heading places the robot outside the pyramid,
    # so driving forward (towards -heading) drives towards the center (0, 0).
    data.qpos[0] += float(h[0]) * dist_from_center
    data.qpos[1] += float(h[1]) * dist_from_center
    mujoco.mj_forward(model, data)

def execute_stair_hop(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    c: TraditionalLQR,
    target_tier_height: float,
    carry_speed: float = 0.45,
    air_deploy_length: float = 0.18,
    viewer=None,
) -> bool:
    """Execute a jump specifically tailored for climbing onto an elevated stair tier."""
    # Temporarily set air landing length to prevent premature deployment
    c.jump_air_landing_length = air_deploy_length
    ok, reason = c.begin_jump(data)
    if not ok:
        print(f"  [Jump] Rejected: {reason}")
        return False
    
    t_start = data.time
    # Jump lifecycle
    while data.time < t_start + 4.0 and c.jump_state != "idle":
        if c.jump_state in ("preload", "thrust"):
            c.set_drive_command(carry_speed, 0.0)
            c.jump_air_landing_length = air_deploy_length
        else:
            c.stop_drive()
        c.step(data, 0.001)
        mujoco.mj_step(model, data)
        if viewer:
            viewer.sync()
            
    c.stop_drive()
    return c.jump_fault == "none"

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tier", type=int, default=1, choices=[1, 2, 3], help="Test up to this tier")
    parser.add_argument("--viewer", action="store_true", help="Launch interactive viewer")
    args = parser.parse_args()

    model, _ = load_model("platform")
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)
    
    # Ground spawn: 2.8 meters from center (Tier 1 wall is at 1.6m)
    spawn_on_diagonal(model, data, c, dist_from_center=2.8, z_offset=0.0)
    c.enable_calibrated_lqr(data)
    c.disable_descent_guard = True
    c.disable_danger_check = True

    viewer = None
    if args.viewer:
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.cam.lookat[:] = (1.2, -1.2, 0.3)
        viewer.cam.distance = 4.2
        viewer.cam.elevation = -14.0
        viewer.cam.azimuth = 100.0

    print("=== Phase 0: Settle on ground ===")
    settle_until = data.time + 1.5
    while data.time < settle_until:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)
        if viewer:
            viewer.sync()

    print(f"Settled at d={float(np.dot(data.xpos[c.base][:2], DIAG)):.3f}, pitch={np.degrees(c.body_pitch(data)):.2f}")

    # ==========================================
    # TIER 1: Ground (0.0m) -> Tier 1 (0.20m)
    # ==========================================
    print("=== Phase 1: Approach & Jump to Tier 1 ===")
    c.set_drive_command(0.35, 0.0)
    
    # Approach until trigger distance
    t1_trig = 1.96  # Wall is at 1.60m
    jumped = False
    approach_deadline = data.time + 8.0
    while data.time < approach_deadline:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)
        if viewer:
            viewer.sync()
        d_base = float(np.dot(data.xpos[c.base][:2], DIAG))
        if d_base <= t1_trig:
            jumped = True
            break
            
    if not jumped:
        print("Failed to reach Tier 1 trigger distance")
        return

    print(f"Triggering Tier 1 hop at d={d_base:.3f}, vx={c.chassis_vx:.3f}")
    success = execute_stair_hop(model, data, c, target_tier_height=0.20, carry_speed=0.35, air_deploy_length=0.18, viewer=viewer)
    print(f"Hop 1 result: success={success}, fault={c.jump_fault}")

    # Post-hop creep & settle on Tier 1
    creep_until = data.time + 1.2
    while data.time < creep_until:
        c.set_drive_command(0.12, 0.0)
        c.step(data, 0.001)
        mujoco.mj_step(model, data)
        if viewer:
            viewer.sync()
    c.stop_drive()

    settle_until = data.time + 1.5
    while data.time < settle_until:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)
        if viewer:
            viewer.sync()

    sup = get_wheel_support(model, data, c)
    lvl = get_platform_level(sup)
    d_base = float(np.dot(data.xpos[c.base][:2], DIAG))
    pitch = np.degrees(c.body_pitch(data))
    roll = np.degrees(c.body_roll(data))
    z = data.xpos[c.base][2]
    ok_stand = c.is_properly_standing(data)
    print(f"After Tier 1: levels={lvl}, z={z:.3f}, d={d_base:.3f}, pitch={pitch:+.2f}, roll={roll:+.2f}, standing={ok_stand}")

    if viewer:
        time.sleep(2.0)
        viewer.close()

if __name__ == "__main__":
    main()
