"""Diagnostic probe to test stair-hop physics, forward carry velocity, and elevated flight profile."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import mujoco
import numpy as np

from balance_lab.plant import load_model
from balance_lab.paths import CALIBRATED_STAND_POSE_PATH
from balance_lab.plant.poses import apply_saved_pose
from balance_lab.control.controller import TraditionalLQR

def test_stair_hop(
    approach_speed: float = 0.40,
    carry_speed: float = 0.40,
    trigger_dist: float = 1.98,
    air_deploy_length: float = 0.19,
    leg_length: float = 0.20,
):
    model, _ = load_model("platform")
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    c.reset(data, calibrate_frame=True)
    h, yaw = c._heading(data)
    data.qpos[0] += float(h[0]) * 2.8
    data.qpos[1] += float(h[1]) * 2.8
    mujoco.mj_forward(model, data)
    c.enable_calibrated_lqr(data)
    c.disable_descent_guard = True
    c.disable_danger_check = True
    c.set_leg_length(leg_length)

    # Settle
    while data.time < 1.5:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)

    # Approach
    c.set_drive_command(approach_speed, 0.0)
    jumped = False
    while data.time < 8.0:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)
        d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
        if d <= trigger_dist:
            c.jump_air_landing_length = air_deploy_length
            ok, reason = c.begin_jump(data)
            if ok:
                jumped = True
                break

    if not jumped:
        print("Jump not triggered")
        return False

    # Jump lifecycle
    while data.time < 12.0 and c.jump_state != "idle":
        if c.jump_state in ("preload", "thrust"):
            c.set_drive_command(carry_speed, 0.0)
            c.jump_air_landing_length = air_deploy_length
        else:
            c.stop_drive()
        c.step(data, 0.001)
        mujoco.mj_step(model, data)

    # Creep forward to seat wheels on tread
    creep_end = data.time + 1.2
    while data.time < creep_end:
        c.set_drive_command(0.12, 0.0)
        c.step(data, 0.001)
        mujoco.mj_step(model, data)
    c.stop_drive()

    # Settle
    settle_end = data.time + 1.5
    while data.time < settle_end:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)

    # Evaluate contact & support
    lvl = {"l": 0.0, "r": 0.0}
    for i in range(data.ncon):
        con = data.contact[i]
        for g in (con.geom1, con.geom2):
            bid = int(model.geom_bodyid[int(g)])
            other = con.geom2 if int(g) == con.geom1 else con.geom1
            other_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(other))
            for side in ("l", "r"):
                if bid == c.sides[side].wheel_body:
                    if other_name == "platform_step2": lvl[side] = max(lvl[side], 0.2)
                    elif other_name == "platform_step1": lvl[side] = max(lvl[side], 0.4)
                    elif other_name == "platform_top": lvl[side] = max(lvl[side], 0.6)

    pitch = np.degrees(c.body_pitch(data))
    roll = np.degrees(c.body_roll(data))
    z = data.xpos[c.base][2]
    d_final = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
    ok_stand = c.is_properly_standing(data)
    print(f"app={approach_speed:.2f} carry={carry_speed:.2f} trig={trigger_dist:.2f} deploy={air_deploy_length:.2f} leg={leg_length:.2f} -> support={lvl} z={z:.3f} d={d_final:.3f} pitch={pitch:+.1f} roll={roll:+.1f} stand={ok_stand}")
    return lvl["l"] >= 0.2 and lvl["r"] >= 0.2

if __name__ == "__main__":
    for trig in [1.90, 1.88, 1.86, 1.84, 1.82, 1.80]:
        for carry in [0.40, 0.48, 0.55]:
            test_stair_hop(approach_speed=0.40, carry_speed=carry, trigger_dist=trig, air_deploy_length=0.17, leg_length=0.24)
