"""Test jumping from the wall-stall onto Tier 1."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import mujoco
import numpy as np

from balance_lab.plant import load_model
from tools.probes.climb_stair_pipeline import spawn_on_diagonal, DIAG
from balance_lab.control.controller import TraditionalLQR

def test_wall_jump(thrust_wheel_torque: float = -2.0, thrust_pitch_bias: float = 0.0, air_deploy: float = 0.16):
    model, _ = load_model("platform")
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)
    spawn_on_diagonal(model, data, c, 2.3, 0.0)
    c.enable_calibrated_lqr(data)
    c.disable_descent_guard = True
    c.disable_danger_check = True

    # Settle
    while data.time < 1.0:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)

    # Drive to wall until stalled
    c.set_drive_command(0.25, 0.0)
    while data.time < 5.0:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)
        d = float(np.dot(data.xpos[c.base][:2], DIAG))
        if d <= 1.83:
            break

    c.jump_air_landing_length = air_deploy
    ok, reason = c.begin_jump(data)
    t_trig = data.time
    
    while data.time < t_trig + 2.0 and c.jump_state != "idle":
        c.step(data, 0.001)
        if c.jump_state == "thrust" and not c.jump_off_ground:
            # Overwrite wheel torque during thrust
            for side in ("l", "r"):
                data.ctrl[c.sides[side].wheel_motor] = c.wheel_sign[side] * thrust_wheel_torque
            # Add pitch torque if specified
            if abs(thrust_pitch_bias) > 1e-5:
                for side in ("l", "r"):
                    data.ctrl[c.sides[side].hip_motors[0]] += thrust_pitch_bias
                    data.ctrl[c.sides[side].hip_motors[1]] += thrust_pitch_bias
        mujoco.mj_step(model, data)
        
    # Post-jump creep
    creep_until = data.time + 1.2
    while data.time < creep_until:
        c.set_drive_command(0.12, 0.0)
        c.step(data, 0.001)
        mujoco.mj_step(model, data)
    c.stop_drive()

    settle_until = data.time + 1.5
    while data.time < settle_until:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)

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

    d_base = float(np.dot(data.xpos[c.base][:2], DIAG))
    pitch = np.degrees(c.body_pitch(data))
    roll = np.degrees(c.body_roll(data))
    z = data.xpos[c.base][2]
    stand = c.is_properly_standing(data)
    print(f"t_wheel={thrust_wheel_torque:+.1f} t_pitch={thrust_pitch_bias:+.1f} deploy={air_deploy:.2f} -> lvl={lvl} z={z:.3f} d={d_base:.3f} pitch={pitch:+.1f} roll={roll:+.1f} stand={stand}")
    return lvl["l"] >= 0.2 and lvl["r"] >= 0.2

if __name__ == "__main__":
    for tw in [0.0, -1.0, -2.0, -3.0, +1.0, +2.0, +3.0]:
        for tp in [0.0, -5.0, +5.0]:
            test_wall_jump(thrust_wheel_torque=tw, thrust_pitch_bias=tp, air_deploy=0.16)
