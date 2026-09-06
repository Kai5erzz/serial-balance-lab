"""Test landing with zero or forward-biased wheel torque."""
from __future__ import annotations

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

DIAG = np.array([-1.0 / np.sqrt(2), 1.0 / np.sqrt(2)])

def test_landing_override(trig_d=2.12, forward_torque_bias=-0.8):
    model, _ = load_model("platform")
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)

    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    data.qpos[0] += float(h[0]) * 2.8
    data.qpos[1] += float(h[1]) * 2.8
    mujoco.mj_forward(model, data)

    c.enable_calibrated_lqr(data)
    c.disable_descent_guard = True
    c.disable_danger_check = True

    c.jump_air_tuck_pitch_bias = np.radians(2.0)
    c.jump_air_deploy_pitch_bias = np.radians(-1.0)
    c.jump_air_landing_length = 0.17

    while data.time < 1.5:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)

    c.set_drive_command(0.38, 0.0)
    while data.time < 8.0:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)
        d_base = float(np.dot(data.xpos[c.base][:2], DIAG))
        if d_base <= trig_d:
            c.begin_jump(data)
            break

    t_trig = data.time
    while data.time < t_trig + 4.0 and c.jump_state != "idle":
        if c.jump_state in ("preload", "thrust"):
            c.set_drive_command(0.38, 0.0)
            c.jump_air_landing_length = 0.17
        else:
            c.stop_drive()

        c.step(data, 0.001)

        # If in landing buffer, clamp wheel command directly on data.ctrl!
        if c.landing_buffer_phase != "inactive":
            for side in ("l", "r"):
                wm = c.sides[side].wheel_motor
                sign = c.wheel_sign[side]
                logical_torque = sign * data.ctrl[wm]
                if logical_torque > forward_torque_bias:
                    data.ctrl[wm] = sign * forward_torque_bias

        mujoco.mj_step(model, data)

    # After jump becomes idle, creep forward to center on tread
    creep_until = data.time + 1.2
    while data.time < creep_until:
        c.set_drive_command(0.12, 0.0)
        c.step(data, 0.001)
        for side in ("l", "r"):
            wm = c.sides[side].wheel_motor
            sign = c.wheel_sign[side]
            if sign * data.ctrl[wm] > 0.0:
                data.ctrl[wm] = sign * -0.2
        mujoco.mj_step(model, data)
    c.stop_drive()

    # Settle
    settle_until = data.time + 1.5
    while data.time < settle_until:
        c.step(data, 0.001)
        mujoco.mj_step(model, data)

    d_base = float(np.dot(data.xpos[c.base][:2], DIAG))
    d_wl = float(np.dot(data.xpos[c.sides["l"].wheel_body][:2], DIAG))
    d_wr = float(np.dot(data.xpos[c.sides["r"].wheel_body][:2], DIAG))
    pitch = np.degrees(c.body_pitch(data))
    roll = np.degrees(c.body_roll(data))
    z = data.xpos[c.base][2]
    stand = c.is_properly_standing(data)

    con_geoms = set()
    for i in range(data.ncon):
        con = data.contact[i]
        b1 = model.geom_bodyid[con.geom1]
        b2 = model.geom_bodyid[con.geom2]
        for side in ("l", "r"):
            if b1 == c.sides[side].wheel_body or b2 == c.sides[side].wheel_body:
                other = con.geom2 if b1 == c.sides[side].wheel_body else con.geom1
                con_geoms.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other))

    print(f"bias={forward_torque_bias:+.2f}: stand={stand} con={con_geoms} z={z:.3f} d_b={d_base:.3f} d_w={d_wl:.3f} p={pitch:+.1f}deg fault={c.jump_fault}")
    return stand

if __name__ == "__main__":
    for b in [0.0, -0.3, -0.6, -1.0, -1.5]:
        test_landing_override(trig_d=2.12, forward_torque_bias=b)
