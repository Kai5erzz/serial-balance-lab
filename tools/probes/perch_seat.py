"""Try to seat the robot onto the tier-1 tread from the captured perch.

Loads artifacts/climb_matrix/perch_state.npz (qpos/qvel at the perch),
re-arms the controller, and drives the wheels forward at the torque cap
with locked legs.  Success: both wheels rest on the step-2 tread top
face and the pose holds for 2 s.

Run:  python tools/probes/perch_seat.py [--torque 3.69] [--hold 2.0]
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import mujoco
import numpy as np

from balance_lab.plant import load_model
from balance_lab.control.controller import TraditionalLQR
import balance_lab.control.jump as _jm

MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast_platform.xml"
STATE = REPO_ROOT / "artifacts" / "climb_matrix" / "perch_state.npz"

TORQUE = 3.69
HOLD = 2.0
for a in sys.argv:
    if a.startswith("--torque="):
        TORQUE = float(a.split("=")[1])
    if a.startswith("--hold="):
        HOLD = float(a.split("=")[1])


def tier_top_of(name: str):
    m = {"ground_outer": 0.0, "platform_step3": 0.0,
         "platform_step2": 0.2, "platform_step1": 0.4, "platform_top": 0.6}
    if name in m:
        return m[name]
    for base in ("platform_top", "platform_step1", "platform_step2",
                 "platform_step3"):
        if name.startswith(base):
            return m[base]
    return None


model = mujoco.MjModel.from_xml_path(str(MODEL))
data = mujoco.MjData(model)
c = TraditionalLQR(model)
state = np.load(STATE)
data.qpos[:] = state["qpos"]
data.qvel[:] = state["qvel"]
mujoco.mj_forward(model, data)
try:
    c.enable_calibrated_lqr(data)
except ValueError:
    # the perch is not the calibrated N-stand pose; arm the same state
    # without the start-condition guard
    c.reset(data)
    c.enabled = True
    c.stance_enabled = True
    c.gas_enabled = True
    c.gimbal_locked = True
    c.lqr_leg_level = "low"
    c.lqr_gain = 1.0
    c.recovery_state = "idle"
    c.control_elapsed = 0.0
c.disable_descent_guard = True
c.disable_danger_check = True
c.set_leg_length(0.24)
_jm.LANDING_LENGTH_KP = 3000.0

def wheel_support():
    lv = {"l": 0.0, "r": 0.0}
    for i in range(data.ncon):
        con = data.contact[i]
        for g in (int(con.geom1), int(con.geom2)):
            nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
            top = tier_top_of(nm or "")
            if top == 0.2 and abs(con.pos[2] - 0.2) <= 0.035:
                bid = int(model.geom_bodyid[g])
                for side in ("l", "r"):
                    if bid == c.sides[side].wheel_body:
                        lv[side] = 0.2
    return lv

seat_start = None
t0 = data.time
while data.time < t0 + 4.0:
    c.step(data, model.opt.timestep)
    for side in ("l", "r"):
        wm = c.sides[side].wheel_motor
        data.ctrl[wm] = c.wheel_sign[side] * TORQUE
    mujoco.mj_step(model, data)
    if c.descent_guard_active:
        c.descent_guard_active = False
    lv = wheel_support()
    both = lv["l"] > 0 and lv["r"] > 0
    if both and seat_start is None:
        seat_start = data.time
        print(f"both wheels on tread at t={data.time:.3f}")
    if seat_start is not None and data.time - seat_start >= HOLD:
        break
    if not both and seat_start is not None:
        print(f"lost tread at t={data.time:.3f}")
        seat_start = None

lv = wheel_support()
d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
print(f"final: lvl=({lv['l']},{lv['r']}) d={d:.3f} "
      f"z={data.xpos[c.base][2]:.3f} "
      f"pitch={np.degrees(c.body_pitch(data)):+.2f} "
      f"roll={np.degrees(c.body_roll(data)):+.2f} "
      f"standing={c.is_properly_standing(data)} "
      f"held={seat_start is not None and data.time - seat_start >= HOLD}")
