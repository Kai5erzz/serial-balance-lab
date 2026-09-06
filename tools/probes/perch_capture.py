"""Capture the tier-1 perch state as an RL handoff artifact.

Runs the best-known ballistic approach (lever package, see
artifacts/climb_matrix/PROGRESS.md) and snapshots full qpos/qvel at the
moment both wheels first touch the tier-1 tread face -- the "perch".
The seating sub-problem (roll 4-6 cm forward and up onto the tread)
starts from this state; it is the natural bootstrap point for an RL
policy or a future scripted climber.

Output: artifacts/climb_matrix/perch_state.npz (qpos, qvel, time, meta)

Run:  python tools/probes/perch_capture.py
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
from balance_lab.paths import CALIBRATED_STAND_POSE_PATH
from balance_lab.plant.poses import apply_saved_pose
from balance_lab.control.controller import TraditionalLQR
import balance_lab.control.jump as _jm

MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast_platform.xml"
OUT = REPO_ROOT / "artifacts" / "climb_matrix" / "perch_state.npz"

APPROACH = 1.5
TRIGGER = 2.40
SPAWN = 2.6


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
apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
mujoco.mj_forward(model, data)
c.reset(data, calibrate_frame=True)
h, _ = c._heading(data)
data.qpos[0] += float(h[0]) * SPAWN
data.qpos[1] += float(h[1]) * SPAWN
mujoco.mj_forward(model, data)
c.enable_calibrated_lqr(data)
c.disable_descent_guard = True
c.disable_danger_check = True
c.set_leg_length(0.24)
c.jump_air_landing_length = 0.27
c.jump_air_deploy_pitch_bias = np.radians(-4.0)
_jm.JUMP_THRUST_LENGTH = 0.33
_jm.JUMP_PRELOAD_LENGTH = 0.117
_jm.JUMP_THRUST_THETA_LQR_GAIN = 0.2
c.jump_thrust_pitch_rate_target = -2.0

while data.time < 2.5:
    c.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)

c.set_drive_command(APPROACH, 0.0)
d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
while data.time < 14.0:
    c.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    if c.descent_guard_active:
        c.descent_guard_active = False
    d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
    if d <= TRIGGER:
        break

ok, reason = c.begin_jump(data)
print(f"begin_jump {ok} {reason}")
captured = None
while data.time < 20.0 and c.jump_state != "idle":
    if c.jump_state in ("preload", "thrust"):
        c.set_drive_command(APPROACH, 0.0)
    c.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    if c.descent_guard_active:
        c.descent_guard_active = False
    if c.jump_state in ("flight", "landing") and captured is None:
        zw = min(float(data.xpos[c.sides[s].wheel_body][2])
                 for s in ("l", "r"))
        dwl = float(np.hypot(data.xpos[c.sides["l"].wheel_body][0],
                             data.xpos[c.sides["l"].wheel_body][1]))
        dwr = float(np.hypot(data.xpos[c.sides["r"].wheel_body][0],
                             data.xpos[c.sides["r"].wheel_body][1]))
        if max(dwl, dwr) < 1.67 and zw > 0.14:
            captured = (data.qpos.copy(), data.qvel.copy(),
                        float(data.time))
            print(f"PERCH captured at t={data.time:.3f} "
                  f"wheel_d=({dwl:.3f},{dwr:.3f}) wheel_z={zw:.3f}")

if captured is not None:
    qpos, qvel, t = captured
    # roll forward another 120 ms of sim so the snapshot is mid-perch,
    # not at the first-contact spike
    t_end = t + 0.12
    while data.time < t_end:
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if c.descent_guard_active:
            c.descent_guard_active = False
    OUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez(OUT, qpos=data.qpos.copy(), qvel=data.qvel.copy(),
             time=float(data.time),
             perch_time=t,
             config=np.array([APPROACH, TRIGGER, SPAWN]))
    print(f"saved {OUT} at t={data.time:.3f} "
          f"z={data.xpos[c.base][2]:.3f} "
          f"pitch={np.degrees(c.body_pitch(data)):+.2f}")
else:
    print("no perch captured this run")
