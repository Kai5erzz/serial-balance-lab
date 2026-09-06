"""Instrumented single wall-hop: print the flight timeline.

Answers, for the current plant, three questions the sweep cannot:
  1. how high do the wheels actually rise (need > 0.2 m + wheel radius);
  2. does the nose/chassis engage the riser lip during preload/thrust
     (the lip pivot that halves the apex requirement);
  3. where do we take off and touch down relative to the wall (1.6 m).

Run:  python tools/probes/diag_hop.py [approach] [redrive]
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

APPROACH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
REDRIVE = float(sys.argv[2]) if len(sys.argv) > 2 else 0.30
# optional: --trig D  -> moving jump triggered at base distance D
TRIG = None
if "--trig" in sys.argv:
    TRIG = float(sys.argv[sys.argv.index("--trig") + 1])
SPAWN = 2.6
if "--spawn" in sys.argv:
    SPAWN = float(sys.argv[sys.argv.index("--spawn") + 1])
THK = 0.0
if "--thk" in sys.argv:
    THK = float(sys.argv[sys.argv.index("--thk") + 1])
TLEN = 0.0
if "--tlen" in sys.argv:
    TLEN = float(sys.argv[sys.argv.index("--tlen") + 1])
PLEN = 0.0
if "--plen" in sys.argv:
    PLEN = float(sys.argv[sys.argv.index("--plen") + 1])
TGAIN = -1.0
if "--tgain" in sys.argv:
    TGAIN = float(sys.argv[sys.argv.index("--tgain") + 1])
LREF = 99.0
if "--lref" in sys.argv:
    LREF = float(sys.argv[sys.argv.index("--lref") + 1])
THOLD = 0.0
if "--thold" in sys.argv:
    THOLD = float(sys.argv[sys.argv.index("--thold") + 1])
DRATE = 0.0
if "--drate" in sys.argv:
    DRATE = float(sys.argv[sys.argv.index("--drate") + 1])
KICK = 0.0
if "--kick" in sys.argv:
    KICK = float(sys.argv[sys.argv.index("--kick") + 1])
RT = None
if "--rt" in sys.argv:
    RT = float(sys.argv[sys.argv.index("--rt") + 1])
PMIN = 0.0
if "--pmin" in sys.argv:
    PMIN = float(sys.argv[sys.argv.index("--pmin") + 1])
CROUCH = 0.0
if "--crouch" in sys.argv:
    CROUCH = float(sys.argv[sys.argv.index("--crouch") + 1])

model, _ = load_model("platform")
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
c.set_leg_length(0.24 if CROUCH <= 0 else CROUCH)
if PMIN > 0:
    import balance_lab.control.jump as _jm
    _jm.JUMP_PRELOAD_MIN_TIME = PMIN

TIER_TOP = {"platform_step2": 0.2, "platform_step1": 0.4,
            "platform_top": 0.6, "ground_outer": 0.0,
            "platform_step3": 0.0}


def contacts_summary() -> str:
    hits = {}
    for i in range(data.ncon):
        con = data.contact[i]
        names = []
        for g in (int(con.geom1), int(con.geom2)):
            names.append(mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM, g))
        for nm in names:
            if nm in TIER_TOP:
                other = [n for n in names if n != nm][0]
                key = f"{other.replace('fast_collision_', '')}->{nm}"
                hits[key] = hits.get(key, 0) + 1
    return ",".join(sorted(hits)) or "-"


def suppress() -> None:
    if c.descent_guard_active:
        c.descent_guard_active = False
        c.descent_guard_drop_seen = False
        c.descent_guard_preedge_elapsed = 0.0
        c.descent_edge_profile_active = False


while data.time < 2.5:
    c.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    suppress()
if CROUCH > 0:
    while data.time < 3.7:
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        suppress()

c.set_drive_command(APPROACH, 0.0)
d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
while data.time < 12.0:
    c.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    suppress()
    d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
    if TRIG is not None:
        if d <= TRIG:
            break
    elif (abs(c.chassis_vx) < 0.08 and d < 2.05) or d <= 1.82:
        break
print(f"stall: d={d:.3f} vx={c.chassis_vx:+.3f} "
      f"pitch={np.degrees(c.body_pitch(data)):+.2f} "
      f"z={data.xpos[c.base][2]:.3f} contacts=[{contacts_summary()}]")

if RT is not None:
    c.jump_thrust_pitch_rate_target = RT
ok, reason = c.begin_jump(data)
print(f"begin_jump: {ok} {reason}")
if ok:
    next_print = 0.0
    apex_base = apex_wheel = 0.0
    takeoff_d = d
    prev_state = ""
    while data.time < 20.0 and c.jump_state != "idle":
        if c.jump_state in ("preload", "thrust"):
            c.set_drive_command(REDRIVE, 0.0)
        c.step(data, model.opt.timestep)
        if THK and c.jump_state == "thrust" and not c.jump_off_ground:
            for side in ("l", "r"):
                data.ctrl[c.sides[side].wheel_motor] = (
                    c.wheel_sign[side] * THK)
        if LREF < 90.0 and c.jump_state == "landing":
            c.drive_forward_target = LREF
        if KICK and c.jump_state == "landing":
            for side in ("l", "r"):
                data.ctrl[c.sides[side].wheel_motor] = (
                    c.wheel_sign[side] * KICK)
        mujoco.mj_step(model, data)
        suppress()
        zb = float(data.xpos[c.base][2])
        zw = min(float(data.xpos[c.sides[s].wheel_body][2])
                 for s in ("l", "r"))
        apex_base = max(apex_base, zb)
        apex_wheel = max(apex_wheel, zw)
        if c.jump_state != prev_state:
            print(f"t={data.time:6.2f} phase->{c.jump_state}")
            prev_state = c.jump_state
            next_print = 0.0
        if data.time >= next_print:
            dd = float(np.hypot(data.xpos[c.base][0],
                                data.xpos[c.base][1]))
            dwl = float(np.hypot(data.xpos[c.sides["l"].wheel_body][0],
                                 data.xpos[c.sides["l"].wheel_body][1]))
            print(f"t={data.time:6.2f} {c.jump_state:9s} z={zb:.3f} "
                  f"wheel_z={zw:.3f} d={dd:.3f} wheel_d={dwl:.3f} "
                  f"vx={c.chassis_vx:+.2f} "
                  f"pitch={np.degrees(c.body_pitch(data)):+6.2f} "
                  f"[{contacts_summary()}]")
            next_print = data.time + 0.02
    print(f"apex: base={apex_base:.3f} wheel={apex_wheel:.3f} "
          f"(wheel needs >= 0.258 = 0.2 tread + 0.058 radius)")
    dwl = float(np.hypot(data.xpos[c.sides["l"].wheel_body][0],
                         data.xpos[c.sides["l"].wheel_body][1]))
    print(f"takeoff base_d={takeoff_d:.3f} final wheel_d={dwl:.3f} "
          f"(tread spans wheel_d 1.3..1.6, lip plane at 1.6)")
    print(f"fault={c.jump_fault} idle z={data.xpos[c.base][2]:.3f}")
