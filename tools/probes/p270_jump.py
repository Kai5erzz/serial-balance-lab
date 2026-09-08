"""Moving jump onto the single 270 mm platform (p270 terrain).

Design (ballistics, lever package vz~2.32): the wheel must cross the lip
plane (radial 1.2) while its centre is above 0.328 m (= 0.27 lip + wheel
radius).  With vz 2.32 that clearance window is t in [0.207, 0.266] s
after takeoff, so the takeoff wheel radius must sit in
[1.2 + 0.207*vx, 1.2 + 0.266*vx]; with the compression-collision
constraint this lands near approach 1.2 m/s, trigger ~2.1 m.

Run:  python tools/probes/p270_jump.py sweep           # trigger/speed grid
      python tools/probes/p270_jump.py cell speed=1.2 trigger=2.10
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import mujoco
import numpy as np

import balance_lab.control.jump as _jm
from balance_lab.plant import load_model
from balance_lab.paths import CALIBRATED_STAND_POSE_PATH
from balance_lab.plant.poses import apply_saved_pose
from balance_lab.control.controller import TraditionalLQR

import os
MODEL = REPO_ROOT / "model" / f"whole_body_lqr_fast_p{int(float(os.environ.get('P_H','0.27'))*1000):03d}.xml"
RESULTS = REPO_ROOT / "artifacts" / "jump_precision" / "p270_results.jsonl"

LIP_RADIAL = 1.2
LIP_TOP = 0.27
WHEEL_R = 0.058
CLEAR_Z = LIP_TOP + WHEEL_R      # 0.328 centre height to clear the lip


def wheel_state(model, data, c):
    zw = min(float(data.xpos[c.sides[s].wheel_body][2]) for s in ("l", "r"))
    dw = max(float(np.hypot(data.xpos[c.sides[s].wheel_body][0],
                            data.xpos[c.sides[s].wheel_body][1]))
             for s in ("l", "r"))
    return zw, dw


def run_cell(model, c, speed: float, trigger: float,
             log=lambda m: None) -> dict:
    _jm.JUMP_THRUST_LENGTH = 0.33
    _jm.JUMP_PRELOAD_LENGTH = 0.117
    _jm.JUMP_THRUST_THETA_LQR_GAIN = 0.2
    spawn_d = trigger + 2.0 + 1.6 * speed
    data = mujoco.MjData(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    data.qpos[0] += float(h[0]) * spawn_d
    data.qpos[1] += float(h[1]) * spawn_d
    mujoco.mj_forward(model, data)
    c.enable_calibrated_lqr(data)
    c.disable_descent_guard = True
    c.disable_danger_check = True
    c.set_leg_length(0.24)
    c.jump_air_landing_length = 0.27
    c.jump_air_deploy_pitch_bias = np.radians(-4.0)
    c.jump_thrust_pitch_rate_target = -2.0

    def radial():
        return float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))

    def suppress():
        if c.descent_guard_active:
            c.descent_guard_active = False
            c.descent_guard_drop_seen = False
            c.descent_guard_preedge_elapsed = 0.0
            c.descent_edge_profile_active = False

    DT = model.opt.timestep
    while data.time < 2.5:
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()

    t_drive = data.time
    deadline = data.time + 14.0
    d = radial()
    while data.time < deadline:
        cmd = min(speed, 0.35 + 0.4 * (data.time - t_drive))
        c.set_drive_command(cmd, 0.0)
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
        d = radial()
        if d <= trigger:
            break
    vx_trigger = float(c.chassis_vx)

    ok, reason = c.begin_jump(data)
    if not ok:
        return dict(speed=speed, trigger=trigger, verdict="rejected",
                    reason=reason, vx_trigger=round(vx_trigger, 3))

    takeoff = td = None
    apex_z = 0.0
    cross_clear = None     # wheel centre height when crossing the lip plane
    last_grounded_d = radial()
    prev_grounded = True
    crossed = False
    while c.jump_state != "idle" and data.time < deadline + 8.0:
        if c.jump_state in ("preload", "thrust"):
            c.set_drive_command(speed, 0.0)
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
        zb = float(data.xpos[c.base][2])
        apex_z = max(apex_z, zb)
        zw, dw = wheel_state(model, data, c)
        grounded = bool(c._wheel_contacts(data)["l"]) and \
            bool(c._wheel_contacts(data)["r"])
        if not crossed and not grounded and dw <= LIP_RADIAL:
            crossed = True
            cross_clear = zw
        if grounded:
            last_grounded_d = dw
            if not prev_grounded and takeoff is not None and td is None:
                td = dict(t=data.time, dw=dw, zw=zw,
                          pitch=float(np.degrees(c.body_pitch(data))),
                          roll=float(np.degrees(c.body_roll(data))),
                          vx=float(c.chassis_vx))
            prev_grounded = True
        else:
            if prev_grounded:
                takeoff = dict(t=data.time, dw=last_grounded_d,
                               vx=float(c.chassis_vx),
                               vz=float(data.qvel[2]))
            prev_grounded = False
    c.stop_drive()

    # settle, retract legs, verdict
    t_end = data.time + 2.0
    retracted = False
    while data.time < t_end:
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
        if not retracted and c.jump_state == "idle":
            c.normal_leg_length = 0.18
            retracted = True
        t_end2 = data.time
    pit = float(np.degrees(c.body_pitch(data)))
    rol = float(np.degrees(c.body_roll(data)))
    # support: wheel contact with p270_top near its top face
    lv = {"l": 0.0, "r": 0.0}
    for i in range(data.ncon):
        con = data.contact[i]
        for g in (int(con.geom1), int(con.geom2)):
            nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
            if nm == "p270_top" and abs(con.pos[2] - LIP_TOP) <= 0.035:
                bid = int(model.geom_bodyid[g])
                for side in ("l", "r"):
                    if bid == c.sides[side].wheel_body:
                        lv[side] = LIP_TOP
    mounted = lv["l"] > 0 and lv["r"] > 0
    upright = abs(pit) < 6.0 and abs(rol) < 6.0
    verdict = ("mount" if mounted and upright and c.jump_fault == "none"
               else "lip" if mounted else
               "fail" if crossed else "short")
    carry = (takeoff["dw"] - td["dw"]) if (takeoff and td) else None
    out = dict(speed=speed, trigger=trigger, verdict=verdict,
               vx_trigger=round(vx_trigger, 3),
               vx_takeoff=round(takeoff["vx"], 3) if takeoff else None,
               vz_takeoff=round(takeoff["vz"], 3) if takeoff else None,
               dw_takeoff=round(takeoff["dw"], 3) if takeoff else None,
               cross_clear=round(cross_clear, 3) if cross_clear is not None
               else None,
               apex_z=round(apex_z, 3),
               dw_td=round(td["dw"], 3) if td else None,
               carry=round(carry, 3) if carry is not None else None,
               td_pitch=round(td["pitch"], 2) if td else None,
               settle_pitch=round(pit, 2), settle_roll=round(rol, 2),
               level_l=lv["l"], level_r=lv["r"],
               standing=bool(c.is_properly_standing(data)),
               fault=c.jump_fault)
    log(f"v={speed} trg={trigger}: {verdict} cross_clear={out['cross_clear']} "
        f"dw_td={out['dw_td']} lv=({lv['l']},{lv['r']}) p={pit:+.1f}")
    return out


def save(out: dict) -> None:
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    with open(RESULTS, "a") as f:
        f.write(json.dumps(out, default=str) + "\n")


def main() -> None:
    which = sys.argv[1] if len(sys.argv) > 1 else "sweep"
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    c = TraditionalLQR(model)
    t0 = time.time()
    if which == "sweep":
        for speed, trg in ((1.2, 2.05), (1.2, 2.10), (1.2, 2.15),
                           (1.3, 2.10), (1.1, 2.10), (1.2, 2.20),
                           (1.3, 2.15), (1.4, 2.15)):
            out = run_cell(model, c, speed, trg,
                           log=lambda m: print("  ", m, flush=True))
            save(out)
            print(f"[sweep] v={speed} trg={trg} -> {out['verdict']}",
                  flush=True)
    elif which == "cell":
        kv = dict(a.split("=") for a in sys.argv[2:] if "=" in a)
        out = run_cell(model, c, float(kv.get("speed", 1.2)),
                       float(kv.get("trigger", 2.10)),
                       log=lambda m: print("  ", m, flush=True))
        save(out)
        print("[cell]", json.dumps(out, default=str), flush=True)
    print(f"[p270] done in {time.time()-t0:.0f}s wall", flush=True)


if __name__ == "__main__":
    main()
