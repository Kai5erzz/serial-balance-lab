"""Mass-reduction experiment: how much weight loss unlocks the jump targets?

The thrust force is torque-limited (40 Nm rail, mass-independent), so
vz^2 ~ 2*stroke*(F/m - g): lighter means faster.  This probe scales the
whole plant mass by k (links + inertia; rotor armature untouched --
weight reduction removes structure, not motors), keeps the controller
honest by patching TOTAL_MASS / ROLL_GAIN_MASS_SCALE in every
star-import namespace, and measures the takeoff vz / mount verdict.

Modes:
  vz    flat-ground moving jump, k ladder -> vz(mass) calibration curve
  p270  moving jump onto the 270 mm platform at a given k, trigger sweep

Run:  python tools/probes/mass_reduction.py vz
      python tools/probes/mass_reduction.py p270 k=0.85
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
import balance_lab.control.controller as _ctrl
import balance_lab.control.guards as _guards
import balance_lab.control.tuning as _tun
from balance_lab.plant import load_model
from balance_lab.paths import CALIBRATED_STAND_POSE_PATH
from balance_lab.plant.poses import apply_saved_pose
from balance_lab.control.controller import TraditionalLQR

RESULTS = REPO_ROOT / "artifacts" / "jump_precision" / "mass_results.jsonl"
BASE_MASS = 12.801
P270 = REPO_ROOT / "model" / "whole_body_lqr_fast_p270.xml"


def apply_mass_scale(model, k: float) -> None:
    """Scale structural mass and inertia; armature (rotor) untouched."""
    model.body_mass[:] *= k
    model.body_inertia[:] *= k
    m_now = float(np.sum(model.body_mass))
    for ns in (_tun, _ctrl, _jm, _guards):
        ns.TOTAL_MASS = m_now
        if hasattr(ns, "ROLL_GAIN_MASS_SCALE"):
            ns.ROLL_GAIN_MASS_SCALE = m_now / 6.120


def pkg_max() -> None:
    _jm.JUMP_THRUST_LENGTH = 0.33
    _jm.JUMP_PRELOAD_LENGTH = 0.117
    _jm.JUMP_THRUST_THETA_LQR_GAIN = 0.2
    _jm.JUMP_THRUST_BOOST_FORCE = 100.0
    _jm.JUMP_LENGTH_LIMIT_PRELOAD = 110.0


def spawn_and_arm(model, c, speed: float, spawn_d: float):
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
    return data


def vz_cell(model, c, k: float, speed: float = 0.6) -> dict:
    pkg_max()
    spawn_d = min(2.0 + 2.0 + 1.6 * speed, 5.6)
    data = spawn_and_arm(model, c, speed, spawn_d)
    DT = model.opt.timestep

    def radial():
        return float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))

    def suppress():
        if c.descent_guard_active:
            c.descent_guard_active = False
            c.descent_guard_drop_seen = False
            c.descent_guard_preedge_elapsed = 0.0
            c.descent_edge_profile_active = False

    while data.time < 2.5:
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
    c.set_drive_command(speed, 0.0)
    while data.time < 14.0:
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
        if radial() <= 2.0:
            break
    ok, reason = c.begin_jump(data)
    apex = 0.0
    takeoff = td = None
    last_grounded_d = radial()
    prev = True
    while c.jump_state != "idle" and data.time < 22.0:
        if c.jump_state in ("preload", "thrust"):
            c.set_drive_command(speed, 0.0)
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
        apex = max(apex, float(data.xpos[c.base][2]))
        g = (bool(c._wheel_contacts(data)["l"])
             and bool(c._wheel_contacts(data)["r"]))
        if g:
            last_grounded_d = radial()
            if not prev and takeoff and not td:
                td = dict(t=data.time, d=radial(),
                          p=float(np.degrees(c.body_pitch(data))))
            prev = True
        else:
            if prev:
                takeoff = dict(t=data.time, d=last_grounded_d,
                               vz=float(data.qvel[2]))
            prev = False
    c.stop_drive()
    carry = (takeoff["d"] - td["d"]) if (takeoff and td) else None
    out = dict(mode="vz", k=k, mass=round(BASE_MASS * k, 3),
               vz_takeoff=round(takeoff["vz"], 3) if takeoff else None,
               apex_z=round(apex, 3),
               flight_s=round(td["t"] - takeoff["t"], 3)
               if (td and takeoff) else None,
               carry=round(carry, 3) if carry is not None else None,
               settle_pitch=round(float(np.degrees(c.body_pitch(data))), 2),
               fault=c.jump_fault)
    print(f"  k={k} m={out['mass']}kg vz={out['vz_takeoff']} "
          f"apex={out['apex_z']} carry={out['carry']}", flush=True)
    return out


def p270_cell(model, c, k: float, speed: float, trigger: float) -> dict:
    pkg_max()
    spawn_d = min(trigger + 2.0 + 1.6 * speed, 5.6)
    data = spawn_and_arm(model, c, speed, spawn_d)
    DT = model.opt.timestep

    def radial():
        return float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))

    def suppress():
        if c.descent_guard_active:
            c.descent_guard_active = False
            c.descent_guard_drop_seen = False
            c.descent_guard_preedge_elapsed = 0.0
            c.descent_edge_profile_active = False

    while data.time < 2.5:
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
    c.set_drive_command(speed, 0.0)
    while data.time < 14.0:
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
        if radial() <= trigger:
            break
    ok, reason = c.begin_jump(data)
    apex = 0.0
    crossed = False
    cross_clear = None
    prev = True
    while c.jump_state != "idle" and data.time < 22.0:
        if c.jump_state in ("preload", "thrust"):
            c.set_drive_command(speed, 0.0)
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
        apex = max(apex, float(data.xpos[c.base][2]))
        zw = min(float(data.xpos[c.sides[s].wheel_body][2]) for s in ("l", "r"))
        dw = max(float(np.hypot(data.xpos[c.sides[s].wheel_body][0],
                                data.xpos[c.sides[s].wheel_body][1]))
                 for s in ("l", "r"))
        g = (bool(c._wheel_contacts(data)["l"])
             and bool(c._wheel_contacts(data)["r"]))
        if not g and not crossed and dw <= 1.2:
            crossed = True
            cross_clear = zw
        prev = g
    c.stop_drive()
    lv = {"l": 0.0, "r": 0.0}
    for i in range(data.ncon):
        con = data.contact[i]
        for g in (int(con.geom1), int(con.geom2)):
            nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
            if nm == "p270_top" and abs(con.pos[2] - 0.27) <= 0.035:
                bid = int(model.geom_bodyid[g])
                for side in ("l", "r"):
                    if bid == c.sides[side].wheel_body:
                        lv[side] = 0.27
    pit = float(np.degrees(c.body_pitch(data)))
    rol = float(np.degrees(c.body_roll(data)))
    mounted = lv["l"] > 0 and lv["r"] > 0
    verdict = ("mount" if mounted and abs(pit) < 8 and abs(rol) < 8
               and c.jump_fault == "none" else
               "lip" if mounted else "crossed" if crossed else "short/crash")
    out = dict(mode="p270", k=k, mass=round(BASE_MASS * k, 3),
               speed=speed, trigger=trigger, verdict=verdict,
               vz=None, apex_z=round(apex, 3),
               cross_clear=round(cross_clear, 3)
               if cross_clear is not None else None,
               level_l=lv["l"], level_r=lv["r"],
               settle_pitch=round(pit, 2), settle_roll=round(rol, 2),
               fault=c.jump_fault)
    print(f"  k={k} m={out['mass']}kg v={speed} trg={trigger}: "
          f"{verdict} cross={out['cross_clear']} lv=({lv['l']},{lv['r']})",
          flush=True)
    return out


def save(out: dict) -> None:
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    with open(RESULTS, "a") as f:
        f.write(json.dumps(out, default=str) + "\n")


def main() -> None:
    which = sys.argv[1] if len(sys.argv) > 1 else "vz"
    kv = dict(a.split("=") for a in sys.argv[2:] if "=" in a)
    t0 = time.time()
    if which == "vz":
        model, _ = load_model("fast")
        c = TraditionalLQR(model)
        for k in (1.0, 0.95, 0.90, 0.85, 0.80):
            apply_mass_scale(model, k)
            out = vz_cell(model, c, k)
            save(out)
    elif which == "p270":
        model = mujoco.MjModel.from_xml_path(str(P270))
        c = TraditionalLQR(model)
        k = float(kv.get("k", 0.85))
        apply_mass_scale(model, k)
        speed = float(kv.get("speed", 1.26))
        for trg in (float(x) for x in kv.get("triggers", "1.95,2.0,2.05,2.1").split(",")):
            out = p270_cell(model, c, k, speed, trg)
            save(out)
    print(f"[mass_reduction] done in {time.time()-t0:.0f}s wall", flush=True)


if __name__ == "__main__":
    main()
