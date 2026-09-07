"""Moving-jump precision ladder (flat ground).

For each commanded approach speed the robot runs up, crosses a fixed
trigger line, and executes the firmware jump while moving.  Per cell we
record the takeoff state, apex, carry (takeoff->touchdown travel),
touchdown attitude, and the 2 s settle verdict -- the "landing
precision map" for the speed ladder.

With the lever package (thrust stroke 0.33) this doubles as the vz
calibration for the 270 mm platform work: apex_z on flat ground gives
vz = sqrt(2 g (apex_z - z_stand)).

Results append to artifacts/jump_precision/results.jsonl.

Run:  python tools/probes/jump_precision.py ladder [stock|lever]
      python tools/probes/jump_precision.py cell speed=1.0 package=lever ...
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

RESULTS = REPO_ROOT / "artifacts" / "jump_precision" / "results.jsonl"
TRIGGER_D = 2.0          # base radial distance where the jump is requested
SETTLE_S = 2.0           # post-touchdown verdict window


def pkg_setup(package: str) -> None:
    if package == "lever":
        _jm.JUMP_THRUST_LENGTH = 0.33
        _jm.JUMP_PRELOAD_LENGTH = 0.117
        _jm.JUMP_THRUST_THETA_LQR_GAIN = 0.2
    else:
        import balance_lab.control.tuning as _tun
        _jm.JUMP_THRUST_LENGTH = _tun.JUMP_THRUST_LENGTH
        _jm.JUMP_PRELOAD_LENGTH = _tun.JUMP_PRELOAD_LENGTH
        _jm.JUMP_THRUST_THETA_LQR_GAIN = _tun.JUMP_THRUST_THETA_LQR_GAIN


def run_cell(model, c, speed: float, package: str,
             log=lambda m: None) -> dict:
    pkg_setup(package)
    spawn_d = TRIGGER_D + 2.0 + 1.6 * speed
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

    c.set_drive_command(speed, 0.0)
    deadline = data.time + 14.0
    d = radial()
    while data.time < deadline:
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
        d = radial()
        if d <= TRIGGER_D:
            break
    vx_trigger = float(c.chassis_vx)

    ok, reason = c.begin_jump(data)
    if not ok:
        return dict(speed_cmd=speed, package=package, spawn_d=spawn_d,
                    vx_trigger=vx_trigger, verdict="rejected", reason=reason,
                    d_trigger=d)

    # flight tracking: freeze the takeoff state at the ground->air edge,
    # record touchdown at the air->ground edge
    takeoff = None          # dict at the moment the wheels leave ground
    td = None               # dict at the moment they touch back down
    apex_z = 0.0
    last_grounded_d = radial()
    prev_grounded = True
    while c.jump_state != "idle" and data.time < deadline + 8.0:
        if c.jump_state in ("preload", "thrust"):
            c.set_drive_command(speed, 0.0)
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
        zb = float(data.xpos[c.base][2])
        apex_z = max(apex_z, zb)
        grounded = bool(c._wheel_contacts(data)["l"]) and \
            bool(c._wheel_contacts(data)["r"])
        if grounded:
            last_grounded_d = radial()
            if not prev_grounded and takeoff is not None and td is None:
                td = dict(t=data.time, d=radial(),
                          pitch=float(np.degrees(c.body_pitch(data))),
                          roll=float(np.degrees(c.body_roll(data))),
                          vx=float(c.chassis_vx))
            prev_grounded = True
        else:
            if prev_grounded:
                takeoff = dict(t=data.time, d=last_grounded_d,
                               vx=float(c.chassis_vx),
                               vz=float(data.qvel[2]))
            prev_grounded = False
    c.stop_drive()

    # settle and verdict: land, then retract legs to the stand length so
    # is_properly_standing (l0<=0.22) tests "landed and can stand", not the
    # transient landing deploy length
    t_end = data.time + SETTLE_S
    retracted = False
    while data.time < t_end:
        c.step(data, DT)
        mujoco.mj_step(model, data)
        suppress()
        if not retracted and c.jump_state == "idle" \
                and data.time > t_end - SETTLE_S + 0.3:
            c.normal_leg_length = 0.18
            retracted = True
    pit = float(np.degrees(c.body_pitch(data)))
    rol = float(np.degrees(c.body_roll(data)))
    z = float(data.xpos[c.base][2])
    d_fin = radial()
    kins = {s: c.vmc_kinematics(data, s) for s in ("l", "r")}
    grounded = c._wheel_support_contacts(data)
    if td is not None and takeoff is not None:
        carry = takeoff["d"] - td["d"]
    upright = abs(pit) < 6.0 and abs(rol) < 6.0
    verdict = "pass" if (upright and c.jump_fault == "none"
                         and c.is_properly_standing(data)) else "fail"
    out = dict(speed_cmd=speed, package=package, spawn_d=round(spawn_d, 3),
               vx_trigger=round(vx_trigger, 3),
               vx_takeoff=round(takeoff["vx"], 3) if takeoff else None,
               vz_takeoff=round(takeoff["vz"], 3) if takeoff else None,
               apex_z=round(apex_z, 3),
               flight_s=round(td["t"] - takeoff["t"], 3)
               if (td and takeoff) else None,
               carry=round(carry, 3) if carry is not None else None,
               td_pitch=round(td["pitch"], 2) if td else None,
               td_roll=round(td["roll"], 2) if td else None,
               td_vx=round(td["vx"], 3) if td else None,
               settle_pitch=round(pit, 2), settle_roll=round(rol, 2),
               settle_z=round(z, 3), d_final=round(d_fin, 3),
               standing=bool(c.is_properly_standing(data)),
               diag_l0=round(max(kins["l"].l0, kins["r"].l0), 4),
               diag_phi=round(max(abs(np.degrees(kins["l"].phi0) - 90),
                                  abs(np.degrees(kins["r"].phi0) - 90)), 2),
               diag_grounded=[bool(grounded["l"]), bool(grounded["r"])],
               fault=c.jump_fault, verdict=verdict, reason=c.jump_fault)
    log(f"v={speed} {package}: {verdict} carry={out['carry']} "
        f"apex={out['apex_z']} td_pitch={out['td_pitch']} fault={c.jump_fault}")
    return out


def save(out: dict) -> None:
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    with open(RESULTS, "a") as f:
        f.write(json.dumps(out, default=str) + "\n")


def main() -> None:
    which = sys.argv[1] if len(sys.argv) > 1 else "ladder"
    package = "lever"
    for a in sys.argv[2:]:
        if a.startswith("package="):
            package = a.split("=")[1]
    model, _ = load_model("fast")
    c = TraditionalLQR(model)
    t0 = time.time()
    if which == "ladder":
        for v in (0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.5):
            out = run_cell(model, c, v, package,
                           log=lambda m: print("  ", m, flush=True))
            save(out)
            print(f"[ladder] v={v} -> {out['verdict']}", flush=True)
    elif which == "cell":
        kv = dict(a.split("=") for a in sys.argv[2:] if "=" in a)
        speed = float(kv.get("speed", 1.0))
        out = run_cell(model, c, speed, kv.get("package", "lever"),
                       log=lambda m: print("  ", m, flush=True))
        save(out)
        print("[cell]", json.dumps(out, default=str), flush=True)
    print(f"[jump_precision] done in {time.time()-t0:.0f}s wall", flush=True)


if __name__ == "__main__":
    main()
