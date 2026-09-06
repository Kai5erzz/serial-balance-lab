"""Stair-climb matrix runner (mode 7, night of 2026-09-07).

The 2026-09-05 cruise-governor fix invalidated every pre-tuned climb
window (the old recipes rode a ~2x approach-speed overshoot that no
longer happens).  This runner re-finds the tier-1 window on the
regulated plant, then chains three wall-hops to the top deck.

One wall-hop cell replicates the proven mode7_climb_viewer sequence:
approach -> stall against the riser -> begin_jump from the stall ->
re-drive through preload/thrust -> creep to seat the wheels -> settle.

Support classification distinguishes a wheel RESTING on a tread
(contact point within 35 mm of the geom's top face) from a wheel merely
touching the riser wall, which the older probes conflated.

Subcommands:
  cell      run a single parameterised hop, print one JSON verdict
  stage-a   sweep redrive/trigger/deploy for tier 1
  stage-b   posture sweep (tuck/deploy pitch biases, landing torque clamp)
  chain     three consecutive hops + 2 s stable-stand acceptance

Results append to artifacts/climb_matrix/results.jsonl.

Run:  python tools/probes/climb_matrix.py stage-a
"""
from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass, asdict, replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import mujoco
import numpy as np

import balance_lab.control.jump as _jump_mod
from balance_lab.plant import load_model
from balance_lab.paths import CALIBRATED_STAND_POSE_PATH
from balance_lab.plant.poses import apply_saved_pose
from balance_lab.control.controller import TraditionalLQR

MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast_platform.xml"
RESULTS = REPO_ROOT / "artifacts" / "climb_matrix" / "results.jsonl"

# tier riser face distance from centre along the diagonal (half-extents)
TIER_WALL = {1: 1.6, 2: 1.3, 3: 1.0}
TIER_TOP = {"ground_outer": 0.0, "platform_step3": 0.0,
            "platform_step2": 0.2, "platform_step1": 0.4, "platform_top": 0.6}


@dataclass
class Cell:
    tag: str = "cell"
    leg_length: float = 0.24
    approach: float = 0.30          # drive command toward the riser
    stall_jump: bool = True         # jump from the wall-stall
    trigger_d: float = 1.95         # used when stall_jump=False
    redrive: float = 0.30           # drive command during preload/thrust
    air_deploy: float = 0.27        # jump_air_landing_length override
    tuck_pitch_bias_deg: float = 0.0
    deploy_pitch_bias_deg: float = 0.0
    landing_wheel_bias: float = None  # clamp wheel torque in landing buffer
    creep: float = 0.18
    spawn_d: float = 2.6
    approach_leg: float = 0.0   # optional pre-crouched approach leg length
    flight_kick: float = 0.0    # logical wheel torque during flight/landing
    preload_min: float = 0.0    # override JUMP_PRELOAD_MIN_TIME (0 = keep)
    boost: float = 0.0          # override JUMP_THRUST_BOOST_FORCE (0 = keep)
    thrust_vxgain: float = -1.0 # override JUMP_THRUST_PITCH_RATE_VX_GAIN
    rate_target: float = 99.0   # instance jump_thrust_pitch_rate_target rad/s
    thrust_max: float = 0.0     # override JUMP_THRUST_MAX_TIME (0 = keep)
    tuck_hold: float = 0.0      # override jump_air_tuck_hold_time (0 = keep)
    deploy_rate: float = 0.0    # override jump_air_deploy_ref_rate (0 = keep)
    thrust_kick: float = 0.0    # logical wheel torque during preload/thrust
    damp_wheels: int = 1        # 0 = kill landing wheel pitch damper
    thrust_len: float = 0.0     # override JUMP_THRUST_LENGTH (0 = keep)
    preload_len: float = 0.0    # override JUMP_PRELOAD_LENGTH (0 = keep)
    theta_gain: float = -1.0    # override JUMP_THRUST_THETA_LQR_GAIN
    land_ref: float = 99.0      # landing pitch-ref shift: drive_forward poke
    deploy_kp: float = 0.0      # forward landing wheel damper gain
    land_len_kp: float = 0.0    # lock-legs: LANDING_LENGTH_KP override
    target_tier: int = 1


def spawn(model, data, c, distance: float = 2.6) -> None:
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    data.qpos[0] += float(h[0]) * distance
    data.qpos[1] += float(h[1]) * distance
    mujoco.mj_forward(model, data)


def suppress_descent_guard(c) -> None:
    if c.descent_guard_active:
        c.descent_guard_active = False
        c.descent_guard_drop_seen = False
        c.descent_guard_preedge_elapsed = 0.0
        c.descent_edge_profile_active = False
        c.approach_brake_active = False


def step_pair(model, data, c) -> None:
    c.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    suppress_descent_guard(c)


def base_diag(data, c) -> float:
    return float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))


def wheel_support(model, data, c) -> tuple[dict, dict]:
    """Per-wheel (tread level, wall contacts) with top-face filtering."""
    level = {"l": 0.0, "r": 0.0}
    wall = {"l": set(), "r": set()}
    for i in range(data.ncon):
        con = data.contact[i]
        g1, g2 = int(con.geom1), int(con.geom2)
        b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
        for side in ("l", "r"):
            bid = c.sides[side].wheel_body
            if bid not in (b1, b2):
                continue
            other = g2 if b1 == bid else g1
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other)
            top = TIER_TOP.get(name)
            if top is None:
                continue
            if abs(con.pos[2] - top) <= 0.035:
                level[side] = max(level[side], top)
            else:
                wall[side].add(name)
    return level, wall


def run_cell(model, c, cell: Cell, log=lambda m: None) -> dict:
    """One approach + wall-hop + mount cycle.  Returns a verdict dict."""
    data = mujoco.MjData(model)
    spawn(model, data, c, cell.spawn_d)
    c.enable_calibrated_lqr(data)
    c.disable_descent_guard = True
    c.disable_danger_check = True
    c.set_leg_length(cell.leg_length)
    c.jump_air_landing_length = cell.air_deploy
    c.jump_air_tuck_pitch_bias = np.radians(cell.tuck_pitch_bias_deg)
    c.jump_air_deploy_pitch_bias = np.radians(cell.deploy_pitch_bias_deg)
    if cell.rate_target < 90.0:
        c.jump_thrust_pitch_rate_target = cell.rate_target
    if cell.tuck_hold > 0:
        c.jump_air_tuck_hold_time = cell.tuck_hold
    if cell.damp_wheels == 0:
        c.jump_landing_wheel_pitch_kp = 0.0
        c.jump_landing_wheel_pitch_kd = 0.0
        c.jump_landing_capture_wheel_pitch_kp = 0.0
        c.jump_landing_capture_wheel_pitch_kd = 0.0
    if cell.thrust_len > 0:
        _jump_mod.JUMP_THRUST_LENGTH = cell.thrust_len
    if cell.preload_len > 0:
        _jump_mod.JUMP_PRELOAD_LENGTH = cell.preload_len
    if cell.theta_gain >= 0:
        _jump_mod.JUMP_THRUST_THETA_LQR_GAIN = cell.theta_gain
    if cell.land_len_kp > 0:
        _jump_mod.LANDING_LENGTH_KP = cell.land_len_kp
    if cell.deploy_rate > 0:
        c.jump_air_deploy_ref_rate = cell.deploy_rate
    import balance_lab.control.tuning as _tun
    _jump_mod.JUMP_PRELOAD_MIN_TIME = (
        cell.preload_min if cell.preload_min > 0
        else _tun.JUMP_PRELOAD_MIN_TIME)
    _jump_mod.JUMP_THRUST_BOOST_FORCE = (
        cell.boost if cell.boost > 0 else _tun.JUMP_THRUST_BOOST_FORCE)
    _jump_mod.JUMP_THRUST_PITCH_RATE_VX_GAIN = (
        cell.thrust_vxgain if cell.thrust_vxgain >= 0
        else _tun.JUMP_THRUST_PITCH_RATE_VX_GAIN)
    _jump_mod.JUMP_THRUST_MAX_TIME = (
        cell.thrust_max if cell.thrust_max > 0
        else _tun.JUMP_THRUST_MAX_TIME)

    while data.time < 2.5:
        step_pair(model, data, c)
    if cell.approach_leg > 0.0:
        c.set_leg_length(cell.approach_leg)
        pre_end = data.time + 1.2
        while data.time < pre_end:
            step_pair(model, data, c)

    wall = TIER_WALL[cell.target_tier]
    c.set_drive_command(cell.approach, 0.0)
    stalled, d = False, base_diag(data, c)
    deadline = data.time + 10.0
    while data.time < deadline:
        step_pair(model, data, c)
        d = base_diag(data, c)
        if cell.stall_jump:
            if (abs(c.chassis_vx) < 0.08 and d < 2.05) or d <= 1.82:
                stalled = True
                break
        elif d <= cell.trigger_d:
            break
    log(f"approach d={d:.3f} vx={c.chassis_vx:+.3f} stalled={stalled}")

    ok, reason = c.begin_jump(data)
    if not ok:
        out = dict(asdict(cell), verdict="rejected", reason=reason,
                   d_trigger=d, pitch=None)
        log(f"rejected: {reason}")
        return out

    while data.time < deadline + 6.0 and c.jump_state != "idle":
        if c.jump_state in ("preload", "thrust"):
            c.set_drive_command(cell.redrive, 0.0)
        elif cell.flight_kick:
            # keep a forward wheel torque through flight/landing so the
            # touchdown rolls OVER the lip corner instead of bouncing off it
            c.set_drive_command(1.0, 0.0)
        c.step(data, model.opt.timestep)
        # post-controller injection: c.step rewrites ctrl, so anything we
        # want in the physics must be written here, after it
        if (c.jump_state == "thrust" and not c.jump_off_ground
                and cell.thrust_kick):
            for side in ("l", "r"):
                wm = c.sides[side].wheel_motor
                data.ctrl[wm] = c.wheel_sign[side] * cell.thrust_kick
        elif c.jump_state == "landing" and cell.flight_kick:
            for side in ("l", "r"):
                wm = c.sides[side].wheel_motor
                data.ctrl[wm] = c.wheel_sign[side] * cell.flight_kick
        elif (c.jump_state == "landing" and cell.land_ref < 90.0):
            # reimplement the landing wheel damper with a nose-down
            # reference so its whole budget drives the wheels FORWARD
            # (the stock ref is clipped to 0 and goes REVERSE on a
            # nose-down strike -- the measured +1.25 -> -0.12 kill)
            pitch = c.body_pitch(data)
            rate = -float(c.last_state["l"][5])
            ref = np.radians(cell.land_ref)
            torque = float(np.clip(
                cell.deploy_kp * (pitch - ref),
                -3.69, 3.69))
            for side in ("l", "r"):
                wm = c.sides[side].wheel_motor
                data.ctrl[wm] = c.wheel_sign[side] * torque
        elif (c.landing_buffer_phase != "inactive"
                and cell.landing_wheel_bias is not None):
            for side in ("l", "r"):
                wm = c.sides[side].wheel_motor
                sign = c.wheel_sign[side]
                if sign * data.ctrl[wm] > cell.landing_wheel_bias:
                    data.ctrl[wm] = sign * cell.landing_wheel_bias
        mujoco.mj_step(model, data)
        suppress_descent_guard(c)
    c.stop_drive()
    fault = c.jump_fault
    log(f"jump done fault={fault} d={base_diag(data, c):.3f}")

    # short grace, then fail fast if we clearly fell back to the ground
    grace_end = data.time + 0.8
    while data.time < grace_end:
        step_pair(model, data, c)
    level, walls = wheel_support(model, data, c)
    d_post = base_diag(data, c)
    if max(level.values()) == 0.0 and d_post > 1.9:
        pit = np.degrees(c.body_pitch(data))
        out = dict(asdict(cell), verdict="ground", reason=fault,
                   d_trigger=d, d_final=d_post, level_l=level["l"],
                   level_r=level["r"], pitch=pit)
        log(f"early-exit ground d={d_post:.3f} lvl={level}")
        return out

    creep_end = data.time + 1.4
    while data.time < creep_end:
        c.set_drive_command(cell.creep, 0.0)
        step_pair(model, data, c)
    c.stop_drive()
    settle_end = data.time + 1.2
    while data.time < settle_end:
        step_pair(model, data, c)

    level, walls = wheel_support(model, data, c)
    pit = np.degrees(c.body_pitch(data))
    rol = np.degrees(c.body_roll(data))
    z = float(data.xpos[c.base][2])
    d_fin = base_diag(data, c)
    mounted = min(level.values()) >= 0.2 * cell.target_tier and \
        min(level.values()) > 0.0
    upright = abs(pit) < 6.0 and abs(rol) < 6.0
    verdict = "mount" if (mounted and upright and fault == "none") else (
        "straddle" if mounted else "fail")
    out = dict(asdict(cell), verdict=verdict, reason=fault, d_trigger=d,
               d_final=d_fin, level_l=level["l"], level_r=level["r"],
               wall_l=sorted(walls["l"]), wall_r=sorted(walls["r"]),
               pitch=pit, roll=rol, z=z)
    log(f"verdict={verdict} lvl={level} d={d_fin:.3f} p={pit:+.2f} z={z:.3f}")
    return out


def save(out: dict) -> None:
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    with open(RESULTS, "a") as f:
        f.write(json.dumps(out, default=str) + "\n")


def stage_a(model, c) -> None:
    """Does higher preload/thrust carry restore the wall-stall mount?"""
    base = Cell(tag="a-redrive", leg_length=0.24, approach=0.30)
    for red in (0.30, 0.45, 0.60, 0.75):
        out = run_cell(model, c, replace(base, redrive=red),
                       log=lambda m: print("   ", m, flush=True))
        save(out)
        print(f"[stage-a] redrive={red} -> {out['verdict']}", flush=True)


def main() -> None:
    which = sys.argv[1] if len(sys.argv) > 1 else "stage-a"
    if "--c30" in sys.argv:
        sys.argv.remove("--c30")
        model = mujoco.MjModel.from_xml_path(
            str(REPO_ROOT / "model" / "whole_body_lqr_fast_platform_c30.xml"))
    elif "--tire" in sys.argv:
        sys.argv.remove("--tire")
        model = mujoco.MjModel.from_xml_path(
            str(REPO_ROOT / "model" / "whole_body_lqr_fast_platform_tire.xml"))
    elif "--full" in sys.argv:
        sys.argv.remove("--full")
        model = mujoco.MjModel.from_xml_path(
            str(REPO_ROOT / "model" / "whole_body_lqr_platform.xml"))
    else:
        model, _ = load_model("platform")
    c = TraditionalLQR(model)
    t0 = time.time()
    if which == "stage-a":
        stage_a(model, c)
    elif which == "cell":
        kv = dict(a.split("=") for a in sys.argv[2:])
        cast = {}
        for k, v in kv.items():
            f = getattr(Cell.__dataclass_fields__[k], "type", None)
            try:
                cast[k] = json.loads(v) if v in ("true", "false") else (
                    float(v) if k not in ("tag",) else v)
            except (ValueError, json.JSONDecodeError):
                cast[k] = v
        out = run_cell(model, c, Cell(**cast),
                       log=lambda m: print("   ", m, flush=True))
        save(out)
        print("[cell]", json.dumps(out, default=str), flush=True)
    print(f"[climb_matrix] {which} done in {time.time()-t0:.0f}s wall",
          flush=True)


if __name__ == "__main__":
    main()
