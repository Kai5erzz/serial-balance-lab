"""Mode 7, take two: face the stairs, run up, and hop onto the FIRST tier.

Acceptance (user-specified): final state has both wheels resting on the
200 mm tread ("stair1"), chassis upright.

Spawn geometry (platform pyramid is yaw-45, tiers face the diagonals):
  ring faces lie at distance = half-extent from centre along the diagonal
  (1.9 outer/ground, 1.6 = 0.2 m tread, 1.3 = 0.4 m tread, 1.0 = top 0.6).
The robot spawns outside the pyramid on the diagonal, nose toward the
centre, drives in, and jumps when close enough to the first riser.

Run:            python mode7_climb_viewer.py            # live window
Headless check: python mode7_climb_viewer.py --headless
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from traditional_lqr import (  # noqa: E402
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)

MODEL = ROOT / "model" / "whole_body_lqr_fast_platform.xml"
HEADLESS = "--headless" in sys.argv
sys.argv = [a for a in sys.argv if a not in ("--headless",)]
LEG_LENGTH = 0.24
APPROACH_SPEED = float(sys.argv[-2]) if len(sys.argv) > 2 else 0.40
# jump trigger: distance from pyramid centre (wall of tier1 at 1.6 m)
JUMP_DIST = float(sys.argv[-1]) if len(sys.argv) > 2 else 2.02
HOPS = 3


def spawn_facing_stairs(model, data, c, distance: float) -> None:
    """Place the robot on the diagonal with its visible nose at the centre.

    The CAD export's forward_body axis points at the TAIL, so the nose is
    along -heading.  Spawning at +heading*distance makes -heading point at
    the pyramid.
    """
    apply_saved_pose(model, data, Path(CALIBRATED_STAND_POSE_PATH))
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    data.qpos[0] += float(h[0]) * distance
    data.qpos[1] += float(h[1]) * distance
    mujoco.mj_forward(model, data)


def wheel_support_level(c, data) -> tuple[float, str]:
    """Return (highest tread under wheels, geom name)."""
    best, name = 0.0, "ground"
    for i in range(data.ncon):
        con = data.contact[i]
        for g in (con.geom1, con.geom2):
            gn = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM, int(g))
            if gn in ("platform_step2",):
                best, name = max(best, 0.2), gn
            elif gn in ("platform_step1",):
                best, name = max(best, 0.4), gn
            elif gn in ("platform_top",):
                best, name = max(best, 0.6), gn
    return best, name



def suppress_descent_guard(c) -> None:
    """Climb mode: the ledge guard reads our uphill wall as a drop edge and
    brakes the run-up; clear its latched state every tick."""
    if c.descent_guard_active:
        c.descent_guard_active = False
        c.descent_guard_drop_seen = False
        c.descent_guard_preedge_elapsed = 0.0
        c.descent_edge_profile_active = False
        c.approach_brake_active = False


model = mujoco.MjModel.from_xml_path(str(MODEL))
data = mujoco.MjData(model)
c = TraditionalLQR(model)
spawn_facing_stairs(model, data, c, 2.6)
c.enable_calibrated_lqr(data)
c.disable_descent_guard = True
c.set_leg_length(LEG_LENGTH)

dist0 = float(np.hypot(data.qpos[0], data.qpos[1]))
print(f"spawn dist={dist0:.2f} nose->centre; approach @{APPROACH_SPEED} m/s")


def step_pair(viewer=None):
    c.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    suppress_descent_guard(c)
    if viewer is not None:
        viewer.sync()


if HEADLESS:
    # settle
    while data.time < 2.5:
        step_pair()
    c.set_drive_command(APPROACH_SPEED, 0.0)
    stalled = False
    while data.time < 12.0:
        step_pair()
        d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
        if abs(c.chassis_vx) < 0.08 and d < 2.05:
            stalled = True            # nose pressed against the riser
            break
        if d <= 1.82:
            stalled = True
            break
    print(f"at wall: dist={d:.2f} vx={c.chassis_vx:+.2f} stalled={stalled}")
    REDRIVE = APPROACH_SPEED
    ok, reason = c.begin_jump(data)
    print("jump:", ok, reason)
    if ok:
        while data.time < 15.0 and c.jump_state != "idle":
            if c.jump_state in ("preload", "thrust"):
                # build horizontal carry from the wall-stall: begin_jump()
                # cleared the command, re-drive until takeoff
                c.set_drive_command(REDRIVE, 0.0)
            step_pair()
        # creep forward to fully mount the tread (landing often straddles
        # the lip with one wheel still on the lower level)
        creep_end = data.time + 1.4
        while data.time < creep_end:
            c.set_drive_command(0.18, 0.0)
            step_pair()
        c.stop_drive()
        while data.time < 17.0:      # settle
            step_pair()
    def wheel_support_set():
        out = {"l": set(), "r": set()}
        for i in range(data.ncon):
            con = data.contact[i]
            for g in (con.geom1, con.geom2):
                gn = mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_GEOM, int(g))
                bid = int(model.geom_bodyid[int(g)])
                other = mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_GEOM,
                    int(con.geom2 if int(g) == con.geom1 else con.geom1))
                for side in ("l", "r"):
                    if bid == c.sides[side].wheel_body:
                        if gn:
                            out[side].add(gn)
                        if other:
                            out[side].add(other)
        return out

    pit = np.degrees(c.body_pitch(data))
    roll = np.degrees(c.body_roll(data))
    sup = wheel_support_set()
    both_up = all(
        "platform_step2" in sup[s] or "platform_step1" in sup[s]
        or "platform_top" in sup[s] for s in ("l", "r")
    )
    print(f"final: pitch={pit:+.2f} roll={roll:+.2f} "
          f"l_support={sorted(sup['l'])} r_support={sorted(sup['r'])}")
    verdict = (
        "PASS: both wheels on tier, upright"
        if (both_up and abs(pit) < 6.0 and abs(roll) < 6.0)
        else "not yet"
    )
    print(f"[tier check] {verdict}")

import mujoco.viewer  # noqa: E402

with mujoco.viewer.launch_passive(model, data) as v:
    v.cam.lookat[:] = (1.2, -1.2, 0.3)
    v.cam.distance = 4.2
    v.cam.elevation = -14.0
    v.cam.azimuth = 100.0
    deadline = time.time() + 240.0
    while data.time < 2.5 and time.time() < deadline:
        step_pair(v)
    c.set_drive_command(APPROACH_SPEED, 0.0)
    for hop in range(HOPS):
        # approach the next riser
        jumped = False
        while time.time() < deadline:
            step_pair(v)
            d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
            if d <= JUMP_DIST:
                ok, reason = c.begin_jump(data)
                if ok:
                    jumped = True
                    print(f"[hop {hop}] jump! dist={d:.2f}")
                    break
                if "cooldown" in reason:
                    time.sleep(0.3)
                    continue
                print(f"[hop {hop}] rejected: {reason}")
                break
        if not jumped:
            break
        while time.time() < deadline and c.jump_state != "idle":
            if c.jump_state in ("preload", "thrust"):
                c.set_drive_command(APPROACH_SPEED, 0.0)
            step_pair(v)
        c.stop_drive()
        lvl, gn = wheel_support_level(c, data)
        print(f"[hop {hop}] landed: support={gn} z="
              f"{data.xpos[c.base, 2]:.3f}")
        # creep forward a little to fully mount the tread, then re-arm
        hold = time.time() + 1.6
        while time.time() < hold:
            step_pair(v)
    c.stop_drive()
    print("viewer session ended")
