"""Mode 7 headless demo: attempt to jump UP the three platform tiers.

EXPERIMENTAL.  The feasibility memo in DESCENT_NOTES.md shows a single hop
onto the 600 mm deck is impossible inside the actuator envelope (~3.5 m/s
takeoff needed vs ~2.06 m/s available), so this script attempts per-tier
hops of +200 mm each -- which sit right at the edge of today's jump
performance (need ~2.15 m/s, have ~2.06 m/s plus high-leg stroke help).

Run:  python mode7_stair_climb_demo.py [leg_length]
The robot spawns at deck-less ground beside the pyramid, jumps toward the
first tier, and every result is logged factually.
"""
from __future__ import annotations

import sys
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
LEG_LENGTH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.24


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)
    apply_saved_pose(model, data, Path(CALIBRATED_STAND_POSE_PATH))
    # Spawn on the surrounding ground outside the pyramid's lowest ring:
    # base sits at world origin; walk-in from +heading side, ~1.9 m out.
    data.qpos[2] += 0.001
    mujoco.mj_forward(model, data)
    # Place near the lowest tier wall along +heading with a clear run-up.
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    data.qpos[0] -= float(h[0]) * 2.2
    data.qpos[1] -= float(h[1]) * 2.2
    mujoco.mj_forward(model, data)
    c.enable_calibrated_lqr(data)
    if abs(LEG_LENGTH - 0.150) > 1e-6:
        try:
            c.set_leg_length(LEG_LENGTH)
        except ValueError as exc:
            raise SystemExit(str(exc))

    end = data.time + 3.0
    while data.time < end:            # settle, governor-free stand
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)

    print(f"mode7 leg_length={LEG_LENGTH:.3f} spawn z="
          f"{data.xpos[c.base, 2]:.3f}")

    hops = 0
    while hops < 4 and data.time < 20.0:
        ok, reason = c.begin_jump(data)
        if not ok:
            print(f"[hop {hops}] begin_jump rejected: {reason}")
            break
        print(f"[hop {hops}] accepted at z={data.xpos[c.base, 2]:.3f}")
        prev_state = c.jump_state
        apex_z = float(data.xpos[c.base, 2])
        while data.time < 6.5:
            c.step(data, model.opt.timestep)
            mujoco.mj_step(model, data)
            z = float(data.xpos[c.base, 2])
            apex_z = max(apex_z, z)
            pit = np.degrees(c.body_pitch(data))
            if abs(pit) > 29 or not np.isfinite(data.qpos).all():
                break
            if c.jump_state != prev_state:
                prev_state = c.jump_state
                print(f"    -> {c.jump_state} z={z:.3f}")
            if c.jump_state == "idle":
                break
        pitch_final = np.degrees(c.body_pitch(data))
        grounded = all(c._wheel_contacts(data).values())
        print(f"[hop {hops}] done: apex_z={apex_z:.3f} "
              f"final_z={data.xpos[c.base, 2]:.3f} "
              f"pitch={pitch_final:+.2f} grounded={grounded}")
        hops += 1
        # settle between hops
        end = data.time + 1.2
        while data.time < end and c.off_ground is False:
            c.step(data, model.opt.timestep)
            mujoco.mj_step(model, data)

    print("mode7 finished after", hops, "hop(s); "
          "see DESCENT_NOTES.md feasibility memo.")


if __name__ == "__main__":
    main()
