"""Pin the base in hover mode, apply common-mode wheel torque, and measure the
ground contact force on the chassis -> determines the plant direction."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import mujoco
import numpy as np

MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    free = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    qadr = model.jnt_qposadr[free]
    dadr = model.jnt_dofadr[free]
    wl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "l_wheel_motor")
    wr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_wheel_motor")
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    l_wheel_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "fast_collision_l_wheel")
    r_wheel_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "fast_collision_r_wheel")

    # settle on the ground with zero ctrl
    data.ctrl[:] = 0.0
    for _ in range(1000):
        mujoco.mj_step(model, data)
    # pin base at current pose
    pinned = data.qpos[qadr:qadr + 7].copy()
    print(f"base pinned at z={data.xpos[base, 2]:.3f}")

    def measure(torque: float) -> None:
        data.qpos[qadr:qadr + 7] = pinned
        data.qvel[dadr:dadr + 6] = 0.0
        mujoco.mj_forward(model, data)
        data.ctrl[wl] = -torque
        data.ctrl[wr] = +torque
        fsum = np.zeros(3)
        for _ in range(200):
            data.qpos[qadr:qadr + 7] = pinned
            data.qvel[dadr:dadr + 6] = 0.0
            mujoco.mj_step(model, data)
            for i in range(data.ncon):
                c = data.contact[i]
                if ground not in (c.geom1, c.geom2):
                    continue
                other = c.geom2 if c.geom1 == ground else c.geom1
                if other not in (l_wheel_geom, r_wheel_geom):
                    continue
                n = c.frame[:3]
                t1 = c.frame[3:6]
                t2 = c.frame[6:9]
                f = n * c.friction[0] + t1 * c.friction[1] + t2 * c.friction[2]
                fsum -= f  # reaction on the chassis
        data.ctrl[wl] = 0.0
        data.ctrl[wr] = 0.0
        avg = fsum / 200
        print(
            f"  torque={torque:+.2f} Nm  avg reaction (x,y,z)=({avg[0]:+.4f},{avg[1]:+.4f},{avg[2]:+.4f}) N "
            f"|horizontal|={np.hypot(avg[0], avg[1]):.4f} N  angle={np.degrees(np.arctan2(avg[1], avg[0])):+.1f} deg"
        )

    measure(1.0)
    measure(-1.0)


if __name__ == "__main__":
    main()
