"""VMC step-up gait: climb the 200 mm tiers by PLACING the wheels.

Not ballistic.  Per tier:
  W1 approach  - LQR drive until the nose stalls against the riser
  W2 seat      - LQR off, compress legs, then extend so the chassis nose
                 rides up the riser face until it seats on the lip edge
  W3 place     - swing both wheels forward-down onto the upper tread
  W4 pull-up   - extend legs against the placed wheels; the chassis
                 pivots over the lip
  W5 level     - retract to the stand geometry, re-enable LQR

Run:  python stepup_gait.py [--viewer]
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT))
import traditional_lqr as TL  # noqa: E402
from traditional_lqr import (  # noqa: E402
    CALIBRATED_STAND_POSE_PATH,
    apply_saved_pose,
)

MODEL = ROOT / "model" / "whole_body_lqr_fast_platform.xml"
HEADLESS = "--viewer" not in sys.argv

L_COMPRESS = 0.13
L_LONG = 0.30
L_STAND = 0.15
KP_L, KD_L = 900.0, 40.0
KP_P, KD_P = 45.0, 2.5
TP_LIMIT = 24.0
F_LIMIT = 260.0


class LegPD:
    """Direct (l0, phi) PD through the VMC, gas-spring cancelled."""

    def __init__(self, c, model):
        self.c = c
        self.dt = model.opt.timestep
        self.phi = {"l": np.radians(90.0), "r": np.radians(90.0)}
        self.prev_phi = {"l": None, "r": None}

    def step(self, data, l0_ref: float, phi_ref: float,
             wheel_torque: float = 0.0) -> None:
        c = self.c
        for side in ("l", "r"):
            kin = c.vmc_kinematics(data, side)
            self.phi[side] = kin.phi0
            if self.prev_phi[side] is None:
                self.prev_phi[side] = kin.phi0
            phi_rate = (kin.phi0 - self.prev_phi[side]) / self.dt
            self.prev_phi[side] = kin.phi0
            l_rate = (kin.l0 - getattr(c, "_gait_prev_l0", {}).get(side, kin.l0)
                      ) / self.dt
            if not hasattr(c, "_gait_prev_l0"):
                c._gait_prev_l0 = {}
            c._gait_prev_l0.setdefault(side, kin.l0)
            l_rate = (kin.l0 - c._gait_prev_l0[side]) / self.dt
            c._gait_prev_l0[side] = kin.l0
            force = (
                KP_L * (l0_ref - kin.l0)
                - KD_L * l_rate
                - float(np.clip(c._gas_virtual_support(kin.l0), 0, 260))
            )
            force = float(np.clip(force, -F_LIMIT, F_LIMIT))
            tp = float(np.clip(
                KP_P * (phi_ref - kin.phi0) - KD_P * phi_rate,
                -TP_LIMIT, TP_LIMIT,
            ))
            c.recovery_output[side] = np.array((force, tp))
            c.last_support[side] = force
            c.last_output[side] = np.array((0.0, tp))
            c.set_vmc_motor_torques(
                data, side, force, tp, torque_limit=TP_LIMIT
            )
            ids = c.sides[side]
            data.ctrl[ids.wheel_motor] = wheel_torque * c.wheel_sign[side]


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    c = TL.TraditionalLQR(model)
    apply_saved_pose(model, data, Path(CALIBRATED_STAND_POSE_PATH))
    mujoco.mj_forward(model, data)
    c.reset(data, calibrate_frame=True)
    h, _ = c._heading(data)
    data.qpos[0] += float(h[0]) * 2.6
    data.qpos[1] += float(h[1]) * 2.6
    mujoco.mj_forward(model, data)
    c.enable_calibrated_lqr(data)
    # L_STAND 0.15 is not a preset; the approach drives at the default 0.13
    c.disable_descent_guard = True
    c.disable_danger_check = True

    def run(seconds: float, l0_ref: float, phi_ref: float,
            wheel: float = 0.0, viewer=None, tag: str = "") -> None:
        end = data.time + seconds
        while data.time < end:
            c.step(data, model.opt.timestep)
            mujoco.mj_step(model, data)
            pd.step(data, l0_ref, phi_ref, wheel)
            if viewer is not None:
                viewer.sync()

    def state() -> str:
        k = c.vmc_kinematics(data, "l")
        return (f"pitch={np.degrees(c.body_pitch(data)):+6.1f} "
                f"z={data.xpos[c.base][2]:.3f} l0={k.l0:.3f} "
                f"phi={np.degrees(k.phi0):+6.1f}")

    viewer = None
    if not HEADLESS:
        v = mujoco.viewer.launch_passive(model, data)
        viewer = v
        v.cam.lookat[:] = (1.2, -1.2, 0.3)
        v.cam.distance = 4.2
        v.cam.elevation = -14.0
        v.cam.azimuth = 100.0
    pd = LegPD(c, model)
    deadline = time.time() + 120.0

    for tier, wall_dist in enumerate((1.75, 1.45, 1.15)):
        if time.time() > deadline:
            break
        print(f"=== TIER {tier + 1} (wall at {wall_dist} m) ===")
        # W1 approach
        try:
            c.enable_calibrated_lqr(data)
        except ValueError as exc:
            print(f"  LQR re-arm rejected ({exc}); staying manual")
            c.enabled = False
        c.set_drive_command(0.30, 0.0)
        stalled = False
        while time.time() < deadline:
            c.step(data, model.opt.timestep)
            mujoco.mj_step(model, data)
            if viewer is not None:
                viewer.sync()
            d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
            if abs(c.chassis_vx) < 0.12 and d < wall_dist + 0.25:
                stalled = True
                break
            if d < wall_dist - 0.05:
                break
        print(f"  W1 approach: dist={d:.2f} vx={c.chassis_vx:+.2f} "
              f"stalled={stalled}")
        c.stop_drive()
        if not stalled:
            print("  wall not reached; aborting tier")
            continue

        # W2 seat: compress, then extend with the nose against the riser
        c.enabled = False
        c.set_drive_command(0.0, 0.0)
        run(0.8, L_COMPRESS, np.radians(90.0), viewer=viewer, tag="W2a")
        run(2.2, L_LONG, np.radians(90.0), viewer=viewer, tag="W2b")
        print(f"  W2 seat: {state()}")

        # W3 place: sweep wheels forward onto the tread.  Long legs keep
        # the four-bar away from the folded singularity where hip torque
        # stops producing swing.
        for phi_deg in np.arange(88.0, 20.0, -3.0):
            run(0.10, 0.28, np.radians(phi_deg), viewer=viewer)
        wheel_ok = True
        for side in ("l", "r"):
            wz = float(data.xpos[c.sides[side].wheel_body][2])
            if wz < 0.24:
                wheel_ok = False
        print(f"  W3 place: {state()} wheels_ok={wheel_ok}")

        # W4 pull up
        run(2.5, L_LONG, np.radians(40.0), viewer=viewer)
        print(f"  W4 pull-up: {state()}")

        # W5 level and re-arm
        run(1.2, L_STAND, np.radians(90.0), viewer=viewer)
        k = c.vmc_kinematics(data, "l")
        mounted = (
            float(data.xpos[c.base][2]) > 0.19 + 0.2 * (tier + 1) - 0.05
            and abs(np.degrees(c.body_pitch(data))) < 25.0
        )
        print(f"  W5 level: {state()} mounted={mounted}")

    c.stop_drive()
    if viewer is not None:
        viewer.close()
    print("gait session ended")


if __name__ == "__main__":
    main()
