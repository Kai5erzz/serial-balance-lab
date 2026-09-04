"""Step-up v2: adaptive-sign VMC walk/hop up the 200 mm tiers.

Every phi channel carries a self-calibrating sign (relay with progress
check): if the error stops shrinking the torque polarity flips once.  This
sidesteps the branch-dependent TP->phi polarity documented in
DESCENT_NOTES.md without needing a pre-built map.

Sequence per tier: LQR approach + wall stall -> compress -> extend (wheels
lift) -> sweep phi forward -> settle wheels on the tread -> pull up ->
level -> re-arm LQR.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import traditional_lqr as TL  # noqa: E402
from traditional_lqr import (  # noqa: E402
    CALIBRATED_STAND_POSE_PATH,
    apply_saved_pose,
)

MODEL = ROOT / "model" / "whole_body_lqr_fast_platform.xml"
HEADLESS = "--viewer" not in sys.argv


class AdaptiveLeg:
    """(l0, phi) servo whose phi torque sign self-calibrates."""

    def __init__(self, c, model):
        self.c = c
        self.dt = model.opt.timestep
        self.phi_sign = 1.0
        self.prev_err = None
        self.win_t = 0.0
        self.prev_phi = {}
        self.prev_l0 = {}

    def tick(self, data, l0_ref: float, phi_ref: float, wheel: float = 0.0):
        c = self.c
        dt = self.dt
        info = {}
        for side in ("l", "r"):
            kin = c.vmc_kinematics(data, side)
            if side not in self.prev_phi:
                self.prev_phi[side] = kin.phi0
                self.prev_l0[side] = kin.l0
            prate = (kin.phi0 - self.prev_phi[side]) / dt
            lrate = (kin.l0 - self.prev_l0[side]) / dt
            self.prev_phi[side] = kin.phi0
            self.prev_l0[side] = kin.l0
            err = phi_ref - kin.phi0
            force = float(np.clip(
                900.0 * (l0_ref - kin.l0) - 40.0 * lrate
                - float(np.clip(c._gas_virtual_support(kin.l0), 0, 260)),
                -260, 260))
            tp_raw = 45.0 * err - 2.5 * prate
            tp = self.phi_sign * float(np.clip(tp_raw, -30, 30))
            c.set_vmc_motor_torques(
                data, side, force, tp, torque_limit=24.0)
            ids = c.sides[side]
            data.ctrl[ids.wheel_motor] = wheel * c.wheel_sign[side]
            info[side] = (kin.l0, kin.phi0, force, tp)
        # relay polarity check on a 0.35 s cadence (uses the LEFT leg)
        err = phi_ref - info["l"][1]
        self.win_t += dt
        if self.win_t >= 0.35:
            # Flip whenever the error is not SHRINKING: a mechanical stop
            # pins the error constant, which the old grow-only check never
            # caught.
            if (
                abs(err) > 0.05
                and self.prev_err is not None
                and abs(err) > abs(self.prev_err) - 1e-3
            ):
                self.phi_sign *= -1.0
            self.prev_err = err
            self.win_t = 0.0
        return info


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
    c.disable_descent_guard = True
    c.disable_danger_check = True

    viewer = None
    if not HEADLESS:
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.cam.lookat[:] = (1.2, -1.2, 0.3)
        viewer.cam.distance = 4.2
        viewer.cam.elevation = -14.0
        viewer.cam.azimuth = 100.0
    leg = AdaptiveLeg(c, model)
    deadline = time.time() + 150.0

    def play(seconds, l0_fn, phi_fn, wheel_fn=lambda: 0.0):
        end = data.time + seconds
        while data.time < end and time.time() < deadline:
            c.step(data, model.opt.timestep)
            mujoco.mj_step(model, data)
            leg.tick(data, l0_fn(), phi_fn(), wheel_fn())
            if viewer is not None:
                viewer.sync()

    def rad(d):
        return np.radians(d)

    for tier, stall_d in enumerate((2.0, 1.85, 1.55)):
        print(f"=== TIER {tier + 1} ===")
        # W1 approach
        try:
            c.enable_calibrated_lqr(data)
            c.set_drive_command(0.30, 0.0)
        except ValueError as exc:
            print(f"  LQR re-arm rejected: {str(exc)[:80]}")
            c.enabled = False
            c.set_drive_command(0.25, 0.0)
        stalled = False
        while time.time() < deadline:
            c.step(data, model.opt.timestep)
            mujoco.mj_step(model, data)
            if viewer is not None:
                viewer.sync()
            d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
            if abs(c.chassis_vx) < 0.12 and d < stall_d + 0.2:
                stalled = True
                break
        print(f"  W1: dist={d:.2f} vx={c.chassis_vx:+.2f} stalled={stalled}")
        c.stop_drive()

        # W2 compress + extend with phi held at 90 (adaptive sign)
        c.enabled = False
        play(0.8, lambda: 0.14, lambda: rad(90))
        play(2.2, lambda: 0.333, lambda: rad(90))
        k = c.vmc_kinematics(data, "l")
        wz = float(data.xpos[c.sides["l"].wheel_body][2])
        print(f"  W2: z={data.xpos[c.base][2]:.3f} l0={k.l0:.3f} "
              f"phi={np.degrees(k.phi0):+.1f} wheel_z={wz:.3f}")

        # W3 sweep phi forward (adaptive sign handles polarity)
        play(2.0, lambda: 0.30, lambda: rad(40.0))
        k = c.vmc_kinematics(data, "l")
        wz = float(data.xpos[c.sides["l"].wheel_body][2])
        print(f"  W3: phi={np.degrees(k.phi0):+.1f} wheel_z={wz:.3f} "
              f"pitch={np.degrees(c.body_pitch(data)):+.1f} "
              f"z={data.xpos[c.base][2]:.3f}")

        # W4 settle wheels onto tread, then pull up with wheel drag
        play(1.5, lambda: 0.20, lambda: rad(45.0))
        play(2.5, lambda: 0.333, lambda: rad(50.0), wheel_fn=lambda: -0.6)
        k = c.vmc_kinematics(data, "l")
        print(f"  W4: z={data.xpos[c.base][2]:.3f} "
              f"pitch={np.degrees(c.body_pitch(data)):+.1f}")

        # W5 level + re-arm
        play(1.5, lambda: 0.15, lambda: rad(90.0))
        mounted = float(data.xpos[c.base][2]) >= 0.19 + 0.2 * (tier + 1) - 0.06
        print(f"  W5: z={data.xpos[c.base][2]:.3f} "
              f"pitch={np.degrees(c.body_pitch(data)):+.1f} "
              f"mounted={mounted}")
        if not mounted:
            print("  tier failed; stopping")
            break

    c.stop_drive()
    if viewer is not None:
        viewer.close()
    print("session ended")


if __name__ == "__main__":
    main()
