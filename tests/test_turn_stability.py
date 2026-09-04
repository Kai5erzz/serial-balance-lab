"""Turn-stability regression: low-leg high-yaw turns must survive and stay flat.

Locks in the 2026-09-05 night result (roll-compensation port + forward
cruise governor): before the fix, (1.0 m/s, +-2.0 rad/s) turns at the
0.130 m preset either diverged past 35 deg of lean or flipped outright
(179 deg) because the unregulated speed runaway tightened the turn radius
past the roll channel's authority.
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from balance_lab import (
    CALIBRATED_STAND_POSE_PATH,
    MODEL_FAST,
    TraditionalLQR,
    apply_saved_pose,
)


def run_turn(
    leg_length: float,
    forward: float,
    yaw_rate: float,
    drive_s: float = 9.0,
    ramp_s: float = 0.5,
) -> tuple[float, float, bool]:
    """Run one turn; return (max |roll| after ramp, mean |roll| tail, alive)."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_FAST))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    controller.set_leg_length(leg_length)
    dt = model.opt.timestep

    def run(seconds: float, fx: float = 0.0, wz: float = 0.0) -> None:
        if fx or wz:
            controller.set_drive_command(fx, wz)
        for _ in range(int(seconds / dt)):
            controller.step(data, dt)
            mujoco.mj_step(model, data)

    run(2.5)
    controller.set_leg_length(leg_length)
    run(2.0)

    up = np.array((0.0, 0.0, 1.0))
    roll_samples: list[float] = []
    max_after_ramp = 0.0
    ramp_steps = int(ramp_s / dt)
    total_steps = int(drive_s / dt)
    for step in range(total_steps):
        fraction = min(1.0, (step + 1) / ramp_steps)
        controller.set_drive_command(forward * fraction, yaw_rate * fraction)
        controller.step(data, dt)
        mujoco.mj_step(model, data)
        roll_deg = abs(float(np.degrees(controller.body_roll(data))))
        roll_samples.append(roll_deg)
        if step >= ramp_steps:
            max_after_ramp = max(max_after_ramp, roll_deg)
    tail = roll_samples[int(0.6 * total_steps):]
    alive = bool(np.all(np.isfinite(data.qpos)) and max(roll_samples) < 60.0)
    return max_after_ramp, float(np.mean(tail)), alive


CASES = (
    # (leg, vx, yaw, max_roll_after_ramp, steady_roll) -- thresholds carry
    # ~25% margin over the 2026-09-05 measured values.
    (0.130, 0.5, 2.0, 3.4, 1.2),
    (0.130, 0.5, -2.0, 1.2, 0.9),
    (0.130, 1.0, 2.0, 7.8, 3.0),
    (0.130, 1.0, -2.0, 6.2, 1.2),
    (0.300, 1.0, 2.0, 11.0, 4.6),
)


def main() -> None:
    for leg, vx, wz, max_limit, steady_limit in CASES:
        max_roll, steady_roll, alive = run_turn(leg, vx, wz)
        label = f"leg={leg:.3f} vx={vx:.1f} yaw={wz:+.1f}"
        assert alive, f"{label}: chassis diverged (roll >= 60 deg)"
        assert max_roll <= max_limit, (
            f"{label}: turn-entry |roll| {max_roll:.2f} deg > {max_limit:.1f}"
        )
        assert steady_roll <= steady_limit, (
            f"{label}: steady |roll| {steady_roll:.2f} deg > {steady_limit:.1f}"
        )
        print(f"PASS {label}: max {max_roll:.2f} deg, steady {steady_roll:.2f} deg")
    print("turn-stability regression: all cases passed")


if __name__ == "__main__":
    main()
