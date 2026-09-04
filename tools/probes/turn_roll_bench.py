"""Turn-roll benchmark: quantify chassis behaviour during commanded turns.

Metrics per case (evaluated over the last 40% of the drive window):
- mean signed roll and the physically required bank atan(a_xy/g): a clean
  steady turn banks toward the circle centre by that angle;
- mean chassis-frame lateral slip speed: a well-compensated turn has ~0;
- max |roll| after the command ramp completes (turn-entry transient).

Command ramp: linear 0.5 s slew, mimicking an RC stick push instead of an
instantaneous step.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import mujoco
import numpy as np

from balance_lab import (
    CALIBRATED_STAND_POSE_PATH,
    GRAVITY,
    MODEL_FAST,
    TraditionalLQR,
    apply_saved_pose,
)


def _unit(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def run_case(
    leg_length: float,
    forward: float,
    yaw_rate: float,
    settle_s: float = 2.5,
    drive_s: float = 9.0,
    ramp_s: float = 0.5,
    ff_enabled: bool = True,
) -> dict[str, float]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_FAST))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    if not ff_enabled:
        # A/B switch: hold the feedforward at zero without touching the rest
        # of the controller (the static trim keeps running in both arms).
        controller._update_roll_lateral_ff = lambda lengths, dt: setattr(
            controller, "lateral_inertial_ff", 0.0
        )
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

    run(settle_s)
    controller.set_leg_length(leg_length)
    run(2.0)

    up = np.array((0.0, 0.0, 1.0))
    roll_samples: list[float] = []
    slip_samples: list[float] = []
    max_roll_after_ramp = 0.0
    ramp_steps = int(ramp_s / dt)
    total_steps = int(drive_s / dt)
    for step in range(total_steps):
        fraction = min(1.0, (step + 1) / ramp_steps)
        controller.set_drive_command(forward * fraction, yaw_rate * fraction)
        controller.step(data, dt)
        mujoco.mj_step(model, data)
        roll_deg = float(np.degrees(controller.body_roll(data)))
        roll_samples.append(roll_deg)
        if step >= ramp_steps:
            max_roll_after_ramp = max(max_roll_after_ramp, abs(roll_deg))
        heading, _ = controller._heading(data)
        lateral = _unit(np.cross(up, heading))
        # base_free is the first joint: qvel[0:3] translational, [3:6] angular
        velocity = data.qvel[0:3]
        slip = float(np.dot(velocity, lateral))
        slip_samples.append(slip)

    tail = slice(int(0.6 * total_steps), None)
    steady_roll = float(np.mean(roll_samples[tail]))
    steady_slip = float(np.mean(slip_samples[tail]))
    lateral_accel = forward * yaw_rate
    bank_required_deg = float(np.degrees(np.arctan2(lateral_accel, GRAVITY)))
    return {
        "steady_roll_deg": steady_roll,
        "bank_required_deg": bank_required_deg,
        "bank_error_deg": steady_roll - bank_required_deg,
        "steady_slip_m_s": steady_slip,
        "max_roll_after_ramp_deg": max_roll_after_ramp,
    }


def main() -> None:
    header = (
        f"{'leg':>5} {'vx':>5} {'yaw':>5} {'ff':>3} | {'roll':>6} {'bank*':>6} "
        f"{'err':>6} {'slip':>7} {'maxAfter':>8}"
    )
    print(header + "\n" + "-" * len(header))
    cases = [
        (0.130, 0.5, 1.0),
        (0.130, 0.5, 2.0),
        (0.130, 0.5, -2.0),
        (0.130, 1.0, 2.0),
        (0.130, 1.0, -2.0),
        (0.130, 1.5, 2.0),
        (0.300, 1.0, 2.0),
    ]
    ff_enabled = "--no-ff" not in sys.argv
    for leg, vx, wz in cases:
        r = run_case(leg, vx, wz, ff_enabled=ff_enabled)
        print(
            f"{leg:5.3f} {vx:5.2f} {wz:5.2f} {'on' if ff_enabled else 'off':>3} | "
            f"{r['steady_roll_deg']:6.2f} {r['bank_required_deg']:6.2f} "
            f"{r['bank_error_deg']:6.2f} {r['steady_slip_m_s']:7.3f} "
            f"{r['max_roll_after_ramp_deg']:8.2f}"
        )


if __name__ == "__main__":
    main()
