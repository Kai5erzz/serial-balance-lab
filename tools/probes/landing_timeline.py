"""Post-landing timeline: quantify how long until the vehicle is 'normal' again.

Runs the stationary firmware jump, then from touchdown measures:
- landing-buffer phases until support hands back to the normal LQR path;
- lqr_gain ramp back to 1.0;
- the landing position-hold pull (jump_landing_position_x_ratio) and how
  long the chassis keeps creeping;
- drive-command response latency: a forward command applied at touchdown
  +0.5 s, timed until |vx| tracks within 20%;
- jump cooldown: when the next begin_jump() is accepted.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2] if __name__ != "__main__" else Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import mujoco
import numpy as np

from balance_lab import (
    CALIBRATED_STAND_POSE_PATH,
    MODEL_FAST,
    TraditionalLQR,
    apply_saved_pose,
)


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_FAST))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    dt = model.opt.timestep

    def run(seconds: float) -> None:
        for _ in range(int(seconds / dt)):
            controller.step(data, dt)
            mujoco.mj_step(model, data)

    run(4.0)  # settle at the LOW equilibrium (jump entry wait)
    accepted, reason = controller.begin_jump(data)
    print(f"jump accepted={accepted} ({reason}) at t={data.time:.3f}")

    touchdown_t = None
    idle_t = None
    rows: list[str] = []
    drive_cmd_t = None
    drive_track_t = None
    cooldown_ok_t = None
    last_log = -1.0
    while data.time < 14.0:
        controller.step(data, dt)
        mujoco.mj_step(model, data)
        t = data.time

        if controller.jump_state == "landing" and touchdown_t is None:
            touchdown_t = t
        if touchdown_t is not None and controller.jump_state == "idle" and idle_t is None:
            idle_t = t
        # apply a drive command 0.5 s after touchdown, time the response
        if touchdown_t is not None and drive_cmd_t is None and t >= touchdown_t + 0.5:
            controller.set_drive_command(0.5, 0.0)
            drive_cmd_t = t
        if (
            drive_cmd_t is not None
            and drive_track_t is None
            # signed criterion: |vx| alone false-triggers on the post-landing
            # backward slide
            and controller.chassis_vx >= 0.8 * 0.5
        ):
            drive_track_t = t
        # probe the cooldown once idle
        if idle_t is not None and cooldown_ok_t is None and t > idle_t + 0.2:
            ok, why = controller.begin_jump(data)
            if ok:
                cooldown_ok_t = t
                controller.cancel_recovery()
                # no actual second jump: immediately reset the machine
                controller._finish_jump("none")
                controller._reset_jump_runtime(clear_cooldown=False)
                controller.jump_state = "idle"

        if touchdown_t is not None and t - last_log >= 0.1:
            last_log = t
            rows.append(
                f"t={t - touchdown_t:5.2f} state={controller.jump_state:8s} "
                f"buf={controller.landing_buffer_phase:8s} gain={controller.lqr_gain:.2f} "
                f"pitch={float(np.degrees(controller.body_pitch(data))):+6.2f} "
                f"l0={0.5 * (controller.vmc_kinematics(data, 'l').l0 + controller.vmc_kinematics(data, 'r').l0):.3f} "
                f"vx={controller.chassis_vx:+5.2f} v_x={controller.lqr_virtual_x:+5.2f} "
                f"pos={controller.lqr_position_state}"
            )

    print("\n".join(rows[:40]))
    print("\n--- summary ---")
    print(f"touchdown at           t={touchdown_t:.3f}")
    print(f"state idle at          t={idle_t:.3f} (+{idle_t - touchdown_t:.3f} s)")
    print(f"drive cmd applied at   t={drive_cmd_t:.3f} (+{drive_cmd_t - touchdown_t:.3f} s)")
    print(f"drive tracked at       t={drive_track_t:.3f} "
          f"(latency {drive_track_t - drive_cmd_t:.3f} s)")
    print(f"next jump accepted at  t={cooldown_ok_t if cooldown_ok_t else float('nan'):.3f}")
    print(f"final virtual_x={controller.lqr_virtual_x:+.3f} position={controller.lqr_position_state}")


if __name__ == "__main__":
    main()
